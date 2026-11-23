//! Traffic population management (milestone 4): the component that decides
//! which cars exist, and when.
//!
//! It decides but does not act. A component cannot build another component:
//! `StepCtx` offers no way back to the conductor, and a `Box<dyn Component>`
//! could not cross a transport even if it did. So the spawner publishes
//! *requests*, and whoever is driving the run turns them into
//! `add_component` and `remove_component` calls.
//!
//! That split is the point rather than a workaround. Deciding inside the
//! sim keeps the traffic pattern inside the determinism guarantee: the
//! choices come from poses and a seeded stream, so the same seed produces
//! the same cars at the same sim instants, and a recorded run can be
//! verified. A driver that picked spawn times itself would put the pattern
//! outside what the log can check.

use std::collections::BTreeMap;
use std::sync::Arc;

use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, Pose, RandomSplitMix64, SimDuration, SimTime,
    StepCtx,
};
use serde::{Deserialize, Serialize};

use crate::actor_of_pose_key;

/// A request to put one traffic car on the road.
///
/// Deliberately specific to this scenario: lanes and speeds are what a
/// freeway demo needs, and the framework never sees this type. The driver
/// decodes it and hands the conductor an ordinary component.
// TODO(PLAN "Scenario configuration"): the general form of this is a
// request naming a component *type* plus opaque parameters, resolved by a
// host-side registry of constructors. Then one request type serves every
// scenario, and at milestone 7 a remote host instantiates from it. This is
// that idea specialised to a single kind of car.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpawnTrafficRequest {
    /// Actor name for the new car, unique for the run.
    pub actor_name: String,
    /// Lateral offset of the lane to drive, in meters.
    pub lane_offset: f64,
    /// Constant speed to hold, m/s.
    pub speed: f64,
    /// Where to appear, as an arc length along the road the spawner
    /// measures against, not a coordinate, so it survives the road
    /// bending.
    pub start_s: f64,
    /// The instant the car should first step. Declared here, so the run is
    /// the same whenever the driver gets round to processing the request,
    /// as long as it lands before this arrives.
    pub first_due: SimTime,
}

/// A request to take one traffic car off the road, by actor name. The
/// driver removes the whole actor, so its controller and physics go
/// together.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DespawnTrafficRequest {
    /// Actor name of the car to remove.
    pub actor_name: String,
}

/// A new speed for one traffic car to want whenever the road ahead is free.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpeedTarget {
    /// Speed to want, m/s.
    pub speed: f64,
}

/// Key the spawner publishes a car's [`SpeedTarget`]s on.
pub fn speed_target_key(world_name: &str, actor_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/actor/{actor_name}/speed_tgt"))
        .expect("valid speed target key")
}

/// Key the spawner publishes [`SpawnTrafficRequest`]s on.
pub fn traffic_spawn_key(world_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/traffic_spawner/spawn")).expect("valid spawn key")
}

/// Key the spawner publishes [`DespawnTrafficRequest`]s on.
pub fn traffic_despawn_key(world_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/traffic_spawner/despawn")).expect("valid despawn key")
}

/// Keeps the road around the ego populated, retiring cars that get too far
/// behind or ahead of it, and watching poses to know where everyone is.
pub struct TrafficSpawner {
    /// How often the road is reviewed. A new car appears one period after
    /// the spawner places it, and the car behind it in its lane drives on
    /// in the meantime, so a longer period eats into the gap between them.
    period: SimDuration,
    /// The road everyone is measured along. Poses are projected onto it,
    /// so every distance here is an arc length on one shared reference
    /// path rather than a raw coordinate, which is what lets cars in
    /// different lanes be compared, and what keeps this working if the
    /// road ever bends.
    road: Arc<crate::Waypoints>,
    /// Actor whose progress the traffic is arranged around.
    ego_name: String,
    /// Lateral offset of the lane the ego starts in. The cars a run starts
    /// with keep their distance from the ego in that lane.
    ego_start_lateral: f64,
    /// Lateral offsets of the lanes to spawn into, in meters.
    lane_offsets: Vec<f64>,
    /// How near a car's lateral offset must be to a lane's for the car to
    /// count as in that lane, in meters.
    lane_tolerance: f64,
    /// How many cars to keep on the road at once.
    target_population: usize,
    /// Meters ahead of the ego that every new car spawns beyond.
    spawn_ahead: f64,
    /// Meters behind the ego at which a car is retired.
    retire_behind: f64,
    /// Meters ahead of the ego at which a car is retired. No car spawns
    /// beyond it.
    retire_ahead: f64,
    /// How far beyond the furthest car in its lane a new car spawns, as a
    /// range in meters.
    gap_range: (f64, f64),
    /// Speed range for a new car, in m/s.
    speed_range: (f64, f64),
    /// Mean seconds between new speed targets for each car.
    retarget_interval: f64,
    /// The cars this spawner has spawned and not yet asked to remove, by
    /// actor name, with where each was last seen. A car leaves as soon as
    /// its removal is requested, so the spawner does not ask twice while
    /// the first request is in flight.
    live_traffic: BTreeMap<String, LiveCar>,
    /// Latest known position of the ego, the reference every decision here
    /// is measured against. Where it starts until its first pose arrives.
    ego_s: f64,
    /// How many cars have been created, ever, which is what makes their
    /// names unique. Never decremented, so a retired `traffic3` never
    /// returns as `traffic3`. Reoccupying a path is legal but arrives as a
    /// fresh sibling, and there is nothing to gain here by inviting it.
    cars_spawned: u64,
}

impl TrafficSpawner {
    /// # Panics
    ///
    /// If a new car could spawn beyond `retire_ahead`, that is if
    /// `retire_ahead` is less than `spawn_ahead` plus the largest gap. The
    /// spawner would never place a car there, so the road would stay empty.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        period: SimDuration,
        road: Arc<crate::Waypoints>,
        ego_name: impl Into<String>,
        ego_start_s: f64,
        ego_start_lateral: f64,
        lane_offsets: Vec<f64>,
        lane_tolerance: f64,
        target_population: usize,
        spawn_ahead: f64,
        retire_behind: f64,
        retire_ahead: f64,
        gap_range: (f64, f64),
        speed_range: (f64, f64),
        retarget_interval: f64,
    ) -> Self {
        assert!(
            retire_ahead >= spawn_ahead + gap_range.1,
            "retire_ahead {retire_ahead} m is within the {} m a car can spawn at",
            spawn_ahead + gap_range.1,
        );

        // Return a spawner with an empty road; the first step fills it.
        TrafficSpawner {
            period,
            road,
            ego_name: ego_name.into(),
            ego_start_lateral,
            lane_offsets,
            lane_tolerance,
            target_population,
            spawn_ahead,
            retire_behind,
            retire_ahead,
            gap_range,
            speed_range,
            retarget_interval,
            live_traffic: BTreeMap::new(),
            ego_s: ego_start_s,
            cars_spawned: 0,
        }
    }
}

impl TrafficSpawner {
    /// The cars a run starts with, due at sim time zero and tracked as cars
    /// the spawner spawned. Cars take the lanes in turn, each a gap beyond
    /// the last car placed in its lane, starting from `retire_behind` behind
    /// the ego. A car that would land beyond `retire_ahead` is left out, and
    /// so is one within the smallest gap of the ego in the ego's lane, where
    /// the next car then goes a gap beyond the ego.
    pub fn fill_around_ego(&mut self, random: &mut RandomSplitMix64) -> Vec<SpawnTrafficRequest> {
        let mut lane_fronts = vec![self.ego_s - self.retire_behind; self.lane_offsets.len()];
        let mut spawns = Vec::with_capacity(self.target_population);
        for slot in 0..self.target_population {
            let lane = slot % lane_fronts.len();
            let start_s = lane_fronts[lane] + random.range_f64(self.gap_range.0, self.gap_range.1);
            let in_ego_lane =
                (self.lane_offsets[lane] - self.ego_start_lateral).abs() <= self.lane_tolerance;
            if in_ego_lane && (start_s - self.ego_s).abs() < self.gap_range.0 {
                lane_fronts[lane] = self.ego_s;
                continue;
            }
            if start_s > self.ego_s + self.retire_ahead {
                continue;
            }
            lane_fronts[lane] = start_s;
            let speed = random.range_f64(self.speed_range.0, self.speed_range.1);
            spawns.push(self.track_new_car(self.lane_offsets[lane], start_s, speed, SimTime::ZERO));
        }

        // Return the requests for the driver to add before the run starts.
        spawns
    }

    /// Names one new car, adds it to the cars tracked, and returns the
    /// request for it.
    fn track_new_car(
        &mut self,
        lane_offset: f64,
        start_s: f64,
        speed: f64,
        first_due: SimTime,
    ) -> SpawnTrafficRequest {
        self.cars_spawned += 1;
        let spawn = SpawnTrafficRequest {
            actor_name: format!("traffic{}", self.cars_spawned),
            lane_offset,
            speed,
            start_s,
            first_due,
        };
        self.live_traffic.insert(
            spawn.actor_name.clone(),
            LiveCar {
                s: start_s,
                lateral: lane_offset,
            },
        );

        // Return the request for the caller to publish or apply.
        spawn
    }
}

/// Where the spawner last saw one of the traffic cars it spawned.
#[derive(Serialize)]
struct LiveCar {
    /// Arc length along the road.
    s: f64,
    /// Lateral offset from the road, positive to the left, which says what
    /// lane the car is in.
    lateral: f64,
}

/// The front of the lane at `lane_offset`: the furthest of `cars` in that
/// lane, or `spawn_ahead` ahead of the ego at `ego_s` if that is further.
fn lane_front<'a>(
    cars: impl IntoIterator<Item = &'a LiveCar>,
    lane_offset: f64,
    lane_tolerance: f64,
    ego_s: f64,
    spawn_ahead: f64,
) -> f64 {
    let ahead_of_ego = ego_s + spawn_ahead;
    let furthest_in_lane = cars
        .into_iter()
        .filter(|car| (car.lateral - lane_offset).abs() <= lane_tolerance)
        .map(|car| car.s)
        .max_by(f64::total_cmp);

    // Return the further of the two, or the point ahead of the ego when
    // the lane is empty.
    match furthest_in_lane {
        Some(car_s) => car_s.max(ahead_of_ego),
        None => ahead_of_ego,
    }
}

impl Component for TrafficSpawner {
    fn id(&self) -> ComponentId {
        ComponentId::new("traffic_spawner").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        // Every actor's pose, the ego's included: the spawner's decisions
        // are all relative positions along the road.
        vec![KeyExpr::new_rooted("*/actor/**/pose").expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        for message in ctx.inbox() {
            // A pose that cannot be read stops the world: a spawner that never
            // sees a car move never retires it.
            let pose = message.decode::<Pose>()?;
            let actor_name = actor_of_pose_key(message.key.as_str());
            // Project onto the reference road rather than reading a
            // coordinate: cars sit in different lanes, so only a shared
            // arc length makes "ahead" and "behind" mean the same thing
            // for all of them, whatever shape the road is. The lateral
            // offset says which lane a car is in.
            let (actor_s, actor_lateral) = self.road.frenet(pose.position.x, pose.position.y);
            if actor_name == self.ego_name {
                self.ego_s = actor_s;
            } else if let Some(car) = self.live_traffic.get_mut(actor_name) {
                car.s = actor_s;
                car.lateral = actor_lateral;
            }
        }

        // Retire the cars the ego has gone past. Each leaves `live_traffic`
        // as soon as its removal is requested, so the spawner does not ask
        // twice while the request is in flight.
        let passed: Vec<String> = self
            .live_traffic
            .iter()
            .filter(|&(_, car)| {
                car.s < self.ego_s - self.retire_behind || car.s > self.ego_s + self.retire_ahead
            })
            .map(|(actor_name, _)| actor_name.clone())
            .collect();
        for actor_name in passed {
            self.live_traffic.remove(&actor_name);
            ctx.publish(
                traffic_despawn_key(ctx.world_name()),
                &DespawnTrafficRequest { actor_name },
            )?;
        }

        // Top the road back up. Each new car spawns a gap beyond the
        // furthest car in its lane, and always more than `spawn_ahead`
        // ahead of the ego.
        let mut random = ctx.step_random();
        while self.live_traffic.len() < self.target_population {
            let gap = random.range_f64(self.gap_range.0, self.gap_range.1);
            let lane_offset =
                self.lane_offsets[random.next_u64() as usize % self.lane_offsets.len()];
            let start_s = lane_front(
                self.live_traffic.values(),
                lane_offset,
                self.lane_tolerance,
                self.ego_s,
                self.spawn_ahead,
            ) + gap;
            if start_s > self.ego_s + self.retire_ahead {
                // The lane has no room left this side of `retire_ahead`, so
                // spawning waits for the next step.
                break;
            }
            let speed = random.range_f64(self.speed_range.0, self.speed_range.1);
            let spawn = self.track_new_car(lane_offset, start_s, speed, ctx.now() + self.period);
            ctx.publish(traffic_spawn_key(ctx.world_name()), &spawn)?;
        }

        if self.retarget_interval > 0.0 {
            let chance = self.period.as_secs_f64() / self.retarget_interval;
            let names: Vec<String> = self.live_traffic.keys().cloned().collect();
            for actor_name in names {
                if random.next_f64() < chance {
                    let target = SpeedTarget {
                        speed: random.range_f64(self.speed_range.0, self.speed_range.1),
                    };
                    ctx.publish(speed_target_key(ctx.world_name(), &actor_name), &target)?;
                }
            }
        }

        // Return the next due time, one period out.
        Ok(ctx.now() + self.period)
    }

    fn state_bytes(&self) -> Option<Vec<u8>> {
        // The cars it tracks decide every future request, and none of that
        // is published, so state-hash mode is what catches a divergence
        // here at the tick it happens.
        Some(
            serde_json::to_vec(&(&self.live_traffic, self.ego_s, self.cars_spawned))
                .expect("spawner state serializes"),
        )
    }
}

/// The world-frame pose at Frenet `(s, lateral)` on `road`: `s` meters
/// along it, displaced `lateral` to the left, facing the way the road runs
/// there.
///
/// What places a car at the position a spawn request asks for, since a
/// request names a lane offset and an arc length rather than coordinates.
pub fn road_pose(road: &crate::Waypoints, s: f64, lateral: f64) -> Pose {
    // Return the Frenet point in world coordinates, facing along the road.
    Pose {
        position: road.point_at_offset(s, lateral),
        orientation: continuo_core::Quat::from_yaw(road.heading_at(s)),
    }
}

/// A straight road of `length` meters along +x from the origin, the demo's
/// map.
///
/// Open rather than closed, which is what makes "ahead" and "behind" mean
/// something absolute: arc lengths only ever increase, so a spawner can
/// compare two cars and a retirement is final. The cost is that the road
/// runs out: past the end a car's arc length clamps and it stops making
/// progress, so `length` has to outlast the run it is built for.
// TODO(PLAN "World and map"): road geometry belongs in the world spec's
// scene graph, referenced by name rather than built here.
pub fn straight_road(length: f64) -> crate::Waypoints {
    // Return the one-segment path; every lane is a lateral offset on it.
    crate::Waypoints::build_straight((0.0, 0.0), (length, 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    const LANE: f64 = 0.0;
    const NEXT_LANE: f64 = 3.5;
    const LANE_TOLERANCE: f64 = 1.75;
    const EGO_S: f64 = 100.0;
    const SPAWN_AHEAD: f64 = 40.0;
    const AHEAD_OF_EGO: f64 = EGO_S + SPAWN_AHEAD;

    #[test]
    fn a_lane_front_is_the_furthest_car_seen_in_the_lane() {
        let furthest = AHEAD_OF_EGO + 10.0;
        let cars = [
            LiveCar {
                s: furthest,
                lateral: LANE,
            },
            LiveCar {
                s: furthest - 5.0,
                lateral: LANE,
            },
        ];
        assert_eq!(
            lane_front(&cars, LANE, LANE_TOLERANCE, EGO_S, SPAWN_AHEAD),
            furthest
        );
    }

    #[test]
    fn a_lane_front_is_never_nearer_the_ego_than_spawn_ahead() {
        let cars = [LiveCar {
            s: AHEAD_OF_EGO - 10.0,
            lateral: LANE,
        }];
        assert_eq!(
            lane_front(&cars, LANE, LANE_TOLERANCE, EGO_S, SPAWN_AHEAD),
            AHEAD_OF_EGO
        );
    }

    #[test]
    fn a_car_seen_in_another_lane_does_not_move_the_lane_front() {
        let cars = [LiveCar {
            s: AHEAD_OF_EGO + 10.0,
            lateral: NEXT_LANE,
        }];
        assert_eq!(
            lane_front(&cars, LANE, LANE_TOLERANCE, EGO_S, SPAWN_AHEAD),
            AHEAD_OF_EGO
        );
    }
}
