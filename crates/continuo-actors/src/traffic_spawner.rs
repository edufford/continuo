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
    Component, ComponentId, ComponentPath, CoreError, KeyExpr, Pose, RandomSplitMix64, SimDuration,
    SimTime, StepCtx, Subscription, derive_component_seed,
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

/// Key the spawner publishes [`SpawnTrafficRequest`]s on.
pub fn traffic_spawn_key(world_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/traffic_spawner/spawn")).expect("valid spawn key")
}

/// Key the spawner publishes [`DespawnTrafficRequest`]s on.
pub fn traffic_despawn_key(world_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/traffic_spawner/despawn")).expect("valid despawn key")
}

/// How a [`TrafficSpawner`] arranges traffic around the ego.
pub struct TrafficSpawnerConfig {
    /// How often the road is reviewed. A new car appears one period after
    /// the spawner places it, and the car behind it in its lane drives on
    /// in the meantime, so a longer period eats into the gap between them.
    pub period: SimDuration,
    /// Actor whose progress the traffic is arranged around.
    pub ego_name: String,
    /// Arc length along the road the ego starts at.
    pub ego_start_s: f64,
    /// Lateral offset of the lane the ego starts in. The cars a run starts
    /// with keep their distance from the ego in that lane.
    pub ego_start_lateral: f64,
    /// Lateral offsets of the lanes to spawn into, in meters.
    pub lane_offsets: Vec<f64>,
    /// How near a car's lateral offset must be to a lane's for the car to
    /// count as in that lane, in meters.
    pub lane_tolerance: f64,
    /// How many cars to keep on the road at once.
    pub target_population: usize,
    /// Meters ahead of the ego that every new car spawns beyond.
    pub spawn_ahead: f64,
    /// Meters behind the ego at which a car is retired.
    pub retire_behind: f64,
    /// Meters ahead of the ego at which a car is retired. No car spawns
    /// beyond it.
    pub retire_ahead: f64,
    /// How far beyond the furthest car in its lane a new car spawns, as a
    /// range in meters.
    pub gap_range: (f64, f64),
    /// Stretches the starting gaps ahead of the ego in its own lane, so it
    /// passes cars beside it before it reaches the car ahead.
    pub ego_headway_factor: f64,
    /// Speed range for a new car, in m/s.
    pub speed_range: (f64, f64),
}

/// Keeps the road around the ego populated, retiring cars that get too far
/// behind or ahead of it, and watching poses to know where everyone is.
pub struct TrafficSpawner {
    /// The road every pose is projected onto, so cars in any lane, on a
    /// straight or curved road, are compared by distance along it.
    road: Arc<crate::Waypoints>,
    /// What the spawner was built with.
    config: TrafficSpawnerConfig,
    /// The cars this spawner has spawned and not yet asked to remove, by
    /// actor name, with where each was last seen. A car leaves as soon as
    /// its removal is requested, so the spawner does not ask twice while
    /// the first request is in flight.
    live_traffic: BTreeMap<String, LiveCar>,
    /// The ego's arc length along the road, from its latest pose, or its
    /// start before the first pose arrives.
    ego_s: f64,
    /// How many cars have been created, ever, which is what makes their
    /// names unique. Never decremented, so a retired `traffic3` never
    /// returns as `traffic3`. Reoccupying a path is legal but arrives as a
    /// fresh sibling, and there is nothing to gain here by inviting it.
    cars_spawned: u64,
}

impl TrafficSpawner {
    /// Increases `retire_ahead` to `spawn_ahead` plus the largest gap if it is
    /// less, with a warning, so a new car never spawns where it would
    /// immediately be retired.
    pub fn new(road: Arc<crate::Waypoints>, mut config: TrafficSpawnerConfig) -> Self {
        let furthest_spawn = config.spawn_ahead + config.gap_range.1;
        if config.retire_ahead < furthest_spawn {
            tracing::warn!(
                "retire_ahead {} m is short of the furthest spawn, so it is increased to {furthest_spawn} m",
                config.retire_ahead,
            );
            config.retire_ahead = furthest_spawn;
        }

        // Return a spawner with no cars yet.
        TrafficSpawner {
            road,
            live_traffic: BTreeMap::new(),
            ego_s: config.ego_start_s,
            cars_spawned: 0,
            config,
        }
    }
}

impl TrafficSpawner {
    /// The cars a run starts with, due at sim time zero. They take the
    /// lanes in turn, each a gap beyond the last in its lane, from
    /// `retire_behind` behind the ego to `retire_ahead` ahead of it, and
    /// keep clear of the ego in its own lane.
    pub fn fill_around_ego(&mut self, world_seed: u64) -> Vec<SpawnTrafficRequest> {
        // The seed the conductor gives the spawner, so the starting cars
        // come from the world seed like everything else in the run.
        let path = ComponentPath::parse(self.id().as_str()).expect("an id is a valid path");
        let mut random = RandomSplitMix64::new(derive_component_seed(world_seed, &path));
        let mut lane_fronts =
            vec![self.ego_s - self.config.retire_behind; self.config.lane_offsets.len()];
        let mut spawns = Vec::with_capacity(self.config.target_population);
        for slot in 0..self.config.target_population {
            let lane = slot % lane_fronts.len();
            let in_ego_lane = (self.config.lane_offsets[lane] - self.config.ego_start_lateral)
                .abs()
                <= self.config.lane_tolerance;
            let mut gap = random.range_f64(self.config.gap_range.0, self.config.gap_range.1);
            if in_ego_lane && lane_fronts[lane] >= self.ego_s {
                gap *= self.config.ego_headway_factor;
            }
            let start_s = lane_fronts[lane] + gap;
            if in_ego_lane && (start_s - self.ego_s).abs() < self.config.gap_range.0 {
                lane_fronts[lane] = self.ego_s;
                continue;
            }
            if start_s > self.ego_s + self.config.retire_ahead {
                continue;
            }
            lane_fronts[lane] = start_s;
            let speed = random.range_f64(self.config.speed_range.0, self.config.speed_range.1);
            spawns.push(self.track_new_car(
                self.config.lane_offsets[lane],
                start_s,
                speed,
                SimTime::ZERO,
            ));
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

    fn subscriptions(&self) -> Vec<Subscription> {
        // The newest pose of every actor, the ego's included: the spawner's
        // decisions are all about where cars are now along the road.
        vec![Subscription::latest(
            KeyExpr::new_rooted("*/actor/**/pose").expect("valid key"),
        )]
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
            if actor_name == self.config.ego_name {
                self.ego_s = actor_s;
            } else if let Some(car) = self.live_traffic.get_mut(actor_name) {
                car.s = actor_s;
                car.lateral = actor_lateral;
            }
        }

        // Retire the cars too far behind or ahead of the ego. Each leaves
        // `live_traffic` as soon as its removal is requested, so the spawner
        // does not ask twice while the request is in flight.
        let retiring: Vec<String> = self
            .live_traffic
            .iter()
            .filter(|&(_, car)| {
                car.s < self.ego_s - self.config.retire_behind
                    || car.s > self.ego_s + self.config.retire_ahead
            })
            .map(|(actor_name, _)| actor_name.clone())
            .collect();
        for actor_name in retiring {
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
        while self.live_traffic.len() < self.config.target_population {
            let gap = random.range_f64(self.config.gap_range.0, self.config.gap_range.1);
            let lane_offset = self.config.lane_offsets
                [random.next_u64() as usize % self.config.lane_offsets.len()];
            let front = lane_front(
                self.live_traffic.values(),
                lane_offset,
                self.config.lane_tolerance,
                self.ego_s,
                self.config.spawn_ahead,
            );
            let start_s = front + gap;
            if start_s > self.ego_s + self.config.retire_ahead {
                // The lane has no room left this side of `retire_ahead`, so
                // spawning waits for the next step.
                break;
            }
            let speed = random.range_f64(self.config.speed_range.0, self.config.speed_range.1);
            let spawn =
                self.track_new_car(lane_offset, start_s, speed, ctx.now() + self.config.period);
            ctx.publish(traffic_spawn_key(ctx.world_name()), &spawn)?;
        }

        // Return the next due time, one period out.
        Ok(ctx.now() + self.config.period)
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
    const GAP_RANGE: (f64, f64) = (25.0, 40.0);

    /// An ego at `EGO_S` in `LANE`, with a lane either side of it.
    fn config() -> TrafficSpawnerConfig {
        TrafficSpawnerConfig {
            period: SimDuration::from_millis(100),
            ego_name: "ego".to_string(),
            ego_start_s: EGO_S,
            ego_start_lateral: LANE,
            lane_offsets: vec![NEXT_LANE, LANE, -NEXT_LANE],
            lane_tolerance: LANE_TOLERANCE,
            target_population: 10,
            spawn_ahead: SPAWN_AHEAD,
            retire_behind: 70.0,
            retire_ahead: 90.0,
            gap_range: GAP_RANGE,
            ego_headway_factor: 2.0,
            speed_range: (22.0, 28.0),
        }
    }

    #[test]
    fn a_retire_ahead_short_of_the_furthest_spawn_is_increased_to_it() {
        let config = TrafficSpawnerConfig {
            retire_ahead: SPAWN_AHEAD,
            ..config()
        };
        let spawner = TrafficSpawner::new(Arc::new(straight_road(1000.0)), config);
        assert_eq!(spawner.config.retire_ahead, SPAWN_AHEAD + GAP_RANGE.1);
    }

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
