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
    Component, ComponentId, CoreError, KeyExpr, Pose, SimDuration, SimTime, StepCtx,
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

/// Keeps the road ahead of the ego populated and retires what it has left
/// behind, watching poses to know where everyone is.
pub struct TrafficSpawner {
    /// How often the road is reviewed. This can be far slower than the
    /// cars it manages: spawns land well ahead of the ego and retirements
    /// well behind it, so a stale position by one period changes nothing
    /// anyone sees.
    period: SimDuration,
    /// The road everyone is measured along. Poses are projected onto it,
    /// so every distance here is an arc length on one shared reference
    /// path rather than a raw coordinate, which is what lets cars in
    /// different lanes be compared, and what keeps this working if the
    /// road ever bends.
    road: Arc<crate::Waypoints>,
    /// Actor whose progress the traffic is arranged around.
    ego_name: String,
    /// Lateral offsets to spawn into. The ego's own lane is not among them:
    /// nothing here models a collision, so traffic stays beside the ego
    /// rather than in front of it.
    lane_offsets: Vec<f64>,
    /// How many cars to keep on the road at once.
    target_population: usize,
    /// Meters ahead of the ego the first car of a fresh stretch appears.
    spawn_ahead: f64,
    /// Meters behind the ego at which a car is retired.
    retire_behind: f64,
    /// Gap range between consecutive spawns, in meters.
    gap_range: (f64, f64),
    /// Speed range for a new car, m/s. Slower than the ego, or it would
    /// never pass anything.
    speed_range: (f64, f64),
    /// Each live car's lane and latest known position, by actor name.
    /// Holds what this spawner has *asked for*, which is why a car is
    /// struck off when its removal is requested rather than when it is
    /// processed: otherwise it would ask twice while the first request was
    /// still in flight.
    live_traffic: BTreeMap<String, LiveCar>,
    /// Latest known position of the ego, the reference every decision here
    /// is measured against.
    ego_s: f64,
    /// How many cars have been created, ever, which is what makes their
    /// names unique. Never decremented, so a retired `traffic3` never
    /// returns as `traffic3`. Reoccupying a path is legal but arrives as a
    /// fresh sibling, and there is nothing to gain here by inviting it.
    cars_spawned: u64,
}

impl TrafficSpawner {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        period: SimDuration,
        road: Arc<crate::Waypoints>,
        ego_name: impl Into<String>,
        lane_offsets: Vec<f64>,
        target_population: usize,
        spawn_ahead: f64,
        retire_behind: f64,
        gap_range: (f64, f64),
        speed_range: (f64, f64),
    ) -> Self {
        // Return a spawner with an empty road; the first step fills it.
        TrafficSpawner {
            period,
            road,
            ego_name: ego_name.into(),
            lane_offsets,
            target_population,
            spawn_ahead,
            retire_behind,
            gap_range,
            speed_range,
            live_traffic: BTreeMap::new(),
            ego_s: 0.0,
            cars_spawned: 0,
        }
    }
}

/// What the spawner knows of one car it asked for.
#[derive(Serialize)]
struct LiveCar {
    /// Index into the spawner's lane offsets. Cars do not change lanes, so
    /// the lane it was spawned into is the lane it is in.
    lane: usize,
    /// Latest known arc length along the road.
    s: f64,
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
            // for all of them, whatever shape the road is.
            let actor_s = self.road.frenet(pose.position.x, pose.position.y).0;
            if actor_name == self.ego_name {
                self.ego_s = actor_s;
            } else if let Some(car) = self.live_traffic.get_mut(actor_name) {
                car.s = actor_s;
            }
        }

        // Retire what the ego has gone past. The car is dropped from the
        // roll here, when the request goes out, not when the driver acts on
        // it: the spawner is tracking what it has *asked for*, so it does
        // not ask twice while a request is in flight.
        let passed: Vec<String> = self
            .live_traffic
            .iter()
            .filter(|&(_, car)| car.s < self.ego_s - self.retire_behind)
            .map(|(actor_name, _)| actor_name.clone())
            .collect();
        for actor_name in passed {
            self.live_traffic.remove(&actor_name);
            ctx.publish(
                traffic_despawn_key(ctx.world_name()),
                &DespawnTrafficRequest { actor_name },
            )?;
        }

        // Top the road back up. Each car goes a gap beyond the furthest car
        // in its lane, measured from where that car is now rather than
        // where it was spawned, since it has driven on since. A lane with
        // nothing that far ahead starts at `spawn_ahead` in front of the
        // ego, so a stretch that has all been passed starts fresh ahead
        // rather than filling in behind.
        let mut random = ctx.step_random();
        while self.live_traffic.len() < self.target_population {
            self.cars_spawned += 1;
            let gap = random.range_f64(self.gap_range.0, self.gap_range.1);
            let lane = random.next_u64() as usize % self.lane_offsets.len();
            let lane_front = self
                .live_traffic
                .values()
                .filter(|car| car.lane == lane)
                .map(|car| car.s)
                .fold(self.ego_s + self.spawn_ahead, f64::max);
            let spawn = SpawnTrafficRequest {
                actor_name: format!("traffic{}", self.cars_spawned),
                lane_offset: self.lane_offsets[lane],
                speed: random.range_f64(self.speed_range.0, self.speed_range.1),
                start_s: lane_front + gap,
                first_due: ctx.now() + self.period,
            };
            self.live_traffic.insert(
                spawn.actor_name.clone(),
                LiveCar {
                    lane,
                    s: spawn.start_s,
                },
            );
            ctx.publish(traffic_spawn_key(ctx.world_name()), &spawn)?;
        }

        // Return the next due time, one period out.
        Ok(ctx.now() + self.period)
    }

    fn state_bytes(&self) -> Option<Vec<u8>> {
        // The roll decides every future request, and none of it is
        // published, so state-hash mode is what catches a divergence here
        // at the tick it happens.
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
    use continuo_core::{ComponentPath, Message};

    use super::*;

    const PERIOD: SimDuration = SimDuration::from_millis(500);

    /// One pose message per car, each at an arc length on `road`'s center
    /// line, as the transport would deliver them.
    fn get_poses(road: &crate::Waypoints, cars: &[(&str, f64)]) -> Vec<Message> {
        // Return the inbox a step would be handed.
        cars.iter()
            .map(|&(name, s)| Message {
                key: KeyExpr::new_rooted(format!("w/actor/{name}/pose")).expect("valid key"),
                publisher: ComponentPath::parse(&format!("{name}/physics")).expect("valid path"),
                seq: 0,
                sim_time: SimTime::ZERO,
                payload: serde_json::to_vec(&road_pose(road, s, 0.0)).expect("a pose serializes"),
            })
            .collect()
    }

    /// Steps `spawner` once and returns the spawn requests it published.
    fn step_once(
        spawner: &mut TrafficSpawner,
        now: SimTime,
        inbox: Vec<Message>,
    ) -> Vec<SpawnTrafficRequest> {
        let mut ctx = StepCtx::new(now, Some(PERIOD), "w", 0, inbox);
        spawner
            .step(&mut ctx)
            .expect("a spawner given readable poses steps");

        // Return the spawn requests, leaving out any despawns.
        ctx.take_outbox()
            .into_iter()
            .filter(|(key, _)| *key == traffic_spawn_key("w"))
            .map(|(_, payload)| serde_json::from_slice(&payload).expect("a spawn request"))
            .collect()
    }

    #[test]
    fn a_new_car_spawns_a_gap_beyond_where_the_last_in_its_lane_has_driven_to() {
        let road = Arc::new(straight_road(1000.0));
        // One lane holding two cars, the first 40 m ahead of the ego and
        // the next 20 m beyond it, at 60 m and 80 m.
        let mut spawner = TrafficSpawner::new(
            PERIOD,
            road.clone(),
            "ego",
            vec![0.0],
            2,
            40.0,
            60.0,
            (20.0, 20.0),
            (20.0, 20.0),
        );
        let first = step_once(
            &mut spawner,
            SimTime::ZERO,
            get_poses(&road, &[("ego", 0.0)]),
        );
        assert_eq!(first.len(), 2, "the empty road fills to its population");

        // The ego leaves `traffic1` far enough behind to retire it, while
        // `traffic2` has driven on from 80 m to 150 m.
        let poses = get_poses(
            &road,
            &[("ego", 100.0), ("traffic1", 30.0), ("traffic2", 150.0)],
        );
        let second = step_once(&mut spawner, SimTime::ZERO + PERIOD, poses);
        assert_eq!(second.len(), 1, "one car replaces the one retired");
        assert!(
            second[0].start_s >= 150.0 + 20.0,
            "the replacement spawned at {} m, within the 20 m gap of the car at 150 m",
            second[0].start_s,
        );
    }
}
