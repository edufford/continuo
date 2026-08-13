//! Sample actor components for continuo worlds: a waypoint path, a
//! path-following controller, unicycle physics, a pose logger, and a
//! traffic spawner.
//!
//! [`control_laws`] holds the laws themselves, as pure functions over
//! their arguments. A controller component is then the wiring around one:
//! read the inbox, call the law, publish the answer.
//!
//! [`PathFollowController`] and [`UnicyclePhysics`] form a car: the
//! composite `[controller, physics]`, where the controller runs at the
//! slower period and intra-composite same-instant delivery feeds its
//! command to the physics within the same step.
//!
//! [`TrafficSpawner`] is what makes a world of those cars *dynamic*. It
//! watches poses and publishes requests to add and remove cars, deciding
//! from sim state so the population it produces is as reproducible as
//! anything else in the run. It cannot act on its own decisions, because
//! building a component is not something a component can do, so whoever
//! drives the
//! run applies the requests; see `continuo-examples`.
//!
//! Everything here is demo furniture rather than framework. The path is
//! geometry an actor should not own, and the spawner's requests describe
//! one freeway scenario; both carry TODOs pointing at the world spec and
//! scenario configuration that replace them.

pub mod control_laws;
mod controller;
mod logger;
mod path;
mod physics;
mod traffic_spawner;

use continuo_core::KeyExpr;

pub use controller::{Cmd, PathFollowController};
pub use logger::PoseLogger;
pub use path::Waypoints;
pub use physics::UnicyclePhysics;
pub use traffic_spawner::{
    DespawnTrafficRequest, SpawnTrafficRequest, TrafficSpawner, road_pose, straight_road,
    traffic_despawn_key, traffic_spawn_key,
};

/// How many detections a scan carries at most, and how long the arrays
/// are that carry one into an FMU.
///
/// A bound rather than a working number. A scan reaches about 120 m down
/// one lane, which holds 26 cars nose to tail at four and a half meters
/// each, and moving traffic leaves gaps, so no world this project runs
/// gets near it. Nothing pays for the headroom on the wire, since a scan
/// carries only what was found and the padding out to a fixed length
/// happens inside the FMU, where a slot past the end holds the free road.
pub const MAX_DETECTIONS: usize = 64;

/// How many points a road may have to cross into an FMU, which declares
/// arrays this long and a count of how many it filled.
///
/// The count exists because the array cannot be trimmed: `fmi-export`
/// 0.3.0 has no way to size one by a parameter. Repeating the last point
/// through the tail instead would hand [`Waypoints::project`] a segment
/// of zero length, so the padding has to be ignored rather than
/// interpreted, and something has to say where it starts.
///
/// It covers the demo's two points with room for a polyline drawn by
/// hand. A road built from a map will want more, and this is where that
/// question first shows.
pub const MAX_WAYPOINTS: usize = 64;

/// Key for an actor's pose in `world`.
pub fn pose_key(world_name: &str, actor_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/actor/{actor_name}/pose")).expect("valid pose key")
}

/// Key for an actor's drive command in `world`.
pub fn cmd_key(world_name: &str, actor_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/actor/{actor_name}/cmd")).expect("valid cmd key")
}
