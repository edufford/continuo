//! Sample actor components for continuo worlds: a waypoint path, a
//! path-following controller, unicycle physics, a pose logger, and a
//! traffic spawner.
//!
//! A controller commands and a plant integrates, and they meet on the
//! messages in [`commands`]: one per axis, normalized, so different
//! components can answer a car's two halves and neither has to know what
//! car it is driving.
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
//! drives the run processes the requests; see `continuo-examples`.
//!
//! Everything here is demo furniture rather than framework. The path is
//! geometry an actor should not own, and the spawner's requests describe
//! one freeway scenario; both carry TODOs pointing at the world spec and
//! scenario configuration that replace them.

pub mod commands;
pub mod control_laws;
mod controller;
mod logger;
mod path;
mod physics;
mod traffic_spawner;

use continuo_core::KeyExpr;

pub use commands::{AccelCmd, SteerCmd};
pub use controller::PathFollowController;
pub use logger::PoseLogger;
pub use path::Waypoints;
pub use physics::{CarState, DriveLimits, UnicyclePhysics};
pub use traffic_spawner::{
    DespawnTrafficRequest, SpawnTrafficRequest, TrafficSpawner, road_pose, straight_road,
    traffic_despawn_key, traffic_spawn_key,
};

/// How many detections a scan carries at most.
///
/// A placeholder for demo development, picked well above anything the
/// scenarios here produce so truncation is not a question yet. It settles
/// once a sensor has a range and a scenario has a traffic density, and
/// until then the arguments for any particular number are guesswork.
///
/// A consumer wanting the scan as a fixed-length array reads it from
/// here, so the cap and the array are one number.
pub const MAX_DETECTIONS: usize = 64;

/// Key for an actor's pose in `world`.
pub fn pose_key(world_name: &str, actor_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/actor/{actor_name}/pose")).expect("valid pose key")
}

/// Key for the acceleration commanded to an actor in `world`.
pub fn accel_cmd_key(world_name: &str, actor_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/actor/{actor_name}/accel_cmd"))
        .expect("valid accel command key")
}

/// Key for the steering commanded to an actor in `world`.
pub fn steer_cmd_key(world_name: &str, actor_name: &str) -> KeyExpr {
    KeyExpr::new_rooted(format!("{world_name}/actor/{actor_name}/steer_cmd"))
        .expect("valid steer command key")
}
