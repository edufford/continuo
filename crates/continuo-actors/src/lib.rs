//! Sample actor components for continuo worlds: a waypoint path, a
//! path-following controller, unicycle physics, a pose logger, and a
//! traffic spawner.
//!
//! [`PathFollowController`] and [`UnicyclePhysics`] form a car: the
//! composite `[controller, physics]`, where the controller runs at the
//! slower period and intra-composite same-instant delivery feeds its
//! command to the physics within the same step.
//!
//! [`TrafficSpawner`] is what makes a world of those cars *dynamic*. It
//! watches poses and publishes requests to add and remove cars, deciding
//! from sim state so the population it produces is as reproducible as
//! anything else in the run. It cannot act on its own decisions - building
//! a component is not something a component can do - so whoever drives the
//! run applies the requests; see `continuo-examples`.
//!
//! Everything here is demo furniture rather than framework. The path is
//! geometry an actor should not own, and the spawner's requests describe
//! one freeway scenario; both carry TODOs pointing at the world spec and
//! scenario configuration that replace them.

mod controller;
mod logger;
mod path;
mod physics;
mod traffic_spawner;

pub use controller::{Cmd, PathFollowController};
pub use logger::PoseLogger;
pub use path::Waypoints;
pub use physics::UnicyclePhysics;
pub use traffic_spawner::{
    DespawnTrafficRequest, SpawnTrafficRequest, TrafficSpawner, road_pose, straight_road,
    traffic_despawn_key, traffic_spawn_key,
};

/// Key for an actor's pose in `world`.
pub fn pose_key(world_name: &str, actor_name: &str) -> continuo_core::KeyExpr {
    continuo_core::KeyExpr::new(format!("continuo/{world_name}/actor/{actor_name}/pose"))
        .expect("valid pose key")
}

/// Key for an actor's drive command in `world`.
pub fn cmd_key(world_name: &str, actor_name: &str) -> continuo_core::KeyExpr {
    continuo_core::KeyExpr::new(format!("continuo/{world_name}/actor/{actor_name}/cmd"))
        .expect("valid cmd key")
}
