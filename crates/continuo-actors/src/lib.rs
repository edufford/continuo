//! Sample actor components for continuo worlds: a waypoint path, a
//! path-following controller, unicycle physics, and a pose logger.
//!
//! Together they form the milestone 1 demo actor: a car composite
//! `[controller, physics]` where the controller runs at a slower period and
//! the intra-composite same-instant delivery feeds its command to the
//! physics in the same step.

mod controller;
mod logger;
mod path;
mod physics;

pub use controller::{Cmd, PathFollowController};
pub use logger::PoseLogger;
pub use path::Waypoints;
pub use physics::UnicyclePhysics;

/// Key for an actor's pose in `world`.
pub fn pose_key(world_name: &str, actor: &str) -> continuo_core::KeyExpr {
    continuo_core::KeyExpr::new(format!("continuo/{world_name}/actor/{actor}/pose"))
        .expect("valid pose key")
}

/// Key for an actor's drive command in `world`.
pub fn cmd_key(world_name: &str, actor: &str) -> continuo_core::KeyExpr {
    continuo_core::KeyExpr::new(format!("continuo/{world_name}/actor/{actor}/cmd"))
        .expect("valid cmd key")
}
