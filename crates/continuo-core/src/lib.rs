//! Core types for continuo: simulation time, identifiers, key expressions,
//! spatial math, wire messages, and the [`Component`] trait.
//!
//! Design reference: PLAN.md at the workspace root.

mod component;
mod error;
mod ids;
mod keyexpr;
mod math;
mod messages;
mod time;

pub use component::{Component, StepCtx};
pub use error::CoreError;
pub use ids::{ComponentId, ComponentPath};
pub use keyexpr::KeyExpr;
pub use math::{EulerDeg, EulerRad, Quat, Vec3};
pub use messages::{Message, Pose, TickDone, TickStart};
pub use time::{SimDuration, SimTime};
