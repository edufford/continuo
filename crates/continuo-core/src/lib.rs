//! Core types for continuo: simulation time, identifiers, key expressions,
//! spatial math, wire messages, and the [`Component`] trait.
//!
//! Design reference: PLAN.md at the workspace root.

mod component;
mod error;
pub mod hash;
mod ids;
mod keyexpr;
mod math;
mod messages;
pub mod rng;
mod time;

pub use component::{Component, StepCtx};
pub use error::CoreError;
pub use hash::{Fnv1a64, hash_bytes};
pub use ids::{ComponentId, ComponentPath};
pub use keyexpr::KeyExpr;
pub use math::{EulerDeg, EulerRad, Quat, Vec3};
pub use messages::{Message, Pose, TickDone, TickStart};
pub use rng::{DetRng, derive_component_seed};
pub use time::{SimDuration, SimTime};
