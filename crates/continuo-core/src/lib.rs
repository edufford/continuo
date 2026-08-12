//! Core types for continuo: simulation time, identifiers, key expressions,
//! spatial math, wire messages, and the [`Component`] trait.
//!
//! Design reference: PLAN.md at the workspace root.

pub mod base64;
mod component;
mod error;
mod finite;
pub mod hash;
mod ids;
mod keyexpr;
mod math;
mod messages;
pub mod random;
pub mod seed;
mod time;

pub use component::{Component, StepCtx};
pub use error::CoreError;
pub use hash::{HashFnv1a64, hash_bytes};
pub use ids::{ComponentId, ComponentPath};
pub use keyexpr::{KEY_ROOT, KeyExpr};
pub use math::{EulerDeg, EulerRad, Quat, Vec3};
pub use messages::{Message, Pose, TickDone, TickStart};
pub use random::RandomSplitMix64;
pub use seed::{derive_component_seed, mix_seeds};
pub use time::{SimDuration, SimTime};
