//! The conductor: owns simulation time and drives the discrete-event loop.
//!
//! Components self-schedule by returning their next due time from `step`;
//! the conductor advances sim time to the earliest due time, steps the due
//! components in declaration order, and routes their messages with the
//! visibility rule from PLAN.md.

mod conductor;
mod config;
mod error;
mod join;
mod pacing;
mod playback;
pub mod record;
mod registry;
mod schedule;
mod verify;

pub use conductor::Conductor;
pub use config::ConductorConfig;
pub use error::ConductorError;
pub use join::JoinMetadata;
pub use pacing::Pacing;
pub use playback::PlaybackComponent;
pub use record::{EventLog, Recorder, TickFingerprint};
pub use verify::{Divergence, Verifier};
