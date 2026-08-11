//! Runs an FMI 3.0 Co-Simulation FMU as a continuo [`Component`].
//!
//! An FMU is data here, not a Rust type: a `.fmu` path plus a mapping saying
//! which messages feed which of its variables and where its outputs go. So
//! adding an FMU to a world compiles nothing, which is the whole point of a
//! standard that ships models as binaries.
//!
//! [`Component`]: continuo_core::Component

pub mod convert;
mod error;
mod fmu_component;
mod mapping;

pub use error::FmuConstructionError;
pub use fmu_component::{FmuComponent, fixture_path};
pub use mapping::{
    FmuMapping, InputBinding, OutputBinding, json_pointer_from_name, json_pointers_for_array,
    json_pointers_for_dimensions,
};
