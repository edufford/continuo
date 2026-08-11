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
mod fmu_get_set;
mod fmu_mapping;
mod fmu_variable;

use std::path::PathBuf;

pub use error::FmuConstructionError;
pub use fmu_component::FmuComponent;
pub use fmu_mapping::{
    FmuMapping, InputBinding, OutputBinding, escape_json_pointer_token, json_pointer_from_name,
    json_pointers_for_array, json_pointers_for_dimensions, unescape_json_pointer_token,
};

/// Where a vendored reference FMU lives, for tests and examples.
pub fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(format!("{name}.fmu"))
}
