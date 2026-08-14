//! A controller for traffic cars, following with IDM and turning with
//! pure pursuit, exported as an FMI 3.0 Co-Simulation FMU.
//!
//! Nothing here decides anything. [`FmuController`] declares what crosses
//! the boundary, and every answer comes from
//! [`continuo_actors::control_laws`], which the native controller calls
//! too. So the two agree because they are one implementation rather than
//! because somebody keeps them in step.
//!
//! What the boundary costs is worth seeing plainly. A road cannot cross
//! as a `Waypoints`, so it crosses as the numbers it was built from and
//! each instance builds its own copy. A scan cannot cross as a list, so
//! it crosses as two arrays of a fixed length, padded out with the free
//! road. Both are the FMI data model rather than a choice, and both are
//! why this crate exists at all: the laws stay where they are, and only
//! the packaging lives here.
//!
//! The `.fmu` carries a compiled snapshot of `continuo-actors`, since the
//! cdylib links it statically. Editing a law without packaging it again
//! therefore leaves this copy behind, and `cargo xtask package-fmus` is
//! what puts it back.

mod error;
mod fmu_controller;
mod package_fmu;

pub use error::PackageError;
pub use fmu_controller::{FmuController, MAX_WAYPOINTS};
pub use package_fmu::{FMU_FILE_NAME, packaged_fmu_path};

// The C entry points FMI loads, generated for the model above. They
// export by name rather than by module path, so this stands as the
// crate's front door whichever file the model itself lives in.
fmi_export::export_fmu!(FmuController);
