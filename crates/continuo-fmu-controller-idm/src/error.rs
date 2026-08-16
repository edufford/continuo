//! What this crate refuses, on both sides of the boundary.
//!
//! [`BadInput`] is what the FMU tells a host at run time, in the one
//! channel FMI gives it: a status and a line of text. [`PackageError`] is
//! what a Rust caller gets when the packaged `.fmu` has not been built.
//! They share nothing but being refusals, and they sit together for the
//! same reason every crate here keeps its errors in one place: it is
//! where a reader looks for what can go wrong.

use std::path::PathBuf;

use crate::MAX_WAYPOINTS;
use crate::package_fmu::{FMU_DIRECTORY, FMU_FILE_NAME};

/// Something the FMU was handed that no controller could run on.
///
/// Each carries the value that arrived, because a host setting a
/// parameter from another tool has no other way to see what it sent. The
/// text is the whole of what crosses back, since FMI carries a status and
/// a log line rather than anything structured.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub(crate) enum BadInput {
    #[error("road_point_count is {given}, and {kind} roads need {least} to {MAX_WAYPOINTS}")]
    PointCount {
        given: usize,
        least: usize,
        kind: &'static str,
    },

    #[error("the first {count} road points are all the same place, which is a road of no length")]
    RoadOfNoLength { count: usize },

    #[error("{name} is {given}, and it has to be a positive number")]
    NotPositive { name: &'static str, given: f64 },
}

/// Why the packaged FMU could not be found.
#[derive(Debug, thiserror::Error)]
pub enum PackageError {
    #[error("cannot find the running executable to search from: {0}")]
    ExeNotFound(#[source] std::io::Error),

    #[error(
        "no {FMU_DIRECTORY}/{FMU_FILE_NAME} in {} or any directory above it: \
         run `cargo install cargo-fmi` once, then `cargo xtask package-fmus`",
        searched_from.display()
    )]
    NotPackaged { searched_from: PathBuf },
}
