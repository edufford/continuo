//! Finding the bundled `.fmu` that this crate is built into.
//!
//! Nothing in the FMU itself uses any of this. It is for the Rust side of
//! the boundary: the golden tests that run the bundled FMU against the
//! laws it was built from, and the demo scenario that hands its path to
//! an `FmuComponent`. Both need the same answer to the same question, so
//! they ask it here rather than each spelling out a path.

use std::path::PathBuf;

use crate::error::BundleError;

/// The file `cargo xtask bundle-fmus` writes, named after the cdylib
/// because FMI takes its model identifier from the shared library.
pub const FMU_FILE_NAME: &str = "continuo_fmu_controller_idm.fmu";

/// The directory the bundled FMU is written to, under `target`.
pub(crate) const FMU_DIRECTORY: &str = "fmu";

/// Where `cargo xtask bundle-fmus` left the bundled FMU.
///
/// The search starts at the running executable and walks up its
/// directories, so it lands on `target/fmu` from a test binary buried in
/// `target/debug/deps` as readily as from an example, whichever profile
/// built either. Taking a path fixed at build time instead would tie the
/// answer to how the caller was compiled.
///
/// Failing carries the command that fixes it, because a missing bundle
/// means a step was not run rather than anything being broken.
pub fn bundled_fmu_path() -> Result<PathBuf, BundleError> {
    let exe = std::env::current_exe().map_err(BundleError::ExeNotFound)?;
    for directory in exe.ancestors() {
        let candidate = directory.join(FMU_DIRECTORY).join(FMU_FILE_NAME);
        if candidate.is_file() {
            return Ok(candidate);
        }
    }

    // Return the failure naming where the search began, which is the
    // only part of it a caller cannot work out.
    Err(BundleError::NotBundled { searched_from: exe })
}
