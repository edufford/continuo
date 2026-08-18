//! The checks a change reaching an FMU crate needs, which `verify` leaves out.
//!
//! A `.fmu` carries its own compiled copy of everything it links, so the
//! comparison against it reads whatever was packaged last and a law edited
//! without packaging again leaves the two disagreeing while
//! `cargo test --workspace` stays green. Packaging costs a release build of
//! the FMU crate, and the comparison sits behind a feature that resolves the
//! graph differently, which is why neither belongs in a check meant to sit in
//! an editing loop and both belong here.

use std::path::{Path, PathBuf};

/// Where packaging leaves its archives, under the workspace root.
const PACKAGED_INTO: &str = "target/fmu";

/// What has to answer before the packaged FMUs can be validated.
const FMPY_IS_INSTALLED: &[&str] = &["python", "-c", "import fmpy"];

/// What to type to turn the skipped validation on.
const INSTALL_FMPY: &str = "python -m pip install fmpy";

/// Packages every FMU, validates what came out, and runs the comparison.
pub fn run() -> Result<(), String> {
    println!("--- cargo xtask package-fmus");
    crate::package_fmus::run()?;

    let root = crate::verify::workspace_root();
    let packaged = packaged_fmus(&root)?;
    let validated = validate(&packaged, &root)?;

    println!("--- cargo test --workspace --all-features");
    crate::verify::run_command(
        &["cargo", "test", "--workspace", "--all-features"],
        &root,
        &[],
    )?;

    if !validated {
        println!("\nEverything that ran passed, but fmpy is not installed, so");
        println!("the packaged FMUs went unvalidated. `{INSTALL_FMPY}` turns that");
        println!("on, and CI validates them either way.");
    } else {
        println!("\nEverything passed.");
    }

    Ok(())
}

/// The archives packaging just wrote, failing if it wrote none.
///
/// Sorted, so a run names them the same way however the filesystem lists
/// them, and empty is a failure rather than a quiet pass: validating nothing
/// and validating everything look identical from the outside.
fn packaged_fmus(root: &Path) -> Result<Vec<PathBuf>, String> {
    let directory = root.join(PACKAGED_INTO);
    let entries = std::fs::read_dir(&directory)
        .map_err(|error| format!("cannot read {}: {error}", directory.display()))?;
    let mut found: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|kind| kind == "fmu"))
        .collect();
    found.sort();
    if found.is_empty() {
        return Err(format!("no .fmu in {}", directory.display()));
    }

    // Return them in the order they will be validated.
    Ok(found)
}

/// Validates each archive with fmpy, or says why it did not.
///
/// fmpy is a second implementation's reading of what was packaged, which is
/// worth more than anything this workspace could check about its own output.
/// It is skipped rather than required because it is a Python dependency a
/// Rust change has no reason to have, the same bargain the viewer's checks
/// get in `verify`.
fn validate(packaged: &[PathBuf], root: &Path) -> Result<bool, String> {
    if !crate::verify::answers(FMPY_IS_INSTALLED, root) {
        println!("--- skipping: python -m fmpy validate");
        return Ok(false);
    }
    for fmu in packaged {
        let name = fmu.file_name().unwrap_or(fmu.as_os_str());
        println!("--- python -m fmpy validate {}", name.to_string_lossy());
        crate::verify::run_command(
            &["python", "-m", "fmpy", "validate", &fmu.to_string_lossy()],
            root,
            &[],
        )?;
    }

    Ok(true)
}
