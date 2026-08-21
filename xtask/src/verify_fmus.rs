//! The checks a change reaching an FMU crate needs, which `verify` leaves out.
//!
//! A `.fmu` carries its own compiled copy of everything it links, so the
//! tests against it read whatever was packaged last, and a law edited
//! without packaging again leaves the two disagreeing while
//! `cargo test --workspace` stays green. Packaging costs a release build of
//! the FMU crate, which is why it does not belong in a check meant to sit in
//! an editing loop and does belong here.
//!
//! The tests ask for the one feature they sit behind rather than for all of
//! them, because this task is about the FMUs. Asking for all of
//! them would add nothing here: `continuo-examples` has `viz` on by default
//! and that enables `continuo-viz-bridge/zenoh`, so a plain `cargo test`
//! already covers the other two, and `verify` runs one.

use std::path::{Path, PathBuf};

use crate::task::{Progress, answers, run_command, run_counting_command, workspace_root};

/// Where packaging leaves its archives, under the workspace root.
const PACKAGED_INTO: &str = "target/fmu";

/// What has to answer before the packaged FMUs can be validated.
const FMPY_IS_INSTALLED: &[&str] = &["python", "-c", "import fmpy"];

/// What to type to turn the skipped validation on.
const INSTALL_FMPY: &str = "python -m pip install fmpy";

/// The feature each FMU crate holds its packaged-FMU tests behind.
const PACKAGED_FMU: &str = "packaged-fmu";

/// Packages every FMU, validates what came out, and runs the tests.
pub fn run() -> Result<(), String> {
    let root = workspace_root();
    let mut progress = Progress::new();

    progress.run("cargo xtask package-fmus", crate::package_fmus::run)?;
    validate(&packaged_fmus(&root)?, &root, &mut progress)?;
    let features = fmu_test_features()?;
    progress.run_tests(
        &format!("cargo test --workspace --features {features}"),
        || {
            run_counting_command(
                &["cargo", "test", "--workspace", "--features", &features],
                &root,
                &[],
            )
        },
    )?;

    progress.report(&format!(
        "The packaged FMUs went unvalidated, which `{INSTALL_FMPY}` turns on. \
         CI validates them either way."
    ));

    Ok(())
}

/// Every FMU crate's test feature, as cargo wants them written.
///
/// Named per crate rather than bare, since a bare name would have to belong
/// to the workspace's default members rather than to one crate in it. Built
/// from the same discovery that packages them, so a second FMU crate is
/// covered here by the edit that adds it and no other. A crate that carries
/// no such feature fails the run naming itself, which is the report worth
/// having: the convention is what makes discovery by prefix work at all.
fn fmu_test_features() -> Result<String, String> {
    // Return them comma separated, which is how cargo takes a list.
    Ok(crate::package_fmus::fmu_crates()?
        .iter()
        .map(|name| format!("{name}/{PACKAGED_FMU}"))
        .collect::<Vec<_>>()
        .join(","))
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
fn validate(packaged: &[PathBuf], root: &Path, progress: &mut Progress) -> Result<(), String> {
    if !answers(FMPY_IS_INSTALLED, root) {
        progress.skip("python -m fmpy validate");
        return Ok(());
    }
    for fmu in packaged {
        let name = fmu.file_name().unwrap_or(fmu.as_os_str()).to_string_lossy();
        let path = fmu.to_string_lossy();
        progress.run(&format!("python -m fmpy validate {name}"), || {
            run_command(&["python", "-m", "fmpy", "validate", &path], root, &[])
        })?;
    }

    Ok(())
}
