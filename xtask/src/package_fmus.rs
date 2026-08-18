//! Packaging every FMU crate in the workspace into `target/fmu`.
//!
//! Discovery is by crate-name prefix rather than by a list, so a second FMU
//! crate is packaged by this and by CI with no edit anywhere.

use std::io;
use std::process::Stdio;

/// The crate-name prefix an FMU crate carries.
///
/// Discovery goes by name rather than by a list, so a second FMU crate,
/// such as one carrying a learned model, is packaged by this and by CI
/// with no edit anywhere.
const FMU_CRATE_PREFIX: &str = "continuo-fmu-";

/// What to install when the packaging subcommand is missing.
const INSTALL_CARGO_FMI: &str = "cargo install cargo-fmi";

/// Packages every FMU crate in the workspace.
pub fn run() -> Result<(), String> {
    let fmu_crates = fmu_crates()?;
    if fmu_crates.is_empty() {
        return Err(format!("no {FMU_CRATE_PREFIX}* crate in this workspace"));
    }
    require_cargo_fmi()?;
    for name in &fmu_crates {
        println!("packaging {name}");
        package(name)?;
    }

    // Return once every one is packaged, having stopped at the first
    // that would not, since a half-packaged target directory is worse
    // than none.
    Ok(())
}

/// The FMU crates this workspace holds, in name order.
///
/// `--no-deps` keeps the answer to workspace members, so a dependency
/// that happened to match the prefix could not join the list. Sorting is
/// for the reader and the log: cargo's order is its own business, and a
/// packaging run that names its crates in a different order each time
/// invites the question of what else moved.
pub(crate) fn fmu_crates() -> Result<Vec<String>, String> {
    let output = crate::cargo()
        .args(["metadata", "--no-deps", "--format-version", "1"])
        .output()
        .map_err(|error| format!("cannot run cargo: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "cargo metadata failed ({})\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    let metadata: serde_json::Value = serde_json::from_slice(&output.stdout)
        .map_err(|error| format!("cannot read cargo metadata: {error}"))?;
    let packages = metadata["packages"]
        .as_array()
        .ok_or("cargo metadata carried no packages")?;
    let mut names: Vec<String> = packages
        .iter()
        .filter_map(|package| package["name"].as_str())
        .filter(|name| name.starts_with(FMU_CRATE_PREFIX))
        .map(str::to_string)
        .collect();
    names.sort();

    // Return the FMU crates, in the order they will be packaged.
    Ok(names)
}

/// Fails with the install command unless `cargo fmi` is there to call.
///
/// Asked once, before anything is packaged, so a missing subcommand is
/// reported as itself rather than as a packaging failure carrying a
/// guess about the cause. `--help` is the probe because it exits zero
/// when the subcommand exists and cargo exits 101 when it does not,
/// which reading cargo's error text would only approximate.
fn require_cargo_fmi() -> Result<(), String> {
    let probed = crate::cargo()
        .args(["fmi", "--help"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    match probed {
        Ok(status) if status.success() => Ok(()),
        Ok(_) => Err(format!("`cargo fmi` is not available: {INSTALL_CARGO_FMI}")),
        Err(error) if error.kind() == io::ErrorKind::NotFound => {
            Err("cannot find cargo on the path".to_string())
        }
        Err(error) => Err(format!("cannot run cargo: {error}")),
    }
}

/// Packages one FMU crate into `target/fmu`.
///
/// `--release` because the default is the debug profile, and how a
/// packaged FMU is optimized is settled when it is packaged: a host
/// loads the binary it finds and cannot rebuild it. The two profiles
/// answer bit for bit alike, which is what lets a release FMU be compared
/// against natively built laws.
fn package(name: &str) -> Result<(), String> {
    let status = crate::cargo()
        .args(["fmi", "--package", name, "bundle", "--release"])
        .status()
        .map_err(|error| format!("cannot run cargo: {error}"))?;
    if !status.success() {
        return Err(format!("packaging {name} failed ({status})"));
    }

    Ok(())
}
