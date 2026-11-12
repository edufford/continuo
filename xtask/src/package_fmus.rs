//! Packaging every FMU crate in the workspace into `target/fmu`.
//!
//! Discovery is by crate-name prefix rather than by a list, so a second FMU
//! crate is packaged by this and by CI with no edit anywhere.

use std::io;
use std::path::PathBuf;
use std::process::{Command, Stdio};

/// The crate-name prefix an FMU crate carries.
///
/// Discovery goes by name rather than by a list, so a second FMU crate,
/// such as one carrying a learned model, is packaged by this and by CI
/// with no edit anywhere.
const FMU_CRATE_PREFIX: &str = "continuo-fmu-";

/// What installs the packaging subcommand, or rebuilds it on the active
/// toolchain. `--force` lets it replace one already installed, so the one
/// command serves both.
const INSTALL_CARGO_FMI: &str = "cargo install cargo-fmi --force";

/// Packages every FMU crate in the workspace.
pub fn run() -> Result<(), String> {
    let fmu_crates = fmu_crates()?;
    if fmu_crates.is_empty() {
        return Err(format!("no {FMU_CRATE_PREFIX}* crate in this workspace"));
    }
    require_cargo_fmi()?;
    warn_if_cargo_fmi_built_by_another_rustc();
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

/// Warns when `cargo fmi` was built by a different rustc than the one
/// about to build the FMU crates, saying how to rebuild it.
///
/// cargo-fmi loads the library it packages and calls into it through
/// Rust's own ABI, taking back a struct describing the model. That ABI is
/// not stable between compiler versions, so a packager left behind by a
/// `rustup update` can read garbage out of a library the new compiler
/// built. It shows up as a crash or an allocation of terabytes, with
/// nothing in it naming the cause.
///
/// A warning rather than an error, since two versions often lay the
/// struct out alike and packaging then works. Rebuilding cargo-fmi here
/// would replace a tool on the user's path without asking.
///
/// Says nothing when either version cannot be read, since cargo records
/// which rustc built a binary only for what `cargo install` put there.
fn warn_if_cargo_fmi_built_by_another_rustc() {
    let Some(built_by) = cargo_install_record()
        .as_deref()
        .and_then(rustc_that_built_cargo_fmi)
    else {
        return;
    };
    let Some(active) = active_rustc() else {
        return;
    };
    if built_by != active {
        eprintln!(
            "warning: `cargo fmi` was built by {built_by}, but this toolchain is \
             {active}.\n\
             It reads the library it packages through Rust's ABI, which is not \
             stable between compiler versions, so packaging may crash.\n\
             Rebuild it on this toolchain with: {INSTALL_CARGO_FMI}"
        );
    }
}

/// The record `cargo install` keeps of what it installed and how.
///
/// It sits in cargo's home, which is `CARGO_HOME` when that is set and
/// `.cargo` under the user's home directory otherwise. The home directory
/// is found the way cargo finds it: `USERPROFILE` on Windows, where cargo
/// ignores the `HOME` that Git Bash and MSYS2 set, and `HOME` elsewhere.
fn cargo_install_record() -> Option<String> {
    let home_var = if cfg!(windows) { "USERPROFILE" } else { "HOME" };
    let cargo_home = std::env::var_os("CARGO_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os(home_var).map(|home| PathBuf::from(home).join(".cargo")))?;

    // Return the record, or nothing when cargo has never installed anything.
    std::fs::read_to_string(cargo_home.join(".crates2.json")).ok()
}

/// The rustc that built the installed cargo-fmi, as `rustc -V` names it,
/// read from cargo's install record.
///
/// The record keeps the whole of `rustc -vV`, and its first line is what
/// `rustc -V` prints, so the two compare directly.
fn rustc_that_built_cargo_fmi(install_record: &str) -> Option<String> {
    let record: serde_json::Value = serde_json::from_str(install_record).ok()?;
    let (_, install) = record["installs"]
        .as_object()?
        .iter()
        .find(|(package, _)| package.starts_with("cargo-fmi "))?;
    let rustc = install["rustc"].as_str()?.lines().next()?;

    // Return the version line alone, which is what the warning shows.
    Some(rustc.to_string())
}

/// The rustc that cargo builds with here, as `rustc -V` names it.
///
/// Asked from the workspace root, so a toolchain file there chooses it
/// for this exactly as it does for cargo.
fn active_rustc() -> Option<String> {
    let rustc = std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into());
    let output = Command::new(rustc)
        .arg("-V")
        .current_dir(crate::task::workspace_root())
        .output()
        .ok()
        .filter(|output| output.status.success())?;

    // Return the version line, trimmed of the newline that ends it.
    Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
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

#[cfg(test)]
mod tests {
    use super::rustc_that_built_cargo_fmi;

    #[test]
    fn the_rustc_that_built_cargo_fmi_is_read_from_the_install_record() {
        let record = r#"{"installs": {
            "ripgrep 14.1.1 (registry+https://github.com/rust-lang/crates.io-index)": {
                "rustc": "rustc 1.80.0 (051478957 2024-07-21)\nrelease: 1.80.0\n"
            },
            "cargo-fmi 0.3.0 (registry+https://github.com/rust-lang/crates.io-index)": {
                "rustc": "rustc 1.92.0 (ded5c06cf 2025-12-08)\nrelease: 1.92.0\n"
            }
        }}"#;
        assert_eq!(
            rustc_that_built_cargo_fmi(record).as_deref(),
            Some("rustc 1.92.0 (ded5c06cf 2025-12-08)")
        );
    }

    #[test]
    fn an_install_record_without_cargo_fmi_names_no_rustc() {
        let record = r#"{"installs": {
            "ripgrep 14.1.1 (registry+https://github.com/rust-lang/crates.io-index)": {
                "rustc": "rustc 1.80.0 (051478957 2024-07-21)\n"
            }
        }}"#;
        assert_eq!(rustc_that_built_cargo_fmi(record), None);
    }
}
