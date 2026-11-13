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
    require_cargo_fmi_built_by_active_rustc()?;
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

/// Fails with the rebuild command when `cargo fmi` was built by a
/// different rustc than the active one.
///
/// cargo-fmi calls into the library it packages through Rust's ABI, which
/// is not stable between compiler versions, so a mismatch can crash or
/// package the wrong model.
///
/// Compares the commit cargo-fmi's binary carries with the active rustc's,
/// and warns and passes when either cannot be read.
fn require_cargo_fmi_built_by_active_rustc() -> Result<(), String> {
    let Some(path) = cargo_fmi_binary() else {
        return skip_rustc_check("no cargo-fmi on the path");
    };
    let binary = match std::fs::read(&path) {
        Ok(binary) => binary,
        Err(error) => {
            return skip_rustc_check(&format!("cannot read {}: {error}", path.display()));
        }
    };
    let Some(built_by_commit) = rustc_commit_in_binary(&binary) else {
        return skip_rustc_check(&format!("no single rustc commit in {}", path.display()));
    };
    let Some(active) = active_rustc() else {
        return skip_rustc_check("the active rustc names no commit");
    };
    if built_by_commit != active.commit {
        // The nine characters `rustc -V` shows, so the two line up.
        let short = &built_by_commit[..9];
        return Err(format!(
            "{} was built by rustc commit {short}, but this toolchain is {}.\n\
             `cargo fmi` reads the library it packages through Rust's ABI, which \
             is not stable between compiler versions, so it could crash or \
             package the wrong model.\n\
             Rebuild it on this toolchain with: {INSTALL_CARGO_FMI}",
            path.display(),
            active.version
        ));
    }

    // Return success, cargo-fmi having been built by the active rustc.
    Ok(())
}

/// The `cargo-fmi` binary on the path, where rustup puts the `bin`
/// directory that cargo looks in first.
fn cargo_fmi_binary() -> Option<PathBuf> {
    let name = format!("cargo-fmi{}", std::env::consts::EXE_SUFFIX);
    std::env::split_paths(&std::env::var_os("PATH")?)
        .map(|dir| dir.join(&name))
        .find(|path| path.is_file())
}

/// Warns that the check on which rustc built cargo-fmi was skipped, and
/// why, then passes, so a skipped check is never mistaken for a passed one.
fn skip_rustc_check(reason: &str) -> Result<(), String> {
    eprintln!("warning: not checking which rustc built cargo-fmi: {reason}");
    Ok(())
}

/// The commit of the rustc that built a binary, read from the
/// `/rustc/<commit>/` paths official toolchains leave in std's panic
/// messages. None when the binary names no commit or more than one.
fn rustc_commit_in_binary(binary: &[u8]) -> Option<String> {
    const PREFIX: &[u8] = b"/rustc/";
    const COMMIT_LEN: usize = 40;
    let mut commits = binary
        .windows(PREFIX.len() + COMMIT_LEN + 1)
        .filter(|path| path.starts_with(PREFIX) && path.ends_with(b"/"))
        .map(|path| &path[PREFIX.len()..PREFIX.len() + COMMIT_LEN])
        .filter(|commit| commit.iter().all(u8::is_ascii_hexdigit));
    let commit = commits.next()?;
    if commits.any(|other| other != commit) {
        return None;
    }

    // Return the one commit, which is ASCII since every byte is hex.
    Some(String::from_utf8_lossy(commit).into_owned())
}

/// The rustc that cargo builds with here.
struct ActiveRustc {
    /// The first line of `rustc -vV`, naming the version for the error.
    version: String,
    /// The full commit it was built from, which is what is compared.
    commit: String,
}

/// The rustc cargo builds with, asked from the workspace root so a
/// toolchain file there applies.
fn active_rustc() -> Option<ActiveRustc> {
    let rustc = std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into());
    let output = Command::new(rustc)
        .arg("-vV")
        .current_dir(crate::task::workspace_root())
        .output()
        .ok()
        .filter(|output| output.status.success())?;

    // Return what its verbose version names.
    parse_rustc_verbose_version(&String::from_utf8_lossy(&output.stdout))
}

/// Reads the version and the commit out of `rustc -vV`. None when the
/// commit is `unknown`, as for a rustc built from source.
fn parse_rustc_verbose_version(verbose_version: &str) -> Option<ActiveRustc> {
    let version = verbose_version.lines().next()?.trim().to_string();
    let commit = verbose_version
        .lines()
        .find_map(|line| line.strip_prefix("commit-hash: "))?
        .trim()
        .to_string();
    if commit.len() != 40 || !commit.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return None;
    }

    // Return the version and the commit together.
    Some(ActiveRustc { version, commit })
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
    use super::{active_rustc, parse_rustc_verbose_version, rustc_commit_in_binary};

    /// A commit built at run time, so the test binary the last test scans
    /// carries no commit literal for a `/rustc/` to land beside.
    fn commit(digits: &str) -> String {
        digits.repeat(40 / digits.len())
    }

    #[test]
    fn a_binary_naming_two_rustc_commits_names_neither() {
        let binary = format!(
            "/rustc/{}/a\0/rustc/{}/b",
            commit("ded5c06cf7"),
            commit("0514789570")
        );
        assert!(rustc_commit_in_binary(binary.as_bytes()).is_none());
    }

    #[test]
    fn a_rustc_reporting_an_unknown_commit_names_none() {
        let verbose_version = "rustc 1.92.0-dev\nbinary: rustc\ncommit-hash: unknown\n";
        assert!(parse_rustc_verbose_version(verbose_version).is_none());
    }

    /// Scans the test binary itself, which the active rustc built, so each
    /// platform checks the scan against its own executable format.
    #[test]
    fn this_test_binary_names_the_rustc_that_built_it() {
        let binary = std::fs::read(std::env::current_exe().unwrap()).unwrap();
        let active = active_rustc().expect("rustc names its commit");
        assert_eq!(rustc_commit_in_binary(&binary), Some(active.commit));
    }
}
