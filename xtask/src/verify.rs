//! What has to pass before a commit, in CI's order.
//!
//! CLAUDE.md lists these commands and they were typed by hand every time,
//! which is tedious but not the reason this exists. The reason is that the
//! commands typed locally and the ones CI runs drifted apart: CI splits its
//! tests as `--lib` then `--test '*'`, where one `cargo test --workspace`
//! reads as covering both, and it does until a target sits behind a feature.
//! A glob names every target it matches, so cargo refuses the whole step over
//! a target whose features are off, where the unqualified form skips it and
//! passes. That went green locally and red on all four agents.
//!
//! So this types CI's own commands. CI stays the authority, and
//! `every_step_runs_a_command_ci_runs` holds this file to that file rather
//! than the other way round.
//!
//! CI's separate debug build is the one step left out, and for a mechanical
//! reason rather than taste: it rebuilds `xtask.exe`, which is the binary
//! running this, and Windows refuses to replace a running one. What CI gains
//! from it is a compile error reported as itself rather than from inside a
//! test step, and clippy has already compiled the same selection two steps
//! earlier here.

use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;

/// One command to run, and what makes it the same command CI runs.
struct Step {
    /// The program and its arguments. A first word of `cargo` is the cargo
    /// that invoked this, so a toolchain chosen by `+toolchain` holds.
    argv: &'static [&'static str],
    /// Where to run it, under the workspace root.
    dir: Dir,
    /// Environment this one command needs, which is how `RUSTDOCFLAGS`
    /// reaches the doc build without leaking into everything after it.
    env: &'static [(&'static str, &'static str)],
    /// What must answer before this step is worth running, or `None` where
    /// nothing may excuse it. A failing probe skips the step and says so.
    skip_unless: Option<&'static [&'static str]>,
    /// The text `.github/workflows/ci.yml` must contain for this to still be
    /// CI's command. `None` says it deliberately is not, and the test below
    /// allows exactly one of those.
    ///
    /// Nothing reads it at run time, which is the point: it is a claim about
    /// CI for the tests to check, not something a run consults.
    #[cfg_attr(not(test), allow(dead_code))]
    in_ci: Option<&'static str>,
}

/// Where a step runs.
enum Dir {
    /// The workspace root.
    Root,
    /// The viewer, whose tools are run from their own directory as CI runs
    /// them, since `pyproject.toml` is what configures both of them.
    Python,
}

/// What has to answer before the viewer's own tests are worth running.
///
/// Asking whether `pytest` is on the path is not the question, and neither is
/// asking only whether the viewer imports. `pytest` is on plenty of machines
/// that have never installed this viewer, and a half-finished install imports
/// while its drawing and image libraries are missing, which surfaces as five
/// failing tests rather than as the setup nobody did.
///
/// So it names what the suite reaches for rather than what `pyproject.toml`
/// declares, and it goes through `python -m` for the same reason the step
/// does: a `pytest` from some other environment would answer for an
/// interpreter that is not the one about to run.
const VIEWER_IS_INSTALLED: &[&str] = &["python", "-c", "import continuo_viz, pygame, PIL, pytest"];

/// What has to answer before the viewer's linting is worth running.
///
/// Only the tool, since `ruff` reads files rather than importing anything.
const RUFF_IS_INSTALLED: &[&str] = &["ruff", "--version"];

/// What to type to turn the skipped steps on.
const INSTALL_THE_VIEWER: &str = "python -m pip install -e . pytest ruff   (in python/)";

/// Every command, cheapest first, which is what stopping at the first failure
/// is for: a formatting slip is reported in seconds rather than after the
/// workspace has compiled.
const STEPS: &[Step] = &[
    Step {
        argv: &["cargo", "fmt", "--all", "--check"],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
        in_ci: Some("cargo fmt --all --check"),
    },
    Step {
        argv: &[
            "cargo",
            "clippy",
            "--workspace",
            "--all-targets",
            "--all-features",
            "--",
            "-D",
            "warnings",
        ],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
        in_ci: Some("cargo clippy --workspace --all-targets --all-features -- -D warnings"),
    },
    Step {
        argv: &["cargo", "doc", "--workspace", "--no-deps"],
        dir: Dir::Root,
        env: &[("RUSTDOCFLAGS", "-D warnings")],
        skip_unless: None,
        in_ci: Some("cargo doc --workspace --no-deps"),
    },
    Step {
        argv: &["ruff", "check", "."],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(RUFF_IS_INSTALLED),
        in_ci: Some("ruff check ."),
    },
    Step {
        argv: &["ruff", "format", "--check", "."],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(RUFF_IS_INSTALLED),
        in_ci: Some("ruff format --check ."),
    },
    // Before the tests, because the packaged-FMU comparison reads the `.fmu`
    // this writes. Skipping it would leave that comparison passing against
    // whatever was packaged last, which is the failure it exists to catch.
    Step {
        argv: &["cargo", "xtask", "package-fmus"],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
        in_ci: Some("cargo xtask package-fmus"),
    },
    Step {
        argv: &[
            "cargo",
            "test",
            "--workspace",
            "--all-features",
            "--lib",
            "--bins",
        ],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
        in_ci: Some("cargo test --workspace --all-features --lib --bins"),
    },
    // `*` is passed as one argument rather than through a shell, so nothing
    // expands it on the way and cargo reads the glob itself, on every
    // platform.
    Step {
        argv: &[
            "cargo",
            "test",
            "--workspace",
            "--all-features",
            "--test",
            "*",
        ],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
        in_ci: Some("cargo test --workspace --all-features --test '*'"),
    },
    Step {
        argv: &["python", "-m", "pytest", "-v"],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(VIEWER_IS_INSTALLED),
        in_ci: Some("python -m pytest -v"),
    },
    // The one command here that is not CI's. CI smokes the demo against the
    // release profile it ships, which costs a release build of the workspace;
    // before a commit the debug run answers the same question, which is
    // whether the demo still runs and what hash it reaches.
    Step {
        argv: &[
            "cargo",
            "run",
            "-p",
            "continuo-examples",
            "--example",
            "traffic",
        ],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
        in_ci: None,
    },
];

/// Runs every step, stopping at the first that fails.
pub fn verify() -> Result<(), String> {
    let root = workspace_root();
    let mut skipped = false;

    for step in STEPS {
        let shown = shown(step);
        if step.skip_unless.is_some_and(|probe| !answers(probe, &root)) {
            println!("--- skipping: {shown}");
            skipped = true;
            continue;
        }
        println!("--- {shown}");
        run(step, &root)?;
    }

    report_skips(skipped);

    // Return once every step that could run has passed, which is what the
    // caller takes as leave to commit.
    Ok(())
}

/// Runs one step, failing with what it was and what it returned.
fn run(step: &Step, root: &Path) -> Result<(), String> {
    let program = if step.argv[0] == "cargo" {
        crate::cargo_path()
    } else {
        step.argv[0].to_string()
    };
    let mut command = Command::new(&program);
    command.args(&step.argv[1..]);
    command.current_dir(match step.dir {
        Dir::Root => root.to_path_buf(),
        Dir::Python => root.join("python"),
    });
    for (name, value) in step.env {
        command.env(name, value);
    }

    let status = command.status().map_err(|error| match error.kind() {
        io::ErrorKind::NotFound => format!("cannot find {program} on the path"),
        _ => format!("cannot run {program}: {error}"),
    })?;
    if !status.success() {
        return Err(format!("`{}` failed ({status})", shown(step)));
    }

    Ok(())
}

/// Whether a probe answers, which is what decides that a step can run.
///
/// Run from the viewer's directory and with its output thrown away, since
/// what is wanted is the exit code and a machine missing the tool would
/// otherwise print a shell error nobody asked for.
fn answers(probe: &[&str], root: &Path) -> bool {
    Command::new(probe[0])
        .args(&probe[1..])
        .current_dir(root.join("python"))
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .is_ok_and(|status| status.success())
}

/// The step as a person would have typed it.
fn shown(step: &Step) -> String {
    let command = step.argv.join(" ");

    // Return it with the directory named where it is not the root, since two
    // `ruff` lines with nothing to tell them apart would read as a repeat.
    match step.dir {
        Dir::Root => command,
        Dir::Python => format!("{command}   (in python/)"),
    }
}

/// Says what was not run, so a pass never reads as more than it was.
fn report_skips(skipped: bool) {
    if !skipped {
        println!("\nEverything passed.");
        return;
    }
    println!(
        "\nEverything that ran passed. The viewer's checks were skipped, \
         which `{INSTALL_THE_VIEWER}` turns on. CI runs them either way."
    );
}

/// The root of this checkout.
///
/// Taken from where this crate was compiled rather than from the working
/// directory, so `cargo xtask verify` means the same thing from anywhere in
/// the tree.
fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the xtask crate sits one level under the workspace root")
        .to_path_buf()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// CI is the authority on what has to pass, and this file follows it. A
    /// command here that CI does not run is a divergence to fix rather than a
    /// feature, and it would be invisible: the local run would go green over
    /// something no agent ever checks, which is the shape of the failure this
    /// whole task exists to stop.
    #[test]
    fn every_step_runs_a_command_ci_runs() {
        let workflow = std::fs::read_to_string(workspace_root().join(".github/workflows/ci.yml"))
            .expect("CI's workflow is readable");
        let commands = ci_commands(&workflow);

        for step in STEPS {
            let Some(in_ci) = step.in_ci else {
                continue;
            };
            assert!(
                commands.contains(&in_ci),
                "ci.yml no longer runs `{in_ci}`, which `cargo xtask verify` \
                 still does. CI is the authority here, so the step in this \
                 file is the one to change."
            );
        }
    }

    /// Every command CI runs, one per line, as a person would have typed it.
    ///
    /// A `run:` step carries its command on that key and a `run: |` block
    /// carries one per line beneath it, so trimming and dropping a leading
    /// `run: ` leaves the command either way.
    ///
    /// Whole lines rather than a search of the file, because a search goes on
    /// matching a command CI has since grown a flag onto. Appending
    /// `--quiet` to CI's unit test step leaves the old command inside the new
    /// one, and a substring check would call that unchanged.
    fn ci_commands(workflow: &str) -> Vec<&str> {
        workflow
            .lines()
            .map(str::trim)
            .map(|line| line.strip_prefix("run: ").unwrap_or(line))
            .collect()
    }

    /// The demo smoke is the one command deliberately not CI's, and it says
    /// why where it is declared. A second one arriving without that argument
    /// is what this catches, since `None` is otherwise a quiet way to opt out
    /// of the check above.
    #[test]
    fn only_the_demo_smoke_departs_from_ci() {
        let departing: Vec<String> = STEPS
            .iter()
            .filter(|step| step.in_ci.is_none())
            .map(shown)
            .collect();
        assert_eq!(
            departing,
            vec!["cargo run -p continuo-examples --example traffic".to_string()],
            "a step stopped matching CI without saying so"
        );
    }
}
