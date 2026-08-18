//! A quick check before a commit, rather than a thorough one.
//!
//! CLAUDE.md lists these commands and they were typed by hand every time.
//! This runs them in that order, cheapest first, so a formatting slip is
//! reported in seconds rather than after the workspace has compiled, and it
//! stops at the first failure.
//!
//! It is deliberately not CI, which stays the authority on whether a commit
//! is good: four platforms, both profiles, the packaged FMUs and the
//! recorded-log smokes. What this is for is catching the ordinary mistake
//! before a push, so what matters most about it is that it is fast enough to
//! sit in an editing loop.
//!
//! That is why it packages no FMUs and asks for no features. Packaging costs
//! a release build of the FMU crate whenever a law changed, 13 seconds
//! against 0.7 when nothing did, and `--all-features` resolves features
//! differently from a plain `cargo test`, so alternating between this and one
//! typed by hand rebuilds much of the graph each way, 4 seconds a turn. The
//! packaged-FMU comparison sits behind that feature and so does not run here.
//! CI runs it every time, and CLAUDE.md says to package by hand after editing
//! a law.

use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;

/// One command to run.
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
}

/// Where a step runs.
enum Dir {
    /// The workspace root.
    Root,
    /// The viewer, whose tools are run from their own directory since
    /// `pyproject.toml` is what configures both of them.
    Python,
}

/// What has to answer before the viewer's own tests are worth running.
///
/// Asking whether `pytest` is on the path is not the question, and neither is
/// asking only whether the viewer imports. `pytest` is on plenty of machines
/// that have never installed this viewer, and a half-finished install imports
/// while its drawing and image libraries are missing, which surfaces as
/// failing tests rather than as the setup nobody did. So it names what the
/// suite reaches for, and goes through `python -m` so the interpreter that
/// answers is the one about to run.
const VIEWER_IS_INSTALLED: &[&str] = &["python", "-c", "import continuo_viz, pygame, PIL, pytest"];

/// What has to answer before the viewer's linting is worth running.
///
/// Only the tool, since `ruff` reads files rather than importing anything.
const RUFF_IS_INSTALLED: &[&str] = &["ruff", "--version"];

/// What to type to turn the skipped steps on.
const INSTALL_THE_VIEWER: &str = "python -m pip install -e . pytest ruff   (in python/)";

/// Every command, cheapest first, which is what stopping at the first failure
/// is for.
const STEPS: &[Step] = &[
    Step {
        argv: &["cargo", "fmt", "--all", "--check"],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
    },
    // `--all-features` here but not on the tests below, because linting a
    // target held behind a feature costs only the lint, where testing one
    // costs the packaging it reads and a rebuild of much of the graph.
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
    },
    // Not optional. The crates cross-reference each other heavily, and a
    // renamed item leaves a broken intra-doc link that still compiles.
    Step {
        argv: &["cargo", "doc", "--workspace", "--no-deps"],
        dir: Dir::Root,
        env: &[("RUSTDOCFLAGS", "-D warnings")],
        skip_unless: None,
    },
    Step {
        argv: &["ruff", "check", "."],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(RUFF_IS_INSTALLED),
    },
    Step {
        argv: &["ruff", "format", "--check", "."],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(RUFF_IS_INSTALLED),
    },
    Step {
        argv: &["cargo", "test", "--workspace"],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
    },
    Step {
        argv: &["python", "-m", "pytest", "-v"],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(VIEWER_IS_INSTALLED),
    },
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
    },
];

/// Runs every step, stopping at the first that fails.
pub fn run() -> Result<(), String> {
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
        run_step(step, &root)?;
    }

    report_skips(skipped);

    // Return once every step that could run has passed, which is as much as
    // this claims: CI is what says the commit is good.
    Ok(())
}

/// Runs one step, failing with what it was and what it returned.
fn run_step(step: &Step, root: &Path) -> Result<(), String> {
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
/// otherwise print an error nobody asked for.
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
