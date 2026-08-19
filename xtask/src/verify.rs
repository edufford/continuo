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
//! That is why it packages no FMUs and asks for no features, which
//! `cargo xtask verify-fmus` is for. Packaging costs a release build of the
//! FMU crate whenever a law changed, 13 seconds against 0.7 when nothing did,
//! and the tests it feeds say nothing without it, so the two belong with a
//! change that reaches a law rather than in an editing loop.

use std::path::Path;

use crate::task::{Progress, answers, run_command, workspace_root};

/// One command `verify` runs, fixed at compile time.
struct VerifyCommand {
    /// The program and its arguments. A first word of `cargo` is the cargo
    /// that invoked this, so a toolchain chosen by `+toolchain` holds.
    argv: &'static [&'static str],
    /// Where to run it, under the workspace root.
    dir: Dir,
    /// Environment this one command needs, which is how `RUSTDOCFLAGS`
    /// reaches the doc build without leaking into everything after it.
    env: &'static [(&'static str, &'static str)],
    /// What must answer before this is worth running, or `None` where
    /// nothing may excuse it. A failing probe skips it and says so.
    skip_unless: Option<&'static [&'static str]>,
}

/// Where a command runs.
enum Dir {
    /// The workspace root.
    Root,
    /// The viewer, whose tools are run from their own directory since
    /// `pyproject.toml` is what configures both of them.
    Python,
}

/// What has to answer before the viewer's own tests are worth running.
///
/// Two questions, because a machine can fail either one alone. Whether the
/// viewer is installed has to be asked of the metadata rather than by
/// importing it: these run from `python/`, where `continuo_viz` is a
/// subdirectory, so it imports from the working directory whether or not
/// anything installed it, and an interpreter belonging to an unrelated
/// project said yes to that here. `importlib.metadata` reads what an install
/// wrote, and an editable install registers itself the same way.
///
/// Whether the libraries are there is the other, since an install made with
/// `--no-deps`, or one whose dependencies were removed later, satisfies the
/// first and still cannot draw. `pytest` is in neither, being a development
/// dependency the viewer does not declare.
///
/// Asked of `python` for the same reason the command runs through `python -m`:
/// the interpreter on the path may be some other project's, and it is the one
/// about to run that has to answer.
const VIEWER_AND_PYTEST_IS_INSTALLED: &[&str] = &[
    "python",
    "-c",
    "import pygame, PIL, pytest, importlib.metadata as m; m.distribution('continuo-viz')",
];

/// What has to answer before the viewer's linting is worth running.
///
/// Only the tool, since `ruff` reads files rather than importing anything.
const RUFF_IS_INSTALLED: &[&str] = &["ruff", "--version"];

/// What to type to turn the skipped commands on.
const INSTALL_THE_VIEWER: &str = "python -m pip install -e . pytest ruff   (in python/)";

/// Every command, cheapest first, which is what stopping at the first failure
/// is for.
const VERIFY_COMMANDS: &[VerifyCommand] = &[
    VerifyCommand {
        argv: &["cargo", "fmt", "--all", "--check"],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
    },
    // `--all-features` here but not on the tests below, because linting a
    // target held behind a feature costs only the lint, where testing one
    // costs the packaging it reads and a rebuild of much of the graph.
    VerifyCommand {
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
    VerifyCommand {
        argv: &["cargo", "doc", "--workspace", "--no-deps"],
        dir: Dir::Root,
        env: &[("RUSTDOCFLAGS", "-D warnings")],
        skip_unless: None,
    },
    VerifyCommand {
        argv: &["ruff", "check", "."],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(RUFF_IS_INSTALLED),
    },
    VerifyCommand {
        argv: &["ruff", "format", "--check", "."],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(RUFF_IS_INSTALLED),
    },
    VerifyCommand {
        argv: &["cargo", "test", "--workspace"],
        dir: Dir::Root,
        env: &[],
        skip_unless: None,
    },
    VerifyCommand {
        argv: &["python", "-m", "pytest", "-v"],
        dir: Dir::Python,
        env: &[],
        skip_unless: Some(VIEWER_AND_PYTEST_IS_INSTALLED),
    },
    VerifyCommand {
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

/// Runs every command, stopping at the first that fails.
pub fn run() -> Result<(), String> {
    let root = workspace_root();
    let mut progress = Progress::new();

    for command in VERIFY_COMMANDS {
        let work_label = label_from(command);
        if command
            .skip_unless
            .is_some_and(|probe| !answers(probe, &root.join("python")))
        {
            progress.skip(&work_label);
            continue;
        }
        progress.run(&work_label, || run_one(command, &root))?;
    }

    progress.report(&format!(
        "The viewer's checks were skipped, which `{INSTALL_THE_VIEWER}` turns \
         on. CI runs them either way."
    ));

    // Return once every command that could run has passed, which is as much as
    // this claims: CI is what says the commit is good.
    Ok(())
}

/// Runs a command in the directory it belongs to.
fn run_one(command: &VerifyCommand, root: &Path) -> Result<(), String> {
    let dir = match command.dir {
        Dir::Root => root.to_path_buf(),
        Dir::Python => root.join("python"),
    };

    run_command(command.argv, &dir, command.env)
}

/// The label a command is announced and timed under, which is what a person
/// would have typed.
fn label_from(command: &VerifyCommand) -> String {
    let typed = command.argv.join(" ");

    // Return it with the directory named where it is not the root, since two
    // `ruff` lines with nothing to tell them apart would read as a repeat.
    match command.dir {
        Dir::Root => typed,
        Dir::Python => format!("{typed}   (in python/)"),
    }
}
