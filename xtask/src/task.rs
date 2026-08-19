//! What the checking tasks do alike: run something, say what it cost, and
//! say what was left out.
//!
//! Both of them are a list of work with a timing table under it, and the
//! difference is only what is on the list, so the running and the reporting
//! live here rather than in whichever task grew them first.

use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Instant;

/// What a task has run, and what each piece of it cost.
pub(crate) struct Progress {
    took_sec: Vec<(String, f64)>,
    skipped: bool,
}

impl Progress {
    pub(crate) fn new() -> Self {
        Progress {
            took_sec: Vec::new(),
            skipped: false,
        }
    }

    /// Announces a piece of work, runs it, and records what it cost.
    ///
    /// Takes the work as a closure rather than a command, since one of them
    /// is a call into another task's module rather than a process to spawn,
    /// and both belong in the same table.
    pub(crate) fn run(
        &mut self,
        work_label: &str,
        work: impl FnOnce() -> Result<(), String>,
    ) -> Result<(), String> {
        println!("--- {work_label}");
        let started = Instant::now();
        work()?;
        self.took_sec
            .push((work_label.to_string(), started.elapsed().as_secs_f64()));

        Ok(())
    }

    /// Says a piece of work was passed over, and remembers that it was.
    pub(crate) fn skip(&mut self, work_label: &str) {
        println!("--- skipping: {work_label}");
        self.skipped = true;
    }

    /// Prints what each piece cost, then what a pass does not cover.
    ///
    /// The table is here because a check earns its place by being quick, and
    /// what keeps that true is the cost of adding to it being on the screen
    /// every run rather than something to go and measure.
    pub(crate) fn report(&self, when_skipped: &str) {
        let total: f64 = self.took_sec.iter().map(|(_, seconds)| seconds).sum();
        println!();
        for (work_label, seconds) in &self.took_sec {
            println!("{seconds:>7.1} s   {work_label}");
        }

        // As wide as the column it closes, seven for the number and two for
        // the unit, so the total reads as a sum of what is above it.
        println!("{:-<9}", "");
        println!("{total:>7.1} s   in total");

        println!();
        if self.skipped {
            println!("Everything that ran passed. {when_skipped}");
        } else {
            println!("Everything passed.");
        }
    }
}

/// Runs one command, failing with what it was and what it returned.
///
/// A first word of `cargo` is the cargo that invoked this, so a toolchain
/// chosen by `+toolchain` holds for everything a task runs.
pub(crate) fn run_command(argv: &[&str], dir: &Path, env: &[(&str, &str)]) -> Result<(), String> {
    let program = if argv[0] == "cargo" {
        crate::cargo_path()
    } else {
        argv[0].to_string()
    };
    let mut command = Command::new(&program);
    command.args(&argv[1..]);
    command.current_dir(dir);
    for (name, value) in env {
        command.env(name, value);
    }

    let status = command.status().map_err(|error| match error.kind() {
        io::ErrorKind::NotFound => format!("cannot find {program} on the path"),
        _ => format!("cannot run {program}: {error}"),
    })?;
    if !status.success() {
        return Err(format!("`{}` failed ({status})", argv.join(" ")));
    }

    Ok(())
}

/// Whether a probe answers, which is what decides that a step can run.
///
/// Output thrown away, since what is wanted is the exit code and a machine
/// missing the tool would otherwise print an error nobody asked for.
pub(crate) fn answers(probe: &[&str], dir: &Path) -> bool {
    Command::new(probe[0])
        .args(&probe[1..])
        .current_dir(dir)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .is_ok_and(|status| status.success())
}

/// The root of this checkout.
///
/// Taken from where this crate was compiled rather than from the working
/// directory, so a task means the same thing from anywhere in the tree.
pub(crate) fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the xtask crate sits one level under the workspace root")
        .to_path_buf()
}
