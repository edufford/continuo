//! What `verify` and `verify-fmus` do alike: run something, say what it
//! cost, say how many tests were in it, and say what was left out.
//!
//! Both of them are a list of work with a timing table under it, and the
//! difference is only what is on the list, so the running and the reporting
//! live here rather than in whichever task grew them first.

use std::io::{self, BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Instant;

/// What a task has run, what each piece of it cost, and how much of it was
/// testing.
pub(crate) struct Progress {
    took_sec: Vec<(String, f64)>,
    num_tests: usize,
    skipped: bool,
}

impl Progress {
    pub(crate) fn new() -> Self {
        Progress {
            took_sec: Vec::new(),
            num_tests: 0,
            skipped: false,
        }
    }

    /// Announces a piece of work, runs it, and records what it cost and how
    /// many tests were in it.
    ///
    /// The work answers `None` where running tests was never the point, and
    /// `Some` where it was. `Some(0)` fails, which is the argument
    /// `verify_fmus` already makes about validating no FMUs: a filter that
    /// resolves no targets costs no time and reads in a table of times
    /// exactly like a full run.
    ///
    /// Takes the work as a closure rather than a command, since one piece of
    /// it is a call into another task's module rather than a process to
    /// spawn, and both belong in the same table.
    pub(crate) fn run(
        &mut self,
        work_label: &str,
        work: impl FnOnce() -> Result<Option<usize>, String>,
    ) -> Result<(), String> {
        println!("--- {work_label}");
        let started = Instant::now();
        match work()? {
            Some(0) => return Err(format!("`{work_label}` ran no tests")),
            Some(num_tests) => self.num_tests += num_tests,
            None => {}
        }
        self.took_sec
            .push((work_label.to_string(), started.elapsed().as_secs_f64()));

        Ok(())
    }

    /// Says a piece of work was passed over, and remembers that it was.
    pub(crate) fn skip(&mut self, work_label: &str) {
        println!("--- skipping: {work_label}");
        self.skipped = true;
    }

    /// Prints what each piece cost, then what a pass covered and what it
    /// does not.
    ///
    /// The table is here because a check earns its place by being quick, and
    /// what keeps that true is the cost of adding to it being on the screen
    /// every run rather than something to go and measure. The count is there
    /// for the opposite reason: an elapsed time says a command ran, never
    /// that it found anything to do.
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
        let num_tests = self.num_tests;
        if self.skipped {
            println!("Everything that could run passed. {num_tests} tests ran. {when_skipped}");
        } else {
            println!("Everything passed. {num_tests} tests ran.");
        }
    }
}

/// Runs one command, returning how many tests its output says passed.
///
/// The output is piped rather than inherited, since counting means reading
/// it, and echoed back a line at a time so a long run still says where it
/// has got to. Only stdout is taken, so cargo's progress and the compiler's
/// diagnostics, which both go to stderr, stay live and stay colored.
///
/// A command that runs no tests answers zero, which is why one runner serves
/// every step: what makes zero a failure is the caller asking for the count,
/// not this.
pub(crate) fn run_command(
    argv: &[&str],
    dir: &Path,
    env: &[(&str, &str)],
) -> Result<usize, String> {
    // A first word of `cargo` is the cargo that invoked this, so a toolchain
    // chosen by `+toolchain` holds for everything a task runs.
    let mut command = Command::new(if argv[0] == "cargo" {
        crate::cargo_path()
    } else {
        argv[0].to_string()
    });
    command
        .args(&argv[1..])
        .current_dir(dir)
        .stdout(Stdio::piped());
    for (name, value) in env {
        command.env(name, value);
    }
    let mut child = command.spawn().map_err(|error| match error.kind() {
        // The missing case on its own, since it is the one a person fixes by
        // installing something rather than by reading an errno.
        io::ErrorKind::NotFound => format!("cannot find {} on the path", argv[0]),
        _ => format!("cannot run {}: {error}", argv[0]),
    })?;
    let reading = child.stdout.take().expect("stdout was piped just above");
    let mut num_passed = 0;
    for line in BufReader::new(reading).lines().map_while(Result::ok) {
        println!("{line}");
        num_passed += parse_num_passed(&line).unwrap_or(0);
    }
    let status = child
        .wait()
        .map_err(|error| format!("`{}` never finished: {error}", argv.join(" ")))?;
    if !status.success() {
        return Err(format!("`{}` failed ({status})", argv.join(" ")));
    }

    // Return what the run said it had checked.
    Ok(num_passed)
}

/// The count in `12 passed`, which is how both tools write one: cargo once
/// per test binary, pytest once for the whole run.
///
/// The last run of digits rather than the last word, so an escape sequence
/// sitting against the number does not matter and neither tool has to be
/// asked about color.
fn parse_num_passed(line: &str) -> Option<usize> {
    line.split_once(" passed")?
        .0
        .rsplit(|c: char| !c.is_ascii_digit())
        .next()?
        .parse()
        .ok()
}

/// Whether a probe answers, which is what decides that a step can run.
///
/// Output thrown away, since what is wanted is the exit code and a machine
/// missing the tool would otherwise print an error nobody asked for.
pub(crate) fn answers(probe: &[&str], dir: &Path) -> bool {
    Command::new(probe[0])
        .args(&probe[1..])
        .current_dir(dir)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
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

#[cfg(test)]
mod tests {
    use super::parse_num_passed;

    #[test]
    fn a_cargo_summary_counts_what_passed() {
        let line = "test result: ok. 12 passed; 0 failed; 0 ignored";
        assert_eq!(parse_num_passed(line), Some(12));
    }

    #[test]
    fn a_pytest_summary_counts_what_passed() {
        let line = "===== 21 passed, 1 skipped in 0.53s =====";
        assert_eq!(parse_num_passed(line), Some(21));
    }

    #[test]
    fn a_colored_summary_counts_the_same() {
        let line = "==== \u{1b}[1m21 passed\u{1b}[0m in 0.53s ====";
        assert_eq!(parse_num_passed(line), Some(21));
    }

    #[test]
    fn a_line_saying_nothing_about_tests_counts_nothing() {
        assert_eq!(parse_num_passed("   Compiling continuo-core v0.1.0"), None);
    }
}
