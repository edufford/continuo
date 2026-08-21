//! What the checking tasks do alike: run something, say what it cost, say
//! how many tests were in it, and say what was left out.
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
    tests: usize,
    skipped: bool,
}

impl Progress {
    pub(crate) fn new() -> Self {
        Progress {
            took_sec: Vec::new(),
            tests: 0,
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

    /// The same for work that runs tests, adding what it ran to the total.
    ///
    /// A step that ran none fails, which is the argument `verify_fmus`
    /// already makes about validating no FMUs: a filter that resolves no
    /// targets costs no time and reads in a table of times exactly like a
    /// full run.
    pub(crate) fn run_tests(
        &mut self,
        work_label: &str,
        work: impl FnOnce() -> Result<usize, String>,
    ) -> Result<(), String> {
        let mut ran = 0;
        self.run(work_label, || {
            ran = work()?;
            Ok(())
        })?;
        if ran == 0 {
            return Err(format!("`{work_label}` ran no tests"));
        }
        self.tests += ran;

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
        let plural = if self.tests == 1 { "" } else { "s" };
        let ran = format!("{} test{plural} ran.", self.tests);
        if self.skipped {
            println!("Everything that could run passed. {ran} {when_skipped}");
        } else {
            println!("Everything passed. {ran}");
        }
    }
}

/// Runs one command, failing with what it was and what it returned.
pub(crate) fn run_command(argv: &[&str], dir: &Path, env: &[(&str, &str)]) -> Result<(), String> {
    let status = command_for(argv, dir, env)
        .status()
        .map_err(|error| cannot_run(argv, &error))?;
    if !status.success() {
        return Err(format!("`{}` failed ({status})", argv.join(" ")));
    }

    Ok(())
}

/// Runs one command and returns how many tests its output says passed.
///
/// Piped rather than inherited, since counting means reading it, and echoed
/// back a line at a time so a long run still says where it has got to. Only
/// stdout is taken: compile progress goes to stderr, so leaving that alone
/// keeps it live and leaves no second pipe to drain. Piping is also what
/// turns a tool's own color off, which costs color on the test output and
/// nothing else.
pub(crate) fn run_counting_command(
    argv: &[&str],
    dir: &Path,
    env: &[(&str, &str)],
) -> Result<usize, String> {
    let mut child = command_for(argv, dir, env)
        .stdout(Stdio::piped())
        .spawn()
        .map_err(|error| cannot_run(argv, &error))?;
    let reading = child.stdout.take().expect("stdout was piped just above");
    let mut tests = 0;
    for line in BufReader::new(reading).lines().map_while(Result::ok) {
        println!("{line}");
        tests += passed_in(&line).unwrap_or(0);
    }
    let status = child
        .wait()
        .map_err(|error| format!("`{}` never finished: {error}", argv.join(" ")))?;
    if !status.success() {
        return Err(format!("`{}` failed ({status})", argv.join(" ")));
    }

    // Return what the run said it had checked.
    Ok(tests)
}

/// The count in `12 passed`, which is how both tools write one: cargo once
/// per test binary, pytest once for the whole run.
///
/// The last run of digits rather than the last word, so an escape sequence
/// sitting against the number does not matter and neither tool has to be
/// asked about color.
fn passed_in(line: &str) -> Option<usize> {
    line.split_once(" passed")?
        .0
        .rsplit(|c: char| !c.is_ascii_digit())
        .next()?
        .parse()
        .ok()
}

/// One command, ready to spawn, with nothing decided about its output.
///
/// A first word of `cargo` is the cargo that invoked this, so a toolchain
/// chosen by `+toolchain` holds for everything a task runs.
fn command_for(argv: &[&str], dir: &Path, env: &[(&str, &str)]) -> Command {
    let mut command = Command::new(if argv[0] == "cargo" {
        crate::cargo_path()
    } else {
        argv[0].to_string()
    });
    command.args(&argv[1..]);
    command.current_dir(dir);
    for (name, value) in env {
        command.env(name, value);
    }

    // Return it for the caller to decide the rest.
    command
}

/// Why a command never started.
fn cannot_run(argv: &[&str], error: &io::Error) -> String {
    // Return the missing case on its own, since it is the one a person fixes
    // by installing something rather than by reading an errno.
    match error.kind() {
        io::ErrorKind::NotFound => format!("cannot find {} on the path", argv[0]),
        _ => format!("cannot run {}: {error}", argv[0]),
    }
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
    use super::passed_in;

    #[test]
    fn a_cargo_summary_counts_what_passed() {
        let line = "test result: ok. 12 passed; 0 failed; 0 ignored";
        assert_eq!(passed_in(line), Some(12));
    }

    #[test]
    fn a_pytest_summary_counts_what_passed() {
        let line = "===== 21 passed, 1 skipped in 0.53s =====";
        assert_eq!(passed_in(line), Some(21));
    }

    #[test]
    fn a_colored_summary_counts_the_same() {
        let line = "==== \u{1b}[1m21 passed\u{1b}[0m in 0.53s ====";
        assert_eq!(passed_in(line), Some(21));
    }

    #[test]
    fn a_line_saying_nothing_about_tests_counts_nothing() {
        assert_eq!(passed_in("   Compiling continuo-core v0.1.0"), None);
    }
}
