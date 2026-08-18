//! Workspace tasks that cargo has no command for.
//!
//! Cargo has no user-defined targets and its aliases cannot chain
//! commands, so anything more than one invocation lives in a binary like
//! this one. `.cargo/config.toml` aliases `cargo xtask` to running it,
//! which is what makes it a real entry point somebody types rather than a
//! side effect hidden inside `cargo build`.

mod package_fmus;
mod verify;

use std::process::{Command, ExitCode};

/// The tasks there are, named in one place so a new one cannot be added
/// without the usage line learning about it.
const USAGE: &str = "usage: cargo xtask [package-fmus|verify]";

fn main() -> ExitCode {
    let task = std::env::args().nth(1);
    let result = match task.as_deref() {
        Some("package-fmus") => package_fmus::run(),
        Some("verify") => verify::run(),
        Some(unknown) => Err(format!(
            "unknown task `{unknown}`
{USAGE}"
        )),
        None => Err(format!(
            "no task given
{USAGE}"
        )),
    };

    // Return the failure as a message rather than a panic, since these
    // are things a person is meant to read and act on.
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::FAILURE
        }
    }
}

/// The cargo that invoked this, or whatever one is on the path.
///
/// `CARGO` is set for anything cargo runs, so a toolchain chosen by
/// `rustup run` or a `+toolchain` argument is the one used throughout
/// rather than being swapped halfway.
pub fn cargo() -> Command {
    Command::new(cargo_path())
}

/// The path [`cargo`] runs, for a caller building its own [`Command`].
pub fn cargo_path() -> String {
    std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string())
}
