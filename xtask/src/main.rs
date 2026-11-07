//! Workspace tasks that cargo has no command for.
//!
//! Cargo has no user-defined targets and its aliases cannot chain
//! commands, so anything more than one invocation lives in a binary like
//! this one. `.cargo/config.toml` aliases `cargo xtask` to running it,
//! which is what makes it a real entry point somebody types rather than a
//! side effect hidden inside `cargo build`.

mod package_fmus;
mod task;
mod verify;
mod verify_fmus;

use std::process::{Command, ExitCode};

/// The tasks there are, named in one place so a new one cannot be added
/// without the usage line learning about it.
const USAGE: &str = "usage: cargo xtask [package-fmus|verify|verify-fmus]";

fn main() -> ExitCode {
    let task = std::env::args().nth(1);
    let result = match task.as_deref() {
        Some("package-fmus") => package_fmus::run(),
        Some("verify") => verify::run(),
        Some("verify-fmus") => verify_fmus::run(),
        Some(unknown) => Err(format!("unknown task `{unknown}`\n{USAGE}")),
        None => Err(format!("no task given\n{USAGE}")),
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
///
/// The env variables `cargo run` sets to describe `xtask`, such as
/// `CARGO_MANIFEST_DIR` and `CARGO_PKG_NAME`, are removed from this
/// command's environment, so the cargo it starts sees what a cargo command
/// typed in the shell would. xtask's own environment and the shell's are
/// left alone, and cargo sets these again for each crate it builds.
/// A build script can track them, and passing them on would then rebuild
/// that crate and everything above it each time a task and a direct cargo
/// command take turns. `ring`'s does, and Zenoh brings it in with its TLS
/// and QUIC transports, so adding either back would also add that rebuild.
pub fn cargo() -> Command {
    let mut command = Command::new(cargo_path());
    for (env_var_name, _) in std::env::vars_os() {
        if env_var_name.to_str().is_some_and(set_by_cargo_run) {
            command.env_remove(env_var_name);
        }
    }
    command
}

/// Whether `env_var_name` is an env variable `cargo run` sets to describe
/// the package it runs.
fn set_by_cargo_run(env_var_name: &str) -> bool {
    env_var_name.starts_with("CARGO_PKG_")
        || matches!(
            env_var_name,
            "CARGO_MANIFEST_DIR" | "CARGO_MANIFEST_PATH" | "CARGO_CRATE_NAME" | "CARGO_BIN_NAME"
        )
}

/// The path [`cargo`] runs.
fn cargo_path() -> String {
    std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string())
}
