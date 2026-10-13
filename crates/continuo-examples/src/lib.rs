//! Shared world-building for the traffic examples.
//!
//! The runnable worlds live in `examples/`, smallest first:
//!
//! - `traffic`: the base demo, an ego car on a straight highway, traffic
//!   spawning ahead of it and retiring once passed, free-run
//!   (`cargo run -p continuo-examples --example traffic`)
//! - `traffic_realtime`: the same world paced to 1x real time
//! - `traffic_record`: records the demo to an event log file
//! - `traffic_verify`: determinism verification of a recorded log
//! - `traffic_resim`: a live ego re-run against played-back traffic

pub mod traffic_world;

use tracing_subscriber::filter::{LevelFilter, filter_fn};
use tracing_subscriber::prelude::*;

/// The module path `fmi` gives every message an FMU logs, which is what
/// tells an FMU's log apart from the host's own.
const FMU_MODULE_PATH: &str = "fmu";

/// The logging category an FMU built with `fmi-export` reports each FMI call
/// under. `fmi` passes the category through as the log target.
const FMU_TRACE_CATEGORY: &str = "trace";

/// Sends the examples' log to standard output at `INFO`, without an FMU's
/// trace of the calls it receives.
///
/// The controller FMU logs every call under its `trace` category, one line
/// per car per step. Switching its logging off when it is instantiated would
/// silence its errors as well, which travel the same way and are the only
/// account of why it refused a call, so this drops exactly the messages an
/// FMU logged under that category and lets everything else through. The
/// match is exact on both, since `Targets` would match the target as a
/// prefix and silence anything whose target began with the same word.
pub fn init_logging() {
    let not_an_fmu_trace = filter_fn(|metadata| {
        metadata.target() != FMU_TRACE_CATEGORY || metadata.module_path() != Some(FMU_MODULE_PATH)
    });

    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer().with_target(false))
        .with(LevelFilter::INFO)
        .with(not_an_fmu_trace)
        .init();
}
