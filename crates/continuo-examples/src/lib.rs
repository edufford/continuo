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

pub mod collision;
pub mod traffic_world;

use tracing_subscriber::filter::LevelFilter;

/// Sends the examples' log to standard output at `INFO`.
pub fn init_logging() {
    tracing_subscriber::fmt()
        .with_target(false)
        .with_max_level(LevelFilter::INFO)
        .init();
}
