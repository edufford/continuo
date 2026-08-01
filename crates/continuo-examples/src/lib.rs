//! Shared world-building for the traffic examples.
//!
//! The runnable worlds live in `examples/`, smallest first:
//!
//! - `traffic` — the base demo: an ego car on a straight highway, traffic
//!   spawning ahead of it and retiring once passed, free-run
//!   (`cargo run -p continuo-examples --example traffic`)
//! - `traffic_realtime` — the same world paced to 1× real time
//! - `traffic_record` — records the demo to an event log file
//! - `traffic_verify` — determinism verification of a recorded log
//! - `traffic_resim` — a live ego re-run against played-back traffic

pub mod traffic_world;
