//! Shared world-building for the traffic examples.
//!
//! The runnable worlds live in `examples/`, smallest first:
//!
//! - `traffic` — the base demo: three cars circulating an oval, free-run
//!   (`cargo run -p continuo-examples --example traffic`)
//! - `traffic_realtime` — the same world paced to 1× real time
//! - `traffic_record` — records the demo to an event log file
//! - `traffic_verify` — determinism verification of a recorded log
//! - `traffic_resim` — open-loop resimulation against a recorded log

pub mod traffic_world;
