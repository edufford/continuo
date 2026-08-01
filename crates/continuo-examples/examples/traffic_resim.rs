//! Open-loop resimulation: the ego runs live while every traffic car it met
//! is replaced by a playback double re-publishing its recorded poses.
//! Nothing is compared — this is the harness for engineering what-ifs.
//! Change the ego's controller, or just the speed it holds, and watch what
//! that does against a traffic scene that stays exactly as recorded.
//!
//! Open-loop means the played-back cars do not react: they drive their
//! recorded trajectories whatever the ego does. That is the point rather
//! than a limitation — holding the scene fixed is what makes two ego
//! variants comparable.
//!
//! The traffic to play back is read out of the log rather than configured.
//! Which cars existed, and when, was the spawner's decision on the recorded
//! run, so the log is the only place that knows. No spawner runs here — the
//! recording *is* the traffic.
//!
//! Run with:
//!   cargo run -p continuo-examples --example traffic_resim -- run.jsonl
//!
//! (Record the log first with `traffic_record`.)

use continuo_conductor::{Conductor, EventLog};
use continuo_core::SimTime;
use continuo_examples::traffic_world;
use continuo_transport::InProcTransport;

/// The what-if: an ego holding 36 m/s against traffic recorded while it held
/// 30, so it reaches the same cars sooner and overtakes more of them.
const WHAT_IF_EGO_SPEED: f64 = 36.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let file = std::env::args()
        .nth(1)
        .ok_or("usage: traffic_resim <file>")?;
    let log = EventLog::read_file(&file)?;

    // The speed is the only thing this run varies; the setup holds the rest
    // at the recorded scenario's values, which is what makes the comparison
    // mean something.
    let mut conductor = Conductor::new(traffic_world::config(), InProcTransport::new())?;
    let num_cars =
        traffic_world::setup_playback_traffic_scenario(&mut conductor, &log, WHAT_IF_EGO_SPEED)?;

    traffic_world::run_playback_traffic_scenario(
        &mut conductor,
        SimTime::from_secs(traffic_world::SIM_SECONDS),
    )?;

    println!(
        "resim done: live ego at {WHAT_IF_EGO_SPEED} m/s against {num_cars} played-back cars; \
         reached sim time {} in {} ticks, world hash {:016x}",
        conductor.sim_time(),
        conductor.tick(),
        conductor.world_hash()
    );

    // Return success; the hybrid run completed.
    Ok(())
}
