//! The base traffic demo: three cars circulating an oval loop, free-run,
//! poses logged once per sim-second. As small as it gets — see
//! `traffic_record`, `traffic_verify`, and `traffic_resim` for the
//! determinism workflows.
//!
//! Run with: `cargo run -p continuo-examples --example traffic`

use continuo_conductor::Conductor;
use continuo_core::SimTime;
use continuo_examples::traffic_world;
use continuo_transport::InProcTransport;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let mut conductor = Conductor::new(traffic_world::config(), InProcTransport::new())?;
    traffic_world::populate(&mut conductor)?;

    // Wall time is fine to read here: the example's main is outside the
    // simulation, where wall clocks are forbidden.
    let started = std::time::Instant::now();
    conductor.run_until(SimTime::from_secs(traffic_world::SIM_SECONDS))?;
    let elapsed = started.elapsed();

    println!(
        "done: world '{}' reached sim time {} in {} ticks (free-run)",
        conductor.world_name(),
        conductor.sim_time(),
        conductor.tick()
    );
    println!(
        "actual time: {:.3} s ({:.0}x real-time), world hash {:016x}",
        elapsed.as_secs_f64(),
        conductor.sim_time().as_secs_f64() / elapsed.as_secs_f64(),
        conductor.world_hash()
    );

    // Return success for the completed run.
    Ok(())
}
