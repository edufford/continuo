//! The base traffic demo: an ego car driving a straight highway while
//! traffic spawns ahead of it and retires behind, free-run, each car printed
//! to the console as it joins and leaves. As small as it gets; see
//! `traffic_record`, `traffic_verify`, and `traffic_resim` for the
//! determinism workflows.
//!
//! Run with: `cargo run -p continuo-examples --example traffic`

use continuo_conductor::Conductor;
use continuo_core::SimTime;
use continuo_examples::traffic_world::{self, TrafficHandler};
use continuo_transport::{InProcTransport, MonitorTransport};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    continuo_examples::init_logging();

    let traffic_handler = TrafficHandler::default();
    let mut transport = MonitorTransport::new(InProcTransport::new());
    transport.add_message_callback(traffic_handler.message_callback());
    let mut conductor = Conductor::new(traffic_world::config(), transport)?;
    conductor.add_membership_callback(traffic_handler.membership_callback());
    traffic_world::setup_live_traffic_scenario(&mut conductor, traffic_handler.car_factory())?;

    // Wall time is fine to read here: the example's main is outside the
    // simulation, where wall clocks are forbidden.
    let started = std::time::Instant::now();
    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_handler,
        SimTime::from_secs(traffic_world::SIM_SECONDS),
        None,
    )?;
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
