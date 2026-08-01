//! Determinism verification: re-runs the traffic demo with every component
//! live, checking each event against a recorded log as it happens. The
//! re-run stops at the first divergence and exits non-zero — divergence
//! here means determinism is broken (or the log was modified). Nothing
//! from the log enters the simulation.
//!
//! Run with:
//!   cargo run -p continuo-examples --example traffic_verify -- run.jsonl
//!
//! (Record the log first with `traffic_record`.)

use continuo_conductor::{Conductor, EventLog, Verifier};
use continuo_core::SimTime;
use continuo_examples::traffic_world::{self, TrafficRequestHandler};
use continuo_transport::{InProcTransport, MonitorTransport};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let file = std::env::args()
        .nth(1)
        .ok_or("usage: traffic_verify <file>")?;
    let expected = EventLog::read_file(&file)?;

    // The verifier taps the same observation points the recorder used,
    // comparing instead of collecting. It checks the log header against the
    // config this run is about to use — the log does not get to say what is
    // being replayed.
    let config = traffic_world::config();
    let verifier = Verifier::new(expected, &config);
    let traffic_request_handler = TrafficRequestHandler::default();
    let mut conductor = Conductor::new(
        config,
        traffic_request_handler.wrap_transport(MonitorTransport::new(
            InProcTransport::new(),
            verifier.message_callback(),
        )),
    )?;
    conductor.set_tick_callback(verifier.tick_callback());
    conductor.set_membership_callback(verifier.membership_callback());
    traffic_world::setup_live_traffic_scenario(&mut conductor)?;

    // The same driver every other example uses, which is what makes this a
    // re-run rather than a lookalike; the verifier only adds a reason to
    // stop early.
    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_request_handler,
        SimTime::from_secs(traffic_world::SIM_SECONDS),
        Some(&verifier),
    )?;

    match verifier.finish() {
        Ok(verified) => println!(
            "verification passed: {verified} events match through sim time {}, \
             final world hash {:016x}",
            conductor.sim_time(),
            conductor.world_hash()
        ),
        Err(divergence) => {
            eprintln!(
                "verification FAILED at sim time {} (stopped early): {divergence}",
                conductor.sim_time()
            );
            std::process::exit(1);
        }
    }

    // Return success: the whole log was verified.
    Ok(())
}
