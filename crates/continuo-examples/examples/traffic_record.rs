//! Records the traffic demo to an event log file (messages + tick
//! fingerprints, human-readable JSON lines).
//!
//! Run with:
//!   cargo run -p continuo-examples --example traffic_record -- run.jsonl
//!
//! The log is consumed by `traffic_verify` (determinism check) and
//! `traffic_resim` (open-loop resimulation).

use continuo_conductor::{Conductor, Recorder};
use continuo_core::SimTime;
use continuo_examples::traffic_world;
use continuo_transport::{InProcTransport, MonitorTransport};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let file = std::env::args()
        .nth(1)
        .ok_or("usage: traffic_record <file>")?;

    // The recorder taps the two observation points: a transport monitor for
    // every published message, the tick callback for every fingerprint. It
    // takes the same config the conductor runs with, so the log header
    // always names the run that produced it.
    let config = traffic_world::config();
    let recorder = Recorder::new(&config);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
    let mut conductor = Conductor::new(config, transport)?;
    conductor.set_tick_callback(recorder.tick_callback());
    conductor.set_membership_callback(recorder.membership_callback());
    traffic_world::populate(&mut conductor)?;

    conductor.run_until(SimTime::from_secs(traffic_world::SIM_SECONDS))?;

    let log = recorder.finish();
    log.write_file(&file)?;
    println!(
        "recorded {} events to {file}, world hash {:016x}",
        log.events.len(),
        conductor.world_hash()
    );

    // Return success for the completed recording.
    Ok(())
}
