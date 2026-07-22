//! Open-loop resimulation: car1 runs live while car2 and car3 are replaced
//! by playback doubles re-publishing their recorded messages from the log.
//! Nothing is compared — this is the harness for engineering what-ifs:
//! change car1's code or parameters and observe the new behavior against
//! the recorded world (the played-back cars do not react; that is what
//! open-loop means).
//!
//! Run with:
//!   cargo run -p continuo-examples --example traffic_resim -- run.jsonl
//!
//! (Record the log first with `traffic_record`.)

use continuo_conductor::{Conductor, EventLog, PlaybackComponent};
use continuo_core::{ComponentId, SimTime};
use continuo_examples::traffic_world;
use continuo_transport::InProcTransport;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let file = std::env::args()
        .nth(1)
        .ok_or("usage: traffic_resim <file>")?;
    let log = EventLog::read_file(&file)?;

    let mut conductor = Conductor::new(traffic_world::config(), InProcTransport::new())?;
    let path = traffic_world::demo_path();
    traffic_world::add_live_car(&mut conductor, &path, "car1", 0)?;
    for car in ["car2", "car3"] {
        conductor.add_component(
            "",
            Box::new(PlaybackComponent::from_log(
                ComponentId::new(car)?,
                &log,
                car,
            )),
        )?;
    }
    traffic_world::add_logger(&mut conductor)?;

    conductor.run_until(SimTime::from_secs(traffic_world::SIM_SECONDS))?;

    println!(
        "resim done: car1 live, car2/car3 played back from the log; reached sim time {} \
         in {} ticks, world hash {:016x}",
        conductor.sim_time(),
        conductor.tick(),
        conductor.world_hash()
    );

    // Return success; the hybrid run completed.
    Ok(())
}
