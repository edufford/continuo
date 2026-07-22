//! Milestone 1/2 demo: three cars circulating an oval loop, free-run, with
//! determinism recording.
//!
//! Each car is a composite `carN = [controller, physics]` — the controller
//! (100 ms period) reads the pose from the previous physics step and
//! publishes a command; the physics (10 ms period), as the later sibling,
//! receives the command same-instant and integrates. A world-level
//! `PoseLogger` samples every second.
//!
//! Usage:
//!   cargo run -p continuo-examples --example traffic
//!   cargo run -p continuo-examples --example traffic -- --record run.jsonl
//!   cargo run -p continuo-examples --example traffic -- --replay run.jsonl
//!
//! `--record` writes the event log (messages + tick fingerprints) to a file;
//! `--replay` re-runs the same seeded world and verifies the new run against
//! the recorded log, exiting non-zero at the first divergence.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use continuo_actors::{PathFollowController, PoseLogger, UnicyclePhysics, Waypoints};
use continuo_conductor::{Conductor, ConductorConfig, EventLog, Recorder};
use continuo_core::{Pose, Quat, SimDuration, SimTime};
use continuo_transport::{InProcTransport, MonitorTransport};

const WORLD_SEED: u64 = 42;

enum Mode {
    Run,
    Record(String),
    Replay(String),
}

fn parse_args() -> Result<Mode, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [] => Ok(Mode::Run),
        [flag, file] if flag == "--record" => Ok(Mode::Record(file.clone())),
        [flag, file] if flag == "--replay" => Ok(Mode::Replay(file.clone())),
        _ => Err("usage: traffic [--record <file> | --replay <file>]".to_string()),
    }
}

/// Builds and free-runs the demo world for 30 sim-seconds, recording the
/// full event log. Returns the log and the published-message count.
fn run_world() -> Result<(EventLog, u64), Box<dyn std::error::Error>> {
    let recorder = Recorder::new("demo", WORLD_SEED);

    // A transport monitor observes every published message out-of-band (at
    // publish time, independent of subscriptions and visibility): here it
    // counts and feeds the recorder.
    let published = Arc::new(AtomicU64::new(0));
    let transport = MonitorTransport::new(InProcTransport::new(), {
        let published = published.clone();
        let mut record_message = recorder.message_callback();
        // `move` applies to the closure's captures (the counter Arc and the
        // recording callback), not to the message: the callback still receives each
        // message by reference (`&Message`).
        move |m| {
            published.fetch_add(1, Ordering::Relaxed);
            record_message(m);
        }
    });

    let path = Arc::new(Waypoints::ellipse((0.0, 0.0), 40.0, 25.0, 72));

    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "demo".into(),
            seed: WORLD_SEED,
            real_time_pacing: false,
        },
        transport,
    )?;
    conductor.set_tick_callback(recorder.tick_callback());

    for (i, car) in ["car1", "car2", "car3"].into_iter().enumerate() {
        let s0 = path.total_length() * i as f64 / 3.0;
        let initial_pose = Pose {
            position: path.point_at(s0),
            orientation: Quat::from_yaw(path.heading_at(s0)),
        };
        // Declared order matters: controller before physics.
        conductor.add_component(
            car,
            Box::new(PathFollowController::new(
                car,
                path.clone(),
                SimDuration::from_millis(100),
                8.0, // m/s
                6.0, // lookahead, m
                1.5, // heading gain, 1/s
                1.2, // max yaw rate, rad/s
                initial_pose,
            )),
        )?;
        conductor.add_component(
            car,
            Box::new(UnicyclePhysics::new(
                car,
                SimDuration::from_millis(10),
                initial_pose,
            )),
        )?;
    }

    // Offset the logger 1 ns past each second boundary: the smallest offset
    // that clears same-instant deferral, so on-boundary poses are visible —
    // and nothing can be scheduled between a boundary and its sample.
    conductor.add_component(
        "",
        Box::new(PoseLogger::new(
            SimDuration::from_secs(1),
            SimDuration::from_nanos(1),
        )),
    )?;

    let end = SimTime::ZERO + SimDuration::from_secs(30);
    // Wall time is fine to read here: the example's main is outside the
    // simulation, where wall clocks are forbidden.
    let started = std::time::Instant::now();
    conductor.run_until(end)?;
    let elapsed = started.elapsed();

    println!(
        "done: world '{}' reached sim time {} in {} ticks (free-run), {} messages published",
        conductor.world(),
        conductor.sim_time(),
        conductor.tick(),
        published.load(Ordering::Relaxed)
    );
    println!(
        "actual time: {:.3} s ({:.0}x real-time), world hash {:016x}",
        elapsed.as_secs_f64(),
        conductor.sim_time().as_secs_f64() / elapsed.as_secs_f64(),
        conductor.world_hash()
    );

    // Return the recorded log and the number of messages published.
    Ok((recorder.finish(), published.load(Ordering::Relaxed)))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let mode = parse_args().map_err(|usage| -> Box<dyn std::error::Error> { usage.into() })?;

    match mode {
        Mode::Run => {
            run_world()?;
        }
        Mode::Record(file) => {
            let (log, _) = run_world()?;
            log.write_file(&file)?;
            println!("recorded {} events to {file}", log.events.len());
        }
        Mode::Replay(file) => {
            let expected = EventLog::read_file(&file)?;
            let (actual, _) = run_world()?;
            match expected.first_divergence(&actual) {
                None => println!(
                    "replay verified: {} events match, final world hash {:016x}",
                    expected.events.len(),
                    expected
                        .final_world_hash()
                        .expect("recorded log contains ticks")
                ),
                Some(divergence) => {
                    eprintln!("replay FAILED: {divergence}");
                    std::process::exit(1);
                }
            }
        }
    }

    // Return success for the completed mode.
    Ok(())
}
