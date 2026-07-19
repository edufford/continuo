//! Milestone 1 demo: three cars circulating an oval loop, free-run.
//!
//! Each car is a composite `carN = [controller, physics]` — the controller
//! (100 ms period) reads the pose from the previous physics step and
//! publishes a command; the physics (10 ms period), as the later sibling,
//! receives the command same-instant and integrates. A world-level
//! `PoseLogger` samples every second.
//!
//! Run with: `cargo run -p continuo-examples --example traffic`

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use continuo_actors::{PathFollowController, PoseLogger, UnicyclePhysics, Waypoints};
use continuo_conductor::{Conductor, ConductorConfig};
use continuo_core::{Pose, Quat, SimDuration, SimTime};
use continuo_transport::{InProcTransport, MonitorTransport};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let path = Arc::new(Waypoints::ellipse((0.0, 0.0), 40.0, 25.0, 72));

    // A transport monitor observes every published message out-of-band (at
    // publish time, independent of subscriptions and visibility). Here it
    // just counts; a recorder would write `m` to the event log instead.
    let published = Arc::new(AtomicU64::new(0));
    let transport = MonitorTransport::new(InProcTransport::new(), {
        let published = published.clone();
        move |_m| {
            published.fetch_add(1, Ordering::Relaxed);
        }
    });

    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "demo".into(),
            real_time_pacing: false,
        },
        transport,
    )?;

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
    conductor.run_until(end)?;

    println!(
        "done: world '{}' reached sim time {} in {} ticks (free-run), {} messages published",
        conductor.world(),
        conductor.sim_time(),
        conductor.tick(),
        published.load(Ordering::Relaxed)
    );
    Ok(())
}
