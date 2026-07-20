//! Determinism smoke test ahead of milestone 2's state hashes: two identical
//! runs of the traffic world must produce byte-identical message streams.
//!
//! Capture uses a transport monitor, which observes every published message
//! (poses *and* commands) at publish time — the same mechanism the milestone
//! 2 event log will use.

use std::sync::{Arc, Mutex};

use continuo_actors::{PathFollowController, UnicyclePhysics, Waypoints};
use continuo_conductor::{Conductor, ConductorConfig};
use continuo_core::{Message, Pose, Quat, SimDuration, SimTime};
use continuo_transport::{InProcTransport, MonitorTransport};

fn run_world(sim_seconds: i64) -> Vec<String> {
    let events = Arc::new(Mutex::new(Vec::new()));
    let transport = MonitorTransport::new(InProcTransport::new(), {
        let events = events.clone();
        move |m: &Message| {
            events
                .lock()
                .expect("events mutex is never poisoned (no panicking holders)")
                .push(format!(
                    "{}|{}|{}|{}|{}",
                    m.time.to_canonical_string(),
                    m.key,
                    m.publisher,
                    m.seq,
                    String::from_utf8(m.payload.clone())
                        .expect("payloads are serialized JSON, which is always valid UTF-8")
                ));
        }
    });

    // 72 samples = one point per 5 degrees of arc; on the 40 m semi-axis the
    // worst-case chord deviation is ~4 cm, smooth enough for the
    // controller's 6 m lookahead. Any fixed count is equally deterministic —
    // this one just keeps the polyline visually round.
    let path = Arc::new(Waypoints::ellipse((0.0, 0.0), 40.0, 25.0, 72));
    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "demo".into(),
            real_time_pacing: false,
        },
        transport,
    )
    .expect("free-run config is always accepted");

    // Each car is registered as a composite `carN = [controller, physics]`.
    // Registration order is declared sibling order, which fixes both the
    // execution order at shared instants and the visibility rule's "earlier
    // sibling": the controller steps first and its command reaches the
    // physics in the same step; the physics' pose reaches the controller at
    // its next step.
    for (i, car) in ["car1", "car2", "car3"].into_iter().enumerate() {
        let s0 = path.total_length() * i as f64 / 3.0;
        let initial_pose = Pose {
            position: path.point_at(s0),
            orientation: Quat::from_yaw(path.heading_at(s0)),
        };
        conductor
            .add_component(
                car,
                Box::new(PathFollowController::new(
                    car,
                    path.clone(),
                    SimDuration::from_millis(100), // control period
                    8.0,                           // speed, m/s
                    6.0,                           // lookahead distance, m
                    1.5,                           // heading gain, 1/s
                    1.2,                           // max yaw rate, rad/s
                    initial_pose,
                )),
            )
            .expect("controller path is unique per car");
        conductor
            .add_component(
                car,
                Box::new(UnicyclePhysics::new(
                    car,
                    SimDuration::from_millis(10), // physics period
                    initial_pose,
                )),
            )
            .expect("physics path is unique per car");
    }

    conductor
        .run_until(SimTime::from_secs(sim_seconds))
        .expect("demo components always schedule strictly forward");

    // The monitor's sink closure (owned by the conductor's transport) holds
    // the second Arc reference to `events`. Dropping the conductor releases
    // it, so the Vec can be taken out of the Arc/Mutex without cloning.
    drop(conductor);
    Arc::try_unwrap(events)
        .expect("conductor drop released the only other Arc reference")
        .into_inner()
        .expect("events mutex is never poisoned (no panicking holders)")
}

#[test]
fn identical_runs_produce_identical_message_streams() {
    let first = run_world(5);
    let second = run_world(5);
    assert!(!first.is_empty(), "expected message traffic");
    assert_eq!(first, second, "two identical runs diverged");
}

#[test]
fn cars_actually_move_around_the_loop() {
    let events = run_world(5);
    let car1: Vec<&String> = events.iter().filter(|e| e.contains("/car1/pose")).collect();
    assert!(
        car1.len() > 100,
        "expected steady pose stream, got {}",
        car1.len()
    );

    let pose = |line: &str| -> Pose {
        let json = line
            .rsplit('|')
            .next()
            .expect("rsplit yields at least one piece for any line");
        serde_json::from_str(json).expect("pose payloads deserialize as Pose")
    };
    let first = pose(car1.first().expect("stream verified non-empty above"));
    let last = pose(car1.last().expect("stream verified non-empty above"));
    let dist = ((last.position.x - first.position.x).powi(2)
        + (last.position.y - first.position.y).powi(2))
    .sqrt();
    // ~8 m/s for 5 s along an oval: it must have gone somewhere.
    assert!(dist > 5.0, "car1 barely moved: {dist} m");
}
