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
            events.lock().unwrap().push(format!(
                "{}|{}|{}|{}|{}",
                m.time.to_canonical_string(),
                m.key,
                m.publisher,
                m.seq,
                String::from_utf8(m.payload.clone()).unwrap()
            ));
        }
    });

    let path = Arc::new(Waypoints::ellipse((0.0, 0.0), 40.0, 25.0, 72));
    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "demo".into(),
            real_time_pacing: false,
        },
        transport,
    )
    .unwrap();

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
                    SimDuration::from_millis(100),
                    8.0,
                    6.0,
                    1.5,
                    1.2,
                    initial_pose,
                )),
            )
            .unwrap();
        conductor
            .add_component(
                car,
                Box::new(UnicyclePhysics::new(
                    car,
                    SimDuration::from_millis(10),
                    initial_pose,
                )),
            )
            .unwrap();
    }

    conductor
        .run_until(SimTime::ZERO + SimDuration::from_secs(sim_seconds))
        .unwrap();

    let events = events.lock().unwrap();
    events.clone()
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

    let pose_json = |line: &str| line.rsplit('|').next().unwrap().to_string();
    let first: Pose = serde_json::from_str(&pose_json(car1.first().unwrap())).unwrap();
    let last: Pose = serde_json::from_str(&pose_json(car1.last().unwrap())).unwrap();
    let dist = ((last.position.x - first.position.x).powi(2)
        + (last.position.y - first.position.y).powi(2))
    .sqrt();
    // ~8 m/s for 5 s along an oval: it must have gone somewhere.
    assert!(dist > 5.0, "car1 barely moved: {dist} m");
}
