//! Determinism tests over the full traffic world: two identical runs must
//! produce identical event logs, every message byte and every tick fingerprint
//! (the milestone 2 hash stream).

use std::sync::Arc;

use continuo_actors::{CarState, PathFollowController, UnicyclePhysics, Waypoints};
use continuo_conductor::record::LogEvent;
use continuo_conductor::{Conductor, ConductorConfig, EventLog, Pacing, Recorder};
use continuo_core::{Pose, Quat, SimDuration, SimTime};
use continuo_transport::{InProcTransport, MonitorTransport};

/// What every car here holds, nobody commanding an acceleration.
const CAR_SPEED: f64 = 8.0;

fn run_world(sim_seconds: i64, world_seed: u64) -> EventLog {
    let config = ConductorConfig {
        world_name: "demo".into(),
        world_seed,
        pacing: Pacing::FreeRun,
    };
    let recorder = Recorder::new(&config);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());

    // 72 samples = one point per 5 degrees of arc; on the 40 m semi-axis the
    // worst-case chord deviation is ~4 cm, smooth enough for the
    // controller's 6 m lookahead. Any fixed count is equally deterministic,
    // this one just keeps the polyline visually round.
    let path = Arc::new(Waypoints::ellipse((0.0, 0.0), 40.0, 25.0, 72));
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(recorder.tick_callback());

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
                    0.0,                           // lateral offset: on the path itself
                    SimDuration::from_millis(100), // control period
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
                    CarState::new(initial_pose, CAR_SPEED),
                )),
            )
            .expect("physics path is unique per car");
    }

    conductor
        .run_until(SimTime::from_secs(sim_seconds))
        .expect("demo components always schedule strictly forward");

    // Return the recorded run for comparison.
    recorder.finish()
}

#[test]
fn identical_runs_produce_identical_event_logs() {
    let first = run_world(5, 42);
    let second = run_world(5, 42);
    assert!(!first.events.is_empty(), "expected recorded traffic");
    // Full comparison: every message byte and every tick fingerprint.
    assert_eq!(first.first_divergence(&second), None);
    assert!(first.final_world_hash().is_some());
}

/// Every pose car1 published, in the order it published them.
fn car1_poses(log: &EventLog) -> Vec<Pose> {
    // Return the stream, read as poses because that is what everything
    // else reads off this key. The plant publishes its speed there too,
    // and a `Pose` decode ignores it.
    log.events
        .iter()
        .filter_map(|e| match e {
            LogEvent::Msg(m) if m.key.contains("/car1/pose") => Some(
                serde_json::from_str(m.payload.get()).expect("pose payloads deserialize as Pose"),
            ),
            _ => None,
        })
        .collect()
}

/// Where car1 is at five sampled steps of the run above.
///
/// A pinned trajectory, so a change meaning to leave the world alone has
/// something to prove it with. The scenario or the model moving it is a
/// deliberate act; anything else moving it is the bug this catches.
///
/// The ellipse rather than a straight road is the point: here the
/// steering law works the whole way round, so a difference in the
/// integration shows. On a straight road every yaw rate is exactly zero
/// and two quite different plants would agree.
const BASELINE_CAR1_POSES: [(usize, f64, f64, f64); 5] = [
    (0, 40.0, 0.0, 1.640_540_530_479_719_4),
    (
        125,
        37.507_851_336_973_054,
        9.601_746_584_654_9,
        2.066_784_740_651_912,
    ),
    (
        250,
        30.781_464_445_016_912,
        16.875_992_092_276_096,
        2.533_986_377_490_154_3,
    ),
    (
        375,
        21.870_721_762_925_36,
        21.355_526_274_873_89,
        2.788_788_189_032_116,
    ),
    (
        499,
        12.320_909_025_749_636,
        24.001_467_737_744_12,
        2.948_859_999_559_971,
    ),
];

#[test]
fn a_constant_speed_car_traces_its_pinned_path() {
    // To the bit. Nobody commands an acceleration here, so the held zero
    // stands, the speed never leaves what the car was built with, and the
    // geometry alone decides where it ends up.
    let poses = car1_poses(&run_world(5, 42));
    for (index, x, y, yaw) in BASELINE_CAR1_POSES {
        let pose = poses[index];
        assert_eq!(pose.position.x, x, "x at step {index}");
        assert_eq!(pose.position.y, y, "y at step {index}");
        assert_eq!(pose.orientation.yaw(), yaw, "yaw at step {index}");
    }
}

#[test]
fn cars_actually_move_around_the_loop() {
    let car1_poses = car1_poses(&run_world(5, 42));
    assert!(
        car1_poses.len() > 100,
        "expected steady pose stream, got {}",
        car1_poses.len()
    );

    let first = car1_poses.first().expect("stream verified non-empty above");
    let last = car1_poses.last().expect("stream verified non-empty above");
    let dist = ((last.position.x - first.position.x).powi(2)
        + (last.position.y - first.position.y).powi(2))
    .sqrt();
    // At CAR_SPEED for 5 s along an oval: it must have gone somewhere.
    assert!(dist > 5.0, "car1 barely moved: {dist} m");
}
