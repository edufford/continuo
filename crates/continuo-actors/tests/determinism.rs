//! Determinism tests over the full traffic world: two identical runs must
//! produce identical event logs, every message byte and every tick fingerprint
//! (the milestone 2 hash stream).

use std::sync::Arc;

use continuo_actors::{CarState, DriveLimits, PathFollowController, UnicyclePhysics, Waypoints};
use continuo_conductor::record::LogEvent;
use continuo_conductor::{Conductor, ConductorConfig, EventLog, Pacing, Recorder};
use continuo_core::{HashFnv1a64, Pose, Quat, SimDuration, SimTime};
use continuo_transport::{InProcTransport, MonitorTransport};

/// What every car here holds, nobody commanding an acceleration.
const CAR_SPEED: f64 = 8.0;

/// What a full command is worth on those cars. The controller is handed
/// the turn rate out of it for the reason `traffic_world` gives: a
/// normalized command means whatever the plant says it means.
const CAR_LIMITS: DriveLimits = DriveLimits::highway_car();

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
                    CAR_LIMITS.yaw_rate_max,
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
                    CAR_LIMITS,
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

/// A fingerprint of car1's whole trajectory, independent of how a pose
/// happens to be written down.
///
/// The world hash cannot do this job: it is taken over payload bytes, so
/// reshaping a message moves it whether or not a car moved. This folds
/// the decoded numbers, so it moves only when a car does.
///
/// From the ellipse rather than a straight road: there the steering law
/// works the whole way round, so a difference in the integration shows.
/// On a straight road every yaw rate is exactly zero and two quite
/// different plants would agree.
const CAR1_TRAJECTORY: u64 = 0x1a32_628b_483a_869d;

/// Every pose folded through [`HashFnv1a64`], which is the hash the world
/// fingerprint is built from and is owned by the workspace for this
/// reason: its constants are the same on every platform and toolchain,
/// where `DefaultHasher` is explicitly not stable between Rust releases.
///
/// No length prefixes, because every field here is eight bytes and a run
/// of fixed-width fields can only be read one way.
fn trajectory_fingerprint(poses: &[Pose]) -> u64 {
    let mut hash = HashFnv1a64::new();
    for pose in poses {
        for value in [pose.position.x, pose.position.y, pose.orientation.yaw()] {
            hash.write_u64(value.to_bits());
        }
    }

    // Return the fold.
    hash.finish()
}

#[test]
fn a_constant_speed_car_traces_its_pinned_path() {
    // Every step of it, not a sample: nobody commands an acceleration
    // here, so the held zero stands, the speed never leaves what the car
    // was built with, and the geometry alone decides where it ends up.
    let poses = car1_poses(&run_world(5, 42));
    assert_eq!(poses.len(), 501, "expected a steady pose stream");
    assert_eq!(
        format!("{:016x}", trajectory_fingerprint(&poses)),
        format!("{CAR1_TRAJECTORY:016x}"),
        "car1 drove a different path"
    );
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
