//! Determinism tests over the full traffic world: two identical runs must
//! produce identical event logs, every message byte and every tick fingerprint
//! (the milestone 2 hash stream).

use std::sync::Arc;

use continuo_actors::{
    CarState, PathFollowController, PlantLimits, RadarScan, RadarSensor, UnicyclePhysics, Waypoints,
};
use continuo_conductor::record::LogEvent;
use continuo_conductor::{Conductor, ConductorConfig, EventLog, Pacing, Recorder};
use continuo_core::{HashFnv1a64, Pose, Quat, SimDuration, SimTime};
use continuo_transport::{InProcTransport, MonitorTransport};

/// What every car here holds, nobody commanding an acceleration.
const CAR_SPEED: f64 = 8.0;

/// What a full command is worth on those cars, taken by the controller
/// and the physics alike for the reason `traffic_world` gives: a
/// normalized command means whatever the plant says it means.
const CAR_LIMITS: PlantLimits = PlantLimits::highway_car();

/// How far each car's radar sees, and how far off the path a car can be
/// and still count as sharing it.
///
/// The three cars are spread evenly around a loop of about 205 m, so at
/// this range each sees the car in front and not the one beyond it.
const RADAR_RANGE: f64 = 120.0;
const RADAR_LANE_TOLERANCE: f64 = 1.75;

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

    // Each car is a composite `carN = [radar, controller, physics]`.
    // Registration order is sibling order, which fixes both the
    // execution order at shared instants and the visibility rule's
    // "earlier sibling": the controller's command reaches the physics
    // in the same step, and the physics' pose reaches the radar and the
    // controller at their next step.
    //
    // Nothing reads the scans. The radars are here so that two runs have
    // scans to compare, and so that a loop's wrap around its seam runs
    // under the conductor and not only in a unit test.
    for (i, car) in ["car1", "car2", "car3"].into_iter().enumerate() {
        let s0 = path.total_length() * i as f64 / 3.0;
        let initial_pose = Pose {
            position: path.point_at(s0),
            orientation: Quat::from_yaw(path.heading_at(s0)),
        };
        conductor
            .add_component_at_start(
                car,
                Box::new(RadarSensor::new(
                    car,
                    path.clone(),
                    SimDuration::from_millis(100), // scan period
                    RADAR_RANGE,
                    RADAR_LANE_TOLERANCE,
                )),
            )
            .expect("radar path is unique per car");
        conductor
            .add_component_at_start(
                car,
                Box::new(PathFollowController::new(
                    car,
                    path.clone(),
                    0.0,                           // lateral offset: on the path itself
                    SimDuration::from_millis(100), // control period
                    6.0,                           // lookahead distance, m
                    1.5,                           // heading gain, 1/s
                    CAR_LIMITS.yaw_rate_max,       // command turns up to the car's limit
                    CAR_LIMITS,
                    initial_pose,
                )),
            )
            .expect("controller path is unique per car");
        conductor
            .add_component_at_start(
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

/// A fingerprint of car1's whole trajectory around the ellipse.
///
/// **This is the first check in the project that a curved world is
/// portable.** `DEMO_WORLD_HASH` cannot be: the demo drives a straight
/// road, so every yaw rate in it is exactly zero and every transcendental
/// is evaluated where all implementations agree anyway. Routing this
/// workspace through `libm` moves that hash not at all, which is the
/// proof it was never testing this.
///
/// An ellipse steers the whole way round, so it evaluates `sin`, `cos`
/// and `atan2` at arguments where implementations are free to differ.
/// Before `libm` it fingerprinted three ways across the four CI agents,
/// the two glibc ones agreeing with each other across architectures while
/// the MSVC CRT and Apple's each differed. This value is what all four
/// produce now.
///
/// It answers a second question the world hash also cannot. That hash is
/// taken over payload bytes, so reshaping a message moves it whether or
/// not a car moved; this folds the decoded numbers, so it moves only when
/// a car does.
const CAR1_TRAJECTORY: u64 = 0xd53c_ae9c_9360_d41d;

/// Every pose folded through [`HashFnv1a64`], the hash the world
/// fingerprint is already built from, rather than `DefaultHasher`, which
/// is explicitly not stable between Rust releases.
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
fn a_curved_world_traces_the_same_path_on_every_platform() {
    // Every step of the run, not a sample. Nobody commands an
    // acceleration here, so the held zero stands and the geometry alone
    // decides where the car ends up.
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

/// Every scan car1's radar published, in order.
fn car1_scans(log: &EventLog) -> Vec<RadarScan> {
    // Return them decoded, since the comparison is on what the radar saw.
    log.events
        .iter()
        .filter_map(|e| match e {
            LogEvent::Msg(m) if m.key.contains("/car1/radar") => {
                Some(serde_json::from_str(m.payload.get()).expect("radar payloads deserialize"))
            }
            _ => None,
        })
        .collect()
}

#[test]
fn two_identical_radar_runs_fingerprint_identically() {
    let first = car1_scans(&run_world(5, 42));
    let second = car1_scans(&run_world(5, 42));
    assert_eq!(first, second, "car1 scanned a different road");
    assert!(!first.is_empty(), "expected a scan stream");

    // It also has to have seen something, since two empty streams compare
    // equal too. The car in front is in range and the one beyond is not,
    // so every scan holds exactly one detection. What the numbers in it
    // are is the unit tests' business.
    for scan in &first {
        assert_eq!(
            scan.detections.len(),
            1,
            "expected the car in front: {scan:?}"
        );
    }
}
