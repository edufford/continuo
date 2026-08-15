//! The packaged FMU against the laws it was built from.
//!
//! The `.fmu` links `continuo-actors` statically, so the shared library it
//! ships carries its own compiled copy of the control laws rather than
//! calling back into this process. These tests drive that copy across the
//! FMI boundary and compare it, to the bit, with what the laws answer
//! here. Nothing is stored to compare against: both sides are computed
//! afresh, and the claim is that two builds of one implementation agree,
//! which is a claim only running both can settle.
//!
//! Two things can part them, and both are worth catching. A packaged FMU
//! older than the laws is the likelier by far, since editing a law is
//! ordinary work and packaging one is a separate command. The other is the
//! boundary itself losing something: a pose that arrives as seven scalars,
//! a scan that arrives as two arrays, and a road that arrives as the
//! numbers it was built from all have somewhere to go wrong that no
//! compiler checks.
//!
//! Driven through [`Component::step`] rather than through a conductor, as
//! `continuo-fmi`'s own suite is, since what is under test is one
//! component's answers rather than how a world schedules it.

// Everything here reads a file `cargo xtask package-fmus` writes, so this
// file compiles away to nothing unless the `packaged-fmu` feature is enabled.
// What that costs is the run where it matters most: a law edited without
// packaging again goes unnoticed by `cargo test --workspace`, and it takes
// both of those commands to see it.
#![cfg(feature = "packaged-fmu")]

use std::path::PathBuf;

use continuo_actors::control_laws::{
    FREE_ROAD, IdmParams, PurePursuitParams, idm_accel, nearest_detection, pure_pursuit_yaw_rate,
};
use continuo_actors::{MAX_DETECTIONS, Waypoints};
use continuo_core::{
    Component, ComponentPath, CoreError, Detection, KeyExpr, Message, Pose, Quat, RandomSplitMix64,
    SimDuration, SimTime, StepCtx,
};
use continuo_fmi::{FmuComponent, FmuMapping, InputBinding, OutputBinding};
use continuo_fmu_controller_idm::{MAX_WAYPOINTS, packaged_fmu_path};
use serde_json::{Value, json};

const WORLD: &str = "continuo/test";

/// How often the controller steps. The laws are a map from inputs to
/// commands rather than anything integrated, so no answer here depends on
/// this, but a step still has to happen somewhere on a clock.
const PERIOD_MS: i64 = 100;

/// What a disagreement most likely means.
///
/// Named in every comparison's failure text, because a stale package
/// presents as the IDM math disagreeing with itself, which says nothing
/// about where to look.
const MAYBE_STALE: &str =
    "the laws may have changed since the FMU was packaged: run `cargo xtask package-fmus`";

/// The tuning the demo's traffic will run, near enough that these tests
/// exercise the numbers the demo does.
const PURSUIT: PurePursuitParams = PurePursuitParams::highway_car(3.5);
const IDM: IdmParams = IdmParams::highway_car(30.0);

fn key(name: &str) -> KeyExpr {
    KeyExpr::new(format!("{WORLD}/{name}")).unwrap()
}

/// Where `cargo xtask package-fmus` left the FMU, or why it is not there.
///
/// Failing rather than skipping, since a suite that quietly tests nothing
/// is worse than one that says what to run.
fn packaged() -> PathBuf {
    packaged_fmu_path().unwrap_or_else(|missing| panic!("{missing}"))
}

/// One message on `key` carrying `payload`, as a publisher would send it.
fn message(key_name: &str, payload: Value) -> Message {
    Message {
        key: key(key_name),
        publisher: ComponentPath::parse("car").unwrap(),
        seq: 0,
        sim_time: SimTime::ZERO,
        payload: serde_json::to_vec(&payload).unwrap(),
    }
}

/// What a car knows about itself and what is ahead of it.
#[derive(Clone)]
struct Situation {
    pose: Pose,
    speed: f64,
    scan: Vec<Detection>,
}

impl Situation {
    /// One line naming what the car was in, for a failure to be read by.
    ///
    /// The scan is reduced to the detection the law would have followed,
    /// because it is mostly padding by construction and sixty-four free
    /// roads printed out bury the numbers that decided anything.
    fn summary(&self) -> String {
        let position = self.pose.position;
        let lead = nearest_detection(&self.scan);

        // Return the line, in the quantities the laws are written in
        // rather than the seven scalars a pose crosses as.
        format!(
            "at ({}, {}) facing {} rad, {} m/s, nearest {} m closing at {} m/s",
            position.x,
            position.y,
            self.pose.orientation.yaw(),
            self.speed,
            lead.range,
            -lead.range_rate,
        )
    }
}

/// A car at `s` along the road, `lateral` meters left of the centerline,
/// pointing `yaw_error` radians off the road's own heading.
fn car_at(road: &Waypoints, s: f64, lateral: f64, yaw_error: f64) -> Pose {
    Pose {
        position: road.point_at_offset(s, lateral),
        orientation: Quat::from_yaw(road.heading_at(s) + yaw_error),
    }
}

/// The wiring an FMU car will be given: a pose and a radar scan in, the
/// two commands out on one key, and everything the laws are tuned by set
/// once as parameters.
///
/// The pose inputs name no pointers, since FMI's structured names and JSON
/// Pointer describe the same shape and `position.x` spells `/position/x`
/// on its own. The radar's do, because nothing derives
/// `/detections/0/range` from a variable named `range`.
fn controller_mapping(road: &Waypoints, pursuit: PurePursuitParams, idm: IdmParams) -> FmuMapping {
    let mut road_x = vec![0.0; MAX_WAYPOINTS];
    let mut road_y = vec![0.0; MAX_WAYPOINTS];
    for (slot, &(x, y)) in road.points().iter().enumerate() {
        road_x[slot] = x;
        road_y[slot] = y;
    }

    let pose = key("pose");
    let radar = key("radar");
    let cmd = key("cmd");

    // Return the sheet, which is data rather than code: adding this FMU to
    // a world writes one of these instead of compiling anything.
    FmuMapping {
        period: SimDuration::from_millis(PERIOD_MS),
        inputs: vec![
            InputBinding::new("position.x", pose.clone()),
            InputBinding::new("position.y", pose.clone()),
            InputBinding::new("orientation.w", pose.clone()),
            InputBinding::new("orientation.x", pose.clone()),
            InputBinding::new("orientation.y", pose.clone()),
            InputBinding::new("orientation.z", pose.clone()),
            InputBinding::new("speed", pose),
            InputBinding::new("range", radar.clone())
                .with_pointer("/detections/*/range")
                .when_missing(json!(FREE_ROAD.range)),
            InputBinding::new("range_rate", radar)
                .with_pointer("/detections/*/range_rate")
                .when_missing(json!(FREE_ROAD.range_rate)),
        ],
        outputs: vec![
            OutputBinding::new("accel_cmd", cmd.clone()),
            OutputBinding::new("yaw_rate_cmd", cmd),
        ],
        initial_values: vec![
            ("road_x".to_string(), json!(road_x)),
            ("road_y".to_string(), json!(road_y)),
            ("road_point_count".to_string(), json!(road.points().len())),
            ("road_closed".to_string(), json!(road.is_closed())),
            ("lateral_tgt".to_string(), json!(pursuit.lateral_tgt)),
            ("lookahead".to_string(), json!(pursuit.lookahead)),
            ("gain_yaw_rate".to_string(), json!(pursuit.gain_yaw_rate)),
            ("max_yaw_rate".to_string(), json!(pursuit.max_yaw_rate)),
            ("v0_speed_tgt".to_string(), json!(idm.v0_speed_tgt)),
            ("t_headway".to_string(), json!(idm.t_headway)),
            ("s0_gap_min".to_string(), json!(idm.s0_gap_min)),
            ("a_accel_max".to_string(), json!(idm.a_accel_max)),
            ("b_decel_comfort".to_string(), json!(idm.b_decel_comfort)),
        ],
    }
}

/// Replaces one parameter's value, for a test sending something the
/// builder above never would.
fn set_parameter(mapping: &mut FmuMapping, name: &str, value: Value) {
    let slot = mapping
        .initial_values
        .iter_mut()
        .find(|(parameter, _)| parameter == name)
        .unwrap_or_else(|| panic!("{name} is not a parameter this mapping sets"));
    slot.1 = value;
}

/// Steps the FMU once in `situation` and returns what it commanded.
///
/// Both outputs are bound to one key, so one message comes back carrying
/// both, which is the shape [`OutputBinding`] merges them into.
fn fmu_commands(fmu: &mut FmuComponent, tick: i64, situation: &Situation) -> (f64, f64) {
    let now = SimTime::from_millis(tick * PERIOD_MS);
    let inbox = vec![
        message(
            "pose",
            json!({
                "position": situation.pose.position,
                "orientation": situation.pose.orientation,
                "speed": situation.speed,
            }),
        ),
        message("radar", json!({ "detections": situation.scan })),
    ];

    let mut ctx = StepCtx::new(now, None, WORLD, 0, inbox);
    fmu.step(&mut ctx).expect("the FMU steps");
    let outbox = ctx.take_outbox();
    assert_eq!(outbox.len(), 1, "both commands share one key");
    let payload: Value = serde_json::from_slice(&outbox[0].1).expect("the payload decodes");

    // Return the pair, read back through JSON as a consumer would. The
    // shortest decimal that round trips is what serde writes, so a number
    // that survives this is the same number to the bit.
    (
        payload["accel_cmd"].as_f64().expect("an acceleration"),
        payload["yaw_rate_cmd"].as_f64().expect("a yaw rate"),
    )
}

/// What the laws answer in `situation`, called here in this process.
fn native_commands(
    road: &Waypoints,
    pursuit: PurePursuitParams,
    idm: IdmParams,
    situation: &Situation,
) -> (f64, f64) {
    let lead = nearest_detection(&situation.scan);

    // Return the same pair the FMU is asked for. A range closes as it
    // falls and an approach rate rises as it closes, so each is the
    // other's negative, and getting that backwards inside the FMU is one
    // of the things these tests are here to catch.
    (
        idm_accel(situation.speed, lead.range, -lead.range_rate, idm),
        pure_pursuit_yaw_rate(road, situation.pose, pursuit),
    )
}

/// Runs every situation through the packaged FMU and through the laws,
/// asserting the two agree to the bit, and returns what was commanded so a
/// caller can check the comparison had something to compare.
fn commands_over(
    road: &Waypoints,
    pursuit: PurePursuitParams,
    idm: IdmParams,
    situations: &[Situation],
) -> Vec<(f64, f64)> {
    let mapping = controller_mapping(road, pursuit, idm);
    let mut fmu = FmuComponent::new("controller", packaged(), mapping).expect("the FMU builds");

    // Return each situation's commands, having agreed at every one. The
    // instance is reused across them because the laws are a map from
    // inputs to commands: nothing carries from one step to the next but
    // the road, which is what makes a sweep like this meaningful at all.
    situations
        .iter()
        .enumerate()
        .map(|(tick, situation)| {
            let from_fmu = fmu_commands(&mut fmu, tick as i64, situation);
            let from_laws = native_commands(road, pursuit, idm, situation);
            assert_eq!(
                from_fmu.0.to_bits(),
                from_laws.0.to_bits(),
                "accel_cmd: the FMU says {} where the laws say {}, {}\n{MAYBE_STALE}",
                from_fmu.0,
                from_laws.0,
                situation.summary(),
            );
            assert_eq!(
                from_fmu.1.to_bits(),
                from_laws.1.to_bits(),
                "yaw_rate_cmd: the FMU says {} where the laws say {}, {}\n{MAYBE_STALE}",
                from_fmu.1,
                from_laws.1,
                situation.summary(),
            );
            from_laws
        })
        .collect()
}

/// Asserts a sweep asked for real commands, both ways in each.
///
/// A comparison of two zeros passes whatever is wrong on either side, and
/// a road every car sits exactly on would give nothing but zeros from the
/// steering law. So each sweep says what it managed to provoke.
///
/// A NaN satisfies neither comparison, so a command that diverged cannot
/// stand as evidence that anything was commanded.
fn assert_both_laws_had_something_to_say(commands: &[(f64, f64)]) {
    assert!(
        commands.iter().any(|&(accel, _)| accel > 0.0),
        "nothing accelerated"
    );
    assert!(
        commands.iter().any(|&(accel, _)| accel < 0.0),
        "nothing braked"
    );
    assert!(
        commands.iter().any(|&(_, yaw_rate)| yaw_rate > 0.0),
        "nothing turned left"
    );
    assert!(
        commands.iter().any(|&(_, yaw_rate)| yaw_rate < 0.0),
        "nothing turned right"
    );
}

/// What a car meets on `road`: the cases somebody chose, then a batch
/// nobody did.
///
/// Both halves are wanted. A chosen case is aimed at something, and says
/// what in a comment beside it, so a failure there names its own subject.
/// A random one is aimed at nothing, which is the point: the corners worth
/// finding are the ones nobody thought to write down.
fn situations_on(road: &Waypoints) -> Vec<Situation> {
    let mut situations = chosen_situations(road);
    situations.extend(random_situations(road));

    // Return both, which every sweep runs together.
    situations
}

/// Every situation worth putting a car in deliberately: along the road,
/// either side of its lane, pointing off it, at four speeds, seeing seven
/// different things ahead.
///
/// One cross product rather than a sweep per law, because a controller
/// answers both at once and an interaction between them is exactly what a
/// hand-picked list would step around.
fn chosen_situations(road: &Waypoints) -> Vec<Situation> {
    // Fractions of the road's length, so the same list means the same
    // places on a road of any size. The last is near enough to the end
    // that a lookahead runs past it, which is where a loop wraps and a
    // road stops.
    const ALONG: [f64; 4] = [0.0, 0.25, 0.6, 0.98];
    // Meters left of the centerline. The lane is 3.5 to the left, so these
    // put the car well right of it, on it, and past it.
    const LATERAL: [f64; 3] = [-2.0, 3.5, 6.0];
    // Radians off the road's own heading, which is what the steering law
    // turns to correct.
    const YAW_ERROR: [f64; 3] = [-0.35, 0.0, 0.5];
    const SPEEDS: [f64; 4] = [0.0, 8.0, 20.0, 33.0];

    let scans = [
        // An open road, where every slot holds the free road and the
        // padding is the whole of what crosses.
        Vec::new(),
        // One lead, in a slot nowhere near the front, so picking the
        // nearest out of the array is part of what is being compared.
        scan_of(&[(37, 40.0, -4.0)]),
        // Close and closing hard, which is the braking end of the law.
        scan_of(&[(0, 6.0, -12.0)]),
        // Pulling away, which is the sign the wanted gap has to be floored
        // at zero for.
        scan_of(&[(63, 25.0, 9.0)]),
        // Three at once, the nearest neither first nor last, so a reader
        // that took the first real slot or the last would answer from the
        // wrong car. A single detection cannot tell those apart from
        // picking correctly, since there is only one to pick.
        scan_of(&[(2, 55.0, -1.0), (19, 18.0, -6.0), (44, 31.0, 2.0)]),
        // Two at the same range, closing at different rates, so which one
        // wins is observable in the command rather than a detail. Ties go
        // to the earlier slot, and either side deciding otherwise is a
        // disagreement this catches.
        scan_of(&[(5, 22.0, -3.0), (40, 22.0, 7.0)]),
        // Every slot real, so nothing is padding and the whole array is
        // set from the message rather than from the mapping's default.
        full_scan(),
    ];

    let mut situations = Vec::new();
    for fraction in ALONG {
        for lateral in LATERAL {
            for yaw_error in YAW_ERROR {
                let pose = car_at(road, road.total_length() * fraction, lateral, yaw_error);
                for speed in SPEEDS {
                    for scan in &scans {
                        situations.push(Situation {
                            pose,
                            speed,
                            scan: scan.clone(),
                        });
                    }
                }
            }
        }
    }

    // Return the lot: 36 poses, at each of 4 speeds, seeing each of 7
    // things ahead.
    situations
}

/// Situations picked at random, to reach what the grid above steps around.
///
/// From a fixed seed, because a failure nobody can reproduce is a failure
/// nobody can fix. The same situations come out on every agent and every
/// run, so a disagreement found here can be walked back into rather than
/// chased, and the workspace's own generator is what picks them, since it
/// is integer arithmetic and answers alike on all four platforms.
///
/// The ranges are wider than anything a road produces. A car may sit past
/// either end of it, twelve meters off to the side, pointing any direction
/// at all, and a gap may be smaller than a car, which is where the
/// following law's floor on the gap it divides by starts to matter. None
/// of that is a world this project runs; all of it is a value a host in
/// another tool can set.
fn random_situations(road: &Waypoints) -> Vec<Situation> {
    // Enough to reach corners, few enough that the suite stays under a
    // second, which is what keeps it something to run while editing.
    const SITUATIONS: usize = 250;
    // The most detections one situation holds. Past a handful, another
    // detection only adds a slot for the nearest not to be in, and the
    // chosen cases already fill every slot in one of theirs.
    const MOST_DETECTIONS: u64 = 5;
    // Arbitrary. What matters is that it never changes, not what it is.
    const SEED: u64 = 0x1D3A_7B91_C0DE_4F62;

    let mut random = RandomSplitMix64::new(SEED);
    let length = road.total_length();

    // Return one situation per draw, each independent of the last.
    (0..SITUATIONS)
        .map(|_| {
            let pose = car_at(
                road,
                random.range_f64(-0.2 * length, 1.2 * length),
                random.range_f64(-12.0, 12.0),
                random.range_f64(-std::f64::consts::PI, std::f64::consts::PI),
            );
            let detections = random.next_u64() % (MOST_DETECTIONS + 1);
            let scan: Vec<(usize, f64, f64)> = (0..detections)
                .map(|_| {
                    (
                        (random.next_u64() % MAX_DETECTIONS as u64) as usize,
                        random.range_f64(-2.0, 300.0),
                        random.range_f64(-25.0, 25.0),
                    )
                })
                .collect();

            Situation {
                pose,
                speed: random.range_f64(0.0, 45.0),
                scan: if scan.is_empty() {
                    Vec::new()
                } else {
                    scan_of(&scan)
                },
            }
        })
        .collect()
}

/// A scan holding a detection in each `(slot, range, range_rate)` given,
/// free road in the slots between them.
///
/// The free road is padding rather than an absence, which is how a
/// fixed-length array carries a scan that is mostly empty: a free-road
/// slot loses to anything real, so nothing has to say how many are worth
/// reading.
fn scan_of(detections: &[(usize, f64, f64)]) -> Vec<Detection> {
    let last = detections
        .iter()
        .map(|&(slot, _, _)| slot)
        .max()
        .expect("a scan of nothing is written as an empty vector");
    let mut scan = vec![FREE_ROAD; last + 1];
    for &(slot, range, range_rate) in detections {
        scan[slot] = Detection { range, range_rate };
    }

    // Return the scan, which stops after the last slot named, so the
    // mapping's own default fills the rest of the FMU's arrays.
    scan
}

/// A scan filling every slot the FMU declares, so nothing about it is
/// padding.
///
/// Ranges fall toward slot 29 and rise again, putting the nearest where
/// neither end of the array is, and the rates differ with them so which
/// one wins reaches the command.
fn full_scan() -> Vec<Detection> {
    // Return one detection per slot, arranged around a nearest that has to
    // be searched for.
    (0..MAX_DETECTIONS)
        .map(|slot| {
            let from_nearest = (slot as f64 - 29.0).abs();
            Detection {
                range: 12.0 + from_nearest * 3.0,
                range_rate: -6.0 + from_nearest * 0.25,
            }
        })
        .collect()
}

#[test]
fn the_packaged_fmu_commands_what_the_laws_command() {
    // The demo's road: straight, and long enough that a car can sit
    // anywhere along it.
    let road = Waypoints::build_straight((0.0, 0.0), (1200.0, 0.0));
    let commands = commands_over(&road, PURSUIT, IDM, &situations_on(&road));
    assert_both_laws_had_something_to_say(&commands);
}

#[test]
fn a_curved_road_commands_what_the_laws_command() {
    // What the demo's straight road cannot show. On it the heading is the
    // same everywhere and a projection lands on the one segment there is,
    // so a road that crossed as its first two points alone would pass. A
    // polyline bending both ways puts every segment of the arc-length
    // table in play.
    let road = Waypoints::build_open(vec![
        (0.0, 0.0),
        (60.0, 0.0),
        (110.0, 40.0),
        (170.0, 40.0),
        (210.0, -10.0),
        (280.0, -10.0),
    ]);
    let commands = commands_over(&road, PURSUIT, IDM, &situations_on(&road));
    assert_both_laws_had_something_to_say(&commands);
}

#[test]
fn a_closed_road_wraps_inside_the_fmu_as_it_does_natively() {
    // `road_closed` is the field whose absence would have been silent: the
    // demo's road is open, so nothing there would ever notice a loop
    // arriving as a polyline that stops. A car near the end of a lap is
    // where the two part, since its aim point is past the end and a loop
    // puts that back at the start.
    let road = Waypoints::ellipse((0.0, 0.0), 120.0, 80.0, 24);
    assert!(road.is_closed(), "the fixture is the case under test");

    let commands = commands_over(&road, PURSUIT, IDM, &situations_on(&road));
    assert_both_laws_had_something_to_say(&commands);
}

#[test]
fn the_padding_past_the_count_is_never_read() {
    // The arrays are as long as the FMU declares whatever the road is, so
    // a host fills the tail with something. Here that something is
    // somewhere no road would go: a reader trusting the array over the
    // count comes back with a road hundreds of meters long, and every
    // command differs.
    let road = Waypoints::build_open(vec![(0.0, 0.0), (80.0, 0.0), (140.0, 45.0)]);
    let situations = situations_on(&road);

    let mut mapping = controller_mapping(&road, PURSUIT, IDM);
    let junk: Vec<f64> = (0..MAX_WAYPOINTS)
        .map(|slot| 500.0 + slot as f64 * 17.0)
        .collect();
    let mut road_x = junk.clone();
    let mut road_y: Vec<f64> = junk.iter().map(|value| -value).collect();
    for (slot, &(x, y)) in road.points().iter().enumerate() {
        road_x[slot] = x;
        road_y[slot] = y;
    }
    set_parameter(&mut mapping, "road_x", json!(road_x));
    set_parameter(&mut mapping, "road_y", json!(road_y));

    let mut fmu = FmuComponent::new("controller", packaged(), mapping).expect("the FMU builds");
    for (tick, situation) in situations.iter().enumerate() {
        let from_fmu = fmu_commands(&mut fmu, tick as i64, situation);
        let from_laws = native_commands(&road, PURSUIT, IDM, situation);
        assert_eq!(
            (from_fmu.0.to_bits(), from_fmu.1.to_bits()),
            (from_laws.0.to_bits(), from_laws.1.to_bits()),
            "the padding reached the road, {}\n{MAYBE_STALE}",
            situation.summary(),
        );
    }
}

/// Builds a controller whose parameters carry `bad`, and returns why its
/// first step failed.
///
/// Every refusal lands in the same place. A parameter is set during
/// initialization, and both the road and the checks on the tuning sets run
/// when `fmi3ExitInitializationMode` calls `configurate`, which is the
/// first moment the parameters mean what the host meant.
fn refusal(bad: &[(&str, Value)]) -> String {
    let road = Waypoints::build_straight((0.0, 0.0), (400.0, 0.0));
    let mut mapping = controller_mapping(&road, PURSUIT, IDM);
    for (name, value) in bad {
        set_parameter(&mut mapping, name, value.clone());
    }

    let mut fmu = FmuComponent::new("controller", packaged(), mapping).expect("the FMU builds");
    let mut ctx = StepCtx::new(SimTime::ZERO, None, WORLD, 0, Vec::new());
    let refused = match fmu.step(&mut ctx) {
        Err(CoreError::ComponentFailure { reason }) => reason,
        other => panic!("{bad:?} should halt the world, and gave {other:?}"),
    };
    assert!(
        ctx.take_outbox().is_empty(),
        "a step that failed published nothing"
    );

    // Return the reason, which names the instance and the call that
    // refused. What was wrong with the value travels separately, on the
    // log channel FMI gives a model for saying so, since a status code is
    // the whole of what a call itself can return.
    refused
}

#[test]
fn a_road_the_fmu_cannot_build_halts_the_world_rather_than_panicking() {
    // `Waypoints` asserts its own minimum, and a panic unwinding out
    // through the C interface would take the host process with it. So the
    // count is checked first and the same information travels as a
    // refusal, which the standard can carry.
    for count in [0, 1, MAX_WAYPOINTS + 1] {
        let reason = refusal(&[("road_point_count", json!(count))]);
        assert!(
            reason.contains("exit_initialization_mode"),
            "{count} points: {reason}"
        );
    }

    // Two points close into nothing, so a loop needs three, and a road set
    // closed without enough points for one is refused the same way.
    let reason = refusal(&[("road_closed", json!(true)), ("road_point_count", json!(2))]);
    assert!(reason.contains("exit_initialization_mode"), "{reason}");

    // What a host gets by setting the count and forgetting the points: the
    // arrays start at zero, so every point of the road is the origin.
    // Nothing in `Waypoints` objects, and a car given it holds still.
    let reason = refusal(&[
        ("road_x", json!(vec![0.0; MAX_WAYPOINTS])),
        ("road_y", json!(vec![0.0; MAX_WAYPOINTS])),
        ("road_point_count", json!(4)),
    ]);
    assert!(reason.contains("exit_initialization_mode"), "{reason}");
}

#[test]
fn a_parameter_the_laws_divide_by_halts_the_world_unless_it_is_positive() {
    // The model description states no bounds, because `fmi-export` 0.3.0
    // has no key for one, so a host in another tool can send any of these
    // and nothing warns it off. Without the check they reach the laws as a
    // NaN, which then spreads into every command the car goes on to make.
    for name in [
        "v0_speed_tgt",
        "a_accel_max",
        "b_decel_comfort",
        "lookahead",
    ] {
        for given in [json!(0.0), json!(-1.0)] {
            let reason = refusal(&[(name, given.clone())]);
            assert!(
                reason.contains("exit_initialization_mode"),
                "{name} = {given}: {reason}"
            );
        }
    }
}

#[test]
fn a_tuning_parameter_changed_between_steps_changes_the_command() {
    // What declaring the law parameters tunable buys, checked where a host
    // actually stands: they are set again between steps rather than at
    // initialization, and the command answers to the new value. Keeping a
    // copy taken during initialization would pass every other test here
    // and quietly ignore the change.
    const SPEED: f64 = 20.0;

    let road = Waypoints::build_straight((0.0, 0.0), (600.0, 0.0));
    let mut mapping = controller_mapping(&road, PURSUIT, IDM);
    mapping
        .inputs
        .push(InputBinding::new("v0_speed_tgt", key("tuning")));
    let mut fmu = FmuComponent::new("controller", packaged(), mapping).expect("the FMU builds");

    let situation = Situation {
        pose: car_at(&road, 100.0, PURSUIT.lateral_tgt, 0.0),
        speed: SPEED,
        scan: Vec::new(),
    };
    let mut wanted = |tick: i64, v0_speed_tgt: f64| {
        let now = SimTime::from_millis(tick * PERIOD_MS);
        let inbox = vec![
            message(
                "pose",
                json!({
                    "position": situation.pose.position,
                    "orientation": situation.pose.orientation,
                    "speed": situation.speed,
                }),
            ),
            message("tuning", json!({ "v0_speed_tgt": v0_speed_tgt })),
        ];
        let mut ctx = StepCtx::new(now, None, WORLD, 0, inbox);
        fmu.step(&mut ctx).expect("the FMU steps");
        let payload: Value =
            serde_json::from_slice(&ctx.take_outbox()[0].1).expect("the payload decodes");
        payload["accel_cmd"].as_f64().expect("an acceleration")
    };

    // Wanting exactly the speed it holds asks for nothing, and wanting
    // twice that asks for more than the default target did.
    let at_the_default = wanted(0, IDM.v0_speed_tgt);
    let at_this_speed = wanted(1, SPEED);
    let at_twice_it = wanted(2, SPEED * 2.0);

    assert_eq!(
        at_this_speed.to_bits(),
        idm_accel(
            SPEED,
            FREE_ROAD.range,
            0.0,
            IdmParams {
                v0_speed_tgt: SPEED,
                ..IDM
            }
        )
        .to_bits(),
        "{MAYBE_STALE}"
    );
    assert!(
        at_this_speed.abs() < 1e-12,
        "at its target: {at_this_speed}"
    );
    assert!(at_twice_it > at_the_default);
}

#[test]
fn the_nearest_of_several_detections_is_the_one_followed() {
    // The sweeps run `nearest_detection` on both sides of the boundary, so
    // they would agree on the wrong car as readily as on the right one.
    // Here the answer is named rather than searched for: the command has to
    // be the one the nearest detection alone would have given.
    const SPEED: f64 = 20.0;

    let road = Waypoints::build_straight((0.0, 0.0), (600.0, 0.0));
    let mut fmu = FmuComponent::new(
        "controller",
        packaged(),
        controller_mapping(&road, PURSUIT, IDM),
    )
    .expect("the FMU builds");
    let situation = |scan: Vec<Detection>| Situation {
        pose: car_at(&road, 100.0, PURSUIT.lateral_tgt, 0.0),
        speed: SPEED,
        scan,
    };

    // Three ahead, the nearest in neither the first slot nor the last, so
    // taking either end would be a different command rather than the same
    // one by luck.
    let scan = scan_of(&[(2, 55.0, -1.0), (19, 18.0, -6.0), (44, 31.0, 2.0)]);
    let (accel_cmd, _) = fmu_commands(&mut fmu, 0, &situation(scan));
    assert_eq!(
        accel_cmd.to_bits(),
        idm_accel(SPEED, 18.0, 6.0, IDM).to_bits(),
        "followed something other than the nearest\n{MAYBE_STALE}"
    );
    for (range, approach_rate) in [(55.0, 1.0), (31.0, -2.0)] {
        assert_ne!(
            accel_cmd.to_bits(),
            idm_accel(SPEED, range, approach_rate, IDM).to_bits(),
            "the assertion above cannot tell {range} m apart from the nearest"
        );
    }

    // A tie goes to the earlier slot, which is what makes a scan answer the
    // same way every time it is read. Both carry the same range and
    // different rates, so which one won reaches the command.
    let tied = scan_of(&[(5, 22.0, -3.0), (40, 22.0, 7.0)]);
    let (accel_cmd, _) = fmu_commands(&mut fmu, 1, &situation(tied));
    assert_eq!(
        accel_cmd.to_bits(),
        idm_accel(SPEED, 22.0, 3.0, IDM).to_bits(),
        "a tie went to the later slot\n{MAYBE_STALE}"
    );
    assert_ne!(
        accel_cmd.to_bits(),
        idm_accel(SPEED, 22.0, -7.0, IDM).to_bits(),
        "the assertion above cannot tell the two tied slots apart"
    );
}

#[test]
fn a_scan_shorter_than_the_arrays_reads_as_free_road_past_its_end() {
    // The radar publishes what it found, so a scan is almost always
    // shorter than the arrays it crosses in. What fills the rest is the
    // mapping's own default rather than anything the sensor said, and
    // getting it wrong reads as a phantom car sitting at whatever the tail
    // of the buffer held.
    let road = Waypoints::build_straight((0.0, 0.0), (400.0, 0.0));
    let lead = Detection {
        range: 30.0,
        range_rate: -2.0,
    };
    let situation = |scan: Vec<Detection>| Situation {
        pose: car_at(&road, 50.0, PURSUIT.lateral_tgt, 0.0),
        speed: 22.0,
        scan,
    };

    // One detection against a full array holding the same detection and
    // free road everywhere else: the FMU cannot tell them apart, because
    // padding is what the short one becomes on the way in.
    let mut padded = vec![FREE_ROAD; MAX_DETECTIONS];
    padded[0] = lead;
    let commands = commands_over(
        &road,
        PURSUIT,
        IDM,
        &[
            situation(vec![lead]),
            situation(padded),
            situation(Vec::new()),
        ],
    );
    assert_eq!(commands[0], commands[1], "a short scan is a padded one");
    assert!(
        commands[0].0 < commands[2].0,
        "a lead 30 m ahead should be braked for where an empty scan is not: {commands:?}"
    );
}
