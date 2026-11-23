//! The highway demo as a dynamic world (milestone 4, section 5): traffic
//! that a component decides to create and retire while the run is under
//! way, and which reproduces exactly when the run is repeated.
//!
//! This is the end-to-end counterpart to the conductor's membership tests.
//! Those drive joins and leaves from the test itself; here the decisions
//! come from inside the sim, which is the part that has to stay
//! deterministic.

use std::collections::BTreeSet;

use continuo_actors::AccelCmd;
use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, Divergence, EventLog, RecordedObservation, Recorder, Verifier,
};
use continuo_core::{ComponentPath, SimTime};
use continuo_examples::traffic_world::{self, TrafficHandler};
use continuo_transport::{InProcTransport, MonitorTransport};

/// Runs the demo world for `seconds`, returning its recorded log.
fn record_highway(seconds: i64) -> EventLog {
    let config = traffic_world::config();
    let recorder = Recorder::new(&config);
    let traffic_handler = TrafficHandler::default();
    let mut transport = MonitorTransport::new(InProcTransport::new());
    transport.add_message_callback(recorder.message_callback());
    transport.add_message_callback(traffic_handler.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(recorder.tick_callback());
    conductor.add_membership_callback(recorder.membership_callback());
    conductor.add_membership_callback(traffic_handler.membership_callback());
    conductor.add_observation_callback(recorder.observation_callback());
    traffic_world::setup_live_traffic_scenario(&mut conductor).expect("the world builds");
    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_handler,
        SimTime::from_secs(seconds),
        None,
    )
    .expect("the run completes");

    // Return the recorded log.
    recorder.finish()
}

/// Actor names in the order they joined and left, from a recorded log.
fn joined(log: &EventLog) -> Vec<&str> {
    log.events
        .iter()
        .filter_map(|event| match event {
            LogEvent::Join(join) => Some(join.path.as_str()),
            _ => None,
        })
        .collect()
}

fn left(log: &EventLog) -> Vec<&str> {
    log.events
        .iter()
        .filter_map(|event| match event {
            LogEvent::Leave(leave) => Some(leave.path.as_str()),
            _ => None,
        })
        .collect()
}

#[test]
fn traffic_arrives_and_retires_while_the_run_is_under_way() {
    let log = record_highway(traffic_world::SIM_SECONDS);
    let joined = joined(&log);
    let left = left(&log);

    // The ego and the spawner are there from the start; every traffic car
    // arrived mid-run.
    assert!(joined.contains(&"ego/controller"));
    assert!(joined.contains(&"traffic_spawner"));
    let traffic_joins = joined.iter().filter(|p| p.starts_with("traffic")).count();
    let traffic_leaves = left.iter().filter(|p| p.starts_with("traffic")).count();

    assert!(
        traffic_joins > 12,
        "more cars joined than the road holds at once, so the population \
         turned over rather than being built once: {traffic_joins} joins"
    );
    assert!(
        traffic_leaves >= 2,
        "and the ego overtook some of them: {traffic_leaves} leaves"
    );
    assert!(
        traffic_leaves < traffic_joins,
        "with cars still on the road when the run ends"
    );
}

#[test]
fn no_two_cars_collide_anywhere_in_the_demo() {
    // An ego collision would fail the run inside `record_highway`. Traffic
    // collisions do not stop the run, so this looks for them in the log.
    let log = record_highway(traffic_world::SIM_SECONDS);
    let collisions: Vec<_> = log
        .events
        .iter()
        .filter(|event| matches!(event, LogEvent::Observed(RecordedObservation::Collision(_))))
        .collect();
    assert!(collisions.is_empty(), "{collisions:?}");
}

#[test]
fn following_actually_engages_somewhere() {
    // A traffic car on a free road holds the speed it spawned at and
    // commands next to nothing, so a firm braking command means it slowed
    // for a car ahead. Braking is a negative acceleration command, as a
    // fraction of the car's braking limit. None at all would mean the
    // radar never reached the controller.
    let log = record_highway(traffic_world::SIM_SECONDS);
    let hardest_braking = log
        .events
        .iter()
        .filter_map(|event| match event {
            LogEvent::Msg(message)
                if message.key.starts_with("continuo/demo/actor/traffic")
                    && message.key.ends_with("/accel_cmd") =>
            {
                let command: AccelCmd = serde_json::from_str(message.payload.get())
                    .expect("an acceleration command decodes");
                Some(command.accel_cmd)
            }
            _ => None,
        })
        .fold(0.0, f64::min);

    assert!(
        hardest_braking < -0.1,
        "no traffic car braked harder than a tenth of its limit: {hardest_braking}"
    );
}

/// Every traffic car's joins and leaves in a log, with their instants. A
/// car's components join and leave together, so the set holds one of each
/// per car.
fn traffic_membership(log: &EventLog) -> BTreeSet<(String, &'static str, SimTime)> {
    let actor_of = |path: &str| {
        let path = ComponentPath::parse(path).expect("a recorded path is valid");
        path.actor()
            .expect("a recorded path has an actor")
            .to_string()
    };
    log.events
        .iter()
        .filter_map(|event| match event {
            LogEvent::Join(join) => Some((actor_of(&join.path), "joined", join.first_due)),
            LogEvent::Leave(leave) => Some((actor_of(&leave.path), "left", leave.leaves_at)),
            _ => None,
        })
        .filter(|(actor, _, _)| actor.starts_with("traffic") && actor != "traffic_spawner")
        .collect()
}

#[test]
fn playback_doubles_join_and_leave_when_their_originals_did() {
    let live_run_log = record_highway(traffic_world::SIM_SECONDS);
    // Without a leave in the live run, the comparison could not catch a
    // double that never leaves. A car leaves only after joining, so there
    // are joins to compare too.
    assert!(
        traffic_membership(&live_run_log)
            .iter()
            .any(|(_, change, _)| *change == "left"),
        "some traffic left during the live run"
    );

    let config = traffic_world::config();
    let playback_recorder = Recorder::new(&config);
    let traffic_handler = TrafficHandler::default();
    let mut transport = MonitorTransport::new(InProcTransport::new());
    transport.add_message_callback(traffic_handler.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_membership_callback(playback_recorder.membership_callback());
    conductor.add_membership_callback(traffic_handler.membership_callback());
    traffic_world::setup_playback_traffic_scenario(&mut conductor, &live_run_log, 30.0)
        .expect("the playback world builds");
    traffic_world::run_playback_traffic_scenario(
        &mut conductor,
        &traffic_handler,
        SimTime::from_secs(traffic_world::SIM_SECONDS),
    )
    .expect("the playback run completes");

    let playback_log = playback_recorder.finish();
    assert_eq!(
        traffic_membership(&playback_log),
        traffic_membership(&live_run_log)
    );
}

#[test]
fn a_car_leaves_as_a_whole_actor() {
    // Every part of a composite goes together, adjacent in the log, because
    // the driver removes the actor rather than its parts.
    let log = record_highway(traffic_world::SIM_SECONDS);
    let left = left(&log);
    let first = left.first().expect("some car leaves");
    let path = ComponentPath::parse(first).expect("a leave names a valid path");
    let actor = path.actor().expect("a leaf path has an actor").as_str();

    // The slash keeps traffic1 from also matching the parts of traffic10.
    let prefix = format!("{actor}/");
    let pair: Vec<&&str> = left.iter().filter(|p| p.starts_with(&prefix)).collect();
    assert_eq!(
        pair,
        vec![
            &format!("{actor}/radar").as_str(),
            &format!("{actor}/controller").as_str(),
            &format!("{actor}/physics").as_str()
        ],
        "one leave per leaf, radar then controller then physics, in declaration order"
    );
}

#[test]
fn the_dynamic_world_reproduces_exactly() {
    // The whole point of deciding inside the sim: the traffic pattern is
    // part of the run, so a second run of the same seed produces it again,
    // down to the last byte.
    let first = record_highway(10);
    let second = record_highway(10);

    assert_eq!(
        first.first_divergence(&second),
        None,
        "two runs of the same seeded scenario must agree everywhere"
    );
    assert_eq!(first.final_world_hash(), second.final_world_hash());
}

/// What a full demo run hashes to.
///
/// [`the_dynamic_world_reproduces_exactly`] compares two runs in one process,
/// so it catches a run that does not repeat but says nothing about whether
/// this machine agrees with any other: both its runs move together and still
/// match. A written-down value is what makes disagreement a failure, since
/// `cargo test` runs on every CI agent.
///
/// There is something to catch. The hash reaches `sin`, `cos`, `asin`, and
/// `atan2` through the unicycle integration, the quaternion conversions and
/// the steering law, and IEEE 754 requires correct rounding for `sqrt` but
/// for none of those.
///
/// It moves when the scenario, the seed, or the hashing moves, each a
/// deliberate act. Update it and README.md's sample output together. A
/// stale controller FMU moves it too, which `cargo xtask package-fmus`
/// fixes.
const DEMO_WORLD_HASH: u64 = 0xacd1_34a5_4d7e_0963;

#[test]
fn the_demo_world_hashes_to_the_same_value_on_every_platform() {
    // The whole demo rather than the ten seconds above, since this is the
    // number README.md quotes.
    let log = record_highway(traffic_world::SIM_SECONDS);
    let hash = log
        .final_world_hash()
        .expect("a run that reached its end has a world hash");

    // Hex, because the point of failing is that someone copies the value into
    // the constant above, which is written in hex.
    assert_eq!(
        hash, DEMO_WORLD_HASH,
        "the demo's world hash is {hash:016x}, expected {DEMO_WORLD_HASH:016x}. \
         If the scenario, the seed, or the hashing changed on purpose, update \
         DEMO_WORLD_HASH and README.md together. If nothing changed, this \
         platform's arithmetic disagrees with the one the value was taken on"
    );
}

/// Re-runs the demo world live against `recorded`, checking events as they
/// happen. Returns how far the run got and the verifier's verdict, the
/// pair is the point, since a divergence is supposed to stop the run rather
/// than merely be reported at the end of it.
fn verify_highway(recorded: EventLog, seconds: i64) -> (SimTime, Result<usize, Divergence>) {
    let config = traffic_world::config();
    let verifier = Verifier::new(recorded, &config);
    let traffic_handler = TrafficHandler::default();
    let mut transport = MonitorTransport::new(InProcTransport::new());
    transport.add_message_callback(verifier.message_callback());
    transport.add_message_callback(traffic_handler.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(verifier.tick_callback());
    conductor.add_membership_callback(verifier.membership_callback());
    conductor.add_membership_callback(traffic_handler.membership_callback());
    traffic_world::setup_live_traffic_scenario(&mut conductor).expect("the world builds");

    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_handler,
        SimTime::from_secs(seconds),
        Some(&verifier),
    )
    .expect("the run completes");

    // Return where the run stopped, and what the verifier made of it.
    (conductor.sim_time(), verifier.finish())
}

#[test]
fn a_recorded_highway_run_verifies_against_a_re_run() {
    // Verification drives the same `run_live_traffic_scenario` every other
    // example does, so this also pins that a re-run of a *dynamic* world
    // rebuilds the same traffic: the spawner asks for the same cars at the
    // same instants, or the joins would not line up.
    let seconds = 10;
    let recorded = record_highway(seconds);
    let (reached, verdict) = verify_highway(recorded, seconds);

    assert!(
        verdict.is_ok(),
        "a faithful re-run of a dynamic world verifies: {:?}",
        verdict.unwrap_err()
    );
    assert_eq!(
        reached,
        SimTime::from_secs(seconds),
        "and having agreed throughout, it runs to the end"
    );
}

#[test]
fn verification_stops_at_the_first_divergence() {
    let seconds = 10;
    let mut recorded = record_highway(seconds);

    // Rewrite the key of a message halfway in. The re-run publishes the
    // real one, so the comparison fails there and everything after it is
    // being checked against a log the run has already left behind.
    let halfway = SimTime::from_secs(seconds / 2);
    let tampered = recorded
        .events
        .iter_mut()
        .find_map(|event| match event {
            LogEvent::Msg(message) if message.sim_time >= halfway => Some(message),
            _ => None,
        })
        .expect("the run publishes messages past its midpoint");
    let tampered_at = tampered.sim_time;
    tampered.key = "continuo/demo/actor/nobody/pose".to_string();

    let (reached, verdict) = verify_highway(recorded, seconds);

    assert!(verdict.is_err(), "a tampered log must not verify");
    assert!(
        reached < SimTime::from_secs(seconds),
        "and the run stops there rather than finishing: reached {reached}, \
         tampered at {tampered_at}"
    );
}
