//! The highway demo as a dynamic world (milestone 4, section 5): traffic
//! that a component decides to create and retire while the run is under
//! way, and which reproduces exactly when the run is repeated.
//!
//! This is the end-to-end counterpart to the conductor's membership tests.
//! Those drive joins and leaves from the test itself; here the decisions
//! come from inside the sim, which is the part that has to stay
//! deterministic.

use continuo_conductor::record::LogEvent;
use continuo_conductor::{Conductor, Divergence, EventLog, Recorder, Verifier};
use continuo_core::SimTime;
use continuo_examples::traffic_world::{self, TrafficRequestHandler};
use continuo_transport::{InProcTransport, MonitorTransport};

/// Runs the demo world for `seconds`, returning its recorded log.
fn record_highway(seconds: i64) -> EventLog {
    let config = traffic_world::config();
    let recorder = Recorder::new(&config);
    let traffic_request_handler = TrafficRequestHandler::default();
    let mut conductor = Conductor::new(
        config,
        traffic_request_handler.wrap_transport(MonitorTransport::new(
            InProcTransport::new(),
            recorder.message_callback(),
        )),
    )
    .expect("free-run config is always accepted");
    conductor.set_tick_callback(recorder.tick_callback());
    conductor.set_membership_callback(recorder.membership_callback());
    conductor.set_observation_callback(recorder.observation_callback());
    traffic_world::setup_live_traffic_scenario(&mut conductor).expect("the world builds");
    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_request_handler,
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

    // The ego, the spawner, and the logger are there from the start; every
    // traffic car arrived mid-run.
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
fn a_car_leaves_as_a_whole_actor() {
    // Both halves of a composite go together, adjacent in the log, because
    // the driver removes the actor rather than its parts.
    let log = record_highway(traffic_world::SIM_SECONDS);
    let left = left(&log);
    let first = left.first().expect("the ego overtakes somebody");
    let actor = first.split('/').next().expect("a leaf path has an actor");

    let pair: Vec<&&str> = left.iter().filter(|p| p.starts_with(actor)).collect();
    assert_eq!(
        pair,
        vec![
            &format!("{actor}/controller").as_str(),
            &format!("{actor}/physics").as_str()
        ],
        "one leave per leaf, controller before physics - declaration order"
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

/// Re-runs the demo world live against `recorded`, checking events as they
/// happen. Returns how far the run got and the verifier's verdict - the
/// pair is the point, since a divergence is supposed to stop the run rather
/// than merely be reported at the end of it.
fn verify_highway(recorded: EventLog, seconds: i64) -> (SimTime, Result<usize, Divergence>) {
    let config = traffic_world::config();
    let verifier = Verifier::new(recorded, &config);
    let traffic_request_handler = TrafficRequestHandler::default();
    let mut conductor = Conductor::new(
        config,
        traffic_request_handler.wrap_transport(MonitorTransport::new(
            InProcTransport::new(),
            verifier.message_callback(),
        )),
    )
    .expect("free-run config is always accepted");
    conductor.set_tick_callback(verifier.tick_callback());
    conductor.set_membership_callback(verifier.membership_callback());
    traffic_world::setup_live_traffic_scenario(&mut conductor).expect("the world builds");

    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_request_handler,
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
    // rebuilds the same traffic: the spawner's requests are applied
    // identically, or the joins would not line up.
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
            LogEvent::Msg(message) if message.time >= halfway => Some(message),
            _ => None,
        })
        .expect("the run publishes messages past its midpoint");
    let tampered_at = tampered.time;
    tampered.key = "continuo/demo/actor/nobody/pose".to_string();

    let (reached, verdict) = verify_highway(recorded, seconds);

    assert!(verdict.is_err(), "a tampered log must not verify");
    assert!(
        reached < SimTime::from_secs(seconds),
        "and the run stops there rather than finishing: reached {reached}, \
         tampered at {tampered_at}"
    );
}
