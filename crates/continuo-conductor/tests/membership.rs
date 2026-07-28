//! Runtime membership (milestone 4): components joining and leaving a
//! running world.
//!
//! Recording membership changes in the event log, and the timeout policy
//! that drops a component, arrive in the later sections of this milestone.

use std::sync::{Arc, Mutex};

use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, ConductorError, JoinMetadata, LeaveMetadata, Pacing, RecordedJoin,
    RecordedLeave, Recorder, Verifier,
};
use continuo_core::{Component, ComponentId, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::{InProcTransport, MonitorTransport};

/// Every step every component took, in order.
type StepLog = Arc<Mutex<Vec<(String, SimTime)>>>;

/// Records each step it takes, so a departure shows up as silence.
struct Ticker {
    id: &'static str,
    period: SimDuration,
    steps: StepLog,
}

impl Component for Ticker {
    fn id(&self) -> ComponentId {
        ComponentId::new(self.id).expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        self.steps
            .lock()
            .expect("step log mutex is never poisoned")
            .push((self.id.to_string(), ctx.now()));

        // Return the next due time, one period out.
        ctx.now() + self.period
    }
}

/// A sim-time duration, in milliseconds.
fn dur_ms(millis: i64) -> SimDuration {
    SimDuration::from_millis(millis)
}

/// A sim-time instant, in milliseconds.
fn t_sim_ms(millis: i64) -> SimTime {
    SimTime::from_millis(millis)
}

fn membership_config() -> ConductorConfig {
    ConductorConfig {
        world_name: "membership-test".into(),
        world_seed: 0,
        pacing: Pacing::FreeRun,
    }
}

fn new_conductor() -> Conductor<InProcTransport> {
    Conductor::new(membership_config(), InProcTransport::new())
        .expect("free-run config is always accepted")
}

/// Two 10 ms tickers; `a` leaves after the t=10 ms instant.
fn run_with_departure(steps: &StepLog) -> Conductor<InProcTransport> {
    let mut conductor = new_conductor();
    for id in ["a", "b"] {
        conductor
            .add_component(
                "",
                Box::new(Ticker {
                    id,
                    period: dur_ms(10),
                    steps: steps.clone(),
                }),
            )
            .expect("registration succeeds");
    }

    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor.remove_component("a").expect("`a` is registered");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");

    // Return the conductor so callers can read its hash.
    conductor
}

fn steps_of(steps: &StepLog, id: &str) -> Vec<SimTime> {
    steps
        .lock()
        .expect("step log mutex is never poisoned")
        .iter()
        .filter(|(who, _)| who == id)
        .map(|(_, when)| *when)
        .collect()
}

#[test]
fn a_departed_component_stops_stepping_while_the_rest_carry_on() {
    let steps: StepLog = Default::default();
    run_with_departure(&steps);

    assert_eq!(
        steps_of(&steps, "a"),
        vec![t_sim_ms(0), t_sim_ms(10)],
        "`a` stepped up to its departure and never again"
    );
    assert_eq!(
        steps_of(&steps, "b"),
        vec![t_sim_ms(0), t_sim_ms(10), t_sim_ms(20), t_sim_ms(30)],
        "`b` is unaffected by its neighbour leaving"
    );
}

#[test]
fn the_same_departure_reproduces_the_same_world_hash() {
    // Departures must not make a run unreproducible: the same scenario with
    // the same leave at the same instant fingerprints identically.
    let first = run_with_departure(&Default::default()).world_hash();
    let second = run_with_departure(&Default::default()).world_hash();
    assert_eq!(first, second);
}

#[test]
fn removing_a_path_nobody_is_registered_at_is_an_error() {
    let mut conductor = new_conductor();
    assert!(matches!(
        conductor.remove_component("nobody"),
        Err(ConductorError::UnknownPath(_))
    ));
}

/// A ticker of the given id and period, logging into `steps`.
fn ticker(id: &'static str, steps: &StepLog) -> Box<Ticker> {
    Box::new(Ticker {
        id,
        period: dur_ms(10),
        steps: steps.clone(),
    })
}

#[test]
fn a_component_admitted_mid_run_first_steps_at_its_declared_time() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    conductor
        .add_component(JoinMetadata::at("", t_sim_ms(25)), ticker("b", &steps))
        .expect("25 ms is still ahead");
    conductor.run_until(t_sim_ms(40)).expect("steps succeed");

    assert_eq!(
        steps_of(&steps, "b"),
        vec![t_sim_ms(25), t_sim_ms(35)],
        "the newcomer starts at the instant it asked for, not at zero and \
         not at the next instant that happened to come along"
    );
    assert_eq!(
        steps_of(&steps, "a"),
        vec![
            t_sim_ms(0),
            t_sim_ms(10),
            t_sim_ms(20),
            t_sim_ms(30),
            t_sim_ms(40)
        ],
        "the incumbent's own cadence is undisturbed"
    );
}

#[test]
fn a_joining_component_is_scheduled_before_its_instant_arrives() {
    // The point of declaring `first_due` up front: the conductor knows the
    // newcomer is due at that instant while the instant is still in the
    // future, so the barrier there waits for it instead of discovering it
    // late.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    assert_eq!(
        conductor.next_scheduled(),
        Some(t_sim_ms(20)),
        "only `a` is due, at its next period"
    );

    conductor
        .add_component(JoinMetadata::at("", t_sim_ms(15)), ticker("b", &steps))
        .expect("15 ms is still ahead");

    assert_eq!(
        conductor.next_scheduled(),
        Some(t_sim_ms(15)),
        "admitting the newcomer scheduled it immediately, making it the \
         earliest thing due"
    );
}

#[test]
fn joining_an_instant_that_has_already_happened_is_an_error() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    // Well behind the run.
    assert!(matches!(
        conductor.add_component(JoinMetadata::at("", t_sim_ms(5)), ticker("late", &steps)),
        Err(ConductorError::JoinInThePast { .. })
    ));
    // And the instant just stepped is closed too: joining it would step
    // t=10 ms a second time.
    assert!(matches!(
        conductor.add_component(JoinMetadata::at("", t_sim_ms(10)), ticker("late", &steps)),
        Err(ConductorError::JoinInThePast { .. })
    ));
    // One nanosecond later is open.
    conductor
        .add_component(
            JoinMetadata::at("", t_sim_ms(10) + SimDuration::from_nanos(1)),
            ticker("just_in_time", &steps),
        )
        .expect("the next instant has not happened yet");

    // A rejected join leaves nothing behind: `late` was never registered.
    assert!(matches!(
        conductor.remove_component("late"),
        Err(ConductorError::UnknownPath(_))
    ));
}

#[test]
fn the_parent_path_shorthand_only_works_before_the_run_starts() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();

    // Passing just the parent path means `JoinMetadata::at_start`, so before
    // anything has stepped it lands on the world's opening instant.
    conductor
        .add_component("", ticker("a", &steps))
        .expect("nothing has stepped yet");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    // It still means sim time zero once the run is under way — now long
    // closed — so the shorthand is rejected rather than quietly resolving to
    // an instant the caller never chose. Joining a running world is
    // something you have to say the time for.
    assert!(matches!(
        conductor.add_component("", ticker("b", &steps)),
        Err(ConductorError::JoinInThePast { .. })
    ));
}

/// Runs a world where `b` joins at 25 ms and `a` leaves at 30 ms, with
/// every observation point tapped, and returns the recorded log.
fn record_a_dynamic_run(config: &ConductorConfig) -> continuo_conductor::EventLog {
    let steps: StepLog = Default::default();
    let recorder = Recorder::new(config);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
    let mut conductor =
        Conductor::new(config.clone(), transport).expect("free-run config is always accepted");
    conductor.set_tick_callback(recorder.tick_callback());
    conductor.set_membership_callback(recorder.membership_callback());

    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .add_component(JoinMetadata::at("", t_sim_ms(25)), ticker("b", &steps))
        .expect("25 ms is still ahead");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");
    conductor.remove_component("a").expect("`a` is registered");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    // Return the recorded log.
    recorder.finish()
}

#[test]
fn the_event_log_records_who_joined_and_left() {
    let log = record_a_dynamic_run(&membership_config());

    let joins: Vec<&RecordedJoin> = log
        .events
        .iter()
        .filter_map(|e| match e {
            LogEvent::Join(join) => Some(join),
            _ => None,
        })
        .collect();
    let leaves: Vec<&RecordedLeave> = log
        .events
        .iter()
        .filter_map(|e| match e {
            LogEvent::Leave(leave) => Some(leave),
            _ => None,
        })
        .collect();

    assert_eq!(joins.len(), 2, "`a` at the start and `b` mid-run");
    assert_eq!(joins[0].path, "a");
    assert_eq!(joins[0].first_due, SimTime::ZERO);
    assert_eq!(joins[1].path, "b");
    assert_eq!(
        joins[1].first_due,
        t_sim_ms(25),
        "what the log keeps is the declared first step, not when the join \
         happened to be applied"
    );

    assert_eq!(leaves.len(), 1);
    assert_eq!(leaves[0].path, "a");
}

#[test]
fn a_membership_event_sits_between_the_ticks_it_falls_between() {
    // Nothing records *when* a join was applied, because its position in the
    // stream already says so: `b` joins after the t=10 ms tick and before
    // the next one. That is also the part that may vary once joins arrive
    // over the transport, which is why it is position rather than a field.
    let log = record_a_dynamic_run(&membership_config());

    let join_at = log
        .events
        .iter()
        .position(|e| matches!(e, LogEvent::Join(join) if join.path == "b"))
        .expect("`b` joined");
    let ticks_before = log.events[..join_at]
        .iter()
        .filter(|e| matches!(e, LogEvent::Tick(_)))
        .count();

    // Ticks at 0 and 10 ms precede it; the 20 ms tick does not.
    assert_eq!(ticks_before, 2);
}

#[test]
fn a_recorded_dynamic_run_verifies_against_a_faithful_re_run() {
    let config = membership_config();
    let expected = record_a_dynamic_run(&config);
    let total_events = expected.events.len();

    // Re-run the same scenario live, checking every event — messages, tick
    // fingerprints, and membership changes alike — as it happens.
    let steps: StepLog = Default::default();
    let verifier = Verifier::new(expected, &config);
    let transport = MonitorTransport::new(InProcTransport::new(), verifier.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.set_tick_callback(verifier.tick_callback());
    conductor.set_membership_callback(verifier.membership_callback());

    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .add_component(JoinMetadata::at("", t_sim_ms(25)), ticker("b", &steps))
        .expect("25 ms is still ahead");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");
    conductor.remove_component("a").expect("`a` is registered");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    assert_eq!(verifier.finish().expect("the re-run matches"), total_events);
}

#[test]
fn a_re_run_that_skips_a_departure_is_caught() {
    let config = membership_config();
    let expected = record_a_dynamic_run(&config);

    // Same world, same join — but `a` never leaves.
    let steps: StepLog = Default::default();
    let verifier = Verifier::new(expected, &config);
    let transport = MonitorTransport::new(InProcTransport::new(), verifier.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.set_tick_callback(verifier.tick_callback());
    conductor.set_membership_callback(verifier.membership_callback());

    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .add_component(JoinMetadata::at("", t_sim_ms(25)), ticker("b", &steps))
        .expect("25 ms is still ahead");

    let end = t_sim_ms(50);
    while !verifier.diverged() && conductor.next_scheduled().is_some_and(|t| t <= end) {
        conductor.step_once().expect("steps succeed");
    }

    verifier
        .finish()
        .expect_err("a world that kept a departed component must diverge");
}

#[test]
fn a_scheduled_departure_stops_the_component_at_the_declared_instant() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor
        .add_component("", ticker("b", &steps))
        .expect("registration succeeds");

    // Declared up front, long before it takes effect.
    conductor
        .remove_component(LeaveMetadata::at("a", t_sim_ms(30)))
        .expect("30 ms is still ahead");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    assert_eq!(
        steps_of(&steps, "a"),
        vec![t_sim_ms(0), t_sim_ms(10), t_sim_ms(20)],
        "half-open: `a` steps up to but not including the instant it leaves at"
    );
    assert_eq!(
        steps_of(&steps, "b"),
        vec![
            t_sim_ms(0),
            t_sim_ms(10),
            t_sim_ms(20),
            t_sim_ms(30),
            t_sim_ms(40),
            t_sim_ms(50)
        ],
    );
}

#[test]
fn a_scheduled_departure_does_not_depend_on_when_it_was_requested() {
    // The point of declaring the instant: the same scenario ends the same
    // way whether the request was made at the start or one tick before it
    // takes effect. Over a transport, that is the difference between a
    // reproducible run and one that depends on delivery.
    let run = |request_after: SimTime| {
        let steps: StepLog = Default::default();
        let mut conductor = new_conductor();
        conductor
            .add_component("", ticker("a", &steps))
            .expect("registration succeeds");
        conductor
            .add_component("", ticker("b", &steps))
            .expect("registration succeeds");

        conductor.run_until(request_after).expect("steps succeed");
        conductor
            .remove_component(LeaveMetadata::at("a", t_sim_ms(30)))
            .expect("30 ms is still ahead");
        conductor.run_until(t_sim_ms(50)).expect("steps succeed");

        // Return the fingerprint and what `a` actually did.
        (conductor.world_hash(), steps_of(&steps, "a"))
    };

    let asked_at_the_start = run(SimTime::ZERO);
    let asked_just_in_time = run(t_sim_ms(20));
    assert_eq!(asked_at_the_start, asked_just_in_time);
    assert_eq!(
        asked_at_the_start.1,
        vec![t_sim_ms(0), t_sim_ms(10), t_sim_ms(20)]
    );
}

#[test]
fn an_instant_left_empty_by_a_leave_produces_no_tick_at_all() {
    // `a` is the only component due at 20 ms, and it leaves at 20 ms. That
    // instant must vanish rather than becoming a tick where nobody steps:
    // an empty tick would take a number, get a fingerprint, and chain into
    // the world hash, all for an instant in which nothing happened.
    //
    // This is why leaves are applied *before* the earliest instant is
    // popped. Applying them after would hand the loop a due set already
    // snapshotted with the departing component in it.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor
        .add_component(
            "",
            Box::new(Ticker {
                id: "b",
                period: dur_ms(30),
                steps: steps.clone(),
            }),
        )
        .expect("registration succeeds");

    conductor
        .remove_component(LeaveMetadata::at("a", t_sim_ms(20)))
        .expect("20 ms is still ahead");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");

    assert_eq!(steps_of(&steps, "a"), vec![t_sim_ms(0), t_sim_ms(10)]);
    assert_eq!(steps_of(&steps, "b"), vec![t_sim_ms(0), t_sim_ms(30)]);
    assert_eq!(
        conductor.tick(),
        3,
        "instants 0, 10 and 30 ticked; 20 held only the departing component \
         and never became a tick"
    );
}

#[test]
fn a_departure_scheduled_for_an_instant_that_has_passed_is_an_error() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(20)).expect("steps succeed");

    // `a` has already stepped at 10 ms, so it cannot retroactively have
    // stopped there.
    assert!(matches!(
        conductor.remove_component(LeaveMetadata::at("a", t_sim_ms(10))),
        Err(ConductorError::LeaveInThePast { .. })
    ));
    // And a departure for a component nobody registered is unknown, whatever
    // instant it names.
    assert!(matches!(
        conductor.remove_component(LeaveMetadata::at("nobody", t_sim_ms(30))),
        Err(ConductorError::UnknownPath(_))
    ));
}

#[test]
fn a_scheduled_departure_is_recorded_with_the_instant_it_takes_effect() {
    let config = membership_config();
    let steps: StepLog = Default::default();
    let recorder = Recorder::new(&config);
    let mut conductor =
        Conductor::new(config, InProcTransport::new()).expect("free-run config is always accepted");
    conductor.set_membership_callback(recorder.membership_callback());

    conductor
        .add_component("", ticker("a", &steps))
        .expect("registration succeeds");
    conductor
        .remove_component(LeaveMetadata::at("a", t_sim_ms(30)))
        .expect("30 ms is still ahead");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    let log = recorder.finish();
    let leaves: Vec<&RecordedLeave> = log
        .events
        .iter()
        .filter_map(|e| match e {
            LogEvent::Leave(leave) => Some(leave),
            _ => None,
        })
        .collect();

    assert_eq!(leaves.len(), 1);
    assert_eq!(leaves[0].path, "a");
    assert_eq!(
        leaves[0].leaves_at,
        t_sim_ms(30),
        "the log keeps the declared instant, which is what shapes the run"
    );
}

#[test]
fn the_same_mid_run_join_reproduces_the_same_world_hash() {
    let run = || {
        let steps: StepLog = Default::default();
        let mut conductor = new_conductor();
        conductor
            .add_component("", ticker("a", &steps))
            .expect("registration succeeds");
        conductor.run_until(t_sim_ms(10)).expect("steps succeed");
        conductor
            .add_component(JoinMetadata::at("", t_sim_ms(25)), ticker("b", &steps))
            .expect("25 ms is still ahead");
        conductor.run_until(t_sim_ms(40)).expect("steps succeed");

        // Return the run's fingerprint.
        conductor.world_hash()
    };
    assert_eq!(run(), run());
}
