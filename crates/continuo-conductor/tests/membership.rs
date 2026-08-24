//! Runtime membership (milestone 4): components joining and leaving a
//! running world, and the event log that records them doing so.
//!
//! Every membership change here is one somebody asked for. The other kind,
//! a component the conductor removes itself after a timeout, goes out
//! through this same path, but what triggers it is a wall-clock measurement,
//! so it is tested in `timing.rs`.

use std::sync::{Arc, Mutex};

use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, ConductorError, JoinMetadata, LeaveMetadata, MembershipChange,
    Pacing, RecordedJoin, RecordedLeave, RecordedObservation, Recorder, Verifier, WORLD_LEVEL,
};
use continuo_core::{Component, ComponentId, CoreError, KeyExpr, SimDuration, SimTime, StepCtx};
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

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        self.steps
            .lock()
            .expect("step log mutex is never poisoned")
            .push((self.id.to_string(), ctx.now()));

        // Return the next due time, one period out.
        Ok(ctx.now() + self.period)
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
            .add_component_at_start(
                WORLD_LEVEL,
                Box::new(Ticker {
                    id,
                    period: dur_ms(10),
                    steps: steps.clone(),
                }),
            )
            .expect("registration succeeds");
    }

    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .remove_component_now("a")
        .expect("`a` is registered");
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
        conductor.remove_component_now("nobody"),
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
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            ticker("b", &steps),
        )
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
fn a_joining_component_is_counted_due_before_its_instant_arrives() {
    // The point of declaring `first_due` up front: the conductor knows the
    // newcomer is due at that instant while the instant is still in the
    // future, so the run reaches it rather than stepping past it. The join
    // is not in the schedule, having no registry slot yet, which is why the
    // count has to look at both.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    assert_eq!(
        conductor.next_due_instant(),
        Some(t_sim_ms(20)),
        "only `a` is due, at its next period"
    );

    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(15)),
            ticker("b", &steps),
        )
        .expect("15 ms is still ahead");

    assert_eq!(
        conductor.next_due_instant(),
        Some(t_sim_ms(15)),
        "a join waiting for 15 ms is an instant the run has to reach, so it \
         is now the earliest thing due"
    );
}

#[test]
fn joining_an_instant_that_has_already_happened_is_an_error() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    // Well behind the run.
    assert!(matches!(
        conductor.add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(5)),
            ticker("late", &steps)
        ),
        Err(ConductorError::JoinInThePast { .. })
    ));
    // And the instant just stepped is closed too: joining it would step
    // t=10 ms a second time.
    assert!(matches!(
        conductor.add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(10)),
            ticker("late", &steps)
        ),
        Err(ConductorError::JoinInThePast { .. })
    ));
    // One nanosecond later is open.
    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(10) + SimDuration::from_nanos(1)),
            ticker("just_in_time", &steps),
        )
        .expect("the next instant has not happened yet");

    // A rejected join leaves nothing behind: `late` was never registered.
    assert!(matches!(
        conductor.remove_component_now("late"),
        Err(ConductorError::UnknownPath(_))
    ));
}

#[test]
fn adding_at_the_start_only_works_before_the_run_starts() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();

    // `add_component_at_start` means sim time zero, so before anything has
    // stepped it lands on the world's opening instant.
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("nothing has stepped yet");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    // Sim time zero has passed once the run is under way, so the same call
    // is rejected rather than resolving to an instant the caller never
    // chose.
    assert!(matches!(
        conductor.add_component_at_start(WORLD_LEVEL, ticker("b", &steps)),
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
    conductor.add_tick_callback(recorder.tick_callback());
    conductor.add_membership_callback(recorder.membership_callback());
    conductor.add_observation_callback(recorder.observation_callback());

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            ticker("b", &steps),
        )
        .expect("25 ms is still ahead");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");
    conductor
        .remove_component_now("a")
        .expect("`a` is registered");
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
         happened to be asked for"
    );

    assert_eq!(leaves.len(), 1);
    assert_eq!(leaves[0].path, "a");
}

#[test]
fn a_join_is_recorded_where_it_takes_effect_not_where_it_was_requested() {
    // `b` is asked for after the t=10 ms tick and declares t=25 ms. The two
    // lines it produces sit in different places, which is the whole point of
    // there being two: the request is recorded where the caller happened to
    // be, and the join at the boundary before `b` first steps.
    let log = record_a_dynamic_run(&membership_config());

    let requested_at = log
        .events
        .iter()
        .position(|e| {
            matches!(
                e,
                LogEvent::Observed(RecordedObservation::JoinRequested(request))
                    if request.path == "b"
            )
        })
        .expect("`b`'s join was requested");
    let joined_at = log
        .events
        .iter()
        .position(|e| matches!(e, LogEvent::Join(join) if join.path == "b"))
        .expect("`b` joined");
    let ticks_before = |index: usize| {
        log.events[..index]
            .iter()
            .filter(|e| matches!(e, LogEvent::Tick(_)))
            .count()
    };

    // The request sits after the ticks at 0 and 10 ms, where the caller
    // made it. The join sits after the 20 ms tick as well, immediately
    // before the 25 ms one it declared.
    assert_eq!(
        ticks_before(requested_at),
        2,
        "where the request was processed"
    );
    assert_eq!(ticks_before(joined_at), 3, "where the join took effect");
}

#[test]
fn a_component_removed_before_its_join_takes_effect_is_never_announced() {
    // A join declared for 25 ms, withdrawn at 10 ms. It was asked for and
    // never admitted, so no observer ever heard of it: announcing the leave
    // alone would report a departure for something that was never there.
    // The two requests are the only trace, which is what they are for.
    let steps: StepLog = Default::default();
    let config = membership_config();
    let recorder = Recorder::new(&config);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
    let mut conductor =
        Conductor::new(config.clone(), transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(recorder.tick_callback());
    conductor.add_membership_callback(recorder.membership_callback());
    conductor.add_observation_callback(recorder.observation_callback());

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            ticker("b", &steps),
        )
        .expect("25 ms is still ahead");
    conductor
        .remove_component_now("b")
        .expect("`b` is registered");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");
    let log = recorder.finish();

    let mentions_b = |event: &LogEvent| match event {
        LogEvent::Join(join) => join.path == "b",
        LogEvent::Leave(leave) => leave.path == "b",
        _ => false,
    };
    assert!(
        !log.events.iter().any(mentions_b),
        "`b` never took effect, so neither half of its membership is recorded"
    );
    assert!(
        log.events.iter().any(|e| matches!(
            e,
            LogEvent::Observed(RecordedObservation::JoinRequested(request))
                if request.path == "b"
        )),
        "the join was still asked for"
    );
    assert!(
        log.events.iter().any(|e| matches!(
            e,
            LogEvent::Observed(RecordedObservation::LeaveRequested(request))
                if request.path == "b" && request.leaves_at.is_none()
        )),
        "and so was the withdrawal, naming no instant"
    );
    assert!(
        !steps
            .lock()
            .expect("step log mutex")
            .iter()
            .any(|(path, _)| path == "b"),
        "`b` never stepped either"
    );
}

#[test]
fn a_recorded_dynamic_run_verifies_against_a_faithful_re_run() {
    let config = membership_config();
    let expected = record_a_dynamic_run(&config);
    // Expectations rather than lines: the log also carries the membership
    // requests, and a re-run is not asked to reproduce those.
    let total_expectations = expected
        .events
        .iter()
        .filter(|event| !matches!(event, LogEvent::Observed(_)))
        .count();
    assert!(
        total_expectations < expected.events.len(),
        "the log has observations in it, or this proves nothing"
    );

    // Re-run the same scenario live, checking every event as it happens:
    // messages, tick fingerprints, and membership changes alike.
    let steps: StepLog = Default::default();
    let verifier = Verifier::new(expected, &config);
    let transport = MonitorTransport::new(InProcTransport::new(), verifier.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(verifier.tick_callback());
    conductor.add_membership_callback(verifier.membership_callback());

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            ticker("b", &steps),
        )
        .expect("25 ms is still ahead");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");
    conductor
        .remove_component_now("a")
        .expect("`a` is registered");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    assert_eq!(
        verifier.finish().expect("the re-run matches"),
        total_expectations
    );
}

#[test]
fn a_re_run_that_skips_a_departure_is_caught() {
    let config = membership_config();
    let expected = record_a_dynamic_run(&config);

    // Same world, same join, but `a` never leaves.
    let steps: StepLog = Default::default();
    let verifier = Verifier::new(expected, &config);
    let transport = MonitorTransport::new(InProcTransport::new(), verifier.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(verifier.tick_callback());
    conductor.add_membership_callback(verifier.membership_callback());

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            ticker("b", &steps),
        )
        .expect("25 ms is still ahead");

    let end = t_sim_ms(50);
    while !verifier.diverged() && conductor.next_due_instant().is_some_and(|t| t <= end) {
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
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("b", &steps))
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
            .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
            .expect("registration succeeds");
        conductor
            .add_component_at_start(WORLD_LEVEL, ticker("b", &steps))
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
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor
        .add_component_at_start(
            WORLD_LEVEL,
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
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
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
    conductor.add_membership_callback(recorder.membership_callback());

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
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
            .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
            .expect("registration succeeds");
        conductor.run_until(t_sim_ms(10)).expect("steps succeed");
        conductor
            .add_component(
                JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
                ticker("b", &steps),
            )
            .expect("25 ms is still ahead");
        conductor.run_until(t_sim_ms(40)).expect("steps succeed");

        // Return the run's fingerprint.
        conductor.world_hash()
    };
    assert_eq!(run(), run());
}

#[test]
fn removing_a_composite_takes_every_leaf_under_it() {
    // An actor leaving a world leaves whole. Removing only some of its
    // parts would leave a controller publishing at a physics model that is
    // gone, so naming the composite takes the subtree.
    let steps: StepLog = Default::default();
    let changes: Arc<Mutex<Vec<MembershipChange>>> = Default::default();
    let mut conductor = new_conductor();
    let announced = changes.clone();
    conductor.add_membership_callback(move |change| {
        announced
            .lock()
            .expect("membership log mutex is never poisoned")
            .push(change.clone());
    });

    for id in ["controller", "physics"] {
        conductor
            .add_component_at_start("car1", ticker(id, &steps))
            .expect("registration succeeds");
    }
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("bystander", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    conductor
        .remove_component_now("car1")
        .expect("`car1` names a composite");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");

    assert_eq!(
        steps_of(&steps, "controller"),
        vec![t_sim_ms(0), t_sim_ms(10)]
    );
    assert_eq!(steps_of(&steps, "physics"), vec![t_sim_ms(0), t_sim_ms(10)]);
    assert_eq!(
        steps_of(&steps, "bystander"),
        vec![t_sim_ms(0), t_sim_ms(10), t_sim_ms(20), t_sim_ms(30)],
        "another actor is untouched by car1 leaving"
    );

    // One leave per leaf, never one for the composite, and in the order the
    // components would have stepped.
    let changes = changes
        .lock()
        .expect("membership log mutex is never poisoned");
    let left: Vec<&str> = changes
        .iter()
        .filter_map(|change| match change {
            MembershipChange::Left(leave) => Some(leave.path.as_str()),
            MembershipChange::Joined(_) => None,
        })
        .collect();
    assert_eq!(left, vec!["car1/controller", "car1/physics"]);
}

#[test]
fn removing_a_composite_is_one_request_against_a_leave_each() {
    // The leaves keep the leaf discipline, because a leaf is what joins and
    // what leaves. The request says what was actually asked for, which was
    // the composite, so the two lines answer different questions and the
    // counts do not match on purpose.
    let steps: StepLog = Default::default();
    let observations: Arc<Mutex<Vec<RecordedObservation>>> = Default::default();
    let mut conductor = new_conductor();
    let observed = observations.clone();
    conductor.add_observation_callback(move |observation| {
        observed
            .lock()
            .expect("observation log mutex is never poisoned")
            .push(observation.clone());
    });

    for id in ["controller", "physics"] {
        conductor
            .add_component_at_start("car1", ticker(id, &steps))
            .expect("registration succeeds");
    }
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    conductor
        .remove_component(LeaveMetadata::at("car1", t_sim_ms(20)))
        .expect("20 ms is still ahead");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");

    let observations = observations
        .lock()
        .expect("observation log mutex is never poisoned");
    let requested: Vec<(&str, Option<SimTime>)> = observations
        .iter()
        .filter_map(|observation| match observation {
            RecordedObservation::LeaveRequested(request) => {
                Some((request.path.as_str(), request.leaves_at))
            }
            _ => None,
        })
        .collect();
    assert_eq!(
        requested,
        vec![("car1", Some(t_sim_ms(20)))],
        "one request, naming the composite and the instant it asked for"
    );
}

#[test]
fn a_composite_leave_can_be_scheduled_like_any_other() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    for id in ["controller", "physics"] {
        conductor
            .add_component_at_start("car1", ticker(id, &steps))
            .expect("registration succeeds");
    }
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("bystander", &steps))
        .expect("registration succeeds");

    conductor
        .remove_component(LeaveMetadata::at("car1", t_sim_ms(20)))
        .expect("20 ms is still ahead");
    conductor.run_until(t_sim_ms(30)).expect("steps succeed");

    for id in ["controller", "physics"] {
        assert_eq!(
            steps_of(&steps, id),
            vec![t_sim_ms(0), t_sim_ms(10)],
            "half-open at 20 ms, for every leaf of the composite"
        );
    }
}

#[test]
fn a_path_naming_neither_a_leaf_nor_a_composite_is_an_error() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component_at_start("car1", ticker("physics", &steps))
        .expect("registration succeeds");

    assert!(matches!(
        conductor.remove_component_now("car2"),
        Err(ConductorError::UnknownPath(_))
    ));
    // And an emptied composite stops naming anything, rather than becoming
    // a path that silently removes nothing.
    conductor
        .remove_component_now("car1")
        .expect("car1 has a leaf");
    assert!(matches!(
        conductor.remove_component_now("car1"),
        Err(ConductorError::UnknownPath(_))
    ));
}

#[test]
fn an_emptied_composite_rejoins_as_the_newest_sibling() {
    // The arrival rule reaches branches too, not just leaves. A composite
    // whose last leaf left is forgotten by its parent, so rebuilding it
    // later puts it at the end of the child list, where its fresh
    // declaration indexes say it belongs. Leaving it in place would restore
    // its old position and let tree order and index order disagree.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component_at_start("car1", ticker("physics", &steps))
        .expect("registration succeeds");
    conductor
        .add_component_at_start("car2", ticker("physics", &steps))
        .expect("registration succeeds");

    conductor
        .remove_component_now("car1")
        .expect("car1 is live");
    conductor
        .add_component_at_start("car1", ticker("physics", &steps))
        .expect("the path is free again");

    // Nothing observable changes at the world level, where actors never see
    // each other same-instant, but the tree is what a nested composite
    // would read, so it has to be right before anything nests.
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");
    assert_eq!(
        steps_of(&steps, "physics").len(),
        4,
        "two live cars, two instants each"
    );
}

/// Taps the conductor's membership callback, returning the log it fills:
/// every change announced, in order, as `(path, kind)`.
fn record_membership_changes(
    conductor: &mut Conductor<InProcTransport>,
) -> Arc<Mutex<Vec<(String, &'static str)>>> {
    let changes: Arc<Mutex<Vec<(String, &'static str)>>> = Default::default();
    let announced = changes.clone();
    conductor.add_membership_callback(move |change| {
        let (path, kind) = match change {
            MembershipChange::Joined(join) => (join.path.clone(), "joined"),
            MembershipChange::Left(leave) => (leave.path.clone(), "left"),
        };
        announced
            .lock()
            .expect("membership log mutex is never poisoned")
            .push((path, kind));
    });

    // Return the log the callback fills.
    changes
}

#[test]
fn a_leave_retires_the_component_at_a_path_rather_than_the_join_waiting_for_it() {
    // Two components claim `a`: the one registered there, and a join
    // declared for 25 ms. A leave names a path, and the one occupying it is
    // what leaves, so the newcomer still arrives at the instant it asked
    // for. Withdrawing the join instead would leave the incumbent running
    // and answer a leave by cancelling something else.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    let announced = record_membership_changes(&mut conductor);
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            Box::new(Ticker {
                id: "a",
                period: dur_ms(10),
                steps: steps.clone(),
            }),
        )
        .expect("25 ms is still ahead");
    conductor
        .remove_component_now("a")
        .expect("`a` is registered");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    assert_eq!(
        steps_of(&steps, "a"),
        vec![
            t_sim_ms(0),
            t_sim_ms(10),
            t_sim_ms(25),
            t_sim_ms(35),
            t_sim_ms(45)
        ],
        "the incumbent stopped at 10 ms and the newcomer took the path at 25 ms"
    );
    assert_eq!(
        *announced.lock().expect("membership log mutex"),
        vec![
            ("a".to_string(), "joined"),
            ("a".to_string(), "left"),
            ("a".to_string(), "joined"),
        ]
    );
}

#[test]
fn a_leave_and_a_join_at_one_instant_hand_the_path_over() {
    // The boundary settles leaves before joins, so a path freed at 25 ms is
    // free for whoever declared 25 ms. The order they are announced in is
    // the order the world changed.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    let announced = record_membership_changes(&mut conductor);
    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            Box::new(Ticker {
                id: "a",
                period: dur_ms(10),
                steps: steps.clone(),
            }),
        )
        .expect("25 ms is still ahead");
    conductor
        .remove_component(LeaveMetadata::at("a", t_sim_ms(25)))
        .expect("`a` is registered");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    assert_eq!(
        steps_of(&steps, "a"),
        vec![
            t_sim_ms(0),
            t_sim_ms(10),
            t_sim_ms(20),
            t_sim_ms(25),
            t_sim_ms(35),
            t_sim_ms(45)
        ],
        "the incumbent stepped up to 20 ms and the newcomer from 25 ms"
    );
    assert_eq!(
        *announced.lock().expect("membership log mutex"),
        vec![
            ("a".to_string(), "joined"),
            ("a".to_string(), "left"),
            ("a".to_string(), "joined"),
        ],
        "retired first, admitted second"
    );
}

#[test]
fn removing_a_composite_takes_the_newcomers_promised_to_it() {
    // An actor leaving whole takes the parts it was still expecting as well
    // as the ones already there. Leaving a waiting join behind would rebuild
    // the composite at its instant, out of one component nobody asked to
    // keep.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    let announced = record_membership_changes(&mut conductor);
    conductor
        .add_component_at_start("car1", ticker("physics", &steps))
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    conductor
        .add_component(
            JoinMetadata::at("car1", t_sim_ms(25)),
            ticker("radar", &steps),
        )
        .expect("25 ms is still ahead");
    conductor
        .remove_component_now("car1")
        .expect("`car1` names a composite");
    conductor.run_until(t_sim_ms(50)).expect("steps succeed");

    assert!(
        steps_of(&steps, "radar").is_empty(),
        "the newcomer promised to `car1` never arrived"
    );
    assert_eq!(
        *announced.lock().expect("membership log mutex"),
        vec![
            ("car1/physics".to_string(), "joined"),
            ("car1/physics".to_string(), "left"),
        ],
        "the waiting join was withdrawn, so neither half of it is announced"
    );
    assert!(
        matches!(
            conductor.remove_component_now("car1"),
            Err(ConductorError::UnknownPath(_))
        ),
        "nothing under `car1` is left to name"
    );
}

/// Publishes on every step, so how much a newcomer is handed at its first
/// step depends on how long it has been subscribed.
struct Talker;

impl Component for Talker {
    fn id(&self) -> ComponentId {
        ComponentId::new("talker").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        ctx.publish(
            KeyExpr::new("chatter").expect("valid key"),
            &ctx.now().to_canonical_string(),
        )?;

        // Return the next due time, one period out.
        Ok(ctx.now() + dur_ms(10))
    }
}

/// Records how many messages it was handed at each step.
struct Listener(Arc<Mutex<Vec<usize>>>);

impl Component for Listener {
    fn id(&self) -> ComponentId {
        ComponentId::new("listener").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        vec![KeyExpr::new("chatter").expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        self.0
            .lock()
            .expect("inbox log mutex is never poisoned")
            .push(ctx.inbox().len());

        // Return the next due time, one period out.
        Ok(ctx.now() + dur_ms(10))
    }
}

/// Runs a talker from zero and asks for a listener at `requested_at`,
/// declaring 25 ms either way.
fn run_with_a_late_joiner(requested_at: i64) -> (Vec<usize>, u64) {
    let heard: Arc<Mutex<Vec<usize>>> = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component_at_start(WORLD_LEVEL, Box::new(Talker))
        .expect("registration succeeds");
    conductor
        .run_until(t_sim_ms(requested_at))
        .expect("steps succeed");
    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, t_sim_ms(25)),
            Box::new(Listener(heard.clone())),
        )
        .expect("25 ms is ahead of either request");
    conductor.run_until(t_sim_ms(60)).expect("steps succeed");
    let seen = heard.lock().expect("inbox log mutex").clone();

    // Return what the newcomer was handed, and the run's fingerprint.
    (seen, conductor.world_hash())
}

#[test]
fn what_a_newcomer_first_sees_does_not_depend_on_when_it_was_asked_for() {
    // Both runs declare the same join at the same instant, so they are the
    // same run and must fingerprint alike. Subscribing a component when its
    // request is taken in rather than when it takes effect would hand the
    // earlier request everything published in between, making the first
    // inbox a function of when the driver got round to asking.
    let (asked_early, hash_early) = run_with_a_late_joiner(0);
    let (asked_late, hash_late) = run_with_a_late_joiner(20);

    assert_eq!(
        asked_early, asked_late,
        "the newcomer was handed a different first inbox for the same join"
    );
    assert_eq!(
        hash_early, hash_late,
        "the same declared join fingerprinted two ways"
    );
}
