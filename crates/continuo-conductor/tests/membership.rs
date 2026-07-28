//! Runtime membership (milestone 4): components joining and leaving a
//! running world.
//!
//! Recording membership changes in the event log, and the timeout policy
//! that drops a component, arrive in the later sections of this milestone.

use std::sync::{Arc, Mutex};

use continuo_conductor::{Conductor, ConductorConfig, ConductorError, JoinMetadata, Pacing};
use continuo_core::{Component, ComponentId, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::InProcTransport;

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

fn new_conductor() -> Conductor<InProcTransport> {
    Conductor::new(
        ConductorConfig {
            world_name: "membership-test".into(),
            world_seed: 0,
            pacing: Pacing::FreeRun,
        },
        InProcTransport::new(),
    )
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
