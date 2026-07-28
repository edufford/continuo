//! Runtime membership (milestone 4): components leaving a running world.
//!
//! Joining mid-run, recording membership changes in the event log, and the
//! timeout policy that drops a component arrive in the later sections of
//! this milestone.

use std::sync::{Arc, Mutex};

use continuo_conductor::{Conductor, ConductorConfig, ConductorError, Pacing};
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
