//! Observation hookups accumulate rather than displace one another.
//!
//! The motivating case is recording a run while something else watches it,
//! which a single-slot callback made impossible in a particularly unhelpful
//! way: the second registration silently won and the first observer's channel
//! went quiet, with nothing failing to say so.

use std::sync::{Arc, Mutex};

use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, MembershipChange, Pacing, Recorder, WORLD_LEVEL,
};
use continuo_core::{Component, ComponentId, CoreError, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::InProcTransport;

/// Steps forever at a fixed period, so the run has ticks to observe.
struct Ticker {
    id: &'static str,
    period: SimDuration,
}

impl Component for Ticker {
    fn id(&self) -> ComponentId {
        ComponentId::new(self.id).expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        // Return the next due time, one period out.
        Ok(ctx.now() + self.period)
    }
}

fn ticker(id: &'static str) -> Box<Ticker> {
    Box::new(Ticker {
        id,
        period: SimDuration::from_millis(10),
    })
}

fn config() -> ConductorConfig {
    ConductorConfig {
        world_name: "observers-test".into(),
        world_seed: 0,
        pacing: Pacing::FreeRun,
    }
}

fn new_conductor() -> Conductor<InProcTransport> {
    Conductor::new(config(), InProcTransport::new()).expect("free-run config is always accepted")
}

/// Records into a shared list, so several observers are distinguishable.
fn recording_into(
    log: &Arc<Mutex<Vec<String>>>,
    label: &'static str,
) -> impl FnMut(&MembershipChange) + Send + 'static {
    let log = log.clone();

    // Return an observer that names itself in the shared list.
    move |change: &MembershipChange| {
        let path = match change {
            MembershipChange::Joined(join) => &join.path,
            MembershipChange::Left(leave) => &leave.path,
        };
        log.lock()
            .expect("observer log mutex is never poisoned")
            .push(format!("{label}:{path}"));
    }
}

#[test]
fn every_membership_observer_is_invoked_in_the_order_it_was_added() {
    let seen: Arc<Mutex<Vec<String>>> = Default::default();
    let mut conductor = new_conductor();
    conductor.add_membership_callback(recording_into(&seen, "first"));
    conductor.add_membership_callback(recording_into(&seen, "second"));

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a"))
        .expect("registration succeeds");

    assert_eq!(
        *seen.lock().expect("observer log mutex is never poisoned"),
        vec!["first:a".to_string(), "second:a".to_string()],
        "both observers see the join, and the earlier one goes first"
    );
}

#[test]
fn every_tick_observer_is_invoked() {
    let counts: Arc<Mutex<(u32, u32)>> = Default::default();
    let mut conductor = new_conductor();
    conductor.add_tick_callback({
        let counts = counts.clone();
        move |_| counts.lock().expect("mutex is never poisoned").0 += 1
    });
    conductor.add_tick_callback({
        let counts = counts.clone();
        move |_| counts.lock().expect("mutex is never poisoned").1 += 1
    });

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a"))
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(30))
        .expect("steps succeed");

    let (first, second) = *counts.lock().expect("mutex is never poisoned");
    assert!(first > 0, "the first observer saw ticks");
    assert_eq!(first, second, "both observers saw exactly the same ticks");
}

#[test]
fn a_second_observer_does_not_silence_a_recorder() {
    // The regression this change exists for. Attaching a watcher alongside a
    // recorder used to leave the log missing every membership event, and
    // nothing reported it.
    let recorder = Recorder::new(&config());
    let watched: Arc<Mutex<Vec<String>>> = Default::default();

    let mut conductor = new_conductor();
    conductor.add_membership_callback(recorder.membership_callback());
    conductor.add_membership_callback(recording_into(&watched, "watcher"));

    conductor
        .add_component_at_start(WORLD_LEVEL, ticker("a"))
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(10))
        .expect("steps succeed");
    conductor
        .remove_component_now("a")
        .expect("`a` is registered");

    let log = recorder.finish();
    let membership_events = log
        .events
        .iter()
        .filter(|event| matches!(event, LogEvent::Join(_) | LogEvent::Leave(_)))
        .count();
    assert_eq!(
        membership_events, 2,
        "the recorder still logged the join and the leave despite a second observer"
    );
    assert_eq!(
        watched
            .lock()
            .expect("observer log mutex is never poisoned")
            .len(),
        2,
        "and the watcher saw both as well"
    );
}
