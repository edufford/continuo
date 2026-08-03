//! Per-component step timing (milestone 4): a component's own `step`
//! measured against the wall-clock limits it declared when it joined.
//!
//! These tests contain the only deliberately slow steps in the suite. There
//! is no way around it: the quantity under test is wall time, so overrunning
//! a limit means actually spending it. Sleeping is what makes that reliable:
//! [`std::thread::sleep`] never returns early, so a step told to take 20 ms
//! is always over a 1 ms limit however loaded the machine is. Limits meant
//! *not* to be hit are set absurdly high for the same reason, since the
//! machine can always stall. Steps stay few, and the whole file spends well
//! under a second.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, ConductorError, EventLog, JoinMetadata, MembershipChange,
    OnTimeout, Pacing, RecordedBudgetMiss, RecordedObservation, RecordedTimeout, Recorder,
    StepTiming, Verifier, WORLD_LEVEL,
};
use continuo_core::{
    Component, ComponentId, ComponentPath, KeyExpr, SimDuration, SimTime, StepCtx,
};
use continuo_transport::{InProcTransport, MonitorTransport};

/// Every step every component took, in order.
type StepLog = Arc<Mutex<Vec<(String, SimTime)>>>;

/// A ticker whose steps cost a fixed amount of wall time, so a declared
/// limit can be missed on purpose.
struct SlowTicker {
    id: &'static str,
    period: SimDuration,
    /// Wall time each step spends before returning. Zero for a component
    /// that is only here to be the neighbour that carries on.
    step_cost: Duration,
    steps: StepLog,
}

impl Component for SlowTicker {
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
        if !self.step_cost.is_zero() {
            std::thread::sleep(self.step_cost);
        }

        // Return the next due time, one period out.
        ctx.now() + self.period
    }
}

/// A sim-time instant, in milliseconds.
fn t_sim_ms(millis: i64) -> SimTime {
    SimTime::from_millis(millis)
}

/// Wall-clock milliseconds, the scale step limits are declared at.
fn wall_ms(millis: u64) -> Duration {
    Duration::from_millis(millis)
}

/// A limit with so much headroom that a step doing nothing cannot reach it
/// even on a badly stalled machine, for the cases that must *not* trip.
const GENEROUS: Duration = Duration::from_secs(60);

/// How long a step told to be slow takes, against limits an order of
/// magnitude below it.
const SLOW: Duration = Duration::from_millis(20);

fn timing_config() -> ConductorConfig {
    ConductorConfig {
        world_name: "timing-test".into(),
        world_seed: 0,
        pacing: Pacing::FreeRun,
    }
}

fn new_conductor() -> Conductor<InProcTransport> {
    Conductor::new(timing_config(), InProcTransport::new())
        .expect("free-run config is always accepted")
}

/// A 10 ms ticker whose steps cost `step_cost` of wall time.
fn ticker(id: &'static str, step_cost: Duration, steps: &StepLog) -> Box<SlowTicker> {
    Box::new(SlowTicker {
        id,
        period: SimDuration::from_millis(10),
        step_cost,
        steps: steps.clone(),
    })
}

fn path(s: &str) -> ComponentPath {
    ComponentPath::parse(s).expect("valid path")
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
fn a_step_over_its_budget_is_counted_against_the_component_that_took_it() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL).with_timing(StepTiming::budget(wall_ms(1))),
            ticker("slow", SLOW, &steps),
        )
        .expect("registration succeeds");
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL).with_timing(StepTiming::budget(GENEROUS)),
            ticker("quick", Duration::ZERO, &steps),
        )
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(10)).expect("steps succeed");

    assert_eq!(
        conductor.budget_misses(&path("slow")),
        Some(2),
        "both of its steps ran 20 ms against a 1 ms budget"
    );
    assert_eq!(
        conductor.budget_misses(&path("quick")),
        Some(0),
        "a budget it never came close to reports nothing"
    );
    assert_eq!(
        conductor.budget_misses(&path("nobody")),
        None,
        "and an unregistered path has no count to report"
    );
    assert_eq!(
        steps_of(&steps, "slow"),
        vec![t_sim_ms(0), t_sim_ms(10)],
        "missing a budget is diagnostic: the component keeps stepping"
    );
}

#[test]
fn a_missed_budget_leaves_the_run_identical() {
    // The soft level's whole claim: it observes and nothing else. Same
    // scenario, same slow steps, and the only difference is whether a budget
    // was declared for them to miss.
    fn run(timing: StepTiming, steps: &StepLog) -> u64 {
        let mut conductor = new_conductor();
        conductor
            .add_component(
                JoinMetadata::at_start(WORLD_LEVEL).with_timing(timing),
                ticker("slow", SLOW, steps),
            )
            .expect("registration succeeds");
        conductor.run_until(t_sim_ms(10)).expect("steps succeed");

        // Return the run's fingerprint.
        conductor.world_hash()
    }

    let missed: StepLog = Default::default();
    let undeclared: StepLog = Default::default();
    let with_budget = run(StepTiming::budget(wall_ms(1)), &missed);
    let without = run(StepTiming::unlimited(), &undeclared);

    assert_eq!(
        with_budget, without,
        "a run that misses every budget fingerprints identically to one \
         that declares none"
    );
    assert_eq!(steps_of(&missed, "slow"), steps_of(&undeclared, "slow"));
}

#[test]
fn a_timeout_halts_the_world_when_that_is_the_policy() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL)
                .with_timing(StepTiming::timeout(wall_ms(1), OnTimeout::Halt)),
            ticker("slow", SLOW, &steps),
        )
        .expect("registration succeeds");
    conductor
        .add_component(WORLD_LEVEL, ticker("later_sibling", Duration::ZERO, &steps))
        .expect("registration succeeds");

    let halted = conductor.run_until(t_sim_ms(10));
    let Err(ConductorError::StepTimeout { path, now, .. }) = halted else {
        panic!("expected a timeout, got {halted:?}");
    };
    assert_eq!(path.to_string(), "slow");
    assert_eq!(now, SimTime::ZERO, "it timed out on its very first step");
    assert_eq!(
        steps_of(&steps, "later_sibling"),
        Vec::new(),
        "the halt ends the instant too, so what was due after it never ran"
    );
}

#[test]
fn a_timeout_removes_the_component_at_the_next_instant_when_that_is_the_policy() {
    let steps: StepLog = Default::default();
    let changes: Arc<Mutex<Vec<MembershipChange>>> = Default::default();
    let mut conductor = new_conductor();
    let observed = changes.clone();
    conductor.add_membership_callback(move |change| {
        observed
            .lock()
            .expect("membership log mutex is never poisoned")
            .push(change.clone());
    });

    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL)
                .with_timing(StepTiming::timeout(wall_ms(1), OnTimeout::Remove)),
            ticker("slow", SLOW, &steps),
        )
        .expect("registration succeeds");
    conductor
        .add_component(WORLD_LEVEL, ticker("neighbour", Duration::ZERO, &steps))
        .expect("registration succeeds");
    conductor
        .run_until(t_sim_ms(30))
        .expect("the run continues");

    assert_eq!(
        steps_of(&steps, "slow"),
        vec![SimTime::ZERO],
        "it kept the tick it timed out in, and took no part in any after it"
    );
    assert_eq!(
        steps_of(&steps, "neighbour"),
        vec![SimTime::ZERO, t_sim_ms(10), t_sim_ms(20), t_sim_ms(30)],
        "losing a component is not the world stopping"
    );

    // It goes out as an ordinary leave, the same event a scheduled
    // departure emits, so the recorder writes it to the log without knowing
    // a timeout caused it.
    let changes = changes
        .lock()
        .expect("membership log mutex is never poisoned");
    let leaves: Vec<_> = changes
        .iter()
        .filter_map(|change| match change {
            MembershipChange::Left(leave) => Some(leave),
            MembershipChange::Joined(_) => None,
        })
        .collect();
    assert_eq!(leaves.len(), 1);
    assert_eq!(leaves[0].path, "slow");
    assert_eq!(
        leaves[0].leaves_at,
        SimTime::ZERO + SimDuration::from_nanos(1),
        "half-open at the earliest instant still open when it timed out: \
         the instant it overran is the last it took part in"
    );
}

#[test]
fn a_step_past_both_levels_is_counted_before_the_run_halts() {
    // The two levels are judged separately, so timing out does not swallow
    // the budget miss that came with it. That count is what says the
    // component itself was slow, rather than something between it and the
    // conductor, the distinction the levels exist to draw.
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL).with_timing(
                StepTiming::budget(wall_ms(1)).with_timeout(wall_ms(5), OnTimeout::Halt),
            ),
            ticker("slow", SLOW, &steps),
        )
        .expect("registration succeeds");

    assert!(matches!(
        conductor.run_until(t_sim_ms(10)),
        Err(ConductorError::StepTimeout { .. })
    ));
    assert_eq!(
        conductor.budget_misses(&path("slow")),
        Some(1),
        "the 20 ms step passed the 1 ms budget as well as the 5 ms timeout"
    );
}

/// A world of one budgeted 10 ms ticker whose steps cost `step_cost`, run to
/// 20 ms with every observation point tapped, returning the recorded log.
fn record_a_run_costing(step_cost: Duration) -> EventLog {
    let config = timing_config();
    let steps: StepLog = Default::default();
    let recorder = Recorder::new(&config);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(recorder.tick_callback());
    conductor.add_membership_callback(recorder.membership_callback());
    conductor.add_observation_callback(recorder.observation_callback());
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL).with_timing(StepTiming::budget(wall_ms(1))),
            ticker("slow", step_cost, &steps),
        )
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(20)).expect("steps succeed");

    // Return the recorded log.
    recorder.finish()
}

fn budget_misses_in(log: &EventLog) -> Vec<&RecordedBudgetMiss> {
    log.events
        .iter()
        .filter_map(|event| match event {
            LogEvent::Observed(RecordedObservation::BudgetMissed(miss)) => Some(miss),
            _ => None,
        })
        .collect()
}

fn timeouts_in(log: &EventLog) -> Vec<&RecordedTimeout> {
    log.events
        .iter()
        .filter_map(|event| match event {
            LogEvent::Observed(RecordedObservation::TimedOut(timeout)) => Some(timeout),
            _ => None,
        })
        .collect()
}

#[test]
fn the_log_records_which_steps_missed_their_budget() {
    // Aggregated centrally on purpose: a run's misses are worth having in
    // one file afterwards, not scattered across whichever process each step
    // ran in once components are distributed.
    let log = record_a_run_costing(SLOW);
    let misses = budget_misses_in(&log);

    assert_eq!(misses.len(), 3, "steps at 0, 10 and 20 ms, all over budget");
    assert_eq!(misses[0].path, "slow");
    assert_eq!(misses[0].sim_time, SimTime::ZERO);
    assert!(
        misses[0].step_ms > misses[0].budget_ms,
        "a miss carries both numbers, so the log says by how much"
    );
}

#[test]
fn a_re_run_that_misses_different_budgets_still_verifies() {
    // Why misses are observations and not expectations. They are a fact
    // about the machine, so a faster re-run of the identical scenario
    // legitimately records none, and comparing them would report two runs
    // that behaved identically as divergent.
    let recorded = record_a_run_costing(SLOW);
    let observations = budget_misses_in(&recorded).len();
    let lines = recorded.events.len();
    assert_eq!(observations, 3);

    // Re-run live, fast enough to miss nothing, checking against that log.
    // No observation callback is attached: the verifier has nothing to
    // compare a miss against, and skips the recorded ones as it walks the
    // log.
    let config = timing_config();
    let steps: StepLog = Default::default();
    let verifier = Verifier::new(recorded, &config);
    let transport = MonitorTransport::new(InProcTransport::new(), verifier.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(verifier.tick_callback());
    conductor.add_membership_callback(verifier.membership_callback());
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL).with_timing(StepTiming::budget(wall_ms(1))),
            ticker("slow", Duration::ZERO, &steps),
        )
        .expect("registration succeeds");
    conductor.run_until(t_sim_ms(20)).expect("steps succeed");

    let verified = verifier
        .finish()
        .expect("the run is the same run; only the machine differed");
    assert_eq!(
        verified,
        lines - observations,
        "the verdict counts expectations matched, not log lines walked past"
    );
}

#[test]
fn comparing_two_logs_ignores_their_budget_misses_too() {
    // The same rule on the other reader: log against log, not live.
    let slow = record_a_run_costing(SLOW);
    let quick = record_a_run_costing(Duration::ZERO);
    assert_eq!(budget_misses_in(&quick).len(), 0);

    assert_eq!(
        slow.first_divergence(&quick),
        None,
        "one log carries three misses and the other none, and they are \
         still recordings of the same run"
    );
}

#[test]
fn the_log_says_why_a_timed_out_component_left() {
    // The leave itself is deliberately indistinguishable from a scripted
    // one, so that replaying the run by asking for that leave still matches.
    // The reason rides alongside as an observation.
    let config = timing_config();
    let steps: StepLog = Default::default();
    let recorder = Recorder::new(&config);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
    let mut conductor =
        Conductor::new(config, transport).expect("free-run config is always accepted");
    conductor.add_tick_callback(recorder.tick_callback());
    conductor.add_membership_callback(recorder.membership_callback());
    conductor.add_observation_callback(recorder.observation_callback());
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL)
                .with_timing(StepTiming::timeout(wall_ms(5), OnTimeout::Remove)),
            ticker("slow", SLOW, &steps),
        )
        .expect("registration succeeds");
    conductor
        .add_component(WORLD_LEVEL, ticker("neighbour", Duration::ZERO, &steps))
        .expect("registration succeeds");
    conductor
        .run_until(t_sim_ms(20))
        .expect("the run continues");
    let log = recorder.finish();

    let timeouts = timeouts_in(&log);
    assert_eq!(timeouts.len(), 1);
    assert_eq!(timeouts[0].path, "slow");
    assert_eq!(timeouts[0].sim_time, SimTime::ZERO);
    assert_eq!(timeouts[0].policy, OnTimeout::Remove, "and what it cost it");
    assert!(timeouts[0].waited_ms > timeouts[0].timeout_ms);

    // The leave is recorded too, and carries nothing about the cause.
    let leaves: Vec<_> = log
        .events
        .iter()
        .filter(|event| matches!(event, LogEvent::Leave(leave) if leave.path == "slow"))
        .collect();
    assert_eq!(leaves.len(), 1, "an ordinary leave, at the next instant");
}

#[test]
fn a_budget_that_the_timeout_would_always_beat_is_rejected() {
    let steps: StepLog = Default::default();
    let mut conductor = new_conductor();

    // Above the timeout, so every step slow enough to miss the budget has
    // already timed out. The declaration reads like a warning level but
    // could never report one.
    assert!(matches!(
        conductor.add_component(
            JoinMetadata::at_start(WORLD_LEVEL).with_timing(
                StepTiming::budget(wall_ms(50)).with_timeout(wall_ms(10), OnTimeout::Halt)
            ),
            ticker("misdeclared", Duration::ZERO, &steps),
        ),
        Err(ConductorError::UnreachableStepBudget { .. })
    ));
    // A rejected join leaves nothing behind, timing or otherwise.
    assert_eq!(conductor.budget_misses(&path("misdeclared")), None);

    // The same two limits the right way round are fine.
    conductor
        .add_component(
            JoinMetadata::at_start(WORLD_LEVEL).with_timing(
                StepTiming::budget(wall_ms(10)).with_timeout(wall_ms(50), OnTimeout::Halt),
            ),
            ticker("declared", Duration::ZERO, &steps),
        )
        .expect("a budget below its timeout can report");
}
