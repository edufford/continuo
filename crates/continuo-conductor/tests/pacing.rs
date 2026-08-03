//! Real-time pacing, end to end through the conductor (milestone 3). The
//! anchor-and-slip arithmetic is unit-tested against a manual clock in
//! `src/pacing.rs`; here the real clock runs, so these check the two
//! properties that matter at the conductor boundary:
//!
//! 1. pacing changes *timing only*, so a paced run and a free run of the same
//!    seeded world produce the identical world hash, and
//! 2. a paced run actually spends wall time (it does not free-run).

use std::time::Instant;

use continuo_conductor::{Conductor, ConductorConfig, ConductorError, Pacing, WORLD_LEVEL};
use continuo_core::{Component, ComponentId, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::InProcTransport;

/// A bare periodic component: steps every `period`, publishes one value, no
/// inbox. Enough to make the schedule advance so pacing has instants to
/// gate.
struct Ticker {
    period: SimDuration,
}

impl Component for Ticker {
    fn id(&self) -> ComponentId {
        ComponentId::new("ticker").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        ctx.publish(
            KeyExpr::new("test/tick").expect("valid key"),
            &ctx.now().as_nanos(),
        )
        .expect("i64 serializes");

        // Return the next due time, one period out.
        ctx.now() + self.period
    }
}

fn run(pacing: Pacing, end: SimTime) -> Conductor<InProcTransport> {
    let mut conductor = Conductor::new(
        ConductorConfig {
            world_name: "pacing-test".into(),
            world_seed: 1,
            pacing,
        },
        InProcTransport::new(),
    )
    .expect("config is accepted");
    conductor
        .add_component(
            WORLD_LEVEL,
            Box::new(Ticker {
                period: SimDuration::from_millis(10),
            }),
        )
        .expect("registration succeeds");
    conductor.run_until(end).expect("ticker schedules forward");

    // Return the finished conductor for inspection.
    conductor
}

#[test]
fn pacing_does_not_change_the_world_hash() {
    let end = SimTime::from_millis(50);
    let free = run(Pacing::FreeRun, end);
    // Both real-time modes, OS-timer and sleep-then-spin, must match the
    // free run exactly: pacing only delays instants, never changes content.
    for paced in [
        run(Pacing::real_time(), end),
        run(Pacing::real_time_precise(), end),
    ] {
        assert_eq!(
            free.world_hash(),
            paced.world_hash(),
            "pacing must only delay instants, never change their content"
        );
        assert_eq!(free.tick(), paced.tick(), "same schedule, same tick count");
    }
}

#[test]
fn a_paced_run_spends_real_wall_time() {
    // 200 ms of sim at 1x should take on the order of 200 ms of wall time,
    // certainly not the sub-millisecond a free run of this tiny world takes.
    // The lower bound is generous to stay robust on any machine; the point
    // is that it did not free-run.
    let start = Instant::now();
    let conductor = run(Pacing::real_time(), SimTime::from_millis(200));
    let elapsed = start.elapsed();

    assert_eq!(conductor.sim_time(), SimTime::from_millis(200));
    assert!(
        elapsed.as_millis() >= 150,
        "paced run finished in {elapsed:?}, far under 1x real time"
    );
}

#[test]
fn a_free_run_reports_no_overruns() -> Result<(), ConductorError> {
    let conductor = run(Pacing::FreeRun, SimTime::from_millis(50));
    assert_eq!(conductor.overrun_reanchor_count(), 0);
    assert_eq!(conductor.total_slip(), std::time::Duration::ZERO);

    // Return success; free-run never paces, so nothing can slip.
    Ok(())
}
