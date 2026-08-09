//! A component whose arithmetic goes non-finite halts the run.
//!
//! `continuo-core`'s tests cover what [`StepCtx::publish`] returns. What they
//! cannot show is whether returning it stops anything, which is the only
//! reason the guard is worth having: the alternative is a run that finishes
//! and reports a world hash taken over a payload nothing can read.
//!
//! Halting is safe for the same reason a schedule violation halts. The value
//! is a pure function of the component's logic and the sim state, so it
//! reproduces at the identical instant on every machine, and refusing it
//! cannot be a source of divergence.

use continuo_conductor::{Conductor, ConductorConfig, ConductorError, Pacing, WORLD_LEVEL};
use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, SimDuration, SimTime, StepCtx, Vec3,
};
use continuo_transport::InProcTransport;

/// Integrates a velocity that goes `NaN` after the first step, which is what
/// a divide by zero or a `0.0 * inf` inside a real integrator produces.
struct DivergingIntegrator {
    position: Vec3,
    velocity: f64,
    steps: u32,
    diverges: bool,
}

impl Component for DivergingIntegrator {
    fn id(&self) -> ComponentId {
        ComponentId::new("integrator").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        self.steps += 1;

        // Finite on the first step, so a halt proves the run was under way and
        // stopped, rather than never having started.
        if self.diverges && self.steps > 1 {
            self.velocity = f64::NAN;
        }
        self.position.x += self.velocity;

        ctx.publish(
            KeyExpr::new("test/actor/diverging/pose").expect("valid key"),
            &self.position,
        )?;

        // Return the next due time, 10 ms from now.
        Ok(ctx.now() + SimDuration::from_millis(10))
    }
}

/// Runs ten steps of a world holding one integrator, and reports the verdict.
fn run_world(diverges: bool) -> Result<(), ConductorError> {
    let config = ConductorConfig {
        world_name: "non-finite-test".into(),
        world_seed: 42,
        pacing: Pacing::FreeRun,
    };
    let mut conductor =
        Conductor::new(config, InProcTransport::new()).expect("free-run config is always accepted");
    conductor
        .add_component(
            WORLD_LEVEL,
            Box::new(DivergingIntegrator {
                position: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                velocity: 1.0,
                steps: 0,
                diverges,
            }),
        )
        .expect("registration succeeds");

    // Return whether ten scheduled steps all completed.
    conductor.run_until(SimTime::from_millis(100))
}

#[test]
fn a_component_publishing_a_non_finite_float_halts_the_run() {
    let error = run_world(true).expect_err("a NaN pose must stop the run");

    assert!(
        matches!(error, ConductorError::StepFailed { .. }),
        "a refused publish is a failed step, not something else: {error:?}"
    );

    // The whole value of failing here rather than at some later consumer is
    // that the report says where to look, so each half is asserted: which
    // component and instant from the conductor, which key and field from the
    // core error underneath.
    let message = format!("{error}");
    for expected in ["integrator", "test/actor/diverging/pose", "NaN at x"] {
        assert!(
            message.contains(expected),
            "the report must name {expected:?}: {message}"
        );
    }
}

#[test]
fn the_same_world_runs_to_the_end_when_nothing_goes_non_finite() {
    // Without this the test above would pass on a world that could not run at
    // all, and the guard would be credited for a failure it had no part in.
    run_world(false).expect("finite arithmetic publishes and the run completes");
}
