//! A component whose arithmetic goes non-finite halts the run.
//!
//! `continuo-core`'s tests already cover what [`StepCtx::publish`] returns.
//! What they cannot show is whether returning it stops anything: components
//! call `publish` and unwrap, so the question is what a real run does when one
//! of them produces a `NaN`. The answer has to be "stops at that component",
//! because the alternative is a run that finishes and reports a world hash
//! taken over a payload nothing can read.
//!
//! Halting is safe here for the same reason a schedule violation halts. The
//! value is a pure function of the component's logic and the sim state, so it
//! reproduces at the identical instant on every machine, and refusing it
//! cannot be a source of divergence.

use continuo_conductor::{Conductor, ConductorConfig, Pacing, WORLD_LEVEL};
use continuo_core::{Component, ComponentId, KeyExpr, SimDuration, SimTime, StepCtx, Vec3};
use continuo_transport::InProcTransport;

/// Integrates a velocity that is `NaN` from the second step onward, which is
/// what a divide by zero or a `0.0 * inf` inside a real integrator produces.
struct DivergingIntegrator {
    position: Vec3,
    velocity: f64,
    steps: u32,
}

impl Component for DivergingIntegrator {
    fn id(&self) -> ComponentId {
        ComponentId::new("integrator").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        self.steps += 1;

        // Finite on the first step, so the test proves the run was under way
        // and stopped, rather than never having started.
        if self.steps > 1 {
            self.velocity = f64::NAN;
        }
        self.position.x += self.velocity;

        ctx.publish(
            KeyExpr::new("test/actor/diverging/pose").expect("valid key"),
            &self.position,
        )
        .expect("the integrator keeps its position finite");

        // Return the next due time, 10 ms from now.
        ctx.now() + SimDuration::from_millis(10)
    }
}

fn run_diverging_world() {
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
            }),
        )
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(100))
        .expect("components schedule strictly forward");
}

#[test]
#[should_panic(expected = "the integrator keeps its position finite")]
fn a_component_publishing_a_non_finite_float_halts_the_run() {
    // Ten steps are scheduled and the second one goes bad, so a run that
    // reached the end would mean the guard let it through.
    run_diverging_world();
}

#[test]
fn the_panic_names_the_field_that_went_non_finite() {
    // The message is the whole value of failing here rather than at some
    // consumer later, so it is worth asserting rather than assuming. Caught
    // rather than expected, since `should_panic` matches one substring and
    // both halves matter: which key, and which field within it.
    let panic = std::panic::catch_unwind(run_diverging_world).expect_err("the run must panic");
    let message = panic
        .downcast_ref::<String>()
        .map(String::as_str)
        .or_else(|| panic.downcast_ref::<&str>().copied())
        .expect("a panic payload that can be read");

    assert!(
        message.contains("test/actor/diverging/pose"),
        "the key says which publish failed: {message}"
    );
    assert!(
        message.contains("NaN at x"),
        "and the path says which field went bad: {message}"
    );
}
