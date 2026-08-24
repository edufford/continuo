//! A component that cannot read a message it subscribed to halts the run.
//!
//! The failure this closes is invisible to everything else the project has.
//! A swallowed decode is deterministic: the same wrong answer comes out of
//! every run, so the world hash holds steady and verification passes against a
//! recording carrying the identical fault. The determinism machinery catches
//! divergence, and this never diverges.
//!
//! Halting is safe for the same reason a schedule violation halts. Whether a
//! payload parses is a pure function of the bytes and the type, so it
//! reproduces at the identical instant on every machine.

use continuo_conductor::{Conductor, ConductorConfig, ConductorError, Pacing, WORLD_LEVEL};
use continuo_core::{Component, ComponentId, CoreError, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::InProcTransport;
use serde::{Deserialize, Serialize};

const KEY: &str = "test/actor/sensor/reading";

#[derive(Serialize)]
struct Reading {
    distance: f64,
}

/// What the reader expects. Incompatible with [`Reading`] on purpose: the
/// field is named differently, so a payload from the publisher below cannot
/// parse as this.
#[derive(Deserialize)]
struct Expected {
    #[allow(dead_code)]
    bearing: f64,
}

/// Publishes one reading per step, in the shape chosen at construction.
struct Sensor {
    matching: bool,
}

impl Component for Sensor {
    fn id(&self) -> ComponentId {
        ComponentId::new("sensor").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        let key = KeyExpr::new(KEY).expect("valid key");
        if self.matching {
            ctx.publish(key, &serde_json::json!({ "bearing": 1.5 }))?;
        } else {
            ctx.publish(key, &Reading { distance: 1.5 })?;
        }

        // Return the next due time, 10 ms from now.
        Ok(ctx.now() + SimDuration::from_millis(10))
    }
}

/// Reads whatever the sensor published, and how it handles a payload it
/// cannot parse is what each test below is about.
struct Reader {
    /// Whether to swallow a failure rather than propagate it, which is the
    /// documented way for a component to opt out of halting.
    tolerant: bool,
    read: u32,
    refused: u32,
}

impl Component for Reader {
    fn id(&self) -> ComponentId {
        ComponentId::new("reader").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        vec![KeyExpr::new(KEY).expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        for message in ctx.inbox() {
            match message.decode::<Expected>() {
                Ok(_) => self.read += 1,
                Err(_) if self.tolerant => self.refused += 1,
                Err(error) => return Err(error),
            }
        }

        // Return the next due time, 10 ms from now.
        Ok(ctx.now() + SimDuration::from_millis(10))
    }
}

/// Runs a sensor and a reader together for ten steps.
fn run_world(matching: bool, tolerant: bool) -> Result<(), ConductorError> {
    let config = ConductorConfig {
        world_name: "decode-test".into(),
        world_seed: 42,
        pacing: Pacing::FreeRun,
    };
    let mut conductor =
        Conductor::new(config, InProcTransport::new()).expect("free-run config is always accepted");
    conductor
        .add_component_at_start(WORLD_LEVEL, Box::new(Sensor { matching }))
        .expect("registration succeeds");
    conductor
        .add_component_at_start(
            WORLD_LEVEL,
            Box::new(Reader {
                tolerant,
                read: 0,
                refused: 0,
            }),
        )
        .expect("registration succeeds");

    // Return whether every scheduled step completed.
    conductor.run_until(SimTime::from_millis(100))
}

#[test]
fn a_component_that_cannot_read_its_inbox_halts_the_run() {
    let error = run_world(false, false).expect_err("an unreadable payload must stop the run");

    assert!(
        matches!(error, ConductorError::StepFailed { .. }),
        "an unreadable payload is a failed step: {error:?}"
    );

    // Which component and instant come from the conductor, which key and
    // publisher from the core error underneath. The publisher matters most:
    // the component that fails to decode is not the one at fault.
    let message = format!("{error}");
    for expected in ["reader", KEY, "sensor"] {
        assert!(
            message.contains(expected),
            "the report must name {expected:?}: {message}"
        );
    }
}

#[test]
fn the_same_world_runs_to_the_end_when_the_payload_parses() {
    // Without this the test above would pass on a world that could not run at
    // all, and the halt would be credited to a guard that had no part in it.
    run_world(true, false).expect("a payload the reader understands lets the run finish");
}

#[test]
fn a_component_may_still_choose_to_carry_on() {
    // Swallowing stays available, which is the point of `decode` returning a
    // `Result` rather than halting by itself. A component that does it has to
    // say so, in a way anyone reading the step can see.
    run_world(false, true).expect("a reader that handles the error keeps the run alive");
}
