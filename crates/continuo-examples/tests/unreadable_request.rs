//! A spawner request the handler cannot read halts the scenario.
//!
//! The two decode sites here are not in a component's `step`. They sit in a
//! `MonitorTransport` callback, an `FnMut(&Message)` with nowhere to return an
//! error to, so the `?` that covers every other site cannot reach them.
//!
//! What they get instead is the next place that *does* have somewhere to
//! return to: `TrafficRequestHandler::apply` runs between ticks and already
//! reports to the scenario loop. A request that cannot be read is a car that
//! never arrives or never leaves, which changes the run in silence, so it is
//! worth the same halt as any other unreadable payload.

use continuo_actors::traffic_spawn_key;
use continuo_conductor::{Conductor, WORLD_LEVEL};
use continuo_core::{Component, ComponentId, CoreError, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_examples::traffic_world::{self, TrafficRequestHandler};
use continuo_transport::InProcTransport;

/// Publishes one request the handler cannot decode, on a key it does read.
///
/// A component rather than a direct transport publish, so the message travels
/// the same path a real request takes and reaches the same callback.
struct Saboteur {
    fired: bool,
}

impl Component for Saboteur {
    fn id(&self) -> ComponentId {
        ComponentId::new("saboteur").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        if !self.fired {
            self.fired = true;
            // Valid JSON on the right key, in the wrong shape: what a schema
            // change between a publisher and its reader produces. A string
            // rather than a struct with wrong fields, because it needs no
            // serialization crate of its own and cannot parse as a request
            // whatever that request's fields become.
            ctx.publish(traffic_spawn_key(ctx.world_name()), &"not a request")?;
        }

        // Return the next due time, well inside the run below.
        Ok(ctx.now() + SimDuration::from_millis(100))
    }
}

/// Runs two seconds of the demo, with or without the saboteur in the world.
fn run_demo(sabotaged: bool) -> Result<(), continuo_conductor::ConductorError> {
    let traffic_request_handler = TrafficRequestHandler::default();
    let mut conductor = Conductor::new(
        traffic_world::config(),
        traffic_request_handler.wrap_transport(InProcTransport::new()),
    )
    .expect("free-run config is always accepted");
    traffic_world::setup_live_traffic_scenario(&mut conductor).expect("the world builds");
    if sabotaged {
        conductor
            .add_component(WORLD_LEVEL, Box::new(Saboteur { fired: false }))
            .expect("registration succeeds");
    }

    // Return whether the scenario ran its course.
    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_request_handler,
        SimTime::from_secs(2),
        None,
    )
}

#[test]
fn a_request_that_cannot_be_read_halts_the_scenario() {
    let error = run_demo(true).expect_err("an unreadable spawn request must stop the run");

    let message = format!("{error}");
    assert!(
        message.contains("traffic_spawner/spawn"),
        "the report must name the key the request arrived on: {message}"
    );
    assert!(
        message.contains("saboteur"),
        "and the publisher, since that is who sent something unreadable: {message}"
    );
}

#[test]
fn the_same_two_seconds_run_clean_without_it() {
    // Without this the test above would pass on a demo that could not run at
    // all, and the halt would be credited to a guard that had no part in it.
    run_demo(false).expect("the demo's own requests are readable");
}
