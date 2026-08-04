//! The traffic demo with a live viewer attached: the same world as
//! `traffic_realtime`, republished onto Zenoh for something to draw.
//!
//! Paced to 1x real time on purpose. Free-run finishes thirty sim-seconds in
//! well under a second, which is correct and unwatchable.
//!
//! Run with (a short duration, since it runs in real time):
//!   cargo run -p continuo-examples --features viz --example traffic_viz -- 30
//!
//! Then, in another terminal, anything subscribing to
//! `continuo_viz/demo/**` will see it. Poses arrive on
//! `continuo_viz/demo/actor/{name}/pose`, and joins and leaves on
//! `continuo_viz/demo/conductor/membership/status`. Each sample carries the
//! sim time, publisher, and sequence number as a Zenoh attachment, in the
//! same shape a recorded event log holds, so one parser reads both.
//!
//! For the same number of sim-seconds, the world hash printed at the end is
//! the one `traffic_realtime` prints without a viewer attached. That is the
//! point, and it is the end-to-end form of what `hash_neutrality.rs` pins:
//! watching a run cannot change it. (The hash covers the run so far, so
//! comparing two runs means giving them the same duration.)

use continuo_conductor::{Conductor, Pacing};
use continuo_core::SimTime;
use continuo_examples::traffic_world::{self, TrafficRequestHandler};
use continuo_transport::InProcTransport;
use continuo_viz_bridge::{VizBridge, ZenohSink};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_target(false)
        .init();

    let seconds: i64 = std::env::args()
        .nth(1)
        .map(|a| a.parse())
        .transpose()?
        .unwrap_or(30);

    let config = traffic_world::config_paced(Pacing::real_time());
    let zenoh_sink = ZenohSink::new()?;
    let viz_bridge = VizBridge::new(&config, zenoh_sink);

    // Two observers on the transport: the request handler the scenario needs
    // in order to run at all, wrapped by the bridge that watches it.
    let traffic_request_handler = TrafficRequestHandler::default();
    let transport =
        viz_bridge.wrap_transport(traffic_request_handler.wrap_transport(InProcTransport::new()));

    let mut conductor = Conductor::new(config, transport)?;
    conductor.add_membership_callback(viz_bridge.membership_callback());
    traffic_world::setup_live_traffic_scenario(&mut conductor)?;

    println!(
        "publishing to continuo_viz/{}/** ; start a viewer to watch",
        conductor.world_name()
    );

    // Wall time is fine to read here: the example's main is outside the
    // simulation, where wall clocks are forbidden.
    let started = std::time::Instant::now();
    traffic_world::run_live_traffic_scenario(
        &mut conductor,
        &traffic_request_handler,
        SimTime::from_secs(seconds),
        None,
    )?;
    let elapsed = started.elapsed();

    println!(
        "done: world '{}' reached sim time {} in {} ticks (1x real-time)",
        conductor.world_name(),
        conductor.sim_time(),
        conductor.tick(),
    );
    println!(
        "actual time: {:.3} s for {} sim-seconds, world hash {:016x}",
        elapsed.as_secs_f64(),
        seconds,
        conductor.world_hash()
    );

    // Frames are dropped rather than queued without bound when a viewer
    // cannot keep up, so this is the number that says whether it did.
    let dropped_viz_frames = viz_bridge.dropped_frames();
    if dropped_viz_frames > 0 {
        println!("viewer: {dropped_viz_frames} frames dropped; the viewer was not keeping up");
    } else {
        println!("viewer: no frames dropped");
    }
    viz_bridge.finish();

    // Return success for the completed run.
    Ok(())
}
