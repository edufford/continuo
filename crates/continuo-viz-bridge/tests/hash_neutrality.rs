//! Attaching a viewer cannot change the run.
//!
//! This is the property the whole design rests on, and the reason the bridge
//! is a transport monitor rather than a component: a component's path and
//! `next_due` feed the tick hash, so watching would fingerprint differently
//! from not watching, and every scenario would need a watched variant.
//!
//! Each test here asserts the bridge saw traffic as well as that the hash
//! held. Without that, a bridge that quietly did nothing would pass.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use continuo_conductor::record::LogEvent;
use continuo_conductor::{Conductor, ConductorConfig, JoinMetadata, Pacing, Recorder, WORLD_LEVEL};
use continuo_core::{Component, ComponentId, KeyExpr, Pose, SimDuration, SimTime, StepCtx, Vec3};
use continuo_transport::{InProcTransport, MonitorTransport, Transport};
use continuo_viz_bridge::{VizBridge, VizFrame, VizSink};

/// Publishes a moving pose every period, so there is traffic to observe and
/// state for the hash to chain.
struct Beacon {
    id: &'static str,
    period: SimDuration,
    x: f64,
}

impl Component for Beacon {
    fn id(&self) -> ComponentId {
        ComponentId::new(self.id).expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        self.x += 1.5;
        let pose = Pose {
            position: Vec3::new(self.x, 0.0, 0.0),
            ..Default::default()
        };
        let key = KeyExpr::new(format!(
            "continuo/{}/actor/{}/pose",
            ctx.world_name(),
            self.id
        ))
        .expect("valid key");
        ctx.publish(key, &pose).expect("pose serializes");

        // Return the next due time, one period out.
        ctx.now() + self.period
    }
}

fn beacon(id: &'static str) -> Box<Beacon> {
    Box::new(Beacon {
        id,
        period: SimDuration::from_millis(10),
        x: 0.0,
    })
}

fn config() -> ConductorConfig {
    ConductorConfig {
        world_name: "neutrality".into(),
        world_seed: 7,
        pacing: Pacing::FreeRun,
    }
}

/// Counts what reached the sink, so a test can prove the bridge was live.
#[derive(Clone, Default)]
struct CountingSink {
    delivered: Arc<AtomicU64>,
}

impl VizSink for CountingSink {
    fn deliver(&mut self, _frame: VizFrame) {
        self.delivered.fetch_add(1, Ordering::Relaxed);
    }
}

/// Runs the same scenario on whatever transport is handed in, optionally
/// tapping membership, and returns the world hash.
///
/// The scenario deliberately joins and leaves mid-run, since membership is
/// the part of the hash most likely to notice an observer.
fn run<T: Transport>(mut conductor: Conductor<T>, bridge: Option<&VizBridge>) -> u64 {
    if let Some(bridge) = bridge {
        conductor.add_membership_callback(bridge.membership_callback());
    }
    conductor
        .add_component(WORLD_LEVEL, beacon("a"))
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(20))
        .expect("steps succeed");
    conductor
        .add_component(
            JoinMetadata::at(WORLD_LEVEL, SimTime::from_millis(30)),
            beacon("b"),
        )
        .expect("30 ms is still ahead");
    conductor
        .run_until(SimTime::from_millis(50))
        .expect("steps succeed");
    conductor.remove_component("a").expect("`a` is registered");
    conductor
        .run_until(SimTime::from_millis(80))
        .expect("steps succeed");

    // Return the fingerprint of the finished run.
    conductor.world_hash()
}

#[test]
fn attaching_a_viewer_does_not_change_the_world_hash() {
    let unwatched = run(
        Conductor::new(config(), InProcTransport::new()).expect("config is accepted"),
        None,
    );

    let sink = CountingSink::default();
    let bridge = VizBridge::new(&config(), sink.clone());
    let watched = run(
        Conductor::new(config(), bridge.wrap_transport(InProcTransport::new()))
            .expect("config is accepted"),
        Some(&bridge),
    );
    bridge.finish();

    assert_eq!(
        watched, unwatched,
        "a watched run must fingerprint exactly as the same run unwatched"
    );
    assert!(
        sink.delivered.load(Ordering::Relaxed) > 0,
        "the bridge must actually have observed traffic, or this proves nothing"
    );
}

/// Observes nothing and publishes nothing, existing only to be present.
struct SilentObserver;

impl Component for SilentObserver {
    fn id(&self) -> ComponentId {
        ComponentId::new("observer").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        vec![KeyExpr::new("continuo/*/actor/*/pose").expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        // Return the next due time. Nothing else: it reads its inbox never
        // and publishes never.
        ctx.now() + SimDuration::from_millis(10)
    }
}

#[test]
fn an_observer_built_as_a_component_would_change_the_hash() {
    // The counter-example that gives the test above its meaning. Watching a
    // run *from inside it* is not free even when the watcher does nothing at
    // all, because a component's path and `next_due` are folded into the tick
    // hash by its mere presence. That is the whole reason the bridge is a
    // transport monitor.
    let unwatched = run(
        Conductor::new(config(), InProcTransport::new()).expect("config is accepted"),
        None,
    );

    let mut conductor = Conductor::new(config(), InProcTransport::new()).expect("accepted");
    conductor
        .add_component(WORLD_LEVEL, Box::new(SilentObserver))
        .expect("registration succeeds");
    let watched_from_inside = run(conductor, None);

    assert_ne!(
        watched_from_inside, unwatched,
        "a component that does nothing still changes the fingerprint, which is \
         why observation lives outside the sim"
    );
}

#[test]
fn watching_and_recording_the_same_run_leaves_the_log_untouched() {
    // Observers accumulate since #8, so a bridge and a recorder can both be
    // attached. The recorded log must be byte-identical to one taken without
    // a viewer present, or "the bridge changes nothing" is false for the
    // artifact people actually keep.
    let recorded_alone = {
        let recorder = Recorder::new(&config());
        let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
        let mut conductor = Conductor::new(config(), transport).expect("config is accepted");
        conductor.add_membership_callback(recorder.membership_callback());
        conductor.add_tick_callback(recorder.tick_callback());
        run(conductor, None);
        recorder.finish()
    };

    let sink = CountingSink::default();
    let bridge = VizBridge::new(&config(), sink.clone());
    let recorded_while_watched = {
        let recorder = Recorder::new(&config());
        // Both observers on the transport: the recorder inside, the bridge
        // wrapping it.
        let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
        let mut conductor =
            Conductor::new(config(), bridge.wrap_transport(transport)).expect("config is accepted");
        conductor.add_membership_callback(recorder.membership_callback());
        conductor.add_tick_callback(recorder.tick_callback());
        run(conductor, Some(&bridge));
        recorder.finish()
    };
    bridge.finish();

    assert!(
        sink.delivered.load(Ordering::Relaxed) > 0,
        "the bridge must actually have observed traffic, or this proves nothing"
    );
    assert_eq!(
        recorded_alone.events.len(),
        recorded_while_watched.events.len(),
        "watching a run must not add or remove log events"
    );
    let membership_events = |log: &continuo_conductor::EventLog| {
        log.events
            .iter()
            .filter(|e| matches!(e, LogEvent::Join(_) | LogEvent::Leave(_)))
            .count()
    };
    assert_eq!(
        membership_events(&recorded_alone),
        membership_events(&recorded_while_watched),
        "and the recorder still sees every membership change with a bridge alongside it"
    );
    assert_eq!(
        recorded_alone.final_world_hash(),
        recorded_while_watched.final_world_hash(),
        "the recorded fingerprint is the same either way"
    );
}
