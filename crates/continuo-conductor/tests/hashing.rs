//! Determinism-harness semantics: seeded RNG streams, tick fingerprints, and the
//! state-hash vs. output-hash distinction.

use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, EventLog, PlaybackComponent, Recorder, Verifier,
};
use continuo_core::{Component, ComponentId, DetRng, KeyExpr, SimDuration, SimTime, StepCtx};
use continuo_transport::{InProcTransport, MonitorTransport};

/// Publishes one deterministic random value per step from a persistent
/// per-component stream.
struct NoiseSource {
    id: &'static str,
    rng: Option<DetRng>,
}

impl Component for NoiseSource {
    fn id(&self) -> ComponentId {
        ComponentId::new(self.id).expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        let rng = self
            .rng
            .get_or_insert_with(|| DetRng::new(ctx.component_seed()));
        let value = rng.next_f64();
        ctx.publish(KeyExpr::new("test/noise").expect("valid key"), &value)
            .expect("f64 serializes");

        // Return the next due time, 10 ms from now.
        ctx.now() + SimDuration::from_millis(10)
    }
}

/// A component with hidden internal state and *no* outputs: only the
/// `state_bytes` hook can make it visible to the determinism check.
struct HiddenCounter {
    count: u64,
    expose_state: bool,
}

impl Component for HiddenCounter {
    fn id(&self) -> ComponentId {
        ComponentId::new("hidden").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        self.count += 1;

        // Return the next due time, 10 ms from now.
        ctx.now() + SimDuration::from_millis(10)
    }

    fn state_bytes(&self) -> Option<Vec<u8>> {
        self.expose_state
            .then(|| serde_json::to_vec(&self.count).expect("count serializes"))
    }
}

fn run_noise_world(seed: u64) -> EventLog {
    let recorder = Recorder::new("hashing-test", seed);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "hashing-test".into(),
            seed,
            real_time_pacing: false,
        },
        transport,
    )
    .expect("free-run config is always accepted");
    conductor.set_tick_callback(recorder.tick_callback());

    conductor
        .add_component(
            "",
            Box::new(NoiseSource {
                id: "noise",
                rng: None,
            }),
        )
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(100))
        .expect("components schedule strictly forward");

    // Return the recorded run for comparison.
    recorder.finish()
}

#[test]
fn same_seed_produces_identical_logs() {
    let a = run_noise_world(42);
    let b = run_noise_world(42);
    assert!(!a.events.is_empty());
    assert_eq!(a.first_divergence(&b), None);
}

#[test]
fn different_seed_diverges_from_the_first_tick() {
    let a = run_noise_world(42);
    let b = run_noise_world(43);
    let divergence = a.first_divergence(&b).expect("seeds must diverge");
    // Headers differ (seed is recorded), which is the earliest possible
    // divergence point.
    assert_eq!(divergence.event_index, None);

    // Beyond the header, the actual streams differ too: compare final
    // hashes directly.
    assert_ne!(a.final_world_hash(), b.final_world_hash());
}

fn run_hidden_world(initial: u64, expose_state: bool) -> u64 {
    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "hidden-test".into(),
            seed: 0,
            real_time_pacing: false,
        },
        InProcTransport::new(),
    )
    .expect("free-run config is always accepted");
    conductor
        .add_component(
            "",
            Box::new(HiddenCounter {
                count: initial,
                expose_state,
            }),
        )
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(50))
        .expect("components schedule strictly forward");

    // Return the run's final world hash.
    conductor.world_hash()
}

#[test]
fn state_bytes_exposes_hidden_state_to_the_hash() {
    // Without the hook, two worlds with different hidden state are
    // indistinguishable (no outputs to hash)...
    assert_eq!(run_hidden_world(0, false), run_hidden_world(1000, false));
    // ...with it, the divergence is caught.
    assert_ne!(run_hidden_world(0, true), run_hidden_world(1000, true));
    // And it stays deterministic: same hidden state, same hash.
    assert_eq!(run_hidden_world(7, true), run_hidden_world(7, true));
}

#[test]
fn live_verification_stops_at_the_first_divergence() {
    let mut expected = run_noise_world(42);
    let total_events = expected.events.len();
    assert_eq!(total_events, 22, "11 steps, one message + one tick each");

    // Tamper with a mid-log message payload (event 8 = the 5th step's
    // message, at sim time 40 ms of 100).
    let LogEvent::Msg(message) = &mut expected.events[8] else {
        panic!("event 8 is expected to be a message");
    };
    message.payload =
        serde_json::value::RawValue::from_string("0.5".to_string()).expect("valid JSON");

    // Re-run the same world against the tampered log, stopping on
    // divergence.
    let checker = Verifier::new(expected, "hashing-test", 42);
    let transport = MonitorTransport::new(InProcTransport::new(), checker.message_callback());
    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "hashing-test".into(),
            seed: 42,
            real_time_pacing: false,
        },
        transport,
    )
    .expect("free-run config is always accepted");
    conductor.set_tick_callback(checker.tick_callback());
    conductor
        .add_component(
            "",
            Box::new(NoiseSource {
                id: "noise",
                rng: None,
            }),
        )
        .expect("registration succeeds");

    let end = SimTime::from_millis(100);
    while !checker.diverged() && conductor.next_scheduled().is_some_and(|t| t <= end) {
        conductor.step_once().expect("steps succeed");
    }

    let divergence = checker.finish().expect_err("tampered log must diverge");
    assert_eq!(divergence.event_index, Some(8));
    // The run stopped early: 5 of 11 steps executed, not the full schedule.
    assert_eq!(conductor.tick(), 5, "stopped at the diverging step");
    assert!(conductor.sim_time() < end);
}

#[test]
fn playback_double_reproduces_the_recorded_messages() {
    let original = run_noise_world(42);

    // Rebuild the world with the noise source replaced by its playback
    // double, recording what the double publishes.
    let recorder = Recorder::new("hashing-test", 42);
    let transport = MonitorTransport::new(InProcTransport::new(), recorder.message_callback());
    let mut conductor = Conductor::new(
        ConductorConfig {
            world: "hashing-test".into(),
            seed: 42,
            real_time_pacing: false,
        },
        transport,
    )
    .expect("free-run config is always accepted");
    conductor.set_tick_callback(recorder.tick_callback());
    conductor
        .add_component(
            "",
            Box::new(PlaybackComponent::from_log(
                ComponentId::new("noise").expect("valid id"),
                &original,
                "noise",
            )),
        )
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(100))
        .expect("playback schedules strictly forward");
    let replayed = recorder.finish();

    // The doubles' messages must be indistinguishable from the originals:
    // same times, keys, sequence numbers, and byte-identical payloads.
    let messages = |log: &EventLog| -> Vec<(String, String, u64, String)> {
        log.events
            .iter()
            .filter_map(|e| match e {
                LogEvent::Msg(m) => Some((
                    m.time.to_canonical_string(),
                    m.key.clone(),
                    m.seq,
                    m.payload.get().to_string(),
                )),
                _ => None,
            })
            .collect()
    };
    assert_eq!(messages(&original), messages(&replayed));
    assert!(!replayed.events.is_empty());
}

#[test]
fn per_step_rng_is_reproducible_and_time_dependent() {
    // StepCtx::rng is pure over (component_seed, now): identical inputs
    // give identical streams, different times give different streams.
    let mut a = StepCtx::new(SimTime::from_millis(10), None, "w", 99, Vec::new());
    let mut b = StepCtx::new(SimTime::from_millis(10), None, "w", 99, Vec::new());
    assert_eq!(a.rng().next_u64(), b.rng().next_u64());

    let mut later = StepCtx::new(SimTime::from_millis(20), None, "w", 99, Vec::new());
    assert_ne!(a.rng().next_u64(), later.rng().next_u64());

    // Silence unused-mut lints via harmless use.
    let _ = (a.take_outbox(), b.take_outbox(), later.take_outbox());
}
