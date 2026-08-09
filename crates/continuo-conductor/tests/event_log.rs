//! Event-log semantics across the three modules that share it: `record`
//! writes a log, `verify` compares against one, `playback` feeds one back
//! into a sim. They are tested together because they are tested against
//! the same fixture log.

use continuo_conductor::record::{LogEvent, LogHeader};
use continuo_conductor::{
    Conductor, ConductorConfig, EventLog, Pacing, PlaybackComponent, RecordedBudgetMiss,
    RecordedObservation, Recorder, TickFingerprint, Verifier, WORLD_LEVEL,
};
use continuo_core::{Component, ComponentId, ComponentPath, KeyExpr, Message, SimTime, StepCtx};
use continuo_transport::InProcTransport;

const WORLD_NAME: &str = "test";
const WORLD_SEED: u64 = 7;
const KEY: &str = "w/a";
const PUBLISHER: &str = "p";
const PAYLOAD: &[u8] = br#"{"v":1.5}"#;
const TICK_HASH: u64 = 0xdead_beef;
const WORLD_HASH: u64 = 0x1234_5678_9abc_def0;

/// The config the fixture log claims to have been recorded from.
fn sample_config() -> ConductorConfig {
    ConductorConfig {
        world_name: WORLD_NAME.to_string(),
        world_seed: WORLD_SEED,
        pacing: Pacing::FreeRun,
    }
}

fn sample_message() -> Message {
    Message {
        key: KeyExpr::new(KEY).unwrap(),
        publisher: ComponentPath::parse(PUBLISHER).unwrap(),
        seq: 0,
        sim_time: SimTime::ZERO,
        payload: PAYLOAD.to_vec(),
    }
}

fn sample_fingerprint() -> TickFingerprint {
    TickFingerprint {
        tick: 1,
        sim_time: SimTime::ZERO,
        tick_hash: TICK_HASH,
        world_hash: WORLD_HASH,
    }
}

/// Three JSON lines: the `("test", seed 7)` header, then the two events of
/// a single tick at t=0: one message (`sample_message`, published on `w/a`
/// by `p`) followed by that tick's fingerprint (`sample_fingerprint`). A
/// verifier fed exactly those two events in that order must agree with it.
fn sample_log() -> EventLog {
    let recorder = Recorder::new(&sample_config());
    let mut msg_callback = recorder.message_callback();
    let mut tick_callback = recorder.tick_callback();
    msg_callback(&sample_message());
    tick_callback(&sample_fingerprint());

    // Return the collected sample log.
    recorder.finish()
}

// --- record ---------------------------------------------------------------

#[test]
fn jsonl_round_trip() {
    let log = sample_log();
    let text = log.to_jsonl();
    assert_eq!(text.lines().count(), 3);
    assert!(text.lines().nth(1).unwrap().contains(r#""v":1.5"#));
    let back = EventLog::from_jsonl(&text).unwrap();
    assert!(log.first_divergence(&back).is_none());
    assert_eq!(back.final_world_hash(), Some(WORLD_HASH));
}

#[test]
fn the_header_names_the_run_that_produced_the_log() {
    assert_eq!(
        sample_log().header,
        LogHeader {
            version: 1,
            world_name: WORLD_NAME.to_string(),
            world_seed: WORLD_SEED,
        }
    );
}

// --- verify ---------------------------------------------------------------

#[test]
fn two_logs_diverge_at_the_first_differing_event() {
    let a = sample_log();
    let mut b = sample_log();
    assert!(a.first_divergence(&b).is_none());

    if let LogEvent::Tick(fingerprint) = &mut b.events[1] {
        fingerprint.world_hash ^= 1;
    }
    let divergence = a.first_divergence(&b).expect("must diverge");
    assert_eq!(divergence.event_index, Some(1));

    let different_seed = ConductorConfig {
        world_seed: WORLD_SEED + 1,
        ..sample_config()
    };
    let c = Recorder::new(&different_seed).finish();
    assert!(
        a.first_divergence(&c)
            .expect("header differs")
            .event_index
            .is_none()
    );
}

#[test]
fn the_verifier_accepts_a_matching_stream() {
    let verifier = Verifier::new(sample_log(), &sample_config());
    verifier.message_callback()(&sample_message());
    verifier.tick_callback()(&sample_fingerprint());
    assert!(!verifier.diverged());
    assert_eq!(verifier.finish().expect("streams match"), 2);
}

#[test]
fn the_verifier_flags_the_first_mismatching_event() {
    let mut expected = sample_log();
    if let LogEvent::Tick(fingerprint) = &mut expected.events[1] {
        fingerprint.world_hash ^= 1;
    }
    let verifier = Verifier::new(expected, &sample_config());
    verifier.message_callback()(&sample_message());
    assert!(!verifier.diverged(), "message still matches");
    verifier.tick_callback()(&sample_fingerprint());
    assert!(verifier.diverged(), "fingerprint mismatch must be caught");
    let divergence = verifier.finish().expect_err("must diverge");
    assert_eq!(divergence.event_index, Some(1));
}

#[test]
fn the_verifier_flags_a_truncated_rerun() {
    // A re-run that stops early cannot be caught as it happens: `check`
    // only runs when the re-run produces something, and producing nothing
    // is not an event. `finish` is what catches it, which is why calling
    // it is not optional.
    let verifier = Verifier::new(sample_log(), &sample_config());
    verifier.message_callback()(&sample_message());
    assert!(!verifier.diverged(), "nothing has gone wrong *yet*");

    let divergence = verifier.finish().expect_err("must diverge");
    assert_eq!(divergence.event_index, Some(1));
    assert!(
        divergence
            .description
            .contains("1 more expected event(s) than the re-run produced"),
        "the count is of expected events, not log lines: {}",
        divergence.description
    );
}

#[test]
fn a_truncated_rerun_is_reported_at_the_expectation_it_stopped_short_of() {
    // The log carries an observation between the last matched event and the
    // first unmatched one. Only `check` steps the cursor over observations,
    // so a re-run that stops early leaves it resting on one, and the reported
    // index must still point at the expectation, which is the line a reader
    // has to go and look at.
    let mut log = sample_log();
    let tick = log.events.pop().expect("fixture ends with its fingerprint");
    log.events
        .push(LogEvent::Observed(RecordedObservation::BudgetMissed(
            RecordedBudgetMiss {
                path: PUBLISHER.to_string(),
                sim_time: SimTime::ZERO,
                step_ms: 9.0,
                budget_ms: 1.0,
            },
        )));
    log.events.push(tick);

    let verifier = Verifier::new(log, &sample_config());
    verifier.message_callback()(&sample_message());
    let divergence = verifier.finish().expect_err("the tick is never matched");

    assert_eq!(
        divergence.event_index,
        Some(2),
        "the unmatched tick, not the observation at index 1"
    );
    assert!(
        divergence
            .description
            .contains("1 more expected event(s) than the re-run produced"),
        "and the observation is not counted: {}",
        divergence.description
    );
}

#[test]
fn the_verifier_rejects_a_header_mismatch_immediately() {
    let wrong_seed = ConductorConfig {
        world_seed: WORLD_SEED + 1,
        ..sample_config()
    };
    let verifier = Verifier::new(sample_log(), &wrong_seed);
    assert!(verifier.diverged());
    let divergence = verifier.finish().expect_err("must diverge");
    assert_eq!(divergence.event_index, None);
}

// --- playback -------------------------------------------------------------

#[test]
fn a_playback_double_republishes_its_recording_verbatim() {
    let log = sample_log();
    let mut playback_double =
        PlaybackComponent::from_log(ComponentId::new(PUBLISHER).unwrap(), &log, PUBLISHER);
    let mut ctx = StepCtx::new(SimTime::ZERO, None, WORLD_NAME, 0, Vec::new());

    let next_due = playback_double.step(&mut ctx).expect(
        "a playback double republishes bytes it \
         recorded, so nothing it publishes can be refused",
    );
    let outbox = ctx.take_outbox();
    assert_eq!(outbox.len(), 1);
    assert_eq!(outbox[0].0.as_str(), KEY);
    assert_eq!(outbox[0].1, PAYLOAD);
    // Nothing else was recorded, so the playback double never wakes again.
    assert_eq!(next_due, SimTime::from_nanos(i64::MAX));
}

#[test]
fn a_playback_double_ignores_other_publishers() {
    let log = sample_log();
    let mut playback_double = PlaybackComponent::from_log(
        ComponentId::new("someone-else").unwrap(),
        &log,
        "someone-else",
    );
    let mut ctx = StepCtx::new(SimTime::ZERO, None, WORLD_NAME, 0, Vec::new());

    assert_eq!(
        playback_double
            .step(&mut ctx)
            .expect("a double with nothing recorded publishes nothing"),
        SimTime::from_nanos(i64::MAX)
    );
    assert!(ctx.take_outbox().is_empty());
}

#[test]
fn a_playback_double_registers_like_any_component() {
    let log = sample_log();
    let mut conductor = Conductor::new(sample_config(), InProcTransport::new())
        .expect("free-run config is always accepted");
    conductor
        .add_component(
            WORLD_LEVEL,
            Box::new(PlaybackComponent::from_log(
                ComponentId::new(PUBLISHER).unwrap(),
                &log,
                PUBLISHER,
            )),
        )
        .expect("registration succeeds");
    conductor
        .run_until(SimTime::from_millis(1))
        .expect("playback schedules strictly forward");
    assert_eq!(conductor.tick(), 1, "the one recorded instant was stepped");
}
