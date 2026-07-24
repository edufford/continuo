//! The determinism harness's event log (milestone 2): tick fingerprints and
//! published messages, recorded as human-readable JSON lines.
//!
//! Recording taps the two existing observation points — a
//! `MonitorTransport` callback for messages and the conductor's tick
//! callback for fingerprints — so the sim itself is untouched by being
//! recorded.
//!
//! A recorded log has two distinct consumers, near-opposite in how the
//! log's data flows relative to the simulation:
//!
//! - **Verification** ([`Verifier`]): the log is an *expected-output
//!   ledger* and nothing flows into the sim. Every component re-runs live;
//!   each event is checked against the log as it happens, and the driving
//!   loop stops at the first divergence — which means "determinism is
//!   broken" (or the log was modified).
//! - **Open-loop resimulation** ([`PlaybackComponent`]): the log is an
//!   *input stimulus*. Selected recorded publishers are replaced by
//!   playback doubles that re-publish their recorded messages into the
//!   sim, while changed components run live against them. Nothing is
//!   compared; the new behavior diverging from the recording is the result
//!   being studied. (The played-back actors do not react to the live ones
//!   — that is what "open-loop" means.)
//!
//! [`EventLog::first_divergence`] remains for comparing two
//! already-recorded logs.

use std::fmt;
use std::path::Path;
use std::sync::{Arc, Mutex};

use continuo_core::{Component, ComponentId, KeyExpr, Message, SimTime, StepCtx, hash::hex_u64};
use serde::{Deserialize, Serialize};
use serde_json::value::RawValue;
use thiserror::Error;

/// Per-tick determinism fingerprint emitted by the conductor.
///
/// `tick_hash` covers everything the tick's stepped components did in
/// declaration order: paths, next-due times, published bytes, and (for
/// components implementing `state_bytes`) internal state. `world_hash`
/// chains tick hashes from the seeded initial value, so a single trailing
/// value fingerprints the entire run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TickFingerprint {
    pub tick: u64,
    pub sim_time: SimTime,
    #[serde(with = "hex_u64")]
    pub tick_hash: u64,
    #[serde(with = "hex_u64")]
    pub world_hash: u64,
}

/// A published message as recorded: paths flattened to strings, payload
/// embedded as raw JSON so the log stays readable.
#[derive(Debug, Serialize, Deserialize)]
pub struct RecordedMessage {
    pub time: SimTime,
    pub key: String,
    pub publisher: String,
    pub seq: u64,
    pub payload: Box<RawValue>,
}

/// One line of the log body, in emission order (messages of a tick precede
/// its fingerprint, since publishes happen during the steps).
#[derive(Debug, Serialize, Deserialize)]
pub enum LogEvent {
    #[serde(rename = "msg")]
    Msg(RecordedMessage),
    #[serde(rename = "tick")]
    Tick(TickFingerprint),
}

/// First line of the log: enough to re-create the run.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LogHeader {
    pub version: u32,
    pub world_name: String,
    pub world_seed: u64,
}

#[derive(Debug, Error)]
pub enum RecordError {
    #[error("event log I/O failed: {0}")]
    Io(#[from] std::io::Error),
    #[error("event log line {line} is not valid: {source}")]
    Parse {
        line: usize,
        #[source]
        source: serde_json::Error,
    },
    #[error("event log is empty (missing header line)")]
    Empty,
}

/// The earliest point at which two logs disagree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Divergence {
    /// Index into `events` (0-based); `None` for header mismatches.
    pub event_index: Option<usize>,
    pub description: String,
}

impl fmt::Display for Divergence {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.event_index {
            Some(i) => write!(f, "divergence at event {i}: {}", self.description),
            None => write!(f, "divergence in header: {}", self.description),
        }
    }
}

/// A complete recorded run.
#[derive(Debug, Serialize, Deserialize)]
pub struct EventLog {
    pub header: LogHeader,
    pub events: Vec<LogEvent>,
}

impl EventLog {
    /// Serializes as JSON lines: header first, one event per line.
    pub fn to_jsonl(&self) -> String {
        let mut out = String::new();
        out.push_str(&serde_json::to_string(&self.header).expect("header serializes"));
        out.push('\n');
        for event in &self.events {
            out.push_str(&serde_json::to_string(event).expect("event serializes"));
            out.push('\n');
        }

        // Return the complete log as JSON-lines text.
        out
    }

    pub fn from_jsonl(text: &str) -> Result<Self, RecordError> {
        let mut lines = text
            .lines()
            .enumerate()
            .filter(|(_, l)| !l.trim().is_empty());
        let (_, header_line) = lines.next().ok_or(RecordError::Empty)?;
        let header: LogHeader = serde_json::from_str(header_line)
            .map_err(|source| RecordError::Parse { line: 1, source })?;
        let mut events = Vec::new();
        for (i, line) in lines {
            events.push(
                serde_json::from_str(line).map_err(|source| RecordError::Parse {
                    line: i + 1,
                    source,
                })?,
            );
        }

        // Return the parsed header and events.
        Ok(EventLog { header, events })
    }

    pub fn write_file(&self, path: impl AsRef<Path>) -> Result<(), RecordError> {
        Ok(std::fs::write(path, self.to_jsonl())?)
    }

    pub fn read_file(path: impl AsRef<Path>) -> Result<Self, RecordError> {
        Self::from_jsonl(&std::fs::read_to_string(path)?)
    }

    /// Latest world hash in the log, if any tick was recorded.
    pub fn final_world_hash(&self) -> Option<u64> {
        self.events.iter().rev().find_map(|e| match e {
            LogEvent::Tick(d) => Some(d.world_hash),
            _ => None,
        })
    }

    /// Compares two runs event by event, returning the earliest mismatch —
    /// `None` means the runs are identical.
    pub fn first_divergence(&self, other: &EventLog) -> Option<Divergence> {
        if self.header != other.header {
            return Some(Divergence {
                event_index: None,
                description: format!("{:?} vs {:?}", self.header, other.header),
            });
        }
        for (i, (a, b)) in self.events.iter().zip(other.events.iter()).enumerate() {
            if let Some(description) = event_mismatch(a, b) {
                return Some(Divergence {
                    event_index: Some(i),
                    description,
                });
            }
        }
        if self.events.len() != other.events.len() {
            return Some(Divergence {
                event_index: Some(self.events.len().min(other.events.len())),
                description: format!(
                    "event counts differ: {} vs {}",
                    self.events.len(),
                    other.events.len()
                ),
            });
        }

        // Return None: headers, every event, and lengths all matched.
        None
    }
}

/// Converts a live transport message into its recorded form.
fn recorded_message(m: &Message) -> RecordedMessage {
    let payload_text = std::str::from_utf8(&m.payload)
        .expect("payloads are serialized JSON, which is always valid UTF-8");
    let payload = RawValue::from_string(payload_text.to_string()).expect("payloads are valid JSON");

    // Return the message with paths flattened and payload embedded as raw
    // JSON.
    RecordedMessage {
        time: m.time,
        key: m.key.to_string(),
        publisher: m.publisher.to_string(),
        seq: m.seq,
        payload,
    }
}

fn event_mismatch(a: &LogEvent, b: &LogEvent) -> Option<String> {
    match (a, b) {
        (LogEvent::Tick(x), LogEvent::Tick(y)) => {
            (x != y).then(|| format!("tick fingerprints differ: {x:?} vs {y:?}"))
        }
        (LogEvent::Msg(x), LogEvent::Msg(y)) => {
            let same = x.time == y.time
                && x.key == y.key
                && x.publisher == y.publisher
                && x.seq == y.seq
                && x.payload.get() == y.payload.get();
            (!same).then(|| {
                format!(
                    "messages differ: {}|{}|{}|{} {} vs {}|{}|{}|{} {}",
                    x.time,
                    x.key,
                    x.publisher,
                    x.seq,
                    x.payload,
                    y.time,
                    y.key,
                    y.publisher,
                    y.seq,
                    y.payload,
                )
            })
        }
        (LogEvent::Tick(_), LogEvent::Msg(_)) | (LogEvent::Msg(_), LogEvent::Tick(_)) => {
            Some("event kinds differ (tick vs msg)".to_string())
        }
    }
}

/// Collects a run into an [`EventLog`] via two cloneable callbacks: attach
/// [`Recorder::message_callback`] to a `MonitorTransport` and
/// [`Recorder::tick_callback`] to `Conductor::set_tick_callback`, run, then call
/// [`Recorder::finish`].
#[derive(Clone)]
pub struct Recorder {
    inner: Arc<Mutex<EventLog>>,
}

impl Recorder {
    pub fn new(world_name: impl Into<String>, world_seed: u64) -> Self {
        Recorder {
            inner: Arc::new(Mutex::new(EventLog {
                header: LogHeader {
                    version: 1,
                    world_name: world_name.into(),
                    world_seed,
                },
                events: Vec::new(),
            })),
        }
    }

    pub fn message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let inner = self.inner.clone();

        // Return the recording callback, holding its own handle to the
        // shared log.
        move |m: &Message| {
            inner
                .lock()
                .expect("recorder mutex is never poisoned")
                .events
                .push(LogEvent::Msg(recorded_message(m)));
        }
    }

    pub fn tick_callback(&self) -> impl FnMut(&TickFingerprint) + Send + 'static {
        let inner = self.inner.clone();

        // Return the recording callback, holding its own handle to the
        // shared log.
        move |d: &TickFingerprint| {
            inner
                .lock()
                .expect("recorder mutex is never poisoned")
                .events
                .push(LogEvent::Tick(*d));
        }
    }

    /// Takes the collected log. Other callback clones may still exist (inside a
    /// conductor); this snapshots the current contents.
    pub fn finish(&self) -> EventLog {
        let guard = self.inner.lock().expect("recorder mutex is never poisoned");

        // Return a snapshot of everything recorded so far.
        EventLog {
            header: guard.header.clone(),
            events: serde_json::from_str(
                &serde_json::to_string(&guard.events).expect("events serialize"),
            )
            .expect("events round-trip"),
        }
    }
}

/// Live replay verification: attach these callbacks to a re-run (message
/// callback on the `MonitorTransport`, tick callback on the conductor) and
/// every event is compared, in order, against the recorded log as it
/// happens — so the driving loop can stop at the first divergence instead
/// of running to completion:
///
/// ```text
/// while !checker.diverged()
///     && conductor.next_scheduled().is_some_and(|t| t <= end)
/// {
///     conductor.step_once()?;
/// }
/// ```
///
/// Both channels matter: message comparison catches log tampering and
/// wire-level drift (a modified log line leaves its neighboring
/// fingerprints intact); fingerprint comparison catches internal-state
/// divergence (`state_bytes`) that never surfaces in messages.
#[derive(Clone)]
pub struct Verifier {
    inner: Arc<Mutex<CheckerInner>>,
}

struct CheckerInner {
    expected: EventLog,
    cursor: usize,
    divergence: Option<Divergence>,
}

impl Verifier {
    /// Builds a checker against `expected`. The re-run's world name and
    /// seed are verified against the log header immediately, so a checker
    /// for the wrong scenario is diverged before the first event.
    pub fn new(expected: EventLog, world: &str, seed: u64) -> Self {
        let divergence = (expected.header.world_name != world
            || expected.header.world_seed != seed)
            .then(|| Divergence {
                event_index: None,
                description: format!(
                    "log was recorded for world {:?} seed {}; replaying world {:?} seed {}",
                    expected.header.world_name, expected.header.world_seed, world, seed
                ),
            });

        // Return the checker, already diverged on a header mismatch.
        Verifier {
            inner: Arc::new(Mutex::new(CheckerInner {
                expected,
                cursor: 0,
                divergence,
            })),
        }
    }

    fn check(inner: &mut CheckerInner, actual: &LogEvent) {
        if inner.divergence.is_some() {
            return;
        }
        match inner.expected.events.get(inner.cursor) {
            None => {
                inner.divergence = Some(Divergence {
                    event_index: Some(inner.cursor),
                    description: "the re-run produced more events than the recorded log"
                        .to_string(),
                });
            }
            Some(expected) => {
                if let Some(description) = event_mismatch(expected, actual) {
                    inner.divergence = Some(Divergence {
                        event_index: Some(inner.cursor),
                        description,
                    });
                }
            }
        }
        inner.cursor += 1;
    }

    pub fn message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let inner = self.inner.clone();

        // Return the checking callback, holding its own handle to the
        // shared cursor state.
        move |m: &Message| {
            let mut inner = inner.lock().expect("checker mutex is never poisoned");
            Self::check(&mut inner, &LogEvent::Msg(recorded_message(m)));
        }
    }

    pub fn tick_callback(&self) -> impl FnMut(&TickFingerprint) + Send + 'static {
        let inner = self.inner.clone();

        // Return the checking callback, holding its own handle to the
        // shared cursor state.
        move |fingerprint: &TickFingerprint| {
            let mut inner = inner.lock().expect("checker mutex is never poisoned");
            Self::check(&mut inner, &LogEvent::Tick(*fingerprint));
        }
    }

    /// Whether a divergence has been found — the driving loop's stop signal.
    pub fn diverged(&self) -> bool {
        self.inner
            .lock()
            .expect("checker mutex is never poisoned")
            .divergence
            .is_some()
    }

    /// The verdict so far: `Ok(verified event count)` if every event matched
    /// and none of the recorded log remains unconsumed, otherwise the first
    /// divergence (including a truncated re-run, which leaves recorded
    /// events unmatched).
    pub fn finish(&self) -> Result<usize, Divergence> {
        let inner = self.inner.lock().expect("checker mutex is never poisoned");
        if let Some(divergence) = &inner.divergence {
            return Err(divergence.clone());
        }
        if inner.cursor < inner.expected.events.len() {
            return Err(Divergence {
                event_index: Some(inner.cursor),
                description: format!(
                    "the recorded log has {} more event(s) than the re-run",
                    inner.expected.events.len() - inner.cursor
                ),
            });
        }

        // Return the number of events verified.
        Ok(inner.cursor)
    }
}

/// Replays one recorded publisher's messages as an ordinary component —
/// the open-loop resimulation stimulus (see the module docs for how this
/// differs from verification).
///
/// Built from an event log filtered to one publisher path (including its
/// sub-components): the recorded messages are re-published at their
/// recorded sim times, on their recorded keys, with byte-identical
/// payloads. Downstream components see them exactly as if the original
/// were running, so a live component can be swapped for its playback
/// double without consumers noticing. The double never reacts to the live
/// world — its behavior is pure data, which also keeps hybrid runs fully
/// deterministic and recordable.
pub struct PlaybackComponent {
    id: ComponentId,
    /// (time, key, payload) in recorded order.
    messages: Vec<(SimTime, KeyExpr, Box<RawValue>)>,
    cursor: usize,
}

impl PlaybackComponent {
    /// Filters `log` to messages recorded from `publisher` (a component
    /// path string) or any of its sub-components. `id` is the double's own
    /// registration id — typically the original actor's name.
    pub fn from_log(id: ComponentId, log: &EventLog, publisher: &str) -> Self {
        let prefix = format!("{publisher}/");
        let messages = log
            .events
            .iter()
            .filter_map(|event| match event {
                LogEvent::Msg(m)
                    if m.publisher == publisher || m.publisher.starts_with(&prefix) =>
                {
                    Some((
                        m.time,
                        KeyExpr::new(m.key.clone()).expect("recorded keys are valid"),
                        m.payload.clone(),
                    ))
                }
                _ => None,
            })
            .collect();

        // Return the double, positioned at the start of its recording.
        PlaybackComponent {
            id,
            messages,
            cursor: 0,
        }
    }
}

impl Component for PlaybackComponent {
    fn id(&self) -> ComponentId {
        self.id.clone()
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        Vec::new()
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        // Publish everything recorded for this instant; skip anything the
        // schedule somehow passed over (e.g. a double registered after its
        // first recorded messages) rather than stalling the run on it.
        while let Some((time, key, payload)) = self.messages.get(self.cursor) {
            if *time > ctx.now() {
                break;
            }
            if *time == ctx.now() {
                ctx.publish(key.clone(), payload)
                    .expect("recorded payloads re-serialize verbatim");
            }
            self.cursor += 1;
        }

        // Return the next recorded message time, or effectively never once
        // the recording is exhausted.
        match self.messages.get(self.cursor) {
            Some((time, _, _)) => *time,
            None => SimTime::from_nanos(i64::MAX),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_log() -> EventLog {
        let recorder = Recorder::new("test", 7);
        let mut msg_callback = recorder.message_callback();
        let mut tick_callback = recorder.tick_callback();
        msg_callback(&Message {
            key: continuo_core::KeyExpr::new("w/a").unwrap(),
            publisher: continuo_core::ComponentPath::parse("p").unwrap(),
            seq: 0,
            time: SimTime::ZERO,
            payload: br#"{"v":1.5}"#.to_vec(),
        });
        tick_callback(&TickFingerprint {
            tick: 1,
            sim_time: SimTime::ZERO,
            tick_hash: 0xdead_beef,
            world_hash: 0x1234_5678_9abc_def0,
        });

        // Return the collected sample log.
        recorder.finish()
    }

    #[test]
    fn jsonl_round_trip() {
        let log = sample_log();
        let text = log.to_jsonl();
        assert_eq!(text.lines().count(), 3);
        assert!(text.lines().nth(1).unwrap().contains(r#""v":1.5"#));
        let back = EventLog::from_jsonl(&text).unwrap();
        assert!(log.first_divergence(&back).is_none());
        assert_eq!(back.final_world_hash(), Some(0x1234_5678_9abc_def0));
    }

    fn sample_message() -> Message {
        Message {
            key: continuo_core::KeyExpr::new("w/a").unwrap(),
            publisher: continuo_core::ComponentPath::parse("p").unwrap(),
            seq: 0,
            time: SimTime::ZERO,
            payload: br#"{"v":1.5}"#.to_vec(),
        }
    }

    fn sample_fingerprint() -> TickFingerprint {
        TickFingerprint {
            tick: 1,
            sim_time: SimTime::ZERO,
            tick_hash: 0xdead_beef,
            world_hash: 0x1234_5678_9abc_def0,
        }
    }

    #[test]
    fn checker_accepts_a_matching_stream() {
        let checker = Verifier::new(sample_log(), "test", 7);
        checker.message_callback()(&sample_message());
        checker.tick_callback()(&sample_fingerprint());
        assert!(!checker.diverged());
        assert_eq!(checker.finish().expect("streams match"), 2);
    }

    #[test]
    fn checker_flags_the_first_mismatching_event() {
        let mut expected = sample_log();
        if let LogEvent::Tick(fingerprint) = &mut expected.events[1] {
            fingerprint.world_hash ^= 1;
        }
        let checker = Verifier::new(expected, "test", 7);
        checker.message_callback()(&sample_message());
        assert!(!checker.diverged(), "message still matches");
        checker.tick_callback()(&sample_fingerprint());
        assert!(checker.diverged(), "fingerprint mismatch must be caught");
        let divergence = checker.finish().expect_err("must diverge");
        assert_eq!(divergence.event_index, Some(1));
    }

    #[test]
    fn checker_flags_a_truncated_rerun() {
        let checker = Verifier::new(sample_log(), "test", 7);
        checker.message_callback()(&sample_message());
        // The re-run ends here; the recorded tick is never matched.
        let divergence = checker.finish().expect_err("must diverge");
        assert_eq!(divergence.event_index, Some(1));
        assert!(divergence.description.contains("more event(s)"));
    }

    #[test]
    fn checker_rejects_a_header_mismatch_immediately() {
        let checker = Verifier::new(sample_log(), "test", 8); // wrong seed
        assert!(checker.diverged());
        let divergence = checker.finish().expect_err("must diverge");
        assert_eq!(divergence.event_index, None);
    }

    #[test]
    fn divergence_detection() {
        let a = sample_log();
        let mut b = sample_log();
        assert!(a.first_divergence(&b).is_none());

        if let LogEvent::Tick(d) = &mut b.events[1] {
            d.world_hash ^= 1;
        }
        let div = a.first_divergence(&b).expect("must diverge");
        assert_eq!(div.event_index, Some(1));

        let c = Recorder::new("test", 8).finish(); // different seed
        assert!(
            a.first_divergence(&c)
                .expect("header differs")
                .event_index
                .is_none()
        );
    }
}
