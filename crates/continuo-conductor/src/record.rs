//! The determinism harness's event log (milestone 2): tick fingerprints and
//! published messages, recorded as human-readable JSON lines.
//!
//! Recording taps the two existing observation points — a
//! `MonitorTransport` callback for messages and the conductor's tick callback for
//! fingerprints — so the sim itself is untouched by being recorded. Replay in
//! milestone 2 is *re-execution + comparison*: run the same scenario again,
//! record it, and diff the logs; [`EventLog::first_divergence`] reports the
//! earliest mismatch.

use std::fmt;
use std::path::Path;
use std::sync::{Arc, Mutex};

use continuo_core::{Message, SimTime, hash::hex_u64};
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
    pub world: String,
    pub seed: u64,
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
    pub fn new(world: impl Into<String>, seed: u64) -> Self {
        Recorder {
            inner: Arc::new(Mutex::new(EventLog {
                header: LogHeader {
                    version: 1,
                    world: world.into(),
                    seed,
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
            let payload_text = std::str::from_utf8(&m.payload)
                .expect("payloads are serialized JSON, which is always valid UTF-8");
            let payload =
                RawValue::from_string(payload_text.to_string()).expect("payloads are valid JSON");
            inner
                .lock()
                .expect("recorder mutex is never poisoned")
                .events
                .push(LogEvent::Msg(RecordedMessage {
                    time: m.time,
                    key: m.key.to_string(),
                    publisher: m.publisher.to_string(),
                    seq: m.seq,
                    payload,
                }));
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
