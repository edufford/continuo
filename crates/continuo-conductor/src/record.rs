//! The determinism harness's event log (milestone 2): tick fingerprints and
//! published messages, recorded as human-readable JSON lines.
//!
//! Recording taps the two existing observation points — a
//! `MonitorTransport` callback for messages and the conductor's tick
//! callback for fingerprints — so the sim itself is untouched by being
//! recorded.
//!
//! A recorded log has two distinct consumers, near-opposite in how the
//! log's data flows relative to the simulation, one module each:
//!
//! - **Verification** ([`crate::Verifier`]): the log is an *expected-output
//!   ledger* and nothing flows into the sim. Every component re-runs live;
//!   each event is checked against the log as it happens, and the driving
//!   loop stops at the first divergence — which means "determinism is
//!   broken" (or the log was modified). Comparing two already-recorded
//!   logs lives there too.
//! - **Open-loop resimulation** ([`crate::PlaybackComponent`]): the log is an
//!   *input stimulus*. Selected recorded publishers are replaced by
//!   playback doubles that re-publish their recorded messages into the
//!   sim, while changed components run live against them. Nothing is
//!   compared; the new behavior diverging from the recording is the result
//!   being studied. (The played-back actors do not react to the live ones
//!   — that is what "open-loop" means.)

use std::path::Path;
use std::sync::{Arc, Mutex};

use continuo_core::{Message, SimTime, hash::hex_u64};
use serde::{Deserialize, Serialize};
use serde_json::value::RawValue;
use thiserror::Error;

use crate::config::ConductorConfig;

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
}

/// Converts a live transport message into its recorded form.
pub(crate) fn recorded_message(m: &Message) -> RecordedMessage {
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

/// Collects a run into an [`EventLog`] via two cloneable callbacks: attach
/// [`Recorder::message_callback`] to a `MonitorTransport` and
/// [`Recorder::tick_callback`] to `Conductor::set_tick_callback`, run, then call
/// [`Recorder::finish`].
#[derive(Clone)]
pub struct Recorder {
    inner: Arc<Mutex<EventLog>>,
}

impl Recorder {
    /// Starts a recording of the run `config` describes. The header comes
    /// from the same config the conductor runs with, so a log can never
    /// claim a world or seed the run did not use.
    pub fn new(config: &ConductorConfig) -> Self {
        // Return an empty log headed by this run's identity.
        Recorder {
            inner: Arc::new(Mutex::new(EventLog {
                header: LogHeader {
                    version: 1,
                    world_name: config.world_name.clone(),
                    world_seed: config.world_seed,
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
