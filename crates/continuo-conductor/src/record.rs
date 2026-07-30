//! The determinism harness's event log (milestone 2): tick fingerprints and
//! published messages, recorded as human-readable JSON lines.
//!
//! Recording taps the conductor's observation points — a `MonitorTransport`
//! callback for messages, and conductor callbacks for tick fingerprints,
//! membership changes, and over-budget steps — so the sim itself is
//! untouched by being recorded.
//!
//! Lines come in two categories: **expectations** are what the run did, and
//! a faithful re-run must produce them again; **observations** are what the
//! machine did, and a re-run is free to differ. Only the first kind is ever
//! compared. Every observation sits under [`LogEvent::Observed`], so which
//! category a line is in is read off its variant — see [`RecordedObservation`].
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
use crate::timing::OnTimeout;

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

/// A component admitted to the world, as recorded.
///
/// Deliberately *not* recorded: the sim time the join was applied at. It is
/// already implied by where this event sits between tick fingerprints, and
/// it is the part that may legitimately vary — once joins arrive over the
/// transport (milestone 7) the boundary that admits one depends on
/// delivery. What shapes the run is `first_due`, which the joiner declares,
/// so a run stays deterministically reproducible as long as that is
/// processed the same, whichever boundary the request landed on.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecordedJoin {
    pub path: String,
    /// The instant the newcomer first steps.
    pub first_due: SimTime,
}

/// A component removed from the world, as recorded.
///
/// Carries the declared instant for the same reason a join carries
/// `first_due`: it is chosen by whoever asked, so it is stable however
/// early or late the request was made, and it is what decides where this
/// component's output stops. What is *not* recorded — here as on a join —
/// is the moment the request was applied, which says nothing extra and is
/// the part that varies with delivery.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecordedLeave {
    pub path: String,
    /// The first instant the component did not step.
    pub leaves_at: SimTime,
}

/// A membership change, as handed to observers while it happens.
///
/// The log records *that* a component joined or left, not how to rebuild it
/// — a component cannot be reconstructed from bytes. Replaying a dynamic
/// run means re-running the scenario that issues the same joins and leaves,
/// and checking the resulting stream against the log.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MembershipChange {
    Joined(RecordedJoin),
    Left(RecordedLeave),
}

/// A step that ran over the wall-clock budget its component declared.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecordedBudgetMiss {
    pub path: String,
    /// The instant the over-budget step ran at.
    pub sim_time: SimTime,
    /// What the step took, and what it was allowed, in wall-clock
    /// milliseconds.
    pub step_ms: f64,
    pub budget_ms: f64,
}

/// A component the conductor stopped waiting for, and what it did about it.
///
/// This is what says *why* a component left. The [`RecordedLeave`] that
/// follows a [`OnTimeout::Remove`] is deliberately identical to a scripted
/// one — the run behaved the same way, so replaying it by asking for that
/// leave at that instant must produce a matching log — and the reason lives
/// here, outside what verification compares. It is also the only record of a
/// [`OnTimeout::Halt`], which produces no leave at all: without this line a
/// halted run's log simply stops, saying nothing about why.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecordedTimeout {
    pub path: String,
    /// The instant whose step the conductor gave up waiting for.
    pub sim_time: SimTime,
    /// How long it waited, and how long it had declared it would, in
    /// wall-clock milliseconds.
    pub waited_ms: f64,
    pub timeout_ms: f64,
    /// What this did to the run — so a reader knows whether to expect a
    /// leave next or the log to end here.
    pub policy: OnTimeout,
}

/// Something the *machine* did rather than the run: one of the log's
/// **observations**.
///
/// The log holds two kinds of line. Every other kind is an **expectation**,
/// recording what the run did, and a faithful re-run must produce it again.
/// An observation is something noticed while the run happened that is worth
/// writing down, but that says nothing about whether the run was right.
///
/// They are noted centrally so they can all be read back from one file
/// afterwards, rather than from whichever process each step ran in.
/// Verification never compares them: another machine notices different
/// things, and treating that as a divergence would report two runs that
/// behaved identically as different.
///
/// Both members so far are wall-clock facts, because that is what milestones
/// 3 and 4 measure. Anything else of the same character belongs here rather
/// than as a new top-level event: the pacing overruns counted by
/// [`Conductor::overrun_reanchor_count`](crate::Conductor::overrun_reanchor_count)
/// are the obvious next one, being a run-level measurement that today exists
/// only as a counter that dies with the process.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RecordedObservation {
    #[serde(rename = "budget")]
    BudgetMissed(RecordedBudgetMiss),
    #[serde(rename = "timeout")]
    TimedOut(RecordedTimeout),
}

/// One line of the log body, in emission order (messages of a tick precede
/// its fingerprint, since publishes happen during the steps; membership
/// changes sit between ticks, where they are applied).
///
/// Every observation nests under the one [`LogEvent::Observed`] variant
/// rather than getting a variant of its own, so that being an observation is
/// a structural fact instead of a list for each reader to keep in step with.
/// A new kind added to [`RecordedObservation`] is categorised correctly the
/// moment it exists; a new top-level variant would silently become an
/// expectation and start reporting false divergences.
#[derive(Debug, Serialize, Deserialize)]
pub enum LogEvent {
    #[serde(rename = "msg")]
    Msg(RecordedMessage),
    #[serde(rename = "tick")]
    Tick(TickFingerprint),
    #[serde(rename = "join")]
    Join(RecordedJoin),
    #[serde(rename = "leave")]
    Leave(RecordedLeave),
    /// Anything the machine did rather than the run — a log line reads
    /// `{"observed":{"budget":{...}}}`.
    #[serde(rename = "observed")]
    Observed(RecordedObservation),
}

impl LogEvent {
    /// The event's kind, for reporting a mismatch between two different
    /// kinds of event.
    pub(crate) fn kind(&self) -> &'static str {
        match self {
            LogEvent::Msg(_) => "msg",
            LogEvent::Tick(_) => "tick",
            LogEvent::Join(_) => "join",
            LogEvent::Leave(_) => "leave",
            LogEvent::Observed(RecordedObservation::BudgetMissed(_)) => "observed/budget",
            LogEvent::Observed(RecordedObservation::TimedOut(_)) => "observed/timeout",
        }
    }
}

impl From<RecordedObservation> for LogEvent {
    fn from(observation: RecordedObservation) -> Self {
        // Return the observation as the log line that records it.
        LogEvent::Observed(observation)
    }
}

impl From<MembershipChange> for LogEvent {
    fn from(change: MembershipChange) -> Self {
        // Return the change as the log line that records it.
        match change {
            MembershipChange::Joined(join) => LogEvent::Join(join),
            MembershipChange::Left(leave) => LogEvent::Leave(leave),
        }
    }
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

    pub fn membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let inner = self.inner.clone();

        // Return the recording callback, holding its own handle to the
        // shared log.
        move |change: &MembershipChange| {
            inner
                .lock()
                .expect("recorder mutex is never poisoned")
                .events
                .push(change.clone().into());
        }
    }

    /// Records what the machine did — over-budget steps, and the timeouts
    /// that say why a component left or a run stopped — so a run's
    /// conditions end up in one file rather than in whichever process each
    /// step ran in. These lines are never compared; see [`RecordedObservation`].
    pub fn observation_callback(&self) -> impl FnMut(&RecordedObservation) + Send + 'static {
        let inner = self.inner.clone();

        // Return the recording callback, holding its own handle to the
        // shared log.
        move |observation: &RecordedObservation| {
            inner
                .lock()
                .expect("recorder mutex is never poisoned")
                .events
                .push(observation.clone().into());
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
