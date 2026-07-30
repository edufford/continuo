//! Comparing runs against a recorded log: the [`Divergence`] two runs
//! disagree at, found either live during a re-run ([`Verifier`]) or after
//! the fact between two recorded logs
//! ([`EventLog::first_divergence`]). See [`crate::record`] for the log
//! itself and for how verification differs from open-loop resimulation.

use std::fmt;
use std::sync::{Arc, Mutex};

use continuo_core::Message;

use crate::config::ConductorConfig;
use crate::record::{EventLog, LogEvent, MembershipChange, TickFingerprint, recorded_message};

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

/// The lines of a log that verification is *about*: everything except the
/// observations, which record what the machine did rather than what the run
/// did and which a faithful re-run is free to differ on (see
/// [`RecordedObservation`]).
///
/// Indexes are kept alongside, so a divergence still points at the line it
/// is in rather than at a position in some filtered stream.
fn expectations(log: &EventLog) -> impl Iterator<Item = (usize, &LogEvent)> {
    // Return the comparable events, each with its index in the log.
    log.events
        .iter()
        .enumerate()
        .filter(|(_, event)| !matches!(event, LogEvent::Observed(_)))
}

/// The one place two recorded events are compared, shared by both readers
/// below: the log-vs-log comparison and the live checker.
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
        (LogEvent::Join(x), LogEvent::Join(y)) => {
            (x != y).then(|| format!("joins differ: {x:?} vs {y:?}"))
        }
        (LogEvent::Leave(x), LogEvent::Leave(y)) => {
            (x != y).then(|| format!("leaves differ: {x:?} vs {y:?}"))
        }
        (expected, actual) => Some(format!(
            "event kinds differ: {} vs {}",
            expected.kind(),
            actual.kind()
        )),
    }
}

// Comparison lives here rather than beside the log format: recording
// writes logs; verification is what reads two of them against each other.
impl EventLog {
    /// Compares two runs event by event, returning the earliest mismatch —
    /// `None` means the runs are identical.
    pub fn first_divergence(&self, other: &EventLog) -> Option<Divergence> {
        if self.header != other.header {
            return Some(Divergence {
                event_index: None,
                description: format!("{:?} vs {:?}", self.header, other.header),
            });
        }
        for ((i, a), (_, b)) in expectations(self).zip(expectations(other)) {
            if let Some(description) = event_mismatch(a, b) {
                return Some(Divergence {
                    event_index: Some(i),
                    description,
                });
            }
        }
        let (mine, theirs) = (expectations(self).count(), expectations(other).count());
        if mine != theirs {
            return Some(Divergence {
                event_index: Some(mine.min(theirs)),
                description: format!(
                    "expected event counts differ: {mine} vs {theirs} (observations are not \
                     compared, so these are not the logs' line counts)"
                ),
            });
        }

        // Return None: headers, every event, and lengths all matched.
        None
    }
}

/// Live replay verification: attach these callbacks to a re-run (message
/// callback on the `MonitorTransport`, tick callback on the conductor) and
/// every event is compared, in order, against the recorded log as it
/// happens — so the driving loop can stop at the first divergence instead
/// of running to completion:
///
/// ```text
/// while !verifier.diverged()
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
    inner: Arc<Mutex<VerifierInner>>,
}

struct VerifierInner {
    /// The log's expectations only, each paired with the line it came from.
    /// Filtering once here is what lets `cursor` mean "how many
    /// expectations have been matched" without every reader having to step
    /// over observations; keeping the line number is what lets a divergence
    /// still point into the log.
    expectations: Vec<(usize, LogEvent)>,
    /// How many lines the log had, for the one divergence that points past
    /// the end of it.
    line_count: usize,
    cursor: usize,
    divergence: Option<Divergence>,
}

impl Verifier {
    /// Builds a verifier that checks the run `config` describes against the
    /// `recorded` log. The world name and seed come from the *re-run*, never
    /// from the log: they are the actual side of the comparison, and taking
    /// them from the config the conductor is built with is what makes the
    /// header check meaningful — a log recorded for another scenario is
    /// diverged before the first event rather than silently verified
    /// against the wrong run.
    pub fn new(recorded: EventLog, config: &ConductorConfig) -> Self {
        // The ordinary case is the one with no code: headers agreeing makes
        // `then` yield `None`, which is a verifier that has found nothing
        // wrong and will start comparing at the first event. Only a
        // mismatch runs the closure below.
        let divergence = (recorded.header.world_name != config.world_name
            || recorded.header.world_seed != config.world_seed)
            .then(|| Divergence {
                event_index: None,
                description: format!(
                    "log was recorded for world {:?} seed {}; replaying world {:?} seed {}",
                    recorded.header.world_name,
                    recorded.header.world_seed,
                    config.world_name,
                    config.world_seed
                ),
            });

        let line_count = recorded.events.len();

        // Return the verifier, clean unless the header check above already
        // diverged it.
        Verifier {
            inner: Arc::new(Mutex::new(VerifierInner {
                expectations: recorded
                    .events
                    .into_iter()
                    .enumerate()
                    .filter(|(_, event)| !matches!(event, LogEvent::Observed(_)))
                    .collect(),
                line_count,
                cursor: 0,
                divergence,
            })),
        }
    }

    fn check(inner: &mut VerifierInner, actual: &LogEvent) {
        if inner.divergence.is_some() {
            return;
        }
        match inner.expectations.get(inner.cursor) {
            None => {
                inner.divergence = Some(Divergence {
                    event_index: Some(inner.line_count),
                    description: "the re-run produced more expected events than the recorded \
                                  log's expected events"
                        .to_string(),
                });
            }
            Some((line, expected)) => {
                if let Some(description) = event_mismatch(expected, actual) {
                    inner.divergence = Some(Divergence {
                        event_index: Some(*line),
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
            let mut inner = inner.lock().expect("verifier mutex is never poisoned");
            Self::check(&mut inner, &LogEvent::Msg(recorded_message(m)));
        }
    }

    pub fn tick_callback(&self) -> impl FnMut(&TickFingerprint) + Send + 'static {
        let inner = self.inner.clone();

        // Return the checking callback, holding its own handle to the
        // shared cursor state.
        move |fingerprint: &TickFingerprint| {
            let mut inner = inner.lock().expect("verifier mutex is never poisoned");
            Self::check(&mut inner, &LogEvent::Tick(*fingerprint));
        }
    }

    pub fn membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let inner = self.inner.clone();

        // Return the checking callback, holding its own handle to the
        // shared cursor state.
        move |change: &MembershipChange| {
            let mut inner = inner.lock().expect("verifier mutex is never poisoned");
            Self::check(&mut inner, &change.clone().into());
        }
    }

    /// Whether a divergence has been found — the driving loop's stop signal.
    pub fn diverged(&self) -> bool {
        self.inner
            .lock()
            .expect("verifier mutex is never poisoned")
            .divergence
            .is_some()
    }

    /// The verdict so far: `Ok(verified event count)` if every event matched
    /// and none of the recorded log remains unconsumed, otherwise the first
    /// divergence (including a truncated re-run, which leaves recorded
    /// events unmatched).
    pub fn finish(&self) -> Result<usize, Divergence> {
        let inner = self.inner.lock().expect("verifier mutex is never poisoned");
        if let Some(divergence) = &inner.divergence {
            return Err(divergence.clone());
        }
        let unmatched = inner.expectations.len() - inner.cursor;
        if unmatched > 0 {
            let (line, _) = inner.expectations[inner.cursor];
            return Err(Divergence {
                event_index: Some(line),
                description: format!(
                    "the recorded log has {unmatched} more expected event(s) than the re-run \
                     produced (observations are not compared, so this is not a line count)"
                ),
            });
        }

        // Return the number of expectations verified.
        Ok(inner.cursor)
    }
}
