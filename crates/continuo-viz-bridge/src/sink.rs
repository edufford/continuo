//! Where observed frames go, kept separate from what is observed.
//!
//! Splitting the sink out is what keeps Zenoh optional. Framing a message
//! into the viewer's schema is the part with the design content, and it needs
//! no networking at all, so it stays in the default build and is unit-tested
//! against an in-memory sink. Only the delivery mechanism sits behind a
//! feature.

use std::io::Write;

use continuo_conductor::record::{LogEvent, RecordedMessage};
use continuo_core::SimTime;
use serde::{Deserialize, Serialize};
use serde_json::value::RawValue;
use tracing::{debug, warn};

/// What kind of thing a payload is, so a reader never has to infer it.
///
/// Stated rather than deduced from the key, which would make every consumer
/// re-implement the same string matching and break the moment a key moves.
///
/// The axis is what produced the payload: the simulated world, or the
/// conductor running it.
// TODO(M7): the tick protocol and the join and leave *requests* land here as
// further variants when they cross the wire, and this leaves the bridge with
// [`Metadata`], whose note explains where to and why.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    /// A component publishing what it simulated, such as a pose or a command.
    SimData,
    /// The conductor announcing a membership change it has applied.
    MembershipStatus,
}

/// What a payload does not say about itself: what kind of thing it is, when it
/// happened, what it was published on, by whom, and where it sits in that
/// publisher's sequence.
///
/// Every frame carries one, whatever kind of payload it is, so a subscriber
/// reads the same fields off everything and switches on `message_type`.
// TODO(M7): this and [`MessageType`] move out of the bridge together. They are
// wire vocabulary rather than anything a viewer owns, and live here only
// because the bridge is currently the one thing putting these on a wire.
// `membership_key` in `continuo-conductor` carries the same note and names the
// destination, `continuo-core`, so all of it should move at once rather than
// as a series of half-moves.
//
// Whether this survives as a type of its own depends on `Message`, which
// already carries sim time, publisher, and seq for *all* traffic and has to
// get them across somehow once components publish remotely. If it gains a
// metadata section, that subsumes this and a sink stops attaching one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    pub message_type: MessageType,
    pub sim_time: SimTime,
    /// The key the payload was published on, before relaying onto the viewer
    /// side channel. This is the one a subscriber should read.
    pub key: String,
    pub publisher: String,
    pub seq: u64,
}

impl Metadata {
    /// The bytes a sink sends alongside the payload.
    ///
    /// Untagged, unlike an event-log line, which needs `{"msg": ...}` only
    /// because lines of every kind share one file.
    pub fn to_bytes(&self) -> Vec<u8> {
        // Return the serialized metadata; every field is plain data, so the
        // only way this fails is a bug in serde itself.
        serde_json::to_vec(self).expect("metadata always serializes")
    }
}

/// One framed event on its way to a viewer.
///
/// `key` routes it (a Zenoh publication key, or just a label for a writer) and
/// `payload` is the bytes a subscriber receives, byte-identical to what was
/// published.
///
/// Framing stops there. Turning these into whatever shape a destination wants
/// is the sink's job, on the worker thread, because everything upstream of the
/// queue runs on the thread stepping the world.
// TODO(M7): `metadata` is a first cut, not a settled wire format. See
// [`Metadata`] for where it goes and what may replace it.
#[derive(Debug, Clone, PartialEq)]
pub struct VizFrame {
    pub key: String,
    pub payload: Vec<u8>,
    pub metadata: Metadata,
}

/// Somewhere framed events can be delivered.
///
/// Deliberately infallible. A viewer that has gone away, a socket that is
/// full, or a file that will not write must never become the simulation's
/// problem, so a sink swallows its own failures and reports them by counting
/// rather than by returning.
pub trait VizSink: Send {
    /// Hands one frame to whatever is downstream.
    ///
    /// Called on the bridge's worker thread, never on the thread stepping the
    /// world, so taking a while here costs frames rather than sim time. Takes
    /// the frame **by value** because the worker has no use for it afterwards
    /// and a sink usually wants to move the bytes onward: borrowing would
    /// force every implementation to clone what it was given.
    ///
    /// Failures are the sink's own to handle. Count them and report at
    /// [`Self::flush`] rather than logging per frame, since a broken
    /// destination fails on *every* frame and the bridge is deliberately fed
    /// at full message rate.
    fn deliver(&mut self, frame: VizFrame);

    /// Called once when the run is finished, for sinks that buffer or that
    /// have failures worth summarizing.
    fn flush(&mut self) {}
}

/// Writes one JSON line per frame to any [`Write`], which is what makes the
/// bridge testable and CI-runnable without Zenoh.
///
/// The line is the event-log `msg` shape, so a file written here and a log
/// written by `Recorder` are read by the same parser.
pub struct WriterSink<W: Write + Send> {
    writer: W,
    /// Counted rather than propagated, and reported once at flush. A failing
    /// writer fails on every frame, so logging each one would be its own
    /// denial of service.
    num_failures: u64,
}

impl<W: Write + Send> WriterSink<W> {
    pub fn new(writer: W) -> Self {
        WriterSink {
            writer,
            num_failures: 0,
        }
    }

    /// How many frames could not be written.
    pub fn num_failures(&self) -> u64 {
        self.num_failures
    }
}

impl<W: Write + Send> VizSink for WriterSink<W> {
    fn deliver(&mut self, frame: VizFrame) {
        let VizFrame {
            payload, metadata, ..
        } = frame;
        // A membership notification's payload is already a complete log line;
        // a component's payload is bare and has to be wrapped in one.
        let line = match metadata.message_type {
            MessageType::MembershipStatus => payload,
            MessageType::SimData => match log_line(metadata, payload) {
                Some(line) => line,
                None => {
                    // `log_line` has already said which way it failed, since
                    // it is the only one that knows.
                    self.num_failures += 1;

                    // Return without writing; a line that cannot be assembled
                    // is counted like any other delivery failure.
                    return;
                }
            },
        };

        // A viewer sink never propagates an error into the run, so a failed
        // write is counted as deliberately as a full channel is.
        let wrote = self
            .writer
            .write_all(&line)
            .and_then(|()| self.writer.write_all(b"\n"));
        if wrote.is_err() {
            self.num_failures += 1;
        }
    }

    fn flush(&mut self) {
        if self.writer.flush().is_err() {
            self.num_failures += 1;
        }
        if self.num_failures > 0 {
            warn!(
                target: "continuo::viz",
                num_failures = self.num_failures,
                "some viewer frames could not be written"
            );
        }
    }
}

/// Rebuilds the event log's `msg` line from a frame's two halves.
///
/// Built by constructing [`RecordedMessage`] itself, so a field added there
/// stops this compiling until [`Metadata`] carries it too. `tests/framing.rs`
/// pins the serde shape, which the type system cannot.
///
/// Takes both halves by value, so the payload buffer and the metadata's
/// strings move into the line rather than being copied into it.
///
/// Returns `None` for a payload that is not JSON text, after logging which way
/// it failed. Not expected, and not worth taking the worker thread down over.
fn log_line(metadata: Metadata, payload: Vec<u8>) -> Option<Vec<u8>> {
    // At `debug` rather than `warn` because a destination that fails does so
    // on every frame, and the bridge is deliberately fed at full message rate.
    // `WriterSink::flush` reports the total once.
    let log_unusable = |reason: &str, error: &dyn std::fmt::Display| {
        debug!(
            target: "continuo::viz",
            key = %metadata.key,
            publisher = %metadata.publisher,
            seq = metadata.seq,
            %error,
            "{reason}; cannot assemble a log line for it"
        );
    };

    let text = match String::from_utf8(payload) {
        Ok(text) => text,
        Err(error) => {
            log_unusable("payload is not UTF-8", &error);

            // Return nothing; the caller counts this like any other failure.
            return None;
        }
    };
    let payload = match RawValue::from_string(text) {
        Ok(payload) => payload,
        Err(error) => {
            log_unusable("payload is not valid JSON", &error);
            return None;
        }
    };

    let event = LogEvent::Msg(RecordedMessage {
        time: metadata.sim_time,
        key: metadata.key,
        publisher: metadata.publisher,
        seq: metadata.seq,
        payload,
    });

    // Return the assembled line, byte-compatible with what `Recorder` writes.
    serde_json::to_vec(&event).ok()
}
