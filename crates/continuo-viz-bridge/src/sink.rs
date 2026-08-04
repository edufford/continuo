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
use tracing::warn;

/// Provenance a raw payload does not carry: the sim instant, the key it was
/// published on, who published it, and their sequence number.
///
/// This is [`RecordedMessage`] *without* the payload, because a frame already
/// carries the payload bytes and repeating them here would put every byte on
/// the wire twice. A sink that wants a self-contained event-log line
/// reassembles the two, which is what [`WriterSink`] does.
///
/// Keeping it separate is also what makes a viewer final across milestone 7.
/// Once components publish these keys natively, a payload is just a payload
/// with provenance alongside it, and nothing native would ever nest its own
/// payload inside its own metadata. A subscriber written against this shape
/// keeps working when the bridge stops being in the middle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MessageMeta {
    pub time: SimTime,
    pub key: String,
    pub publisher: String,
    pub seq: u64,
}

impl MessageMeta {
    /// The bytes a sink sends alongside the payload.
    ///
    /// Deliberately untagged, unlike an event-log line. A log line needs
    /// `{"msg": ...}` because lines of every kind share one file, whereas this
    /// only ever travels attached to a message, so a tag would be ceremony a
    /// native publisher would not repeat.
    pub fn to_bytes(&self) -> Vec<u8> {
        // Return the serialized metadata; every field is plain data, so the
        // only way this fails is a bug in serde itself.
        serde_json::to_vec(self).expect("message metadata always serializes")
    }
}

/// One framed event on its way to a viewer.
///
/// `key` routes it (a Zenoh publication key, or just a label for a writer) and
/// `payload` is the bytes a subscriber receives, byte-identical to what the
/// component published.
///
/// Framing stops there. Turning these into whatever shape a destination wants
/// is the sink's job, on the worker thread, because everything upstream of the
/// queue runs on the thread stepping the world.
// TODO(M7): `meta` is a placeholder, not a settled wire format. `Message`
// carries time, publisher, and seq for *all* traffic, and those have to cross
// the wire somehow once components publish remotely. Whatever is chosen there
// should subsume this: if `Message` gains a metadata section of its own, a
// sink stops needing to attach one.
#[derive(Debug, Clone, PartialEq)]
pub struct VizFrame {
    pub key: String,
    pub payload: Vec<u8>,
    /// Provenance for a published message.
    ///
    /// `None` for a conductor notification, whose payload is already a
    /// complete event-log line and needs nothing alongside it.
    pub meta: Option<MessageMeta>,
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
        let VizFrame { payload, meta, .. } = frame;
        let line = match meta {
            Some(meta) => match log_line(&meta, &payload) {
                Some(line) => line,
                None => {
                    self.num_failures += 1;

                    // Return without writing; a line that cannot be assembled
                    // is counted like any other delivery failure.
                    return;
                }
            },
            // A conductor notification is already a complete line.
            None => payload,
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
/// This is where the schema coupling lives, and it is load-bearing rather than
/// decorative: the line is built by constructing [`RecordedMessage`] itself, so
/// a field added there stops this compiling until it is carried in
/// [`MessageMeta`] too. `tests/framing.rs` pins the resulting *serde* shape,
/// which the type system cannot.
///
/// Returns `None` for a payload that is not valid JSON text. Every payload is
/// canonical JSON today, so this is an invariant rather than an expected case,
/// but a viewer is the wrong place to assert one: the worker thread would take
/// the whole bridge down with it, over a frame nobody would miss.
fn log_line(meta: &MessageMeta, payload: &[u8]) -> Option<Vec<u8>> {
    let text = std::str::from_utf8(payload).ok()?;
    let payload = RawValue::from_string(text.to_string()).ok()?;
    let event = LogEvent::Msg(RecordedMessage {
        time: meta.time,
        key: meta.key.clone(),
        publisher: meta.publisher.clone(),
        seq: meta.seq,
        payload,
    });

    // Return the assembled line, byte-compatible with what `Recorder` writes.
    serde_json::to_vec(&event).ok()
}
