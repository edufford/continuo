//! Where observed frames go, kept separate from what is observed.
//!
//! Splitting the sink out is what keeps Zenoh optional. Framing a message
//! into the viewer's schema is the part with the design content, and it needs
//! no networking at all, so it stays in the default build and is unit-tested
//! against an in-memory sink. Only the delivery mechanism sits behind a
//! feature.

use std::io::Write;

use tracing::warn;

/// One framed event on its way to a viewer.
///
/// `key` routes it (a Zenoh publication key, or just a label for a writer),
/// and `payload` is the bytes a subscriber receives. `metadata` carries the
/// sim time, publisher, and sequence number that a raw payload does not
/// contain, so a sink can attach it out of band and leave the payload bytes
/// byte-identical to what the component published.
// TODO(M7): the metadata split is a placeholder, not a settled wire format.
// `Message` carries time, publisher, and seq for *all* traffic, and those
// have to cross the wire somehow once components publish remotely. Whatever
// is chosen there should subsume this: if `Message` gains a metadata section
// of its own, a sink stops needing to attach one.
#[derive(Debug, Clone, PartialEq)]
pub struct VizFrame {
    pub key: String,
    pub payload: Vec<u8>,
    pub metadata: Vec<u8>,
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
        // A viewer sink never propagates an error into the run, so a failed
        // write is counted as deliberately as a full channel is.
        let wrote = self
            .writer
            .write_all(&frame.metadata)
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
