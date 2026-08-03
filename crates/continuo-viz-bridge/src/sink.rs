//! Where observed frames go, kept separate from what is observed.
//!
//! Splitting the sink out is what keeps Zenoh optional. Framing a message
//! into the viewer's schema is the part with the design content, and it needs
//! no networking at all, so it stays in the default build and is unit-tested
//! against an in-memory sink. Only the delivery mechanism sits behind a
//! feature.

use std::io::Write;
use std::sync::{Arc, Mutex};

/// One framed event on its way to a viewer.
///
/// `key` routes it (a Zenoh publication key, or just a label for a writer),
/// and `payload` is the bytes a subscriber receives. `metadata` carries the
/// sim time, publisher, and sequence number that a raw payload does not
/// contain, so a sink can attach it out of band and leave the payload bytes
/// byte-identical to what the component published.
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
    fn deliver(&mut self, frame: &VizFrame);

    /// Called once when the run is finished, for sinks that buffer.
    fn flush(&mut self) {}
}

/// Writes one JSON line per frame to any [`Write`], which is what makes the
/// bridge testable and CI-runnable without Zenoh.
///
/// The line is the event-log `msg` shape, so a file written here and a log
/// written by `Recorder` are read by the same parser.
pub struct WriterSink<W: Write + Send> {
    writer: W,
}

impl<W: Write + Send> WriterSink<W> {
    pub fn new(writer: W) -> Self {
        WriterSink { writer }
    }
}

impl<W: Write + Send> VizSink for WriterSink<W> {
    fn deliver(&mut self, frame: &VizFrame) {
        // A viewer sink never propagates an error into the run, so a failed
        // write is dropped as deliberately as a full channel is.
        let _ = self.writer.write_all(&frame.metadata);
        let _ = self.writer.write_all(b"\n");
    }

    fn flush(&mut self) {
        let _ = self.writer.flush();
    }
}

/// Collects frames in memory, for tests that assert on what was observed.
#[derive(Clone, Default)]
pub struct CollectingSink {
    frames: Arc<Mutex<Vec<VizFrame>>>,
}

impl CollectingSink {
    pub fn new() -> Self {
        CollectingSink::default()
    }

    /// Every frame delivered so far.
    pub fn frames(&self) -> Vec<VizFrame> {
        // Return a snapshot, so a caller can assert without holding the lock.
        self.frames
            .lock()
            .expect("collecting sink mutex is never poisoned")
            .clone()
    }
}

impl VizSink for CollectingSink {
    fn deliver(&mut self, frame: &VizFrame) {
        self.frames
            .lock()
            .expect("collecting sink mutex is never poisoned")
            .push(frame.clone());
    }
}
