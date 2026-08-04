//! The `Write` delivery sink, which needs no networking.
//!
//! This is what makes the bridge runnable and testable without Zenoh: every
//! part of the design except the socket is exercised through here, in the
//! default build.

use std::io::Write;

use continuo_conductor::record::{LogEvent, RecordedMessage};
use serde_json::value::RawValue;
use tracing::{debug, warn};

use crate::protocol::{MessageType, Metadata, VizFrame};
use crate::viz_sink::VizSink;

/// Writes one JSON line per frame to any [`Write`].
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
