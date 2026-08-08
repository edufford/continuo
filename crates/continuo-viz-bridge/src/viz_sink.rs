//! Where observed frames go.
//!
//! The contract alone. Each destination gets a file of its own, so the one
//! that needs networking is the only one behind a feature: `writer_sink` in
//! the default build, `zenoh_sink` behind `zenoh`.

use crate::protocol::VizFrame;

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
