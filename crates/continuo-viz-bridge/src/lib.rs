//! Live visualization output (milestone 5): republishing what a run observes
//! so a viewer can draw it.
//!
//! This is an *observer*, not a participant. It wraps the transport the way
//! `Recorder` does rather than joining the world as a component, because a
//! component's path and `next_due` feed the tick hash, so a viz component
//! would make a watched run fingerprint differently from the same run
//! unwatched. Attaching this changes nothing about the run, which
//! `hash_neutrality` pins.
//!
//! Nothing here throttles. Once components publish over Zenoh natively
//! (milestone 7) a viewer will receive whatever rate the world produces, so a
//! bridge that thinned the stream would be behavior the later path cannot
//! reproduce. Aggregating and drawing at a sensible frame rate is the
//! viewer's job.
//!
//! The sim is never blocked by a viewer. Frames go onto a bounded channel and
//! are dropped when it is full, because a slow or absent viewer must not
//! become back pressure on a step.

mod sink;

#[cfg(feature = "zenoh")]
mod zenoh_sink;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::thread::JoinHandle;
use std::time::Duration;

use continuo_conductor::record::{LogEvent, RecordedMessage};
use continuo_conductor::{MembershipChange, membership_key};
use continuo_core::Message;
use continuo_transport::{MonitorTransport, Transport};
use serde_json::value::RawValue;

pub use sink::{CollectingSink, VizFrame, VizSink, WriterSink};

#[cfg(feature = "zenoh")]
pub use zenoh_sink::ZenohSink;

/// How many frames may be queued for the viewer before new ones are dropped.
///
/// Sized for a viewer that stalls for a moment rather than one that has gone
/// away: a few hundred milliseconds of a busy world. Beyond that, the oldest
/// frames are of no interest to a live view anyway.
const DEFAULT_CAPACITY: usize = 4096;

/// How long the worker waits for a frame before re-checking for shutdown.
/// Short enough that finishing a run is not perceptibly delayed, long enough
/// that an idle bridge is not spinning.
const SHUTDOWN_POLL: Duration = Duration::from_millis(20);

/// Observes a run and hands framed events to a [`VizSink`].
///
/// Attach [`Self::message_callback`] to a `MonitorTransport` (or use
/// [`Self::wrap_transport`]) and [`Self::membership_callback`] to
/// `Conductor::add_membership_callback`.
///
/// Observers accumulate, so recording a run and watching it are not mutually
/// exclusive: a `Recorder` and a bridge each add their own callback and both
/// are invoked.
pub struct VizBridge {
    tx: SyncSender<VizFrame>,
    dropped: Arc<AtomicU64>,
    /// Set by [`VizBridge::finish`] to end the worker.
    ///
    /// The worker cannot simply wait for the channel to close, because every
    /// callback holds a clone of the sender and callbacks normally outlive
    /// the bridge handle. Waiting on the channel alone deadlocks in exactly
    /// the ordinary case: finishing a run while the tap is still installed.
    shutdown: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl VizBridge {
    /// Starts a bridge delivering to `sink` on its own thread.
    pub fn new(sink: impl VizSink + 'static) -> Self {
        VizBridge::with_capacity(sink, DEFAULT_CAPACITY)
    }

    pub fn with_capacity(mut sink: impl VizSink + 'static, capacity: usize) -> Self {
        let (tx, rx) = sync_channel::<VizFrame>(capacity);
        let shutdown = Arc::new(AtomicBool::new(false));
        let worker = {
            let shutdown = shutdown.clone();
            std::thread::spawn(move || {
                loop {
                    match rx.recv_timeout(SHUTDOWN_POLL) {
                        Ok(frame) => sink.deliver(&frame),
                        // Nothing queued. Only now is it safe to stop, since
                        // anything still in flight has been delivered.
                        Err(RecvTimeoutError::Timeout) => {
                            if shutdown.load(Ordering::Acquire) {
                                break;
                            }
                        }
                        Err(RecvTimeoutError::Disconnected) => break,
                    }
                }
                sink.flush();
            })
        };

        // Return a bridge whose worker owns the sink, so delivery never
        // happens on the thread that is stepping the world.
        VizBridge {
            tx,
            dropped: Arc::new(AtomicU64::new(0)),
            shutdown,
            worker: Some(worker),
        }
    }

    /// Wraps `inner` so every published message is offered to the viewer.
    pub fn wrap_transport<T: Transport>(&self, inner: T) -> MonitorTransport<T> {
        // Return the wrapped transport, tapped for the viewer.
        MonitorTransport::new(inner, self.message_callback())
    }

    /// The transport tap, for composing with other monitors by hand.
    pub fn message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let tx = self.tx.clone();
        let dropped = self.dropped.clone();

        // Return the tap, holding its own handle on the queue.
        move |m: &Message| {
            let frame = VizFrame {
                key: m.key.to_string(),
                payload: m.payload.clone(),
                metadata: message_line(m),
            };
            offer(&tx, &dropped, frame);
        }
    }

    /// The membership tap, so a viewer learns exactly when a component left
    /// rather than inferring it from silence.
    pub fn membership_callback(
        &self,
        world_name: &str,
    ) -> impl FnMut(&MembershipChange) + Send + 'static {
        let tx = self.tx.clone();
        let dropped = self.dropped.clone();
        let key = membership_key(world_name).to_string();

        // Return the tap, holding its own handle on the queue.
        move |change: &MembershipChange| {
            let line = membership_line(change);
            let frame = VizFrame {
                key: key.clone(),
                payload: line.clone(),
                metadata: line,
            };
            offer(&tx, &dropped, frame);
        }
    }

    /// How many frames were dropped because the viewer was not keeping up.
    ///
    /// Diagnostic only. Dropping is the designed behavior rather than a
    /// failure, since a live view wants the latest state and not a backlog.
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Closes the queue and waits for the sink to finish.
    pub fn finish(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        if let Some(worker) = self.worker.take() {
            self.shutdown.store(true, Ordering::Release);
            let _ = worker.join();
        }
    }
}

impl Drop for VizBridge {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Queues a frame, counting it as dropped rather than waiting for room.
fn offer(tx: &SyncSender<VizFrame>, dropped: &AtomicU64, frame: VizFrame) {
    match tx.try_send(frame) {
        Ok(()) => {}
        Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
            dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Frames a message as the event log's `msg` line, so a live stream and a
/// recorded log are read by the same parser.
fn message_line(m: &Message) -> Vec<u8> {
    let payload_text = std::str::from_utf8(&m.payload)
        .expect("payloads are serialized JSON, which is always valid UTF-8");
    let payload = RawValue::from_string(payload_text.to_string()).expect("payloads are valid JSON");
    let event = LogEvent::Msg(RecordedMessage {
        time: m.time,
        key: m.key.to_string(),
        publisher: m.publisher.to_string(),
        seq: m.seq,
        payload,
    });

    // Return the serialized line; a viewer frame is never large enough for
    // this to be worth reusing a buffer for.
    serde_json::to_vec(&event).expect("a recorded message always serializes")
}

/// Frames a membership change as the event log's `join` or `leave` line.
fn membership_line(change: &MembershipChange) -> Vec<u8> {
    let event = match change {
        MembershipChange::Joined(join) => LogEvent::Join(join.clone()),
        MembershipChange::Left(leave) => LogEvent::Leave(leave.clone()),
    };

    // Return the serialized line, matching what `Recorder` would have written.
    serde_json::to_vec(&event).expect("a membership change always serializes")
}
