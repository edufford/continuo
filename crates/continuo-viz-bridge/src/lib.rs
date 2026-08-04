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
//!
//! Everything published here goes under `continuo/{world}/viz/`, a side
//! channel no simulation component reads. Relaying onto the *original* key
//! would collide with components that publish it themselves once there is a
//! Zenoh transport, and worse, a message that arrived over Zenoh would be
//! echoed straight back onto the key it came from. The original key is not
//! lost: it travels in the frame's metadata, which is where a viewer reads it
//! from anyway, alongside the sim time, publisher, and sequence number.

mod sink;

#[cfg(feature = "zenoh")]
mod zenoh_sink;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use continuo_conductor::record::{LogEvent, RecordedMessage};
use continuo_conductor::{ConductorConfig, MembershipChange, membership_key};
use continuo_core::Message;
use continuo_transport::{MonitorTransport, Transport};
use serde_json::value::RawValue;
use tracing::{debug, warn};

pub use sink::{VizFrame, VizSink, WriterSink};

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

/// Thread name for the delivery worker, so it is identifiable in a panic
/// message, a debugger, or a process listing.
const WORKER_THREAD_NAME: &str = "continuo-viz";

/// How long [`VizBridge::shutdown`] waits for the worker before detaching it.
///
/// Generous enough that an ordinary sink finishes its queue, short enough
/// that a wedged one does not hold up process exit. A viewer is never worth
/// blocking a program for.
const JOIN_TIMEOUT: Duration = Duration::from_secs(2);

/// How often the shutdown wait re-checks whether the worker has finished.
const JOIN_POLL: Duration = Duration::from_millis(5);

/// Observes a run and hands framed events to a [`VizSink`].
///
/// Attach [`Self::message_callback`] to a `MonitorTransport` (or use
/// [`Self::wrap_transport`]) and [`Self::membership_callback`] to
/// `Conductor::add_membership_callback`.
pub struct VizBridge {
    /// Names the side channel this bridge publishes under. Taken from the
    /// run's own config, so a viewer can never be pointed at a world the
    /// conductor is not running, the same reason `Recorder` takes it.
    world_name: String,
    tx: SyncSender<VizFrame>,
    dropped_frames: Arc<AtomicU64>,
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
    /// Starts a bridge for the run `config` describes, delivering to `sink`
    /// on its own thread.
    pub fn new(config: &ConductorConfig, sink: impl VizSink + 'static) -> Self {
        VizBridge::with_capacity(config, sink, DEFAULT_CAPACITY)
    }

    pub fn with_capacity(
        config: &ConductorConfig,
        mut sink: impl VizSink + 'static,
        capacity: usize,
    ) -> Self {
        let (tx, rx) = sync_channel::<VizFrame>(capacity);
        let shutdown = Arc::new(AtomicBool::new(false));
        let dropped_frames = Arc::new(AtomicU64::new(0));
        let worker = {
            let shutdown = shutdown.clone();
            let dropped_frames = dropped_frames.clone();
            std::thread::Builder::new()
                // Named so it is identifiable in a panic message, a debugger,
                // or a process listing, none of which show a closure.
                .name(WORKER_THREAD_NAME.to_string())
                .spawn(move || {
                    loop {
                        match rx.recv_timeout(SHUTDOWN_POLL) {
                            Ok(frame) => sink.deliver(frame),
                            // Nothing queued. Only now is it safe to stop,
                            // since anything still in flight has been
                            // delivered.
                            Err(RecvTimeoutError::Timeout) => {
                                if shutdown.load(Ordering::Acquire) {
                                    break;
                                }
                            }
                            Err(RecvTimeoutError::Disconnected) => break,
                        }
                    }
                    sink.flush();
                    debug!(
                        target: "continuo::viz",
                        dropped_frames = dropped_frames.load(Ordering::Relaxed),
                        "viz bridge worker stopping"
                    );
                })
                .expect("spawning the viz worker thread")
        };

        // Return a bridge whose worker owns the sink, so delivery never
        // happens on the thread that is stepping the world.
        VizBridge {
            world_name: config.world_name.clone(),
            tx,
            dropped_frames,
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
        let dropped_frames = self.dropped_frames.clone();
        let world_name = self.world_name.clone();

        // Return the tap, holding its own handle on the queue.
        move |m: &Message| {
            let frame = VizFrame {
                key: viz_key(&world_name, m.key.as_str()),
                payload: m.payload.clone(),
                metadata: message_line(m),
            };
            try_queue(&tx, &dropped_frames, frame);
        }
    }

    /// The membership tap, so a viewer learns exactly when a component left
    /// rather than inferring it from silence.
    pub fn membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let tx = self.tx.clone();
        let dropped_frames = self.dropped_frames.clone();
        let key = viz_key(&self.world_name, membership_key(&self.world_name).as_str());

        // Return the tap, holding its own handle on the queue.
        move |change: &MembershipChange| {
            let line = membership_line(change);
            let frame = VizFrame {
                key: key.clone(),
                payload: line.clone(),
                metadata: line,
            };
            try_queue(&tx, &dropped_frames, frame);
        }
    }

    /// How many frames were dropped because the viewer was not keeping up.
    ///
    /// Diagnostic only. Dropping is the designed behavior rather than a
    /// failure, since a live view wants the latest state and not a backlog.
    pub fn dropped_frames(&self) -> u64 {
        self.dropped_frames.load(Ordering::Relaxed)
    }

    /// Closes the queue and waits for the sink to finish.
    pub fn finish(mut self) {
        self.shutdown();
    }

    /// Ends the worker and waits for it, but only up to [`JOIN_TIMEOUT`].
    ///
    /// Bounded because this also runs from `Drop`, so a sink wedged inside
    /// `deliver` would otherwise hang the whole program at exit rather than
    /// just the bridge. `JoinHandle` has no timed join, hence polling
    /// `is_finished` to a deadline.
    ///
    /// Giving up detaches the thread rather than killing it, which is the
    /// only option Rust offers and the right outcome anyway: the sink is
    /// stuck, the run is over, and the frames it still holds are of no
    /// interest to anyone.
    fn shutdown(&mut self) {
        let Some(worker) = self.worker.take() else {
            return;
        };
        self.shutdown.store(true, Ordering::Release);

        let deadline = Instant::now() + JOIN_TIMEOUT;
        while !worker.is_finished() {
            if Instant::now() >= deadline {
                warn!(
                    target: "continuo::viz",
                    timeout_ms = JOIN_TIMEOUT.as_millis() as u64,
                    "viz bridge worker did not stop in time; detaching it"
                );

                // Return without joining, leaving the thread detached.
                return;
            }
            std::thread::sleep(JOIN_POLL);
        }
        debug!(target: "continuo::viz", "joining the viz bridge worker");
        let _ = worker.join();
    }
}

impl Drop for VizBridge {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Maps a key a component published on to the viewer's side channel.
///
/// `continuo/demo/actor/car1/pose` becomes
/// `continuo/demo/viz/actor/car1/pose`. Keys that do not carry the expected
/// world prefix are nested whole rather than rewritten, so an unconventional
/// key is still separated from live traffic instead of being relayed onto
/// itself.
fn viz_key(world_name: &str, published_key: &str) -> String {
    let prefix = format!("continuo/{world_name}/");

    // Return the side-channel key for this publication.
    match published_key.strip_prefix(&prefix) {
        Some(rest) => format!("continuo/{world_name}/viz/{rest}"),
        None => format!("continuo/{world_name}/viz/{published_key}"),
    }
}

/// Queues a frame, counting it as dropped rather than waiting for room.
fn try_queue(tx: &SyncSender<VizFrame>, dropped_frames: &AtomicU64, frame: VizFrame) {
    match tx.try_send(frame) {
        Ok(()) => {}
        Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
            dropped_frames.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Frames a message as the event log's `msg` line, so a live stream and a
/// recorded log are read by the same parser.
///
/// The *field* set is structural: this builds [`LogEvent::Msg`] from
/// [`RecordedMessage`], so adding or removing a field there breaks
/// compilation here. What is not structural is the *serde* shape, meaning the
/// tag names and how `payload` is embedded, which could change in
/// `continuo-conductor::record` without touching this file. That gap is what
/// `a_message_is_framed_as_the_event_logs_msg_line` in `tests/framing.rs`
/// pins, by asserting the emitted line parses as
/// `{"msg": {key, publisher, seq, payload}}`.
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
///
/// Coupled to [`LogEvent`] the same way [`message_line`] is, and pinned the
/// same way by `membership_changes_are_framed_as_join_and_leave_lines` in
/// `tests/framing.rs`.
fn membership_line(change: &MembershipChange) -> Vec<u8> {
    let event = match change {
        MembershipChange::Joined(join) => LogEvent::Join(join.clone()),
        MembershipChange::Left(leave) => LogEvent::Leave(leave.clone()),
    };

    // Return the serialized line, matching what `Recorder` would have written.
    serde_json::to_vec(&event).expect("a membership change always serializes")
}
