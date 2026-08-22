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
//! Everything published here goes under `continuo_viz/`, a side channel no
//! simulation component reads, mirroring `continuo/` segment for segment
//! beneath it. Relaying onto the *original* key would collide with components
//! that publish it themselves once there is a Zenoh transport, and worse, a
//! message that arrived over Zenoh would be echoed straight back onto the key
//! it came from. Rooting the mirror outside `continuo/` makes that impossible
//! rather than merely unlikely. The original key is not lost: it travels in the
//! frame's metadata, which is where a viewer reads it from anyway, alongside
//! the sim time, publisher, and sequence number.
//!
//! A payload crosses the wire once. The bridge frames a message as its bytes
//! plus [`Metadata`] and hands both to the sink, which decides what to do
//! with them: `ZenohSink` publishes the payload and attaches the metadata,
//! while `WriterSink` reassembles the two into the event log's `msg` line.
//! Serializing in the sink rather than in the tap also keeps that work off the
//! thread stepping the world.

mod protocol;
mod viz_sink;
mod writer_sink;

#[cfg(feature = "zenoh")]
mod zenoh_sink;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use continuo_conductor::record::LogEvent;
use continuo_conductor::{ConductorConfig, MembershipChange, membership_key};
use continuo_core::{KEY_ROOT, Message, SimTime};
use continuo_transport::{MonitorTransport, Transport};
use tracing::{debug, warn};

pub use protocol::{MessageType, Metadata, VizFrame};
pub use viz_sink::VizSink;
pub use writer_sink::WriterSink;

#[cfg(feature = "zenoh")]
pub use zenoh_sink::{ZenohSink, ZenohSinkError};

/// How many frames may be queued for the viewer before new ones are dropped.
///
/// Sized for a viewer that stalls for a moment rather than one that has gone
/// away: a few hundred milliseconds of a busy world. Beyond that, the oldest
/// frames are of no interest to a live view anyway.
const DEFAULT_CAPACITY: usize = 4096;

/// How long the worker waits for a frame before re-checking for shutdown,
/// and therefore also how often [`VizBridge::shutdown`] re-checks whether it
/// has finished. Short enough that finishing a run is not perceptibly
/// delayed, long enough that an idle bridge is not spinning.
///
/// One constant rather than two because the second would be lying: the
/// worker only observes the shutdown flag when this timeout expires, so a
/// waiter polling faster cannot learn anything sooner.
const SHUTDOWN_POLL: Duration = Duration::from_millis(20);

/// Thread name for the delivery worker, so it is identifiable in a panic
/// message, a debugger, or a process listing.
const WORKER_THREAD_NAME: &str = "continuo-viz";

/// Root the viewer side channel sits under, mirroring [`KEY_ROOT`] segment for
/// segment beneath it.
///
/// Namespaced to this project rather than a bare `viz/`, which on a Zenoh
/// network shared with anything else would be claiming a very general name.
pub const VIZ_KEY_ROOT: &str = "continuo_viz";

/// Publisher name on membership metadata.
// TODO(M7): the conductor applies the change, so it is named as the publisher,
// but the bridge is what puts the bytes on the wire today. When membership
// status crosses the transport the conductor publishes it itself and stamps
// its own name, and this goes away with the sequence counter beside it.
const MEMBERSHIP_PUBLISHER: &str = "conductor";

/// How long [`VizBridge::shutdown`] waits for the worker before detaching it.
///
/// Generous enough that an ordinary sink finishes its queue, short enough
/// that a wedged one does not hold up process exit. A viewer is never worth
/// blocking a program for.
const JOIN_TIMEOUT: Duration = Duration::from_secs(2);

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
    ///
    /// Runs on the thread stepping the world, so it does as little as it can:
    /// copy the payload, note where the message came from, and queue it.
    /// Serializing anything is the sink's job, on the worker.
    pub fn message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let tx = self.tx.clone();
        let dropped_frames = self.dropped_frames.clone();

        // Return the tap, holding its own handle on the queue.
        move |m: &Message| {
            let frame = VizFrame {
                key: viz_key(m.key.as_str()),
                payload: m.payload.clone(),
                metadata: Metadata {
                    message_type: MessageType::SimData,
                    sim_time: m.sim_time,
                    key: m.key.to_string(),
                    publisher: m.publisher.to_string(),
                    seq: m.seq,
                },
            };
            try_queue(&tx, &dropped_frames, frame);
        }
    }

    /// The membership tap, so a viewer learns exactly when a component left
    /// rather than inferring it from silence.
    pub fn membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let tx = self.tx.clone();
        let dropped_frames = self.dropped_frames.clone();
        let published_key = membership_key(&self.world_name).to_string();
        let frame_key = viz_key(&published_key);

        // A stub, so membership metadata has the same shape as everything
        // else. It counts notifications and nothing more: a real `seq` is
        // stamped by the conductor, decides `(publisher, seq)` delivery order,
        // and feeds the tick hash, and none of that is true here.
        // TODO(M7): membership status crosses the transport, so the conductor
        // stamps it centrally like any other publication and this goes away.
        let mut seq: u64 = 0;

        // Return the tap, holding its own handle on the queue.
        move |change: &MembershipChange| {
            let frame = VizFrame {
                key: frame_key.clone(),
                payload: membership_line(change),
                metadata: Metadata {
                    message_type: MessageType::MembershipStatus,
                    sim_time: membership_takes_effect_at(change),
                    key: published_key.clone(),
                    publisher: MEMBERSHIP_PUBLISHER.to_string(),
                    seq,
                },
            };
            seq += 1;
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
            std::thread::sleep(SHUTDOWN_POLL);
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

/// The side-channel key a publication is relayed onto.
///
/// `continuo/demo/actor/car1/pose` becomes
/// `continuo_viz/demo/actor/car1/pose`: one root swapped for the other, every
/// segment beneath it untouched.
///
/// Swapping roots rather than nesting inside the world is what keeps this two
/// constants and no world-dependent parsing. Components publish under
/// [`KEY_ROOT`] and the side channel is rooted outside it, so no relayed key
/// can equal a published one and a message cannot be echoed back onto the key
/// it arrived on. A key that is not under [`KEY_ROOT`] at all is nested whole,
/// which lands it on the side channel just the same.
fn viz_key(published_key: &str) -> String {
    // The separator is required, not merely stripped if present, so a world
    // named `continuoX` cannot match the root and be handed back as `X/...`.
    let rest = published_key
        .strip_prefix(KEY_ROOT)
        .and_then(|rest| rest.strip_prefix('/'))
        .unwrap_or(published_key);

    // Return the side-channel key for this publication.
    format!("{VIZ_KEY_ROOT}/{rest}")
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

/// The instant a membership change takes effect, which is what its metadata
/// reports as the sim time.
///
/// The *declared* instant, and deliberately not the one the conductor
/// processed the change at, which `RecordedJoin` and `RecordedLeave` refuse
/// to record because it varies with delivery. The declared instant is chosen
/// by whoever asked, so it is stable however early or late the request
/// arrived.
fn membership_takes_effect_at(change: &MembershipChange) -> SimTime {
    // Return the instant the newcomer first steps, or the first instant the
    // departing component does not.
    match change {
        MembershipChange::Joined(join) => join.first_due,
        MembershipChange::Left(leave) => leave.leaves_at,
    }
}

/// Frames a membership change as the event log's `join` or `leave` line.
///
/// Unlike a message, this is serialized here rather than in the sink, because
/// there is no separate payload to hold it apart from: the line *is* the
/// payload. It is cheap and rare, two of them per component for a whole run,
/// against a pose every 10 ms.
///
/// The *field* set is structural, since this builds [`LogEvent`] itself, so
/// adding or removing a field breaks compilation here. What is not structural
/// is the *serde* shape, meaning the tag names, which could change in
/// `continuo-conductor::record` without touching this file. That gap is what
/// `membership_changes_are_framed_as_join_and_leave_lines` in
/// `tests/framing.rs` pins.
fn membership_line(change: &MembershipChange) -> Vec<u8> {
    let event = match change {
        MembershipChange::Joined(join) => LogEvent::Join(join.clone()),
        MembershipChange::Left(leave) => LogEvent::Leave(leave.clone()),
    };

    // Return the serialized line, matching what `Recorder` would have written.
    serde_json::to_vec(&event).expect("a membership change always serializes")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_root_segment_changes() {
        assert_eq!(
            viz_key("continuo/demo/actor/car1/pose"),
            "continuo_viz/demo/actor/car1/pose"
        );
    }

    #[test]
    fn a_key_outside_the_root_is_nested_whole() {
        // Still lands on the side channel, so a key that does not follow the
        // convention cannot be relayed back onto itself. The integration tests
        // only ever see conventional keys, so this would otherwise go
        // unexercised.
        assert_eq!(viz_key("elsewhere/pose"), "continuo_viz/elsewhere/pose");
    }

    #[test]
    fn a_relayed_key_can_never_equal_a_published_one() {
        // The property the whole scheme rests on: components publish under
        // `continuo/`, the side channel is rooted outside it, so a message
        // cannot be echoed back onto the key it arrived on. That matters once
        // there is a real Zenoh transport and the bridge is subscribed to the
        // same network it publishes to.
        for published in [
            "continuo/demo/actor/car1/pose",
            "continuo/demo/conductor/membership/status",
            "elsewhere/pose",
        ] {
            let relayed = viz_key(published);
            assert_ne!(relayed, published);

            // Compared chunk by chunk rather than by prefix, because
            // `continuo_viz` *does* start with `continuo`. What has to differ
            // is the root chunk, not the leading characters.
            let root = relayed.split('/').next().expect("a key has a first chunk");
            assert_eq!(root, VIZ_KEY_ROOT);
            assert_ne!(root, KEY_ROOT);
        }
    }

    #[test]
    fn a_world_whose_name_extends_the_root_is_not_mistaken_for_it() {
        // The separator is required when stripping, so `continuoX` is a key
        // outside the root and is nested whole, rather than having `continuo`
        // shaved off it to leave `X/demo/...`.
        assert_eq!(
            viz_key("continuoX/demo/actor/car1/pose"),
            "continuo_viz/continuoX/demo/actor/car1/pose"
        );
    }
}
