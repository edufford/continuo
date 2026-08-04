//! What the bridge puts on the wire, and what it does when nobody is
//! draining it.
//!
//! These run in the default build, with no Zenoh linked, which is the point
//! of keeping framing and delivery apart.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Barrier, Mutex};
use std::time::{Duration, Instant};

use continuo_conductor::{ConductorConfig, MembershipChange, Pacing, RecordedJoin, RecordedLeave};
use continuo_core::{ComponentPath, KeyExpr, Message, SimTime};
use continuo_viz_bridge::{VizBridge, VizFrame, VizSink};

/// Collects frames in memory so a test can assert on what was delivered.
///
/// Lives here rather than in the crate because it exists only for these
/// tests, and shipping it would put a test double in the public API.
#[derive(Clone, Default)]
struct CollectingSink {
    frames: Arc<Mutex<Vec<VizFrame>>>,
}

impl CollectingSink {
    fn new() -> Self {
        CollectingSink::default()
    }

    /// A snapshot, so a caller can assert without holding the lock.
    fn frames(&self) -> Vec<VizFrame> {
        self.frames
            .lock()
            .expect("collecting sink mutex is never poisoned")
            .clone()
    }
}

impl VizSink for CollectingSink {
    fn deliver(&mut self, frame: VizFrame) {
        self.frames
            .lock()
            .expect("collecting sink mutex is never poisoned")
            .push(frame);
    }
}

fn config() -> ConductorConfig {
    ConductorConfig {
        world_name: "demo".into(),
        world_seed: 0,
        pacing: Pacing::FreeRun,
    }
}

fn pose_message(seq: u64) -> Message {
    Message {
        key: KeyExpr::new("continuo/demo/actor/car1/pose").expect("valid key"),
        publisher: ComponentPath::parse("car1/physics").expect("valid path"),
        seq,
        time: SimTime::from_millis(500),
        payload: br#"{"position":{"x":1.5,"y":0.0,"z":0.0}}"#.to_vec(),
    }
}

#[test]
fn a_message_is_framed_as_the_event_logs_msg_line() {
    let sink = CollectingSink::new();
    let bridge = VizBridge::new(&config(), sink.clone());
    let mut tap = bridge.message_callback();
    tap(&pose_message(7));
    bridge.finish();

    let frames = sink.frames();
    assert_eq!(frames.len(), 1);
    let frame = &frames[0];

    // Published onto the viewer's side channel rather than back onto the
    // key it came from, which is what stops a bridged message colliding with
    // a component publishing the same key, or echoing one that arrived over
    // the transport.
    assert_eq!(frame.key, "continuo/demo/viz/actor/car1/pose");
    assert_eq!(frame.payload, pose_message(7).payload);

    // The metadata line is what a recorded log holds, so one parser reads
    // both.
    let line: serde_json::Value =
        serde_json::from_slice(&frame.metadata).expect("metadata is JSON");
    let msg = &line["msg"];
    // The original key is not lost: it rides in the metadata, which is where
    // a viewer reads it from for every source.
    assert_eq!(msg["key"], "continuo/demo/actor/car1/pose");
    assert_eq!(msg["publisher"], "car1/physics");
    assert_eq!(msg["seq"], 7);
    assert_eq!(msg["payload"]["position"]["x"], 1.5);
}

#[test]
fn membership_changes_are_framed_as_join_and_leave_lines() {
    let sink = CollectingSink::new();
    let bridge = VizBridge::new(&config(), sink.clone());
    let mut tap = bridge.membership_callback();
    tap(&MembershipChange::Joined(RecordedJoin {
        path: "traffic7/physics".into(),
        first_due: SimTime::from_millis(250),
    }));
    tap(&MembershipChange::Left(RecordedLeave {
        path: "traffic7/physics".into(),
        leaves_at: SimTime::from_secs(9),
    }));
    bridge.finish();

    let frames = sink.frames();
    assert_eq!(frames.len(), 2);
    for frame in &frames {
        assert_eq!(
            frame.key, "continuo/demo/viz/conductor/membership/status",
            "membership goes down the same side channel as everything else"
        );
    }

    let join: serde_json::Value =
        serde_json::from_slice(&frames[0].metadata).expect("join is JSON");
    assert_eq!(join["join"]["path"], "traffic7/physics");
    let leave: serde_json::Value =
        serde_json::from_slice(&frames[1].metadata).expect("leave is JSON");
    assert_eq!(leave["leave"]["path"], "traffic7/physics");
}

/// Blocks in `deliver` until released, so the queue behind it can be filled.
struct BlockingSink {
    gate: Arc<Barrier>,
    passed_the_gate: Arc<AtomicBool>,
}

impl VizSink for BlockingSink {
    fn deliver(&mut self, _frame: VizFrame) {
        if !self.passed_the_gate.swap(true, Ordering::SeqCst) {
            self.gate.wait();
        }
    }
}

#[test]
fn a_viewer_that_stops_draining_is_dropped_rather_than_waited_for() {
    // The load-bearing property: a stalled viewer must never become back
    // pressure on a step, so frames are discarded instead of blocking.
    let gate = Arc::new(Barrier::new(2));
    let sink = BlockingSink {
        gate: gate.clone(),
        passed_the_gate: Arc::new(AtomicBool::new(false)),
    };
    let bridge = VizBridge::with_capacity(&config(), sink, 2);
    let mut tap = bridge.message_callback();

    // Far more than the worker can hold while it is stuck on the first
    // frame. None of these may block.
    for seq in 0..500 {
        tap(&pose_message(seq));
    }
    let dropped = bridge.dropped_frames();

    // Release the worker *before* asserting. A failed assertion unwinds
    // through `Drop`, which joins the worker, so leaving it parked on the
    // barrier would turn a test failure into a hung test run.
    gate.wait();
    bridge.finish();

    assert!(
        dropped > 0,
        "a full queue must drop frames, not wait for room"
    );
}

/// Never returns from `deliver`, standing in for a sink wedged on a socket.
struct WedgedSink;

impl VizSink for WedgedSink {
    fn deliver(&mut self, _frame: VizFrame) {
        loop {
            std::thread::sleep(Duration::from_secs(3600));
        }
    }
}

#[test]
fn a_wedged_sink_is_detached_rather_than_waited_for_forever() {
    // `shutdown` also runs from `Drop`, so an unbounded join would turn a
    // stuck sink into a program that never exits. Giving up and detaching is
    // the only option Rust offers, and the right one: the run is over and the
    // frames still held are of no interest.
    let bridge = VizBridge::with_capacity(&config(), WedgedSink, 1);
    let mut tap = bridge.message_callback();
    tap(&pose_message(0));

    let started = Instant::now();
    bridge.finish();
    let waited = started.elapsed();

    assert!(
        waited < Duration::from_secs(30),
        "finish must give up on a wedged sink, but waited {waited:?}"
    );
}
