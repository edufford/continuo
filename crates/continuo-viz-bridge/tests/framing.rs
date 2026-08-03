//! What the bridge puts on the wire, and what it does when nobody is
//! draining it.
//!
//! These run in the default build, with no Zenoh linked, which is the point
//! of keeping framing and delivery apart.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Barrier};

use continuo_conductor::{MembershipChange, RecordedJoin, RecordedLeave, membership_key};
use continuo_core::{ComponentPath, KeyExpr, Message, SimTime};
use continuo_viz_bridge::{CollectingSink, VizBridge, VizFrame, VizSink};

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
    let bridge = VizBridge::new(sink.clone());
    let mut tap = bridge.message_callback();
    tap(&pose_message(7));
    bridge.finish();

    let frames = sink.frames();
    assert_eq!(frames.len(), 1);
    let frame = &frames[0];

    // The key routes it, and the payload is the component's own bytes,
    // unchanged. That is what lets a milestone 7 viewer subscribing to the
    // same key see the same thing.
    assert_eq!(frame.key, "continuo/demo/actor/car1/pose");
    assert_eq!(frame.payload, pose_message(7).payload);

    // The metadata line is what a recorded log holds, so one parser reads
    // both.
    let line: serde_json::Value =
        serde_json::from_slice(&frame.metadata).expect("metadata is JSON");
    let msg = &line["msg"];
    assert_eq!(msg["key"], "continuo/demo/actor/car1/pose");
    assert_eq!(msg["publisher"], "car1/physics");
    assert_eq!(msg["seq"], 7);
    assert_eq!(msg["payload"]["position"]["x"], 1.5);
}

#[test]
fn membership_changes_are_framed_as_join_and_leave_lines() {
    let sink = CollectingSink::new();
    let bridge = VizBridge::new(sink.clone());
    let mut tap = bridge.membership_callback("demo");
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
        assert_eq!(frame.key, membership_key("demo").to_string());
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
    fn deliver(&mut self, _frame: &VizFrame) {
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
    let bridge = VizBridge::with_capacity(sink, 2);
    let mut tap = bridge.message_callback();

    // Far more than the worker can hold while it is stuck on the first
    // frame. None of these may block.
    for seq in 0..500 {
        tap(&pose_message(seq));
    }
    let dropped = bridge.dropped();

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
