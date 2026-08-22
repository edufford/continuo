//! What is observed, as it goes on the wire.
//!
//! The schema, kept apart from where frames end up. It has the design content
//! and no networking in it at all, so it stays in the default build and is
//! pinned by tests that never link Zenoh, while only delivery sits behind a
//! feature.
//!
//! `python/continuo_viz/protocol.py` is the other end, named to match. Nothing
//! makes the two agree, because JSON over a network fails by producing nothing
//! rather than by failing to compile, so a change here is a change there.

use continuo_core::SimTime;
use serde::{Deserialize, Serialize};

/// What kind of thing a payload is, so a reader never has to infer it.
///
/// Stated rather than deduced from the key, which would make every consumer
/// re-implement the same string matching and break the moment a key moves.
///
/// The axis is what produced the payload: the simulated world, or the
/// conductor running it.
// TODO(M7): the tick protocol and the join and leave *requests* land here as
// further variants when they cross the wire, and this leaves the bridge with
// [`Metadata`], whose note explains where to and why.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    /// A component publishing what it simulated, such as a pose or a command.
    SimData,
    /// The conductor announcing a membership change that has taken effect.
    MembershipStatus,
}

/// What a payload does not say about itself: what kind of thing it is, when it
/// happened, what it was published on, by whom, and where it sits in that
/// publisher's sequence.
///
/// Every frame carries one, whatever kind of payload it is, so a subscriber
/// reads the same fields off everything and switches on `message_type`.
// TODO(M7): this and [`MessageType`] move out of the bridge together. They are
// wire vocabulary rather than anything a viewer owns, and live here only
// because the bridge is currently the one thing putting these on a wire.
// `membership_key` in `continuo-conductor` carries the same note and names the
// destination, `continuo-core`, so all of it should move at once rather than
// as a series of half-moves.
//
// Whether this survives as a type of its own depends on `Message`, which
// already carries sim time, publisher, and seq for *all* traffic and has to
// get them across somehow once components publish remotely. If it gains a
// metadata section, that subsumes this and a sink stops attaching one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    pub message_type: MessageType,
    pub sim_time: SimTime,
    /// The key the payload was published on, before relaying onto the viewer
    /// side channel. This is the one a subscriber should read.
    pub key: String,
    pub publisher: String,
    pub seq: u64,
}

impl Metadata {
    /// The bytes a sink sends alongside the payload.
    ///
    /// Untagged, unlike an event-log line, which needs `{"msg": ...}` only
    /// because lines of every kind share one file.
    pub fn to_bytes(&self) -> Vec<u8> {
        // Return the serialized metadata; every field is plain data, so the
        // only way this fails is a bug in serde itself.
        serde_json::to_vec(self).expect("metadata always serializes")
    }
}

/// One framed event on its way to a viewer.
///
/// `key` routes it (a Zenoh publication key, or just a label for a writer) and
/// `payload` is the bytes a subscriber receives, byte-identical to what was
/// published.
///
/// Framing stops there. Turning these into whatever shape a destination wants
/// is the sink's job, on the worker thread, because everything upstream of the
/// queue runs on the thread stepping the world.
// TODO(M7): `metadata` is a first cut, not a settled wire format. See
// [`Metadata`] for where it goes and what may replace it.
#[derive(Debug, Clone, PartialEq)]
pub struct VizFrame {
    pub key: String,
    pub payload: Vec<u8>,
    pub metadata: Metadata,
}
