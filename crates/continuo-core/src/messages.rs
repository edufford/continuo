use serde::{Deserialize, Serialize};

use crate::error::CoreError;
use crate::ids::{ComponentId, ComponentPath};
use crate::keyexpr::KeyExpr;
use crate::math::{Quat, Vec3};
use crate::time::SimTime;

/// Conductor → components: a new step boundary. In-process the activation is
/// a direct call, but the protocol is honored in types so the distributed
/// transport (milestone 7) carries the same messages.
// TODO(M7): serialize TickStart/TickDone over the transport for remote hosts
// (keys continuo/{world}/tick and continuo/{world}/tick/done per PLAN.md).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TickStart {
    pub tick: u64,
    pub sim_time: SimTime,
}

/// Component → conductor: step completed; `next_due` is the next sim time
/// this component should step (strictly greater than `sim_time`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TickDone {
    pub tick: u64,
    pub component_id: ComponentId,
    pub next_due: SimTime,
}

/// Pose in the world frame: meters, unit quaternion. Planar models publish
/// `z = 0` and yaw-only quaternions; the schema never changes for 3D.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Pose {
    pub position: Vec3,
    pub orientation: Quat,
}

/// One thing a sensor found ahead of it: how far off it is, and how fast
/// that is changing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Detection {
    /// Meters to it.
    pub range: f64,
    /// How fast the range is changing, m/s, negative when closing.
    pub range_rate: f64,
}

/// A routed unit on the transport: canonical JSON payload bytes plus the
/// metadata that makes delivery deterministic.
///
/// `seq` is a per-publisher monotonic counter stamped at publish time;
/// inboxes are delivered sorted by `(publisher, seq)` so ordering never
/// depends on execution or arrival order. Payloads stay serialized bytes
/// end-to-end so the canonical form survives for state hashing (milestone 2).
#[derive(Debug, Clone, PartialEq)]
pub struct Message {
    pub key: KeyExpr,
    pub publisher: ComponentPath,
    pub seq: u64,
    pub sim_time: SimTime,
    pub payload: Vec<u8>,
}

impl Message {
    /// Deserializes the JSON payload.
    ///
    /// The error names this message's key and publisher, which the caller
    /// would otherwise have to attach itself, because a component that cannot
    /// read a payload is rarely the one at fault.
    ///
    /// It is [`CoreError`] rather than `serde_json::Error` so that the usual
    /// shape inside a step is `?`: [`Component::step`](crate::Component::step)
    /// returns the same type, and a failure there halts the world. Swallowing
    /// one stays possible, by matching on the `Result` and carrying on, but it
    /// then says so where anyone can see it.
    ///
    /// That visibility is the point. A component running on stale data fails
    /// deterministically, so the hash holds steady and verification passes
    /// against a recording carrying the same fault: nothing in the determinism
    /// machinery can see it.
    pub fn decode<'a, T: Deserialize<'a>>(&'a self) -> Result<T, CoreError> {
        serde_json::from_slice(&self.payload).map_err(|source| CoreError::PayloadDecode {
            key: self.key.as_str().to_string(),
            publisher: self.publisher.to_string(),
            source,
        })
    }
}
