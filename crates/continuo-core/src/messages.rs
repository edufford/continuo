use serde::{Deserialize, Serialize};

use crate::ids::{ComponentId, ComponentPath};
use crate::keyexpr::KeyExpr;
use crate::math::{Quat, Vec3};
use crate::time::SimTime;

/// Conductor → components: a new step boundary. In-process the activation is
/// a direct call, but the protocol is honored in types so the distributed
/// transport (milestone 7) carries the same messages.
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
    pub time: SimTime,
    pub payload: Vec<u8>,
}

impl Message {
    /// Deserializes the JSON payload.
    pub fn decode<'a, T: Deserialize<'a>>(&'a self) -> Result<T, serde_json::Error> {
        serde_json::from_slice(&self.payload)
    }
}
