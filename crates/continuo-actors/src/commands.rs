//! What a controller asks a plant for, one message per axis.
//!
//! Two messages rather than one, because a car's longitudinal and lateral
//! halves are separate problems solved by separate laws, and nothing says
//! one component has to hold both. A plant holds each independently, so a
//! world running a learned follower beside native steering needs no new
//! shape, only a second publisher.
//!
//! Both say *commanded* in the name and in the payload, because commanded
//! is not actual. A car already stopped and still told to brake is doing
//! nothing at all, and the wire should not read as though it were.

use serde::{Deserialize, Serialize};

/// Acceleration a controller asks its plant to hold, m/s^2.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct AccelCmd {
    /// Positive speeds the car up and negative slows it down.
    pub accel_cmd: f64,
}

/// Yaw rate a controller asks its plant to hold, rad/s.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct SteerCmd {
    /// Positive turns counter-clockwise.
    pub yaw_rate_cmd: f64,
}
