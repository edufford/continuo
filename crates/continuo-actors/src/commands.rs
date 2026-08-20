//! What a controller asks a plant for, one message per axis.
//!
//! Two rather than one, because following and steering are separate laws
//! and one component need not hold both. A plant holds each on its own,
//! so a learned follower beside native steering wants a second publisher
//! and no new shape.
//!
//! Both say *commanded*, in the key and in the payload, because commanded
//! is not actual: a stopped car told to brake is doing nothing.

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
