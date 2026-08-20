//! What a controller asks a plant for, one message per axis.
//!
//! Two rather than one, because following and steering are separate laws
//! and one component need not hold both. A plant holds each on its own,
//! so a learned follower beside native steering wants a second publisher
//! and no new shape.
//!
//! **Both are normalized to [-1, 1]** and neither carries a unit. A pedal
//! and a steering wheel travel between stops; how much car is behind them
//! is the car's business, so the plant holds the rates and a command says
//! only what fraction of one it wants. A controller that named an
//! acceleration would be asserting something about a vehicle it does not
//! own, and two cars given the same command would have to behave alike.
//!
//! Both say *commanded*, in the key and in the payload, because commanded
//! is not actual: a stopped car told to brake is doing nothing.

use serde::{Deserialize, Serialize};

/// How much of its acceleration a controller asks a plant for.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct AccelCmd {
    /// -1 for the hardest braking the car does, +1 for the hardest
    /// acceleration, 0 for neither.
    pub accel_cmd: f64,
}

/// How much of its turn a controller asks a plant for.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct SteerCmd {
    /// -1 for full right lock, +1 for full left, 0 for straight ahead.
    pub steer_cmd: f64,
}
