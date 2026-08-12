use std::sync::Arc;

use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, Pose, SimDuration, SimTime, StepCtx,
};
use serde::{Deserialize, Serialize};

use crate::control_laws::{PurePursuitParams, pure_pursuit_yaw_rate};
use crate::path::Waypoints;

/// Drive command from a controller to its physics sibling.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Cmd {
    /// Forward speed, m/s.
    pub speed: f64,
    /// Yaw rate, rad/s (positive = counter-clockwise).
    pub yaw_rate: f64,
}

/// Path follower: reads the latest pose, asks
/// [`pure_pursuit_yaw_rate`] where to steer, and publishes that with the
/// speed it was built with.
///
/// Follows the road in **Frenet coordinates**: an arc length `s` found by
/// projection, and a fixed lateral offset it holds. So every car on a road
/// shares one [`Waypoints`], and a lane is a number rather than a curve of
/// its own. Pass `0.0` to drive the road itself.
///
/// Declared *before* its physics sibling in the car composite, so its
/// command is delivered same-instant when both are due.
pub struct PathFollowController {
    actor_name: String,
    road: Arc<Waypoints>,
    period: SimDuration,
    speed: f64,
    pursuit: PurePursuitParams,
    last_pose: Pose,
}

impl PathFollowController {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        actor_name: impl Into<String>,
        road: Arc<Waypoints>,
        lateral: f64,
        period: SimDuration,
        speed: f64,
        lookahead: f64,
        gain: f64,
        max_yaw_rate: f64,
        initial_pose: Pose,
    ) -> Self {
        PathFollowController {
            actor_name: actor_name.into(),
            road,
            period,
            speed,
            pursuit: PurePursuitParams {
                lateral,
                lookahead,
                gain,
                max_yaw_rate,
            },
            last_pose: initial_pose,
        }
    }
}

impl Component for PathFollowController {
    fn id(&self) -> ComponentId {
        ComponentId::new("controller").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        // World segment wildcarded: the world name is only known at step time.
        // TODO(PLAN "Scenario configuration"): once scenarios instantiate
        // components, pass the world name at construction and subscribe
        // precisely (same in UnicyclePhysics).
        vec![KeyExpr::new_rooted(format!("*/actor/{}/pose", self.actor_name)).expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        // Latest pose wins; inbox is (publisher, seq)-sorted and all pose
        // messages here come from our physics sibling.
        if let Some(message) = ctx.inbox().last() {
            // A pose that cannot be read stops the world. Keeping the previous
            // one would go on steering from it without saying so.
            self.last_pose = message.decode::<Pose>()?;
        }

        let cmd = Cmd {
            speed: self.speed,
            yaw_rate: pure_pursuit_yaw_rate(&self.road, self.last_pose, self.pursuit),
        };

        let key = crate::cmd_key(ctx.world_name(), &self.actor_name);
        ctx.publish(key, &cmd)?;

        // Return the next due time, one control period from now.
        Ok(ctx.now() + self.period)
    }
}
