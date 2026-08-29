use std::sync::Arc;

use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, Pose, SimDuration, SimTime, StepCtx,
};

use crate::commands::SteerCmd;
use crate::control_laws::{PurePursuitParams, pure_pursuit_yaw_rate, steer_fraction};
use crate::path::Waypoints;
use crate::physics::DriveLimits;

/// Path follower: reads the latest pose, asks
/// [`pure_pursuit_yaw_rate`] where to steer, and publishes that.
///
/// Lateral only. Speed is the plant's business, and a car with nobody
/// commanding an acceleration holds the one it was built with.
///
/// What it publishes is normalized against the [`DriveLimits`] it is
/// given, which are the plant's, while the law's own `max_yaw_rate` is
/// tuning: set it below the plant's and the car holds a gentler turn than
/// it could. Hand the two halves of a car different limits and it turns
/// at a rate nobody intended, which nothing here can detect, because a
/// normalized command carries no unit to disagree about.
///
/// [`DriveLimits`]: crate::DriveLimits
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
    pursuit_params: PurePursuitParams,
    /// The plant being commanded, of which only the turn is read here.
    /// A controller that commanded a speed would want the rest.
    limits: DriveLimits,
    last_pose: Pose,
}

impl PathFollowController {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        actor_name: impl Into<String>,
        road: Arc<Waypoints>,
        lateral_tgt: f64,
        period: SimDuration,
        lookahead: f64,
        gain_yaw_rate: f64,
        max_yaw_rate: f64,
        limits: DriveLimits,
        initial_pose: Pose,
    ) -> Self {
        PathFollowController {
            actor_name: actor_name.into(),
            road,
            period,
            pursuit_params: PurePursuitParams {
                lateral_tgt,
                lookahead,
                gain_yaw_rate,
                max_yaw_rate,
            },
            limits,
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
        // precisely (same in `UnicyclePhysics`).
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

        let yaw_rate = pure_pursuit_yaw_rate(&self.road, self.last_pose, self.pursuit_params);
        let cmd = SteerCmd {
            steer_cmd: steer_fraction(yaw_rate, self.limits.yaw_rate_max),
        };

        let key = crate::steer_cmd_key(ctx.world_name(), &self.actor_name);
        ctx.publish(key, &cmd)?;

        // Return the next due time, one control period from now.
        Ok(ctx.now() + self.period)
    }
}
