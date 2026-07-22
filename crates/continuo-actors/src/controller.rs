use std::sync::Arc;

use continuo_core::{Component, ComponentId, KeyExpr, Pose, SimDuration, SimTime, StepCtx};
use serde::{Deserialize, Serialize};

use crate::path::Waypoints;

/// Drive command from a controller to its physics sibling.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Cmd {
    /// Forward speed, m/s.
    pub speed: f64,
    /// Yaw rate, rad/s (positive = counter-clockwise).
    pub yaw_rate: f64,
}

/// Pure-pursuit-flavored path follower: projects the latest pose onto the
/// path, aims at a lookahead point, and commands a clamped yaw rate.
///
/// Declared *before* its physics sibling in the car composite, so its
/// command is delivered same-instant when both are due.
pub struct PathFollowController {
    actor: String,
    path: Arc<Waypoints>,
    period: SimDuration,
    speed: f64,
    lookahead: f64,
    gain: f64,
    max_yaw_rate: f64,
    last_pose: Pose,
}

impl PathFollowController {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        actor: impl Into<String>,
        path: Arc<Waypoints>,
        period: SimDuration,
        speed: f64,
        lookahead: f64,
        gain: f64,
        max_yaw_rate: f64,
        initial_pose: Pose,
    ) -> Self {
        PathFollowController {
            actor: actor.into(),
            path,
            period,
            speed,
            lookahead,
            gain,
            max_yaw_rate,
            last_pose: initial_pose,
        }
    }
}

fn wrap_pi(angle: f64) -> f64 {
    let wrapped = angle.rem_euclid(std::f64::consts::TAU);

    // Return the equivalent angle in (-pi, pi].
    if wrapped > std::f64::consts::PI {
        wrapped - std::f64::consts::TAU
    } else {
        wrapped
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
        vec![KeyExpr::new(format!("continuo/*/actor/{}/pose", self.actor)).expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        // Latest pose wins; inbox is (publisher, seq)-sorted and all pose
        // messages here come from our physics sibling.
        if let Some(message) = ctx.inbox().last() {
            if let Ok(pose) = message.decode::<Pose>() {
                self.last_pose = pose;
            }
        }

        let position = self.last_pose.position;
        let s = self.path.project(position.x, position.y);
        let target = self.path.point_at(s + self.lookahead);
        let desired_heading = f64::atan2(target.y - position.y, target.x - position.x);
        let heading_error = wrap_pi(desired_heading - self.last_pose.orientation.yaw());
        let cmd = Cmd {
            speed: self.speed,
            yaw_rate: (self.gain * heading_error).clamp(-self.max_yaw_rate, self.max_yaw_rate),
        };

        let key = crate::cmd_key(ctx.world(), &self.actor);
        ctx.publish(key, &cmd).expect("cmd serializes");

        // Return the next due time, one control period from now.
        ctx.now() + self.period
    }
}
