use continuo_core::{
    Component, ComponentId, KeyExpr, Pose, Quat, SimDuration, SimTime, StepCtx, Vec3,
};

use crate::controller::Cmd;

/// Planar unicycle kinematics: integrates the latest command
/// (sample-and-hold) and publishes the pose. Publishes `z = 0` and yaw-only
/// quaternions per the pose convention.
pub struct UnicyclePhysics {
    actor_name: String,
    period: SimDuration,
    x: f64,
    y: f64,
    yaw: f64,
    cmd: Cmd,
}

impl UnicyclePhysics {
    pub fn new(actor_name: impl Into<String>, period: SimDuration, initial_pose: Pose) -> Self {
        UnicyclePhysics {
            actor_name: actor_name.into(),
            period,
            x: initial_pose.position.x,
            y: initial_pose.position.y,
            yaw: initial_pose.orientation.yaw(),
            cmd: Cmd::default(),
        }
    }

    fn pose(&self) -> Pose {
        Pose {
            position: Vec3::new(self.x, self.y, 0.0),
            orientation: Quat::from_yaw(self.yaw),
        }
    }
}

impl Component for UnicyclePhysics {
    fn id(&self) -> ComponentId {
        ComponentId::new("physics").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        vec![KeyExpr::new_rooted(format!("*/actor/{}/cmd", self.actor_name)).expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        if let Some(message) = ctx.inbox().last() {
            // TODO(PLAN "Deferred"): a failed decode leaves `self.cmd` alone, so
            // this integrates the previous command indefinitely without saying so.
            if let Ok(cmd) = message.decode::<Cmd>() {
                self.cmd = cmd;
            }
        }

        if let Some(dt) = ctx.dt() {
            let dt = dt.as_secs_f64();
            // Midpoint heading keeps arcs smooth at coarse steps while
            // staying a closed-form deterministic update.
            let mid_yaw = self.yaw + 0.5 * self.cmd.yaw_rate * dt;
            self.x += self.cmd.speed * mid_yaw.cos() * dt;
            self.y += self.cmd.speed * mid_yaw.sin() * dt;
            self.yaw = (self.yaw + self.cmd.yaw_rate * dt).rem_euclid(std::f64::consts::TAU);
        }

        let key = crate::pose_key(ctx.world_name(), &self.actor_name);
        ctx.publish(key, &self.pose()).expect("pose serializes");

        // Return the next due time, one physics period from now.
        ctx.now() + self.period
    }

    /// Example of implementing `state_bytes` to hash internal state, even
    /// though UnicyclePhysics has no meaningful state separate from its
    /// published output.
    fn state_bytes(&self) -> Option<Vec<u8>> {
        #[derive(serde::Serialize)]
        struct State<'a> {
            x: f64,
            y: f64,
            yaw: f64,
            cmd: &'a Cmd,
        }

        // Return the canonical state JSON for the tick fingerprint.
        Some(
            serde_json::to_vec(&State {
                x: self.x,
                y: self.y,
                yaw: self.yaw,
                cmd: &self.cmd,
            })
            .expect("state serializes"),
        )
    }
}
