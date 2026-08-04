use std::collections::BTreeMap;

use continuo_core::{Component, ComponentId, KeyExpr, Pose, SimDuration, SimTime, StepCtx};
use tracing::info;

/// World-level observer: samples the latest pose per actor and logs it.
///
/// Being a world-level actor, it sees poses with next-step visibility, so
/// an observer receives time-T data strictly after T. Log lines therefore carry
/// the *message's* sim time, not the logger's step time. When an actor is
/// seen for the first time, its earliest received pose (the spawn pose,
/// t = join time) is logged before the latest one.
///
/// After its first step, the logger phase-shifts its schedule by `offset`
/// past the period boundaries (steps at `offset`, `period + offset`, …).
/// With an offset of one publisher period, poses published exactly on a
/// boundary, which same-instant visibility would defer, are already
/// visible when the logger samples.
pub struct PoseLogger {
    period: SimDuration,
    offset: SimDuration,
    latest: BTreeMap<String, (SimTime, Pose)>,
}

impl PoseLogger {
    pub fn new(period: SimDuration, offset: SimDuration) -> Self {
        PoseLogger {
            period,
            offset,
            latest: BTreeMap::new(),
        }
    }
}

fn log_pose(label: &str, time: SimTime, key: &str, pose: &Pose) {
    info!(
        target: "continuo::poses",
        sim_time = %time,
        key,
        x = format_args!("{:.2}", pose.position.x),
        y = format_args!("{:.2}", pose.position.y),
        yaw_deg = format_args!("{:.1}", pose.orientation.yaw().to_degrees()),
        "{label}"
    );
}

impl Component for PoseLogger {
    fn id(&self) -> ComponentId {
        ComponentId::new("logger").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        vec![KeyExpr::new_rooted("*/actor/*/pose").expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> SimTime {
        for message in ctx.inbox() {
            if let Ok(pose) = message.decode::<Pose>() {
                let key = message.key.as_str().to_string();
                // Inbox is (publisher, seq)-sorted, so the first message from
                // a new actor is its earliest pose.
                if !self.latest.contains_key(&key) {
                    log_pose("initial pose", message.time, &key, &pose);
                }
                self.latest.insert(key, (message.time, pose));
            }
        }
        for (key, (time, pose)) in &self.latest {
            log_pose("pose", *time, key, pose);
        }

        // Return the next due time: the phase offset after the first step,
        // then every period.
        if ctx.dt().is_none() {
            // First step (at join time): establish the phase offset.
            ctx.now() + self.offset
        } else {
            ctx.now() + self.period
        }
    }
}
