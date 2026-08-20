use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, Pose, Quat, SimDuration, SimTime, StepCtx, Vec3,
};
use serde::{Deserialize, Serialize};

use crate::commands::{AccelCmd, SteerCmd};

/// Where a car is and how fast it is going.
///
/// Flat, so `position` and `orientation` sit where [`Pose`] puts them and
/// anything reading a pose off this key goes on reading one.
///
/// It is what a plant is built with and what a plant publishes, and when
/// scenarios come from files it is what those files carry.
///
/// TODO(PLAN "Features"): this is the plant's integrator state, not its
/// kinematic state. Acceleration and yaw rate belong beside these, and
/// adding them wants a message of its own rather than a `pose` key that
/// has grown fields.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct CarState {
    /// Where the car is, meters, `z` always zero for a planar model.
    pub position: Vec3,
    /// Which way it points, a yaw-only unit quaternion.
    pub orientation: Quat,
    /// How fast it is going, m/s, never negative.
    pub speed: f64,
}

impl CarState {
    /// A car at `pose` doing `speed`.
    pub fn new(pose: Pose, speed: f64) -> Self {
        CarState {
            position: pose.position,
            orientation: pose.orientation,
            speed,
        }
    }

    /// Where the car is, without the speed.
    pub fn pose(&self) -> Pose {
        Pose {
            position: self.position,
            orientation: self.orientation,
        }
    }
}

/// Planar unicycle kinematics: integrates the commands it is holding, an
/// acceleration and a yaw rate, and publishes where that put the car,
/// with the speed it owns beside the pose. Publishes `z = 0` and yaw-only
/// quaternions per the pose convention.
///
/// Each command is held on its own key, so a car whose controller
/// publishes one goes on integrating whatever the other last said.
/// Silence is hold rather than stop, which is what lets a constant-speed
/// car have no longitudinal publisher: the held zero stands, and the
/// speed it was built with is the speed it keeps.
pub struct UnicyclePhysics {
    actor_name: String,
    period: SimDuration,
    x: f64,
    y: f64,
    yaw: f64,
    speed: f64,
    accel_cmd: f64,
    yaw_rate_cmd: f64,
}

impl UnicyclePhysics {
    pub fn new(actor_name: impl Into<String>, period: SimDuration, initial: CarState) -> Self {
        UnicyclePhysics {
            actor_name: actor_name.into(),
            period,
            x: initial.position.x,
            y: initial.position.y,
            yaw: initial.orientation.yaw(),
            speed: initial.speed,
            accel_cmd: 0.0,
            yaw_rate_cmd: 0.0,
        }
    }

    /// Applies the newest message on each command key.
    ///
    /// Newest *per key* rather than newest overall, because holding one
    /// command must not depend on whether the other spoke. The inbox is
    /// read backwards so the first match on a key wins, which leaves the
    /// older ones undecoded rather than decoded and overwritten.
    fn take_commands(&mut self, ctx: &StepCtx) -> Result<(), CoreError> {
        let mut took_accel = false;
        let mut took_steer = false;
        for message in ctx.inbox().iter().rev() {
            // A command that cannot be read stops the world. Keeping the
            // previous one would integrate it indefinitely without saying so.
            if !took_accel && message.key.as_str().ends_with("/accel_cmd") {
                self.accel_cmd = message.decode::<AccelCmd>()?.accel_cmd;
                took_accel = true;
            } else if !took_steer && message.key.as_str().ends_with("/steer_cmd") {
                self.yaw_rate_cmd = message.decode::<SteerCmd>()?.yaw_rate_cmd;
                took_steer = true;
            }
            if took_accel && took_steer {
                break;
            }
        }

        // Return success: both commands are as new as the inbox allows.
        Ok(())
    }

    /// Advances the model by `dt` seconds on the commands it is holding.
    fn advance(&mut self, dt: f64) {
        // Speed first, so the step travels at what was asked for rather
        // than at what the last one ended on. It stops at zero because
        // this model has no reverse: a brake held past a standstill would
        // otherwise drive the car back up the road.
        self.speed = (self.speed + self.accel_cmd * dt).max(0.0);
        // Midpoint heading keeps arcs smooth at coarse steps while staying
        // a closed-form deterministic update.
        let mid_yaw = self.yaw + 0.5 * self.yaw_rate_cmd * dt;
        self.x += self.speed * mid_yaw.cos() * dt;
        self.y += self.speed * mid_yaw.sin() * dt;
        self.yaw = (self.yaw + self.yaw_rate_cmd * dt).rem_euclid(std::f64::consts::TAU);
    }

    /// Where the car is and how fast, which is all it publishes.
    fn state(&self) -> CarState {
        CarState {
            position: Vec3::new(self.x, self.y, 0.0),
            orientation: Quat::from_yaw(self.yaw),
            speed: self.speed,
        }
    }
}

impl Component for UnicyclePhysics {
    fn id(&self) -> ComponentId {
        ComponentId::new("physics").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        // World segment wildcarded for the reason `PathFollowController`
        // gives, and its TODO covers this one too.
        vec![
            KeyExpr::new_rooted(format!("*/actor/{}/accel_cmd", self.actor_name))
                .expect("valid key"),
            KeyExpr::new_rooted(format!("*/actor/{}/steer_cmd", self.actor_name))
                .expect("valid key"),
        ]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        self.take_commands(ctx)?;

        if let Some(dt) = ctx.dt() {
            self.advance(dt.as_secs_f64());
        }

        let key = crate::pose_key(ctx.world_name(), &self.actor_name);
        ctx.publish(key, &self.state())?;

        // Return the next due time, one physics period from now.
        Ok(ctx.now() + self.period)
    }

    /// The integrator state, which is everything the plant carries that
    /// the next step depends on.
    ///
    /// The held commands are not in it. They arrive as messages the
    /// fingerprint already covers, so hashing them here would be counting
    /// the same bytes twice.
    fn state_bytes(&self) -> Option<Vec<u8>> {
        #[derive(serde::Serialize)]
        struct State {
            x: f64,
            y: f64,
            yaw: f64,
            speed: f64,
        }

        // Return the canonical state JSON for the tick fingerprint.
        Some(
            serde_json::to_vec(&State {
                x: self.x,
                y: self.y,
                yaw: self.yaw,
                speed: self.speed,
            })
            .expect("state serializes"),
        )
    }
}

#[cfg(test)]
mod tests {
    use continuo_core::{ComponentPath, Message};

    use super::*;

    /// The period every test here steps at, and its length in seconds,
    /// which every expected value below is worked from.
    const PERIOD: SimDuration = SimDuration::from_millis(10);
    const STEP_SECS: f64 = 0.01;

    /// How many advancing steps a test takes unless it needs longer.
    const STEPS: i64 = 100;

    /// The speed a car starts at where the test is not about starting
    /// from rest.
    const CRUISE_SPD: f64 = 7.0;

    /// Where a car starts where the test is not about where it starts:
    /// off the origin and pointing along `+x`, so a plant that lost the
    /// pose it was built with fails rather than passing.
    fn start_pose() -> Pose {
        Pose {
            position: Vec3::new(12.5, -3.5, 0.0),
            orientation: Quat::from_yaw(0.0),
        }
    }

    /// A car at [`start_pose`] doing `speed`.
    fn plant(speed: f64) -> UnicyclePhysics {
        // Return a plant holding neither command, as one is before it is
        // spoken to.
        UnicyclePhysics::new("car", PERIOD, CarState::new(start_pose(), speed))
    }

    /// One command message for that car, on the key its last segment
    /// names.
    ///
    /// `seq` is the publisher's own counter, so the two helpers below
    /// share one sequence: every command here comes from the same
    /// controller, and a publisher numbers its messages in the order it
    /// sent them whatever key each went to.
    fn command<T: Serialize>(tail: &str, seq: u64, value: &T) -> Message {
        // Return the message as the transport would deliver it.
        Message {
            key: KeyExpr::new_rooted(format!("w/actor/car/{tail}")).expect("valid key"),
            publisher: ComponentPath::parse("car/controller").expect("valid path"),
            seq,
            sim_time: SimTime::ZERO,
            payload: serde_json::to_vec(value).expect("a command serializes"),
        }
    }

    fn accel(seq: u64, accel_cmd: f64) -> Message {
        command("accel_cmd", seq, &AccelCmd { accel_cmd })
    }

    fn steer(seq: u64, yaw_rate_cmd: f64) -> Message {
        command("steer_cmd", seq, &SteerCmd { yaw_rate_cmd })
    }

    /// Steps `plant` once and hands back the payload it published.
    fn step_at(
        plant: &mut UnicyclePhysics,
        now: SimTime,
        dt: Option<SimDuration>,
        inbox: Vec<Message>,
    ) -> Vec<u8> {
        let mut ctx = StepCtx::new(now, dt, "w", 0, inbox);
        plant
            .step(&mut ctx)
            .expect("a plant given readable commands steps");
        let mut outbox = ctx.take_outbox();
        assert_eq!(
            outbox.len(),
            1,
            "a step publishes one pose and nothing else"
        );
        let (key, payload) = outbox.remove(0);
        assert_eq!(key.as_str(), "continuo/w/actor/car/pose");

        // Return the bytes, for a test to read as whatever it is checking.
        payload
    }

    /// The state `plant` published after its first step and `steps`
    /// advancing ones, with `inbox` waiting at the first and nothing
    /// after.
    fn run(plant: &mut UnicyclePhysics, steps: i64, inbox: Vec<Message>) -> CarState {
        let mut payload = step_at(plant, SimTime::ZERO, None, inbox);
        for step in 1..=steps {
            payload = step_at(plant, SimTime::from_millis(step * 10), Some(PERIOD), vec![]);
        }

        // Return where that left the car.
        serde_json::from_slice(&payload).expect("a car state")
    }

    /// How far a car got along `+x`, which is what the tests measure
    /// rather than an absolute position.
    fn travelled(state: &CarState) -> f64 {
        state.position.x - start_pose().position.x
    }

    #[test]
    fn an_initial_state_round_trips_through_json() {
        // Short decimals throughout, so this checks that serde maps the
        // fields the way it says it does rather than checking a parser.
        //
        // TODO(PLAN "Determinism and correctness"): `from_str` into an
        // `f64` misses what `to_string` produced by an ulp for about one
        // number in eight, so a realistic value here would fail. Every
        // component decoding a pose has the same gap.
        let state = CarState::new(
            Pose {
                position: Vec3::new(3.0, -4.0, 0.0),
                orientation: Quat {
                    w: 0.6,
                    x: 0.0,
                    y: 0.0,
                    z: 0.8,
                },
            },
            CRUISE_SPD,
        );
        let text = serde_json::to_string(&state).expect("a state serializes");
        assert_eq!(
            serde_json::from_str::<CarState>(&text).expect("and deserializes"),
            state
        );

        // The field order is the contract. `position` and `orientation`
        // come first and in `Pose`'s own shape, so a pose decoder finds
        // what it expects and the speed sits past the end of it.
        //
        // The tail is built through `to_string` rather than written out,
        // because Rust's own `Display` for a float and serde's are not the
        // same: 7.0 prints as `7` one way and `7.0` the other, so spelling
        // the number here would be asserting the wrong text.
        let speed = serde_json::to_string(&CRUISE_SPD).expect("a speed serializes");
        assert!(text.starts_with("{\"position\":{"), "{text}");
        assert!(text.ends_with(&format!(",\"speed\":{speed}}}")), "{text}");
    }

    #[test]
    fn a_pose_decoder_reads_what_the_plant_publishes() {
        let mut plant = plant(CRUISE_SPD);
        let payload = step_at(&mut plant, SimTime::ZERO, None, vec![]);

        // The whole compatibility claim, in two readings of one payload:
        // taken as a pose it is the pose, and taken as a car state the
        // speed is there for whoever does want it.
        let pose: Pose = serde_json::from_slice(&payload).expect("still a pose");
        assert_eq!(pose, start_pose());
        let state: CarState = serde_json::from_slice(&payload).expect("and a car state");
        assert_eq!(state.speed, CRUISE_SPD);
    }

    #[test]
    fn a_car_nobody_commands_holds_the_speed_it_was_built_with() {
        // A whole second of nothing said to it, which is the
        // constant-speed car the demo is full of.
        let state = run(&mut plant(CRUISE_SPD), STEPS, vec![]);
        assert_eq!(state.speed, CRUISE_SPD);
        let expected = CRUISE_SPD * STEPS as f64 * STEP_SECS;
        assert!((travelled(&state) - expected).abs() < 1e-9, "{state:?}");
        assert_eq!(state.position.y, start_pose().position.y);
    }

    #[test]
    fn held_acceleration_integrates_into_speed() {
        const ACCEL: f64 = 2.0;

        // Said once and never again, so what the car does for the other
        // ninety-nine steps is what holding it means.
        let state = run(&mut plant(0.0), STEPS, vec![accel(1, ACCEL)]);
        let seconds = STEPS as f64 * STEP_SECS;
        assert!((state.speed - ACCEL * seconds).abs() < 1e-9, "{state:?}");

        // The step integrates speed before it travels, so what it covers
        // is the sum of the speeds each step ended at rather than the
        // a*t^2/2 of the closed form, which is half a step behind it.
        let steps = STEPS as f64;
        let summed = ACCEL * STEP_SECS * STEP_SECS * steps * (steps + 1.0) / 2.0;
        assert!((travelled(&state) - summed).abs() < 1e-9, "{state:?}");
    }

    #[test]
    fn speed_never_integrates_below_zero() {
        const BRAKE: f64 = -10.0;

        // Braking held for twice as long as stopping takes, which is the
        // case that would reverse a car taking the arithmetic at its word.
        let braking_steps = 2 * (CRUISE_SPD / -BRAKE / STEP_SECS) as i64;
        let mut plant = plant(CRUISE_SPD);
        let stopped = run(&mut plant, braking_steps, vec![accel(1, BRAKE)]);
        assert_eq!(stopped.speed, 0.0);

        // And a stopped car goes nowhere, however long the brake is held.
        let still_stopped = run(&mut plant, STEPS, vec![]);
        assert_eq!(still_stopped.speed, 0.0);
        assert_eq!(still_stopped.position.x, stopped.position.x);
    }

    #[test]
    fn accel_and_steer_are_held_independently() {
        const FIRST_ACCEL: f64 = 1.0;
        const SECOND_ACCEL: f64 = 4.0;
        const YAW_RATE: f64 = 0.5;

        let mut plant = plant(CRUISE_SPD);
        step_at(
            &mut plant,
            SimTime::ZERO,
            None,
            vec![accel(1, FIRST_ACCEL), steer(2, YAW_RATE)],
        );

        // One step where only the acceleration is restated. The new one
        // takes effect, and the yaw rate nobody mentioned goes on turning
        // the car, which is what holding each on its own key buys.
        let payload = step_at(
            &mut plant,
            SimTime::from_millis(10),
            Some(PERIOD),
            vec![accel(3, SECOND_ACCEL)],
        );
        let state: CarState = serde_json::from_slice(&payload).expect("a car state");
        let expected_speed = CRUISE_SPD + SECOND_ACCEL * STEP_SECS;
        assert!((state.speed - expected_speed).abs() < 1e-9, "{state:?}");
        let expected_yaw = YAW_RATE * STEP_SECS;
        assert!(
            (state.orientation.yaw() - expected_yaw).abs() < 1e-9,
            "{state:?}"
        );
    }

    #[test]
    fn only_the_newest_command_on_a_key_is_applied() {
        const ACCEL_CMD_1: f64 = -10.0;
        const ACCEL_CMD_2: f64 = 0.0;
        const ACCEL_CMD_3: f64 = 2.0;

        // Three accelerations in one inbox, which is what a controller
        // running faster than its plant delivers. The car takes the last
        // and never the ones it overtook, so a step samples the command
        // rather than replaying every one that was sent.
        let mut plant = plant(0.0);
        step_at(
            &mut plant,
            SimTime::ZERO,
            None,
            vec![
                accel(1, ACCEL_CMD_1),
                accel(2, ACCEL_CMD_2),
                accel(3, ACCEL_CMD_3),
            ],
        );
        let payload = step_at(&mut plant, SimTime::from_millis(10), Some(PERIOD), vec![]);
        let state: CarState = serde_json::from_slice(&payload).expect("a car state");
        assert!(
            (state.speed - ACCEL_CMD_3 * STEP_SECS).abs() < 1e-9,
            "{state:?}"
        );
    }
}
