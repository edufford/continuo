use continuo_core::{
    Component, ComponentId, CoreError, KeyExpr, Pose, Quat, SimDuration, SimTime, StepCtx, Vec3,
};
use serde::{Deserialize, Serialize};

use crate::commands::{AccelCmd, SteerCmd};

/// Where a car is and how fast it is going.
///
/// One struct doing two jobs, because both are the same four numbers: it
/// is what a plant is built with, and it is what a plant publishes. When
/// scenarios come from files rather than from Rust, it is what those files
/// carry as well, so its serde form is part of the contract rather than an
/// implementation detail.
///
/// The fields are flat, and that is the load-bearing part: `position` and
/// `orientation` sit exactly where [`Pose`] puts them, so everything
/// already reading a pose off this key goes on reading one and ignores the
/// speed beside it.
///
/// Speed is here and acceleration is not. Speed is state, integrated by
/// the plant and visible nowhere else. Acceleration is a held command, so
/// the first message to arrive replaces whatever the plant was built with,
/// and zero is the right start for a car nobody commands.
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
    /// A car standing at `pose` and doing `speed`.
    pub fn new(pose: Pose, speed: f64) -> Self {
        CarState {
            position: pose.position,
            orientation: pose.orientation,
            speed,
        }
    }

    /// Where the car is, without the speed, for whoever wants only that.
    pub fn pose(&self) -> Pose {
        Pose {
            position: self.position,
            orientation: self.orientation,
        }
    }
}

/// Planar unicycle kinematics: integrates the acceleration and the yaw
/// rate it is holding, and publishes where that has put the car. Publishes
/// `z = 0` and yaw-only quaternions per the pose convention.
///
/// **It owns speed**, which is why what it publishes carries one. Nothing
/// else can say how fast a car is going: a controller knows only what it
/// asked for, and the clamp at zero parts that from what happened.
///
/// The two commands are held separately, each replaced only by a message
/// on its own key, so a car whose controller publishes one goes on
/// integrating whatever the other last said. Silence is hold rather than
/// stop, which is what lets a constant-speed car have no longitudinal
/// publisher at all: nobody commands an acceleration, the held zero
/// stands, and the speed it was built with is the speed it keeps.
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
    /// Newest *per key* rather than newest overall, because the two
    /// commands arrive separately and holding one must not depend on
    /// whether the other spoke this step. The inbox is read backwards so
    /// the first match on a key is the one that wins, which leaves the
    /// older messages on that key undecoded rather than decoded and then
    /// overwritten.
    ///
    /// The last path segment is what tells the two apart, which is exact
    /// rather than a guess: [`Self::subscriptions`] admits these two keys
    /// and nothing else.
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

        // Return success: both commands are now as new as the inbox allows.
        Ok(())
    }

    /// Advances the model by `dt` seconds on the commands it is holding.
    fn advance(&mut self, dt: f64) {
        // Speed first, so the step travels at what was asked for rather
        // than at what the previous one ended on. It stops at zero because
        // this model has no reverse gear, and a brake held past a
        // standstill would otherwise drive the car back up the road.
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

    /// The integrator state and both held commands.
    ///
    /// The commands are in it because they are state the plant carries and
    /// nothing publishes: a car holding a brake and a car holding nothing
    /// publish the same pose for one step and only then part. Hashing what
    /// is held catches that at the step where they diverge rather than at
    /// the step where it starts to show.
    fn state_bytes(&self) -> Option<Vec<u8>> {
        #[derive(serde::Serialize)]
        struct State {
            x: f64,
            y: f64,
            yaw: f64,
            speed: f64,
            accel_cmd: f64,
            yaw_rate_cmd: f64,
        }

        // Return the canonical state JSON for the tick fingerprint.
        Some(
            serde_json::to_vec(&State {
                x: self.x,
                y: self.y,
                yaw: self.yaw,
                speed: self.speed,
                accel_cmd: self.accel_cmd,
                yaw_rate_cmd: self.yaw_rate_cmd,
            })
            .expect("state serializes"),
        )
    }
}

#[cfg(test)]
mod tests {
    use continuo_core::{ComponentPath, Message};

    use super::*;

    /// The period every test here steps at.
    const PERIOD: SimDuration = SimDuration::from_millis(10);

    /// A car at the origin pointing along `+x`, doing `speed`.
    fn plant(speed: f64) -> UnicyclePhysics {
        // Return a plant holding neither command, as one is before anyone
        // has spoken to it.
        UnicyclePhysics::new("car", PERIOD, CarState::new(Pose::default(), speed))
    }

    /// One command message for that car, on the key its last segment names.
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

        // Return the bytes, so a test can read them as whatever it is
        // checking they can be read as.
        payload
    }

    /// The state `plant` published after its first step and `steps`
    /// advancing ones, with `inbox` waiting at the first and nothing after.
    fn run(plant: &mut UnicyclePhysics, steps: i64, inbox: Vec<Message>) -> CarState {
        let mut payload = step_at(plant, SimTime::ZERO, None, inbox);
        for step in 1..=steps {
            payload = step_at(plant, SimTime::from_millis(step * 10), Some(PERIOD), vec![]);
        }

        // Return where that left the car.
        serde_json::from_slice(&payload).expect("a car state")
    }

    #[test]
    fn an_initial_state_round_trips_through_json() {
        // Short decimals throughout, so what is being checked is that
        // serde maps the fields the way this says it does. Longer ones
        // would drag `serde_json`'s own float parsing into a test that is
        // not about it: `from_str` into an `f64` is a bit off the value
        // `to_string` was given for about one number in eight.
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
            21.5,
        );
        let text = serde_json::to_string(&state).expect("a state serializes");
        assert_eq!(
            serde_json::from_str::<CarState>(&text).expect("and deserializes"),
            state
        );

        // Flat, and speed last, because that is what leaves a pose sitting
        // where a pose sits. A nested `pose` field would round-trip just as
        // well and be readable by nothing that reads poses today.
        assert!(text.starts_with("{\"position\":{"), "{text}");
        assert!(text.ends_with(",\"speed\":21.5}"), "{text}");
    }

    #[test]
    fn a_pose_decoder_reads_what_the_plant_publishes() {
        let mut plant = plant(9.0);
        let payload = step_at(&mut plant, SimTime::ZERO, None, vec![]);

        // The whole compatibility claim in one assertion: the plant now
        // publishes a speed as well, and everything reading this key as a
        // pose reads it unchanged and ignores what it did not ask for.
        let pose: Pose = serde_json::from_slice(&payload).expect("still a pose");
        assert_eq!(pose, Pose::default());
        assert!(
            String::from_utf8(payload).expect("utf-8").contains("speed"),
            "the speed has to be there for anyone who does want it"
        );
    }

    #[test]
    fn a_car_nobody_commands_holds_the_speed_it_was_built_with() {
        // A whole second of nothing being said to it, which is the
        // constant-speed car the demo is full of.
        let state = run(&mut plant(7.0), 100, vec![]);
        assert_eq!(state.speed, 7.0);
        assert!((state.position.x - 7.0).abs() < 1e-9, "{state:?}");
        assert_eq!(state.position.y, 0.0);
    }

    #[test]
    fn held_acceleration_integrates_into_speed() {
        // Said once and then never again, so what the car does for the
        // remaining ninety-nine steps is what holding it means.
        let state = run(&mut plant(0.0), 100, vec![accel(1, 2.0)]);
        assert!((state.speed - 2.0).abs() < 1e-9, "{state:?}");

        // From rest under a steady acceleration the distance is a*t^2/2,
        // and the step integrates speed before it travels, so it runs a
        // little ahead of the closed form rather than behind it.
        assert!(
            (state.position.x - 1.01).abs() < 1e-9,
            "{state:?} against 1.01 m"
        );
    }

    #[test]
    fn speed_never_integrates_below_zero() {
        // Braking hard enough to stop in a tenth of a second, held for two
        // seconds, which is the case that would reverse a car that took the
        // arithmetic at its word.
        let mut plant = plant(1.0);
        let stopped = run(&mut plant, 200, vec![accel(1, -10.0)]);
        assert_eq!(stopped.speed, 0.0);

        // And a stopped car goes nowhere, however long the brake is held.
        let still_stopped = run(&mut plant, 100, vec![]);
        assert_eq!(still_stopped.speed, 0.0);
        assert_eq!(still_stopped.position.x, stopped.position.x);
    }

    #[test]
    fn accel_and_steer_are_held_independently() {
        let mut plant = plant(10.0);
        step_at(
            &mut plant,
            SimTime::ZERO,
            None,
            vec![accel(1, 1.0), steer(2, 0.5)],
        );

        // One step where only the acceleration is restated. The new one
        // takes effect, and the yaw rate nobody mentioned goes on turning
        // the car, which is what holding each on its own key buys.
        let payload = step_at(
            &mut plant,
            SimTime::from_millis(10),
            Some(PERIOD),
            vec![accel(3, 4.0)],
        );
        let state: CarState = serde_json::from_slice(&payload).expect("a car state");
        assert!((state.speed - 10.04).abs() < 1e-9, "{state:?}");
        assert!(
            (state.orientation.yaw() - 0.005).abs() < 1e-9,
            "{state:?} against 0.5 rad/s for 10 ms"
        );
    }
}
