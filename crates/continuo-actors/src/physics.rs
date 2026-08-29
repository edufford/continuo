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
    /// Where the car is, in meters.
    pub position: Vec3,
    /// Which way it points, a unit quaternion.
    pub orientation: Quat,
    /// How fast it is going along its heading, m/s.
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

/// What a normalized command is worth on a particular car.
///
/// [`AccelCmd`] and [`SteerCmd`] carry a fraction; these are the rates it
/// is a fraction of. Two cars given one command differ exactly as far as
/// these do.
///
/// Braking has its own limit because a car brakes harder than it
/// accelerates. One number for both would get one of them wrong.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveLimits {
    /// m/s^2 at a command of +1.
    pub accel_max: f64,
    /// m/s^2 at a command of -1, written positive.
    pub decel_max: f64,
    /// rad/s at a steer command of +1.
    pub yaw_rate_max: f64,
}

impl DriveLimits {
    /// The car this project drives, at ordinary passenger-car rates.
    pub const fn highway_car() -> Self {
        DriveLimits {
            accel_max: 3.0,
            decel_max: 5.0,
            yaw_rate_max: 1.2,
        }
    }
}

/// Planar unicycle kinematics: turns the commands it is holding into an
/// acceleration and a yaw rate, integrates those, and publishes where
/// that put the car, with the speed it owns beside the pose. Publishes
/// `z = 0` and yaw-only quaternions per the pose convention.
///
/// It also owns [`DriveLimits`], which is what makes a normalized command
/// mean something. A controller asks for a fraction and the plant decides
/// what fraction of what, so the same command drives a hatchback and a
/// truck differently without either controller knowing which it has.
///
/// Each command is held on its own key, so a car whose controller
/// publishes one goes on integrating whatever the other last said.
/// Silence is hold rather than stop, which is what lets a constant-speed
/// car have no longitudinal publisher: the held zero stands, and the
/// speed it was built with is the speed it keeps.
pub struct UnicyclePhysics {
    actor_name: String,
    period: SimDuration,
    limits: DriveLimits,
    x: f64,
    y: f64,
    yaw: f64,
    speed: f64,
    accel_cmd: f64,
    steer_cmd: f64,
}

impl UnicyclePhysics {
    pub fn new(
        actor_name: impl Into<String>,
        period: SimDuration,
        limits: DriveLimits,
        initial: CarState,
    ) -> Self {
        UnicyclePhysics {
            actor_name: actor_name.into(),
            period,
            limits,
            x: initial.position.x,
            y: initial.position.y,
            yaw: initial.orientation.yaw(),
            speed: initial.speed,
            accel_cmd: 0.0,
            steer_cmd: 0.0,
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
                self.steer_cmd = message.decode::<SteerCmd>()?.steer_cmd;
                took_steer = true;
            }
            if took_accel && took_steer {
                break;
            }
        }

        // Return success: both commands are as new as the inbox allows.
        Ok(())
    }

    /// The acceleration a held command asks for, m/s^2.
    ///
    /// Clamped at the stops, because a command past them is a controller
    /// asking for a car it does not have.
    fn commanded_accel(&self) -> f64 {
        let fraction = self.accel_cmd.clamp(-1.0, 1.0);

        // Return it against whichever limit it is a fraction of, since a
        // car brakes harder than it accelerates.
        if fraction < 0.0 {
            fraction * self.limits.decel_max
        } else {
            fraction * self.limits.accel_max
        }
    }

    /// The yaw rate a held command asks for, rad/s.
    ///
    /// Clamped at the stops for the reason [`Self::commanded_accel`]
    /// gives. One limit rather than two, since a car turns as hard one way
    /// as the other.
    fn commanded_yaw_rate(&self) -> f64 {
        self.steer_cmd.clamp(-1.0, 1.0) * self.limits.yaw_rate_max
    }

    /// Advances the model by `dt` seconds on the commands it is holding.
    fn advance(&mut self, dt: f64) {
        // Speed first, so the step travels at what was asked for rather
        // than at what the last one ended on. It stops at zero because
        // this model has no reverse: a brake held past a standstill would
        // otherwise drive the car back up the road.
        self.speed = (self.speed + self.commanded_accel() * dt).max(0.0);
        let yaw_rate = self.commanded_yaw_rate();
        // Midpoint heading keeps arcs smooth at coarse steps while staying
        // a closed-form deterministic update.
        let mid_yaw = self.yaw + 0.5 * yaw_rate * dt;
        self.x += self.speed * libm::cos(mid_yaw) * dt;
        self.y += self.speed * libm::sin(mid_yaw) * dt;
        // TODO(PLAN "Determinism and correctness"): folding into
        // [0, TAU) rounds a negative angle where it leaves a positive one
        // alone, so two cars mirrored about the road do not stay mirrored.
        self.yaw = (self.yaw + yaw_rate * dt).rem_euclid(std::f64::consts::TAU);
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

    /// The integrator state: where the car is and how fast.
    ///
    /// Not the held commands. They are copies of what reached the plant,
    /// and every published command is in the fingerprint already. A
    /// divergence in what arrived rather than in what was sent would wait
    /// for the pose to show it, but that is true of any component holding
    /// a decoded input: this hook is for state a component makes and does
    /// not publish.
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
    use crate::control_laws::{accel_fraction, steer_fraction};

    /// The period every test here steps at, and its length in seconds,
    /// which every expected value below is worked from.
    const PERIOD: SimDuration = SimDuration::from_millis(10);
    const STEP_SECS: f64 = 0.01;

    /// How many advancing steps a test takes unless it needs longer.
    const STEPS: i64 = 100;

    /// The speed a car starts at where the test is not about starting
    /// from rest.
    const CRUISE_SPD: f64 = 7.0;

    /// What a full command is worth on every car here, so an expected
    /// value is worked from a command and a limit rather than written
    /// down.
    const LIMITS: DriveLimits = DriveLimits::highway_car();

    /// Where a car starts where the test is not about where it starts:
    /// off the origin and pointing along `+x`, so a plant that lost the
    /// pose it was built with fails rather than passing.
    ///
    /// Short decimals on purpose, and the same goes for [`CRUISE_SPD`].
    /// `an_initial_state_round_trips_through_json` compares a serde round
    /// trip exactly, which a realistic value would fail.
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
        UnicyclePhysics::new("car", PERIOD, LIMITS, CarState::new(start_pose(), speed))
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

    fn steer(seq: u64, steer_cmd: f64) -> Message {
        command("steer_cmd", seq, &SteerCmd { steer_cmd })
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
        // This checks that serde maps the fields the way it says it
        // does, not that a float parser is exact, which is why
        // [`start_pose`] and [`CRUISE_SPD`] are short decimals.
        //
        // TODO(PLAN "Determinism and correctness"): `from_str` into an
        // `f64` misses what `to_string` produced by an ulp for about one
        // number in eight. Every component decoding a pose has that gap.
        let state = CarState::new(start_pose(), CRUISE_SPD);
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
        const ACCEL_CMD: f64 = 1.0;

        // Said once and never again, so what the car does for the other
        // ninety-nine steps is what holding it means.
        let state = run(&mut plant(0.0), STEPS, vec![accel(1, ACCEL_CMD)]);
        let rate = ACCEL_CMD * LIMITS.accel_max;
        let seconds = STEPS as f64 * STEP_SECS;
        assert!((state.speed - rate * seconds).abs() < 1e-9, "{state:?}");

        // The step integrates speed before it travels, so what it covers
        // is the sum of the speeds each step ended at rather than the
        // a*t^2/2 of the closed form, which is half a step behind it.
        let steps = STEPS as f64;
        let summed = rate * STEP_SECS * STEP_SECS * steps * (steps + 1.0) / 2.0;
        assert!((travelled(&state) - summed).abs() < 1e-9, "{state:?}");
    }

    #[test]
    fn speed_never_integrates_below_zero() {
        const BRAKE_CMD: f64 = -1.0;

        // Braking held for twice as long as stopping takes, which is the
        // case that would reverse a car taking the arithmetic at its word.
        let decel = (BRAKE_CMD * LIMITS.decel_max).abs();
        let braking_steps = 2 * (CRUISE_SPD / decel / STEP_SECS) as i64;
        let mut plant = plant(CRUISE_SPD);
        let stopped = run(&mut plant, braking_steps, vec![accel(1, BRAKE_CMD)]);
        assert_eq!(stopped.speed, 0.0);

        // And a stopped car goes nowhere, however long the brake is held.
        let still_stopped = run(&mut plant, STEPS, vec![]);
        assert_eq!(still_stopped.speed, 0.0);
        assert_eq!(still_stopped.position.x, stopped.position.x);
    }

    #[test]
    fn accel_and_steer_are_held_independently() {
        const FIRST_ACCEL_CMD: f64 = 0.25;
        const SECOND_ACCEL_CMD: f64 = 1.0;
        const STEER_CMD: f64 = 0.5;

        let mut plant = plant(CRUISE_SPD);
        step_at(
            &mut plant,
            SimTime::ZERO,
            None,
            vec![accel(1, FIRST_ACCEL_CMD), steer(2, STEER_CMD)],
        );

        // One step where only the acceleration is restated. The new one
        // takes effect, and the yaw rate nobody mentioned goes on turning
        // the car, which is what holding each on its own key buys.
        let payload = step_at(
            &mut plant,
            SimTime::from_millis(10),
            Some(PERIOD),
            vec![accel(3, SECOND_ACCEL_CMD)],
        );
        let state: CarState = serde_json::from_slice(&payload).expect("a car state");
        let expected_speed = CRUISE_SPD + SECOND_ACCEL_CMD * LIMITS.accel_max * STEP_SECS;
        assert!((state.speed - expected_speed).abs() < 1e-9, "{state:?}");
        let expected_yaw = STEER_CMD * LIMITS.yaw_rate_max * STEP_SECS;
        assert!(
            (state.orientation.yaw() - expected_yaw).abs() < 1e-9,
            "{state:?}"
        );
    }

    #[test]
    fn only_the_newest_command_on_a_key_is_applied() {
        const ACCEL_CMD_1: f64 = -1.0;
        const ACCEL_CMD_2: f64 = 0.0;
        const ACCEL_CMD_3: f64 = 1.0;

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
            (state.speed - ACCEL_CMD_3 * LIMITS.accel_max * STEP_SECS).abs() < 1e-9,
            "{state:?}"
        );
    }

    #[test]
    fn a_zero_command_moves_nothing_at_all() {
        // The demo rests on this. No car in it commands an acceleration,
        // so the held zero has to leave the speed and the heading exactly
        // where they were rather than nearly there. A negative zero has to
        // be as harmless, since a controller multiplying its way to one is
        // not doing anything unusual.
        for command in [0.0, -0.0] {
            let state = run(
                &mut plant(CRUISE_SPD),
                STEPS,
                vec![accel(1, command), steer(2, command)],
            );
            assert_eq!(state.speed, CRUISE_SPD, "speed moved on {command}");
            assert_eq!(state.orientation.yaw(), 0.0, "yaw moved on {command}");
            assert_eq!(state.position.y, start_pose().position.y);
        }
    }

    #[test]
    fn an_acceleration_normalized_by_the_limits_arrives_as_itself() {
        // A controller divides by the limits and a plant multiplies by
        // them, and this is the only place the two halves of that meet.
        // Braking and accelerating divide by different numbers, so a
        // controller taking one for the other, or the plant doing so,
        // reaches the car as a rate nobody asked for.
        for wanted in [LIMITS.accel_max, 1.5, 0.0, -2.0, -LIMITS.decel_max] {
            let command = accel_fraction(wanted, LIMITS.accel_max, LIMITS.decel_max);
            let state = run(&mut plant(CRUISE_SPD), 1, vec![accel(1, command)]);
            let gained = state.speed - CRUISE_SPD;
            assert!(
                (gained - wanted * STEP_SECS).abs() < 1e-9,
                "{wanted} m/s^2 asked for as {command} and arrived as {}",
                gained / STEP_SECS
            );
        }
    }

    #[test]
    fn a_yaw_rate_normalized_by_the_limits_arrives_as_itself() {
        // The same round trip on the other axis, where one limit serves
        // both directions and the sign is the whole of what a mirrored
        // pair checks.
        for wanted in [LIMITS.yaw_rate_max, 0.5, 0.0, -0.5, -LIMITS.yaw_rate_max] {
            let command = steer_fraction(wanted, LIMITS.yaw_rate_max);
            let state = run(&mut plant(CRUISE_SPD), 1, vec![steer(1, command)]);
            let turned = wanted * STEP_SECS;
            assert!(
                (state.orientation.yaw() - turned).abs() < 1e-9,
                "{wanted} rad/s asked for as {command} and arrived at {}",
                state.orientation.yaw()
            );
        }
    }

    #[test]
    fn a_command_past_the_stops_is_held_at_the_drive_limits() {
        const PAST_THE_STOPS: f64 = 4.0;

        // One step of each limit, asked for by a controller wanting four
        // times the car. What is checked is the rate, not that two
        // commands agree: agreeing says nothing about what they agree on.
        let quickest = run(&mut plant(0.0), 1, vec![accel(1, PAST_THE_STOPS)]);
        let gained = LIMITS.accel_max * STEP_SECS;
        assert!((quickest.speed - gained).abs() < 1e-9, "{quickest:?}");

        let hardest = run(&mut plant(CRUISE_SPD), 1, vec![accel(1, -PAST_THE_STOPS)]);
        let shed = LIMITS.decel_max * STEP_SECS;
        assert!(
            (CRUISE_SPD - hardest.speed - shed).abs() < 1e-9,
            "{hardest:?}"
        );

        let tightest = run(&mut plant(CRUISE_SPD), 1, vec![steer(1, PAST_THE_STOPS)]);
        let turned = LIMITS.yaw_rate_max * STEP_SECS;
        assert!(
            (tightest.orientation.yaw() - turned).abs() < 1e-9,
            "{tightest:?}"
        );
    }
}
