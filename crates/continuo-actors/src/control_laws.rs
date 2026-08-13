//! The control laws themselves: pure functions from what an actor knows
//! to what it commands.
//!
//! Nothing here takes a component, an inbox or a clock, so a law needs
//! only its arguments and answers every caller alike. That is what lets
//! one implementation serve a component in this process and a copy of it
//! compiled into something else, rather than two that have to be kept in
//! step by hand.

use continuo_core::{Detection, Pose};

use crate::path::Waypoints;

/// How a pure-pursuit follower is tuned: which lane to hold, how far ahead
/// to aim, and how hard to turn toward the aim point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PurePursuitParams {
    /// Meters left of the road's centerline to hold, the Frenet `d`.
    pub lateral_tgt: f64,
    /// How far along the road the aim point sits, in meters.
    pub lookahead: f64,
    /// Yaw rate commanded per radian of heading error, in 1/s.
    pub gain_yaw_rate: f64,
    /// The most yaw rate to command in either direction, rad/s.
    pub max_yaw_rate: f64,
}

/// Yaw rate that steers a follower at `pose` onto the lane
/// `lateral_tgt` meters left of `road`, positive counter-clockwise.
///
/// Projects the pose onto the road, aims at the point `lookahead` further
/// along it, and turns toward that point in proportion to how far off the
/// heading is, no harder than the follower is allowed to turn. Holding a
/// lane needs no geometry of its own, because the aim point is offset
/// from the one road everything on it shares.
pub fn pure_pursuit_yaw_rate(road: &Waypoints, pose: Pose, params: PurePursuitParams) -> f64 {
    let position = pose.position;
    let s = road.project(position.x, position.y);
    let target = road.point_at_offset(s + params.lookahead, params.lateral_tgt);
    let desired_heading = f64::atan2(target.y - position.y, target.x - position.x);
    let heading_error = wrap_to_pi(desired_heading - pose.orientation.yaw());

    // Return the turn toward the aim point, inside the clamp.
    (params.gain_yaw_rate * heading_error).clamp(-params.max_yaw_rate, params.max_yaw_rate)
}

/// The same angle measured the short way round, in [-pi, pi].
///
/// Wrapping the magnitude and putting the sign back sends both signs
/// through one calculation, so the answer is exactly odd:
/// `wrap_to_pi(-a)` is `-wrap_to_pi(a)` to the bit. Folding a negative
/// angle up into [0, TAU) and back down instead would round it twice
/// where a positive angle is not rounded at all, and two followers the
/// same distance either side of a lane would steer back at slightly
/// different rates.
///
/// Nothing here rounds. A remainder is exact, and the subtraction only
/// runs when more than half a turn is left, so its two sides are within
/// a factor of two of each other and it is exact as well. Multiplying by
/// 1 or -1 is exact. An angle of exactly half a turn keeps the sign it
/// arrived with, both ways round being equally short.
fn wrap_to_pi(angle: f64) -> f64 {
    let part_turn = angle.abs() % std::f64::consts::TAU;
    let short_way = if part_turn > std::f64::consts::PI {
        part_turn - std::f64::consts::TAU
    } else {
        part_turn
    };

    // Return the short way round, pointing the way the angle did.
    angle.signum() * short_way
}

/// The range standing for nothing detected, in meters.
///
/// A million kilometers, which is less a distance than a number picked
/// against two constraints. Far enough that a following law reads it as
/// an open road: what IDM asks for at this range moves the answer by
/// under 2e-12 m/s^2, for any speed and closing rate a road produces.
/// Small enough to stay finite through whatever is done to it, since a
/// scan travels as JSON, which has no infinity, and something
/// downstream may yet take a car length off it or hand it to a 32-bit
/// float.
pub const FREE_ROAD_RANGE: f64 = 1e9;

/// Nothing there: [`FREE_ROAD_RANGE`] away and not moving.
///
/// What an unfilled slot holds in a fixed-length scan, which is how a
/// scan crosses into an FMU.
pub const FREE_ROAD: Detection = Detection {
    range: FREE_ROAD_RANGE,
    range_rate: 0.0,
};

/// The nearest detection in a scan, or [`FREE_ROAD`] if the scan holds
/// nothing nearer.
///
/// A sensor reports what it found in no particular order, because
/// relevance is the consumer's idea rather than the sensor's. For
/// following, the nearest thing ahead is the one that matters, so
/// picking it out is this function's job and not the radar's.
///
/// Ties go to the earlier slot, so a scan answers the same way every
/// time it is read, which is what the world hash needs of it. Free-road
/// slots lose to anything real, so a fixed-length scan padded out with
/// them needs no count of what is in it.
pub fn nearest_detection(scan: &[Detection]) -> Detection {
    let mut nearest = FREE_ROAD;
    for &detection in scan {
        if detection.range < nearest.range {
            nearest = detection;
        }
    }

    // Return the nearest, which is the free road until something beats it.
    nearest
}

/// How an IDM follower is tuned: how fast it wants to go, how much room
/// it wants at that speed, and how hard it will work for either.
///
/// These are the published equation's five and nothing else, each
/// carrying its symbol so the two can be read side by side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdmParams {
    /// Speed held on an open road, m/s.
    pub v0_speed_tgt: f64,
    /// Seconds of gap wanted at whatever speed is being held.
    pub t_headway: f64,
    /// Meters of gap wanted at a standstill.
    pub s0_gap_min: f64,
    /// The most acceleration commanded, m/s^2.
    pub a_accel_max: f64,
    /// The braking taken as comfortable, m/s^2 and positive. Also the
    /// hardest [`idm_accel`] will ask for, which costs a car the
    /// emergency stop it might otherwise make.
    pub b_decel_comfort: f64,
}

impl IdmParams {
    /// The set this project drives cars with, taking the speed each one
    /// wants to hold.
    ///
    /// `t_headway` and `s0_gap_min` are the values Treiber's own
    /// simulator lists for a car, at
    /// <https://traffic-simulation.de/info/info_IDM.html>. The
    /// acceleration and braking there, 0.3 and 3.0, are picked to bring
    /// stop-and-go waves out of a crowded road, which is a different
    /// thing to show than a car following another car, so `a_accel_max`
    /// and `b_decel_comfort` are this project's, within what ordinary
    /// driving uses.
    pub fn highway_car(v0_speed_tgt: f64) -> Self {
        IdmParams {
            v0_speed_tgt,
            t_headway: 1.5,
            s0_gap_min: 2.0,
            a_accel_max: 1.5,
            b_decel_comfort: 2.0,
        }
    }
}

/// Acceleration the Intelligent Driver Model commands, m/s^2.
///
/// `gap` is the room ahead in meters. `approach_rate` is how fast that
/// room is closing, in m/s and positive while it closes, which is the
/// paper's own convention: it defines the approaching rate as the
/// follower's speed minus the lead's. A [`Detection`] measures the same
/// quantity the other way up, so a caller holding one passes
/// `-range_rate`.
///
/// Both are relative, which is what a radar measures and all the law
/// wants: the lead's own speed appears nowhere in it.
///
/// The room it asks for is `s0_gap_min`, plus `t_headway` seconds of
/// travel, plus enough to shed the approach rate against a brake between
/// comfortable and hardest. It accelerates when it has more room than
/// that and brakes when it has less, easing off either way as it nears
/// `v0_speed_tgt`. An open road is the same expression rather than a case
/// of its own, since at [`FREE_ROAD_RANGE`] the room asked for is
/// nothing beside the room there is.
///
/// Add, multiply, divide and sqrt only, which IEEE 754 pins, so four
/// platforms agree to the last bit. The fourth power is two squarings
/// rather than `powi`, whose expansion is the compiler's business rather
/// than the standard's.
///
/// One thing here is not in the published equation, and the body says
/// what it is for: the wanted gap does not go below zero. The command is
/// held inside `[-b_decel_comfort, a_accel_max]`, which adds no
/// parameter of its own, and buys a finite answer for a gap of nothing
/// at the price of the emergency braking a real car would have.
pub fn idm_accel(speed: f64, gap: f64, approach_rate: f64, params: IdmParams) -> f64 {
    // The paper's s* reads s0 + s1*sqrt(v/v0) + v*T + v*dv/(2*sqrt(ab)),
    // and this is it with s1 = 0, as the common four-parameter form has
    // it, under the one adaptation: a wanted gap is a distance, so it
    // does not go below zero.
    //
    // Something has to say that, because a lead pulling away contributes
    // a negative closing term, and the equation as published lets that
    // take the whole expression negative, where it squares back into
    // braking. Then a car brakes as the road ahead clears: at 20 m/s
    // with 20 m of gap and a lead pulling away to 30, -1.28 m/s^2 where
    // the answer wanted is +1.20. A limit on the output cannot fix that,
    // since -1.28 is a perfectly ordinary number to command. What is
    // wrong with it is its sign.
    let closing_room =
        speed * approach_rate / (2.0 * (params.a_accel_max * params.b_decel_comfort).sqrt());
    let gap_wanted = (params.s0_gap_min + speed * params.t_headway + closing_room).max(0.0);

    // How much of the open road is left to take, and how much of it the
    // lead has taken. The fourth power is what holds the first term near
    // full until the speed is nearly there, rather than easing off from
    // a standstill onward.
    let speed_ratio = speed / params.v0_speed_tgt;
    let speed_ratio_squared = speed_ratio * speed_ratio;
    let free_road = 1.0 - speed_ratio_squared * speed_ratio_squared;
    // Every real gap divides as it stands. The floor only keeps a zero
    // out of the divisor, since a wanted gap of zero over it would be a
    // NaN spreading into everything downstream. What comes out of a gap
    // of nothing is an enormous quotient, and the clamp takes that to
    // the braking limit, which is the answer a collided pair deserves.
    let crowding = gap_wanted / gap.max(f64::MIN_POSITIVE);

    // Return the acceleration the equation gives, which is what is left
    // of the open road once the lead's share is taken off it, held
    // inside the two rates the parameters name.
    //
    // The lower bound is the one doing the work: as the gap goes to
    // nothing the crowding term runs away, taking the command to
    // negative infinity, which is a brake without any limit at all. The
    // upper bound only guards, since the equation on its own never gives
    // more than a_accel_max, that being what a standstill on an empty
    // road gives.
    (params.a_accel_max * (free_road - crowding * crowding))
        .clamp(-params.b_decel_comfort, params.a_accel_max)
}

#[cfg(test)]
mod tests {
    use continuo_core::{Quat, Vec3};

    use super::*;

    /// A hundred meters of road along +x, which every test here steers on.
    fn road() -> Waypoints {
        Waypoints::build_straight((0.0, 0.0), (100.0, 0.0))
    }

    fn pose_at(x: f64, y: f64, yaw: f64) -> Pose {
        Pose {
            position: Vec3::new(x, y, 0.0),
            orientation: Quat::from_yaw(yaw),
        }
    }

    /// Aims 10 m ahead at the road itself, turning a radian per radian.
    fn tuning() -> PurePursuitParams {
        PurePursuitParams {
            lateral_tgt: 0.0,
            lookahead: 10.0,
            gain_yaw_rate: 1.0,
            max_yaw_rate: 1.0,
        }
    }

    #[test]
    fn a_follower_on_its_lane_and_pointing_along_it_commands_no_turn() {
        assert_eq!(
            pure_pursuit_yaw_rate(&road(), pose_at(20.0, 0.0, 0.0), tuning()),
            0.0
        );
    }

    #[test]
    fn a_follower_off_its_lane_turns_back_toward_it() {
        // Left of the lane the aim point is to the right, so the command
        // is clockwise, and one as far the other side turns the other way
        // by exactly as much. Exactly, with no tolerance, because a
        // mirrored geometry that answered only to within rounding would
        // mean the law had a preferred side.
        let from_the_left = pure_pursuit_yaw_rate(&road(), pose_at(20.0, 3.0, 0.0), tuning());
        let from_the_right = pure_pursuit_yaw_rate(&road(), pose_at(20.0, -3.0, 0.0), tuning());
        assert!(from_the_left < 0.0, "{from_the_left}");
        assert_eq!(from_the_left, -from_the_right);
    }

    #[test]
    fn a_lane_offset_is_what_the_law_holds_rather_than_the_road() {
        let holding_a_lane = PurePursuitParams {
            lateral_tgt: 3.0,
            ..tuning()
        };

        // On the centerline there is now three meters of error to work
        // off, and none once the car is on the lane it was given.
        let toward_the_lane =
            pure_pursuit_yaw_rate(&road(), pose_at(20.0, 0.0, 0.0), holding_a_lane);
        assert!(toward_the_lane > 0.0, "{toward_the_lane}");
        assert_eq!(
            pure_pursuit_yaw_rate(&road(), pose_at(20.0, 3.0, 0.0), holding_a_lane),
            0.0
        );
    }

    #[test]
    fn the_command_never_leaves_the_clamp() {
        let hard_and_capped = PurePursuitParams {
            gain_yaw_rate: 10.0,
            max_yaw_rate: 0.5,
            ..tuning()
        };

        // Pointing across the road, where a quarter turn of error times a
        // gain of ten asks for far more than the follower may command.
        let quarter_turn = std::f64::consts::FRAC_PI_2;
        let facing_right =
            pure_pursuit_yaw_rate(&road(), pose_at(20.0, 0.0, -quarter_turn), hard_and_capped);
        let facing_left =
            pure_pursuit_yaw_rate(&road(), pose_at(20.0, 0.0, quarter_turn), hard_and_capped);
        assert_eq!(facing_right, 0.5);
        assert_eq!(facing_left, -0.5);
    }

    #[test]
    fn wrapping_an_angle_answers_its_mirror_exactly() {
        // Out to +/- 8 radians, a little past a full turn either side of
        // zero, so the sweep covers angles that wrap and angles that do
        // not.
        const STEP: f64 = 0.04;
        const STEPS: i32 = 200;

        // Each angle is a product, so it has a full mantissa. Reaching
        // them by summing instead, as -8.0 plus a multiple of the step,
        // would leave the low bits clear, and an angle with room to
        // spare there comes back from a fold up and down unchanged. The
        // sweep would then pass whatever this function did.
        for k in -STEPS..=STEPS {
            let angle = f64::from(k) * STEP;
            let wrapped = wrap_to_pi(angle);
            assert!(wrapped.abs() <= std::f64::consts::PI, "{angle} {wrapped}");
            assert_eq!(wrapped, -wrap_to_pi(-angle), "{angle}");
        }
    }

    #[test]
    fn half_a_turn_keeps_the_sign_it_came_with() {
        // The one angle with two equally short ways round, and the only
        // input where the answer is a choice rather than a value.
        assert_eq!(wrap_to_pi(std::f64::consts::PI), std::f64::consts::PI);
        assert_eq!(wrap_to_pi(-std::f64::consts::PI), -std::f64::consts::PI);
    }

    #[test]
    fn an_error_past_half_a_turn_steers_the_short_way_round() {
        let free_to_turn = PurePursuitParams {
            max_yaw_rate: 10.0,
            ..tuning()
        };

        // Past the end of the road and facing back down it, where the aim
        // point is just the far side of half a turn away. Measured the
        // long way the error would be most of a full turn, and the command
        // a hard turn in the wrong direction.
        let yaw_rate = pure_pursuit_yaw_rate(&road(), pose_at(110.0, 0.5, 3.0), free_to_turn);
        assert!(yaw_rate > 0.0 && yaw_rate < 0.25, "{yaw_rate}");
    }

    #[test]
    fn the_nearest_detection_wins_regardless_of_its_slot() {
        let far = Detection {
            range: 40.0,
            range_rate: -1.0,
        };
        let near = Detection {
            range: 12.0,
            range_rate: -3.0,
        };
        let farthest = Detection {
            range: 90.0,
            range_rate: 5.0,
        };
        assert_eq!(nearest_detection(&[far, near, farthest]), near);

        // The same three cars, reported in another order, are the same
        // three cars, and the sensor promises no order at all.
        assert_eq!(nearest_detection(&[near, farthest, far]), near);
    }

    #[test]
    fn an_empty_scan_selects_the_free_road() {
        assert_eq!(nearest_detection(&[]), FREE_ROAD);
    }

    #[test]
    fn padding_never_beats_a_car() {
        // A fixed-size scan holding one car and nothing else, which is
        // the shape it arrives in once it has crossed into an FMU.
        let car = Detection {
            range: 30.0,
            range_rate: -2.0,
        };
        let mut scan = [FREE_ROAD; 8];
        scan[5] = car;
        assert_eq!(nearest_detection(&scan), car);

        // And one holding nothing is the free road, padding and all.
        assert_eq!(nearest_detection(&[FREE_ROAD; 8]), FREE_ROAD);
    }

    #[test]
    fn a_tie_goes_to_the_earlier_slot() {
        // Two cars at exactly one range, one closing and one not. Which
        // of them is followed matters less than that the answer is not
        // left to the order the slots happen to arrive in.
        let closing = Detection {
            range: 25.0,
            range_rate: -4.0,
        };
        let opening = Detection {
            range: 25.0,
            range_rate: 1.0,
        };
        assert_eq!(nearest_detection(&[closing, opening]), closing);
    }

    /// A car wanting 30 m/s, tuned the way the demo tunes every car.
    fn following() -> IdmParams {
        IdmParams::highway_car(30.0)
    }

    #[test]
    fn an_open_road_from_rest_accelerates_at_the_limit() {
        // Nothing ahead and no speed yet, which is the whole of what the
        // car will do and exactly that, since the room a standstill asks
        // for is nothing against a free road.
        assert_eq!(
            idm_accel(0.0, FREE_ROAD_RANGE, 0.0, following()),
            following().a_accel_max
        );
    }

    #[test]
    fn an_open_road_at_the_speed_it_wants_commands_nothing() {
        let accel = idm_accel(30.0, FREE_ROAD_RANGE, 0.0, following());
        assert!(accel.abs() < 1e-12, "{accel}");
    }

    #[test]
    fn the_published_equation_gives_the_value_worked_by_hand() {
        // Doing 20 behind a lead 60 m off and closing at 5, worked from
        // the equation with the numbers highway_car sets:
        //   room wanted = 2 + 20*1.5 + 20*5 / (2*sqrt(1.5*2))
        //               = 60.86751345948129
        //   accel       = 1.5 * (1 - (20/30)^4 - (60.86751345948129/60)^2)
        //               = -0.33998554410468607
        //
        // Sixty meters rather than forty so the answer lands inside the
        // limits, since a clamped value would check the clamp instead of
        // the equation.
        let accel = idm_accel(20.0, 60.0, 5.0, following());
        assert!((accel + 0.339_985_544_104_686_07).abs() < 1e-12, "{accel}");
    }

    #[test]
    fn the_equilibrium_gap_commands_nothing() {
        // Where IDM settles behind a lead holding a steady speed. It is
        // further back than the room the law asks for, because the open
        // road term is already short of full at 20 against a wanted 30,
        // and the two only cancel where the gap makes up the difference.
        let accel = idm_accel(20.0, 35.722_003_561_692_034, 0.0, following());
        assert!(accel.abs() < 1e-12, "{accel}");
    }

    #[test]
    fn a_close_slow_lead_brakes_no_harder_than_the_limit() {
        // Thirty meters a second onto something 5 m ahead: the equation
        // asks for hundreds, and the car commands what it has.
        assert_eq!(
            idm_accel(30.0, 5.0, 20.0, following()),
            -following().b_decel_comfort
        );
    }

    #[test]
    fn a_lead_pulling_away_never_buys_room_below_the_minimum() {
        // The wanted gap floors at zero, so however fast a lead pulls
        // away, what is left is an open road rather than a negative
        // distance squaring back into braking. Two different retreats
        // therefore answer alike, and both accelerate.
        let fast = idm_accel(20.0, 32.0, -1000.0, following());
        let faster = idm_accel(20.0, 32.0, -5000.0, following());
        assert_eq!(fast, faster);
        assert!(fast > 0.0, "{fast}");

        // The case that made the floor necessary, which is ordinary
        // rather than extreme: a lead pulling away to 30 with 20 m of
        // gap. Unfloored the equation commands -1.28 here.
        let clearing = idm_accel(20.0, 20.0, -10.0, following());
        assert!(clearing > 1.0, "{clearing}");
    }

    #[test]
    fn a_gap_of_nothing_brakes_rather_than_answering_nothing() {
        // Both of these are already a collision, and the law's business
        // with them is only to stay a number.
        assert_eq!(
            idm_accel(20.0, 0.0, 0.0, following()),
            -following().b_decel_comfort
        );
        assert_eq!(
            idm_accel(20.0, -3.0, 0.0, following()),
            -following().b_decel_comfort
        );
    }

    #[test]
    fn the_command_is_always_finite_and_inside_the_limits() {
        let params = following();
        for &speed in &[0.0, 5.0, 20.0, 30.0, 60.0] {
            for &gap in &[-1.0, 0.0, 0.5, 5.0, 40.0, FREE_ROAD_RANGE] {
                for &approach_rate in &[-60.0, -1.0, 0.0, 1.0, 60.0] {
                    let accel = idm_accel(speed, gap, approach_rate, params);
                    assert!(
                        accel.is_finite()
                            && accel <= params.a_accel_max
                            && accel >= -params.b_decel_comfort,
                        "{speed} {gap} {approach_rate} gave {accel}"
                    );
                }
            }
        }
    }
}
