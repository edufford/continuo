//! The control laws themselves: pure functions from what an actor knows
//! to what it commands.
//!
//! Nothing here takes a component, an inbox or a clock, so a law needs
//! only its arguments and answers every caller alike. That is what lets
//! one implementation serve a component in this process and a copy of it
//! compiled into something else, rather than two that have to be kept in
//! step by hand.

use continuo_core::Pose;

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
/// Far enough that a following law computes the free road from it
/// without being told the road is free: the gap is so large that the
/// room the law wants is nothing beside it. So an empty scan, a slot no
/// sensor filled, and a lead a kilometer off are all one case, and there
/// is no emptiness to test for anywhere downstream.
pub const FREE_ROAD_RANGE: f64 = 1e9;

/// The nearest detection in a scan, as `(range, range_rate)`, or the
/// free road if the scan holds nothing nearer.
///
/// A sensor reports what it found in no particular order, because
/// relevance is the consumer's idea rather than the sensor's. For
/// following, the nearest thing ahead is the one that matters, so
/// picking it out is this function's job and not the radar's.
///
/// The two slices are read in step and a shorter one ends the scan. Ties
/// go to the earlier slot, so a scan answers the same way every time it
/// is read, which is what the world hash needs of it. A slot at
/// [`FREE_ROAD_RANGE`] loses to anything real, so a fixed-size array
/// padded out to its length needs no separate count of what is in it.
pub fn nearest_detection(ranges: &[f64], range_rates: &[f64]) -> (f64, f64) {
    let mut nearest = (FREE_ROAD_RANGE, 0.0);
    for (&range, &range_rate) in ranges.iter().zip(range_rates) {
        if range < nearest.0 {
            nearest = (range, range_rate);
        }
    }

    // Return the nearest, which is the free road until something beats it.
    nearest
}

/// The smallest gap [`idm_accel`] will divide by, in meters.
///
/// Not IDM's. The published equation divides by the gap and says nothing
/// about a gap of nothing, so this is what keeps the answer a number.
/// What it is set to hardly matters: a run that reaches it has put two
/// bodies in one place already, which is a problem for something other
/// than the follower to report.
const GAP_FLOOR: f64 = 0.1;

/// How an IDM follower is tuned: how fast it wants to go, how much room
/// it wants at that speed, and how hard it will work for either.
///
/// Each field gives its name in the published equation, since that
/// equation is what the law here is written from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdmParams {
    /// Speed held on an open road, m/s. IDM's `v0`.
    pub speed_tgt: f64,
    /// Seconds of gap wanted at whatever speed is being held. IDM's `T`.
    pub time_headway: f64,
    /// Meters of gap wanted at a standstill. IDM's `s0`.
    pub gap_min: f64,
    /// The most acceleration commanded, m/s^2. IDM's `a`.
    pub accel_max: f64,
    /// The braking taken as comfortable, m/s^2 and positive. IDM's `b`.
    pub decel_comfort: f64,
    /// The most braking commanded, m/s^2 and positive. Not IDM's, which
    /// bounds braking nowhere. A law free to ask for anything is a poor
    /// thing to put in front of an actuator.
    pub decel_max: f64,
}

impl IdmParams {
    /// The set this project drives cars with, taking the speed each one
    /// wants to hold.
    ///
    /// `time_headway` and `gap_min` are the values Treiber's own
    /// simulator lists for a car, at
    /// <https://traffic-simulation.de/info/info_IDM.html>. The
    /// acceleration and braking there, 0.3 and 3.0, are picked to bring
    /// stop-and-go waves out of a crowded road, which is a different
    /// thing to show than a car following another car, so `accel_max`
    /// and `decel_comfort` are this project's, within what ordinary
    /// driving uses. So is `decel_max`, at firm braking rather than
    /// emergency braking.
    ///
    /// None of them are knobs. A scenario misbehaving with these is a
    /// reason to look at the wiring, because numbers fitted until a demo
    /// looks right are where a wiring mistake goes to hide.
    pub fn highway_car(speed_tgt: f64) -> Self {
        IdmParams {
            speed_tgt,
            time_headway: 1.5,
            gap_min: 2.0,
            accel_max: 1.5,
            decel_comfort: 2.0,
            decel_max: 4.0,
        }
    }
}

/// Acceleration the Intelligent Driver Model commands, m/s^2.
///
/// `gap` is the room ahead in meters and `approach_rate` is how fast
/// that room is closing, positive when closing. Both are relative, which
/// is what a radar measures and all this wants: the lead's own speed
/// appears nowhere in the law.
///
/// The room it asks for is `gap_min`, plus `time_headway` seconds of
/// travel, plus enough to shed the approach rate against a brake between
/// comfortable and hardest. It accelerates when it has more room than
/// that and brakes when it has less, easing off either way as it nears
/// `speed_tgt`. An open road is the same expression rather than a case
/// of its own, since at [`FREE_ROAD_RANGE`] the room asked for is
/// nothing beside the room there is.
///
/// Add, multiply, divide and sqrt only, which IEEE 754 pins, so four
/// platforms agree to the last bit. The fourth power is two squarings
/// rather than `powi`, whose expansion is the compiler's business rather
/// than the standard's.
pub fn idm_accel(speed: f64, gap: f64, approach_rate: f64, params: IdmParams) -> f64 {
    // A lead pulling away asks for no extra room at all, which is what
    // the floor at zero says. Without it a fast enough retreat would
    // talk the law into wanting less room than gap_min.
    let closing_room =
        speed * approach_rate / (2.0 * (params.accel_max * params.decel_comfort).sqrt());
    let gap_wanted = params.gap_min + (speed * params.time_headway + closing_room).max(0.0);

    // How much of the open road is left to take, and how much of it the
    // lead has taken. The fourth power is what holds the first term near
    // full until the speed is nearly there, rather than easing off from
    // a standstill onward.
    let speed_ratio = speed / params.speed_tgt;
    let speed_ratio_squared = speed_ratio * speed_ratio;
    let free_road = 1.0 - speed_ratio_squared * speed_ratio_squared;
    let crowding = gap_wanted / gap.max(GAP_FLOOR);

    // Return what the road leaves once the lead has had its share.
    (params.accel_max * (free_road - crowding * crowding))
        .clamp(-params.decel_max, params.accel_max)
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
        assert_eq!(
            nearest_detection(&[40.0, 12.0, 90.0], &[-1.0, -3.0, 5.0]),
            (12.0, -3.0)
        );

        // The same three cars, reported in another order, are the same
        // three cars, and the sensor promises no order at all.
        assert_eq!(
            nearest_detection(&[12.0, 90.0, 40.0], &[-3.0, 5.0, -1.0]),
            (12.0, -3.0)
        );
    }

    #[test]
    fn an_empty_scan_selects_the_free_road() {
        assert_eq!(nearest_detection(&[], &[]), (FREE_ROAD_RANGE, 0.0));
    }

    #[test]
    fn padding_never_beats_a_car() {
        // A fixed-size array holding one car and nothing else, which is
        // the shape a scan arrives in once it has crossed into an FMU.
        let mut ranges = [FREE_ROAD_RANGE; 8];
        let mut range_rates = [0.0; 8];
        ranges[5] = 30.0;
        range_rates[5] = -2.0;
        assert_eq!(nearest_detection(&ranges, &range_rates), (30.0, -2.0));

        // And one holding nothing is the free road, padding and all.
        assert_eq!(
            nearest_detection(&[FREE_ROAD_RANGE; 8], &[0.0; 8]),
            (FREE_ROAD_RANGE, 0.0)
        );
    }

    #[test]
    fn a_tie_goes_to_the_earlier_slot() {
        // Two cars at exactly one range, one closing and one not. Which
        // of them is followed matters less than that the answer is not
        // left to the order the slots happen to arrive in.
        assert_eq!(nearest_detection(&[25.0, 25.0], &[-4.0, 1.0]), (25.0, -4.0));
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
            following().accel_max
        );
    }

    #[test]
    fn an_open_road_at_the_speed_it_wants_commands_nothing() {
        let accel = idm_accel(30.0, FREE_ROAD_RANGE, 0.0, following());
        assert!(accel.abs() < 1e-12, "{accel}");
    }

    #[test]
    fn the_published_equation_gives_the_value_worked_by_hand() {
        // Doing 20 behind a lead 40 m off and closing at 5, worked from
        // the equation with the numbers highway_car sets:
        //   room wanted = 2 + 20*1.5 + 20*5 / (2*sqrt(1.5*2))
        //               = 60.86751345948129
        //   accel       = 1.5 * (1 - (20/30)^4 - (60.86751345948129/40)^2)
        //               = -2.2695971038651734
        let accel = idm_accel(20.0, 40.0, 5.0, following());
        assert!((accel + 2.269_597_103_865_173_4).abs() < 1e-12, "{accel}");
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
            -following().decel_max
        );
    }

    #[test]
    fn a_lead_pulling_away_never_buys_room_below_the_minimum() {
        // The closing allowance floors at zero, so no retreat, however
        // fast, talks the law into wanting less than gap_min and a
        // headway. Two different retreats therefore answer alike.
        let fast = idm_accel(20.0, 32.0, -1000.0, following());
        let faster = idm_accel(20.0, 32.0, -5000.0, following());
        assert_eq!(fast, faster);
        assert!(fast > 0.0, "{fast}");
    }

    #[test]
    fn the_gap_floor_keeps_a_touching_lead_finite() {
        // Both of these are already a collision, and the law's business
        // with them is only to stay a number.
        assert_eq!(
            idm_accel(20.0, 0.0, 0.0, following()),
            -following().decel_max
        );
        assert_eq!(
            idm_accel(20.0, -3.0, 0.0, following()),
            -following().decel_max
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
                            && accel <= params.accel_max
                            && accel >= -params.decel_max,
                        "{speed} {gap} {approach_rate} gave {accel}"
                    );
                }
            }
        }
    }

    #[test]
    fn a_scan_ends_where_the_shorter_of_its_two_slices_does() {
        // Nothing publishes a mismatched pair. Reading past the end of
        // one of them would be reading whichever detection last held
        // that slot.
        assert_eq!(nearest_detection(&[50.0, 5.0], &[-1.0]), (50.0, -1.0));
    }
}
