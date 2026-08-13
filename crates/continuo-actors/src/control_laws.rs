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
    pub lateral: f64,
    /// How far along the road the aim point sits, in meters.
    pub lookahead: f64,
    /// Yaw rate commanded per radian of heading error, in 1/s.
    pub gain: f64,
    /// The most yaw rate to command in either direction, rad/s.
    pub max_yaw_rate: f64,
}

/// Yaw rate that steers a follower at `pose` onto the lane `lateral`
/// meters left of `road`, positive counter-clockwise.
///
/// Projects the pose onto the road, aims at the point `lookahead` further
/// along it, and turns toward that point in proportion to how far off the
/// heading is, no harder than the follower is allowed to turn. Holding a
/// lane needs no geometry of its own, because the aim point is offset
/// from the one road everything on it shares.
pub fn pure_pursuit_yaw_rate(road: &Waypoints, pose: Pose, params: PurePursuitParams) -> f64 {
    let position = pose.position;
    let s = road.project(position.x, position.y);
    let target = road.point_at_offset(s + params.lookahead, params.lateral);
    let desired_heading = f64::atan2(target.y - position.y, target.x - position.x);
    let heading_error = wrap_pi(desired_heading - pose.orientation.yaw());

    // Return the turn toward the aim point, inside the clamp.
    (params.gain * heading_error).clamp(-params.max_yaw_rate, params.max_yaw_rate)
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
            lateral: 0.0,
            lookahead: 10.0,
            gain: 1.0,
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
        // is clockwise, and one as far the other side turns the other
        // way by as much. Only as much to within rounding, since the
        // negative error comes back through the wrap and the positive one
        // does not.
        let from_the_left = pure_pursuit_yaw_rate(&road(), pose_at(20.0, 3.0, 0.0), tuning());
        let from_the_right = pure_pursuit_yaw_rate(&road(), pose_at(20.0, -3.0, 0.0), tuning());
        assert!(from_the_left < 0.0, "{from_the_left}");
        let mirrored = from_the_left + from_the_right;
        assert!(mirrored.abs() < 1e-12, "{from_the_left} {from_the_right}");
    }

    #[test]
    fn a_lane_offset_is_what_the_law_holds_rather_than_the_road() {
        let holding_a_lane = PurePursuitParams {
            lateral: 3.0,
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
            gain: 10.0,
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
}
