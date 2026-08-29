//! The controller behind the FMI boundary: what crosses it, and what
//! the laws make of that.
//!
//! Nothing here decides anything. The declaration below says what a host
//! may set and read, and every answer comes from
//! [`continuo_actors::control_laws`], which the native controller calls
//! too. So the two agree because they are one implementation rather than
//! because somebody keeps them in step.

use continuo_actors::control_laws::{
    FREE_ROAD, IdmParams, PurePursuitParams, accel_fraction, idm_accel, nearest_detection,
    pure_pursuit_yaw_rate, steer_fraction,
};
use continuo_actors::{DriveLimits, MAX_DETECTIONS, Waypoints};
use continuo_core::{Detection, Pose, Quat, Vec3};
use fmi::fmi3::{Fmi3Error, Fmi3Res};
use fmi_export::FmuModel;
use fmi_export::fmi3::{Context, DefaultLoggingCategory, UserModel};

use crate::error::BadInput;

/// How many points of road this FMU can be handed.
///
/// It lives here because it is a fact about this interface rather than
/// about roads: `fmi-export` 0.3.0 cannot size an array by a parameter,
/// so the array is fixed and `road_point_count` says how much of it is
/// real. Padding the tail by repeating the last point instead would hand
/// [`Waypoints::project`] a segment of no length, so the padding has to
/// be ignored rather than read.
///
/// It covers the demo's two points with room for a polyline drawn by
/// hand. A road built from a map will want more, and this is where that
/// question first shows.
pub const MAX_WAYPOINTS: usize = 64;

/// The speed a car holds until a host says otherwise, m/s.
///
/// The only tuning number this crate picks. Everything else it starts
/// from is a tuning set [`continuo_actors::control_laws`] already names,
/// so the FMU and the native controller start from the same places.
const DEFAULT_TARGET_SPEED: f64 = 30.0;

/// What the following law's parameters hold until a host sets them.
const DEFAULT_IDM: IdmParams = IdmParams::highway_car(DEFAULT_TARGET_SPEED);

/// What the steering law's parameters hold until a host sets them,
/// holding the road's own centerline.
const DEFAULT_PURSUIT: PurePursuitParams = PurePursuitParams::highway_car(0.0);

/// What the car's limits hold until a host sets them.
const DEFAULT_LIMITS: DriveLimits = DriveLimits::highway_car();

/// A parameter something divides by, checked before it gets there.
///
/// Rejects a zero, a negative, an infinity and a NaN alike, since what
/// they have in common is that no answer computed from them means
/// anything.
fn require_positive(name: &'static str, given: f64) -> Result<f64, BadInput> {
    if given.is_finite() && given > 0.0 {
        Ok(given)
    } else {
        Err(BadInput::NotPositive { name, given })
    }
}

/// A controller for traffic cars, following with IDM and turning with
/// pure pursuit, from one pose and one radar scan.
///
/// Each variable's **first doc line is its FMI description**, and the
/// packager takes that line alone. So a first line has to be a whole
/// sentence, and anything longer goes on the lines below it, which stay
/// here for whoever reads the code.
// `user_model = false` because the `impl UserModel` below is written by
// hand. Left true, the derive writes one whose `calculate_values` does
// nothing, and this FMU would export cleanly and command zero forever.
#[derive(FmuModel, Debug)]
#[model(co_simulation = true, model_exchange = false, user_model = false)]
pub struct FmuController {
    /// Where the car is, meters east.
    #[variable(causality = Input, name = "position.x", start = Self::default().position_x)]
    position_x: f64,
    /// Where the car is, meters north.
    #[variable(causality = Input, name = "position.y", start = Self::default().position_y)]
    position_y: f64,
    /// Which way the car points: the quaternion's scalar part.
    #[variable(causality = Input, name = "orientation.w", start = Self::default().orientation_w)]
    orientation_w: f64,
    /// Which way the car points: the quaternion's x part.
    #[variable(causality = Input, name = "orientation.x", start = Self::default().orientation_x)]
    orientation_x: f64,
    /// Which way the car points: the quaternion's y part.
    #[variable(causality = Input, name = "orientation.y", start = Self::default().orientation_y)]
    orientation_y: f64,
    /// Which way the car points: the quaternion's z part.
    #[variable(causality = Input, name = "orientation.z", start = Self::default().orientation_z)]
    orientation_z: f64,
    /// How fast the car is going, m/s, as the car itself reports it.
    ///
    /// A radar measures nothing about the car carrying it, so this
    /// arrives from the pose, standing in for a wheel speed sensor.
    #[variable(causality = Input, start = Self::default().speed)]
    speed: f64,

    /// Meters to each thing the radar found ahead, free road past those.
    ///
    /// A slot the radar did not fill holds [`FREE_ROAD`], which loses to
    /// anything real, so nothing has to say how many are worth reading.
    #[variable(causality = Input, start = Self::default().range)]
    range: [f64; MAX_DETECTIONS],
    /// How fast each range is changing, m/s, negative while closing.
    #[variable(causality = Input, start = Self::default().range_rate)]
    range_rate: [f64; MAX_DETECTIONS],

    // The road is fixed where the law parameters below are tunable, and
    // the difference is real rather than cautious. Each instance builds
    // its `Waypoints` once and keeps it, so a road changed after
    // initialization would be accepted and then ignored. Declaring it
    // fixed is what tells a host that before it tries.
    /// The road's points, meters east, the first road_point_count real.
    #[variable(causality = Parameter, variability = Fixed, start = Self::default().road_x)]
    road_x: [f64; MAX_WAYPOINTS],
    /// The road's points, meters north, the first road_point_count real.
    #[variable(causality = Parameter, variability = Fixed, start = Self::default().road_y)]
    road_y: [f64; MAX_WAYPOINTS],
    /// How many of those points are the road, the rest being padding.
    #[variable(causality = Parameter, variability = Fixed, start = Self::default().road_point_count)]
    road_point_count: u32,
    /// Whether the road wraps at its end rather than stopping there.
    ///
    /// True when the last point joins back to the first. Two roads with
    /// the same points and different answers here are different roads.
    #[variable(causality = Parameter, variability = Fixed, start = Self::default().road_closed)]
    road_closed: bool,

    // Tunable, so a host may change any of these between steps: a car
    // wanting a different speed, or a different lane, says so here. That
    // is why a step reads them rather than keeping a copy taken at
    // initialization, and why the check on them runs there too.
    /// Pure pursuit: meters left of the centerline to hold, making a lane.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().lateral_tgt)]
    lateral_tgt: f64,
    /// Pure pursuit: how far along the road to aim, in meters.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().lookahead)]
    lookahead: f64,
    /// Pure pursuit: yaw rate per radian of heading error, in 1/s.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().gain_yaw_rate)]
    gain_yaw_rate: f64,
    /// Pure pursuit: the most yaw rate to command either way, rad/s.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().max_yaw_rate)]
    max_yaw_rate: f64,

    /// IDM: speed held on an open road, m/s.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().v0_speed_tgt)]
    v0_speed_tgt: f64,
    /// IDM: seconds of gap wanted at whatever speed is held.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().t_headway)]
    t_headway: f64,
    /// IDM: meters of gap wanted at a standstill.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().s0_gap_min)]
    s0_gap_min: f64,
    /// IDM: the most acceleration commanded, m/s^2.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().a_accel_max)]
    a_accel_max: f64,
    /// IDM: comfortable braking, m/s^2 and positive, the hardest asked for.
    #[variable(causality = Parameter, variability = Tunable, start = Self::default().b_decel_comfort)]
    b_decel_comfort: f64,

    /// The car's acceleration at a full command of +1.0, in m/s^2.
    #[variable(causality = Parameter, variability = Fixed, start = Self::default().plant_accel_max)]
    plant_accel_max: f64,
    /// The car's deceleration at a full command of -1.0, in m/s^2 and positive.
    #[variable(causality = Parameter, variability = Fixed, start = Self::default().plant_decel_max)]
    plant_decel_max: f64,
    /// The car's yaw rate at a full command of +1.0, in rad/s.
    ///
    /// Separate from the steering law's `max_yaw_rate`, which is tuning:
    /// a follower told to turn no harder than half of this commands half
    /// a lock at most, and means it.
    #[variable(causality = Parameter, variability = Fixed, start = Self::default().plant_yaw_rate_max)]
    plant_yaw_rate_max: f64,

    /// The normalized acceleration command, -1.0 to +1.0, no unit.
    ///
    /// `initial = Calculated`, since it is worked out from the inputs
    /// rather than begun at anything. Saying so is what lists it under
    /// `InitialUnknowns`, and a `start` attribute beside it would be a
    /// second answer to the same question, which FMI forbids and a
    /// checking importer refuses to load.
    #[variable(causality = Output, initial = Calculated)]
    accel_cmd: f64,
    /// The normalized steering command, -1.0 for full right lock.
    #[variable(causality = Output, initial = Calculated)]
    steer_cmd: f64,

    /// The road, built from the parameters above and kept.
    ///
    /// No `#[variable]`, so FMI never sees it: it is ordinary Rust state
    /// living as long as the instance. Rebuilding it per step would
    /// reallocate and walk the whole polyline again for every car at
    /// every control instant, to get the same road back each time.
    road: Option<Waypoints>,
}

impl Default for FmuController {
    /// Every start value this FMU has, written once.
    ///
    /// The declarations above read their `start` from here, so what a
    /// host finds in the model description and what an instance holds
    /// before anything sets it cannot come apart. That costs building one
    /// of these per variable while the metadata is assembled, which
    /// happens at packaging and at instantiation and never in a step.
    fn default() -> Self {
        FmuController {
            position_x: 0.0,
            position_y: 0.0,
            orientation_w: 1.0,
            orientation_x: 0.0,
            orientation_y: 0.0,
            orientation_z: 0.0,
            speed: 0.0,
            range: [FREE_ROAD.range; MAX_DETECTIONS],
            range_rate: [FREE_ROAD.range_rate; MAX_DETECTIONS],
            road_x: [0.0; MAX_WAYPOINTS],
            road_y: [0.0; MAX_WAYPOINTS],
            road_point_count: 2,
            road_closed: false,
            lateral_tgt: DEFAULT_PURSUIT.lateral_tgt,
            lookahead: DEFAULT_PURSUIT.lookahead,
            gain_yaw_rate: DEFAULT_PURSUIT.gain_yaw_rate,
            max_yaw_rate: DEFAULT_PURSUIT.max_yaw_rate,
            v0_speed_tgt: DEFAULT_IDM.v0_speed_tgt,
            t_headway: DEFAULT_IDM.t_headway,
            s0_gap_min: DEFAULT_IDM.s0_gap_min,
            a_accel_max: DEFAULT_IDM.a_accel_max,
            b_decel_comfort: DEFAULT_IDM.b_decel_comfort,
            plant_accel_max: DEFAULT_LIMITS.accel_max,
            plant_decel_max: DEFAULT_LIMITS.decel_max,
            plant_yaw_rate_max: DEFAULT_LIMITS.yaw_rate_max,
            accel_cmd: 0.0,
            steer_cmd: 0.0,
            road: None,
        }
    }
}

impl FmuController {
    /// The road its parameters describe, or why they describe none.
    ///
    /// The count is checked before anything is built, because
    /// [`Waypoints`] asserts its own minimum and a panic unwinding out
    /// through the C interface would take the host process with it. The
    /// same information travels as a refusal instead, which the standard
    /// can carry.
    ///
    /// A road of no length is refused for a plainer reason. The point
    /// arrays start full of zeros, so a host that sets the count and
    /// forgets the points sends a road whose every point is the origin.
    /// Nothing in `Waypoints` objects to that, and a car steering along
    /// it holds still at a single spot. Better to say so.
    fn road_from_parameters(&self) -> Result<Waypoints, BadInput> {
        let given = self.road_point_count as usize;
        let (least, kind) = if self.road_closed {
            (3, "closed")
        } else {
            (2, "open")
        };
        if given < least || given > MAX_WAYPOINTS {
            return Err(BadInput::PointCount { given, least, kind });
        }
        let points: Vec<(f64, f64)> = (0..given)
            .map(|i| (self.road_x[i], self.road_y[i]))
            .collect();
        let road = if self.road_closed {
            Waypoints::build_closed(points)
        } else {
            Waypoints::build_open(points)
        };
        if road.total_length() <= 0.0 {
            return Err(BadInput::RoadOfNoLength { count: given });
        }

        // Return the road, built by the code that built the native
        // controller's road, so the two are one geometry rather than two
        // readings of the same numbers.
        Ok(road)
    }

    /// The road, reported to the host if there is none to build.
    fn build_road(&self, context: &dyn Context<Self>) -> Result<Waypoints, Fmi3Error> {
        self.road_from_parameters()
            .map_err(|bad| self.refuse(context, bad))
    }

    /// Reports to the host what it handed over that cannot be run on.
    fn refuse(&self, context: &dyn Context<Self>, bad: BadInput) -> Fmi3Error {
        context.log(
            Fmi3Error::Error.into(),
            DefaultLoggingCategory::LogAll,
            format_args!("{bad}"),
        );

        Fmi3Error::Error
    }

    /// The steering law's parameters, checked.
    ///
    /// Read afresh per step rather than kept, because these are tunable
    /// and a host may have changed one since the last. Keeping a copy
    /// would take the change and act on the old value, and checking once
    /// would let a new zero through.
    fn pursuit_params(&self) -> Result<PurePursuitParams, BadInput> {
        // Return the set, once the two that cannot take any value are
        // sound: an aim point at or behind the follower steers it the
        // wrong way, and a negative clamp panics inside the law.
        Ok(PurePursuitParams {
            lateral_tgt: self.lateral_tgt,
            lookahead: require_positive("lookahead", self.lookahead)?,
            gain_yaw_rate: self.gain_yaw_rate,
            max_yaw_rate: require_positive("max_yaw_rate", self.max_yaw_rate)?,
        })
    }

    /// The following law's parameters, checked, and read per step for
    /// the same reason [`Self::pursuit_params`] is.
    fn idm_params(&self) -> Result<IdmParams, BadInput> {
        // Return the set, once the three the equation divides by are
        // numbers it can divide by.
        Ok(IdmParams {
            v0_speed_tgt: require_positive("v0_speed_tgt", self.v0_speed_tgt)?,
            t_headway: self.t_headway,
            s0_gap_min: self.s0_gap_min,
            a_accel_max: require_positive("a_accel_max", self.a_accel_max)?,
            b_decel_comfort: require_positive("b_decel_comfort", self.b_decel_comfort)?,
        })
    }

    /// What to ask the plant for, given the road the laws command along.
    ///
    /// Every number here comes from [`continuo_actors::control_laws`]
    /// including the normalization. What this adds is the type conversions
    /// between them: a pose out of seven scalars, and a scan out of two
    /// detection arrays.
    fn calculate_commands(&self, road: &Waypoints) -> Result<(f64, f64), BadInput> {
        let pose = Pose {
            position: Vec3::new(self.position_x, self.position_y, 0.0),
            orientation: Quat {
                w: self.orientation_w,
                x: self.orientation_x,
                y: self.orientation_y,
                z: self.orientation_z,
            },
        };
        let pursuit_params = self.pursuit_params()?;
        let yaw_rate = pure_pursuit_yaw_rate(road, pose, pursuit_params);
        let steer_cmd = steer_fraction(
            yaw_rate,
            require_positive("plant_yaw_rate_max", self.plant_yaw_rate_max)?,
        );

        // The two arrays back into the detections they were taken from,
        // which is the shape the law reads. Padding loses to anything
        // real, so the count the sensor knew is not needed here.
        let scan: [Detection; MAX_DETECTIONS] = std::array::from_fn(|i| Detection {
            range: self.range[i],
            range_rate: self.range_rate[i],
        });
        let lead = nearest_detection(&scan);
        // A range closes as it falls and an approach rate rises as it
        // closes, so each is the other's negative.
        let accel = idm_accel(self.speed, lead.range, -lead.range_rate, self.idm_params()?);

        let accel_cmd = accel_fraction(
            accel,
            require_positive("plant_accel_max", self.plant_accel_max)?,
            require_positive("plant_decel_max", self.plant_decel_max)?,
        );

        // Return what to command: normalized acceleration and steering.
        Ok((accel_cmd, steer_cmd))
    }
}

impl UserModel for FmuController {
    /// The categories a host can switch on: `logAll` and `trace`, which
    /// is what `fmi-export` offers unless a model defines an enum of its
    /// own. One message is raised here and it belongs under `logAll`,
    /// since a refusal is not tracing.
    type LoggingCategory = DefaultLoggingCategory;

    /// Builds the road and commands from it, once the parameters are
    /// final.
    ///
    /// `fmi3ExitInitializationMode` calls this last, which is the first
    /// moment the parameters mean what the host meant and the last
    /// before an output is read for real. It is also the only place the
    /// road is ever built, so nothing downstream has to wonder which
    /// parameters it was built from.
    fn configurate(&mut self, context: &dyn Context<Self>) -> Result<(), Fmi3Error> {
        self.road = Some(self.build_road(context)?);
        // The outputs answer for the road just built, which is what a
        // host reading them straight afterward expects.
        self.calculate_values(context)?;

        Ok(())
    }

    /// Keeps the two output variables answering for the inputs.
    ///
    /// There is no road until `configurate` builds one, and this runs
    /// before it does. Both happen inside `fmi3ExitInitializationMode`,
    /// which recomputes whatever was dirtied and only then configures,
    /// and a host reading an output during initialization arrives
    /// earlier still. With no road to steer along there is nothing to
    /// command, so the outputs keep the values they were made with until
    /// there is one.
    fn calculate_values(&mut self, context: &dyn Context<Self>) -> Result<Fmi3Res, Fmi3Error> {
        let Some(road) = self.road.as_ref() else {
            return Ok(Fmi3Res::OK);
        };
        let (accel_cmd, steer_cmd) = self
            .calculate_commands(road)
            .map_err(|bad| self.refuse(context, bad))?;

        self.accel_cmd = accel_cmd;
        self.steer_cmd = steer_cmd;

        Ok(Fmi3Res::OK)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A controller holding `road` in its parameters, the way a host
    /// hands one across.
    fn controller_given(road: &Waypoints) -> FmuController {
        let mut controller = FmuController {
            road_point_count: road.points().len() as u32,
            road_closed: road.is_closed(),
            ..FmuController::default()
        };
        for (i, &(x, y)) in road.points().iter().enumerate() {
            controller.road_x[i] = x;
            controller.road_y[i] = y;
        }

        // Return it with the road written into the arrays and everything
        // past the road left as it was.
        controller
    }

    /// Where a road puts a car and which way it points it, sampled
    /// along and past both ends, as the bits of the answers.
    fn sampled_along(road: &Waypoints) -> Vec<(u64, u64, u64)> {
        (-10..=110)
            .map(|pct| {
                let s = road.total_length() * pct as f64 / 100.0;
                let point = road.point_at(s);
                (
                    point.x.to_bits(),
                    point.y.to_bits(),
                    road.heading_at(s).to_bits(),
                )
            })
            .collect()
    }

    #[test]
    fn a_road_arrives_as_the_road_it_was_sent_as() {
        // A bend and a loop, so the flag is asked for as well as the
        // points.
        for sent in [
            Waypoints::build_open(vec![(0.0, 0.0), (30.0, 0.0), (55.0, 18.0), (80.0, 18.0)]),
            Waypoints::build_closed(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]),
        ] {
            let rebuilt = controller_given(&sent)
                .road_from_parameters()
                .expect("the count is the road's own");
            assert_eq!(rebuilt.is_closed(), sent.is_closed());
            assert_eq!(sampled_along(&rebuilt), sampled_along(&sent));
        }
    }

    #[test]
    fn the_padding_past_the_count_is_never_read() {
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));
        let mut controller = controller_given(&road);
        // Somewhere no road would go, in every slot the road did not
        // fill. A reader that trusted the array over the count would
        // come back with a road hundreds of meters long.
        for i in road.points().len()..MAX_WAYPOINTS {
            controller.road_x[i] = 500.0 + i as f64;
            controller.road_y[i] = -500.0;
        }

        let rebuilt = controller
            .road_from_parameters()
            .expect("two points are enough for an open road");
        assert_eq!(rebuilt.total_length(), road.total_length());
        assert_eq!(sampled_along(&rebuilt), sampled_along(&road));
    }

    #[test]
    fn a_count_no_road_could_have_is_refused_rather_than_asserted() {
        let straight = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));

        // One point is not a road, and `Waypoints` would assert on it.
        let mut too_few = controller_given(&straight);
        too_few.road_point_count = 1;
        assert_eq!(
            too_few.road_from_parameters().unwrap_err(),
            BadInput::PointCount {
                given: 1,
                least: 2,
                kind: "open",
            }
        );

        // Two points close into nothing, so a loop needs three.
        let mut not_a_loop = controller_given(&straight);
        not_a_loop.road_closed = true;
        assert_eq!(
            not_a_loop.road_from_parameters().unwrap_err(),
            BadInput::PointCount {
                given: 2,
                least: 3,
                kind: "closed",
            }
        );

        // More points than arrived would read past what the host sent.
        let mut too_many = controller_given(&straight);
        too_many.road_point_count = MAX_WAYPOINTS as u32 + 1;
        assert_eq!(
            too_many.road_from_parameters().unwrap_err(),
            BadInput::PointCount {
                given: MAX_WAYPOINTS + 1,
                least: 2,
                kind: "open",
            }
        );
    }

    #[test]
    fn a_count_set_over_points_nobody_sent_is_refused() {
        // What a host gets by setting the count and forgetting the
        // points: the arrays start at zero, so every point of the road
        // is the origin. It builds without complaint, and a car given it
        // would sit still at one spot.
        let mut forgot_the_road = FmuController {
            road_point_count: 4,
            ..FmuController::default()
        };
        assert_eq!(
            forgot_the_road.road_from_parameters().unwrap_err(),
            BadInput::RoadOfNoLength { count: 4 }
        );

        // A closed road of repeated points goes the same way.
        forgot_the_road.road_closed = true;
        assert_eq!(
            forgot_the_road.road_from_parameters().unwrap_err(),
            BadInput::RoadOfNoLength { count: 4 }
        );
    }

    /// Asserts a controller commands nothing, naming the parameter it
    /// refused and quoting the value back.
    fn check_refusal(controller: &FmuController, road: &Waypoints, name: &str, given: f64) {
        match controller.calculate_commands(road) {
            Err(BadInput::NotPositive {
                name: refused,
                given: quoted,
            }) => {
                assert_eq!(refused, name);
                // Bits, because a NaN is not equal to itself.
                assert_eq!(quoted.to_bits(), given.to_bits());
            }
            other => panic!("{name} = {given} gave {other:?}"),
        }
    }

    #[test]
    fn a_parameter_used_for_division_is_refused_unless_it_is_positive() {
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));

        // A host in another tool can send any of these, and the model
        // description cannot warn it off, since `fmi-export` 0.3.0
        // declares no bounds for a variable.
        for given in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut target_speed = controller_given(&road);
            target_speed.v0_speed_tgt = given;
            check_refusal(&target_speed, &road, "v0_speed_tgt", given);

            let mut acceleration = controller_given(&road);
            acceleration.a_accel_max = given;
            check_refusal(&acceleration, &road, "a_accel_max", given);

            let mut braking = controller_given(&road);
            braking.b_decel_comfort = given;
            check_refusal(&braking, &road, "b_decel_comfort", given);

            let mut aim_point = controller_given(&road);
            aim_point.lookahead = given;
            check_refusal(&aim_point, &road, "lookahead", given);

            // The three the normalizing divides by
            let mut turn = controller_given(&road);
            turn.max_yaw_rate = given;
            check_refusal(&turn, &road, "max_yaw_rate", given);

            let mut plant_accel = controller_given(&road);
            plant_accel.plant_accel_max = given;
            check_refusal(&plant_accel, &road, "plant_accel_max", given);

            let mut plant_braking = controller_given(&road);
            plant_braking.plant_decel_max = given;
            check_refusal(&plant_braking, &road, "plant_decel_max", given);

            let mut plant_turn = controller_given(&road);
            plant_turn.plant_yaw_rate_max = given;
            check_refusal(&plant_turn, &road, "plant_yaw_rate_max", given);
        }
    }

    #[test]
    fn a_parameter_changed_between_steps_changes_the_command() {
        // What tunable buys, and what a step reading its parameters
        // afresh is for: a car told to want a different speed wants it
        // from the next command, without being rebuilt.
        const SPEED: f64 = 20.0;

        let road = Waypoints::build_straight((0.0, 0.0), (1000.0, 0.0));
        let mut controller = controller_given(&road);
        controller.speed = SPEED;

        let (wanting_default, _) = controller.calculate_commands(&road).expect("a sound set");
        controller.v0_speed_tgt = SPEED;
        let (wanting_this_speed, _) = controller.calculate_commands(&road).expect("a sound set");
        controller.v0_speed_tgt = SPEED * 2.0;
        let (wanting_twice_it, _) = controller.calculate_commands(&road).expect("a sound set");

        // Wanting exactly the speed it holds asks for nothing, and
        // wanting twice that asks for more than the default target did.
        assert!(
            wanting_this_speed.abs() < 1e-12,
            "at its target: {wanting_this_speed}"
        );
        assert!(wanting_twice_it > wanting_default);
    }

    #[test]
    fn the_default_parameters_pass_every_check() {
        // The commands come out finite for the defaults, so the checks
        // above should not refuse them.
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));
        let (accel_cmd, steer_cmd) = controller_given(&road)
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");
        assert!(accel_cmd.is_finite() && steer_cmd.is_finite());
    }

    #[test]
    fn the_commands_are_fractions_of_the_limits_rather_than_rates() {
        const HALF_A_PEDAL: f64 = 0.5;

        let road = Waypoints::build_straight((0.0, 0.0), (1000.0, 0.0));
        let mut controller = controller_given(&road);
        // At rest with nothing ahead, the law asks for exactly its own
        // `a_accel_max`, so a `plant_accel_max` of twice that has to be
        // asked for half a pedal.
        controller.speed = 0.0;
        controller.plant_accel_max = DEFAULT_IDM.a_accel_max / HALF_A_PEDAL;

        let (accel_cmd, _) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");
        assert!(
            (accel_cmd - HALF_A_PEDAL).abs() < 1e-12,
            "a law at its limit against twice it: {accel_cmd}"
        );

        // Twice `plant_accel_max` again for the same rate, which is what
        // names the divisor: a conversion taking the law's own limit
        // instead would answer 1.0 both times.
        controller.plant_accel_max *= 2.0;
        let (on_twice_the_limit, _) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");
        assert!(
            (on_twice_the_limit - HALF_A_PEDAL / 2.0).abs() < 1e-12,
            "{on_twice_the_limit}"
        );
    }

    #[test]
    fn a_law_capped_below_the_plant_commands_no_more_than_its_cap() {
        const HALF_A_LOCK: f64 = 0.5;

        // Pointing a quarter turn left of the road, so the law asks for
        // far more turn than either cap allows and what comes back is the
        // cap rather than the geometry. Negative, since a car pointing
        // left of its lane steers right to get back.
        let road = Waypoints::build_straight((0.0, 0.0), (400.0, 0.0));
        let facing_across = Quat::from_yaw(std::f64::consts::FRAC_PI_2);
        let mut controller = controller_given(&road);
        controller.orientation_w = facing_across.w;
        controller.orientation_z = facing_across.z;

        let (_, at_the_plants_turn) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");
        assert!(
            (at_the_plants_turn + 1.0).abs() < 1e-12,
            "capped where the car is: {at_the_plants_turn}"
        );

        // Half the turn as tuning, on the same car. A follower held to
        // half of what it could do commands half a lock and gets it,
        // which is the config a single number could not express.
        controller.max_yaw_rate = controller.plant_yaw_rate_max * HALF_A_LOCK;
        let (_, held_to_half) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");
        assert!(
            (held_to_half + HALF_A_LOCK).abs() < 1e-12,
            "held to half the turn: {held_to_half}"
        );

        // And a cap above the plant asks for a full lock rather than for
        // more than one, which is the clamp inside the conversion.
        controller.max_yaw_rate = controller.plant_yaw_rate_max * 2.0;
        let (_, past_the_plant) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");
        assert!(
            (past_the_plant + 1.0).abs() < 1e-12,
            "allowed more turn than the car has: {past_the_plant}"
        );
    }

    #[test]
    fn the_commands_are_what_the_laws_answer_to_the_bit() {
        // A car off its lane behind a closing lead, so both laws have
        // something to say. The slot is nowhere near the front, so
        // picking the lead out of the scan is part of what is checked.
        const POSITION: (f64, f64) = (25.0, 1.5);
        const ORIENTATION: (f64, f64) = (0.9, 0.1);
        const SPEED: f64 = 22.0;
        const LANE: f64 = 3.5;
        const SLOT: usize = 37;
        const GAP: f64 = 40.0;
        const CLOSING_AT: f64 = 4.0;

        let road = Waypoints::build_open(vec![(0.0, 0.0), (60.0, 0.0), (110.0, 40.0)]);
        let mut controller = controller_given(&road);
        controller.position_x = POSITION.0;
        controller.position_y = POSITION.1;
        controller.orientation_w = ORIENTATION.0;
        controller.orientation_z = ORIENTATION.1;
        controller.speed = SPEED;
        controller.lateral_tgt = LANE;
        controller.range[SLOT] = GAP;
        controller.range_rate[SLOT] = -CLOSING_AT;

        let (accel_cmd, steer_cmd) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");

        let pose = Pose {
            position: Vec3::new(POSITION.0, POSITION.1, 0.0),
            orientation: Quat {
                w: ORIENTATION.0,
                x: 0.0,
                y: 0.0,
                z: ORIENTATION.1,
            },
        };
        let expected_steer = steer_fraction(
            pure_pursuit_yaw_rate(
                &road,
                pose,
                PurePursuitParams {
                    lateral_tgt: LANE,
                    ..DEFAULT_PURSUIT
                },
            ),
            DEFAULT_PURSUIT.max_yaw_rate,
        );
        let expected_accel = accel_fraction(
            idm_accel(SPEED, GAP, CLOSING_AT, DEFAULT_IDM),
            DEFAULT_LIMITS.accel_max,
            DEFAULT_LIMITS.decel_max,
        );

        assert_eq!(steer_cmd.to_bits(), expected_steer.to_bits());
        assert_eq!(accel_cmd.to_bits(), expected_accel.to_bits());
        // The approach rate is the range rate's negative, so a lead
        // closing must not read as one pulling away.
        assert!(
            accel_cmd
                < accel_fraction(
                    idm_accel(SPEED, GAP, -CLOSING_AT, DEFAULT_IDM),
                    DEFAULT_LIMITS.accel_max,
                    DEFAULT_LIMITS.decel_max,
                ),
            "a closing lead should be braked for harder than a receding one"
        );
    }

    #[test]
    fn an_empty_scan_drives_the_open_road() {
        // Under the speed it wants, whatever that has been set to, so
        // the assertion below holds for any target rather than for 30.
        let speed = DEFAULT_IDM.v0_speed_tgt / 3.0;
        let road = Waypoints::build_straight((0.0, 0.0), (500.0, 0.0));
        let mut controller = controller_given(&road);
        controller.speed = speed;

        let (accel_cmd, _) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");

        // Nothing was detected, so every slot holds the free road, and
        // the answer is the one an open road gives at this speed.
        assert_eq!(
            accel_cmd.to_bits(),
            accel_fraction(
                idm_accel(speed, FREE_ROAD.range, 0.0, DEFAULT_IDM),
                DEFAULT_LIMITS.accel_max,
                DEFAULT_LIMITS.decel_max,
            )
            .to_bits()
        );
        assert!(accel_cmd > 0.0, "below its target it should accelerate");
    }
}
