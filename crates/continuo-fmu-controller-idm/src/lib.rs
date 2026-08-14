//! The demo's car controller, exported as an FMI 3.0 Co-Simulation FMU.
//!
//! Nothing here decides anything. The interface declaration below says
//! what crosses the boundary, and every answer comes from
//! [`continuo_actors::control_laws`], which the native controller calls
//! too. So the two agree because they are one implementation rather than
//! because somebody keeps them in step.
//!
//! What the boundary costs is worth seeing plainly. A road cannot cross
//! as a [`Waypoints`], so it crosses as the numbers it was built from and
//! each instance builds its own copy. A scan cannot cross as a list, so
//! it crosses as two arrays of a fixed length, padded out with the free
//! road. Both are the FMI data model rather than a choice, and both are
//! why this crate exists at all: the laws stay where they are, and only
//! the packaging lives here.
//!
//! The `.fmu` carries a compiled snapshot of `continuo-actors`, since the
//! cdylib links it statically. Editing a law without rebundling therefore
//! leaves this copy behind, and `cargo xtask bundle-fmus` is what puts it
//! back.

use std::path::PathBuf;

use continuo_actors::control_laws::{
    FREE_ROAD, IdmParams, PurePursuitParams, idm_accel, nearest_detection, pure_pursuit_yaw_rate,
};
use continuo_actors::{MAX_DETECTIONS, Waypoints};
use continuo_core::{Detection, Pose, Quat, Vec3};
use fmi::fmi3::{Fmi3Error, Fmi3Res};
use fmi_export::fmi3::{Context, DefaultLoggingCategory, UserModel};
use fmi_export::{FmuModel, export_fmu};

/// The file `cargo xtask bundle-fmus` writes, named after the cdylib
/// because FMI takes its model identifier from the shared library.
pub const FMU_FILE_NAME: &str = "continuo_fmu_controller_idm.fmu";

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

/// A car controller: IDM for how fast to go, pure pursuit for where to
/// point, from one pose and one radar scan.
///
/// Each variable's **first doc line is its FMI description**, and the
/// bundler takes that line alone. So a first line has to be a whole
/// sentence, and anything longer goes on the lines below it, which stay
/// here for whoever reads the code.
#[derive(FmuModel, Debug)]
#[model(co_simulation = true, model_exchange = false, user_model = false)]
pub struct IdmController {
    /// Where the car is, meters east.
    #[variable(causality = Input, name = "position.x", start = 0.0)]
    position_x: f64,
    /// Where the car is, meters north.
    #[variable(causality = Input, name = "position.y", start = 0.0)]
    position_y: f64,
    /// Which way the car points: the quaternion's scalar part.
    #[variable(causality = Input, name = "orientation.w", start = 1.0)]
    orientation_w: f64,
    /// Which way the car points: the quaternion's x part.
    #[variable(causality = Input, name = "orientation.x", start = 0.0)]
    orientation_x: f64,
    /// Which way the car points: the quaternion's y part.
    #[variable(causality = Input, name = "orientation.y", start = 0.0)]
    orientation_y: f64,
    /// Which way the car points: the quaternion's z part.
    #[variable(causality = Input, name = "orientation.z", start = 0.0)]
    orientation_z: f64,
    /// How fast the car is going, m/s, as the car itself reports it.
    ///
    /// A radar measures nothing about the car carrying it, so this
    /// arrives from the pose, standing in for a wheel speed sensor.
    #[variable(causality = Input, start = 0.0)]
    speed: f64,

    /// Meters to each thing the radar found ahead, free road past those.
    ///
    /// A slot the radar did not fill holds [`FREE_ROAD`], which loses to
    /// anything real, so nothing has to say how many are worth reading.
    #[variable(causality = Input, start = [FREE_ROAD.range; MAX_DETECTIONS])]
    range: [f64; MAX_DETECTIONS],
    /// How fast each range is changing, m/s, negative while closing.
    #[variable(causality = Input, start = [FREE_ROAD.range_rate; MAX_DETECTIONS])]
    range_rate: [f64; MAX_DETECTIONS],

    // The road is fixed where the law parameters below are tunable, and
    // the difference is real rather than cautious. Each instance builds
    // its `Waypoints` once and keeps it, so a road changed after
    // initialization would be accepted and then ignored. Declaring it
    // fixed is what tells a host that before it tries.
    /// The road's points, meters east, the first road_point_count real.
    #[variable(causality = Parameter, variability = Fixed, start = [0.0; MAX_WAYPOINTS])]
    road_x: [f64; MAX_WAYPOINTS],
    /// The road's points, meters north, the first road_point_count real.
    #[variable(causality = Parameter, variability = Fixed, start = [0.0; MAX_WAYPOINTS])]
    road_y: [f64; MAX_WAYPOINTS],
    /// How many of those points are the road, the rest being padding.
    #[variable(causality = Parameter, variability = Fixed, start = 2u32)]
    road_point_count: u32,
    /// Whether the road wraps at its end rather than stopping there.
    ///
    /// True when the last point joins back to the first. Two roads with
    /// the same points and different answers here are different roads.
    #[variable(causality = Parameter, variability = Fixed, start = false)]
    road_closed: bool,

    // Tunable, so a host may change any of these between steps: a car
    // wanting a different speed, or a different lane, says so here. That
    // is why a step reads them rather than keeping a copy taken at
    // initialization, and why the check on them runs there too.
    /// Meters left of the centerline to hold, which is what makes a lane.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_PURSUIT.lateral_tgt)]
    lateral_tgt: f64,
    /// How far along the road to aim, in meters.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_PURSUIT.lookahead)]
    lookahead: f64,
    /// Yaw rate per radian of heading error, in 1/s.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_PURSUIT.gain_yaw_rate)]
    gain_yaw_rate: f64,
    /// The most yaw rate to command either way, rad/s.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_PURSUIT.max_yaw_rate)]
    max_yaw_rate: f64,

    /// Speed held on an open road, m/s.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_IDM.v0_speed_tgt)]
    v0_speed_tgt: f64,
    /// Seconds of gap wanted at whatever speed is held.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_IDM.t_headway)]
    t_headway: f64,
    /// Meters of gap wanted at a standstill.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_IDM.s0_gap_min)]
    s0_gap_min: f64,
    /// The most acceleration commanded, m/s^2.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_IDM.a_accel_max)]
    a_accel_max: f64,
    /// Comfortable braking, m/s^2 and positive, the hardest commanded.
    #[variable(causality = Parameter, variability = Tunable, start = DEFAULT_IDM.b_decel_comfort)]
    b_decel_comfort: f64,

    /// Acceleration to hold, m/s^2.
    ///
    /// Calculated rather than started, since it is worked out from the
    /// inputs. Saying so is what lists it among the initial unknowns, and
    /// a start value beside it would be a second answer to the same
    /// question, which a checking importer refuses to load.
    #[variable(causality = Output, initial = Calculated)]
    accel_cmd: f64,
    /// Yaw rate to hold, rad/s, positive counter-clockwise.
    #[variable(causality = Output, initial = Calculated)]
    yaw_rate_cmd: f64,

    /// The road, built from the parameters above and kept.
    ///
    /// No `#[variable]`, so FMI never sees it: it is ordinary Rust state
    /// living as long as the instance. Rebuilding it per step would
    /// reallocate and walk the whole polyline again for every car at
    /// every control instant, to get the same road back each time.
    road: Option<Waypoints>,
}

impl Default for IdmController {
    /// The same values the variables declare as their starts.
    ///
    /// Both are needed and neither is the other's copy: the declared
    /// starts are what a host reads out of the model description, and
    /// these are what the struct holds before anything sets it. Naming
    /// the constants twice is what keeps them one set of numbers.
    fn default() -> Self {
        IdmController {
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
            accel_cmd: 0.0,
            yaw_rate_cmd: 0.0,
            road: None,
        }
    }
}

/// Something the FMU was handed that no controller could run on.
///
/// Each carries the value that arrived, because a host setting a
/// parameter from another tool has no other way to see what it sent. The
/// text is the whole of what crosses back: FMI carries a status and a log
/// line, not a structured error.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
enum BadInput {
    #[error("road_point_count is {given}, and {kind} roads need {least} to {MAX_WAYPOINTS}")]
    PointCount {
        given: usize,
        least: usize,
        kind: &'static str,
    },

    #[error("the first {count} road points are all the same place, which is a road of no length")]
    RoadOfNoLength { count: usize },

    #[error("{name} is {given}, and it has to be a positive number")]
    NotPositive { name: &'static str, given: f64 },
}

/// A parameter the laws divide by, checked before they see it.
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

impl IdmController {
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
            DefaultLoggingCategory::default(),
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
        // Return the set, once the aim point is somewhere ahead. An aim
        // point on top of the follower gives no direction to steer
        // toward, and one behind it gives the wrong one.
        Ok(PurePursuitParams {
            lateral_tgt: self.lateral_tgt,
            lookahead: require_positive("lookahead", self.lookahead)?,
            gain_yaw_rate: self.gain_yaw_rate,
            max_yaw_rate: self.max_yaw_rate,
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

    /// What the laws command, given the road they command along.
    ///
    /// Every number here comes from [`continuo_actors::control_laws`].
    /// What this adds is the translation either side of it: a pose out of
    /// seven scalars, and a scan out of two arrays.
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
        let yaw_rate_cmd = pure_pursuit_yaw_rate(road, pose, self.pursuit_params()?);

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
        let accel_cmd = idm_accel(self.speed, lead.range, -lead.range_rate, self.idm_params()?);

        // Return what to hold: how hard to push, and how fast to turn.
        Ok((accel_cmd, yaw_rate_cmd))
    }
}

impl UserModel for IdmController {
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
        let (accel_cmd, yaw_rate_cmd) = self
            .calculate_commands(road)
            .map_err(|bad| self.refuse(context, bad))?;

        self.accel_cmd = accel_cmd;
        self.yaw_rate_cmd = yaw_rate_cmd;

        Ok(Fmi3Res::OK)
    }
}

export_fmu!(IdmController);

/// Why the bundled FMU could not be found.
#[derive(Debug, thiserror::Error)]
pub enum BundleError {
    #[error("cannot find this test's own executable: {0}")]
    CurrentExe(#[source] std::io::Error),

    #[error(
        "no fmu/{FMU_FILE_NAME} above {}: run `cargo install cargo-fmi` once, \
         then `cargo xtask bundle-fmus`",
        searched_from.display()
    )]
    NotBundled { searched_from: PathBuf },
}

/// Where `cargo xtask bundle-fmus` left the packaged FMU.
///
/// Found by walking up from the running executable rather than from a
/// build-time path, so it works the same from a test binary, an example
/// and a tool, whichever profile built them.
pub fn bundled_fmu_path() -> Result<PathBuf, BundleError> {
    let exe = std::env::current_exe().map_err(BundleError::CurrentExe)?;
    for directory in exe.ancestors() {
        let candidate = directory.join("fmu").join(FMU_FILE_NAME);
        if candidate.is_file() {
            return Ok(candidate);
        }
    }

    // Return the failure carrying the command that fixes it, since a
    // missing bundle is a step not run rather than anything broken.
    Err(BundleError::NotBundled { searched_from: exe })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A controller carrying `road` as its parameters, the way a host
    /// hands one across.
    fn carrying(road: &Waypoints) -> IdmController {
        let mut controller = IdmController {
            road_point_count: road.points().len() as u32,
            road_closed: road.is_closed(),
            ..IdmController::default()
        };
        for (i, &(x, y)) in road.points().iter().enumerate() {
            controller.road_x[i] = x;
            controller.road_y[i] = y;
        }

        // Return it with the road written into the arrays and everything
        // past the road left as it was.
        controller
    }

    /// Every answer a road gives, as the bits it gives them in.
    fn answers(road: &Waypoints) -> Vec<(u64, u64, u64)> {
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
            let arrived = carrying(&sent)
                .road_from_parameters()
                .expect("the count is the road's own");
            assert_eq!(arrived.is_closed(), sent.is_closed());
            assert_eq!(answers(&arrived), answers(&sent));
        }
    }

    #[test]
    fn the_padding_past_the_count_is_never_read() {
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));
        let mut controller = carrying(&road);
        // Somewhere no road would go, in every slot the road did not
        // fill. A reader that trusted the array over the count would
        // come back with a road hundreds of meters long.
        for i in road.points().len()..MAX_WAYPOINTS {
            controller.road_x[i] = 500.0 + i as f64;
            controller.road_y[i] = -500.0;
        }

        let arrived = controller
            .road_from_parameters()
            .expect("two points are enough for an open road");
        assert_eq!(arrived.total_length(), road.total_length());
        assert_eq!(answers(&arrived), answers(&road));
    }

    #[test]
    fn a_count_no_road_could_have_is_refused_rather_than_asserted() {
        let straight = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));

        // One point is not a road, and `Waypoints` would assert on it.
        let mut too_few = carrying(&straight);
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
        let mut not_a_loop = carrying(&straight);
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
        let mut too_many = carrying(&straight);
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
        let mut forgot_the_road = IdmController {
            road_point_count: 4,
            ..IdmController::default()
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
    fn check_refusal(controller: &IdmController, road: &Waypoints, name: &str, given: f64) {
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
    fn a_parameter_the_laws_divide_by_is_refused_unless_it_is_positive() {
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));

        // A host in another tool can send any of these, and the model
        // description cannot warn it off, since `fmi-export` 0.3.0
        // declares no bounds for a variable.
        for given in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut target_speed = carrying(&road);
            target_speed.v0_speed_tgt = given;
            check_refusal(&target_speed, &road, "v0_speed_tgt", given);

            let mut acceleration = carrying(&road);
            acceleration.a_accel_max = given;
            check_refusal(&acceleration, &road, "a_accel_max", given);

            let mut braking = carrying(&road);
            braking.b_decel_comfort = given;
            check_refusal(&braking, &road, "b_decel_comfort", given);

            let mut aim_point = carrying(&road);
            aim_point.lookahead = given;
            check_refusal(&aim_point, &road, "lookahead", given);
        }
    }

    #[test]
    fn a_parameter_changed_between_steps_changes_the_command() {
        // What tunable buys, and what a step reading its parameters
        // afresh is for: a car told to want a different speed wants it
        // from the next command, without being rebuilt.
        let road = Waypoints::build_straight((0.0, 0.0), (1000.0, 0.0));
        let mut controller = carrying(&road);
        controller.speed = 20.0;

        let (holding_20, _) = controller.calculate_commands(&road).expect("a sound set");
        controller.v0_speed_tgt = 20.0;
        let (wanting_20, _) = controller.calculate_commands(&road).expect("a sound set");
        controller.v0_speed_tgt = 40.0;
        let (wanting_40, _) = controller.calculate_commands(&road).expect("a sound set");

        // Wanting exactly the speed it holds asks for nothing; wanting
        // more asks for more than the default target did.
        assert!(wanting_20.abs() < 1e-12, "at its target: {wanting_20}");
        assert!(wanting_40 > holding_20);
    }

    #[test]
    fn every_parameter_the_laws_divide_by_is_checked() {
        // The commands come out finite for the defaults, so the check
        // above is refusing what is wrong rather than everything.
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));
        let (accel_cmd, yaw_rate_cmd) = carrying(&road)
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");
        assert!(accel_cmd.is_finite() && yaw_rate_cmd.is_finite());
    }

    #[test]
    fn the_commands_are_what_the_laws_answer_to_the_bit() {
        let road = Waypoints::build_open(vec![(0.0, 0.0), (60.0, 0.0), (110.0, 40.0)]);
        let mut controller = carrying(&road);
        controller.position_x = 25.0;
        controller.position_y = 1.5;
        controller.orientation_z = 0.1;
        controller.orientation_w = 0.9;
        controller.speed = 22.0;
        controller.lateral_tgt = 3.5;
        // A lead 40 m off and closing, in a slot nowhere near the front,
        // so picking it out is part of what is being checked.
        controller.range[37] = 40.0;
        controller.range_rate[37] = -4.0;

        let (accel_cmd, yaw_rate_cmd) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");

        let pose = Pose {
            position: Vec3::new(25.0, 1.5, 0.0),
            orientation: Quat {
                w: 0.9,
                x: 0.0,
                y: 0.0,
                z: 0.1,
            },
        };
        let expected_yaw_rate = pure_pursuit_yaw_rate(
            &road,
            pose,
            PurePursuitParams {
                lateral_tgt: 3.5,
                lookahead: DEFAULT_PURSUIT.lookahead,
                gain_yaw_rate: DEFAULT_PURSUIT.gain_yaw_rate,
                max_yaw_rate: DEFAULT_PURSUIT.max_yaw_rate,
            },
        );
        let expected_accel = idm_accel(22.0, 40.0, 4.0, DEFAULT_IDM);

        assert_eq!(yaw_rate_cmd.to_bits(), expected_yaw_rate.to_bits());
        assert_eq!(accel_cmd.to_bits(), expected_accel.to_bits());
        // The approach rate is the range rate's negative, so a lead
        // closing at 4 m/s must not read as one pulling away.
        assert!(
            accel_cmd < idm_accel(22.0, 40.0, -4.0, DEFAULT_IDM),
            "a closing lead should be braked for harder than a receding one"
        );
    }

    #[test]
    fn an_empty_scan_drives_the_open_road() {
        let road = Waypoints::build_straight((0.0, 0.0), (500.0, 0.0));
        let mut controller = carrying(&road);
        controller.speed = 10.0;

        let (accel_cmd, _) = controller
            .calculate_commands(&road)
            .expect("the defaults are a set the laws can run on");

        // Nothing was detected, so every slot holds the free road, and
        // the answer is the one an open road gives at this speed.
        assert_eq!(
            accel_cmd.to_bits(),
            idm_accel(10.0, FREE_ROAD.range, 0.0, DEFAULT_IDM).to_bits()
        );
        assert!(accel_cmd > 0.0, "an open road below v0 should accelerate");
    }
}
