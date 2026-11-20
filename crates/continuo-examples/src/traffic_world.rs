//! The demo world shared by every traffic example: an ego car driving a
//! straight highway, with traffic spawning ahead of it in the neighboring
//! lanes and retiring once it has gone past.
//!
//! Two scenarios, kept side by side so the difference between them is one
//! diff rather than two files: [`setup_live_traffic_scenario`], whose cars
//! a spawner decides on while the run happens, and
//! [`setup_playback_traffic_scenario`], whose cars are doubles of a
//! recorded run.
//!
//! Each comes in two halves, a `setup_` and a `run_`, shared so the
//! examples cannot drift apart. [`run_live_traffic_scenario`] applies the
//! spawner's requests and checks for collisions between ticks.
//! [`run_playback_traffic_scenario`] has nothing to apply, since recorded
//! cars are already a schedule.
//!
//! Everything is generic over the transport, so an example picks its own:
//! plain `InProcTransport` for the base demo, wrapped in a
//! `MonitorTransport` when recording or verifying.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex, OnceLock};

use continuo_actors::control_laws::{FREE_ROAD, IdmParams, PurePursuitParams};
use continuo_actors::{
    CarState, DespawnTrafficRequest, PathFollowController, PlantLimits, RadarSensor,
    SpawnTrafficRequest, TrafficSpawner, UnicyclePhysics, Waypoints, accel_cmd_key,
    actor_of_pose_key, pose_key, radar_key, road_pose, steer_cmd_key, straight_road,
    traffic_despawn_key, traffic_spawn_key,
};
use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, ConductorError, EventLog, JoinMetadata, LeaveMetadata,
    MembershipChange, Pacing, PlaybackComponent, RecordedCollision, RecordedObservation, Verifier,
    WORLD_LEVEL,
};
use continuo_core::{
    Component, ComponentId, ComponentPath, CoreError, KeyExpr, Message, Pose, SimDuration, SimTime,
};
use continuo_fmi::{FmuComponent, FmuConstructionError, FmuMapping, InputBinding, OutputBinding};
use continuo_transport::Transport;
use serde_json::json;

use crate::collision::CollisionMonitor;

/// One seed for the whole demo family: record, verify, and resim must all
/// agree on it.
pub const WORLD_SEED: u64 = 42;
pub const WORLD_NAME: &str = "demo";
pub const SIM_SECONDS: i64 = 30;

/// The car the scenario is arranged around. Alone in its lane, and faster
/// than the traffic, so it spends the run overtaking.
const EGO_NAME: &str = "ego";

/// Where the ego starts: at the road's origin, in the centre lane. Unlike
/// [`EGO_SPEED`] these are not tuned against anything. They are what the
/// rest of the scenario is measured from, which is why traffic spawns
/// relative to the ego and the road is long enough for where it ends up.
const EGO_START: f64 = 0.0;
const EGO_LANE: f64 = 0.0;

/// Enough road for the ego's 30 s at 30 m/s, plus the stretch ahead that
/// traffic spawns into.
const ROAD_LENGTH: f64 = 1600.0;
const LANE_WIDTH: f64 = 3.5;
/// Lanes traffic may use: either side of the ego, never [`EGO_LANE`]
/// itself: nothing here models a collision, so traffic goes beside the ego
/// rather than in front of it, and the overtaking stays clean.
const TRAFFIC_LANES: [f64; 2] = [LANE_WIDTH, -LANE_WIDTH];

// Sized so the run actually shows what it is meant to. The ego closes on
// traffic at 8-14 m/s, and the six cars start between 40 m and roughly
// 250 m ahead, so the first is overtaken within seconds and the last
// before the run ends, with replacements spawning ahead as they go. Slow
// the traffic down or push it further ahead and the demo becomes thirty
// seconds of nothing being passed.
const EGO_SPEED: f64 = 30.0;
const TRAFFIC_SPEED: (f64, f64) = (16.0, 22.0);
const TRAFFIC_POPULATION: usize = 6;
const SPAWN_AHEAD: f64 = 40.0;
const RETIRE_BEHIND: f64 = 60.0;
const SPAWN_GAP: (f64, f64) = (20.0, 50.0);

/// What a full command is worth on every car in this world.
const CAR_LIMITS: PlantLimits = PlantLimits::highway_car();

/// How often a car decides what to do: its radar scans and its controller
/// commands at this period, and its physics integrates ten times between.
const CONTROL_PERIOD: SimDuration = SimDuration::from_millis(100);
const PHYSICS_PERIOD: SimDuration = SimDuration::from_millis(10);

/// How far ahead a car's radar sees, in meters.
const RADAR_MAX_RANGE: f64 = 120.0;

/// Half a lane either side, so a car counts as sharing the radar's lane
/// when it is nearer that lane's center than any other.
const RADAR_LANE_TOLERANCE: f64 = LANE_WIDTH / 2.0;

/// Which components a car is built from, and so how it drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CarConfiguration {
    /// Holds the speed it started at whatever is ahead, steering with
    /// the native controller: `[controller, physics]`.
    ConstantSpeed,
    /// Follows the car ahead in its lane on the controller FMU, and wants
    /// the speed it started at whenever the road ahead is free:
    /// `[radar, controller, physics]`.
    IdmFollowing,
}

/// Why a traffic scenario could not be built or run.
#[derive(Debug, thiserror::Error)]
pub enum ScenarioError {
    #[error(transparent)]
    Conductor(#[from] ConductorError),

    /// Nobody packaged the controller FMU, so no following car can be
    /// built. The message names the command that fixes it.
    #[error(transparent)]
    NotPackaged(#[from] continuo_fmu_controller_idm::PackageError),

    #[error("cannot build the controller FMU for {actor_name}: {source}")]
    Controller {
        actor_name: String,
        #[source]
        source: FmuConstructionError,
    },

    /// The ego touched another car, which its controller should prevent.
    #[error("{} and {} collided at sim time {}", .0.first, .0.second, .0.sim_time)]
    EgoCollision(RecordedCollision),
}

/// A handle on the one road every car in this world drives, built once and
/// handed out, not rebuilt per call, which is what keeps a run that spawns
/// twenty cars from carrying twenty copies of the same geometry.
///
/// Immutable once built, so sharing it across the several worlds a test
/// process may run is safe and means nothing.
fn shared_road() -> Arc<Waypoints> {
    static ROAD: OnceLock<Arc<Waypoints>> = OnceLock::new();

    // Return another handle on the single road.
    ROAD.get_or_init(|| Arc::new(straight_road(ROAD_LENGTH)))
        .clone()
}

/// The demo world's conductor configuration (free-run).
pub fn config() -> ConductorConfig {
    // Return the free-run config; `config_paced` shares everything but the
    // pacing mode, so every mode runs the identical world.
    config_paced(Pacing::FreeRun)
}

/// The demo world's conductor configuration with the pacing mode chosen
/// explicitly. Same seed and name whatever the pacing, so all modes produce
/// the same world hash.
pub fn config_paced(pacing: Pacing) -> ConductorConfig {
    ConductorConfig {
        world_name: WORLD_NAME.into(),
        world_seed: WORLD_SEED,
        pacing,
    }
}

/// Registers one car under `actor_name`, built as `car_config` says,
/// starting `start_s` along the road in the lane at `lane_offset` at
/// `speed`.
///
/// The components register in the order they run within an instant:
/// radar, controller, physics. So the scan reaches the controller and the
/// command reaches the physics in the instant each was made.
fn add_car<T: Transport>(
    conductor: &mut Conductor<T>,
    actor_name: &str,
    car_config: CarConfiguration,
    start_s: f64,
    lane_offset: f64,
    speed: f64,
    first_due: SimTime,
) -> Result<(), ScenarioError> {
    let initial_state = CarState::new(road_pose(&shared_road(), start_s, lane_offset), speed);
    let world_name = conductor.world_name();

    let components = match car_config {
        CarConfiguration::ConstantSpeed => vec![
            native_controller_component(actor_name, lane_offset, initial_state),
            physics_component(actor_name, initial_state),
        ],
        CarConfiguration::IdmFollowing => vec![
            radar_component(actor_name),
            fmu_controller_component(world_name, actor_name, lane_offset, initial_state)?,
            physics_component(actor_name, initial_state),
        ],
    };
    for component in components {
        conductor.add_component(JoinMetadata::at(actor_name, first_due), component)?;
    }

    // Return success; the car is registered.
    Ok(())
}

/// A car's radar, scanning its own lane ahead.
fn radar_component(actor_name: &str) -> Box<dyn Component> {
    Box::new(RadarSensor::new(
        actor_name,
        shared_road(),
        CONTROL_PERIOD,
        RADAR_MAX_RANGE,
        RADAR_LANE_TOLERANCE,
    ))
}

/// A car's native controller, steering along its lane and commanding no
/// acceleration.
fn native_controller_component(
    actor_name: &str,
    lane_offset: f64,
    initial_state: CarState,
) -> Box<dyn Component> {
    Box::new(PathFollowController::new(
        actor_name,
        shared_road(),
        lane_offset,
        CONTROL_PERIOD,
        6.0,                     // lookahead, m
        1.5,                     // heading gain, 1/s
        CAR_LIMITS.yaw_rate_max, // command turns up to the car's limit
        CAR_LIMITS,
        initial_state.pose(),
    ))
}

/// A car's controller FMU, run from the `.fmu` that
/// `cargo xtask package-fmus` builds and
/// [`continuo_fmu_controller_idm::packaged_fmu_path`] finds.
fn fmu_controller_component(
    world_name: &str,
    actor_name: &str,
    lane_offset: f64,
    initial_state: CarState,
) -> Result<Box<dyn Component>, ScenarioError> {
    let mapping = controller_fmu_mapping(world_name, actor_name, lane_offset, initial_state);
    let controller = FmuComponent::new(
        "controller",
        actor_name,
        continuo_fmu_controller_idm::packaged_fmu_path()?,
        mapping,
    )
    .map_err(|source| ScenarioError::Controller {
        actor_name: actor_name.to_string(),
        source,
    })?;

    // Return the controller, ready to join.
    Ok(Box::new(controller))
}

/// A car's physics, which owns its state and starts from `initial_state`.
fn physics_component(actor_name: &str, initial_state: CarState) -> Box<dyn Component> {
    Box::new(UnicyclePhysics::new(
        actor_name,
        PHYSICS_PERIOD,
        CAR_LIMITS,
        initial_state,
    ))
}

/// Wires one following car's controller FMU into the world: the car's pose
/// and radar scan in, its two commands out on the plant's keys, and the
/// road, tunings and plant limits set once as parameters.
///
/// Radar slots past the cars detected read as [`FREE_ROAD`]. The pose
/// inputs start at `initial_state`, since the controller steps once before
/// the physics has published a pose. Its `trace` category is switched off,
/// which would otherwise log every call, one line per car per step.
fn controller_fmu_mapping(
    world_name: &str,
    actor_name: &str,
    lane_offset: f64,
    initial_state: CarState,
) -> FmuMapping {
    let road = shared_road();
    let mut road_pts_x = vec![0.0; continuo_fmu_controller_idm::MAX_WAYPOINTS];
    let mut road_pts_y = vec![0.0; continuo_fmu_controller_idm::MAX_WAYPOINTS];
    for (slot, &(x, y)) in road.points().iter().enumerate() {
        road_pts_x[slot] = x;
        road_pts_y[slot] = y;
    }
    let pursuit = PurePursuitParams::highway_car(lane_offset);
    let idm = IdmParams::highway_car(initial_state.speed);

    let pose_msg_key = pose_key(world_name, actor_name);
    let radar_msg_key = radar_key(world_name, actor_name);
    let inputs = vec![
        InputBinding::new("position.x", pose_msg_key.clone()),
        InputBinding::new("position.y", pose_msg_key.clone()),
        InputBinding::new("orientation.w", pose_msg_key.clone()),
        InputBinding::new("orientation.x", pose_msg_key.clone()),
        InputBinding::new("orientation.y", pose_msg_key.clone()),
        InputBinding::new("orientation.z", pose_msg_key.clone()),
        InputBinding::new("speed", pose_msg_key),
        InputBinding::new("range", radar_msg_key.clone())
            .with_pointer("/detections/*/range")
            .when_missing(json!(FREE_ROAD.range)),
        InputBinding::new("range_rate", radar_msg_key)
            .with_pointer("/detections/*/range_rate")
            .when_missing(json!(FREE_ROAD.range_rate)),
    ];
    let outputs = vec![
        OutputBinding::new("accel_cmd", accel_cmd_key(world_name, actor_name)),
        OutputBinding::new("steer_cmd", steer_cmd_key(world_name, actor_name)),
    ];
    let parameters = [
        ("road.pts_x", json!(road_pts_x)),
        ("road.pts_y", json!(road_pts_y)),
        ("road.point_count", json!(road.points().len())),
        ("road.closed", json!(road.is_closed())),
        ("pure_pursuit.lateral_tgt", json!(pursuit.lateral_tgt)),
        ("pure_pursuit.lookahead", json!(pursuit.lookahead)),
        ("pure_pursuit.gain_yaw_rate", json!(pursuit.gain_yaw_rate)),
        ("pure_pursuit.max_yaw_rate", json!(pursuit.max_yaw_rate)),
        ("idm.v0_speed_tgt", json!(idm.v0_speed_tgt)),
        ("idm.t_headway", json!(idm.t_headway)),
        ("idm.s0_gap_min", json!(idm.s0_gap_min)),
        ("idm.a_accel_max", json!(idm.a_accel_max)),
        ("idm.b_decel_comfort", json!(idm.b_decel_comfort)),
        ("plant.accel_max", json!(CAR_LIMITS.accel_max)),
        ("plant.decel_max", json!(CAR_LIMITS.decel_max)),
        ("plant.yaw_rate_max", json!(CAR_LIMITS.yaw_rate_max)),
    ];
    let starting_inputs = [
        ("position.x", json!(initial_state.position.x)),
        ("position.y", json!(initial_state.position.y)),
        ("orientation.w", json!(initial_state.orientation.w)),
        ("orientation.x", json!(initial_state.orientation.x)),
        ("orientation.y", json!(initial_state.orientation.y)),
        ("orientation.z", json!(initial_state.orientation.z)),
        ("speed", json!(initial_state.speed)),
    ];

    // Return the sheet. It is data rather than code, so a scenario file
    // could carry it once scenarios come from files.
    FmuMapping {
        period: CONTROL_PERIOD,
        inputs,
        outputs,
        initial_values: parameters
            .into_iter()
            .chain(starting_inputs)
            .map(|(name, value)| (name.to_string(), value))
            .collect(),
        log_categories_off: Some(vec!["trace".to_string()]),
    }
}

/// Registers the ego with the initial conditions given: starting `start_s`
/// along the road, in the lane at `lane_offset`, holding `speed`.
///
/// All three are parameters because they are the knobs a what-if run
/// turns, and the ego is the component under study. Both setups pass
/// [`EGO_START`] and [`EGO_LANE`]; where they differ is the speed.
/// [`setup_live_traffic_scenario`] passes [`EGO_SPEED`] and
/// [`setup_playback_traffic_scenario`] takes whatever the experiment wants,
/// which is what keeps the two runs comparable to each other.
///
/// It always joins at sim time zero, which is not a parameter: an ego that
/// turned up partway through would not be the run's subject.
fn add_ego<T: Transport>(
    conductor: &mut Conductor<T>,
    start_s: f64,
    lane_offset: f64,
    speed: f64,
) -> Result<(), ScenarioError> {
    // Return once the ego is registered.
    add_car(
        conductor,
        EGO_NAME,
        CarConfiguration::ConstantSpeed,
        start_s,
        lane_offset,
        speed,
        SimTime::ZERO,
    )
}

/// Registers the traffic spawner, the component that decides which cars
/// exist while the run is under way.
///
/// Not needed by a run that plays traffic back from a log: there the
/// recording is the traffic, and a live spawner would be deciding on top of
/// it. See `traffic_resim`.
fn add_spawner<T: Transport>(conductor: &mut Conductor<T>) -> Result<(), ConductorError> {
    conductor.add_component_at_start(
        WORLD_LEVEL,
        Box::new(TrafficSpawner::new(
            SimDuration::from_millis(500),
            shared_road(),
            EGO_NAME,
            TRAFFIC_LANES.to_vec(),
            TRAFFIC_POPULATION,
            SPAWN_AHEAD,
            RETIRE_BEHIND,
            SPAWN_GAP,
            TRAFFIC_SPEED,
        )),
    )?;

    // Return success; the spawner is registered.
    Ok(())
}

/// Registers the live scenario's fixed cast: the ego and the spawner that
/// manages traffic around it. Every car arrives later, while the run is
/// under way, so this is the whole world only at sim time zero, and
/// [`run_live_traffic_scenario`] is what builds the rest of it.
///
/// *Live* because the traffic is decided as the run happens. `traffic_resim`
/// assembles the other kind by hand: the same ego, but no spawner, and
/// cars played back from a log rather than chosen.
pub fn setup_live_traffic_scenario<T: Transport>(
    conductor: &mut Conductor<T>,
) -> Result<(), ScenarioError> {
    add_ego(conductor, EGO_START, EGO_LANE, EGO_SPEED)?;
    add_spawner(conductor)?;

    // Return success; the fixed part of the world is registered.
    Ok(())
}

/// How far along the road each row of cars starts behind the one ahead,
/// once [`setup_scale_scenario`] has filled every lane and wrapped.
const SCALE_ROW_SPACING: f64 = 50.0;

/// The size the live demo holds at once, and the baseline a scaling run
/// measures against: the ego plus its traffic, across the ego's lane and the
/// traffic's.
///
/// Public so that a scaling run has something to measure against, namely the
/// size everything else in the project is tuned for. It is the *size* that
/// carries over and not the scenario: the demo decides its traffic while it
/// runs, and [`setup_scale_scenario`] holds the cast still.
pub const BASELINE_DEMO_CARS: usize = TRAFFIC_POPULATION + 1;
pub const BASELINE_DEMO_LANES: usize = TRAFFIC_LANES.len() + 1;

/// Registers `cars` cars across `lanes` lanes, for measuring what a world
/// costs as it grows rather than for watching anything happen in it.
///
/// No spawner, which is not an oversight: it would make the population a
/// moving target, which is the one thing a measurement of population wants
/// held still.
///
/// Cars fill the lanes and then wrap onto another row further along the
/// road, so `cars` need not divide evenly by `lanes`. Speeds follow a fixed
/// pattern rather than the live scenario's random spread: what is wanted is
/// a repeatable amount of work, not a plausible scene, and nothing here
/// avoids anything else because nothing here models a collision.
///
/// # Panics
///
/// If `lanes` is zero, which is not a world.
pub fn setup_scale_scenario<T: Transport>(
    conductor: &mut Conductor<T>,
    cars: usize,
    lanes: usize,
) -> Result<(), ScenarioError> {
    assert!(lanes > 0, "a world needs at least one lane");

    for index in 0..cars {
        let lane_offset = ((index % lanes) as f64 - (lanes - 1) as f64 * 0.5) * LANE_WIDTH;
        let start_s = (index / lanes) as f64 * SCALE_ROW_SPACING;
        let speed = TRAFFIC_SPEED.0 + (index % 7) as f64;
        add_car(
            conductor,
            &format!("car{index}"),
            CarConfiguration::ConstantSpeed,
            start_s,
            lane_offset,
            speed,
            SimTime::ZERO,
        )?;
    }

    // Return success; the whole cast is registered.
    Ok(())
}

/// Sets up the other scenario: the same ego, at `ego_speed` instead of the
/// scenario's own 30 m/s, against a playback double of every car
/// `recorded` contained. Returns how many playback doubles joined.
///
/// Line this up against [`setup_live_traffic_scenario`] and the whole
/// difference is two lines: no spawner, and the cars come from the log.
/// That is what makes an open-loop what-if a fair comparison: the traffic
/// cannot react to the ego, so the scene is held fixed while the ego
/// varies, and any difference in the outcome belongs to the ego alone.
///
/// Which cars existed, and when, is read out of the log rather than
/// configured, because it was the spawner's decision on the recorded run
/// and the log is the only place that now knows.
pub fn setup_playback_traffic_scenario<T: Transport>(
    conductor: &mut Conductor<T>,
    recorded: &EventLog,
    ego_speed: f64,
) -> Result<usize, ScenarioError> {
    add_ego(conductor, EGO_START, EGO_LANE, ego_speed)?;

    // Each double joins and leaves at the instants its original did, both
    // queued now, so the world's membership matches the recording.
    let cars = recorded_traffic_cars(recorded);
    for car in &cars {
        let id = ComponentId::new(car.name.as_str()).expect("a recorded path segment is an id");
        conductor.add_component(
            JoinMetadata::at(WORLD_LEVEL, car.joined_at),
            Box::new(PlaybackComponent::from_log(id, recorded, &car.name)),
        )?;
        if let Some(left_at) = car.left_at {
            conductor.remove_component(LeaveMetadata::at(car.name.as_str(), left_at))?;
        }
    }

    // Return the size of the scene that was rebuilt.
    Ok(cars.len())
}

/// A traffic car a recorded run contained, and when it joined and left.
struct RecordedCar {
    name: String,
    joined_at: SimTime,
    /// When it left, or `None` if it was still there when the run ended.
    left_at: Option<SimTime>,
}

/// The traffic cars a recorded run contained, in the order they joined.
///
/// A car is named by the first segment of its components' paths
/// (`traffic7/physics`). Its components all join and leave together, so
/// the first of each gives the car's instants. The spawner's path has
/// no `/`, so it is skipped, and the ego is skipped by name.
fn recorded_traffic_cars(recorded: &EventLog) -> Vec<RecordedCar> {
    let mut cars: Vec<RecordedCar> = Vec::new();
    for event in &recorded.events {
        match event {
            LogEvent::Join(join) => {
                let Some((name, _)) = join.path.split_once('/') else {
                    continue;
                };
                if name != EGO_NAME && !cars.iter().any(|car| car.name == name) {
                    cars.push(RecordedCar {
                        name: name.to_string(),
                        joined_at: join.first_due,
                        left_at: None,
                    });
                }
            }
            LogEvent::Leave(leave) => {
                let Some((name, _)) = leave.path.split_once('/') else {
                    continue;
                };
                if let Some(car) = cars.iter_mut().find(|car| car.name == name) {
                    car.left_at.get_or_insert(leave.leaves_at);
                }
            }
            _ => {}
        }
    }

    // Return each car once, first join first.
    cars
}

/// Handles a traffic scenario from outside the sim: turns the spawner's
/// requests into cars, watches every car for collisions, and prints the
/// run's console output. A playback scenario has no spawner, so it has no
/// requests.
#[derive(Clone)]
pub struct TrafficHandler {
    /// Requests collected since the last [`Self::apply_traffic_requests`],
    /// in publish order.
    pending_requests: Arc<Mutex<Vec<TrafficRequest>>>,
    /// The first request that could not be read, held until
    /// [`Self::apply_traffic_requests`] reports it.
    ///
    /// The collecting callback is an `FnMut(&Message)` with nowhere to return
    /// an error to, so a request it cannot decode is put here instead. First
    /// one wins: the run stops at the next tick boundary, and later failures
    /// are consequences of a scenario that is already wrong.
    unreadable_request: Arc<Mutex<Option<CoreError>>>,
    /// Every car's pose, watched for contact with another car's.
    collision_monitor: CollisionMonitor,
    /// What the run prints to the console, queued until the handler prints
    /// it between ticks.
    console_output: Arc<Mutex<ConsoleOutput>>,
}

impl Default for TrafficHandler {
    fn default() -> Self {
        TrafficHandler {
            pending_requests: Arc::default(),
            unreadable_request: Arc::default(),
            collision_monitor: CollisionMonitor::new(WORLD_NAME, EGO_NAME),
            console_output: Arc::default(),
        }
    }
}

/// Lines waiting to be printed to the console, and what the handler keeps
/// to write them.
#[derive(Default)]
struct ConsoleOutput {
    /// Actors that have joined but not yet published a pose.
    joining: BTreeSet<String>,
    /// The latest pose of each actor still in the world.
    latest_poses: BTreeMap<String, (SimTime, Pose)>,
    /// Lines not yet printed, in the order they happened.
    unprinted: Vec<ConsoleLine>,
}

/// One line of the run's console output.
enum ConsoleLine {
    /// An actor joining or leaving.
    Actor(ActorChange),
    /// An actor's first pose arriving before any join was seen for it, a
    /// warning, since every actor should join before it publishes.
    PoseBeforeJoin(ActorChange),
    /// Two traffic cars colliding.
    Collision(RecordedCollision),
}

/// What a line about one actor says, and the pose it prints with the
/// sim time that pose was published at.
struct ActorChange {
    description: &'static str,
    actor: String,
    sim_time: SimTime,
    pose: Pose,
}

/// One membership change the sim asked for.
enum TrafficRequest {
    Spawn(SpawnTrafficRequest),
    Despawn(DespawnTrafficRequest),
}

impl TrafficHandler {
    /// A `MonitorTransport` callback feeding every message to the traffic
    /// requests, the collision monitor, and the console output.
    pub fn message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let mut traffic_requests_on_message = self.traffic_request_message_callback();
        let mut collision_monitor_on_message = self.collision_monitor.message_callback();
        let mut console_output_on_message = self.console_output_message_callback();

        // Return one callback feeding the traffic requests, the collision
        // monitor, and the console output.
        move |message: &Message| {
            traffic_requests_on_message(message);
            collision_monitor_on_message(message);
            console_output_on_message(message);
        }
    }

    /// A membership callback feeding every join and leave to the collision
    /// monitor and the console output.
    pub fn membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let mut collision_monitor_on_membership = self.collision_monitor.membership_callback();
        let mut console_output_on_membership = self.console_output_membership_callback();

        // Return one callback feeding both.
        move |change: &MembershipChange| {
            collision_monitor_on_membership(change);
            console_output_on_membership(change);
        }
    }

    /// The collision monitor.
    pub fn collision_monitor(&self) -> &CollisionMonitor {
        &self.collision_monitor
    }

    /// Handles the step just taken: its collisions, its console output and
    /// its traffic requests. Fails on a collision involving the ego.
    fn process_step<T: Transport>(
        &self,
        conductor: &mut Conductor<T>,
    ) -> Result<(), ScenarioError> {
        // Before printing, so the output includes the collisions.
        let collision_check = self.process_collisions(conductor);
        // Before halting, so the output shows what led up to it.
        self.print_console_output();
        match collision_check {
            Ok(()) => {}
            Err(halt) => return Err(halt),
        }

        // Return the result of applying the traffic requests. They go last,
        // since a car they remove leaves the collision monitor at once and
        // its collision would go unchecked.
        self.apply_traffic_requests(conductor)
    }

    /// The part of [`Self::message_callback`] that notes each actor's
    /// latest pose, queueing a console output line with the first one.
    fn console_output_message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let console_output = self.console_output.clone();
        let pose_key_pattern =
            KeyExpr::new_rooted(format!("{WORLD_NAME}/actor/*/pose")).expect("valid pose key");

        // Return the noting callback, holding its own handles.
        move |message: &Message| {
            if !pose_key_pattern.matches(&message.key) {
                return;
            }
            // A pose that cannot be read is the collision monitor's to report.
            let Ok(pose) = message.decode::<Pose>() else {
                return;
            };
            let actor = actor_of_pose_key(message.key.as_str()).to_string();
            let mut console_output = console_output
                .lock()
                .expect("console output mutex is never poisoned");
            // An actor's first pose is printed with its join, or as a
            // warning if no join was seen for it, so a missed join still
            // shows.
            let first_pose = |description| ActorChange {
                description,
                actor: actor.clone(),
                sim_time: message.sim_time,
                pose,
            };
            if console_output.joining.remove(&actor) {
                let line = ConsoleLine::Actor(first_pose("joined"));
                console_output.unprinted.push(line);
            } else if !console_output.latest_poses.contains_key(&actor) {
                // This is the first pose for an actor that's not in `joining`.
                let line = ConsoleLine::PoseBeforeJoin(first_pose("pose before join"));
                console_output.unprinted.push(line);
            }
            console_output
                .latest_poses
                .insert(actor, (message.sim_time, pose));
        }
    }

    /// The part of [`Self::membership_callback`] that notes an actor
    /// joining, and queues a console output line when it leaves, with the
    /// last pose it published.
    fn console_output_membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let console_output = self.console_output.clone();

        // Return the noting callback, holding its own handle.
        move |change: &MembershipChange| {
            let (joined, path) = match change {
                MembershipChange::Joined(join) => (true, &join.path),
                MembershipChange::Left(leave) => (false, &leave.path),
            };
            let path = ComponentPath::parse(path).expect("a membership change names a valid path");
            let Some(actor) = path.actor() else {
                return;
            };
            let actor = actor.as_str();
            let mut console_output = console_output
                .lock()
                .expect("console output mutex is never poisoned");
            // An actor's components join and leave one after another, so
            // only the first of them changes anything here, and each actor is
            // printed joining and leaving once.
            if joined {
                if !console_output.latest_poses.contains_key(actor) {
                    console_output.joining.insert(actor.to_string());
                }
            } else {
                console_output.joining.remove(actor);
                if let Some((sim_time, pose)) = console_output.latest_poses.remove(actor) {
                    console_output
                        .unprinted
                        .push(ConsoleLine::Actor(ActorChange {
                            description: "left",
                            actor: actor.to_string(),
                            sim_time,
                            pose,
                        }));
                }
            }
        }
    }

    /// The part of [`Self::message_callback`] that collects the spawner's
    /// requests.
    fn traffic_request_message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let pending_requests = self.pending_requests.clone();
        let unreadable_request = self.unreadable_request.clone();
        let spawn = traffic_spawn_key(WORLD_NAME);
        let despawn = traffic_despawn_key(WORLD_NAME);

        // Return the collecting callback, holding its own handles.
        move |message: &Message| {
            // Only these two keys are requests. Anything else on the transport
            // is someone else's traffic and is not this handler's to read.
            let request = if message.key == spawn {
                message
                    .decode::<SpawnTrafficRequest>()
                    .map(TrafficRequest::Spawn)
            } else if message.key == despawn {
                message
                    .decode::<DespawnTrafficRequest>()
                    .map(TrafficRequest::Despawn)
            } else {
                return;
            };

            match request {
                Ok(request) => pending_requests
                    .lock()
                    .expect("request mutex is never poisoned")
                    .push(request),
                // A request on one of this handler's own keys that it cannot
                // read is a car that never arrives or never leaves, changing
                // the scenario in silence. `apply_traffic_requests` reports it
                // at the next tick boundary, which is the first place with
                // somewhere to return an error to.
                Err(error) => {
                    let mut slot = unreadable_request
                        .lock()
                        .expect("request mutex is never poisoned");
                    if slot.is_none() {
                        *slot = Some(error);
                    }
                }
            }
        }
    }

    /// Prints the console output queued since the last call, in the order
    /// it happened.
    fn print_console_output(&self) {
        let unprinted = std::mem::take(
            &mut self
                .console_output
                .lock()
                .expect("console output mutex is never poisoned")
                .unprinted,
        );
        let actor_text = |change: &ActorChange| {
            format!(
                "{}: actor={:?} (x={:.2} y={:.2} yaw_deg={:.1} at {} sec)",
                change.description,
                change.actor,
                change.pose.position.x,
                change.pose.position.y,
                change.pose.orientation.yaw().to_degrees(),
                change.sim_time,
            )
        };
        for line in unprinted {
            match line {
                ConsoleLine::Actor(change) => tracing::info!("{}", actor_text(&change)),
                ConsoleLine::PoseBeforeJoin(change) => tracing::warn!("{}", actor_text(&change)),
                ConsoleLine::Collision(collision) => tracing::warn!(
                    "collision: first={:?} second={:?} (at {} sec)",
                    collision.first,
                    collision.second,
                    collision.sim_time
                ),
            }
        }
    }

    /// Records each collision that began since the last call in the event
    /// log and queues it for the console. A collision involving the ego
    /// halts the run.
    fn process_collisions<T: Transport>(
        &self,
        conductor: &mut Conductor<T>,
    ) -> Result<(), ScenarioError> {
        let collisions = match self.collision_monitor.check_collisions() {
            Ok(collisions) => collisions,
            // The ego's collision halts the run, recorded first so the event
            // log says why.
            Err(ScenarioError::EgoCollision(collision)) => {
                conductor.report_observation(RecordedObservation::Collision(collision.clone()));
                return Err(ScenarioError::EgoCollision(collision));
            }
            Err(error) => return Err(error),
        };
        let mut console_output = self
            .console_output
            .lock()
            .expect("console output mutex is never poisoned");
        for collision in collisions {
            conductor.report_observation(RecordedObservation::Collision(collision.clone()));
            console_output
                .unprinted
                .push(ConsoleLine::Collision(collision));
        }

        // Return success; no collision involved the ego.
        Ok(())
    }

    /// Applies everything the spawner has asked for since the last call,
    /// in the order it asked.
    ///
    /// Private on purpose: a loop that stepped the conductor without
    /// calling this would run a different world from the recorded one, so
    /// the only way to drive a traffic scenario is
    /// [`run_live_traffic_scenario`] or [`run_playback_traffic_scenario`].
    fn apply_traffic_requests<T: Transport>(
        &self,
        conductor: &mut Conductor<T>,
    ) -> Result<(), ScenarioError> {
        // Before anything is applied, since a request that could not be read
        // means the collected ones are an incomplete account of what the sim
        // asked for.
        if let Some(error) = self
            .unreadable_request
            .lock()
            .expect("request mutex is never poisoned")
            .take()
        {
            return Err(ConductorError::Core(error).into());
        }

        let collected = std::mem::take(
            &mut *self
                .pending_requests
                .lock()
                .expect("request mutex is never poisoned"),
        );
        for request in collected {
            match request {
                TrafficRequest::Spawn(spawn) => add_car(
                    conductor,
                    &spawn.actor_name,
                    CarConfiguration::IdmFollowing,
                    spawn.start_s,
                    spawn.lane_offset,
                    spawn.speed,
                    spawn.first_due,
                )?,
                // The spawner drops a car from its roll when it asks, so it
                // never asks twice, but a scenario that also removed cars
                // by hand could, and a request for someone already gone has
                // simply been satisfied early.
                TrafficRequest::Despawn(despawn) => {
                    match conductor.remove_component_now(&despawn.actor_name) {
                        Ok(()) | Err(ConductorError::UnknownPath(_)) => {}
                        Err(other) => return Err(other.into()),
                    }
                }
            }
        }

        // Return once the queue is empty.
        Ok(())
    }
}

/// Runs the world to `end`, applying the spawner's requests as they come
/// and stopping at the first collision involving the ego.
///
/// The sim decides *what* joins and leaves; this turns those decisions into
/// components, and decides nothing itself. The split is forced, since a
/// component cannot hand over a `Box<dyn Component>`, which is also why
/// join-over-transport waits for milestone 7. It is also what keeps
/// the traffic pattern reproducible: the spawner chose it from poses and a
/// seeded stream, so a recorded run can be verified against a re-run.
///
/// Timing here is not load-bearing. Every request declares the instant it
/// takes effect, so *when* this loop applies one does not shape the run,
/// only that it lands before that instant, which draining after every tick
/// leaves a whole spawner period of room for.
///
/// Pass `Some` verifier to stop at the first divergence rather than at
/// `end`. Verification runs the world exactly as any other run does, which
/// is the whole point of it, so it drives the same loop instead of a copy
/// that could drift from this one.
// TODO(PLAN "Scenario configuration"): the general form of this loop is a
// host-side registry mapping component *type names* to constructors, so one
// driver serves any scenario instead of this one knowing about cars. At
// milestone 7 a host plays exactly this part for remote components.
pub fn run_live_traffic_scenario<T: Transport>(
    conductor: &mut Conductor<T>,
    traffic_handler: &TrafficHandler,
    end: SimTime,
    verifier: Option<&Verifier>,
) -> Result<(), ScenarioError> {
    // Once diverged, every later event is compared against a log the run
    // has already left behind, so the first difference is the only one
    // worth reporting.
    while verifier.is_none_or(|verifier| !verifier.diverged())
        && conductor
            .next_due_instant()
            .is_some_and(|instant| instant <= end)
    {
        conductor.step_once()?;
        traffic_handler.process_step(conductor)?;
    }

    // Return once nothing remains scheduled at or before `end`.
    Ok(())
}

/// Runs a playback world to `end`, stopping at the first collision
/// involving the ego. The playback counterpart to
/// [`run_live_traffic_scenario`].
///
/// The recorded cars cannot react to the ego, so a what-if varies the ego
/// alone and watches for its collisions. It takes no verifier, since a
/// what-if is meant to differ from the recording.
pub fn run_playback_traffic_scenario<T: Transport>(
    conductor: &mut Conductor<T>,
    traffic_handler: &TrafficHandler,
    end: SimTime,
) -> Result<(), ScenarioError> {
    while conductor
        .next_due_instant()
        .is_some_and(|instant| instant <= end)
    {
        conductor.step_once()?;
        traffic_handler.process_step(conductor)?;
    }

    // Return once the recorded schedule has been played out to `end`.
    Ok(())
}

#[cfg(test)]
mod tests {
    use continuo_conductor::{RecordedJoin, RecordedLeave};
    use continuo_core::{Quat, Vec3};

    use super::*;

    /// A pose for `actor` at `x` along the road, published at `sim_secs`.
    fn pose_message(actor: &str, x: f64, sim_secs: f64) -> Message {
        Message {
            key: pose_key(WORLD_NAME, actor),
            publisher: ComponentPath::parse(&format!("{actor}/physics")).expect("valid path"),
            seq: 0,
            sim_time: SimTime::from_secs_f64(sim_secs),
            payload: serde_json::to_vec(&Pose {
                position: Vec3::new(x, 0.0, 0.0),
                orientation: Quat::IDENTITY,
            })
            .expect("a pose serializes"),
        }
    }

    /// The lines queued for the console, each as its description, its
    /// position along the road, and its pose's sim time.
    fn queued_lines(traffic_handler: &TrafficHandler) -> Vec<(&'static str, f64, f64)> {
        let console_output = traffic_handler.console_output.lock().expect("not poisoned");

        // Return the actor lines; nothing here collides.
        console_output
            .unprinted
            .iter()
            .map(|line| match line {
                ConsoleLine::Actor(change) | ConsoleLine::PoseBeforeJoin(change) => (
                    change.description,
                    change.pose.position.x,
                    change.sim_time.as_secs_f64(),
                ),
                ConsoleLine::Collision(_) => panic!("no car collided"),
            })
            .collect()
    }

    #[test]
    fn a_car_is_printed_joining_with_its_first_pose_and_leaving_once_with_its_last() {
        let traffic_handler = TrafficHandler::default();
        let mut on_message = traffic_handler.message_callback();
        let mut on_membership = traffic_handler.membership_callback();
        // A car's components join and leave one after another.
        let leaves = ["radar", "controller", "physics"];
        for leaf in leaves {
            on_membership(&MembershipChange::Joined(RecordedJoin {
                path: format!("car/{leaf}"),
                first_due: SimTime::from_secs_f64(1.0),
            }));
        }
        on_message(&pose_message("car", 10.0, 1.0));
        on_message(&pose_message("car", 12.0, 1.1));
        for leaf in leaves {
            on_membership(&MembershipChange::Left(RecordedLeave {
                path: format!("car/{leaf}"),
                leaves_at: SimTime::from_secs_f64(1.2),
            }));
        }

        // The leaving line prints its last pose, published at 1.1 s.
        assert_eq!(
            queued_lines(&traffic_handler),
            [("joined", 10.0, 1.0), ("left", 12.0, 1.1)]
        );
    }

    #[test]
    fn a_pose_before_any_join_prints_one_warning() {
        let traffic_handler = TrafficHandler::default();
        let mut on_message = traffic_handler.message_callback();
        on_message(&pose_message("stranger", 5.0, 1.0));
        on_message(&pose_message("stranger", 6.0, 1.1));

        assert_eq!(
            queued_lines(&traffic_handler),
            [("pose before join", 5.0, 1.0)]
        );
    }
}
