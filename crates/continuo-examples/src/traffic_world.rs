//! The demo world shared by every traffic example: an ego car driving a
//! straight highway, with traffic spawning ahead of it in the neighbouring
//! lanes and retiring once it has gone past.
//!
//! Two scenarios, kept side by side so the difference between them is one
//! diff rather than two files: [`setup_live_traffic_scenario`], whose cars
//! a spawner decides on while the run happens, and
//! [`setup_playback_traffic_scenario`], whose cars are doubles of a
//! recorded run.
//!
//! Each comes in two halves, a `setup_` and a `run_`, and both halves are
//! shared so the examples cannot drift apart.
//! [`run_live_traffic_scenario`] applies the spawner's requests between
//! ticks; sharing that matters as much as sharing the setup, because an
//! example that stepped the conductor without applying them would be
//! running a different world, and `traffic_verify` exists to notice exactly
//! that kind of difference. [`run_playback_traffic_scenario`] has nothing
//! to apply, since recorded cars are already a schedule, and the pair
//! sitting together is what makes that visible.
//!
//! Everything is generic over the transport, so an example picks its own:
//! plain `InProcTransport` for the base demo, wrapped in a
//! `MonitorTransport` when recording or verifying.

use std::sync::{Arc, Mutex, OnceLock};

use continuo_actors::control_laws::{FREE_ROAD, IdmParams, PurePursuitParams};
use continuo_actors::{
    CarState, DespawnTrafficRequest, PathFollowController, PlantLimits, PoseLogger, RadarSensor,
    SpawnTrafficRequest, TrafficSpawner, UnicyclePhysics, Waypoints, accel_cmd_key, pose_key,
    radar_key, road_pose, steer_cmd_key, straight_road, traffic_despawn_key, traffic_spawn_key,
};
use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, ConductorError, EventLog, JoinMetadata, Pacing, PlaybackComponent,
    Verifier, WORLD_LEVEL,
};
use continuo_core::{Component, ComponentId, CoreError, Message, SimDuration, SimTime};
use continuo_fmi::{FmuComponent, FmuConstructionError, FmuMapping, InputBinding, OutputBinding};
use continuo_fmu_controller_idm::{MAX_WAYPOINTS, PackageError, packaged_fmu_path};
use continuo_transport::{MonitorTransport, Transport};
use serde_json::json;

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

/// How far ahead a following car's radar sees, in meters. Well past the
/// gap IDM keeps at highway speed, so a car brakes for a slower one long
/// before it would have to brake hard.
const RADAR_MAX_RANGE: f64 = 120.0;

/// Half a lane either side, so a car counts as sharing the radar's lane
/// when it is nearer that lane's center than any other.
const RADAR_LANE_TOLERANCE: f64 = LANE_WIDTH / 2.0;

/// How a car decides its speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Guidance {
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
    NotPackaged(#[from] PackageError),

    #[error("cannot build the controller FMU for {actor_name}: {source}")]
    Controller {
        actor_name: String,
        #[source]
        source: FmuConstructionError,
    },
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

/// Registers one car as a composite under `actor_name`, guided as
/// `guidance` says, starting `start_s` along the road in the lane at
/// `lane_offset` and doing `speed`.
///
/// Declared order matters. A following car registers its radar, then its
/// controller, then its physics, so at every control instant the scan
/// reaches the controller and the command reaches the physics within the
/// same instant. A constant-speed car has no radar and the same order
/// otherwise.
///
/// `speed` goes to the physics, which owns it. A constant-speed car
/// commands no acceleration, so it holds that speed for the whole run; a
/// following car wants it back whenever the road ahead is free.
///
/// Every part takes [`CAR_LIMITS`], since a normalized command means
/// whatever the plant says it means and a controller working from a
/// different set would command the wrong rate.
fn add_car<T: Transport>(
    conductor: &mut Conductor<T>,
    actor_name: &str,
    guidance: Guidance,
    start_s: f64,
    lane_offset: f64,
    speed: f64,
    first_due: SimTime,
) -> Result<(), ScenarioError> {
    // A lane is the lateral offset the controller holds, not geometry of
    // its own, so every component of every car works off the one road.
    let road = shared_road();
    let initial = CarState::new(road_pose(&road, start_s, lane_offset), speed);
    let physics = Box::new(UnicyclePhysics::new(
        actor_name,
        PHYSICS_PERIOD,
        CAR_LIMITS,
        initial,
    ));
    let components: Vec<Box<dyn Component>> = match guidance {
        Guidance::ConstantSpeed => vec![
            Box::new(PathFollowController::new(
                actor_name,
                road,
                lane_offset,
                CONTROL_PERIOD,
                6.0,                     // lookahead, m
                1.5,                     // heading gain, 1/s
                CAR_LIMITS.yaw_rate_max, // command turns up to the car's limit
                CAR_LIMITS,
                initial.pose(),
            )),
            physics,
        ],
        Guidance::IdmFollowing => {
            let mapping =
                controller_mapping(conductor.world_name(), actor_name, lane_offset, initial);
            let controller = FmuComponent::new("controller", packaged_fmu_path()?, mapping)
                .map_err(|source| ScenarioError::Controller {
                    actor_name: actor_name.to_string(),
                    source,
                })?;
            vec![
                Box::new(RadarSensor::new(
                    actor_name,
                    road,
                    CONTROL_PERIOD,
                    RADAR_MAX_RANGE,
                    RADAR_LANE_TOLERANCE,
                )),
                Box::new(controller),
                physics,
            ]
        }
    };
    for component in components {
        conductor.add_component(JoinMetadata::at(actor_name, first_due), component)?;
    }

    // Return success; the car is registered.
    Ok(())
}

/// The sheet wiring one following car's controller FMU into the world:
/// which messages feed its inputs, where its commands go, and the numbers
/// it is built with.
///
/// The keys are concrete, which is why `world_name` is an argument. A
/// native component cannot do that, since its constructor does not know
/// the world and so it wildcards the world in its subscriptions; a
/// scenario does know it.
///
/// Four kinds of thing, kept apart below:
///
/// - The pose inputs read the car's own pose and name no pointers, since
///   `position.x` spells `/position/x` on its own.
/// - The radar inputs read the car's own scan through one pattern each.
///   The scan carries only the cars detected, so every slot past them
///   reads as [`FREE_ROAD`].
/// - The parameters feed no message and are set once: the road, as the
///   points [`shared_road`] was built from, so the FMU and the radar
///   measure one geometry; the steering and following tunings; and the
///   plant's limits, which the FMU divides by.
/// - The outputs publish on the plant's two command keys, each payload
///   field named after the output variable, which is what the plant
///   decodes.
///
/// `initial` also gives the pose inputs their starting values. At the
/// instant a car joins, its controller steps before its physics has
/// published anything, so without them it would steer from the origin and
/// accelerate from a standstill it is not in. It is the state the physics
/// is built from, so the two halves of a car cannot be handed different
/// ones.
fn controller_mapping(
    world_name: &str,
    actor_name: &str,
    lane_offset: f64,
    initial: CarState,
) -> FmuMapping {
    let road = shared_road();
    let mut road_x = vec![0.0; MAX_WAYPOINTS];
    let mut road_y = vec![0.0; MAX_WAYPOINTS];
    for (slot, &(x, y)) in road.points().iter().enumerate() {
        road_x[slot] = x;
        road_y[slot] = y;
    }
    let pursuit = PurePursuitParams::highway_car(lane_offset);
    let idm = IdmParams::highway_car(initial.speed);

    let pose = pose_key(world_name, actor_name);
    let radar = radar_key(world_name, actor_name);
    let inputs = vec![
        InputBinding::new("position.x", pose.clone()),
        InputBinding::new("position.y", pose.clone()),
        InputBinding::new("orientation.w", pose.clone()),
        InputBinding::new("orientation.x", pose.clone()),
        InputBinding::new("orientation.y", pose.clone()),
        InputBinding::new("orientation.z", pose.clone()),
        InputBinding::new("speed", pose),
        InputBinding::new("range", radar.clone())
            .with_pointer("/detections/*/range")
            .when_missing(json!(FREE_ROAD.range)),
        InputBinding::new("range_rate", radar)
            .with_pointer("/detections/*/range_rate")
            .when_missing(json!(FREE_ROAD.range_rate)),
    ];
    let outputs = vec![
        OutputBinding::new("accel_cmd", accel_cmd_key(world_name, actor_name)),
        OutputBinding::new("steer_cmd", steer_cmd_key(world_name, actor_name)),
    ];
    let parameters = [
        ("road_x", json!(road_x)),
        ("road_y", json!(road_y)),
        ("road_point_count", json!(road.points().len())),
        ("road_closed", json!(road.is_closed())),
        ("lateral_tgt", json!(pursuit.lateral_tgt)),
        ("lookahead", json!(pursuit.lookahead)),
        ("gain_yaw_rate", json!(pursuit.gain_yaw_rate)),
        ("max_yaw_rate", json!(pursuit.max_yaw_rate)),
        ("v0_speed_tgt", json!(idm.v0_speed_tgt)),
        ("t_headway", json!(idm.t_headway)),
        ("s0_gap_min", json!(idm.s0_gap_min)),
        ("a_accel_max", json!(idm.a_accel_max)),
        ("b_decel_comfort", json!(idm.b_decel_comfort)),
        ("plant_accel_max", json!(CAR_LIMITS.accel_max)),
        ("plant_decel_max", json!(CAR_LIMITS.decel_max)),
        ("plant_yaw_rate_max", json!(CAR_LIMITS.yaw_rate_max)),
    ];
    let starting_inputs = [
        ("position.x", json!(initial.position.x)),
        ("position.y", json!(initial.position.y)),
        ("orientation.w", json!(initial.orientation.w)),
        ("orientation.x", json!(initial.orientation.x)),
        ("orientation.y", json!(initial.orientation.y)),
        ("orientation.z", json!(initial.orientation.z)),
        ("speed", json!(initial.speed)),
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
        Guidance::ConstantSpeed,
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

/// Registers the live scenario's fixed cast: the ego, the spawner that
/// manages traffic around it, and the pose logger. Every car arrives later,
/// while the run is under way, so this is the whole world only at sim time
/// zero, and [`run_live_traffic_scenario`] is what builds the rest of it.
///
/// *Live* because the traffic is decided as the run happens. `traffic_resim`
/// assembles the other kind by hand: the same ego and logger, but no
/// spawner, and cars played back from a log rather than chosen.
pub fn setup_live_traffic_scenario<T: Transport>(
    conductor: &mut Conductor<T>,
) -> Result<(), ScenarioError> {
    add_ego(conductor, EGO_START, EGO_LANE, EGO_SPEED)?;
    add_spawner(conductor)?;
    add_logger(conductor)?;

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
/// No spawner and no logger, and neither is an oversight. A spawner would
/// make the population a moving target, which is the one thing a
/// measurement of population wants held still, and a pose logger at this
/// size writes more than the run it is reporting on.
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
            Guidance::ConstantSpeed,
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

    // Every playback double is registered now, even though most of their
    // originals joined mid-run: a double with nothing recorded yet
    // publishes nothing and reports its first recorded message as its next
    // due time, so it idles until the instant it first appeared.
    let actor_names = recorded_traffic_actor_names(recorded);
    for actor_name in &actor_names {
        let id = ComponentId::new(actor_name.as_str()).expect("a recorded path segment is an id");
        conductor.add_component_at_start(
            WORLD_LEVEL,
            Box::new(PlaybackComponent::from_log(id, recorded, actor_name)),
        )?;
    }
    add_logger(conductor)?;

    // Return the size of the scene that was rebuilt.
    Ok(actor_names.len())
}

/// The traffic actors a recorded run contained, in the order they joined.
///
/// Cars are composites, so their join paths have a parent
/// (`traffic7/physics`); the spawner and the logger are world-level leaves
/// and have none, which separates them here without matching on names.
fn recorded_traffic_actor_names(recorded: &EventLog) -> Vec<String> {
    let mut actor_names: Vec<String> = Vec::new();
    for event in &recorded.events {
        let LogEvent::Join(join) = event else {
            continue;
        };
        let Some((actor_name, _)) = join.path.split_once('/') else {
            continue;
        };
        if actor_name != EGO_NAME && !actor_names.iter().any(|seen| seen == actor_name) {
            actor_names.push(actor_name.to_string());
        }
    }

    // Return each car once, first join first.
    actor_names
}

/// Registers the world-level pose logger, offset 1 ns past each second
/// boundary: the smallest offset that clears same-instant deferral, so
/// on-boundary poses are visible, and nothing can be scheduled between a
/// boundary and its sample.
fn add_logger<T: Transport>(conductor: &mut Conductor<T>) -> Result<(), ConductorError> {
    conductor.add_component_at_start(
        WORLD_LEVEL,
        Box::new(PoseLogger::new(
            SimDuration::from_secs(1),
            SimDuration::from_nanos(1),
        )),
    )?;

    // Return success; the logger is registered.
    Ok(())
}

/// Carries the spawner's decisions out to the conductor: collects the
/// requests it publishes, then turns them into membership changes.
///
/// Both halves are here because they are one job split by a boundary the
/// sim cannot cross: a component can decide a car should exist but cannot
/// build one. [`Self::wrap_transport`] wires up the collecting, and
/// [`run_live_traffic_scenario`] does the building between ticks.
#[derive(Clone, Default)]
pub struct TrafficRequestHandler {
    /// Requests collected since the last [`Self::apply`], in publish order.
    pending_requests: Arc<Mutex<Vec<Request>>>,
    /// The first request that could not be read, held until [`Self::apply`]
    /// reports it.
    ///
    /// The collecting callback is an `FnMut(&Message)` with nowhere to return
    /// an error to, so a request it cannot decode is put here instead. First
    /// one wins: the run stops at the next tick boundary, and later failures
    /// are consequences of a scenario that is already wrong.
    unreadable_request: Arc<Mutex<Option<CoreError>>>,
}

/// One membership change the sim asked for.
enum Request {
    Spawn(SpawnTrafficRequest),
    Despawn(DespawnTrafficRequest),
}

impl TrafficRequestHandler {
    /// A monitor callback that picks the spawner's requests out of the
    /// traffic. Attach it to a `MonitorTransport` wrapping the conductor's
    /// transport.
    pub fn callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let pending_requests = self.pending_requests.clone();
        let unreadable_request = self.unreadable_request.clone();
        let spawn = traffic_spawn_key(WORLD_NAME);
        let despawn = traffic_despawn_key(WORLD_NAME);

        // Return the collecting callback, holding its own handles.
        move |message: &Message| {
            // Only these two keys are requests. Anything else on the transport
            // is someone else's traffic and is not this handler's to read.
            let request = if message.key == spawn {
                message.decode::<SpawnTrafficRequest>().map(Request::Spawn)
            } else if message.key == despawn {
                message
                    .decode::<DespawnTrafficRequest>()
                    .map(Request::Despawn)
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
                // the scenario in silence. `apply` reports it at the next tick
                // boundary, which is the first place with somewhere to return
                // an error to.
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

    /// Wraps `inner` so the spawner's requests are collected as they are
    /// published, which is what [`run_live_traffic_scenario`] then acts on.
    ///
    /// Takes a transport rather than building one, so an example that also
    /// wants its own observer nests the two:
    ///
    /// ```ignore
    /// traffic_request_handler.wrap_transport(MonitorTransport::new(
    ///     InProcTransport::new(),
    ///     recorder.message_callback(),
    /// ))
    /// ```
    pub fn wrap_transport<T: Transport>(&self, inner: T) -> MonitorTransport<T> {
        // Return the transport with request collection layered on.
        MonitorTransport::new(inner, self.callback())
    }

    /// Applies everything the spawner has asked for since the last call,
    /// in the order it asked.
    ///
    /// Private on purpose: a loop that stepped the conductor without
    /// calling this would run a different world from the recorded one, so
    /// the only way to drive this scenario is
    /// [`run_live_traffic_scenario`], which cannot forget.
    fn apply<T: Transport>(&self, conductor: &mut Conductor<T>) -> Result<(), ScenarioError> {
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
                Request::Spawn(spawn) => add_car(
                    conductor,
                    &spawn.actor_name,
                    Guidance::IdmFollowing,
                    spawn.start_s,
                    spawn.lane_offset,
                    spawn.speed,
                    spawn.first_due,
                )?,
                // The spawner drops a car from its roll when it asks, so it
                // never asks twice, but a scenario that also removed cars
                // by hand could, and a request for someone already gone has
                // simply been satisfied early.
                Request::Despawn(despawn) => {
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

/// Runs the world to `end`, applying the spawner's requests as they come.
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
    traffic_request_handler: &TrafficRequestHandler,
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
        traffic_request_handler.apply(conductor)?;
    }

    // Return once nothing remains scheduled at or before `end`.
    Ok(())
}

/// Runs a playback world to `end`.
///
/// The counterpart to [`run_live_traffic_scenario`], and the contrast is
/// the whole content of it: there is nothing to apply between ticks,
/// because nothing in this world asks for anything. Recorded cars are
/// already a schedule, since each double knows every instant it publishes
/// at before the run starts, so the conductor only has to follow it.
///
/// Which is why this needs no request handler and no verifier: with no
/// decisions being made there is nothing that could be made differently,
/// and a divergence would have to come from the ego alone. Comparing that
/// against the log is the *opposite* of what a what-if wants.
pub fn run_playback_traffic_scenario<T: Transport>(
    conductor: &mut Conductor<T>,
    end: SimTime,
) -> Result<(), ConductorError> {
    // Return once the recorded schedule has been played out to `end`.
    conductor.run_until(end)
}

#[cfg(test)]
mod tests {
    use continuo_actors::{AccelCmd, SteerCmd};
    use continuo_core::{KeyExpr, StepCtx};

    use super::*;

    #[test]
    fn the_controller_fmu_publishes_what_the_plant_decodes() {
        // The FMU names its outputs and the plant names its payload fields,
        // and nothing compiles one against the other. A car whose commands
        // went to the wrong key would hold its speed in silence, so this
        // checks the meeting point directly: one step of a following car's
        // controller, read back the way the physics reads it.
        let initial = CarState::new(road_pose(&shared_road(), 100.0, LANE_WIDTH), 20.0);
        let mapping = controller_mapping(WORLD_NAME, "car", LANE_WIDTH, initial);
        let mut controller = FmuComponent::new(
            "controller",
            packaged_fmu_path().unwrap_or_else(|missing| panic!("{missing}")),
            mapping,
        )
        .expect("the controller FMU builds");

        let mut ctx = StepCtx::new(SimTime::ZERO, None, WORLD_NAME, 0, Vec::new());
        controller.step(&mut ctx).expect("the controller steps");
        let outbox = ctx.take_outbox();

        let published = |key: KeyExpr| {
            &outbox
                .iter()
                .find(|(published_key, _)| *published_key == key)
                .unwrap_or_else(|| panic!("nothing was published on {key}"))
                .1
        };
        serde_json::from_slice::<AccelCmd>(published(accel_cmd_key(WORLD_NAME, "car")))
            .expect("the plant decodes the acceleration command");
        serde_json::from_slice::<SteerCmd>(published(steer_cmd_key(WORLD_NAME, "car")))
            .expect("the plant decodes the steering command");
    }
}
