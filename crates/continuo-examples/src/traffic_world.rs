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

use continuo_actors::{
    CarState, DespawnTrafficRequest, DriveLimits, PathFollowController, PoseLogger,
    SpawnTrafficRequest, TrafficSpawner, UnicyclePhysics, Waypoints, road_pose, straight_road,
    traffic_despawn_key, traffic_spawn_key,
};
use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, ConductorConfig, ConductorError, EventLog, JoinMetadata, Pacing, PlaybackComponent,
    Verifier, WORLD_LEVEL,
};
use continuo_core::{Component, ComponentId, CoreError, Message, SimDuration, SimTime};
use continuo_transport::{MonitorTransport, Transport};

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
const CAR_LIMITS: DriveLimits = DriveLimits::highway_car();

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

/// Registers one car as the composite `{actor_name} = [controller,
/// physics]`. Declared order matters: the controller is registered before
/// the physics, so its command reaches the physics same-instant when both
/// are due.
///
/// `speed` goes to the physics rather than the controller, because the
/// physics owns it. Nothing here commands an acceleration, so what the car
/// starts at is what it holds for the whole run.
///
/// Both halves read their turn rate out of [`CAR_LIMITS`], since a
/// normalized command means whatever the plant says it means and a
/// controller working from a different number would steer to the wrong
/// rate.
fn add_car<T: Transport>(
    conductor: &mut Conductor<T>,
    actor_name: &str,
    start_s: f64,
    lane_offset: f64,
    speed: f64,
    first_due: SimTime,
) -> Result<(), ConductorError> {
    // A lane is the lateral offset the controller holds, not geometry of
    // its own, so both components of every car work off the one road.
    let road = shared_road();
    let initial_pose = road_pose(&road, start_s, lane_offset);
    let components: [Box<dyn Component>; 2] = [
        Box::new(PathFollowController::new(
            actor_name,
            road.clone(),
            lane_offset,
            SimDuration::from_millis(100),
            6.0, // lookahead, m
            1.5, // heading gain, 1/s
            CAR_LIMITS.yaw_rate_max,
            initial_pose,
        )),
        Box::new(UnicyclePhysics::new(
            actor_name,
            SimDuration::from_millis(10),
            CAR_LIMITS,
            CarState::new(initial_pose, speed),
        )),
    ];
    for component in components {
        conductor.add_component(JoinMetadata::at(actor_name, first_due), component)?;
    }

    // Return success; the car is registered.
    Ok(())
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
) -> Result<(), ConductorError> {
    // Return once the ego is registered.
    add_car(
        conductor,
        EGO_NAME,
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
    conductor.add_component(
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
) -> Result<(), ConductorError> {
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
) -> Result<(), ConductorError> {
    assert!(lanes > 0, "a world needs at least one lane");

    for index in 0..cars {
        let lane_offset = ((index % lanes) as f64 - (lanes - 1) as f64 * 0.5) * LANE_WIDTH;
        let start_s = (index / lanes) as f64 * SCALE_ROW_SPACING;
        let speed = TRAFFIC_SPEED.0 + (index % 7) as f64;
        add_car(
            conductor,
            &format!("car{index}"),
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
) -> Result<usize, ConductorError> {
    add_ego(conductor, EGO_START, EGO_LANE, ego_speed)?;

    // Every playback double is registered now, even though most of their
    // originals joined mid-run: a double with nothing recorded yet
    // publishes nothing and reports its first recorded message as its next
    // due time, so it idles until the instant it first appeared.
    let actor_names = recorded_traffic_actor_names(recorded);
    for actor_name in &actor_names {
        let id = ComponentId::new(actor_name.as_str()).expect("a recorded path segment is an id");
        conductor.add_component(
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
    conductor.add_component(
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
    fn apply<T: Transport>(&self, conductor: &mut Conductor<T>) -> Result<(), ConductorError> {
        // Before anything is applied, since a request that could not be read
        // means the collected ones are an incomplete account of what the sim
        // asked for.
        if let Some(error) = self
            .unreadable_request
            .lock()
            .expect("request mutex is never poisoned")
            .take()
        {
            return Err(ConductorError::Core(error));
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
                    match conductor.remove_component(&despawn.actor_name) {
                        Ok(()) | Err(ConductorError::UnknownPath(_)) => {}
                        Err(other) => return Err(other),
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
) -> Result<(), ConductorError> {
    // Once diverged, every later event is compared against a log the run
    // has already left behind, so the first difference is the only one
    // worth reporting.
    while verifier.is_none_or(|verifier| !verifier.diverged())
        && conductor
            .next_scheduled()
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
