//! Collisions in a running scenario: a traffic collision is recorded and the
//! run goes on, the ego's halts it, and a car that has left cannot be hit.
//!
//! Each car is a bare plant. With no commands it holds its speed and
//! heading, so the cars drive straight through each other and the
//! collision falls where the arithmetic puts it. The monitor halts for the
//! actor named `ego`, as the demo names its ego.

use continuo_actors::{CarState, PlantLimits, UnicyclePhysics};
use continuo_conductor::record::LogEvent;
use continuo_conductor::{
    Conductor, EventLog, JoinMetadata, RecordedCollision, RecordedObservation, Recorder,
};
use continuo_core::{Pose, Quat, SimDuration, SimTime, Vec3};
use continuo_examples::traffic_world::{
    self, ScenarioError, TrafficRequestHandler, WorldMonitor, run_live_traffic_scenario,
};
use continuo_transport::{InProcTransport, MonitorTransport};

/// One car heading east: its name, the `x` and `y` it starts at, and its
/// speed.
type TestCar<'a> = (&'a str, f64, f64, f64);

/// A world holding only `cars`, watched by `world_monitor` the way the
/// examples watch the demo. With no spawner, there are no traffic requests
/// to collect.
fn watched_world(
    world_monitor: &WorldMonitor,
    cars: &[TestCar],
) -> Conductor<MonitorTransport<InProcTransport>> {
    let mut transport = MonitorTransport::new(InProcTransport::new());
    transport.add_message_callback(world_monitor.message_callback());
    let mut conductor =
        Conductor::new(traffic_world::config(), transport).expect("config is accepted");
    conductor.add_membership_callback(world_monitor.membership_callback());
    for &(name, x, y, speed) in cars {
        let pose = Pose {
            position: Vec3::new(x, y, 0.0),
            orientation: Quat::IDENTITY,
        };
        let physics = UnicyclePhysics::new(
            name,
            SimDuration::from_millis(10),
            PlantLimits::highway_car(),
            CarState::new(pose, speed),
        );
        conductor
            .add_component(JoinMetadata::at_start(name), Box::new(physics))
            .expect("the car joins");
    }

    // Return the world, ready to run.
    conductor
}

/// The collisions an event log recorded, in order.
fn recorded_collisions(log: &EventLog) -> Vec<RecordedCollision> {
    log.events
        .iter()
        .filter_map(|event| match event {
            LogEvent::Observed(RecordedObservation::Collision(collision)) => {
                Some(collision.clone())
            }
            _ => None,
        })
        .collect()
}

#[test]
fn a_traffic_collision_is_recorded_once_and_does_not_halt() {
    // The fast car reaches the slow one at about 4.5 s and is clear of it by
    // 5.5 s.
    let world_monitor = WorldMonitor::default();
    let mut conductor = watched_world(
        &world_monitor,
        &[("slow", 50.0, 3.5, 10.0), ("fast", 0.0, 3.5, 20.0)],
    );
    let recorder = Recorder::new(&traffic_world::config());
    conductor.add_observation_callback(recorder.observation_callback());
    run_live_traffic_scenario(
        &mut conductor,
        &TrafficRequestHandler::default(),
        &world_monitor,
        SimTime::from_secs(10),
        None,
    )
    .expect("a traffic collision is recorded, not a halt");

    let collisions = recorded_collisions(&recorder.finish());
    assert_eq!(collisions.len(), 1, "{collisions:?}");
    assert_eq!(
        (collisions[0].first.as_str(), collisions[0].second.as_str()),
        ("fast", "slow")
    );
}

#[test]
fn an_ego_collision_halts_the_scenario_naming_both_cars() {
    // Closing at 20 m/s from 50 m, the bumpers meet at 2.275 s.
    let world_monitor = WorldMonitor::default();
    let mut conductor = watched_world(
        &world_monitor,
        &[("ego", 0.0, 0.0, 30.0), ("slow", 50.0, 0.0, 10.0)],
    );
    let recorder = Recorder::new(&traffic_world::config());
    conductor.add_observation_callback(recorder.observation_callback());
    let error = run_live_traffic_scenario(
        &mut conductor,
        &TrafficRequestHandler::default(),
        &world_monitor,
        SimTime::from_secs(10),
        None,
    )
    .expect_err("the ego running into a car halts the run");

    let ScenarioError::EgoCollision(collision) = &error else {
        panic!("expected the ego's collision, got {error}");
    };
    assert_eq!(
        (collision.first.as_str(), collision.second.as_str()),
        ("ego", "slow")
    );
    assert!(
        (2.2..2.4).contains(&collision.sim_time.as_secs_f64()),
        "{error}"
    );
    let message = error.to_string();
    assert!(
        message.contains("ego") && message.contains("slow"),
        "{message}"
    );

    // Recorded before the halt, so the log says why the run stopped.
    assert_eq!(
        recorded_collisions(&recorder.finish()),
        vec![collision.clone()]
    );
}

#[test]
fn a_car_that_has_left_cannot_be_hit_where_it_stood() {
    // The ego reaches a car parked 60 m ahead before 2 s, so left in place
    // it is a collision.
    let ego_and_parked_car = [("ego", 0.0, 0.0, 30.0), ("parked", 60.0, 0.0, 0.0)];
    let end = SimTime::from_secs(5);

    let no_requests = TrafficRequestHandler::default();

    let monitor_keeping_parked_car = WorldMonitor::default();
    let mut conductor = watched_world(&monitor_keeping_parked_car, &ego_and_parked_car);
    assert!(matches!(
        run_live_traffic_scenario(
            &mut conductor,
            &no_requests,
            &monitor_keeping_parked_car,
            end,
            None
        ),
        Err(ScenarioError::EgoCollision(_))
    ));

    // Removed at 1 s, its last pose goes with it.
    let monitor_retiring_parked_car = WorldMonitor::default();
    let mut conductor = watched_world(&monitor_retiring_parked_car, &ego_and_parked_car);
    run_live_traffic_scenario(
        &mut conductor,
        &no_requests,
        &monitor_retiring_parked_car,
        SimTime::from_secs(1),
        None,
    )
    .expect("nothing has met yet");
    conductor
        .remove_component_now("parked")
        .expect("the parked car is there to remove");
    run_live_traffic_scenario(
        &mut conductor,
        &no_requests,
        &monitor_retiring_parked_car,
        end,
        None,
    )
    .expect("a car that has left cannot be hit");
    assert_eq!(
        monitor_retiring_parked_car
            .collision_monitor()
            .num_collisions(),
        0
    );
}
