//! The demo world shared by every traffic example: three cars staggered
//! around an oval loop, each a composite `carN = [controller, physics]`,
//! plus a world-level pose logger.
//!
//! Builders are generic over the transport so each example can pick its
//! own (plain `InProcTransport` for the base demo, `MonitorTransport` when
//! recording or verifying).

use std::sync::Arc;

use continuo_actors::{PathFollowController, PoseLogger, UnicyclePhysics, Waypoints};
use continuo_conductor::{Conductor, ConductorConfig, ConductorError};
use continuo_core::{Pose, Quat, SimDuration};
use continuo_transport::Transport;

/// One seed for the whole demo family: record, verify, and resim must all
/// agree on it.
pub const WORLD_SEED: u64 = 42;
pub const WORLD_NAME: &str = "demo";
pub const SIM_SECONDS: i64 = 30;

/// The demo world's conductor configuration (free-run).
pub fn config() -> ConductorConfig {
    ConductorConfig {
        world: WORLD_NAME.into(),
        seed: WORLD_SEED,
        real_time_pacing: false,
    }
}

/// The oval loop all cars follow. 72 samples = one point per 5 degrees of
/// arc; on the 40 m semi-axis the worst-case chord deviation is ~4 cm,
/// smooth relative to the controller's 6 m lookahead.
pub fn demo_path() -> Arc<Waypoints> {
    Arc::new(Waypoints::ellipse((0.0, 0.0), 40.0, 25.0, 72))
}

/// Registers one live car (composite `controller → physics`) staggered to
/// position `i` of 3 around the loop. Declared order matters: the
/// controller is registered before the physics, so its command reaches the
/// physics same-instant when both are due.
pub fn add_live_car<T: Transport>(
    conductor: &mut Conductor<T>,
    path: &Arc<Waypoints>,
    car: &str,
    i: usize,
) -> Result<(), ConductorError> {
    let s0 = path.total_length() * i as f64 / 3.0;
    let initial_pose = Pose {
        position: path.point_at(s0),
        orientation: Quat::from_yaw(path.heading_at(s0)),
    };
    conductor.add_component(
        car,
        Box::new(PathFollowController::new(
            car,
            path.clone(),
            SimDuration::from_millis(100),
            8.0, // m/s
            6.0, // lookahead, m
            1.5, // heading gain, 1/s
            1.2, // max yaw rate, rad/s
            initial_pose,
        )),
    )?;
    conductor.add_component(
        car,
        Box::new(UnicyclePhysics::new(
            car,
            SimDuration::from_millis(10),
            initial_pose,
        )),
    )?;

    // Return success; the car is registered.
    Ok(())
}

/// Registers the world-level pose logger, offset 1 ns past each second
/// boundary: the smallest offset that clears same-instant deferral, so
/// on-boundary poses are visible — and nothing can be scheduled between a
/// boundary and its sample.
pub fn add_logger<T: Transport>(conductor: &mut Conductor<T>) -> Result<(), ConductorError> {
    conductor.add_component(
        "",
        Box::new(PoseLogger::new(
            SimDuration::from_secs(1),
            SimDuration::from_nanos(1),
        )),
    )?;

    // Return success; the logger is registered.
    Ok(())
}

/// Populates the full live world: three cars plus the pose logger.
pub fn populate<T: Transport>(conductor: &mut Conductor<T>) -> Result<(), ConductorError> {
    let path = demo_path();
    for (i, car) in ["car1", "car2", "car3"].into_iter().enumerate() {
        add_live_car(conductor, &path, car, i)?;
    }
    add_logger(conductor)?;

    // Return success; the world is fully populated.
    Ok(())
}
