//! Watching a run for cars that collide.
//!
//! [`CollisionMonitor`] reads poses off the transport, so watching cannot
//! change the run. Each car is a [`CAR_LENGTH`] by [`CAR_WIDTH`] rectangle at
//! its pose, with no assumption about roads or lanes.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};

use continuo_actors::{CAR_LENGTH, CAR_WIDTH, actor_of_pose_key};
use continuo_conductor::{ConductorError, MembershipChange, RecordedCollision};
use continuo_core::{ComponentPath, CoreError, KeyExpr, Message, Pose, SimTime, Vec3};

use crate::traffic_world::ScenarioError;

/// How far each footprint grows, in meters, for a pair in a collision
/// episode to count as still in contact.
const CONTACT_MARGIN: f64 = 0.5;

/// Watches every car for contact with another: traffic in contact logs a
/// warning once per collision episode, and the ego in contact stops the run
/// with [`ScenarioError::EgoCollision`].
// TODO(M7): each check is a snapshot of every car's latest pose, which holds
// while every car publishes often and at the same instants. Remote hosts may
// publish less often or late. Dead reckoning to the check's instant would
// line the poses up; a swept test over each car's previous and current pose
// would also catch cars passing through each other between poses.
#[derive(Clone)]
pub struct CollisionMonitor {
    contact_tracker: Arc<Mutex<ContactTracker>>,
    /// Every actor's pose key in the world being watched.
    pose_key_pattern: KeyExpr,
    ego_name: String,
}

impl CollisionMonitor {
    /// A monitor for the world `world_name`, halting on contact with
    /// `ego_name`.
    pub fn new(world_name: &str, ego_name: &str) -> Self {
        CollisionMonitor {
            contact_tracker: Arc::default(),
            pose_key_pattern: KeyExpr::new_rooted(format!("{world_name}/actor/*/pose"))
                .expect("valid pose key"),
            ego_name: ego_name.to_string(),
        }
    }

    /// A `MonitorTransport` callback that takes in every car's pose.
    pub fn message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let contact_tracker = self.contact_tracker.clone();
        let pose_key_pattern = self.pose_key_pattern.clone();

        // Return the callback, holding its own handle on what it watches.
        move |message: &Message| {
            if !pose_key_pattern.matches(&message.key) {
                return;
            }
            let mut contact_tracker = contact_tracker
                .lock()
                .expect("collision monitor mutex is never poisoned");
            match message.decode::<Pose>() {
                Ok(pose) => {
                    contact_tracker.place_on_grid(
                        actor_of_pose_key(message.key.as_str()),
                        ActorPlacement::of(&pose, message.sim_time),
                    );
                }
                // The callback cannot return an error, so the next check
                // reports the first one.
                Err(error) => {
                    if contact_tracker.unreadable_pose.is_none() {
                        contact_tracker.unreadable_pose = Some(error);
                    }
                }
            }
        }
    }

    /// A membership callback that removes a car when it leaves.
    pub fn membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let contact_tracker = self.contact_tracker.clone();

        // Return the callback, holding its own handle on what it watches.
        move |change: &MembershipChange| {
            // The first of a car's components to leave removes the car.
            if let MembershipChange::Left(leave) = change {
                let path = ComponentPath::parse(&leave.path).expect("a leave names a valid path");
                if let Some(actor) = path.actor() {
                    contact_tracker
                        .lock()
                        .expect("collision monitor mutex is never poisoned")
                        .remove(actor.as_str());
                }
            }
        }
    }

    /// How many collision episodes have started, the ego's included.
    pub fn num_collisions(&self) -> usize {
        self.lock().num_collisions
    }

    /// The collisions that began since the last check, failing on the
    /// ego's. Crate-private so only the live loop calls it.
    pub(crate) fn check_collisions(&self) -> Result<Vec<RecordedCollision>, ScenarioError> {
        let mut contact_tracker = self.lock();
        if let Some(error) = contact_tracker.unreadable_pose.take() {
            return Err(ConductorError::Core(error).into());
        }
        if !contact_tracker.changed {
            return Ok(Vec::new());
        }
        contact_tracker.changed = false;
        let collisions = contact_tracker.update_contacts();

        for collision in &collisions {
            if collision.first == self.ego_name || collision.second == self.ego_name {
                return Err(ScenarioError::EgoCollision(collision.clone()));
            }
            tracing::warn!(
                first = %collision.first,
                second = %collision.second,
                sim_time = %collision.sim_time,
                "traffic cars collide"
            );
        }

        // Return the traffic's collisions, none of which halt the run.
        Ok(collisions)
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, ContactTracker> {
        self.contact_tracker
            .lock()
            .expect("collision monitor mutex is never poisoned")
    }
}

/// An actor's position and heading in the plane, and when they were
/// published.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ActorPlacement {
    /// The center, in world coordinates.
    x: f64,
    y: f64,
    /// A unit vector along the actor's heading, in the plane.
    forward: Vec3,
    /// When the pose was published.
    sim_time: SimTime,
}

impl ActorPlacement {
    fn of(pose: &Pose, sim_time: SimTime) -> Self {
        // The body's forward axis, flattened onto the plane.
        let forward = pose.orientation.forward();
        let flat = Vec3::new(forward.x, forward.y, 0.0);
        let length = flat.length();

        // Return the placement at the pose's position.
        ActorPlacement {
            x: pose.position.x,
            y: pose.position.y,
            forward: Vec3::new(flat.x / length, flat.y / length, 0.0),
            sim_time,
        }
    }
}

/// Whether the footprints of two placed actors, each grown by `margin`,
/// touch or overlap, by the separating axis test with a zero gap counted as
/// contact.
// TODO(PLAN "World and map"): every actor is given a car's size. Read each
// actor's size from the world spec instead.
fn are_touching(a: &ActorPlacement, b: &ActorPlacement, margin: f64) -> bool {
    let half_length = CAR_LENGTH / 2.0 + margin;
    let half_width = CAR_WIDTH / 2.0 + margin;
    // Up crossed with forward is a quarter turn left of it.
    let left_of = |forward: Vec3| Vec3::UP.cross(forward);
    let reach = |placement: &ActorPlacement, axis: Vec3| {
        half_length * placement.forward.dot(axis).abs()
            + half_width * left_of(placement.forward).dot(axis).abs()
    };
    let between = Vec3::new(b.x - a.x, b.y - a.y, 0.0);
    let axes = [a.forward, left_of(a.forward), b.forward, left_of(b.forward)];

    // Return whether no axis separates them.
    axes.iter()
        .all(|&axis| between.dot(axis).abs() <= reach(a, axis) + reach(b, axis))
}

/// A grid cell, by its integer coordinates.
type Cell = (i64, i64);

/// The side of a grid cell, in meters: a grown footprint's diagonal, about
/// 1.3 m more than two touching cars can be apart, so they always share a
/// cell or adjacent ones.
fn cell_size() -> f64 {
    let length = CAR_LENGTH + 2.0 * CONTACT_MARGIN;
    let width = CAR_WIDTH + 2.0 * CONTACT_MARGIN;

    // Return the grown footprint's diagonal.
    (length * length + width * width).sqrt()
}

/// The cell a center falls in.
fn cell_of(placement: &ActorPlacement) -> Cell {
    let size = cell_size();

    // Return the cell's integer coordinates.
    (
        (placement.x / size).floor() as i64,
        (placement.y / size).floor() as i64,
    )
}

/// A car's latest placement and the cell it is filed under.
#[derive(Debug, Clone, Copy)]
struct CarOnGrid {
    placement: ActorPlacement,
    cell: Cell,
}

/// Every car's latest placement, filed on a grid, and the pairs in a
/// collision episode.
#[derive(Default)]
struct ContactTracker {
    cars: BTreeMap<String, CarOnGrid>,
    /// The cars in each occupied cell.
    grid: BTreeMap<Cell, BTreeSet<String>>,
    /// The pairs in a collision episode, each named in order.
    in_contact: BTreeSet<(String, String)>,
    /// Whether a pose has arrived or a car has gone since the last check.
    changed: bool,
    num_collisions: usize,
    /// The first pose that could not be read, for the next check to report.
    unreadable_pose: Option<CoreError>,
}

impl ContactTracker {
    /// Records `actor`'s latest placement, moving it to another cell if it
    /// crossed into one.
    fn place_on_grid(&mut self, actor: &str, placement: ActorPlacement) {
        let cell = cell_of(&placement);
        if let Some(prev_cell) = self.cars.get(actor).map(|car| car.cell)
            && prev_cell != cell
        {
            self.leave_cell(actor, prev_cell);
        }
        self.grid.entry(cell).or_default().insert(actor.to_string());
        self.cars
            .insert(actor.to_string(), CarOnGrid { placement, cell });
        self.changed = true;
    }

    /// Removes `actor`, its placement, its cell and any collision episode
    /// it is in.
    fn remove(&mut self, actor: &str) {
        if let Some(car) = self.cars.remove(actor) {
            self.leave_cell(actor, car.cell);
            self.in_contact
                .retain(|(first, second)| first != actor && second != actor);
            self.changed = true;
        }
    }

    fn leave_cell(&mut self, actor: &str, cell: Cell) {
        if let Some(cars) = self.grid.get_mut(&cell) {
            cars.remove(actor);
            if cars.is_empty() {
                self.grid.remove(&cell);
            }
        }
    }

    /// Every pair of cars in the same cell or adjacent ones, each named in
    /// order. Any pair whose footprints touch is among them.
    fn pairs_in_reach(&self) -> BTreeSet<(String, String)> {
        let mut pairs = BTreeSet::new();
        for (&(column, row), cars) in &self.grid {
            for neighbor_column in column - 1..=column + 1 {
                for neighbor_row in row - 1..=row + 1 {
                    let Some(neighbors) = self.grid.get(&(neighbor_column, neighbor_row)) else {
                        continue;
                    };
                    // Each pair once, under one name: the smaller actor name
                    // first, and never a car with itself. Collision episodes
                    // are keyed by that name.
                    for first in cars {
                        for second in neighbors.iter().filter(|second| *second > first) {
                            pairs.insert((first.clone(), second.clone()));
                        }
                    }
                }
            }
        }

        // Return the candidates, ready for the contact test.
        pairs
    }

    /// Updates the collision episodes and returns those starting now. Ongoing
    /// ones are checked directly with grown footprints; the grid finds new
    /// ones.
    fn update_contacts(&mut self) -> Vec<RecordedCollision> {
        // `remove` ends a car's collision episodes, so every car here is known.
        let mut still_in_contact: BTreeSet<(String, String)> = self
            .in_contact
            .iter()
            .filter(|(first_name, second_name)| {
                are_touching(
                    &self.cars[first_name].placement,
                    &self.cars[second_name].placement,
                    CONTACT_MARGIN,
                )
            })
            .cloned()
            .collect();
        let mut started = Vec::new();
        for pair_names in self.pairs_in_reach() {
            if self.in_contact.contains(&pair_names) {
                continue;
            }
            let first_placement = &self.cars[&pair_names.0].placement;
            let second_placement = &self.cars[&pair_names.1].placement;
            if are_touching(first_placement, second_placement, 0.0) {
                started.push(RecordedCollision {
                    first: pair_names.0.clone(),
                    second: pair_names.1.clone(),
                    // The later pose, since the collision is known only once
                    // both are. The two times are equal when both cars step
                    // at the same instants; `CollisionMonitor`'s TODO covers
                    // when they do not.
                    sim_time: first_placement.sim_time.max(second_placement.sim_time),
                });
                still_in_contact.insert(pair_names);
            }
        }
        self.in_contact = still_in_contact;
        self.num_collisions += started.len();

        // Return the collision episodes that began with this update.
        started
    }
}

#[cfg(test)]
mod test_collision_properties;

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    use continuo_core::{Quat, Vec3};

    use super::*;

    /// An actor at `(x, y)` heading `yaw` radians counterclockwise from
    /// east.
    fn placed(x: f64, y: f64, yaw: f64) -> ActorPlacement {
        ActorPlacement::of(
            &Pose {
                position: Vec3::new(x, y, 0.0),
                orientation: Quat::from_yaw(yaw),
            },
            SimTime::ZERO,
        )
    }

    #[test]
    fn a_placement_points_where_the_pose_does() {
        // A quarter turn left, so the forward axis should point north.
        let found = placed(3.0, -2.0, FRAC_PI_2);
        assert_eq!((found.x, found.y), (3.0, -2.0));
        assert!(found.forward.x.abs() < 1e-12, "{found:?}");
        assert!((found.forward.y - 1.0).abs() < 1e-12, "{found:?}");
    }

    #[test]
    fn a_car_closing_on_another_in_its_lane_collides_from_bumper_to_bumper() {
        // Both cars share their long edges' lines, the case a test on edges
        // crossing gets wrong. Exactly a car length apart they touch end to
        // end, with no gap, which counts.
        let lead = placed(10.0, 0.0, 0.0);
        let behind = |gap: f64| placed(10.0 - CAR_LENGTH - gap, 0.0, 0.0);
        assert!(are_touching(&lead, &behind(-0.1), 0.0));
        assert!(are_touching(&lead, &behind(0.0), 0.0));
        assert!(!are_touching(&lead, &behind(0.1), 0.0));
    }

    #[test]
    fn cars_side_by_side_in_neighboring_lanes_do_not_collide() {
        // 3.5 m apart center to center, which is less than one car length,
        // so a test on distance alone would call this a collision.
        let left = placed(0.0, 3.5, 0.0);
        let right = placed(0.0, 0.0, 0.0);
        assert!(!are_touching(&left, &right, 0.0));
        assert!(are_touching(&left, &right, CAR_WIDTH));
    }

    #[test]
    fn rectangles_touching_only_at_a_corner_collide() {
        // Corner to corner exactly. A zero yaw is exactly the identity, and
        // halving and doubling the extents are exact, so the touch is exact
        // rather than a rounding either way.
        let a = placed(0.0, 0.0, 0.0);
        assert!(are_touching(&a, &placed(CAR_LENGTH, CAR_WIDTH, 0.0), 0.0));
        assert!(!are_touching(
            &a,
            &placed(CAR_LENGTH, CAR_WIDTH + 0.01, 0.0),
            0.0
        ));
    }

    #[test]
    fn a_car_crossing_another_at_right_angles_overlaps_where_their_bodies_meet() {
        // The north-bound car's nose reaches into the east-bound car's side,
        // where neither center is within the other's footprint.
        let east = placed(0.0, 0.0, 0.0);
        let reach = CAR_WIDTH / 2.0 + CAR_LENGTH / 2.0;
        assert!(are_touching(
            &east,
            &placed(0.0, -reach + 0.1, FRAC_PI_2),
            0.0
        ));
        assert!(!are_touching(
            &east,
            &placed(0.0, -reach - 0.1, FRAC_PI_2),
            0.0
        ));
    }

    #[test]
    fn a_car_turned_45_degrees_can_be_apart_along_its_own_edges_alone() {
        // 4.45 m ahead, the turned car's extent along the road, about
        // 2.23 m, still overlaps the other's, so the other car's edge
        // directions see no gap. Its own left edge direction does, which is
        // why the test takes the edge directions of both cars.
        let east = placed(0.0, 0.0, 0.0);
        assert!(!are_touching(&east, &placed(4.45, 0.0, FRAC_PI_4), 0.0));
        assert!(are_touching(&east, &placed(4.3, 0.0, FRAC_PI_4), 0.0));
    }

    #[test]
    fn a_pair_overlapping_twice_without_parting_is_one_collision_episode() {
        let mut contact_tracker = ContactTracker::default();
        contact_tracker.place_on_grid("a", placed(0.0, 0.0, 0.0));
        contact_tracker.place_on_grid("b", placed(4.0, 0.0, 0.0));
        assert_eq!(contact_tracker.update_contacts().len(), 1);

        // Just apart, but within the margin: the same collision episode.
        contact_tracker.place_on_grid("b", placed(CAR_LENGTH + 0.2, 0.0, 0.0));
        assert!(contact_tracker.update_contacts().is_empty());
        contact_tracker.place_on_grid("b", placed(4.0, 0.0, 0.0));
        assert!(contact_tracker.update_contacts().is_empty());

        // Apart by more than the margin, then back: a second collision
        // episode.
        contact_tracker.place_on_grid(
            "b",
            placed(CAR_LENGTH + 2.0 * CONTACT_MARGIN + 0.1, 0.0, 0.0),
        );
        assert!(contact_tracker.update_contacts().is_empty());
        contact_tracker.place_on_grid("b", placed(4.0, 0.0, 0.0));
        assert_eq!(contact_tracker.update_contacts().len(), 1);
        assert_eq!(contact_tracker.num_collisions, 2);
    }

    #[test]
    fn a_collision_episode_goes_on_whichever_cells_its_cars_are_filed_in() {
        // Apart, but within the margin, so the collision episode goes on.
        // Filing the second car three cells away by hand takes the pair out
        // of the grid's candidates, which must not end it.
        let mut contact_tracker = ContactTracker::default();
        contact_tracker.place_on_grid("a", placed(0.0, 0.0, 0.0));
        contact_tracker.place_on_grid("b", placed(4.0, 0.0, 0.0));
        assert_eq!(contact_tracker.update_contacts().len(), 1);

        contact_tracker.place_on_grid("b", placed(CAR_LENGTH + 0.2, 0.0, 0.0));
        let filed_cell = contact_tracker.cars["b"].cell;
        let elsewhere = (filed_cell.0 + 3, filed_cell.1);
        contact_tracker.leave_cell("b", filed_cell);
        contact_tracker
            .grid
            .entry(elsewhere)
            .or_default()
            .insert("b".to_string());
        contact_tracker.cars.get_mut("b").expect("b is placed").cell = elsewhere;
        assert!(contact_tracker.pairs_in_reach().is_empty());

        assert!(contact_tracker.update_contacts().is_empty());
        assert!(
            contact_tracker
                .in_contact
                .contains(&("a".to_string(), "b".to_string()))
        );
        assert_eq!(contact_tracker.num_collisions, 1);
    }

    #[test]
    fn a_car_that_leaves_takes_its_cell_and_its_collision_episodes_with_it() {
        let mut contact_tracker = ContactTracker::default();
        contact_tracker.place_on_grid("a", placed(0.0, 0.0, 0.0));
        contact_tracker.place_on_grid("b", placed(4.0, 0.0, 0.0));
        contact_tracker.update_contacts();
        contact_tracker.remove("b");
        assert!(contact_tracker.in_contact.is_empty());
        assert_eq!(contact_tracker.grid.values().flatten().count(), 1);
        assert!(contact_tracker.update_contacts().is_empty());
    }
}
