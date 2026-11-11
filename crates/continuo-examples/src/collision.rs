//! Watching a run for cars that collide.
//!
//! [`CollisionMonitor`] reads every car's pose off the transport the way a
//! recorder does, so watching changes nothing about the run: no component
//! steps for it, and nothing it holds reaches the world hash. Each car is a
//! [`CAR_LENGTH`] by [`CAR_WIDTH`] rectangle at its pose, and two cars
//! collide when their rectangles touch or overlap. Nothing here asks which
//! road or lane a car is on, so the check holds on curves, merges and
//! crossings as well as on the demo's straight road.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};

use continuo_actors::{CAR_LENGTH, CAR_WIDTH, CarState};
use continuo_conductor::{ConductorError, MembershipChange};
use continuo_core::{CoreError, KeyExpr, Message, SimTime};

use crate::traffic_world::ScenarioError;

/// How far each footprint grows, in meters, for a pair already in contact
/// to count as still touching.
///
/// A pair starts an episode when the footprints touch and ends it only once
/// the grown footprints are apart, so two cars hovering at contact report
/// once rather than on every tick they cross it.
const CONTACT_MARGIN: f64 = 0.5;

/// The ego's footprint touching or overlapping another car's.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[error("{ego} collided with {other} at sim time {at}")]
pub struct Collision {
    pub ego: String,
    pub other: String,
    /// The later of the two poses that overlapped.
    pub at: SimTime,
}

/// Watches every car's pose for an overlap with another car's.
///
/// An overlap between two cars that are not the ego is logged as a warning
/// once per episode and the run goes on, since traffic overlapping is a
/// flaw in the scenario rather than in what it tests. An overlap involving
/// the ego stops the run with [`ScenarioError::Collision`], since the
/// demo's claim is that the ego's controller never lets one happen.
///
/// Candidate pairs come from a grid of cells a little wider than the
/// furthest two touching cars can be apart, so a check costs the cars
/// present rather than every pair of them, and runs only when something has
/// changed since the last one.
///
/// Each check is a snapshot, comparing every car's latest pose. In the demo
/// every car's physics publishes on one 10 ms grid, so those poses share an
/// instant; a car on another period would be compared up to one of its
/// periods out of step. A snapshot also misses two cars passing through
/// each other between poses, which side-on means crossing two car widths in
/// one period: about 360 m/s at 10 ms.
// TODO(M7): poses from a remote host may arrive less often or late, which
// shrinks both of those margins. Dead reckoning, moving each latest pose to
// the check's instant along its own speed and heading, would line the poses
// up but still test a snapshot. A swept test, keeping each car's previous
// pose and checking the motion between the two, would cover both cases.
#[derive(Clone)]
pub struct CollisionMonitor {
    watch: Arc<Mutex<Watch>>,
    /// Every actor's pose key in the world being watched.
    pose_keys: KeyExpr,
    ego_name: String,
}

impl CollisionMonitor {
    /// A monitor for the world named `world_name`, halting on any overlap
    /// that involves the actor named `ego_name`.
    pub fn new(world_name: &str, ego_name: &str) -> Self {
        CollisionMonitor {
            watch: Arc::default(),
            pose_keys: KeyExpr::new_rooted(format!("{world_name}/actor/*/pose"))
                .expect("valid pose key"),
            ego_name: ego_name.to_string(),
        }
    }

    /// A monitor callback that takes in every car's pose. Attach it to a
    /// `MonitorTransport` wrapping the conductor's transport.
    pub fn message_callback(&self) -> impl FnMut(&Message) + Send + 'static {
        let watch = self.watch.clone();
        let pose_keys = self.pose_keys.clone();

        // Return the callback, holding its own handle on what it watches.
        move |message: &Message| {
            if !pose_keys.matches(&message.key) {
                return;
            }
            let mut watch = watch
                .lock()
                .expect("collision monitor mutex is never poisoned");
            match message.decode::<CarState>() {
                Ok(state) => {
                    watch.poses_seen += 1;
                    watch.place(
                        actor_of(message.key.as_str()),
                        Footprint::of(&state, message.sim_time),
                    );
                }
                // A pose nobody can read is a car the check cannot see. The
                // callback has nowhere to return an error to, so the first
                // one waits for the next check, which reports it.
                Err(error) => {
                    if watch.unreadable_pose.is_none() {
                        watch.unreadable_pose = Some(error);
                    }
                }
            }
        }
    }

    /// A membership callback that forgets a car once it leaves, so its last
    /// pose cannot collide with anything afterward. Register it with
    /// `Conductor::add_membership_callback`.
    pub fn membership_callback(&self) -> impl FnMut(&MembershipChange) + Send + 'static {
        let watch = self.watch.clone();

        // Return the callback, holding its own handle on what it watches.
        move |change: &MembershipChange| {
            // A car leaves one component at a time, so the first of them
            // to go is the one that forgets it.
            if let MembershipChange::Left(leave) = change {
                let actor = leave.path.split('/').next().unwrap_or(&leave.path);
                watch
                    .lock()
                    .expect("collision monitor mutex is never poisoned")
                    .remove(actor);
            }
        }
    }

    /// How many overlap episodes have started, the ego's included.
    pub fn episodes(&self) -> usize {
        self.lock().episodes
    }

    /// How many poses the monitor has taken in, which is what shows it was
    /// watching at all.
    pub fn poses_seen(&self) -> usize {
        self.lock().poses_seen
    }

    /// Looks for pairs that came into contact since the last check.
    ///
    /// Crate-private for the reason `LiveTrafficHandler::apply` is: the
    /// only loop that drives the live scenario is the one that calls it.
    pub(crate) fn check(&self) -> Result<(), ScenarioError> {
        let mut watch = self.lock();
        if let Some(error) = watch.unreadable_pose.take() {
            return Err(ConductorError::Core(error).into());
        }
        if !watch.changed {
            return Ok(());
        }
        watch.changed = false;

        for (first, second, at) in watch.update_contacts() {
            if first == self.ego_name || second == self.ego_name {
                let other = if first == self.ego_name {
                    second
                } else {
                    first
                };
                return Err(Collision {
                    ego: self.ego_name.clone(),
                    other,
                    at,
                }
                .into());
            }
            tracing::warn!(%first, %second, %at, "traffic cars collide");
        }

        // Return success; nothing touched the ego.
        Ok(())
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Watch> {
        self.watch
            .lock()
            .expect("collision monitor mutex is never poisoned")
    }
}

/// The actor a pose key belongs to: the segment after `actor`.
fn actor_of(pose_key: &str) -> &str {
    pose_key
        .split('/')
        .skip_while(|segment| *segment != "actor")
        .nth(1)
        .expect("a pose key names the actor it belongs to")
}

/// A car's position and heading, as the contact test wants them.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Footprint {
    /// The center, in world coordinates.
    x: f64,
    y: f64,
    /// A unit vector along the car's heading.
    forward: (f64, f64),
    /// When the pose was published.
    sim_time: SimTime,
}

impl Footprint {
    fn of(state: &CarState, sim_time: SimTime) -> Self {
        // The pose's rotation applied to the x axis, written out. A
        // quaternion turns a vector with multiplies and adds alone, so no
        // angle is formed and no transcendental is evaluated. Cars here stay
        // level, so the result has length one up to rounding, and dividing
        // by it keeps the contact test's half extents in meters.
        let q = state.orientation;
        let x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        let y = 2.0 * (q.x * q.y + q.w * q.z);
        let length = (x * x + y * y).sqrt();

        // Return the footprint at the pose's position.
        Footprint {
            x: state.position.x,
            y: state.position.y,
            forward: (x / length, y / length),
            sim_time,
        }
    }
}

/// Whether two footprints touch or overlap once each grows by `margin` on
/// every side.
///
/// The separating axis test: two rectangles are apart exactly when, along
/// one of their four edge directions, the distance between their centers
/// exceeds the sum of their half extents along it. A distance equal to that
/// sum is touching, and counts: cars that meet with no gap have collided.
fn touch(a: &Footprint, b: &Footprint, margin: f64) -> bool {
    let half_length = CAR_LENGTH / 2.0 + margin;
    let half_width = CAR_WIDTH / 2.0 + margin;
    let reach = |footprint: &Footprint, axis: (f64, f64)| {
        half_length * dot(footprint.forward, axis).abs()
            + half_width * dot(left_of(footprint.forward), axis).abs()
    };
    let between = (b.x - a.x, b.y - a.y);
    let axes = [a.forward, left_of(a.forward), b.forward, left_of(b.forward)];

    // Return whether no axis separates them.
    axes.iter()
        .all(|&axis| dot(between, axis).abs() <= reach(a, axis) + reach(b, axis))
}

fn dot(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

/// The direction a quarter turn to the left of `forward`.
fn left_of(forward: (f64, f64)) -> (f64, f64) {
    (-forward.1, forward.0)
}

/// A grid cell, by its integer coordinates.
type Cell = (i64, i64);

/// The side of a grid cell, in meters.
///
/// The grid only finds new contacts, and the furthest apart two centers can
/// be while their footprints touch is one footprint's diagonal, about
/// 4.85 m. The cell is the diagonal of a footprint grown by
/// [`CONTACT_MARGIN`], about 1.3 m more, so two cars that touch are in the
/// same cell or adjacent ones however the division rounds.
fn cell_size() -> f64 {
    let length = CAR_LENGTH + 2.0 * CONTACT_MARGIN;
    let width = CAR_WIDTH + 2.0 * CONTACT_MARGIN;

    // Return the grown footprint's diagonal.
    (length * length + width * width).sqrt()
}

/// The cell a center falls in. IEEE 754 rounds a division correctly and
/// `floor` exactly, so a center lands in the same cell on every platform.
fn cell_of(footprint: &Footprint) -> Cell {
    let size = cell_size();

    // Return the cell's integer coordinates.
    (
        (footprint.x / size).floor() as i64,
        (footprint.y / size).floor() as i64,
    )
}

/// What the monitor knows, behind the one lock its callbacks share.
#[derive(Default)]
struct Watch {
    /// Each car's latest footprint and the cell it is filed under.
    cars: BTreeMap<String, (Footprint, Cell)>,
    /// The cars in each occupied cell. An empty cell is removed rather
    /// than kept.
    grid: BTreeMap<Cell, BTreeSet<String>>,
    /// The pairs in an episode, each named in order.
    in_contact: BTreeSet<(String, String)>,
    /// Whether a pose has arrived or a car has gone since the last check.
    changed: bool,
    poses_seen: usize,
    episodes: usize,
    /// The first pose that could not be read, held until a check reports
    /// it.
    unreadable_pose: Option<CoreError>,
}

impl Watch {
    /// Records `actor`'s latest footprint, moving it to another cell if it
    /// crossed into one.
    fn place(&mut self, actor: &str, footprint: Footprint) {
        let cell = cell_of(&footprint);
        if let Some((_, previous)) = self.cars.get(actor) {
            let previous = *previous;
            if previous != cell {
                self.leave_cell(actor, previous);
            }
        }
        self.grid.entry(cell).or_default().insert(actor.to_string());
        self.cars.insert(actor.to_string(), (footprint, cell));
        self.changed = true;
    }

    /// Forgets `actor`, its footprint, its cell and any episode it is in.
    fn remove(&mut self, actor: &str) {
        if let Some((_, cell)) = self.cars.remove(actor) {
            self.leave_cell(actor, cell);
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
                    for first in cars {
                        for second in neighbors.range::<String, _>((
                            std::ops::Bound::Excluded(first),
                            std::ops::Bound::Unbounded,
                        )) {
                            pairs.insert((first.clone(), second.clone()));
                        }
                    }
                }
            }
        }

        // Return the candidates, ready for the contact test.
        pairs
    }

    /// Brings the episodes up to date with the latest footprints, returning
    /// the pairs whose episode starts now and when each did.
    ///
    /// A pair already in an episode stays in it while its grown footprints
    /// touch. Those pairs are checked directly rather than found through the
    /// grid, so whether an episode goes on never depends on which cells its
    /// cars are filed in. The grid's candidates are checked only for new
    /// contacts, which start when the footprints themselves touch.
    fn update_contacts(&mut self) -> Vec<(String, String, SimTime)> {
        // Every car in an episode is still known, since `remove` ends a
        // car's episodes when it forgets the car.
        let mut still_in_contact: BTreeSet<(String, String)> = self
            .in_contact
            .iter()
            .filter(|(first, second)| {
                touch(&self.cars[first].0, &self.cars[second].0, CONTACT_MARGIN)
            })
            .cloned()
            .collect();
        let mut started = Vec::new();
        for pair in self.pairs_in_reach() {
            if self.in_contact.contains(&pair) {
                continue;
            }
            let (first, _) = &self.cars[&pair.0];
            let (second, _) = &self.cars[&pair.1];
            if touch(first, second, 0.0) {
                started.push((
                    pair.0.clone(),
                    pair.1.clone(),
                    first.sim_time.max(second.sim_time),
                ));
                still_in_contact.insert(pair);
            }
        }
        self.in_contact = still_in_contact;
        self.episodes += started.len();

        // Return the episodes that began with this update.
        started
    }
}

#[cfg(test)]
mod tests {
    use continuo_core::{Pose, Quat, Vec3};
    use proptest::prelude::*;

    use super::*;

    /// A footprint at `(x, y)` pointing along `forward`, which need not be
    /// a unit vector.
    fn footprint(x: f64, y: f64, forward: (f64, f64)) -> Footprint {
        let length = (forward.0 * forward.0 + forward.1 * forward.1).sqrt();
        Footprint {
            x,
            y,
            forward: (forward.0 / length, forward.1 / length),
            sim_time: SimTime::ZERO,
        }
    }

    const EAST: (f64, f64) = (1.0, 0.0);
    const NORTH: (f64, f64) = (0.0, 1.0);

    #[test]
    fn a_footprint_points_where_the_pose_does() {
        // Built from the quaternion of a quarter turn left, so the forward
        // axis should come out pointing north.
        let state = CarState::new(
            Pose {
                position: Vec3 {
                    x: 3.0,
                    y: -2.0,
                    z: 0.0,
                },
                orientation: Quat::from_yaw(std::f64::consts::FRAC_PI_2),
            },
            10.0,
        );
        let found = Footprint::of(&state, SimTime::ZERO);
        assert_eq!((found.x, found.y), (3.0, -2.0));
        assert!(found.forward.0.abs() < 1e-12, "{found:?}");
        assert!((found.forward.1 - 1.0).abs() < 1e-12, "{found:?}");
    }

    #[test]
    fn a_car_closing_on_another_in_its_lane_collides_from_bumper_to_bumper() {
        // Both cars share their long edges' lines, the case a test on edges
        // crossing gets wrong. Exactly a car length apart they touch end to
        // end, with no gap, which counts.
        let lead = footprint(10.0, 0.0, EAST);
        let behind = |gap: f64| footprint(10.0 - CAR_LENGTH - gap, 0.0, EAST);
        assert!(touch(&lead, &behind(-0.1), 0.0));
        assert!(touch(&lead, &behind(0.0), 0.0));
        assert!(!touch(&lead, &behind(0.1), 0.0));
    }

    #[test]
    fn cars_side_by_side_in_neighboring_lanes_do_not_collide() {
        // 3.5 m apart center to center, which is less than one car length,
        // so a test on distance alone would call this a collision.
        let left = footprint(0.0, 3.5, EAST);
        let right = footprint(0.0, 0.0, EAST);
        assert!(!touch(&left, &right, 0.0));
        assert!(touch(&left, &right, CAR_WIDTH));
    }

    #[test]
    fn rectangles_touching_only_at_a_corner_collide() {
        // Corner to corner exactly. Halving and doubling the extents are
        // exact, so the touch is exact rather than a rounding either way.
        let a = footprint(0.0, 0.0, EAST);
        assert!(touch(&a, &footprint(CAR_LENGTH, CAR_WIDTH, EAST), 0.0));
        assert!(!touch(
            &a,
            &footprint(CAR_LENGTH, CAR_WIDTH + 0.01, EAST),
            0.0
        ));
    }

    #[test]
    fn a_car_crossing_another_at_right_angles_overlaps_where_their_bodies_meet() {
        // The north-bound car's nose reaches into the east-bound car's side,
        // where neither center is within the other's footprint.
        let east = footprint(0.0, 0.0, EAST);
        let north_touching = footprint(0.0, -(CAR_WIDTH / 2.0 + CAR_LENGTH / 2.0) + 0.1, NORTH);
        let north_clear = footprint(0.0, -(CAR_WIDTH / 2.0 + CAR_LENGTH / 2.0) - 0.1, NORTH);
        assert!(touch(&east, &north_touching, 0.0));
        assert!(!touch(&east, &north_clear, 0.0));
    }

    #[test]
    fn a_car_turned_45_degrees_can_be_apart_along_its_own_edges_alone() {
        // 4.45 m ahead, the turned car's extent along the road, about
        // 2.23 m, still overlaps the other's, so the other car's edge
        // directions see no gap. Its own left edge direction does, which is
        // why the test takes the edge directions of both cars.
        let east = footprint(0.0, 0.0, EAST);
        assert!(!touch(&east, &footprint(4.45, 0.0, (1.0, 1.0)), 0.0));
        assert!(touch(&east, &footprint(4.3, 0.0, (1.0, 1.0)), 0.0));
    }

    #[test]
    fn a_pair_overlapping_twice_without_parting_is_one_episode() {
        let mut watch = Watch::default();
        watch.place("a", footprint(0.0, 0.0, EAST));
        watch.place("b", footprint(4.0, 0.0, EAST));
        assert_eq!(watch.update_contacts().len(), 1);

        // Just apart, but within the margin: the same episode.
        watch.place("b", footprint(CAR_LENGTH + 0.2, 0.0, EAST));
        assert!(watch.update_contacts().is_empty());
        watch.place("b", footprint(4.0, 0.0, EAST));
        assert!(watch.update_contacts().is_empty());

        // Apart by more than the margin, then back: a second episode.
        watch.place(
            "b",
            footprint(CAR_LENGTH + 2.0 * CONTACT_MARGIN + 0.1, 0.0, EAST),
        );
        assert!(watch.update_contacts().is_empty());
        watch.place("b", footprint(4.0, 0.0, EAST));
        assert_eq!(watch.update_contacts().len(), 1);
        assert_eq!(watch.episodes, 2);
    }

    #[test]
    fn an_episode_goes_on_whichever_cells_its_cars_are_filed_in() {
        // Apart, but within the margin, so the episode goes on. Filing the
        // second car three cells away by hand takes the pair out of the
        // grid's candidates, which must not end it.
        let mut watch = Watch::default();
        watch.place("a", footprint(0.0, 0.0, EAST));
        watch.place("b", footprint(4.0, 0.0, EAST));
        assert_eq!(watch.update_contacts().len(), 1);

        watch.place("b", footprint(CAR_LENGTH + 0.2, 0.0, EAST));
        let (_, filed) = watch.cars["b"];
        let elsewhere = (filed.0 + 3, filed.1);
        watch.leave_cell("b", filed);
        watch
            .grid
            .entry(elsewhere)
            .or_default()
            .insert("b".to_string());
        watch.cars.get_mut("b").expect("b is placed").1 = elsewhere;
        assert!(watch.pairs_in_reach().is_empty());

        assert!(watch.update_contacts().is_empty());
        assert!(
            watch
                .in_contact
                .contains(&("a".to_string(), "b".to_string()))
        );
        assert_eq!(watch.episodes, 1);
    }

    #[test]
    fn a_car_that_leaves_takes_its_cell_and_its_episodes_with_it() {
        let mut watch = Watch::default();
        watch.place("a", footprint(0.0, 0.0, EAST));
        watch.place("b", footprint(4.0, 0.0, EAST));
        watch.update_contacts();
        watch.remove("b");
        assert!(watch.in_contact.is_empty());
        assert_eq!(watch.grid.values().flatten().count(), 1);
        assert!(watch.update_contacts().is_empty());
    }

    /// Every pair whose footprints touch, found by checking each pair of
    /// cars: the answer the grid must reproduce.
    fn every_pair_in_contact(watch: &Watch) -> BTreeSet<(String, String)> {
        let mut pairs = BTreeSet::new();
        for (first, (a, _)) in &watch.cars {
            for (second, (b, _)) in watch
                .cars
                .range::<String, _>((std::ops::Bound::Excluded(first), std::ops::Bound::Unbounded))
            {
                if touch(a, b, 0.0) {
                    pairs.insert((first.clone(), second.clone()));
                }
            }
        }

        // Return the pairs every-pair checking finds.
        pairs
    }

    /// The pairs among the grid's candidates whose footprints touch.
    fn grid_pairs_in_contact(watch: &Watch) -> BTreeSet<(String, String)> {
        watch
            .pairs_in_reach()
            .into_iter()
            .filter(|(first, second)| touch(&watch.cars[first].0, &watch.cars[second].0, 0.0))
            .collect()
    }

    /// One thing that can happen to a car between two checks: it moves to
    /// a point at a heading, or it leaves.
    #[derive(Debug, Clone)]
    enum Event {
        Place {
            car: u8,
            x: f64,
            y: f64,
            heading: (f64, f64),
        },
        Leave {
            car: u8,
        },
    }

    /// Events among a dozen cars in a 40 m square around the origin, which
    /// is a few cells across in each direction, so cars cross cell edges,
    /// share cells, and sit at negative coordinates. A heading is drawn as
    /// a vector rather than an angle so no transcendental is involved.
    fn any_events() -> impl Strategy<Value = Vec<Event>> {
        let place = (
            0u8..12,
            -20.0..20.0f64,
            -20.0..20.0f64,
            -1.0..1.0f64,
            -1.0..1.0f64,
        )
            .prop_filter("a heading needs a direction", |(_, _, _, hx, hy)| {
                hx * hx + hy * hy > 1e-6
            })
            .prop_map(|(car, x, y, hx, hy)| Event::Place {
                car,
                x,
                y,
                heading: (hx, hy),
            });
        let leave = (0u8..12).prop_map(|car| Event::Leave { car });
        proptest::collection::vec(prop_oneof![4 => place, 1 => leave], 1..80)
    }

    proptest! {
        // A failure's seed lands in `proptest-regressions/collision.txt`
        // under this crate, and every run replays it before drawing fresh
        // cases. It is committed, the fix lands as a unit test above, and
        // the file is deleted when proptest moves in the lock or the
        // generator changes, as DECISIONS.md settles for the frenet
        // properties.
        #![proptest_config(ProptestConfig::with_cases(512))]

        #[test]
        fn the_grid_finds_exactly_the_pairs_every_pair_checking_finds(
            events in any_events(),
        ) {
            let mut watch = Watch::default();
            for event in events {
                match event {
                    Event::Place { car, x, y, heading } => {
                        watch.place(&format!("car{car}"), footprint(x, y, heading));
                    }
                    Event::Leave { car } => watch.remove(&format!("car{car}")),
                }
                prop_assert_eq!(grid_pairs_in_contact(&watch), every_pair_in_contact(&watch));
            }

            // Every car the grid files is one it knows, in the cell it says.
            for (cell, cars) in &watch.grid {
                prop_assert!(!cars.is_empty());
                for car in cars {
                    prop_assert_eq!(watch.cars[car].1, *cell);
                }
            }
            prop_assert_eq!(watch.grid.values().map(BTreeSet::len).sum::<usize>(), watch.cars.len());
        }
    }
}
