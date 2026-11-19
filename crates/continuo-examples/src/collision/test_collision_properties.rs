//! Property tests for the collision monitor. On random placements, each
//! checks the monitor's method against a brute force one: `are_touching`, a
//! separating axis test, against testing every corner and every pair of
//! edges of the two rectangles, and the grid's candidate pairs against
//! checking every pair of cars.
//!
//! The grid can match every pair because a cell is wider than two touching
//! cars' centers can be apart, so a touching pair is always filed in one cell
//! or in neighboring ones.

use std::collections::BTreeSet;
use std::f64::consts::PI;

use continuo_actors::{CAR_LENGTH, CAR_WIDTH};
use continuo_core::{Pose, Quat, SimTime, Vec3};
use proptest::prelude::*;

use super::{ActorPlacement, ContactTracker, are_touching};

/// An actor at `(x, y)` heading `yaw` radians counterclockwise from east.
fn placed(x: f64, y: f64, yaw: f64) -> ActorPlacement {
    ActorPlacement::of(
        &Pose {
            position: Vec3::new(x, y, 0.0),
            orientation: Quat::from_yaw(yaw),
        },
        SimTime::ZERO,
    )
}

/// The corners of an actor's footprint, each side moved out by `grow`, in
/// counterclockwise order.
fn corners(placement: &ActorPlacement, grow: f64) -> [(f64, f64); 4] {
    let (fx, fy) = (placement.forward.x, placement.forward.y);
    let half_length = CAR_LENGTH / 2.0 + grow;
    let half_width = CAR_WIDTH / 2.0 + grow;
    let corner = |along: f64, across: f64| {
        (
            placement.x + along * fx - across * fy,
            placement.y + along * fy + across * fx,
        )
    };

    // Return front right, front left, back left, back right.
    [
        corner(half_length, -half_width),
        corner(half_length, half_width),
        corner(-half_length, half_width),
        corner(-half_length, -half_width),
    ]
}

/// The z component of the cross product (b - a) x (p - a): positive when
/// `p` is left of the line from `a` through `b`, negative when right, and
/// zero on it. The orientation test, `orient2d` in computational geometry.
fn side(a: (f64, f64), b: (f64, f64), p: (f64, f64)) -> f64 {
    (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0)
}

/// Whether two convex quadrilaterals given counterclockwise overlap: a
/// corner of one lies inside the other, or an edge of one crosses an edge of
/// the other. Two convex polygons overlap only if their boundaries cross or
/// one contains the other, which puts its corners inside. See O'Rourke,
/// "Computational Geometry in C", chapter 7.
fn quadrilaterals_overlap(a: &[(f64, f64); 4], b: &[(f64, f64); 4]) -> bool {
    let edges = |q: &[(f64, f64); 4]| (0..4).map(|i| (q[i], q[(i + 1) % 4])).collect::<Vec<_>>();
    let inside = |p: (f64, f64), q: &[(f64, f64); 4]| {
        edges(q).iter().all(|&(from, to)| side(from, to, p) >= 0.0)
    };
    let cross = |(p, q): ((f64, f64), (f64, f64)), (r, s): ((f64, f64), (f64, f64))| {
        side(p, q, r) * side(p, q, s) < 0.0 && side(r, s, p) * side(r, s, q) < 0.0
    };

    // Return whether any corner is inside or any two edges cross.
    a.iter().any(|&p| inside(p, b))
        || b.iter().any(|&p| inside(p, a))
        || edges(a)
            .iter()
            .any(|&ea| edges(b).iter().any(|&eb| cross(ea, eb)))
}

/// Every pair whose footprints touch, found by checking each pair of cars:
/// the answer the grid must reproduce.
fn every_pair_in_contact(contact_tracker: &ContactTracker) -> BTreeSet<(String, String)> {
    let mut pairs = BTreeSet::new();
    for (first_name, first_car) in &contact_tracker.cars {
        for (second_name, second_car) in contact_tracker
            .cars
            .iter()
            .filter(|(second_name, _)| *second_name > first_name)
        {
            if are_touching(&first_car.placement, &second_car.placement, 0.0) {
                pairs.insert((first_name.clone(), second_name.clone()));
            }
        }
    }

    // Return the pairs every-pair checking finds.
    pairs
}

/// The pairs among the grid's candidates whose footprints touch.
fn grid_pairs_in_contact(contact_tracker: &ContactTracker) -> BTreeSet<(String, String)> {
    contact_tracker
        .pairs_in_reach()
        .into_iter()
        .filter(|(first_name, second_name)| {
            are_touching(
                &contact_tracker.cars[first_name].placement,
                &contact_tracker.cars[second_name].placement,
                0.0,
            )
        })
        .collect()
}

/// One thing that can happen to a car between two checks: it moves to a
/// point at a heading, or it leaves.
#[derive(Debug, Clone)]
enum Event {
    Place { car: u8, x: f64, y: f64, yaw: f64 },
    Leave { car: u8 },
}

prop_compose! {
    /// An actor within 6 m of the origin in each direction, at any heading,
    /// so two of them overlap about as often as not.
    fn any_placement()(
        x in -6.0..6.0f64,
        y in -6.0..6.0f64,
        yaw in -PI..PI,
    ) -> ActorPlacement {
        placed(x, y, yaw)
    }
}

/// How many cars the events move and remove.
const NUM_CARS: u8 = 12;

prop_compose! {
    /// A car moving to a point in a 40 m square around the origin, which is
    /// a few cells across in each direction, so cars cross cell edges, share
    /// cells, and sit at negative coordinates.
    fn any_place_event()(
        car in 0..NUM_CARS,
        x in -20.0..20.0f64,
        y in -20.0..20.0f64,
        yaw in -PI..PI,
    ) -> Event {
        Event::Place { car, x, y, yaw }
    }
}

prop_compose! {
    /// A car leaving.
    fn any_leave_event()(car in 0..NUM_CARS) -> Event {
        Event::Leave { car }
    }
}

/// Up to 80 events, four moves to each leave.
fn any_events() -> impl Strategy<Value = Vec<Event>> {
    // Each event is drawn from one of the two strategies by weight, four
    // moves to each leave, so most cars are on the grid at any time and
    // pairs meet and share cells. A car that leaves can be placed again.
    let any_event = prop_oneof![4 => any_place_event(), 1 => any_leave_event()];

    // Return lists of 1 to 80 such events.
    proptest::collection::vec(any_event, 1..=80)
}

proptest! {
    // A failure's seed lands in
    // `proptest-regressions/collision/test_collision_properties.txt` under
    // this crate, and every run replays it before drawing fresh cases. It is
    // committed, the fix lands as a unit test in `collision.rs`, and the
    // file is deleted when proptest moves in the lock or the generator
    // changes, as DECISIONS.md settles for the frenet properties.
    #![proptest_config(ProptestConfig::with_cases(512))]

    #[test]
    fn the_contact_test_agrees_with_intersecting_the_rectangles(
        a in any_placement(),
        b in any_placement(),
    ) {
        // Rounding can split the two methods only at an exact touch, so each
        // is held to the other only where a nanometer cannot change the
        // answer. Shrinking both rectangles can only remove overlap, so if
        // they still overlap the real ones certainly do.
        if quadrilaterals_overlap(&corners(&a, -1e-9), &corners(&b, -1e-9)) {
            prop_assert!(are_touching(&a, &b, 0.0));
        }
        // Growing both can only add overlap, so if they are still apart the
        // real ones certainly are.
        if !quadrilaterals_overlap(&corners(&a, 1e-9), &corners(&b, 1e-9)) {
            prop_assert!(!are_touching(&a, &b, 0.0));
        }
    }

    #[test]
    fn the_grid_finds_exactly_the_pairs_every_pair_checking_finds(
        events in any_events(),
    ) {
        let mut contact_tracker = ContactTracker::default();
        for event in events {
            match event {
                Event::Place { car, x, y, yaw } => {
                    contact_tracker.place_on_grid(&format!("car{car}"), placed(x, y, yaw));
                }
                Event::Leave { car } => contact_tracker.remove(&format!("car{car}")),
            }
            prop_assert_eq!(
                grid_pairs_in_contact(&contact_tracker),
                every_pair_in_contact(&contact_tracker)
            );

            // Every car the grid files is one it knows, in the cell it says.
            for (cell, cars) in &contact_tracker.grid {
                prop_assert!(!cars.is_empty());
                for car in cars {
                    prop_assert_eq!(contact_tracker.cars[car].cell, *cell);
                }
            }
            prop_assert_eq!(
                contact_tracker.grid.values().map(BTreeSet::len).sum::<usize>(),
                contact_tracker.cars.len()
            );
        }
    }
}
