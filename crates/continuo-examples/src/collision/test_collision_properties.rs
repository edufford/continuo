//! The grid's promise, checked against every pair: it finds exactly the
//! touching pairs that checking each pair of cars finds.

use std::collections::BTreeSet;

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

/// Events among a dozen cars in a 40 m square around the origin, which is a
/// few cells across in each direction, so cars cross cell edges, share
/// cells, and sit at negative coordinates.
fn any_events() -> impl Strategy<Value = Vec<Event>> {
    let place = (
        0u8..12,
        -20.0..20.0f64,
        -20.0..20.0f64,
        -std::f64::consts::PI..std::f64::consts::PI,
    )
        .prop_map(|(car, x, y, yaw)| Event::Place { car, x, y, yaw });
    let leave = (0u8..12).prop_map(|car| Event::Leave { car });
    proptest::collection::vec(prop_oneof![4 => place, 1 => leave], 1..80)
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
        }

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
