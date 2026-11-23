//! Property tests on `TrafficSpawner::fill_around_ego`: what holds for the
//! cars a run starts with under any sensible configuration, rather than
//! the demo's one.
//!
//! A configuration has one to five lanes of any width from 2.5 to 5 m,
//! with the ego in any of them, and gaps, retire distances and a headway factor drawn
//! independently. `retire_ahead` may come out shorter than the furthest
//! spawn, which the spawner increases to that spawn.

use std::sync::Arc;

use continuo_actors::{SpawnTrafficRequest, TrafficSpawner, TrafficSpawnerConfig, straight_road};
use continuo_core::{SimDuration, SimTime};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// How far a distance between two starts may fall short of the gap it was
/// built from. A start is the ego's position plus a gap, and subtracting
/// the position back out can round below the gap by a fraction of a
/// nanometer.
const TOLERANCE: f64 = 1e-9;

/// A configuration and the world seed to fill it with.
#[derive(Debug, Clone)]
struct Case {
    ego_start_s: f64,
    lane_width: f64,
    lane_offsets: Vec<f64>,
    ego_start_lateral: f64,
    target_population: usize,
    spawn_ahead: f64,
    retire_behind: f64,
    retire_ahead: f64,
    gap_range: (f64, f64),
    ego_headway_factor: f64,
    world_seed: u64,
}

impl Case {
    /// How far ahead of the ego a starting car can be, once the spawner
    /// has increased a `retire_ahead` short of the furthest spawn.
    fn reach_ahead(&self) -> f64 {
        self.retire_ahead.max(self.spawn_ahead + self.gap_range.1)
    }

    /// The cars the spawner starts this case's run with.
    fn starting_cars(&self) -> Vec<SpawnTrafficRequest> {
        let config = TrafficSpawnerConfig {
            period: SimDuration::from_millis(100),
            ego_name: "ego".to_string(),
            ego_start_s: self.ego_start_s,
            ego_start_lateral: self.ego_start_lateral,
            lane_offsets: self.lane_offsets.clone(),
            lane_tolerance: self.lane_width / 2.0,
            target_population: self.target_population,
            spawn_ahead: self.spawn_ahead,
            retire_behind: self.retire_behind,
            retire_ahead: self.retire_ahead,
            gap_range: self.gap_range,
            ego_headway_factor: self.ego_headway_factor,
            speed_range: (20.0, 30.0),
        };
        let road = Arc::new(straight_road(2000.0));

        // Return the fill for this case's seed.
        TrafficSpawner::new(road, config).fill_around_ego(self.world_seed)
    }

    /// Where the starting cars in the ego's lane begin, in order along the
    /// road.
    fn starts_in_ego_lane(&self) -> Vec<f64> {
        let mut starts: Vec<f64> = self
            .starting_cars()
            .iter()
            .filter(|spawn| spawn.lane_offset == self.ego_start_lateral)
            .map(|spawn| spawn.start_s)
            .collect();
        starts.sort_by(f64::total_cmp);

        // Return the starts in order along the road.
        starts
    }
}

prop_compose! {
    fn any_case()(
        lane_count in 1usize..=5,
        lane_width in 2.5..5.0f64,
        ego_lane_fraction in 0.0..1.0f64,
        ego_start_s in 200.0..1000.0f64,
        target_population in 1usize..=30,
        spawn_ahead in 0.0..100.0f64,
        retire_behind in 1.0..200.0f64,
        retire_ahead in 0.0..300.0f64,
        smallest_gap in 1.0..50.0f64,
        gap_spread in 0.0..50.0f64,
        ego_headway_factor in 1.0..4.0f64,
        world_seed in any::<u64>(),
    ) -> Case {
        let lane_offsets: Vec<f64> = (0..lane_count).map(|i| i as f64 * lane_width).collect();
        let ego_lane = (ego_lane_fraction * lane_count as f64) as usize;
        Case {
            ego_start_s,
            lane_width,
            ego_start_lateral: lane_offsets[ego_lane],
            lane_offsets,
            target_population,
            spawn_ahead,
            retire_behind,
            retire_ahead,
            gap_range: (smallest_gap, smallest_gap + gap_spread),
            ego_headway_factor,
            world_seed,
        }
    }
}

proptest! {
    // Each case is a few dozen draws, so a thousand of them cost far less
    // than a compile. A failure's seed lands beside this file as
    // `traffic_spawner_properties.proptest-regressions`, which is
    // committed and replayed before fresh cases are drawn. An integration
    // test has no crate root for the default location to be relative to.
    #![proptest_config(ProptestConfig {
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::with_cases(1024)
    })]

    #[test]
    fn the_starting_cars_are_due_at_zero_between_the_retire_distances(case in any_case()) {
        for spawn in case.starting_cars() {
            prop_assert_eq!(spawn.first_due, SimTime::ZERO);
            prop_assert!(spawn.start_s > case.ego_start_s - case.retire_behind);
            prop_assert!(spawn.start_s <= case.ego_start_s + case.reach_ahead());
        }
    }

    #[test]
    fn no_starting_car_is_within_the_smallest_gap_of_the_ego_in_its_lane(case in any_case()) {
        for start_s in case.starts_in_ego_lane() {
            prop_assert!((start_s - case.ego_start_s).abs() >= case.gap_range.0 - TOLERANCE);
        }
    }

    #[test]
    fn starting_cars_ahead_of_the_ego_in_its_lane_leave_the_longer_headway(
        case in any_case(),
    ) {
        let ahead: Vec<f64> = case
            .starts_in_ego_lane()
            .into_iter()
            .filter(|&start_s| start_s > case.ego_start_s)
            .collect();
        for pair in ahead.windows(2) {
            prop_assert!(
                pair[1] - pair[0] >= case.ego_headway_factor * case.gap_range.0 - TOLERANCE
            );
        }
    }
}
