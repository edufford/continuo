//! A forward-looking radar, and what it reports.
//!
//! The sensor reads ground truth: it is handed every pose in the world
//! and works out what is ahead of its own car from the road's geometry.
//! That is the simplest thing producing the quantities a follower wants,
//! and it is deliberately not a sensor model. A real one would add a
//! mounting pose on the car carrying it, extents on the things it
//! measures to, noise, a field of view, occlusion, several returns per
//! vehicle needing clustering before anything is an object at all, and
//! tracking before those objects have identity between scans.
//!
//! **One detection per car is itself the idealization**, not only the
//! values inside it. What has to survive a real sensor model is the
//! interface rather than the arithmetic: a scan carries relative
//! measurements and nothing else, because a radar knows nothing about
//! the car it is bolted to.

use std::collections::BTreeMap;
use std::sync::Arc;

use continuo_core::{
    Component, ComponentId, CoreError, Detection, KeyExpr, SimDuration, SimTime, StepCtx,
};
use serde::{Deserialize, Serialize};

use crate::path::Waypoints;
use crate::physics::CarState;
use crate::{CAR_LENGTH, MAX_DETECTIONS};

/// What one radar found this scan.
///
/// **The order means nothing**, and neither does a slot: the same car
/// can arrive in a different place next scan, and nothing says it is the
/// same car. That is what a detection is, a measurement taken this
/// instant rather than an object being tracked, and identity would have
/// to come from a tracker that does not exist here.
///
/// No consumer wants one anyway. A following law takes the nearest
/// detection, and a learned one either sorts the set itself or encodes
/// it so the order cannot matter. Determinism needs only that two runs
/// build the same scan, which comes from reading the inbox in its own
/// order rather than from sorting on a float.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct RadarScan {
    /// What is ahead, nearest first only by accident.
    pub detections: Vec<Detection>,
}

/// Reports what is ahead of one car in its own lane, as a range and a
/// closing rate per car found.
///
/// Lanes and "ahead" both come from the road: every pose is projected
/// onto it, a car counts as in the same lane when its lateral offset is
/// within `lane_tolerance` of this one's, and it is ahead when its arc
/// length is larger. A closed road wraps, so on a loop everything is
/// ahead of everything and `max_range` alone decides what is reported.
///
/// **Range is bumper to bumper**, the arc length between the two cars
/// less one [`CAR_LENGTH`]. That single subtraction stands in for two
/// things a sensor model owes: a radar sits somewhere on the car
/// carrying it rather than at its origin, and it measures to where the
/// line between them meets the other car's body rather than to that
/// car's origin. Collapsing both into one constant is right only while
/// every car is the same length, the radar sits at the front bumper, and
/// the two are lined up along the road. Doing it properly needs the
/// simulation to publish extents, which is PLAN.md's "World and map"
/// work. A range below zero is two cars overlapping, and it is reported
/// as it stands, since a follower being told it has run into something
/// is the answer that deserves.
///
/// **Range rate is the other car's published speed less this one's**,
/// negative while closing, which is the sign a Doppler radar measures
/// directly. Reading it off the poses rather than differencing two scans
/// is what keeps the sensor rate independent: one sample per car is
/// enough, so a car joining mid-run is in the very next scan.
///
/// It keeps nothing between steps. Each scan is built from the poses in
/// that step's inbox and nothing else, so there is no freshness rule to
/// tune and no per-actor state to clear out when a car leaves, which a
/// scan spanning steps would need. What that costs is a bound rather
/// than a guarantee, and it is worth naming: a departed car ghosts for
/// at most one scan, since its last pose can still be in the window read
/// after it left. It also means **anything being watched has to publish
/// at least once per radar period**, or it blinks in and out of the
/// scans.
pub struct RadarSensor {
    actor_name: String,
    road: Arc<Waypoints>,
    period: SimDuration,
    max_range: f64,
    max_detections: usize,
    lane_tolerance: f64,
}

impl RadarSensor {
    /// A radar on `actor_name`, scanning every `period` out to
    /// `max_range` meters, counting a car as sharing its lane when the
    /// two lateral offsets are within `lane_tolerance`.
    pub fn new(
        actor_name: impl Into<String>,
        road: Arc<Waypoints>,
        period: SimDuration,
        max_range: f64,
        lane_tolerance: f64,
    ) -> Self {
        RadarSensor {
            actor_name: actor_name.into(),
            road,
            period,
            max_range,
            max_detections: MAX_DETECTIONS,
            lane_tolerance,
        }
    }

    /// Caps the scan at something other than [`MAX_DETECTIONS`].
    ///
    /// The default is the constant a fixed-length consumer builds its
    /// arrays from, so the two cannot drift apart. Lowering it is for
    /// tests, and for a sensor whose returns are genuinely fewer.
    pub fn with_max_detections(mut self, max_detections: usize) -> Self {
        self.max_detections = max_detections;
        self
    }

    /// The newest pose per actor in this step's inbox.
    ///
    /// The inbox is `(publisher, seq)`-sorted, so reading it forwards and
    /// overwriting leaves each actor's latest. Two runs read the same
    /// order, which is where the scan's determinism comes from.
    fn latest_poses(ctx: &StepCtx) -> Result<BTreeMap<String, CarState>, CoreError> {
        let mut latest = BTreeMap::new();
        for message in ctx.inbox() {
            // A pose that cannot be read stops the world. Scanning past
            // it would report a road with one fewer car on it, and a
            // follower would accelerate into the space it left.
            let state = message.decode::<CarState>()?;
            latest.insert(actor_of(message.key.as_str()).to_string(), state);
        }

        // Return one pose per actor, the newest each published.
        Ok(latest)
    }

    /// The scan `own` sees of everything else in `poses`.
    fn scan(&self, own: &CarState, poses: &BTreeMap<String, CarState>) -> RadarScan {
        let (own_s, own_lateral) = self.road.frenet(own.position.x, own.position.y);
        let mut detections = Vec::new();
        for (name, other) in poses {
            if name == &self.actor_name {
                continue;
            }
            let (other_s, other_lateral) = self.road.frenet(other.position.x, other.position.y);
            if (other_lateral - own_lateral).abs() > self.lane_tolerance {
                continue;
            }
            let along = self.arc_ahead(own_s, other_s);
            if along <= 0.0 {
                continue;
            }
            let range = along - CAR_LENGTH;
            if range > self.max_range {
                continue;
            }
            detections.push(Detection {
                range,
                range_rate: other.speed - own.speed,
            });
        }
        self.cap(&mut detections);

        // Return what that leaves, in the order the actors came in.
        RadarScan { detections }
    }

    /// How far ahead of `own_s` the arc length `other_s` is, following
    /// the road's own idea of what comes after what.
    ///
    /// A loop wraps, so a car just behind is most of a lap ahead and
    /// `max_range` is what rules it out. A road does not, so a car
    /// behind stays behind and comes back negative.
    fn arc_ahead(&self, own_s: f64, other_s: f64) -> f64 {
        let along = other_s - own_s;

        // Return it brought back onto the road, which only a loop can do.
        if self.road.is_closed() {
            along.rem_euclid(self.road.total_length())
        } else {
            along
        }
    }

    /// Drops whatever will not fit in a scan.
    ///
    /// A road here never fills one, so this is a bound rather than a
    /// working limit. Which detections go still has to be decided rather
    /// than left to whichever happened to be found first: the farthest
    /// go, because a follower needs the nearest and would rather lose a
    /// car it was never going to follow. Membership is all this decides.
    /// What survives keeps the order it was found in, so nothing
    /// downstream can start reading a scan as though it were sorted.
    fn cap(&self, detections: &mut Vec<Detection>) {
        if detections.len() <= self.max_detections {
            return;
        }
        let mut slots: Vec<usize> = (0..detections.len()).collect();
        // `total_cmp` because it orders every pair of floats there is,
        // and a stable sort then leaves equal ranges in the order the
        // actors came in.
        slots.sort_by(|&a, &b| detections[a].range.total_cmp(&detections[b].range));
        slots.truncate(self.max_detections);
        slots.sort_unstable();
        let kept = slots.into_iter().map(|slot| detections[slot]).collect();
        *detections = kept;
    }
}

/// The actor a pose key belongs to.
///
/// The subscription delivers `.../actor/{name}/pose` and nothing else,
/// so the segment before the last one is the name.
fn actor_of(key: &str) -> &str {
    key.rsplit('/')
        .nth(1)
        .expect("a pose key names the actor it belongs to")
}

impl Component for RadarSensor {
    fn id(&self) -> ComponentId {
        ComponentId::new("radar").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        // Every actor's pose, since a sensor cannot know in advance what
        // it will find. World segment wildcarded for the reason
        // `PathFollowController` gives, and its TODO covers this one too.
        vec![KeyExpr::new_rooted("*/actor/*/pose").expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        let poses = Self::latest_poses(ctx)?;
        // Nothing goes out until this car's own pose has arrived. A radar
        // that does not know where it is measures nothing, where an empty
        // scan would say the road ahead is clear. On the first step that
        // is the whole of it, the inbox starting empty.
        if let Some(own) = poses.get(&self.actor_name) {
            let scan = self.scan(own, &poses);
            ctx.publish(crate::radar_key(ctx.world_name(), &self.actor_name), &scan)?;
        }

        // Return the next due time, one scan period from now.
        Ok(ctx.now() + self.period)
    }
}

#[cfg(test)]
mod tests {
    use continuo_core::{ComponentPath, Message, Quat, Vec3};

    use super::*;

    /// The scan period, which is the period a follower controls at.
    const PERIOD: SimDuration = SimDuration::from_millis(100);

    /// Half a lane, so a car one lane over is out and a car wandering
    /// inside its own is still in.
    const LANE_TOLERANCE: f64 = 1.75;

    /// One lane over.
    const LANE_WIDTH: f64 = 3.5;

    /// Far enough that nothing here is ruled out by range, except in the
    /// test that is about range.
    const MAX_RANGE: f64 = 200.0;

    /// Half a kilometer of road along +x.
    fn road() -> Arc<Waypoints> {
        Arc::new(Waypoints::build_straight((0.0, 0.0), (500.0, 0.0)))
    }

    /// A radar on the car every test here scans from.
    fn radar_on(road: Arc<Waypoints>) -> RadarSensor {
        RadarSensor::new("ego", road, PERIOD, MAX_RANGE, LANE_TOLERANCE)
    }

    /// A car `x` meters along the road, `lateral` meters left of it,
    /// doing `speed`.
    fn car(x: f64, lateral: f64, speed: f64) -> CarState {
        CarState {
            position: Vec3::new(x, lateral, 0.0),
            orientation: Quat::from_yaw(0.0),
            speed,
        }
    }

    /// One pose message per car, as the transport would deliver them.
    ///
    /// Each car publishes its own, so the inbox arrives in name order and
    /// the callers below list their cars that way.
    fn poses(cars: &[(&str, CarState)]) -> Vec<Message> {
        // Return the window a step would be handed.
        cars.iter()
            .map(|(name, state)| Message {
                key: KeyExpr::new_rooted(format!("w/actor/{name}/pose")).expect("valid key"),
                publisher: ComponentPath::parse(&format!("{name}/physics")).expect("valid path"),
                seq: 0,
                sim_time: SimTime::ZERO,
                payload: serde_json::to_vec(state).expect("a car state serializes"),
            })
            .collect()
    }

    /// Steps `radar` once and hands back what it published, if anything.
    fn step_once(radar: &mut RadarSensor, now: SimTime, inbox: Vec<Message>) -> Option<RadarScan> {
        let mut ctx = StepCtx::new(now, Some(PERIOD), "w", 0, inbox);
        radar
            .step(&mut ctx)
            .expect("a radar given readable poses steps");
        let mut outbox = ctx.take_outbox();
        assert!(outbox.len() <= 1, "a step publishes at most one scan");

        // Return the scan as it went out, read back from its own bytes.
        outbox.pop().map(|(key, payload)| {
            assert_eq!(key.as_str(), "continuo/w/actor/ego/radar");
            serde_json::from_slice(&payload).expect("a scan")
        })
    }

    /// What a radar on `road` sees of `cars`.
    fn scan_of(road: Arc<Waypoints>, cars: &[(&str, CarState)]) -> RadarScan {
        // Return the scan, which there always is once the ego's own pose
        // is in the window.
        step_once(&mut radar_on(road), SimTime::ZERO, poses(cars))
            .expect("the ego published its own pose, so it scanned")
    }

    /// The ranges in a scan, nearest first, since the order a scan comes
    /// in is not part of what it promises.
    fn ranges(scan: &RadarScan) -> Vec<f64> {
        let mut ranges: Vec<f64> = scan.detections.iter().map(|found| found.range).collect();
        ranges.sort_by(f64::total_cmp);

        // Return them ordered, for a test to compare against.
        ranges
    }

    /// The one detection in a scan that should hold exactly one.
    fn only(scan: &RadarScan) -> Detection {
        assert_eq!(scan.detections.len(), 1, "expected one detection");

        // Return it, for a test checking what is inside it.
        scan.detections[0]
    }

    #[test]
    fn a_scan_reports_every_car_ahead_in_lane_exactly_once() {
        let scan = scan_of(
            road(),
            &[
                ("behind", car(30.0, 0.0, 20.0)),
                ("ego", car(50.0, 0.0, 20.0)),
                ("far", car(160.0, 0.0, 20.0)),
                ("mid", car(110.0, 0.0, 20.0)),
                ("near", car(70.0, 0.0, 20.0)),
                ("wide", car(90.0, LANE_WIDTH, 20.0)),
            ],
        );

        // The three ahead in lane and nothing twice, compared as a set,
        // because which slot a car lands in promises nothing.
        assert_eq!(
            ranges(&scan),
            vec![20.0 - CAR_LENGTH, 60.0 - CAR_LENGTH, 110.0 - CAR_LENGTH]
        );
    }

    #[test]
    fn a_car_in_another_lane_is_not_detected() {
        let over = scan_of(
            road(),
            &[
                ("ego", car(0.0, 0.0, 20.0)),
                ("other", car(40.0, LANE_WIDTH, 20.0)),
            ],
        );
        assert!(
            over.detections.is_empty(),
            "a lane over: {:?}",
            over.detections
        );

        // The tolerance is what says so, and the same car half a lane
        // over is a car wandering inside its own.
        let inside = scan_of(
            road(),
            &[
                ("ego", car(0.0, 0.0, 20.0)),
                ("other", car(40.0, LANE_TOLERANCE, 20.0)),
            ],
        );
        assert_eq!(ranges(&inside), vec![40.0 - CAR_LENGTH]);

        // The band sits around this car rather than around the road, so
        // a radar in an outside lane watches that lane and not the road.
        let outside = scan_of(
            road(),
            &[
                ("ego", car(0.0, LANE_WIDTH, 20.0)),
                ("middle", car(60.0, 0.0, 20.0)),
                ("same", car(40.0, LANE_WIDTH, 20.0)),
            ],
        );
        assert_eq!(ranges(&outside), vec![40.0 - CAR_LENGTH]);
    }

    #[test]
    fn cars_behind_are_not_detected() {
        // A car abreast is not ahead either, which is also what keeps a
        // car from finding itself when another shares its arc length.
        let scan = scan_of(
            road(),
            &[
                ("abreast", car(50.0, 0.0, 20.0)),
                ("behind", car(10.0, 0.0, 20.0)),
                ("ego", car(50.0, 0.0, 20.0)),
            ],
        );
        assert!(
            scan.detections.is_empty(),
            "nothing is ahead: {:?}",
            scan.detections
        );
    }

    #[test]
    fn range_rate_is_negative_when_closing_and_positive_when_opening() {
        let closing = scan_of(
            road(),
            &[("ego", car(0.0, 0.0, 25.0)), ("lead", car(40.0, 0.0, 18.0))],
        );
        assert_eq!(only(&closing).range_rate, -7.0);

        let opening = scan_of(
            road(),
            &[("ego", car(0.0, 0.0, 25.0)), ("lead", car(40.0, 0.0, 31.0))],
        );
        assert_eq!(only(&opening).range_rate, 6.0);
    }

    #[test]
    fn a_lead_holding_a_steady_gap_reports_zero_range_rate() {
        let scan = scan_of(
            road(),
            &[("ego", car(0.0, 0.0, 22.5)), ("lead", car(40.0, 0.0, 22.5))],
        );
        assert_eq!(only(&scan).range_rate, 0.0);
    }

    #[test]
    fn range_and_range_rate_match_the_known_geometry_of_a_staged_pair() {
        // Fifty meters of road between the two origins, so a car length
        // less than that between the bumpers, and a lead twelve slower.
        let scan = scan_of(
            road(),
            &[
                ("ego", car(10.0, 0.0, 30.0)),
                ("lead", car(60.0, 0.0, 18.0)),
            ],
        );
        assert_eq!(
            only(&scan),
            Detection {
                range: 50.0 - CAR_LENGTH,
                range_rate: -12.0,
            }
        );
    }

    #[test]
    fn an_overlapping_car_reports_a_negative_range() {
        // Nearer than a car length is two cars in the same place. The
        // scan says so rather than flooring at zero, since a follower
        // told the road ahead was clear would drive further into it.
        let scan = scan_of(
            road(),
            &[("ego", car(0.0, 0.0, 20.0)), ("hit", car(3.0, 0.0, 20.0))],
        );
        assert_eq!(only(&scan).range, 3.0 - CAR_LENGTH);
    }

    #[test]
    fn a_car_beyond_the_radars_range_is_not_detected() {
        // Range is what the radar measures, so the cut is bumper to
        // bumper: the car a car length past the limit is the last one in.
        let scan = scan_of(
            road(),
            &[
                ("edge", car(MAX_RANGE + CAR_LENGTH, 0.0, 20.0)),
                ("ego", car(0.0, 0.0, 20.0)),
                ("past", car(MAX_RANGE + CAR_LENGTH + 0.5, 0.0, 20.0)),
            ],
        );
        assert_eq!(ranges(&scan), vec![MAX_RANGE]);
    }

    #[test]
    fn a_scan_over_its_cap_keeps_the_nearest() {
        let mut radar = radar_on(road()).with_max_detections(2);
        let cars = poses(&[
            ("ego", car(0.0, 0.0, 20.0)),
            ("far", car(150.0, 0.0, 20.0)),
            ("mid", car(100.0, 0.0, 20.0)),
            ("near", car(50.0, 0.0, 20.0)),
        ]);
        let scan = step_once(&mut radar, SimTime::ZERO, cars).expect("the ego scanned");

        // The two nearest, and the farthest is what went.
        assert_eq!(ranges(&scan), vec![50.0 - CAR_LENGTH, 100.0 - CAR_LENGTH]);
    }

    #[test]
    fn a_departed_car_vanishes_from_the_scan_after_its_last_pose() {
        let mut radar = radar_on(road());
        let both = poses(&[("ego", car(0.0, 0.0, 20.0)), ("gone", car(40.0, 0.0, 20.0))]);
        let scan = step_once(&mut radar, SimTime::ZERO, both).expect("the ego scanned");
        assert_eq!(ranges(&scan), vec![40.0 - CAR_LENGTH]);

        // The next window carries no pose from it, and that is the whole
        // of the cleanup: nothing was kept, so nothing has to be dropped.
        let alone = poses(&[("ego", car(2.0, 0.0, 20.0))]);
        let scan =
            step_once(&mut radar, SimTime::from_millis(100), alone).expect("the ego scanned");
        assert!(
            scan.detections.is_empty(),
            "the departed car came back: {:?}",
            scan.detections
        );
    }

    #[test]
    fn a_loop_scans_across_its_own_seam() {
        // On a ring the car behind is the car most of a lap ahead, so
        // range is what rules it out rather than the sign of a
        // subtraction. A road would answer both the other way round.
        let ring = Arc::new(Waypoints::ellipse((0.0, 0.0), 40.0, 40.0, 360));
        let total = ring.total_length();
        let at = |s: f64, speed: f64| {
            let s = s.rem_euclid(total);
            CarState {
                position: ring.point_at(s),
                orientation: Quat::from_yaw(ring.heading_at(s)),
                speed,
            }
        };
        let mut radar = RadarSensor::new("ego", ring.clone(), PERIOD, 60.0, LANE_TOLERANCE);
        let cars = poses(&[
            ("behind", at(total - 60.0, 20.0)),
            ("ego", at(total - 10.0, 20.0)),
            ("seam", at(20.0, 20.0)),
        ]);
        let scan = step_once(&mut radar, SimTime::ZERO, cars).expect("the ego scanned");

        // Thirty meters of ring between them, ten of it past the seam.
        // Approximate because a ring here is a polygon, so a point put on
        // it at an arc length comes back off it a few bits away.
        assert!(
            (only(&scan).range - (30.0 - CAR_LENGTH)).abs() < 1e-9,
            "across the seam: {:?}",
            scan.detections
        );
    }

    #[test]
    fn the_first_step_publishes_nothing_rather_than_guessing() {
        // A first step is handed an empty inbox, so the radar does not
        // know where its own car is, let alone anyone else's.
        let mut radar = radar_on(road());
        let mut ctx = StepCtx::new(SimTime::ZERO, None, "w", 0, vec![]);
        radar
            .step(&mut ctx)
            .expect("a radar with nothing to read steps");
        assert!(
            ctx.take_outbox().is_empty(),
            "an empty scan would have said the road ahead was clear"
        );

        // It is the ego's own pose that decides rather than the inbox
        // being empty: a window carrying everyone else is still a radar
        // with no idea where it is.
        let others = poses(&[("lead", car(60.0, 0.0, 20.0))]);
        assert!(step_once(&mut radar, SimTime::from_millis(100), others).is_none());
    }
}
