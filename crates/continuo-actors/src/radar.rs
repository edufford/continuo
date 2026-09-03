//! A forward-looking radar and the scan it publishes.
//!
//! The sensor reads ground truth. It is handed every pose in the world
//! and uses the road's geometry to find the cars ahead of its own. It is
//! a simplified sensor model. A more realistic one would add a mounting
//! pose, extents on the cars it measures, noise, a field of view,
//! occlusion, several returns per vehicle that need clustering, and
//! tracking to give those returns identity between scans.

use std::collections::BTreeMap;
use std::sync::Arc;

use continuo_core::{
    Component, ComponentId, CoreError, Detection, KeyExpr, SimDuration, SimTime, StepCtx,
};
use serde::{Deserialize, Serialize};

use crate::path::Waypoints;
use crate::physics::CarState;
use crate::{CAR_LENGTH, MAX_DETECTIONS};

/// What one radar found in one scan.
///
/// The order is arbitrary and a slot carries no identity: the same car
/// can land in a different slot next scan. A detection is a measurement
/// taken this instant, not a tracked object, and nothing here tracks.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct RadarScan {
    /// Every car ahead in this lane and within range.
    pub detections: Vec<Detection>,
}

/// Reports the cars ahead of one car in its own lane, as a range and a
/// range rate each.
///
/// Lanes and "ahead" both come from the road. Every pose is projected
/// onto it with [`Waypoints::frenet`](crate::Waypoints::frenet()). A
/// car is in the same lane when its lateral offset is within
/// `lane_tolerance` of this car's, and ahead when its arc length is
/// larger. On a closed road the arc length wraps, so every car is ahead
/// of every other and `max_range` alone decides what is reported.
///
/// Range is bumper to bumper: the arc length between the two cars less
/// one [`CAR_LENGTH`]. That subtraction stands in for two things a real
/// sensor model would have, the radar's mounting position on its own
/// car and the extent of the car it measures to. It is right only while
/// every car is the same length and both are lined up along the road.
/// Doing it properly needs the simulation to publish extents, which is
/// PLAN.md's "World and map" work. A range below zero means the two
/// cars overlap and is reported as is, so a follower is not told the
/// road is clear.
///
/// Range rate is the other car's published speed minus this one's,
/// negative while closing, which is the sign a Doppler radar measures.
/// Reading it from the poses rather than differencing two sequential
/// scans means one sample per car is enough, so a car joining mid-run
/// is in the next scan.
///
/// The road's corners limit how exact a range can be. `frenet` projects
/// onto the nearest segment, and at a vertex the nearest segment
/// changes, which jumps a car's arc length by `2 * |d| * tan(theta / 2)`
/// for lateral offset `d` and turn angle `theta`. Two cars 30 m apart in
/// a lane 3.5 m out read up to 1.26 m off over the determinism ellipse's
/// 72 samples, and 0.29 m over 360. The straight-line distance would be
/// worse, since a chord across a bend is not the road a follower has to
/// cover. A road geometry that accounts for curvature is the deferred
/// fix.
///
/// The sensor keeps no state between steps. Each scan is built from the
/// poses in that step's inbox, so there is no freshness rule to tune and
/// nothing to clear when a car leaves. The cost is a bound rather than a
/// guarantee: a departed car can appear in one more scan, since its last
/// pose can still be in the next window, and a car must publish at
/// least once per radar period or it drops out of the scans between.
pub struct RadarSensor {
    actor_name: String,
    road: Arc<Waypoints>,
    period: SimDuration,
    max_range: f64,
    max_detections: usize,
    lane_tolerance: f64,
}

impl RadarSensor {
    /// A radar on `actor_name` that scans every `period`, sees out to
    /// `max_range` meters, and counts a car as sharing its lane when the
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

    /// Caps the scan below [`MAX_DETECTIONS`].
    ///
    /// The default is the constant a fixed-length consumer sizes its
    /// arrays by, so the two cannot drift apart. Lowering it with this
    /// function is for tests.
    pub fn with_max_detections(mut self, max_detections: usize) -> Self {
        self.max_detections = max_detections;
        self
    }

    /// The newest pose per actor in this step's inbox.
    ///
    /// The inbox is sorted by `(publisher, seq)`, so reading it forward
    /// and overwriting leaves each actor's latest. Two runs read the
    /// same order, which is what makes the scan deterministic.
    fn latest_poses(ctx: &StepCtx) -> Result<BTreeMap<String, CarState>, CoreError> {
        let mut latest = BTreeMap::new();
        for message in ctx.inbox() {
            // A pose that cannot be decoded stops the world.
            let state = message.decode::<CarState>()?;
            latest.insert(actor_of(message.key.as_str()).to_string(), state);
        }

        // Return the newest pose per actor.
        Ok(latest)
    }

    /// The scan `own` car sees of the other cars in `poses`.
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
            let distance_ahead = self.arc_ahead(own_s, other_s);
            if distance_ahead <= 0.0 {
                continue;
            }
            let range = distance_ahead - CAR_LENGTH;
            if range > self.max_range {
                continue;
            }
            detections.push(Detection {
                range,
                range_rate: other.speed - own.speed,
            });
        }
        self.cap(&mut detections);

        // Return what survives the cap, in the order the actors came in.
        RadarScan { detections }
    }

    /// How far ahead of `own_s` the arc length `other_s` is.
    ///
    /// On a loop the difference wraps, so a car just behind reads as
    /// most of a lap ahead. On an open road a car behind comes back
    /// negative.
    fn arc_ahead(&self, own_s: f64, other_s: f64) -> f64 {
        let difference = other_s - own_s;

        // Return the difference, wrapped onto a loop.
        if self.road.is_closed() {
            difference.rem_euclid(self.road.total_length())
        } else {
            difference
        }
    }

    /// Drops the farthest detections when more were found than fit.
    ///
    /// No road in the demo currently fills a scan, so this is a bound
    /// rather than a working limit. The farthest go because a follower
    /// needs the nearest. Only membership is decided here: what survives
    /// keeps the order it was found in, so nothing downstream can start
    /// reading a scan as pre-sorted.
    fn cap(&self, detections: &mut Vec<Detection>) {
        if detections.len() <= self.max_detections {
            return;
        }
        // `total_cmp` orders every pair of floats, and a stable sort
        // keeps equal ranges in the order the actors came in.
        let mut slots: Vec<usize> = (0..detections.len()).collect();
        slots.sort_by(|&a, &b| detections[a].range.total_cmp(&detections[b].range));
        let mut kept = vec![false; detections.len()];
        for &slot in &slots[..self.max_detections] {
            kept[slot] = true;
        }
        // `retain` visits the detections in their original order, so the
        // survivors keep it.
        let mut slot = 0;
        detections.retain(|_| {
            let keep = kept[slot];
            slot += 1;
            keep
        });
    }
}

/// The actor name in a pose key.
///
/// The name is the segment after `actor`, wherever that sits in the key.
fn actor_of(pose_key: &str) -> &str {
    pose_key
        .split('/')
        .skip_while(|segment| *segment != "actor")
        .nth(1)
        .expect("a pose key names the actor it belongs to")
}

impl Component for RadarSensor {
    fn id(&self) -> ComponentId {
        ComponentId::new("radar").expect("valid id")
    }

    fn subscriptions(&self) -> Vec<KeyExpr> {
        // Every actor's pose, since a sensor cannot know in advance what
        // it will find. The world segment is wildcarded for the reason
        // given in `PathFollowController`, and its TODO covers this too.
        vec![KeyExpr::new_rooted("*/actor/*/pose").expect("valid key")]
    }

    fn step(&mut self, ctx: &mut StepCtx) -> Result<SimTime, CoreError> {
        let poses = Self::latest_poses(ctx)?;
        // Nothing is published until this car's own pose has arrived. A
        // radar that does not know where it is measures nothing, and an
        // empty scan would say the road ahead is clear. The first step's
        // inbox is empty, so it always publishes nothing.
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

    /// The scan period, matching the controller's.
    const PERIOD: SimDuration = SimDuration::from_millis(100);

    /// One lane over.
    const LANE_WIDTH: f64 = 3.5;

    /// Half a lane, so a car one lane over is out and a car wandering
    /// inside its own is still in.
    const LANE_TOLERANCE: f64 = LANE_WIDTH / 2.0;

    /// Far enough that range currently rules nothing out, except in the
    /// test about range.
    const MAX_RANGE: f64 = 200.0;

    /// Half a kilometer of road along +x.
    fn road() -> Arc<Waypoints> {
        Arc::new(Waypoints::build_straight((0.0, 0.0), (500.0, 0.0)))
    }

    /// A radar on the ego car, which every test scans from.
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
    /// The inbox arrives in publisher order, so the callers list their
    /// cars by name.
    fn poses(cars: &[(&str, CarState)]) -> Vec<Message> {
        // Return the inbox a step would be handed.
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

    /// Steps `radar` once and returns what it published, if anything.
    fn step_once(radar: &mut RadarSensor, now: SimTime, inbox: Vec<Message>) -> Option<RadarScan> {
        let mut ctx = StepCtx::new(now, Some(PERIOD), "w", 0, inbox);
        radar
            .step(&mut ctx)
            .expect("a radar given readable poses steps");
        let mut outbox = ctx.take_outbox();
        assert!(outbox.len() <= 1, "a step publishes at most one scan");

        // Return the scan decoded from its published bytes.
        outbox.pop().map(|(key, payload)| {
            assert_eq!(key.as_str(), "continuo/w/actor/ego/radar");
            serde_json::from_slice(&payload).expect("a scan")
        })
    }

    /// What a radar on `road` sees of `cars`.
    fn scan_of(road: Arc<Waypoints>, cars: &[(&str, CarState)]) -> RadarScan {
        // Return the scan, which exists whenever the ego's pose is in the
        // inbox.
        step_once(&mut radar_on(road), SimTime::ZERO, poses(cars))
            .expect("the ego published its own pose, so it scanned")
    }

    /// The ranges in a scan, sorted, since a scan's order is not part of
    /// its contract.
    fn ranges(scan: &RadarScan) -> Vec<f64> {
        let mut ranges: Vec<f64> = scan.detections.iter().map(|found| found.range).collect();
        ranges.sort_by(f64::total_cmp);

        // Return them sorted for comparison.
        ranges
    }

    /// The one detection in a scan that should hold exactly one.
    fn only(scan: &RadarScan) -> Detection {
        assert_eq!(scan.detections.len(), 1, "expected one detection");

        // Return it.
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

        // The three cars ahead in lane, each once. Compared sorted, since
        // slot order is not part of the contract.
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

        // The same car half a lane over is within the tolerance.
        let inside = scan_of(
            road(),
            &[
                ("ego", car(0.0, 0.0, 20.0)),
                ("other", car(40.0, LANE_TOLERANCE, 20.0)),
            ],
        );
        assert_eq!(ranges(&inside), vec![40.0 - CAR_LENGTH]);

        // The band is centered on this car, not the road, so a radar in
        // an outside lane watches that lane.
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
        // A car alongside is not ahead either. The same rule keeps a car
        // from detecting itself.
        let scan = scan_of(
            road(),
            &[
                ("alongside", car(50.0, 0.0, 20.0)),
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
        // Fifty meters between the two origins, so a car length less
        // between the bumpers, and a lead 12 m/s slower.
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
        // Closer than a car length means the cars overlap. The range goes
        // negative rather than flooring at zero, so a follower is not
        // told the road is clear.
        let scan = scan_of(
            road(),
            &[("ego", car(0.0, 0.0, 20.0)), ("hit", car(3.0, 0.0, 20.0))],
        );
        assert_eq!(only(&scan).range, 3.0 - CAR_LENGTH);
    }

    #[test]
    fn a_car_beyond_the_radars_range_is_not_detected() {
        // The cut is on the bumper-to-bumper range, so the car exactly a
        // car length past `MAX_RANGE` is the last one in.
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

        // The two nearest survive and the farthest is dropped.
        assert_eq!(ranges(&scan), vec![50.0 - CAR_LENGTH, 100.0 - CAR_LENGTH]);
    }

    #[test]
    fn a_departed_car_vanishes_from_the_scan_after_its_last_pose() {
        let mut radar = radar_on(road());
        let both = poses(&[("ego", car(0.0, 0.0, 20.0)), ("gone", car(40.0, 0.0, 20.0))]);
        let scan = step_once(&mut radar, SimTime::ZERO, both).expect("the ego scanned");
        assert_eq!(ranges(&scan), vec![40.0 - CAR_LENGTH]);

        // The next inbox has no pose from it, and nothing was kept, so
        // it is gone.
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
        // On a ring the car behind is most of a lap ahead, so range rules
        // it out rather than the sign of the difference.
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
        // Approximate because the ring is a polygon, so a point placed at
        // an arc length projects back a few bits away.
        assert!(
            (only(&scan).range - (30.0 - CAR_LENGTH)).abs() < 1e-9,
            "across the seam: {:?}",
            scan.detections
        );
    }

    #[test]
    fn the_first_step_publishes_nothing_rather_than_guessing() {
        // The first step's inbox is empty, so the radar does not know
        // where its own car is.
        let mut radar = radar_on(road());
        let mut ctx = StepCtx::new(SimTime::ZERO, None, "w", 0, vec![]);
        radar
            .step(&mut ctx)
            .expect("a radar with nothing to read steps");
        assert!(
            ctx.take_outbox().is_empty(),
            "an empty scan would have said the road ahead was clear"
        );

        // The ego's own pose is what decides, not whether the inbox is
        // empty: an inbox with everyone else's pose still leaves the
        // radar without its own position.
        let others = poses(&[("lead", car(60.0, 0.0, 20.0))]);
        assert!(step_once(&mut radar, SimTime::from_millis(100), others).is_none());
    }
}
