//! Property tests on `Waypoints::frenet`: what holds on any road, rather
//! than on the handful of fixtures the unit tests in `path.rs` draw by
//! hand.
//!
//! Every road here keeps its segments a road's width clear of one
//! another except where they meet, since two segments closer than that
//! share lane space, and a lane point beside one is then beside the
//! other too. An open road is a random walk, turning as sharply as a
//! road can, and a closed one is a star-shaped polygon around a center.
//! Both lie anywhere in the plane facing any way, so no property passes
//! by leaning on an axis.

use std::f64::consts::TAU;

use continuo_actors::Waypoints;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// How close two answers must be to count as the same.
///
/// Coordinates here run to a thousand meters or so, where a rounding
/// error is around 1e-13, and no distance a car cares about is anywhere
/// near a nanometer.
const TOLERANCE: f64 = 1e-9;

/// The width of a lane: three and a half meters, a standard lane and
/// what the unit tests place one at.
const LANE_WIDTH: f64 = 3.5;

/// How close two segments of one road that do not meet may come: three
/// lane widths, which is what a road carrying a lane each side of its
/// centerline spans. Closer than that they would share lane space, and
/// a lane point beside one would be beside the other too, so nothing
/// said here about the nearest segment could hold.
const CLEARANCE: f64 = 3.0 * LANE_WIDTH;

/// How far from the road a point still counts as beside it: half the
/// clearance, the far edge of the lane each side. Any two segments both
/// within that of one point are within the clearance of each other, so
/// they meet at a vertex, and two segments that meet agree on which side
/// of the road the point is.
const BESIDE_THE_ROAD: f64 = CLEARANCE / 2.0;

/// The sharpest turn a road makes at a vertex: 150 degrees, so the two
/// segments meeting there diverge by at least 30 and neither runs back
/// along the other.
const SHARPEST_TURN: f64 = TAU * 150.0 / 360.0;

/// The distance from `(x, y)` to the nearest point of the segment from
/// `a` to `b`, which is one of its ends when the point lies past it.
fn distance_to_segment(a: (f64, f64), b: (f64, f64), x: f64, y: f64) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let along = (((x - a.0) * dx + (y - a.1) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
    let (px, py) = (a.0 + dx * along, a.1 + dy * along);

    // Return the distance to that nearest point.
    ((x - px).powi(2) + (y - py).powi(2)).sqrt()
}

/// Whether the segment from `a` to `b` crosses the one from `c` to `d`.
fn segments_cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let side = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| {
        (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0)
    };

    // Return whether each segment's ends lie on opposite sides of the
    // other's line, which is what crossing means.
    side(a, b, c) * side(a, b, d) < 0.0 && side(c, d, a) * side(c, d, b) < 0.0
}

/// Whether every pair of segments that do not share a vertex stays the
/// clearance apart. Two segments that do not cross are nearest at one
/// end of one of them, so four point distances cover it.
fn keeps_clearance(points: &[(f64, f64)], is_closed: bool) -> bool {
    let segments = if is_closed {
        points.len()
    } else {
        points.len() - 1
    };
    let ends = |i: usize| (points[i], points[(i + 1) % points.len()]);
    let shares_a_vertex =
        |i: usize, j: usize| j == i + 1 || (is_closed && i == 0 && j == segments - 1);

    // Return whether no pair comes too close.
    (0..segments).all(|i| {
        (i + 1..segments).all(|j| {
            let ((a, b), (c, d)) = (ends(i), ends(j));
            shares_a_vertex(i, j)
                || (!segments_cross(a, b, c, d)
                    && [(a, c, d), (b, c, d), (c, a, b), (d, a, b)]
                        .into_iter()
                        .all(|(p, from, to)| distance_to_segment(from, to, p.0, p.1) >= CLEARANCE))
        })
    })
}

/// The distance from `(x, y)` to each segment of `road`, in road order.
///
/// Worked out from the points on their own, so it shares nothing with
/// `frenet` beyond the geometry and stands as the oracle for its offset.
fn distance_to_each_segment(road: &Waypoints, x: f64, y: f64) -> Vec<f64> {
    let points = road.points();

    // Return one distance per segment.
    (0..road.num_segments())
        .map(|i| distance_to_segment(points[i], points[(i + 1) % points.len()], x, y))
        .collect()
}

/// The nearest segment of `road` to `(x, y)`, as its index and its
/// distance, the earliest of any that tie.
fn nearest_segment(road: &Waypoints, x: f64, y: f64) -> (usize, f64) {
    // Return the first segment no other beats.
    distance_to_each_segment(road, x, y)
        .into_iter()
        .enumerate()
        .fold((0, f64::INFINITY), |best, (i, distance)| {
            if distance < best.1 {
                (i, distance)
            } else {
                best
            }
        })
}

/// The widest road a lane is placed on: ten lanes. A segment with
/// nothing else near it has room beyond any lane, and this is where the
/// tests stop reaching out.
const WIDEST_ROAD: f64 = 10.0 * LANE_WIDTH;

/// How far a lane can sit off the point `(x, y)` of segment
/// `own_segment` before some other segment is nearer than that one: half
/// the distance to the nearest other segment, since moving a point by a
/// distance brings it at most that much closer to anything, and never
/// more than the widest road. A point on a vertex is on two segments at
/// once and has no room at all.
fn room_for_a_lane(road: &Waypoints, x: f64, y: f64, own_segment: usize) -> f64 {
    // The smallest of the other segments' distances, or infinity when
    // there is no other segment, which the cap below then answers.
    let nearest_other = distance_to_each_segment(road, x, y)
        .into_iter()
        .enumerate()
        .filter(|&(i, _)| i != own_segment)
        .map(|(_, distance)| distance)
        .fold(f64::INFINITY, f64::min);

    // Return half of it, and no more than a lane is ever placed at.
    (nearest_other / 2.0).min(WIDEST_ROAD)
}

/// The distance from `(x, y)` to the line through `from` and `to`, rather
/// than to the segment between them.
fn distance_to_line(from: (f64, f64), to: (f64, f64), x: f64, y: f64) -> f64 {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);

    // Return the perpendicular distance, the cross product over the
    // length.
    ((dx * (y - from.1) - dy * (x - from.0)) / (dx * dx + dy * dy).sqrt()).abs()
}

/// Which stretch an open road's arc length falls on: the road itself,
/// or the line its end segment holds before the start or past the end.
/// A closed road has no ends and is all one stretch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stretch {
    BeforeTheStart,
    OnTheRoad,
    PastTheEnd,
}

fn stretch(road: &Waypoints, s: f64) -> Stretch {
    let stretch = if road.is_closed() || (0.0..=road.total_length()).contains(&s) {
        Stretch::OnTheRoad
    } else if s < 0.0 {
        Stretch::BeforeTheStart
    } else {
        Stretch::PastTheEnd
    };

    // Return which stretch the arc length falls on.
    stretch
}

/// A rotation and a shift, taking a road drawn against the axes anywhere
/// in the plane facing any way.
#[derive(Debug, Clone, Copy)]
struct Placement {
    angle: f64,
    shift: (f64, f64),
}

impl Placement {
    fn apply(&self, (x, y): (f64, f64)) -> (f64, f64) {
        let (sin, cos) = (libm::sin(self.angle), libm::cos(self.angle));

        // Return the point rotated about the origin and then shifted.
        (
            cos * x - sin * y + self.shift.0,
            sin * x + cos * y + self.shift.1,
        )
    }
}

fn placement() -> impl Strategy<Value = Placement> {
    (0.0..TAU, -500.0..500.0, -500.0..500.0).prop_map(|(angle, dx, dy)| Placement {
        angle,
        shift: (dx, dy),
    })
}

/// A point anywhere in a square three kilometers on a side around the
/// origin. Every road here starts within half a kilometer of the origin
/// and runs for under a kilometer, so the square holds all of them with
/// room to spare, and a point drawn from it lands on the road, beside
/// it, or far from it in whatever proportion the road's size gives.
fn anywhere() -> impl Strategy<Value = (f64, f64)> {
    (-1500.0..1500.0, -1500.0..1500.0)
}

/// An open road of 2 to 8 points: a random walk from anywhere, facing
/// any way, in strides of a meter to a hundred, turning at each vertex
/// by up to the sharpest turn either way. It can turn back on itself so
/// long as its segments keep the clearance, which the filter holds it
/// to. Its strides keep every segment far longer than the road's
/// minimum, and the refusal check guards the builder all the same: a
/// road it would refuse is a case to discard rather than a panic to
/// report.
///
/// Every range here excludes what a float can otherwise be, since a
/// road with a NaN in it is not a road.
fn open_road() -> impl Strategy<Value = Waypoints> {
    (2..=8usize)
        .prop_flat_map(|n| {
            (
                (-500.0..500.0, -500.0..500.0),
                0.0..TAU,
                proptest::collection::vec(-SHARPEST_TURN..SHARPEST_TURN, n - 2),
                proptest::collection::vec(1.0..100.0f64, n - 1),
            )
        })
        .prop_map(|(start, first_heading, turns, strides)| {
            let mut heading = first_heading;
            let mut points = vec![start];
            for (i, stride) in strides.into_iter().enumerate() {
                if i > 0 {
                    heading += turns[i - 1];
                }
                let (x, y) = points[i];
                points.push((
                    x + stride * libm::cos(heading),
                    y + stride * libm::sin(heading),
                ));
            }

            // Return the walk.
            points
        })
        .prop_filter("segments closer than the clearance", |points| {
            keeps_clearance(points, false)
        })
        .prop_filter("a segment the road would refuse", |points| {
            Waypoints::check_for_too_short_segments(points, false).is_none()
        })
        .prop_map(Waypoints::build_open)
}

/// A closed road of 3 to 12 points around a center, each at its own
/// angle and radius and walked in angular order, so it cannot cross
/// itself, then rotated and shifted anywhere. Half of them are walked
/// the other way round, so a corner turns right as often as it turns
/// left.
///
/// The angles come from proportional gaps. The widest gap is under what
/// the others add up to, so no gap reaches half a turn: a segment
/// spanning that much would pass across the center and through the
/// segments on the far side, and the polygon would no longer be simple.
/// No two points are closer than about three degrees apart at five
/// meters out, a quarter of a meter, which keeps every segment far longer
/// than the road's minimum. The filters hold it to the clearance and
/// guard the builder all the same, as `open_road` says.
fn closed_road() -> impl Strategy<Value = Waypoints> {
    (3..=12usize)
        .prop_flat_map(|n| {
            (
                proptest::collection::vec(1.0..(n - 1) as f64, n),
                proptest::collection::vec(5.0..100.0f64, n),
                any::<bool>(),
                placement(),
            )
        })
        .prop_map(|(gaps, radii, clockwise, placement)| {
            let whole_turn: f64 = gaps.iter().sum();
            let mut turned = 0.0;
            let mut points: Vec<_> = gaps
                .iter()
                .zip(&radii)
                .map(|(gap, radius)| {
                    let angle = TAU * turned / whole_turn;
                    turned += gap;
                    placement.apply((radius * libm::cos(angle), radius * libm::sin(angle)))
                })
                .collect();
            if clockwise {
                points.reverse();
            }

            // Return the polygon, placed and walked either way round.
            points
        })
        .prop_filter("segments closer than the clearance", |points| {
            keeps_clearance(points, true)
        })
        .prop_filter("a segment the road would refuse", |points| {
            Waypoints::check_for_too_short_segments(points, true).is_none()
        })
        .prop_map(Waypoints::build_closed)
}

/// Either kind of road, since most of what `frenet` promises holds on
/// both.
fn any_road() -> impl Strategy<Value = Waypoints> {
    prop_oneof![open_road(), closed_road()]
}

proptest! {
    // Each case is a few segments' worth of arithmetic, so a thousand of
    // them cost less than one compile, and every run draws a fresh
    // thousand. About a third of the open roads drawn fail the clearance
    // and are redrawn, roughly one redraw for every two cases, which the
    // default allowance of 65,536 redraws covers at that count but not
    // on a stress run of a few hundred thousand cases. A million redraws
    // suits a run of two million. A failure's seed lands beside this
    // file as `frenet_properties.proptest-regressions`, to be committed
    // so the case is rerun from then on; the default location is
    // relative to a crate root, which an integration test does not have.
    #![proptest_config(ProptestConfig {
        max_local_rejects: 1_000_000,
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::with_cases(1024)
    })]

    #[test]
    fn frenet_recovers_the_arc_length_of_a_point_on_the_road(
        road in any_road(),
        fraction in 0.0..1.0f64,
    ) {
        // The total itself is where a closed road wraps back to zero, so
        // the fraction stops short of it.
        let s = fraction * road.total_length();
        prop_assume!(s < road.total_length());

        let on_road = road.point_at(s);
        let (back_s, offset) = road.frenet(on_road.x, on_road.y);
        prop_assert!((back_s - s).abs() < TOLERANCE, "{s} came back {back_s}");
        prop_assert!(offset.abs() < TOLERANCE, "{s} read {offset} off the road");
    }

    #[test]
    fn frenet_recovers_a_lane_point_while_the_lane_has_room(
        road in any_road(),
        fraction in 0.0..1.0f64,
        share_of_room in -0.95..0.95f64,
    ) {
        let s = fraction * road.total_length();
        prop_assume!(s < road.total_length());
        let on_road = road.point_at(s);

        // The segment the point is on is the nearest to it, at no
        // distance, and the room beside it is what bounds the lane. A
        // point within a micron of a vertex has room too small for any
        // lane and too small for rounding to respect, so it is skipped.
        let (own_segment, _) = nearest_segment(&road, on_road.x, on_road.y);
        let room = room_for_a_lane(&road, on_road.x, on_road.y, own_segment);
        prop_assume!(room > 1e-6);

        let lateral = share_of_room * room;
        let point = road.point_at_offset(s, lateral);
        let (back_s, back_offset) = road.frenet(point.x, point.y);
        prop_assert!((back_s - s).abs() < TOLERANCE, "s {s} came back {back_s}");
        prop_assert!(
            (back_offset - lateral).abs() < TOLERANCE,
            "lateral {lateral} came back {back_offset}"
        );
    }

    #[test]
    fn the_offset_is_the_distance_to_the_road(
        road in any_road(),
        (x, y) in anywhere(),
    ) {
        let (s, offset) = road.frenet(x, y);
        let (_, dist_to_road) = nearest_segment(&road, x, y);

        // Past an end the road carries on as the line its end segment
        // holds, and the offset measures across that line rather than to
        // the road.
        let points = road.points();
        let end_line = match stretch(&road, s) {
            Stretch::OnTheRoad => None,
            Stretch::BeforeTheStart => Some((points[0], points[1])),
            Stretch::PastTheEnd => Some((points[points.len() - 2], points[points.len() - 1])),
        };
        if let Some((from, to)) = end_line {
            // Before the start or past the end. The line passes through
            // the road, so it can only be nearer than the road is.
            let dist_to_line = distance_to_line(from, to, x, y);
            prop_assert!(
                (offset.abs() - dist_to_line).abs() < TOLERANCE,
                "({x}, {y}) read {offset} off a line {dist_to_line} away"
            );
            prop_assert!(offset.abs() <= dist_to_road + TOLERANCE);
        } else {
            // On the road, which a closed road is everywhere.
            prop_assert!(
                (offset.abs() - dist_to_road).abs() < TOLERANCE,
                "({x}, {y}) read {offset} off a road {dist_to_road} away"
            );
        }
    }

    #[test]
    fn the_offset_changes_no_faster_than_a_point_moving_beside_the_road(
        road in any_road(),
        fraction in 0.0..1.0f64,
        lateral in -BESIDE_THE_ROAD..BESIDE_THE_ROAD,
        heading in 0.0..TAU,
        step in 0.001..0.5f64,
    ) {
        // A straight walk of a few dozen short steps from a point beside
        // the road, so it crosses the road, rounds corners, runs off the
        // ends, and can wander out of the band beside the road as it
        // goes. The offset may change by at most as far as the point
        // moved.
        //
        // Its magnitude is a distance, to the road or to the line the
        // road holds past an end, and a distance stays continuous as the
        // point moves. Its sign says which side of the nearest segment
        // the point is on, and two segments that meet agree on that all
        // round their vertex, where two that do not can face opposite
        // ways. Beside the road only segments that meet can both be
        // nearest, so the signed offset stays continuous there; far from
        // a road that doubles back it cannot, and only the magnitude is
        // checked.
        //
        // Past an end of an open road the offset measures to the line
        // the end segment holds. Where the walk crosses the perpendicular
        // at that end, the line and the segment meet, so both checks hold
        // across it. Where a nearer segment takes over from the line
        // instead, the offset steps by the difference between the two,
        // so a pair that changes stretch and nearest segment at once is
        // the one pair left unchecked.
        const STEPS: usize = 64;
        struct Sample {
            at: (f64, f64),
            stretch: Stretch,
            nearest_segment: usize,
            offset: f64,
            beside: bool,
        }
        let start = road.point_at_offset(fraction * road.total_length(), lateral);
        let (dx, dy) = (step * libm::cos(heading), step * libm::sin(heading));
        let sample = |k: usize| {
            let at = (start.x + dx * k as f64, start.y + dy * k as f64);
            let (s, offset) = road.frenet(at.0, at.1);
            let (nearest_segment, distance) = nearest_segment(&road, at.0, at.1);

            // Return the sample with everything the checks below ask of it.
            Sample {
                at,
                stretch: stretch(&road, s),
                nearest_segment,
                offset,
                beside: distance < BESIDE_THE_ROAD,
            }
        };
        let mut previous = sample(0);
        for k in 1..=STEPS {
            let current = sample(k);
            let at = current.at;
            let same_stretch = current.stretch == previous.stretch;
            let same_segment = current.nearest_segment == previous.nearest_segment;
            if same_stretch || same_segment {
                let change = (current.offset.abs() - previous.offset.abs()).abs();
                prop_assert!(
                    change <= step + TOLERANCE,
                    "offset magnitude jumped {change} at {at:?} where the point moved {step}"
                );
                if current.beside && previous.beside {
                    let change = (current.offset - previous.offset).abs();
                    prop_assert!(
                        change <= step + TOLERANCE,
                        "offset jumped {change} at {at:?} where the point moved {step}"
                    );
                }
            }
            previous = current;
        }
    }

    #[test]
    fn a_closed_roads_arc_length_stays_within_one_lap(
        road in closed_road(),
        (x, y) in anywhere(),
    ) {
        // The arc length can come back as the total itself rather than
        // as zero. Outside the corner where the road closes, a point is
        // measured to that corner, which ends the last segment and
        // starts the first. Both are the same distance away in exact
        // arithmetic, so rounding decides which answers, and the last
        // segment answers with the total where the first answers with
        // zero. `point_at` treats the total as on the road, so the range
        // is closed at both ends.
        let (s, _) = road.frenet(x, y);
        let total = road.total_length();
        prop_assert!((0.0..=total).contains(&s), "({x}, {y}) read {s} on a lap of {total}");
    }

    #[test]
    fn a_roads_arc_length_runs_on_past_both_ends(
        road in open_road(),
        share_beyond in 0.0..0.9f64,
        share_lateral in -0.9..0.9f64,
        past_the_end: bool,
    ) {
        // A point set down past one end of the road, in what would be its
        // lane had the road gone on: so far along the line the end
        // segment holds, and so far to either side of it, a negative
        // offset lying to the right. The arc length has to carry on
        // counting along that line and the offset has to hold, or every
        // car past the end would pile onto one arc length.
        let points = road.points();
        let (from, end) = if past_the_end {
            (points[points.len() - 2], points[points.len() - 1])
        } else {
            (points[1], points[0])
        };

        // The point stays within the room beside the end, so the end
        // segment is still the nearest road there. Further out, a road
        // bending back toward its own extension could put another segment
        // nearer, and a point there belongs to that segment's lane. The
        // distance out and the distance aside add up to at most the room.
        let own_segment = if past_the_end { road.num_segments() - 1 } else { 0 };
        let room = room_for_a_lane(&road, end.0, end.1, own_segment);
        let beyond = share_beyond * room;
        let expected_offset = share_lateral * (room - beyond);

        // Out along the end segment's own direction, then to its left.
        let length = ((end.0 - from.0).powi(2) + (end.1 - from.1).powi(2)).sqrt();
        let (ux, uy) = ((end.0 - from.0) / length, (end.1 - from.1) / length);
        let (x, y) = if past_the_end {
            (
                end.0 + beyond * ux - expected_offset * uy,
                end.1 + beyond * uy + expected_offset * ux,
            )
        } else {
            // The road runs the other way here, so its left is this
            // direction's right.
            (
                end.0 + beyond * ux + expected_offset * uy,
                end.1 + beyond * uy - expected_offset * ux,
            )
        };
        let expected_s = if past_the_end {
            road.total_length() + beyond
        } else {
            -beyond
        };

        let (s, offset) = road.frenet(x, y);
        prop_assert!(
            (s - expected_s).abs() < TOLERANCE,
            "({x}, {y}) read {s}, not {expected_s}"
        );
        prop_assert!(
            (offset - expected_offset).abs() < TOLERANCE,
            "({x}, {y}) read {offset}, not {expected_offset}"
        );
    }
}
