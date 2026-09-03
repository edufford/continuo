//! Property tests on `Waypoints::frenet`: what holds on any road, rather
//! than on the handful of fixtures the unit tests in `path.rs` draw by
//! hand.
//!
//! Every road here is generated so that it cannot cross itself. A crossing
//! puts one point on two stretches of road at once, and which of them is
//! nearer is then a matter of rounding. An open road is a walk that never
//! turns back along one axis, and a closed one is a star-shaped polygon
//! around a center, and both are then turned and moved anywhere in the
//! plane so that no property passes by leaning on an axis.

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

/// The distance from `(x, y)` to each segment of `road`, in road order.
///
/// Worked out from the points on their own, so it shares nothing with
/// `frenet` beyond the geometry and stands as the oracle for its offset.
fn distance_to_each_segment(road: &Waypoints, x: f64, y: f64) -> Vec<f64> {
    let points = road.points();

    // Return one distance per segment, each to the nearest point of that
    // segment rather than of its line.
    (0..road.num_segments())
        .map(|i| {
            let (ax, ay) = points[i];
            let (bx, by) = points[(i + 1) % points.len()];
            let (dx, dy) = (bx - ax, by - ay);
            let along = (((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            let (px, py) = (ax + dx * along, ay + dy * along);
            ((x - px).powi(2) + (y - py).powi(2)).sqrt()
        })
        .collect()
}

/// The same distances, nearest first.
fn distances_nearest_first(road: &Waypoints, x: f64, y: f64) -> Vec<f64> {
    let mut distances = distance_to_each_segment(road, x, y);
    distances.sort_by(f64::total_cmp);

    // Return them sorted, so the nearest segment is first and the runner-up
    // second.
    distances
}

/// The distance from `(x, y)` to the line through `from` and `to`, rather
/// than to the segment between them.
fn distance_to_line(from: (f64, f64), to: (f64, f64), x: f64, y: f64) -> f64 {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);

    // Return the perpendicular distance, the cross product over the
    // length.
    ((dx * (y - from.1) - dy * (x - from.0)) / (dx * dx + dy * dy).sqrt()).abs()
}

/// Which stretch of an open road's arc length `s` falls on. The offset is
/// continuous within each stretch and can step between two of them,
/// where the line a road holds past its end gives way to a nearer bend.
#[derive(Debug, PartialEq, Eq)]
enum Stretch {
    BeforeTheStart,
    OnTheRoad,
    PastTheEnd,
}

fn stretch(road: &Waypoints, s: f64) -> Stretch {
    // Return the stretch. A loop has no ends, so all of it is one.
    if road.is_closed() || (0.0..=road.total_length()).contains(&s) {
        Stretch::OnTheRoad
    } else if s < 0.0 {
        Stretch::BeforeTheStart
    } else {
        Stretch::PastTheEnd
    }
}

/// A turn and a shift, taking a road drawn against the axes anywhere in
/// the plane facing any way.
#[derive(Debug, Clone, Copy)]
struct Placement {
    angle: f64,
    shift: (f64, f64),
}

impl Placement {
    fn apply(&self, (x, y): (f64, f64)) -> (f64, f64) {
        let (sin, cos) = (libm::sin(self.angle), libm::cos(self.angle));

        // Return the point turned about the origin and then shifted.
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

/// A point anywhere a road can reach and well beyond.
fn anywhere() -> impl Strategy<Value = (f64, f64)> {
    (-1000.0..1000.0, -1000.0..1000.0)
}

/// An open road of 2 to 8 points that never turns back on itself. Each
/// point lies at least a meter further along one axis than the last and
/// anywhere within a hundred meters of it on the other, so a bend can be
/// as sharp as it likes short of doubling back, and no segment can be
/// short enough for the road to refuse.
///
/// Every range here excludes what a float can otherwise be, since a
/// road with a NaN in it is not a road, and the refusal check guards the
/// builder all the same: a road it would refuse is a case to discard
/// rather than a panic to report.
fn open_road() -> impl Strategy<Value = Waypoints> {
    (2..=8usize)
        .prop_flat_map(|n| {
            (
                proptest::collection::vec(1.0..100.0f64, n - 1),
                proptest::collection::vec(-100.0..100.0f64, n),
                placement(),
            )
        })
        .prop_map(|(steps, ys, placement)| {
            let mut x = 0.0;
            let mut points = Vec::with_capacity(ys.len());
            for (i, y) in ys.into_iter().enumerate() {
                if i > 0 {
                    x += steps[i - 1];
                }
                points.push(placement.apply((x, y)));
            }

            // Return the walk, placed.
            points
        })
        .prop_filter("a segment the road would refuse", |points| {
            Waypoints::check_for_too_short_segments(points, false).is_none()
        })
        .prop_map(Waypoints::build_open)
}

/// A closed road of 3 to 12 points around a center, each at its own
/// angle and radius and walked in angular order, so it cannot cross
/// itself. Half of them are walked the other way round, so a corner
/// turns right as often as it turns left.
///
/// The angles come from proportional gaps. The widest gap is under what
/// the others add up to, so no gap reaches half a turn: a segment
/// spanning that much would pass across the center and through the
/// segments on the far side, and the polygon would no longer be simple.
/// No two points are closer than about three degrees apart at five
/// meters out, a quarter of a meter, which keeps every segment far longer
/// than the road's minimum. The filter guards the builder all the same,
/// as `open_road` says.
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
    // thousand. A failure's seed lands in `proptest-regressions` beside
    // this file, to be committed so the case is rerun from then on; the
    // default location is relative to a crate root, which an integration
    // test does not have.
    #![proptest_config(ProptestConfig {
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::with_cases(1024)
    })]

    #[test]
    fn a_point_on_the_road_reads_its_own_arc_length(
        road in any_road(),
        fraction in 0.0..1.0f64,
    ) {
        // The total itself is where a loop wraps back to zero, so the
        // fraction stops short of it.
        let s = fraction * road.total_length();
        prop_assume!(s < road.total_length());

        let on_road = road.point_at(s);
        let (back_s, offset) = road.frenet(on_road.x, on_road.y);
        prop_assert!((back_s - s).abs() < TOLERANCE, "{s} came back {back_s}");
        prop_assert!(offset.abs() < TOLERANCE, "{s} read {offset} off the road");
    }

    #[test]
    fn a_lane_point_round_trips_within_the_bends_reach(
        road in any_road(),
        fraction in 0.0..1.0f64,
        share_of_reach in -0.95..0.95f64,
    ) {
        let s = fraction * road.total_length();
        prop_assume!(s < road.total_length());
        let on_road = road.point_at(s);

        // How far a lane can sit off this point of the road before some
        // other stretch of it is nearer than the stretch it is on: half
        // the distance to the nearest other stretch, since moving a point
        // by a distance brings it at most that much closer to anything.
        // The nearest stretch of all is the one it is on, at no distance,
        // so the runner-up is the one that bounds the reach. A point on a
        // vertex is on two stretches at once and has no reach at all, and
        // one within a micron of a vertex has none worth a lane. A road
        // with nothing else near has more reach than any lane needs.
        let distances = distances_nearest_first(&road, on_road.x, on_road.y);
        let reach = distances.get(1).copied().unwrap_or(f64::INFINITY) / 2.0;
        let reach = reach.min(50.0);
        prop_assume!(reach > 1e-6);

        let lateral = share_of_reach * reach;
        let point = road.point_at_offset(s, lateral);
        let (back_s, back_lateral) = road.frenet(point.x, point.y);
        prop_assert!((back_s - s).abs() < TOLERANCE, "s {s} came back {back_s}");
        prop_assert!(
            (back_lateral - lateral).abs() < TOLERANCE,
            "lateral {lateral} came back {back_lateral}"
        );
    }

    #[test]
    fn the_offset_is_the_distance_to_the_road(
        road in any_road(),
        (x, y) in anywhere(),
    ) {
        let (s, offset) = road.frenet(x, y);
        let nearest = distances_nearest_first(&road, x, y)[0];
        match stretch(&road, s) {
            Stretch::OnTheRoad => {
                prop_assert!(
                    (offset.abs() - nearest).abs() < TOLERANCE,
                    "({x}, {y}) read {offset} off a road {nearest} away"
                );
            }
            end => {
                // Past an end the road carries on as the line its last
                // segment holds, and the offset measures across that line.
                // The line passes through the road, so it can only be
                // nearer than the road is.
                let points = road.points();
                let (from, to) = match end {
                    Stretch::BeforeTheStart => (points[0], points[1]),
                    _ => (points[points.len() - 2], points[points.len() - 1]),
                };
                let to_line = distance_to_line(from, to, x, y);
                prop_assert!(
                    (offset.abs() - to_line).abs() < TOLERANCE,
                    "({x}, {y}) read {offset} off a line {to_line} away"
                );
                prop_assert!(offset.abs() <= nearest + TOLERANCE);
            }
        }
    }

    #[test]
    fn the_offset_changes_no_faster_than_the_point_moves(
        road in any_road(),
        (x, y) in anywhere(),
        heading in 0.0..TAU,
        step in 0.001..5.0f64,
    ) {
        // A straight walk of a few dozen steps from anywhere, so it
        // crosses the road, rounds corners and runs off the ends of it.
        // The offset may change by at most as far as the point moved,
        // except where the point steps from one stretch of an open road
        // to another. Within the road, or within the line it holds past
        // an end, the offset is a distance and a distance cannot step.
        const STEPS: usize = 64;
        let (dx, dy) = (step * libm::cos(heading), step * libm::sin(heading));
        let mut previous = road.frenet(x, y);
        for k in 1..=STEPS {
            let (px, py) = (x + dx * k as f64, y + dy * k as f64);
            let current = road.frenet(px, py);
            if stretch(&road, previous.0) == stretch(&road, current.0) {
                let change = (current.1 - previous.1).abs();
                prop_assert!(
                    change <= step + TOLERANCE,
                    "offset jumped {change} at ({px}, {py}) where the point moved {step}"
                );
            }
            previous = current;
        }
    }

    #[test]
    fn a_loops_arc_length_stays_within_one_lap(
        road in closed_road(),
        (x, y) in anywhere(),
    ) {
        // The total itself is on the path, as `point_at` treats it: a
        // point outside the seam's corner is measured to that corner from
        // whichever of its two segments rounds nearer, and the last one
        // answers with the total where the first answers with zero.
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
        // segment holds, and so far to the left of it. The arc length has
        // to carry on counting along that line and the offset has to
        // hold, or every car past the end would pile onto one arc length.
        let points = road.points();
        let (from, end) = if past_the_end {
            (points[points.len() - 2], points[points.len() - 1])
        } else {
            (points[1], points[0])
        };

        // Only where that end is the nearest road, with room to spare. A
        // road bending back toward its own extension puts another stretch
        // nearer, and a point there is in that stretch's lane instead,
        // which is the right answer and not this one. The room is half
        // the distance from the end to the nearest other stretch, since
        // going that far from the end brings the point at most that much
        // closer to anything, and the two shares together stay inside it.
        // An open road's end is at least a meter from every other stretch
        // by construction, so there is always some, and a road with
        // nothing else near has more than any lane needs.
        let mut distances = distance_to_each_segment(&road, end.0, end.1);
        distances.remove(if past_the_end { distances.len() - 1 } else { 0 });
        let nearest_other = distances.into_iter().fold(f64::INFINITY, f64::min);
        let room = (nearest_other / 2.0).min(100.0);
        let beyond = share_beyond * room;
        let lateral = share_lateral * (room - beyond);

        // Out along the end segment's own direction, then to its left.
        let length = ((end.0 - from.0).powi(2) + (end.1 - from.1).powi(2)).sqrt();
        let (ux, uy) = ((end.0 - from.0) / length, (end.1 - from.1) / length);
        let (x, y) = if past_the_end {
            (end.0 + beyond * ux - lateral * uy, end.1 + beyond * uy + lateral * ux)
        } else {
            // The road runs the other way here, so its left is this
            // direction's right.
            (end.0 + beyond * ux + lateral * uy, end.1 + beyond * uy - lateral * ux)
        };
        let expected_s = if past_the_end {
            road.total_length() + beyond
        } else {
            -beyond
        };

        let (s, offset) = road.frenet(x, y);
        prop_assert!((s - expected_s).abs() < TOLERANCE, "({x}, {y}) read {s}, not {expected_s}");
        prop_assert!((offset - lateral).abs() < TOLERANCE, "({x}, {y}) read {offset}, not {lateral}");
    }
}
