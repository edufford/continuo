use continuo_core::Vec3;

/// A 2D polyline with arc-length parameterization, closed into a loop or
/// open with two ends. The demo "map" until the world spec exists (see
/// PLAN.md, World and map).
// TODO(PLAN "World and map"): replace with named paths from the world spec
// scene graph (published on continuo/{world}/map) once it exists; actors
// should reference paths by name, not own their geometry.
#[derive(Debug, Clone)]
pub struct Waypoints {
    points: Vec<(f64, f64)>,
    /// cumulative[i] = arc length at the start of segment i; the last entry
    /// is the total path length.
    cumulative: Vec<f64>,
    /// Total path length, kept directly so lookups need no last-element
    /// access.
    total: f64,
    /// Whether the last point connects back to the first. Decides what
    /// happens off either end: a loop wraps, a road stops.
    is_closed: bool,
}

impl Waypoints {
    /// Builds a closed loop from at least 3 planar points (the last point
    /// connects back to the first). Arc lengths wrap, so there is no way to
    /// run off it.
    pub fn build_closed(points: Vec<(f64, f64)>) -> Self {
        assert!(points.len() >= 3, "a closed path needs at least 3 points");

        // Return the loop with its precomputed arc-length table.
        Self::build(points, true)
    }

    /// Builds an open path from at least 2 planar points: a road with a
    /// start and an end rather than a circuit.
    ///
    /// Arc lengths **clamp** instead of wrapping, so a lookahead past the
    /// end returns the final point and keeps pointing that way, rather than
    /// teleporting a follower back to the beginning. Anything driving one
    /// of these has to be retired before it runs out of road.
    pub fn build_open(points: Vec<(f64, f64)>) -> Self {
        assert!(points.len() >= 2, "an open path needs at least 2 points");

        // Return the path with its precomputed arc-length table.
        Self::build(points, false)
    }

    /// A single straight segment, the simplest open path.
    pub fn build_straight(from: (f64, f64), to: (f64, f64)) -> Self {
        // Return the one-segment path.
        Self::build_open(vec![from, to])
    }

    /// How short a segment may be before a road carrying it is refused.
    ///
    /// A millimeter is far below anything this world models, where a car
    /// is `CAR_LENGTH` and a lane a few meters wide, and far above the
    /// float noise and import artifacts an invalid segment could come from.
    pub const MIN_SEGMENT_LENGTH: f64 = 1e-3;

    /// Find the first waypoint too close to the one before it, if any.
    ///
    /// [`build_open`](Self::build_open()) and
    /// [`build_closed`](Self::build_closed()) refuse a road with a
    /// segment shorter than [`MIN_SEGMENT_LENGTH`](Self::MIN_SEGMENT_LENGTH),
    /// and an external caller like the FMU controller can check before
    /// building rather than be panicked.
    pub fn check_for_too_short_segments(points: &[(f64, f64)], is_closed: bool) -> Option<usize> {
        let segments = if is_closed {
            points.len()
        } else {
            points.len() - 1
        };

        // Return the waypoint that's too close to its previous neighbor, since
        // that would be the one to take out.
        (0..segments)
            .find(|&i| {
                let a = points[i];
                let b = points[(i + 1) % points.len()];
                ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt() < Self::MIN_SEGMENT_LENGTH
            })
            .map(|i| (i + 1) % points.len())
    }

    fn build(points: Vec<(f64, f64)>, is_closed: bool) -> Self {
        // TODO(PLAN "World and map"): hand back a Result rather than
        // panic. A road written out as a Rust literal is a bug worth
        // halting on, but an imported map is data, and the FMU
        // controller already has to check ahead of this call so that it
        // can answer its host with an error instead.
        if let Some(i) = Self::check_for_too_short_segments(&points, is_closed) {
            panic!(
                "waypoint {i} at {:?} is too close to the one before it, under {} m",
                points[i],
                Self::MIN_SEGMENT_LENGTH
            );
        }

        // A closed path has one segment per point (the last closes the
        // loop); an open one has a segment between each adjacent pair.
        let num_segments = if is_closed {
            points.len()
        } else {
            points.len() - 1
        };
        let mut cumulative = Vec::with_capacity(num_segments + 1);
        let mut total = 0.0;
        cumulative.push(0.0);
        for i in 0..num_segments {
            let a = points[i];
            // The modulo only ever wraps on a closed path's last segment;
            // an open one stops before reaching it.
            let b = points[(i + 1) % points.len()];
            total += ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            cumulative.push(total);
        }

        // Return the path with its precomputed arc-length table.
        Waypoints {
            points,
            cumulative,
            total,
            is_closed,
        }
    }

    /// How many segments the arc-length table covers.
    fn num_segments(&self) -> usize {
        // Return the segment count. The table holds each segment's end
        // plus the leading zero, so it is one longer than the count.
        // Whether the path closes is already baked into it by `build`.
        self.cumulative.len() - 1
    }

    /// An axis-aligned ellipse approximated by `samples` points.
    pub fn ellipse(center: (f64, f64), semi_x: f64, semi_y: f64, samples: usize) -> Self {
        let points = (0..samples)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / samples as f64;
                (
                    center.0 + semi_x * libm::cos(angle),
                    center.1 + semi_y * libm::sin(angle),
                )
            })
            .collect();

        // Return the sampled ellipse as a closed loop.
        Self::build_closed(points)
    }

    /// The points the path was built from, in order.
    ///
    /// These and [`is_closed`](Self::is_closed()) are the whole of a path,
    /// since the arc-length table is derived from them. So the two are what
    /// travels when a path has to reach somewhere a Rust value cannot, such
    /// as a controller inside an FMU, which rebuilds its own copy through
    /// the same builders.
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// Whether the last point connects back to the first.
    ///
    /// It travels beside [`points`](Self::points()) because one set of
    /// points makes two different paths: a loop wraps at the end where a
    /// road stops.
    pub fn is_closed(&self) -> bool {
        self.is_closed
    }

    pub fn total_length(&self) -> f64 {
        self.total
    }

    /// Brings an arbitrary arc length onto the path: a loop wraps round, a
    /// road stops at its ends.
    ///
    /// The result can be the total length itself, from either branch. A
    /// road clamps to its end by definition; a loop gets there for a small
    /// enough negative `s`, where the remainder is too close to the total
    /// to be a separate `f64` and rounds up to it. Callers must treat the
    /// total as on the path rather than one past it.
    fn resolve_arc_length(&self, s: f64) -> f64 {
        // Return the equivalent arc length that is actually on the path.
        if self.is_closed {
            s.rem_euclid(self.total)
        } else {
            s.clamp(0.0, self.total)
        }
    }

    /// Segment index and interpolation fraction at arc length `s`.
    fn locate(&self, s: f64) -> (usize, f64) {
        let s = self.resolve_arc_length(s);
        // partition_point: first segment whose end is beyond s. Capped
        // because `resolve_arc_length` can hand back the total, and at the
        // path's end nothing is "beyond", so the answer would otherwise run
        // off the table. The cap reads that as "at the end of the last
        // segment", which for a loop is its start point, the same answer
        // wrapping would have given.
        let i = self.cumulative[1..]
            .partition_point(|&end| end <= s)
            .min(self.num_segments() - 1);
        let seg_start = self.cumulative[i];
        let seg_len = self.cumulative[i + 1] - seg_start;
        let frac = if seg_len > 0.0 {
            (s - seg_start) / seg_len
        } else {
            0.0
        };

        // Return the containing segment index and the fraction along it.
        (i, frac)
    }

    /// World-frame position at arc length `s`, brought onto the path first
    /// (a loop wraps, a road clamps), with `z = 0`.
    pub fn point_at(&self, s: f64) -> Vec3 {
        let (i, frac) = self.locate(s);
        let a = self.points[i];
        let b = self.points[(i + 1) % self.points.len()];

        // Return the interpolated point on that segment (z = 0).
        Vec3::new(a.0 + (b.0 - a.0) * frac, a.1 + (b.1 - a.1) * frac, 0.0)
    }

    /// World-frame position at arc length `s`, displaced `lateral` meters
    /// to the **left** of the path: the Frenet `(s, d)` pair, resolved
    /// into world coordinates.
    ///
    /// This is what lets one road serve every lane: a lane is an offset
    /// rather than a curve of its own, so nothing has to generate parallel
    /// geometry, and lanes of a bending road stay the right distance apart
    /// by construction.
    pub fn point_at_offset(&self, s: f64, lateral: f64) -> Vec3 {
        let base = self.point_at(s);
        let heading = self.heading_at(s);

        // Return the point displaced along the left normal, which is the
        // heading turned a quarter circle counter-clockwise.
        Vec3::new(
            base.x - lateral * libm::sin(heading),
            base.y + lateral * libm::cos(heading),
            0.0,
        )
    }

    /// Path heading (yaw, radians) at arc length `s`.
    pub fn heading_at(&self, s: f64) -> f64 {
        let (i, _) = self.locate(s);
        let a = self.points[i];
        let b = self.points[(i + 1) % self.points.len()];

        // Return the segment's direction as a yaw angle.
        libm::atan2(b.1 - a.1, b.0 - a.0)
    }

    /// Where `(x, y)` sits on the path: the arc length of the closest
    /// point on it, and how far to the **left** of the path the point
    /// lies. Deterministic: an exact tie goes to the earliest segment.
    ///
    /// This is the Frenet `(s, d)` pair. Both halves are what let a lane
    /// be a number rather than a curve: two cars share a lane when their
    /// `d` agree, and one is ahead of the other when its `s` is larger.
    ///
    /// **It is not the inverse of
    /// [`point_at_offset`](Self::point_at_offset())**, which runs the
    /// other way, from an arc length and offset back to a point. A
    /// lane round a corner is not the same length as the road it
    /// follows, so measuring both by the road's arc length breaks
    /// down at a vertex and the two calls stop agreeing there. A lane
    /// on a road stays well clear of that;
    /// `frenet_is_not_the_inverse_of_point_at_offset_near_a_bend` is
    /// where it does not.
    ///
    /// Off either end of an **open** path both halves carry on: `s` runs
    /// past the end or below zero, and `d` goes on measuring across the
    /// line the road was holding. A car that has run out of road is then
    /// still in its lane and still a known distance ahead of the one
    /// behind it, where stopping `s` at the end would pile every car
    /// beyond it onto one arc length and hide them from each other.
    ///
    /// **Inside a bend the arc length steps.** Both segments reach a
    /// point there and the nearer one changes at the bisector, so `s`
    /// jumps as a point crosses it, by twice that point's distance to
    /// the segment that lost. The offset holds across the same
    /// crossing, both segments being equally near just there. Nothing
    /// smooths this away, since it is what nearest means on a polyline,
    /// and `RadarSensor` records what it costs a range.
    ///
    /// **Everywhere else, a point beyond a segment's end is measured to
    /// the vertex the road turns at**, which is the nearest road there
    /// is. Measuring across the segment's extension line instead would
    /// fall away to nothing as the point carried on past the corner, and
    /// two points either side of the wedge outside a bend would read
    /// meters apart. A closed path has no ends, so this is every clamp
    /// on one.
    pub fn frenet(&self, x: f64, y: f64) -> (f64, f64) {
        // Point-to-segment projection, evaluated per segment, keeping the
        // closest hit. For each segment a -> b:
        //   (dx, dy)  segment direction vector b - a
        //   len2      its squared length, which avoids a sqrt. `build`
        //             holds every segment at `MIN_SEGMENT_LENGTH` or
        //             more, so this is never zero and never near it
        //   t         where the query point projects onto the segment,
        //             as a fraction of its length:
        //             dot(p - a, b - a) / |b - a|^2, kept unclamped
        //             because which end it ran past is what decides the
        //             answer below
        //   (px, py)  the closest point on the segment, so t clamped
        //   d2        squared distance from the query point to (px, py)
        //             (squared distances compare identically to distances,
        //             so no sqrt is needed)
        // Segments are compared by distance to the segment itself rather
        // than to its line, so an extended end can never claim a point
        // that belongs to a bend somewhere else.
        let mut best_i = 0;
        let mut best_t = 0.0;
        let mut best_d2 = f64::INFINITY;
        for i in 0..self.num_segments() {
            let a = self.points[i];
            let b = self.points[(i + 1) % self.points.len()];
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len2 = dx * dx + dy * dy;
            let t = ((x - a.0) * dx + (y - a.1) * dy) / len2;
            let (px, py) = (a.0 + dx * t.clamp(0.0, 1.0), a.1 + dy * t.clamp(0.0, 1.0));
            let d2 = (x - px).powi(2) + (y - py).powi(2);
            // An exact tie goes to the earliest segment. Two segments
            // tie only where the nearest point on both is the vertex they
            // share, so they have the same distance anyway.
            if d2 < best_d2 {
                best_d2 = d2;
                best_i = i;
                best_t = t;
            }
        }

        let seg_start = self.cumulative[best_i];
        let seg_len = self.cumulative[best_i + 1] - seg_start;
        // Signed distance from the winning segment's line, positive to
        // the left: the 2D cross product of the direction with p - a,
        // over the segment's length, which the arc-length table already
        // holds. `build` already checks that every segment has a minimum
        // length, so this division needs no guard under it.
        let offset = self.side_of(best_i, x, y) / seg_len;

        // A projection past either end of a segment means the road
        // stopped before the point did. Off an open path that is the road
        // running out, and both halves carry on; anywhere else the road
        // turned, and the vertex is what to measure to.
        let ran_out_of_road = !self.is_closed
            && ((best_i == 0 && best_t < 0.0)
                || (best_i + 1 == self.num_segments() && best_t > 1.0));

        // Return where the closest road is, and which side of it the
        // query point lies on.
        if !(0.0..=1.0).contains(&best_t) && !ran_out_of_road {
            (
                seg_start + seg_len * best_t.clamp(0.0, 1.0),
                self.outside_of_the_bend(best_i, best_t) * best_d2.sqrt(),
            )
        } else {
            (seg_start + seg_len * best_t, offset)
        }
    }

    /// Twice the signed area of the triangle on segment `i` and
    /// `(x, y)`: positive with the point left of the segment, zero with
    /// the three in line.
    ///
    /// Divided by the segment's length this is the signed distance from
    /// its line, which is the same dot product that gives the arc length
    /// taken against the normal instead of along the segment.
    fn side_of(&self, i: usize, x: f64, y: f64) -> f64 {
        let a = self.points[i];
        let (dx, dy) = self.direction(i);

        // Return the cross product of the segment with a -> (x, y).
        dx * (y - a.1) - dy * (x - a.0)
    }

    /// Which side of the road the wedge outside a bend is on, as `1.0`
    /// or `-1.0`.
    ///
    /// A projection stopping at a vertex is past the end of both
    /// segments meeting there, and that region is the outside of the
    /// bend by construction. So the side belongs to the corner rather
    /// than to the point, and the two segment directions settle it
    /// between them: the outside of a left turn is the right, and the
    /// other way about.
    ///
    /// Asking the point instead cannot answer where it is in line with
    /// the segment it ran past, which is a car carrying straight on
    /// where the road turned. Two segments in line have no bend and no
    /// wedge, so the zero this would give is unreachable.
    fn outside_of_the_bend(&self, i: usize, t: f64) -> f64 {
        let segments = self.num_segments();
        let (arriving, leaving) = if t > 1.0 {
            (i, (i + 1) % segments)
        } else {
            ((i + segments - 1) % segments, i)
        };
        let (ax, ay) = self.direction(arriving);
        let (bx, by) = self.direction(leaving);

        // Return the far side of the turn the road makes here.
        -(ax * by - ay * bx).signum()
    }

    /// The direction segment `i` runs in, as `b - a`.
    fn direction(&self, i: usize) -> (f64, f64) {
        let a = self.points[i];
        let b = self.points[(i + 1) % self.points.len()];
        (b.0 - a.0, b.1 - a.1)
    }

    /// How far along the path `(x, y)` sits, for a caller with no use
    /// for the lateral half of [`frenet`](Self::frenet()).
    pub fn project_arc_length(&self, x: f64, y: f64) -> f64 {
        self.frenet(x, y).0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A road running 10 m east from the origin, then turning a right
    /// angle north for another 10 m, its corner at R below.
    ///
    /// It leaves two regions either side of it: the overlap, where both
    /// segments reach a point, and the wedge, where neither does. The
    /// tests cross both, and `(13, -4)` marks where they probe the
    /// wedge, whole meters from R so its distance is exact.
    ///
    /// ```text
    ///                                     ^ (10, 10)
    ///                                     |  north: the leaving segment
    ///         the overlap: both           |
    ///         segments reach a  \ ' ' ' ' |
    ///         point here, and   ' \ ' ' ' |
    ///         the nearer swaps  ' ' \ ' ' |
    ///         at the bisector   ' ' ' \ ' |
    ///                           ' ' ' ' \ | (10, 0)
    ///   east >----------------------------R - - - - - - -
    ///      (0, 0)                         : . . . . . . .  the wedge:
    ///                                     : . . . . . . .  neither segment
    ///                                     : . (13, -4). .  reaches it, so R
    ///                                     : . . . . . . .  is the nearest
    ///                                     : . . . . . . .  road point
    /// ```
    fn left_corner() -> Waypoints {
        Waypoints::build_open(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])
    }

    /// The same road turning south instead, so its corner turns right
    /// where [`left_corner`]'s turns left and the wedge sits above the
    /// road rather than below it.
    fn right_corner() -> Waypoints {
        Waypoints::build_open(vec![(0.0, 0.0), (10.0, 0.0), (10.0, -10.0)])
    }

    /// A road bending gently left rather than turning: 10 m east, then
    /// 25 m along a 7-24-25 triangle, about 16 degrees off.
    fn gentle_left_corner() -> Waypoints {
        Waypoints::build_open(vec![(0.0, 0.0), (10.0, 0.0), (34.0, 7.0)])
    }

    /// The same gentle bend the other way.
    fn gentle_right_corner() -> Waypoints {
        Waypoints::build_open(vec![(0.0, 0.0), (10.0, 0.0), (34.0, -7.0)])
    }

    fn square() -> Waypoints {
        Waypoints::build_closed(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
    }

    /// The same square walked the other way round, so its corners turn
    /// right where `square`'s turn left. Every rule about which side of
    /// the road a point is on has to answer both.
    fn square_clockwise() -> Waypoints {
        Waypoints::build_closed(vec![(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)])
    }

    #[test]
    fn arc_length_lookup() {
        let p = square();
        assert_eq!(p.total_length(), 40.0);
        let mid_bottom = p.point_at(5.0);
        assert!((mid_bottom.x - 5.0).abs() < 1e-12 && mid_bottom.y.abs() < 1e-12);
        let wrapped = p.point_at(45.0);
        assert!((wrapped.x - 5.0).abs() < 1e-12);
    }

    #[test]
    fn a_loop_wrapping_onto_its_own_end_stays_on_the_path() {
        let p = square();

        // Too small a step back to be representable as 40 - eps, so the
        // wrap lands on the total rather than just under it. The lookup
        // has to survive that and answer with the loop's start point.
        let s = (-1e-18f64).rem_euclid(p.total_length());
        assert_eq!(s, p.total_length());
        let start = p.point_at(-1e-18);
        assert!(start.x.abs() < 1e-12 && start.y.abs() < 1e-12);
        assert!((p.heading_at(-1e-18) - -std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn heading_follows_segments() {
        let p = square();
        assert!((p.heading_at(5.0) - 0.0).abs() < 1e-12); // bottom: +x
        assert!((p.heading_at(15.0) - std::f64::consts::FRAC_PI_2).abs() < 1e-12); // right: +y
    }

    #[test]
    fn projection_recovers_arc_length() {
        // Square loop, counter-clockwise from the origin:
        //   bottom (0,0)->(10,0) s=[0,10), right (10,0)->(10,10) s=[10,20),
        //   top (10,10)->(0,10) s=[20,30), left (0,10)->(0,0) s=[30,40).
        let p = square();
        let s = p.project_arc_length(7.0, -1.0); // below the bottom edge
        assert!((s - 7.0).abs() < 1e-12);
        let s = p.project_arc_length(11.0, 3.0); // right of the right edge
        assert!((s - 13.0).abs() < 1e-12);
        let s = p.project_arc_length(4.0, 11.0); // above the top edge (runs right-to-left)
        assert!((s - 26.0).abs() < 1e-12);
        let s = p.project_arc_length(-1.0, 3.0); // left of the left edge (runs top-to-bottom)
        assert!((s - 37.0).abs() < 1e-12);
    }

    #[test]
    fn an_open_path_stops_at_its_ends_instead_of_wrapping() {
        // A road, not a circuit: 100 m of it along +x.
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));
        assert_eq!(road.total_length(), 100.0);

        let middle = road.point_at(40.0);
        assert!((middle.x - 40.0).abs() < 1e-12 && middle.y.abs() < 1e-12);

        // Past the end, a follower's lookahead finds the end and keeps
        // pointing down the road, where a closed path would have sent it
        // back to the start.
        let past_end = road.point_at(140.0);
        assert!((past_end.x - 100.0).abs() < 1e-12);
        assert!((road.heading_at(140.0) - 0.0).abs() < 1e-12);
        // And before the start, likewise.
        let before_start = road.point_at(-40.0);
        assert!(before_start.x.abs() < 1e-12);
    }

    #[test]
    fn an_open_path_projects_along_its_length() {
        // Two segments, so the arc-length table has an interior joint.
        let road = Waypoints::build_open(vec![(0.0, 0.0), (100.0, 0.0), (100.0, 50.0)]);
        assert_eq!(road.total_length(), 150.0);

        assert!((road.project_arc_length(30.0, 5.0) - 30.0).abs() < 1e-12);
        assert!((road.project_arc_length(100.0, 20.0) - 120.0).abs() < 1e-12);
        // Off either end the arc length carries on rather than wrapping
        // round to the other one, so a point 20 m before the start is at
        // -20 and one 30 m past the end is at 180.
        assert!((road.project_arc_length(-20.0, 0.0) + 20.0).abs() < 1e-12);
        assert!((road.project_arc_length(100.0, 80.0) - 180.0).abs() < 1e-12);
    }

    #[test]
    fn projection_at_corner_ties_to_earliest_segment() {
        let p = square();
        // (11, -1) is closest to the (10, 0) corner, shared by the bottom
        // segment (t=1, s=10) and the right segment (t=0, s=10): both give
        // the same distance and the same arc length here, and the strict
        // '<' tie-break keeps the bottom (earliest) segment's answer.
        let s = p.project_arc_length(11.0, -1.0);
        assert!((s - 10.0).abs() < 1e-12);
        // A point exactly on a corner projects to that corner.
        let s = p.project_arc_length(0.0, 10.0);
        assert!((s - 30.0).abs() < 1e-12);
    }

    #[test]
    fn frenet_recovers_both_arc_length_and_signed_lateral() {
        // A bend, so nothing here passes by reading `y` back out as if
        // the road were the x axis.
        let road = Waypoints::build_open(vec![(0.0, 0.0), (100.0, 0.0), (100.0, 50.0)]);

        // Left of the way the road runs is positive, right negative.
        let (s, lateral) = road.frenet(30.0, 5.0);
        assert!((s - 30.0).abs() < 1e-12 && (lateral - 5.0).abs() < 1e-12);
        let (s, lateral) = road.frenet(30.0, -5.0);
        assert!((s - 30.0).abs() < 1e-12 && (lateral + 5.0).abs() < 1e-12);
        // The second segment runs +y, whose left is -x, so the sign
        // follows the road rather than the axes.
        let (s, lateral) = road.frenet(96.0, 20.0);
        assert!((s - 120.0).abs() < 1e-12 && (lateral - 4.0).abs() < 1e-12);

        // The pair `point_at_offset` resolves, so a point built at an
        // offset comes back carrying it. The last of the three is past
        // the bend, where the two segments' offsets point different ways.
        for (s, lateral) in [(10.0, 3.5), (70.0, -3.5), (130.0, 3.5)] {
            let point = road.point_at_offset(s, lateral);
            let (back_s, back_lateral) = road.frenet(point.x, point.y);
            assert!((back_s - s).abs() < 1e-12, "arc length {back_s} for {s}");
            assert!(
                (back_lateral - lateral).abs() < 1e-12,
                "lateral {back_lateral} for {lateral}"
            );
        }

        // Off the end of the road both halves carry on: 20 m past the
        // last waypoint is s = 170, still dead center in its lane.
        // Measuring to the end point instead would put it 20 m sideways,
        // and stopping s there would hide it from anything else out past
        // the end.
        let (s, lateral) = road.frenet(100.0, 70.0);
        assert!((s - 170.0).abs() < 1e-12 && lateral.abs() < 1e-12);

        // On a loop the same rule puts the inside on the left, since
        // `square` runs counter-clockwise.
        let (_, outside) = square().frenet(7.0, -1.0);
        let (_, inside) = square().frenet(7.0, 1.0);
        assert!((outside + 1.0).abs() < 1e-12 && (inside - 1.0).abs() < 1e-12);
    }

    #[test]
    fn frenet_offset_is_continuous_all_around_a_corner() {
        // A full circle, so it crosses the wedge where neither segment
        // reaches, runs alongside each of them in turn, and passes
        // through the overlap inside the bend. The offset holds across
        // all of it, the bisector included: the nearer segment changes
        // there, but both are the same distance away, which is what
        // makes it the bisector.
        //
        // Every corner shape a road makes: a right angle either way, a
        // gentle bend either way, and a loop's seam either way round,
        // where the arc length wraps as well. The radii bracket the
        // corner: inside it, across it, and wide enough to sweep past
        // whatever else is near.
        //
        // The offset shouldn't change faster than the point moves, so a
        // step larger than the point itself took is a discontinuity
        // rather than a coarse sweep. The arc length carries no such
        // bound, and `frenet`'s own docs say why.
        const RADII: [f64; 3] = [0.5, 3.0, 12.0];
        // A quarter of a degree apart, once around.
        const SAMPLES: usize = 1440;
        for (road, center, what) in [
            (left_corner(), (10.0, 0.0), "a right angle left"),
            (right_corner(), (10.0, 0.0), "a right angle right"),
            (gentle_left_corner(), (10.0, 0.0), "a gentle bend left"),
            (gentle_right_corner(), (10.0, 0.0), "a gentle bend right"),
            (square(), (0.0, 0.0), "a loop's seam"),
            (square_clockwise(), (0.0, 0.0), "a loop's seam, clockwise"),
        ] {
            for radius in RADII {
                // Where the point sits on this sample of the circle,
                // and how far off the road the sweep reads it as being.
                let offset_at = |sample: usize| {
                    let angle = std::f64::consts::TAU * sample as f64 / SAMPLES as f64;
                    road.frenet(
                        center.0 + radius * libm::cos(angle),
                        center.1 + radius * libm::sin(angle),
                    )
                    .1
                };

                // How far the point itself travels between two samples,
                // which is the most its offset may change between them.
                let step_distance = std::f64::consts::TAU * radius / SAMPLES as f64;

                // The largest the offset moves between two neighbors.
                let mut largest_change = 0.0f64;
                let mut previous = offset_at(0);
                for sample in 1..=SAMPLES {
                    let current = offset_at(sample);
                    largest_change = largest_change.max((current - previous).abs());
                    previous = current;
                }

                assert!(
                    largest_change <= step_distance + 1e-9,
                    "offset jumped {largest_change} at {what}, r = {radius}, \
                     where the point moved {step_distance}"
                );
            }
        }
    }

    #[test]
    fn frenet_keeps_one_side_of_the_road_all_round_a_corner() {
        // Which side a car is on is the whole of what a lane band reads,
        // and past a corner that answer comes from the corner rather
        // than from either segment. It has to agree with the segments
        // either side of it, or a car would read as swapping lanes by
        // rounding a bend.
        for mirror in [1.0, -1.0] {
            let road = if mirror > 0.0 {
                left_corner()
            } else {
                right_corner()
            };

            // Outside the bend the whole way: alongside the arriving
            // segment, through the wedge past the corner, then alongside
            // the leaving one.
            for (x, y) in [(5.0, -2.0), (13.0, -4.0), (12.0, 5.0)] {
                let (_, lateral) = road.frenet(x, y * mirror);
                assert!(lateral * -mirror > 0.0, "({x}, {y}) read {lateral}");
            }

            // And down the other side, which takes the three other ways
            // an answer comes back: a segment's own perpendicular, the
            // overlap where both reach and the nearer one wins, and past
            // the end of the road.
            for (x, y) in [(5.0, 2.0), (9.0, 5.0), (8.0, 12.0)] {
                let (_, lateral) = road.frenet(x, y * mirror);
                assert!(lateral * mirror > 0.0, "({x}, {y}) read {lateral}");
            }
        }
    }

    #[test]
    fn frenet_is_not_the_inverse_of_point_at_offset_near_a_bend() {
        // `point_at_offset` is not `frenet` backwards, and near a corner
        // no function is. The round trip holds while the offset stays
        // inside the bend's reach, which is where every lane on a road
        // sits, and both ways of failing past it are here so that neither
        // is rediscovered from inside a planner.
        let road = left_corner();

        // Inside the reach, and the pair comes back as it went in.
        for (s, lane) in [(5.0, 3.5), (5.0, -3.5), (15.0, 2.0)] {
            let point = road.point_at_offset(s, lane);
            let (back_s, back_lane) = road.frenet(point.x, point.y);
            assert!((back_s - s).abs() < 1e-12, "s {s} came back {back_s}");
            assert!(
                (back_lane - lane).abs() < 1e-12,
                "lane {lane} came back {back_lane}"
            );
        }

        // Outside the bend one pair answers for a whole arc, since every
        // point 5 m from the corner is 5 m from the road. So the map is
        // not one to one, and `point_at_offset` can only ever hand back
        // one of them.
        assert_eq!(road.frenet(15.0, 0.0), road.frenet(13.0, -4.0));

        // And inside it the two offsets overlap, so a point placed at one
        // arc length is nearer another stretch of road and answers with
        // that instead. Nothing is wrong with either function: the pair
        // it went in as belongs to no point on this road.
        let stray = road.point_at_offset(7.0, 3.5);
        let (s, lane) = road.frenet(stray.x, stray.y);
        assert!((s - 13.5).abs() < 1e-12, "{s}");
        assert!((lane - 3.0).abs() < 1e-12, "{lane}");
    }
    #[test]
    fn frenet_wraps_to_zero_at_a_closed_roads_seam() {
        // The last segment ends where the first begins, so the arc
        // length has to come back to zero there rather than carry on
        // past the total. Following the road through the seam is what
        // pins that: just before it reads near the total, at it exactly
        // zero, and just after it a little above zero.
        //
        // Both ways round the loop, since the two arrive at the seam
        // along different segments.
        for road in [square(), square_clockwise()] {
            let total = road.total_length();
            for expected in [total - 0.5, 0.0, 0.5] {
                let on_the_road = road.point_at(expected);
                let (s, lateral) = road.frenet(on_the_road.x, on_the_road.y);
                assert!((s - expected).abs() < 1e-12, "{expected} came back {s}");
                assert!(lateral.abs() < 1e-12, "{expected} read {lateral} off it");
            }
        }
    }

    #[test]
    fn frenet_goes_on_measuring_past_either_end_of_an_open_road() {
        // A road that has run out is the one case where a projection
        // stopping short does not mean a corner, so both halves carry on:
        // the arc length runs past the end or below zero, and the offset
        // goes on measuring across the line the road was holding.
        //
        // Pinning the arc length at the end instead would pile every car
        // beyond it onto one value, so nothing out there could tell how
        // far ahead anything else was, or that it was ahead at all.
        let road = Waypoints::build_straight((0.0, 0.0), (100.0, 0.0));
        for (x, y, expect_s, expect_lateral) in [
            (-20.0, 0.0, -20.0, 0.0),
            (-20.0, 3.5, -20.0, 3.5),
            (120.0, 0.0, 120.0, 0.0),
            (300.0, -3.5, 300.0, -3.5),
        ] {
            let (s, lateral) = road.frenet(x, y);
            assert!((s - expect_s).abs() < 1e-12, "({x}, {y}) gave s = {s}");
            assert!(
                (lateral - expect_lateral).abs() < 1e-12,
                "({x}, {y}) gave lateral = {lateral}"
            );
        }
    }

    /// A point as the bits it is made of, so the comparisons below are
    /// exact rather than close enough.
    fn bits(v: Vec3) -> (u64, u64, u64) {
        (v.x.to_bits(), v.y.to_bits(), v.z.to_bits())
    }

    #[test]
    fn a_road_rebuilt_from_its_points_is_the_same_road() {
        // A standard lane width, so the offset is one a real lane sits at.
        const LANE_OFFSET: f64 = 3.5;

        // A bend and a loop, so the copy is checked both where an open
        // path clamps and where a closed one wraps.
        let roads = [
            Waypoints::build_open(vec![(0.0, 0.0), (30.0, 0.0), (55.0, 18.0), (80.0, 18.0)]),
            square(),
        ];

        for road in roads {
            let copy = if road.is_closed() {
                Waypoints::build_closed(road.points().to_vec())
            } else {
                Waypoints::build_open(road.points().to_vec())
            };
            assert_eq!(copy.total_length().to_bits(), road.total_length().to_bits());

            // Past both ends as well as along it, since that is where the
            // two kinds of path answer differently.
            for pct in -10..=110 {
                let s = road.total_length() * pct as f64 / 100.0;
                assert_eq!(bits(copy.point_at(s)), bits(road.point_at(s)), "s = {s}");
                assert_eq!(
                    copy.heading_at(s).to_bits(),
                    road.heading_at(s).to_bits(),
                    "s = {s}"
                );
                // The call a lane-following controller makes, and worth
                // comparing in its own right: a left normal taken across a
                // corner rather than along one segment would answer
                // differently here while the centerline and the heading
                // still agreed.
                assert_eq!(
                    bits(copy.point_at_offset(s, LANE_OFFSET)),
                    bits(road.point_at_offset(s, LANE_OFFSET)),
                    "s = {s}"
                );
            }

            // Projection from all round the path, which is where a segment
            // table built any other way would show.
            for i in -2..=12 {
                for j in -3..=3 {
                    let (x, y) = (10.0 * i as f64, 5.0 * j as f64);
                    assert_eq!(
                        copy.project_arc_length(x, y).to_bits(),
                        road.project_arc_length(x, y).to_bits(),
                        "({x}, {y})"
                    );
                }
            }
        }
    }

    #[test]
    fn only_waypoints_closer_than_the_threshold_are_refused() {
        const UNDER: f64 = Waypoints::MIN_SEGMENT_LENGTH / 10.0;
        const OVER: f64 = Waypoints::MIN_SEGMENT_LENGTH * 10.0;
        let corner = |third: (f64, f64)| vec![(0.0, 0.0), (10.0, 0.0), third, (20.0, 10.0)];
        for (points, is_closed, expected) in [
            (corner((10.0, 0.0)), false, Some(2)),
            (corner((10.0 + UNDER, UNDER)), false, Some(2)),
            (corner((10.0 + OVER, OVER)), false, None),
            (corner((12.0, 3.0)), false, None),
            (
                vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 0.0)],
                true,
                Some(0),
            ),
            (
                vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
                true,
                None,
            ),
        ] {
            assert_eq!(
                Waypoints::check_for_too_short_segments(&points, is_closed),
                expected,
                "{points:?}, closed = {is_closed}"
            );
        }

        // This project's demo roads should not have any segments
        // that are too short.
        for road in [
            Waypoints::ellipse((0.0, 0.0), 30.0, 20.0, 72),
            Waypoints::build_straight((0.0, 0.0), (1200.0, 0.0)),
            square(),
        ] {
            assert_eq!(
                Waypoints::check_for_too_short_segments(road.points(), road.is_closed()),
                None
            );
        }
    }

    #[test]
    #[should_panic(expected = "waypoint 2 at (10.0, 0.0) is too close to the one before it")]
    fn building_a_road_checks_its_own_segment_lengths() {
        Waypoints::build_open(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 0.0), (10.0, 10.0)]);
    }
}
