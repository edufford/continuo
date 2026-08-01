use continuo_core::Vec3;

/// A 2D polyline with arc-length parameterization, closed into a loop or
/// open with two ends — the demo "map" until the world spec exists (see
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

    fn build(points: Vec<(f64, f64)>, is_closed: bool) -> Self {
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
        // plus the leading zero, so it is one longer than the count -
        // whether the path closes is already baked into it by `build`.
        self.cumulative.len() - 1
    }

    /// An axis-aligned ellipse approximated by `samples` points.
    pub fn ellipse(center: (f64, f64), semi_x: f64, semi_y: f64, samples: usize) -> Self {
        let points = (0..samples)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / samples as f64;
                (
                    center.0 + semi_x * angle.cos(),
                    center.1 + semi_y * angle.sin(),
                )
            })
            .collect();

        // Return the sampled ellipse as a closed loop.
        Self::build_closed(points)
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
        // path's end nothing is "beyond" — the answer would otherwise run
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
    /// to the **left** of the path — the Frenet `(s, d)` pair, resolved
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
            base.x - lateral * heading.sin(),
            base.y + lateral * heading.cos(),
            0.0,
        )
    }

    /// Path heading (yaw, radians) at arc length `s`.
    pub fn heading_at(&self, s: f64) -> f64 {
        let (i, _) = self.locate(s);
        let a = self.points[i];
        let b = self.points[(i + 1) % self.points.len()];

        // Return the segment's direction as a yaw angle.
        f64::atan2(b.1 - a.1, b.0 - a.0)
    }

    /// Arc length of the closest point on the path to `(x, y)`.
    /// Deterministic: ties resolve to the earliest segment.
    pub fn project(&self, x: f64, y: f64) -> f64 {
        // Point-to-segment projection, evaluated per segment, keeping the
        // closest hit. For each segment a→b:
        //   (dx, dy)  segment direction vector b - a
        //   len2      its squared length (avoids a sqrt; zero for
        //             degenerate duplicate points)
        //   t         the query point's position along the segment as a
        //             fraction in [0, 1]: the perpendicular-foot parameter
        //             dot(p - a, b - a) / |b - a|^2, clamped so points
        //             "past" either endpoint project onto that endpoint
        //   (px, py)  the resulting closest point on the segment
        //   d2        squared distance from the query point to (px, py)
        //             (squared distances compare identically to distances,
        //             so no sqrt is needed)
        // The winning segment converts t back to arc length via its
        // cumulative start offset.
        let mut best_s = 0.0;
        let mut best_d2 = f64::INFINITY;
        for i in 0..self.num_segments() {
            let a = self.points[i];
            let b = self.points[(i + 1) % self.points.len()];
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len2 = dx * dx + dy * dy;
            let t = if len2 > 0.0 {
                (((x - a.0) * dx + (y - a.1) * dy) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (px, py) = (a.0 + dx * t, a.1 + dy * t);
            let d2 = (x - px).powi(2) + (y - py).powi(2);
            // Strict '<': on an exact tie (e.g. a shared corner) the
            // earliest segment wins, deterministically.
            if d2 < best_d2 {
                best_d2 = d2;
                best_s = self.cumulative[i] + (self.cumulative[i + 1] - self.cumulative[i]) * t;
            }
        }

        // Return the arc length of the closest point found.
        best_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Waypoints {
        Waypoints::build_closed(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
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
        //   bottom (0,0)→(10,0) s=[0,10), right (10,0)→(10,10) s=[10,20),
        //   top (10,10)→(0,10) s=[20,30), left (0,10)→(0,0) s=[30,40).
        let p = square();
        let s = p.project(7.0, -1.0); // below the bottom edge
        assert!((s - 7.0).abs() < 1e-12);
        let s = p.project(11.0, 3.0); // right of the right edge
        assert!((s - 13.0).abs() < 1e-12);
        let s = p.project(4.0, 11.0); // above the top edge (runs right-to-left)
        assert!((s - 26.0).abs() < 1e-12);
        let s = p.project(-1.0, 3.0); // left of the left edge (runs top-to-bottom)
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
        // pointing down the road - where a closed path would have sent it
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

        assert!((road.project(30.0, 5.0) - 30.0).abs() < 1e-12);
        assert!((road.project(100.0, 20.0) - 120.0).abs() < 1e-12);
        // Off either end, projection lands on the nearest end point rather
        // than round the other side.
        assert!(road.project(-20.0, 0.0).abs() < 1e-12);
        assert!((road.project(100.0, 80.0) - 150.0).abs() < 1e-12);
    }

    #[test]
    fn projection_at_corner_ties_to_earliest_segment() {
        let p = square();
        // (11, -1) is closest to the (10, 0) corner, shared by the bottom
        // segment (t=1, s=10) and the right segment (t=0, s=10): both give
        // the same distance and the same arc length here, and the strict
        // '<' tie-break keeps the bottom (earliest) segment's answer.
        let s = p.project(11.0, -1.0);
        assert!((s - 10.0).abs() < 1e-12);
        // A point exactly on a corner projects to that corner.
        let s = p.project(0.0, 10.0);
        assert!((s - 30.0).abs() < 1e-12);
    }
}
