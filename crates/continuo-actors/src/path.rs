use continuo_core::Vec3;

/// A closed 2D polyline with arc-length parameterization — the demo "map"
/// until the world spec exists (see PLAN.md, World and map).
#[derive(Debug, Clone)]
pub struct Waypoints {
    points: Vec<(f64, f64)>,
    /// cumulative[i] = arc length at the start of segment i; the last entry
    /// is the total loop length.
    cumulative: Vec<f64>,
}

impl Waypoints {
    /// Builds a closed loop from at least 3 planar points (the last point
    /// connects back to the first).
    pub fn closed(points: Vec<(f64, f64)>) -> Self {
        assert!(points.len() >= 3, "a closed path needs at least 3 points");
        let mut cumulative = Vec::with_capacity(points.len() + 1);
        let mut total = 0.0;
        cumulative.push(0.0);
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            total += ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            cumulative.push(total);
        }
        Waypoints { points, cumulative }
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
        Self::closed(points)
    }

    pub fn total_length(&self) -> f64 {
        *self.cumulative.last().unwrap()
    }

    fn wrap(&self, s: f64) -> f64 {
        s.rem_euclid(self.total_length())
    }

    /// Segment index and interpolation fraction at arc length `s`.
    fn locate(&self, s: f64) -> (usize, f64) {
        let s = self.wrap(s);
        // partition_point: first segment whose end is beyond s.
        let i = self.cumulative[1..].partition_point(|&end| end <= s);
        let seg_start = self.cumulative[i];
        let seg_len = self.cumulative[i + 1] - seg_start;
        let frac = if seg_len > 0.0 {
            (s - seg_start) / seg_len
        } else {
            0.0
        };
        (i, frac)
    }

    /// World-frame position at arc length `s` (wrapped), `z = 0`.
    pub fn point_at(&self, s: f64) -> Vec3 {
        let (i, frac) = self.locate(s);
        let a = self.points[i];
        let b = self.points[(i + 1) % self.points.len()];
        Vec3::new(a.0 + (b.0 - a.0) * frac, a.1 + (b.1 - a.1) * frac, 0.0)
    }

    /// Path heading (yaw, radians) at arc length `s`.
    pub fn heading_at(&self, s: f64) -> f64 {
        let (i, _) = self.locate(s);
        let a = self.points[i];
        let b = self.points[(i + 1) % self.points.len()];
        f64::atan2(b.1 - a.1, b.0 - a.0)
    }

    /// Arc length of the closest point on the path to `(x, y)`.
    /// Deterministic: ties resolve to the earliest segment.
    pub fn project(&self, x: f64, y: f64) -> f64 {
        let mut best_s = 0.0;
        let mut best_d2 = f64::INFINITY;
        for i in 0..self.points.len() {
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
            if d2 < best_d2 {
                best_d2 = d2;
                best_s = self.cumulative[i] + (self.cumulative[i + 1] - self.cumulative[i]) * t;
            }
        }
        best_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Waypoints {
        Waypoints::closed(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
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
    fn heading_follows_segments() {
        let p = square();
        assert!((p.heading_at(5.0) - 0.0).abs() < 1e-12); // bottom: +x
        assert!((p.heading_at(15.0) - std::f64::consts::FRAC_PI_2).abs() < 1e-12); // right: +y
    }

    #[test]
    fn projection_recovers_arc_length() {
        let p = square();
        let s = p.project(7.0, -1.0); // below the bottom edge
        assert!((s - 7.0).abs() < 1e-12);
        let s = p.project(11.0, 3.0); // right of the right edge
        assert!((s - 13.0).abs() < 1e-12);
    }
}
