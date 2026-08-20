//! Minimal spatial math with continuo's canonical conventions.
//!
//! SI units. Right-handed Z-up (ENU) world frame; body frames are
//! X-forward/Y-left/Z-up (REP-103 style). Orientation on the wire is always
//! a unit quaternion with named fields; Euler angles exist only at human
//! boundaries (config, APIs) with one convention, intrinsic Z-Y-X: apply
//! yaw about Z, then pitch about the new Y, then roll about the new X.

use serde::{Deserialize, Serialize};

/// Position or free vector in meters, named fields only.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }
}

/// Unit quaternion (Hamilton convention), named fields only, never arrays,
/// so component order can't be misread.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

/// Euler angles in radians: the API-level orientation convenience.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct EulerRad {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// Euler angles in degrees: the config-file orientation form (`rpy_deg`).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct EulerDeg {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Rotation about Z only, the common case for planar (2D) models.
    ///
    /// Positive turns counter-clockwise seen from above, which is left
    /// for anything driving along the ground. That follows from the
    /// right-handed Z-up frame rather than being a choice made here, and
    /// it is the sign every planar model in the workspace steers by.
    pub fn from_yaw(yaw: f64) -> Self {
        EulerRad {
            roll: 0.0,
            pitch: 0.0,
            yaw,
        }
        .to_quat()
    }

    /// Scales to unit length. The all-zero quaternion has no direction to
    /// preserve and would otherwise produce NaNs, so it (and anything within
    /// float underflow of it) normalizes to `IDENTITY`.
    pub fn normalized(self) -> Self {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if n == 0.0 {
            return Quat::IDENTITY;
        }

        // Return the same rotation scaled to unit length.
        Quat {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }

    /// Canonical quaternion → Euler conversion (intrinsic Z-Y-X).
    ///
    /// Pitch is constrained to [-π/2, π/2]; at the gimbal singularity
    /// (|pitch| = π/2) roll and yaw are not independent and roll is
    /// reported as 0 by the atan2 identities.
    ///
    /// Equations: the standard aerospace (3-2-1) sequence. See Diebel,
    /// "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation
    /// Vectors" (2006), eq. 290, and Wikipedia "Conversion between
    /// quaternions and Euler angles" (Quaternion → ZYX Euler). The asin
    /// argument is clamped against float drift pushing it past ±1.
    pub fn to_euler(self) -> EulerRad {
        let q = self.normalized();
        let roll = f64::atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        );
        let pitch = (2.0 * (q.w * q.y - q.z * q.x)).clamp(-1.0, 1.0).asin();
        let yaw = f64::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        );

        // Return the intrinsic Z-Y-X decomposition of the rotation.
        EulerRad { roll, pitch, yaw }
    }

    /// Yaw component only, a convenience for planar models.
    ///
    /// Measured counter-clockwise from the +x axis, which is the angle
    /// [`Quat::from_yaw`] takes.
    pub fn yaw(self) -> f64 {
        self.to_euler().yaw
    }
}

/// Hamilton product: `a * b` applies `b` first, then `a`.
///
/// Equations: the standard Hamilton quaternion product. See Wikipedia
/// "Quaternion" § Hamilton product (w = scalar part, (x, y, z) = vector
/// part; expansion of (w₁ + v₁)(w₂ + v₂) with i² = j² = k² = ijk = −1).
impl std::ops::Mul for Quat {
    type Output = Quat;
    fn mul(self, rhs: Quat) -> Quat {
        Quat {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl EulerRad {
    /// Canonical Euler → quaternion conversion (intrinsic Z-Y-X):
    /// `q = qz(yaw) * qy(pitch) * qx(roll)`.
    ///
    /// Equations: expansion of that half-angle product for the aerospace
    /// (3-2-1) sequence. See Diebel (2006) eq. 297 and Wikipedia
    /// "Conversion between quaternions and Euler angles" (Euler ZYX →
    /// quaternion). Verified against the composition identity by the
    /// `intrinsic_zyx_composition_order` test.
    pub fn to_quat(self) -> Quat {
        let (sr, cr) = (self.roll * 0.5).sin_cos();
        let (sp, cp) = (self.pitch * 0.5).sin_cos();
        let (sy, cy) = (self.yaw * 0.5).sin_cos();

        // Return the composed rotation, unit length by construction.
        Quat {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    pub fn to_degrees(self) -> EulerDeg {
        EulerDeg {
            roll: self.roll.to_degrees(),
            pitch: self.pitch.to_degrees(),
            yaw: self.yaw.to_degrees(),
        }
    }
}

impl EulerDeg {
    pub fn to_radians(self) -> EulerRad {
        EulerRad {
            roll: self.roll.to_radians(),
            pitch: self.pitch.to_radians(),
            yaw: self.yaw.to_radians(),
        }
    }

    pub fn to_quat(self) -> Quat {
        self.to_radians().to_quat()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_1_SQRT_2;
    use std::f64::consts::PI;

    /// Tight tolerance for closed-form results (single trig evaluations).
    const EPS: f64 = 1e-12;
    /// Looser tolerance for round trips, which accumulate error across
    /// several trig and atan2/asin evaluations.
    const ROUND_TRIP_EPS: f64 = 1e-9;

    fn assert_close(a: f64, b: f64, eps: f64) {
        assert!((a - b).abs() < eps, "{a} != {b} (eps {eps})");
    }

    #[test]
    fn yaw_90_degrees() {
        let q = EulerDeg {
            roll: 0.0,
            pitch: 0.0,
            yaw: 90.0,
        }
        .to_quat();
        assert_close(q.w, FRAC_1_SQRT_2, EPS);
        assert_close(q.x, 0.0, EPS);
        assert_close(q.y, 0.0, EPS);
        assert_close(q.z, FRAC_1_SQRT_2, EPS);
    }

    #[test]
    fn identity_is_zero_euler() {
        let e = Quat::IDENTITY.to_euler();
        assert_close(e.roll, 0.0, EPS);
        assert_close(e.pitch, 0.0, EPS);
        assert_close(e.yaw, 0.0, EPS);
    }

    #[test]
    fn zero_quaternion_normalizes_to_identity() {
        let zero = Quat {
            w: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(zero.normalized(), Quat::IDENTITY);
        // And therefore converts to a well-defined orientation, not NaNs.
        let e = zero.to_euler();
        assert_close(e.roll, 0.0, EPS);
        assert_close(e.pitch, 0.0, EPS);
        assert_close(e.yaw, 0.0, EPS);
    }

    #[test]
    fn euler_round_trip() {
        let cases = [
            (0.1, -0.2, 0.3),
            (-1.0, 0.5, 2.5),
            (0.0, 0.0, -3.0),
            (3.0, -1.4, -2.9),
        ];
        for (roll, pitch, yaw) in cases {
            let e = EulerRad { roll, pitch, yaw };
            let back = e.to_quat().to_euler();
            assert_close(back.roll, roll, ROUND_TRIP_EPS);
            assert_close(back.pitch, pitch, ROUND_TRIP_EPS);
            assert_close(back.yaw, yaw, ROUND_TRIP_EPS);
        }
    }

    #[test]
    fn intrinsic_zyx_composition_order() {
        // q = qz(yaw) * qy(pitch) * qx(roll)
        let e = EulerRad {
            roll: 0.3,
            pitch: -0.4,
            yaw: 1.2,
        };
        let qz = EulerRad {
            roll: 0.0,
            pitch: 0.0,
            yaw: e.yaw,
        }
        .to_quat();
        let qy = EulerRad {
            roll: 0.0,
            pitch: e.pitch,
            yaw: 0.0,
        }
        .to_quat();
        let qx = EulerRad {
            roll: e.roll,
            pitch: 0.0,
            yaw: 0.0,
        }
        .to_quat();
        let composed = qz * qy * qx;
        let direct = e.to_quat();
        assert_close(composed.w, direct.w, EPS);
        assert_close(composed.x, direct.x, EPS);
        assert_close(composed.y, direct.y, EPS);
        assert_close(composed.z, direct.z, EPS);
    }

    #[test]
    fn yaw_wraps_at_pi() {
        let q = Quat::from_yaw(PI - 0.01);
        assert_close(q.yaw(), PI - 0.01, ROUND_TRIP_EPS);
    }

    #[test]
    fn serde_named_fields() {
        let q = Quat::IDENTITY;
        assert_eq!(
            serde_json::to_string(&q).unwrap(),
            r#"{"w":1.0,"x":0.0,"y":0.0,"z":0.0}"#
        );
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(
            serde_json::to_string(&v).unwrap(),
            r#"{"x":1.0,"y":2.0,"z":3.0}"#
        );
    }
}
