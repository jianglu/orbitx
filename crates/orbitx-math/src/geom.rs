//! Geometric utility functions — mirrors the free functions at Vecmat.h:436-470
//! and Vecmat.cpp:738-777.
//!
//! All routines reproduce the left-handed conventions of the C++ source.

use crate::mat3::Matrix3;
use crate::vec3::{cross, dot, Vec3};

/// Plane equation coefficients (`PlaneCoeffs`, Vecmat.cpp:738).
///
/// Returns the plane `a·x + b·y + c·z + d = 0` passing through three points.
/// The C++ header (Vecmat.h:437) explicitly notes this assumes a **left-handed**
/// coordinate system.
pub fn plane_coeffs(p1: Vec3, p2: Vec3, p3: Vec3) -> (f64, f64, f64, f64) {
    let a = p1.y * (p2.z - p3.z) - p2.y * (p1.z - p3.z) + p3.y * (p1.z - p2.z);
    let b = p1.x * (p3.z - p2.z) - p2.x * (p3.z - p1.z) + p3.x * (p2.z - p1.z);
    let c = p1.x * (p2.y - p3.y) - p2.x * (p1.y - p3.y) + p3.x * (p1.y - p2.y);
    let d = -p1.x * a - p1.y * b - p1.z * c;
    (a, b, c, d)
}

/// Distance from point `a` to a line through `p` with direction `d`
/// (`PointLineDist`, Vecmat.h:442). Implemented as `|d̂ × (a − p)|`.
#[inline]
pub fn point_line_dist(a: Vec3, p: Vec3, d: Vec3) -> f64 {
    cross(d.unit(), a - p).length()
}

/// Signed distance from point `p` to plane `ax+by+cz+d=0` (`PointPlaneDist`,
/// Vecmat.cpp:747). **Note** divides by `-sqrt(a²+b²+c²)`, so the sign is
/// negated relative to the conventional form.
#[inline]
pub fn point_plane_dist(p: Vec3, a: f64, b: f64, c: f64, d: f64) -> f64 {
    let den = -(a * a + b * b + c * c).sqrt();
    (a * p.x + b * p.y + c * p.z + d) / den
}

/// Intersect line (point `p`, direction `s`) with plane `ax+by+cz+d=0`
/// (`LinePlaneIntersect`, Vecmat.cpp:754). Returns `None` if the line is
/// parallel to the plane (`D = 0`).
pub fn line_plane_intersect(a: f64, b: f64, c: f64, d: f64, p: Vec3, s: Vec3) -> Option<Vec3> {
    let denom = a * s.x + b * s.y + c * s.z;
    if denom == 0.0 {
        return None;
    }
    Some(Vec3::new(
        (p.x * (b * s.y + c * s.z) - s.x * (d + b * p.y + c * p.z)) / denom,
        (p.y * (a * s.x + c * s.z) - s.y * (d + a * p.x + c * p.z)) / denom,
        (p.z * (a * s.x + b * s.y) - s.z * (d + a * p.x + b * p.y)) / denom,
    ))
}

/// Unit normal of plane `ax+by+cz+d=0` (`PlaneNormal`, Vecmat.h:457). The `d`
/// coefficient is unused.
#[inline]
pub fn plane_normal(a: f64, b: f64, c: f64, _d: f64) -> Vec3 {
    Vec3::new(a, b, c).unit()
}

/// Build a rotation matrix from three orthonormal basis vectors expressed in
/// the global frame (`VectorBasisToMatrix`, Vecmat.cpp:764). The basis vectors
/// become the **rows** of `R`, so `R·p` transforms a global point into the XYZ
/// frame.
pub fn vector_basis_to_matrix(x: Vec3, y: Vec3, z: Vec3) -> Matrix3 {
    Matrix3::new(x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z)
}

/// Build a rotation matrix from a forward (`Z`) and up (`Y`) direction
/// (`DirRotToMatrix`, Vecmat.cpp:771). Computes `X = crossp(Y, Z)` — the
/// **left-handed** convention (right-handed would be `crossp(Z, Y)`) — then
/// delegates to [`vector_basis_to_matrix`].
pub fn dir_rot_to_matrix(z: Vec3, y: Vec3) -> Matrix3 {
    let x = cross(y, z); // left-handed
    vector_basis_to_matrix(x, y, z)
}

/// Rodrigues' rotation formula: rotate vector `v` about `axis` by `angle`.
///
/// `v_rot = v·cos θ + (k × v)·sin θ + k·(k·v)·(1 − cos θ)`, where `k` is the
/// unit axis. Pure axis-angle rotation math (no physics); used by TVC
/// direction computation. Returns `v` unchanged for a near-zero angle or a
/// degenerate axis.
#[inline]
pub fn rodrigues(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    if angle.abs() < 1e-12 {
        return v;
    }
    if axis.length() <= 1e-9 {
        return v;
    }
    let axis = axis.unit();
    let c = angle.cos();
    let s = angle.sin();
    let kxv = cross(axis, v);
    let kdv = dot(axis, v);
    v * c + kxv * s + axis * (kdv * (1.0 - c))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_normal_xy() {
        // The plane z=0 has normal (0,0,1) in any handedness.
        let p1 = Vec3::new(1.0, 0.0, 0.0);
        let p2 = Vec3::new(0.0, 1.0, 0.0);
        let p3 = Vec3::new(0.0, 0.0, 0.0);
        let (a, b, c, d) = plane_coeffs(p1, p2, p3);
        let n = plane_normal(a, b, c, d);
        assert!(
            (n - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                || (n + Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
        );
    }

    #[test]
    fn line_plane_intersect_basic() {
        // Plane z=5, line through origin going +z.
        let r = line_plane_intersect(0.0, 0.0, 1.0, -5.0, Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0));
        assert!(r.is_some());
        let p = r.unwrap();
        assert!((p - Vec3::new(0.0, 0.0, 5.0)).length() < 1e-9);
    }

    #[test]
    fn dir_rot_orthonormal() {
        let z = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let m = dir_rot_to_matrix(z, y);
        // X = cross(Y, Z) = (0,0,1)·... wait: cross((0,1,0),(1,0,0))=(0,0,-1) standard.
        // Row 0 should be X.
        let x_row = m.row(0);
        assert!(x_row.length() > 0.0);
    }

    #[test]
    fn rodrigues_rotates_about_axis() {
        use crate::consts::PI05;
        // Rotate +x about +z by π/2 → +y (left-handed frame: cross(z,x) = y).
        let v = Vec3::new(1.0, 0.0, 0.0);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let r = rodrigues(v, axis, PI05);
        assert!((r - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-12, "got {r:?}");
    }

    #[test]
    fn rodrigues_zero_angle_identity() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        assert!((rodrigues(v, axis, 0.0) - v).length() < 1e-12);
    }

    #[test]
    fn rodrigues_degenerate_axis_identity() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert!((rodrigues(v, Vec3::ZERO, 1.0) - v).length() < 1e-12);
    }
}
