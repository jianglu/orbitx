use super::*;
use orbitx_math::Vec3;

#[test]
fn equator_speed_matches_omega_r() {
    let r = 6_371_000.0;
    let period = 86_164.1;
    let pos = Vec3::new(r, 0.0, 0.0); // 赤道（Y=北）
    let v = surface_inertial_velocity(pos, period);
    let expected = std::f64::consts::TAU / period * r;
    assert!(
        (v.length() - expected).abs() < 1e-3,
        "|v| = {}, expect {}",
        v.length(),
        expected
    );
    // ω = Ŷ ⇒ v = ω × (R X̂) = −ω R Ẑ
    assert!((v.z + expected).abs() < 1e-3, "v.z = {}, expect −{}", v.z, expected);
    assert!(v.x.abs() < 1e-6 && v.y.abs() < 1e-6);
}

#[test]
fn pole_nearly_zero() {
    let r = 6_371_000.0;
    let period = 86_164.1;
    let pos = Vec3::new(0.0, r, 0.0);
    let v = surface_inertial_velocity(pos, period);
    assert!(v.length() < 1e-6, "pole |v| = {}", v.length());
}
