use super::*;

#[test]
fn new_sets_rot_orthogonal_to_axial_dir() {
    let p = DockPort::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert!((p.rot - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12);
    assert!(orbitx_math::dot(p.dir.unit(), p.rot.unit()).abs() < 0.1);
}

#[test]
fn with_rot_preserves_explicit_rot() {
    let rot = Vec3::new(0.0, 1.0, 0.0);
    let p = DockPort::with_rot(
        Vec3::new(4.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        rot,
    );
    assert!((p.rot - rot).length() < 1e-12);
    assert!(p.connected_to.is_none());
}
