use super::*;
use crate::body::GravityModel;
use orbitx_math::vec3::Vec3;

fn earth_at(pos: Vec3, dynamics: bool) -> CelestialBody {
    CelestialBody {
        name: "Earth".into(),
        mass: 5.97e24,
        size: 6.371e6,
        pos,
        parent_idx: None,
        dynamics,
        ephemeris: None,
        fixed_pos: Some(pos),
        rotation: None,
        rotation_enabled: false,
        gravity: GravityModel::PointMass,
        atmosphere: None,
        color: [1.0; 4],
        radius_m: 6.371e6,
        min_render_radius: 0.2,
    }
}

#[test]
fn centers_primary_and_skips_visual_sun() {
    let mut sun = earth_at(Vec3::new(1.5e11, 0.0, 0.0), false);
    sun.name = "Sun".into();
    sun.mass = 1.9e30;
    let earth = earth_at(Vec3::new(1.0e8, 0.0, 0.0), true);
    let bodies = vec![earth, sun];
    let grav = grav_bodies(&bodies, 0);
    assert_eq!(grav.len(), 1);
    assert!(grav[0].pos.length() < 1.0);
}
