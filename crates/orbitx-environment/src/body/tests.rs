use super::*;

#[test]
fn rotation_disabled_reports_zero_period() {
    let body = CelestialBody {
        name: "Earth".into(),
        mass: 1.0,
        size: 1.0,
        pos: Vec3::ZERO,
        parent_idx: None,
        dynamics: true,
        ephemeris: None,
        fixed_pos: Some(Vec3::ZERO),
        rotation: Some(RotationState::from_config(&orbitx_config::RotationConfig {
            enabled: false,
            sid_rot_period: 86_164.0,
            sid_rot_offset: 0.0,
            obliquity: 0.0,
            lan: 0.0,
            lan_mjd: 51544.5,
            precession_period: 0.0,
            precession_obliquity: 0.0,
            precession_lan: 0.0,
        })),
        rotation_enabled: false,
        gravity: GravityModel::PointMass,
        atmosphere: None,
        color: [1.0, 1.0, 1.0, 1.0],
        radius_m: 1.0,
        min_render_radius: 0.2,
    };
    assert_eq!(body.sid_rot_period(), 0.0);
}
