use std::path::Path;

use orbitx_config::PlanetaryScenario;
use orbitx_dynamics::gacc_nbody;
use orbitx_math::vec3::Vec3;

use super::*;
use crate::resolve_ephemeris_data;

fn load_toml(toml: &str, data: &Path) -> PlanetarySystem {
    let scn = PlanetaryScenario::from_toml_str(toml).unwrap();
    PlanetarySystem::load(&scn, data).unwrap()
}

#[test]
fn celestials_sorted_by_mass() {
    let psys = load_toml(
        r#"
name = "Test"
star = "Sun"
mjd = 51544.5
primary = "Earth"
[[bodies]]
name = "Sun"
mass = 1.989e30
size = 6.96e8
[[bodies]]
name = "Earth"
mass = 5.97e24
size = 6.371e6
[[bodies]]
name = "Moon"
mass = 7.35e22
size = 1.74e6
parents = [["Moon", "Earth"]]
"#,
        Path::new("/nonexistent"),
    );
    assert_eq!(psys.celestials.len(), 3);
    assert!(psys.bodies[psys.celestials[0]].mass > psys.bodies[psys.celestials[1]].mass);
    assert!(psys.bodies[psys.celestials[1]].mass > psys.bodies[psys.celestials[2]].mass);
}

#[test]
fn gacc_point_mass_only() {
    let mut psys = load_toml(
        r#"
name = "Test"
star = "Earth"
mjd = 51544.5
primary = "Earth"
[[bodies]]
name = "Earth"
mass = 5.97e24
size = 6.371e6
fixed_pos = [0.0, 0.0, 0.0]
"#,
        Path::new("/nonexistent"),
    );
    psys.bodies_mut()[0].pos = Vec3::ZERO;
    let acc = psys.gacc(Vec3::new(7.0e6, 0.0, 0.0), None);
    assert!(acc.x < 0.0, "acc.x = {} should be negative", acc.x);
}

#[test]
fn gacc_with_jcoeff() {
    let mut psys = load_toml(
        r#"
name = "Test"
star = "Earth"
mjd = 51544.5
primary = "Earth"
[[bodies]]
name = "Earth"
mass = 5.97e24
size = 6.371e6
fixed_pos = [0.0, 0.0, 0.0]
[bodies.gravity]
type = "Jcoeff"
values = [1.0826e-3]
"#,
        Path::new("/nonexistent"),
    );
    psys.bodies_mut()[0].pos = Vec3::ZERO;
    let acc = psys.gacc(Vec3::new(7.0e6, 3.0e6, 0.0), None);
    assert!(acc.y.abs() > 0.001, "J2 perturbation acc.y = {}", acc.y);
}

#[test]
fn grav_bodies_centers_primary() {
    let mut psys = load_toml(
        r#"
name = "Test"
star = "Earth"
mjd = 51544.5
primary = "Earth"
[[bodies]]
name = "Earth"
mass = 5.97e24
size = 6.371e6
fixed_pos = [1.0e11, 0.0, 0.0]
"#,
        Path::new("/nonexistent"),
    );
    psys.bodies_mut()[0].pos = Vec3::new(1.0e11, 0.0, 0.0);
    let grav = psys.grav_bodies();
    assert_eq!(grav.len(), 1);
    assert!(grav[0].pos.length() < 1.0);
    assert!((grav[0].mass - 5.97e24).abs() < 1e10);
}

#[test]
fn fake_sun_absent_from_grav_and_fixed_after_advance() {
    let path = orbitx_config::resolve_scenario_spec("earth").unwrap();
    let scn = PlanetaryScenario::from_file(&path).unwrap();
    let mut psys = PlanetarySystem::load(&scn, Path::new("/nonexistent")).unwrap();
    assert_eq!(psys.bodies().len(), 2);
    assert!(psys.bodies().iter().any(|b| b.name == "Sun"));
    let grav = psys.grav_bodies();
    assert_eq!(grav.len(), 1);
    assert!((grav[0].mass - 5.973698968e24).abs() / 5.973698968e24 < 1e-9);
    let sun = psys.bodies().iter().find(|b| b.name == "Sun").unwrap().pos;
    psys.advance(10.0);
    psys.update();
    let sun2 = psys.bodies().iter().find(|b| b.name == "Sun").unwrap().pos;
    assert!((sun2 - sun).length() < 1.0);
    let earth = psys.bodies().iter().find(|b| b.name == "Earth").unwrap().pos;
    assert!(earth.length() < 1.0);
    assert!(psys.primary_surface().sid_rot_period > 80_000.0);
}

#[test]
fn rotation_off_surface_period_is_zero() {
    let psys = load_toml(
        r#"
name = "Test"
star = "Sun"
mjd = 51544.5
primary = "Earth"
[[bodies]]
name = "Earth"
mass = 5.97e24
size = 6.371e6
fixed_pos = [0.0, 0.0, 0.0]
[bodies.rotation]
enabled = false
sid_rot_period = 86164.0
sid_rot_offset = 0.0
obliquity = 0.0
lan = 0.0
lan_mjd = 51544.5
"#,
        Path::new("/nonexistent"),
    );
    assert_eq!(psys.primary_surface().sid_rot_period, 0.0);
}

#[test]
fn moon_ephemeris_moves() {
    let data = resolve_ephemeris_data(None);
    let mut psys = load_toml(
        r#"
name = "EarthMoon"
star = "Sun"
mjd = 51544.5
primary = "Earth"
[[bodies]]
name = "Earth"
mass = 5.973698968e24
size = 6.37101e6
fixed_pos = [0.0, 0.0, 0.0]
[[bodies]]
name = "Moon"
mass = 7.3477e22
size = 1.738e6
[bodies.ephemeris]
type = "Elp82"
dat_file = "ELP82.dat"
prec = 1e-6
parents = [["Moon", "Earth"]]
"#,
        &data,
    );
    let p0 = psys.bodies().iter().find(|b| b.name == "Moon").unwrap().pos;
    psys.advance(10.0);
    psys.update();
    let p1 = psys.bodies().iter().find(|b| b.name == "Moon").unwrap().pos;
    assert!((p1 - p0).length() > 1.0e6, "moon moved {}", (p1 - p0).length());
}

#[test]
fn dynamical_sun_almost_cancels_at_origin() {
    let psys = load_toml(
        r#"
name = "Test"
star = "Sun"
mjd = 51544.5
primary = "Earth"
[[bodies]]
name = "Earth"
mass = 5.973698968e24
size = 6.37101e6
fixed_pos = [0.0, 0.0, 0.0]
[[bodies]]
name = "Sun"
mass = 1.9885e30
size = 6.96e8
dynamics = true
fixed_pos = [1.4959787e11, 0.0, 0.0]
"#,
        Path::new("/nonexistent"),
    );
    let grav = psys.grav_bodies();
    assert_eq!(grav.len(), 2);
    let primary = psys.primary_grav_index();
    let sun_i = if primary == 0 { 1 } else { 0 };
    let ship = Vec3::new(7.0e6, 0.0, 0.0);
    let sun_at_origin = gacc_nbody(Vec3::ZERO, &grav, Some(primary));
    assert!(
        sun_at_origin.length() > 1e-3,
        "sun pull at earth = {}",
        sun_at_origin.length()
    );
    let rel = gacc_nbody(ship, &grav, None) - sun_at_origin;
    let earth_only = gacc_nbody(ship, &grav, Some(sun_i));
    assert!(
        (rel - earth_only).length() < 1e-4,
        "sun term should cancel down to tide, residual {}",
        (rel - earth_only).length()
    );
}
