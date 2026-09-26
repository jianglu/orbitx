//! 发射台 / scenario 初始状态（地心系，地球在原点）。

use orbitx_config::ScenarioConfig;
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{surface_inertial_velocity, Assembly, StageSpec};

/// 体 +Y（头部）对齐径向 up。
pub fn launch_attitude(up: Vec3) -> (Matrix3, Quat) {
    let ref_axis = if up.y.abs() < 0.9 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let by = up;
    let r = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(r);
    (r, q)
}

pub fn initial_state(
    stages: &[StageSpec],
    scenario: Option<&ScenarioConfig>,
    earth_r: f64,
    sid_period: f64,
) -> Result<StateVectors, String> {
    let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;

    if let Some(scn) = scenario {
        if let Some(ship) = scn.ships.first() {
            if ship.status == "orbiting" {
                if let (Some(rpos), Some(rvel)) = (ship.rpos, ship.rvel) {
                    let base = Vec3::new(rpos[0], rpos[1], rpos[2]);
                    let vel = Vec3::new(rvel[0], rvel[1], rvel[2]);
                    let radial = base * (1.0 / base.length().max(1e-3));
                    let pos = base + radial * half_h;
                    let (init_r, init_q) = launch_attitude(radial);
                    return Ok(StateVectors {
                        pos,
                        vel,
                        r: init_r,
                        q: init_q,
                        ..Default::default()
                    });
                }
                return Err("scenario orbiting ship needs rpos and rvel".into());
            }
            if ship.status == "landed" {
                if let (Some(lng), Some(lat)) = (ship.longitude, ship.latitude) {
                    let lng_r = lng.to_radians();
                    let lat_r = lat.to_radians();
                    let pos = Vec3::new(
                        earth_r * lat_r.cos() * lng_r.cos(),
                        earth_r * lat_r.sin(),
                        earth_r * lat_r.cos() * lng_r.sin(),
                    );
                    let radial = pos * (1.0 / pos.length().max(1e-3));
                    let pos = pos + radial * half_h;
                    let (init_r, init_q) = launch_attitude(radial);
                    let vel = surface_inertial_velocity(pos, sid_period);
                    return Ok(StateVectors {
                        pos,
                        vel,
                        r: init_r,
                        q: init_q,
                        ..Default::default()
                    });
                }
                // landed 无经纬度 → 默认发射台
            }
        }
    }

    // 默认：赤道附近竖立发射台（地心系，z = R）。
    let launch = Vec3::new(0.0, 0.0, earth_r);
    let radial = launch * (1.0 / launch.length());
    let pos = launch + radial * half_h;
    let (init_r, init_q) = launch_attitude(radial);
    let vel = surface_inertial_velocity(pos, sid_period);
    Ok(StateVectors {
        pos,
        vel,
        r: init_r,
        q: init_q,
        ..Default::default()
    })
}

pub fn apply_fuel_levels(asm: &mut Assembly, stages: &[StageSpec], scn: &ScenarioConfig) {
    let Some(ship) = scn.ships.first() else {
        return;
    };
    let Some(ref fuel_levels) = ship.fuel_level else {
        return;
    };
    for (i, &level) in fuel_levels.iter().enumerate() {
        if i < asm.vessels.len() && i < stages.len() {
            asm.vessels[i].fuel_mass = stages[i].fuel_mass * level;
        }
    }
}
