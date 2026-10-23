//! 发射台 / scenario 初始状态（地心系，地球在原点）。

use orbitx_config::ScenarioConfig;
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{geocentric_up, surface_inertial_velocity, StageSpec};

/// 体 +Y（头部）对齐径向 up。
pub fn launch_attitude(up: Vec3) -> (Matrix3, Quat) {
    attitude_for_axis(up, 0.0)
}

/// 体 +Y 对齐给定轴向，并绕轴旋转 heading（弧度，正向按右手则）。
pub fn attitude_for_axis(axis: Vec3, heading_rad: f64) -> (Matrix3, Quat) {
    let up = axis.unit();
    let ref_axis = if up.y.abs() < 0.9 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let mut bx = cross(up, ref_axis).unit();
    let mut bz = cross(bx, up).unit();
    if heading_rad.abs() > 0.0 {
        let (c, s) = (heading_rad.cos(), heading_rad.sin());
        let rotate = |v: Vec3| v * c + cross(up, v) * s;
        bx = rotate(bx);
        bz = rotate(bz);
    }
    let by = up;
    let r = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(r);
    (r, q)
}

/// 发射场当地 ENU 基（地心系）：(east, north, up)。与地表一致性规范一致。
pub fn enu_basis_geocentric(lat_deg: f64, lon_deg: f64) -> (Vec3, Vec3, Vec3) {
    let up = geocentric_up(lat_deg, lon_deg);
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let east = Vec3::new(-lon.sin(), 0.0, lon.cos());
    let north = Vec3::new(
        -lat.sin() * lon.cos(),
        lat.cos(),
        -lat.sin() * lon.sin(),
    );
    (east, north, up)
}

/// 台位支持面（火箭底部落点）地心半径：`R_elev + base + 台架高度`。
///
/// 发射场坐标系换算，不走地形采样：地形低于场坪时台架等效架空、
/// 高于场坪时采样侧截平（见 `surface_session`），两侧与本式自洽。
pub fn pad_support_radius_m(elev_radius_m: f64, base_elevation_m: f64, mount_height_m: f64) -> f64 {
    elev_radius_m + base_elevation_m + mount_height_m
}

/// 按发射场台位摆放：轴基点当地偏移 → 台位方向，支持面按发射场坐标系换算
/// （`R_elev + base_elevation_m + axis_offset_m[2]`，上分量只加一次），
/// 质心沿轴 +half_h 近似（装配后由会话按底部精确归零），姿态绕轴 heading，初速共转。
#[allow(clippy::too_many_arguments)]
pub fn initial_state_on_pad(
    stages: &[StageSpec],
    lat_deg: f64,
    lon_deg: f64,
    axis_offset_m: [f64; 3],
    axis_dir_enu: [f64; 3],
    heading_deg: f64,
    elev_radius_m: f64,
    base_elevation_m: f64,
    sid_period: f64,
) -> StateVectors {
    let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
    let (east, north, up) = enu_basis_geocentric(lat_deg, lon_deg);

    // 场坪平面上的台位点（东/北偏移）→ 方向；支持面半径沿该方向按坐标系换算。
    let pad_point = up * (elev_radius_m + base_elevation_m)
        + east * axis_offset_m[0]
        + north * axis_offset_m[1];
    let pad_dir = pad_point.unit();
    let support_point = pad_dir
        * pad_support_radius_m(elev_radius_m, base_elevation_m, axis_offset_m[2]);

    let axis_world = (east * axis_dir_enu[0] + north * axis_dir_enu[1] + up * axis_dir_enu[2]).unit();
    let pos = support_point + axis_world * half_h;
    let (init_r, init_q) = attitude_for_axis(axis_world, heading_deg.to_radians());
    let vel = surface_inertial_velocity(pos, sid_period);
    StateVectors {
        pos,
        vel,
        r: init_r,
        q: init_q,
        ..Default::default()
    }
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
