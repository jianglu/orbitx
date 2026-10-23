//! Host-injected ground contact policy (P5.1).
//!
//! Runtime produces [`GroundContact`]; vessel / crash only consume it.
//! This module must **not** depend on `orbitx-surface`.

use std::sync::Arc;

use orbitx_dynamics::{to_body, to_inertial};
use orbitx_math::{Matrix3, Vec3};

/// 地表类别（陆/水），决定触地结果（着陆 vs 坠入水面）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceKind {
    Land,
    Water,
}

/// 一次地表采样：地心半径 + 地面法向 + 实际命中层级 + 类别。
#[derive(Debug, Clone, Copy)]
pub struct GroundSample {
    /// 地心地面半径（m）。
    pub radius_m: f64,
    /// 地面法向（**惯性系**单位向量）。
    pub normal: Vec3,
    /// 实际命中的全球包层级（回退后）；覆盖网格/光滑球为 None。
    pub level_used: Option<u32>,
    /// 水陆类别。
    pub kind: SurfaceKind,
}

/// 体固系地心单位向量：纬度 = +Y 北、经度从 +X 向 +Z。
///
/// 经纬只定义在行星体固系；惯性位置须先 `to_inertial(R, ·)`。
pub fn geocentric_up(lat_deg: f64, lon_deg: f64) -> Vec3 {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    Vec3::new(
        lat.cos() * lon.cos(),
        lat.sin(),
        lat.cos() * lon.sin(),
    )
}

/// Sample geocentric ground radius [m] at geographic degrees.
pub trait GroundSampler: Send + Sync {
    fn ground_radius_m(&self, lat_deg: f64, lon_deg: f64) -> f64;

    /// Surface class at geographic degrees. Default: Land (e.g. sphere fallback samplers).
    fn surface_kind_at(&self, _lat_deg: f64, _lon_deg: f64) -> SurfaceKind {
        SurfaceKind::Land
    }

    /// 富采样（法向/实际层级）。默认：法向 = 径向，层级 None。
    fn sample_at(&self, lat_deg: f64, lon_deg: f64) -> GroundSample {
        GroundSample {
            radius_m: self.ground_radius_m(lat_deg, lon_deg),
            normal: geocentric_up(lat_deg, lon_deg),
            level_used: None,
            kind: self.surface_kind_at(lat_deg, lon_deg),
        }
    }
}

/// Per-tick ground policy injected by the host (Runtime).
#[derive(Clone, Default)]
pub enum GroundContact {
    /// Surface scheduling off: no DEM touchdown forces, no terrain crash.
    #[default]
    Off,
    /// Surface active but tiles not ready: smooth sphere placeholder.
    Sphere { radius_m: f64 },
    /// Working set ready: query contact height via sampler（经纬按体固系）。
    Query {
        sampler: Arc<dyn GroundSampler>,
        /// 体固 → 惯性旋转矩阵（与 [`orbitx_dynamics::PlanetOrientation::rot`] 同源）。
        rot: Matrix3,
    },
}

impl GroundContact {
    /// 构造 Query（`rot = I` 时体固=惯性，便于测例）。
    pub fn query(sampler: Arc<dyn GroundSampler>, rot: Matrix3) -> Self {
        Self::Query { sampler, rot }
    }
}

impl std::fmt::Debug for GroundContact {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Off => write!(f, "GroundContact::Off"),
            Self::Sphere { radius_m } => f
                .debug_struct("GroundContact::Sphere")
                .field("radius_m", radius_m)
                .finish(),
            Self::Query { rot, .. } => f
                .debug_struct("GroundContact::Query")
                .field("rot", rot)
                .finish(),
        }
    }
}

impl GroundContact {
    /// Effective ground radius at a world-space (inertial) point, if contact is active.
    ///
    /// Returns `None` when [`GroundContact::Off`].
    pub fn radius_at_world(&self, world_pos: Vec3) -> Option<f64> {
        match self {
            Self::Off => None,
            Self::Sphere { radius_m } => Some(*radius_m),
            Self::Query { sampler, rot } => {
                let (lat, lon) = geocentric_lat_lon_deg(to_body(*rot, world_pos));
                Some(sampler.ground_radius_m(lat, lon))
            }
        }
    }

    /// Ground radius at assembly CG (or sphere constant). `None` if Off.
    pub fn radius_at_cg(&self, cg_pos: Vec3) -> Option<f64> {
        self.radius_at_world(cg_pos)
    }

    /// Surface kind at a world-space point. `None` when [`GroundContact::Off`]; Sphere is Land.
    pub fn surface_kind_at_world(&self, world_pos: Vec3) -> Option<SurfaceKind> {
        match self {
            Self::Off => None,
            Self::Sphere { .. } => Some(SurfaceKind::Land),
            Self::Query { sampler, rot } => {
                let (lat, lon) = geocentric_lat_lon_deg(to_body(*rot, world_pos));
                Some(sampler.surface_kind_at(lat, lon))
            }
        }
    }

    /// 富采样（半径/法向/实际层级/类别）。`None` when [`GroundContact::Off`].
    pub fn sample_at_world(&self, world_pos: Vec3) -> Option<GroundSample> {
        match self {
            Self::Off => None,
            Self::Sphere { radius_m } => {
                let r = world_pos.length().max(1e-9);
                Some(GroundSample {
                    radius_m: *radius_m,
                    normal: world_pos * (1.0 / r),
                    level_used: None,
                    kind: SurfaceKind::Land,
                })
            }
            Self::Query { sampler, rot } => {
                let (lat, lon) = geocentric_lat_lon_deg(to_body(*rot, world_pos));
                let mut s = sampler.sample_at(lat, lon);
                // 法向由体固经纬给出；转回惯性系与世界位姿一致。
                s.normal = to_inertial(*rot, s.normal);
                Some(s)
            }
        }
    }
}

/// 体固系经纬：纬度 = +Y 北、经度从 +X 向 +Z。
///
/// `pos` 须已在行星体固系；惯性点请先 [`to_body`]。
pub fn geocentric_lat_lon_deg(pos: Vec3) -> (f64, f64) {
    let r = pos.length().max(1e-9);
    let lat = (pos.y / r).clamp(-1.0, 1.0).asin().to_degrees();
    let lon = pos.z.atan2(pos.x).to_degrees();
    (lat, lon)
}

/// 惯性系点 → 体固经纬（经 [`to_body`]）。
pub fn geocentric_lat_lon_deg_orient(pos_inertial: Vec3, rot: Matrix3) -> (f64, f64) {
    geocentric_lat_lon_deg(to_body(rot, pos_inertial))
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_dynamics::{surface_inertial_velocity, PlanetOrientation, RotationState};
    use orbitx_math::PI2;

    #[test]
    fn lat_lon_roundtrip_equator() {
        let r = 6_371_010.0;
        let lat0 = 0.0_f64;
        let lon0 = 90.0_f64;
        let lat_r = lat0.to_radians();
        let lon_r = lon0.to_radians();
        let pos = Vec3::new(
            r * lat_r.cos() * lon_r.cos(),
            r * lat_r.sin(),
            r * lat_r.cos() * lon_r.sin(),
        );
        let (lat, lon) = geocentric_lat_lon_deg(pos);
        assert!((lat - lat0).abs() < 1e-6);
        assert!((lon - lon0).abs() < 1e-6);
    }

    #[test]
    fn off_returns_none() {
        assert!(GroundContact::Off
            .radius_at_cg(Vec3::new(0.0, 0.0, 6.4e6))
            .is_none());
    }

    #[test]
    fn sphere_constant() {
        let g = GroundContact::Sphere {
            radius_m: 6_371_010.0,
        };
        assert!((g.radius_at_cg(Vec3::new(1.0, 2.0, 3.0)).unwrap() - 6_371_010.0).abs() < 1e-9);
    }

    #[test]
    fn corotating_body_fixed_lon_stable() {
        // 简化球 R=I、ω=+Y：共转后体固 lon 不变。
        let r = 6_371_000.0;
        let period = 86_164.0;
        let omega = Vec3::new(0.0, PI2 / period, 0.0);
        let t = 60.0;
        let pos0 = Vec3::new(r, 0.0, 0.0);
        let (lat0, lon0) = geocentric_lat_lon_deg(pos0);
        // 惯性位 ≈ 积分 ω×r：绕 +Y 转 +ωt
        let angle = omega.y * t;
        let c = angle.cos();
        let s = angle.sin();
        let pos_t = Vec3::new(pos0.x * c + pos0.z * s, pos0.y, -pos0.x * s + pos0.z * c);
        let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        let (lat_i, lon_i) = geocentric_lat_lon_deg(pos_t);
        let (lat_b, lon_b) = geocentric_lat_lon_deg_orient(pos_t, rot);
        assert!((lat_i - lat0).abs() < 1e-9);
        assert!(
            (lon_i - lon0).abs() > 0.1,
            "inertial lon should drift, got {lon_i} vs {lon0}"
        );
        assert!((lat_b - lat0).abs() < 1e-6);
        assert!(
            (lon_b - lon0).abs() < 1e-6,
            "body-fixed lon stable: {lon_b} vs {lon0}"
        );
        let _ = surface_inertial_velocity(pos0, omega);
    }

    #[test]
    fn corotating_with_obliquity_lon_stable() {
        let cfg = orbitx_config::RotationConfig {
            enabled: true,
            sid_rot_period: 86164.10132,
            sid_rot_offset: 0.0,
            obliquity: 0.4090928023,
            lan: 0.0,
            lan_mjd: 51544.5,
            precession_period: 0.0,
            precession_obliquity: 0.0,
            precession_lan: 0.0,
        };
        let mut state = RotationState::from_config(&cfg);
        let o0 = PlanetOrientation::from_rotation(&state);
        let r_body = Vec3::new(6_371_000.0, 0.0, 0.0);
        let (lat0, lon0) = geocentric_lat_lon_deg(r_body);
        state.update_rotation(600.0);
        let o1 = PlanetOrientation::from_rotation(&state);
        let pos_inert = to_inertial(o1.rot, r_body);
        let (lat_b, lon_b) = geocentric_lat_lon_deg_orient(pos_inert, o1.rot);
        assert!((lat_b - lat0).abs() < 1e-6);
        assert!((lon_b - lon0).abs() < 1e-6);
        // 惯性方向应相对 t=0 有可见差（黄赤交角 + 自转）
        let pos0_inert = to_inertial(o0.rot, r_body);
        assert!((pos_inert - pos0_inert).length() > 100.0);
    }

    #[test]
    fn query_uses_rot_for_radius() {
        struct LonBump;
        impl GroundSampler for LonBump {
            fn ground_radius_m(&self, _lat: f64, lon: f64) -> f64 {
                6_371_000.0 + lon // 用 lon 当假高程，便于断言走了体固经纬
            }
        }
        let r = 6_371_000.0;
        let angle = 0.2_f64;
        let c = angle.cos();
        let s = angle.sin();
        let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        let pos0 = Vec3::new(r, 0.0, 0.0);
        let pos_t = to_inertial(rot, pos0);
        let g = GroundContact::query(Arc::new(LonBump), rot);
        let (_, lon_b) = geocentric_lat_lon_deg_orient(pos_t, rot);
        let expect = 6_371_000.0 + lon_b;
        assert!((g.radius_at_world(pos_t).unwrap() - expect).abs() < 1e-6);
    }
}
