//! Host-injected ground contact policy (P5.1).
//!
//! Runtime produces [`GroundContact`]; vessel / crash only consume it.
//! This module must **not** depend on `orbitx-surface`.

use std::sync::Arc;

use orbitx_math::Vec3;

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
    /// 地面法向（地心系单位向量）。
    pub normal: Vec3,
    /// 实际命中的全球包层级（回退后）；覆盖网格/光滑球为 None。
    pub level_used: Option<u32>,
    /// 水陆类别。
    pub kind: SurfaceKind,
}

/// 地心单位向量：纬度 = Y 北、经度从 +X 向 +Z。
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
        /// 相对会话起点的 +Y 自转角 [rad]；与共转 `ω=+Y` 同口径。
        spin_rad: f64,
    },
}

impl GroundContact {
    /// 构造 Query（`spin_rad = 0` 时与摆放/旧惯性经纬一致）。
    pub fn query(sampler: Arc<dyn GroundSampler>, spin_rad: f64) -> Self {
        Self::Query { sampler, spin_rad }
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
            Self::Query { spin_rad, .. } => f
                .debug_struct("GroundContact::Query")
                .field("spin_rad", spin_rad)
                .finish(),
        }
    }
}

impl GroundContact {
    /// Effective ground radius at a world-space point, if contact is active.
    ///
    /// Returns `None` when [`GroundContact::Off`].
    pub fn radius_at_world(&self, world_pos: Vec3) -> Option<f64> {
        match self {
            Self::Off => None,
            Self::Sphere { radius_m } => Some(*radius_m),
            Self::Query { sampler, spin_rad } => {
                let (lat, lon) = geocentric_lat_lon_deg_spin(world_pos, *spin_rad);
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
            Self::Query { sampler, spin_rad } => {
                let (lat, lon) = geocentric_lat_lon_deg_spin(world_pos, *spin_rad);
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
            Self::Query { sampler, spin_rad } => {
                let (lat, lon) = geocentric_lat_lon_deg_spin(world_pos, *spin_rad);
                let mut s = sampler.sample_at(lat, lon);
                // 法向由体固经纬给出；转回惯性系与世界位姿一致。
                s.normal = pos_inertial_from_body(s.normal, *spin_rad);
                Some(s)
            }
        }
    }
}

/// 会话自转角 [rad]：`2π · sim_t / T`（与共转 `ω=+Y` 一致）。
/// `sid_rot_period ≤ 0` 时返回 0。
pub fn spin_y_rad(sim_t_s: f64, sid_rot_period: f64) -> f64 {
    if sid_rot_period.abs() < 1e-9 {
        return 0.0;
    }
    std::f64::consts::TAU * sim_t_s / sid_rot_period
}

/// 惯性 → 体固：绕 +Y 转 `−spin`（撤销共转积累的经度东/西移）。
///
/// 共转 `v = ωŶ × r` 使赤道点沿 −Z 走；积分等价于 `R_y(+spin)`，故体固取 `R_y(−spin)`。
pub fn pos_body_fixed(pos: Vec3, spin_rad: f64) -> Vec3 {
    if spin_rad.abs() < 1e-15 {
        return pos;
    }
    let c = spin_rad.cos();
    let s = spin_rad.sin();
    Vec3::new(
        pos.x * c - pos.z * s,
        pos.y,
        pos.x * s + pos.z * c,
    )
}

/// 体固 → 惯性：`R_y(+spin)`。
pub fn pos_inertial_from_body(pos: Vec3, spin_rad: f64) -> Vec3 {
    if spin_rad.abs() < 1e-15 {
        return pos;
    }
    let c = spin_rad.cos();
    let s = spin_rad.sin();
    Vec3::new(
        pos.x * c + pos.z * s,
        pos.y,
        -pos.x * s + pos.z * c,
    )
}

/// Inverse of [`crate`] launch placement: Y = north, lon from +X toward +Z.
///
/// Matches `orbitx-runtime` `place::initial_state` landed placement.
/// 惯性系经纬（不含自转）；地表采样请用 [`geocentric_lat_lon_deg_spin`]。
pub fn geocentric_lat_lon_deg(pos: Vec3) -> (f64, f64) {
    let r = pos.length().max(1e-9);
    let lat = (pos.y / r).clamp(-1.0, 1.0).asin().to_degrees();
    let lon = pos.z.atan2(pos.x).to_degrees();
    (lat, lon)
}

/// 体固系经纬：先 [`pos_body_fixed`] 再 [`geocentric_lat_lon_deg`]。
pub fn geocentric_lat_lon_deg_spin(pos: Vec3, spin_rad: f64) -> (f64, f64) {
    geocentric_lat_lon_deg(pos_body_fixed(pos, spin_rad))
}

#[cfg(test)]
mod tests {
    use super::*;

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
        // 赤道 +X：共转 v = −ω R Ẑ，积分 ≈ R_y(+spin)；体固 lon 应不变。
        let r = 6_371_000.0;
        let period = 86_164.0;
        let t = 60.0;
        let spin = spin_y_rad(t, period);
        let pos0 = Vec3::new(r, 0.0, 0.0);
        let (lat0, lon0) = geocentric_lat_lon_deg(pos0);
        // 惯性位：绕 +Y 转 +spin
        let pos_t = pos_inertial_from_body(pos0, spin);
        let (lat_i, lon_i) = geocentric_lat_lon_deg(pos_t);
        let (lat_b, lon_b) = geocentric_lat_lon_deg_spin(pos_t, spin);
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
    }

    #[test]
    fn query_uses_spin_for_radius() {
        struct LonBump;
        impl GroundSampler for LonBump {
            fn ground_radius_m(&self, _lat: f64, lon: f64) -> f64 {
                6_371_000.0 + lon // 用 lon 当假高程，便于断言走了体固经纬
            }
        }
        let r = 6_371_000.0;
        let spin = 0.2_f64;
        let pos0 = Vec3::new(r, 0.0, 0.0);
        let pos_t = pos_inertial_from_body(pos0, spin);
        let g = GroundContact::query(Arc::new(LonBump), spin);
        let (_, lon_b) = geocentric_lat_lon_deg_spin(pos_t, spin);
        let expect = 6_371_000.0 + lon_b;
        assert!((g.radius_at_world(pos_t).unwrap() - expect).abs() < 1e-6);
    }
}
