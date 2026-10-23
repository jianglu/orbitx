//! P5.1 surface package session: hold package, schedule prefetch, inject GroundContact.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use orbitx_math::Vec3;
use orbitx_surface::{enu_basis, LocalElevationGrid, SurfacePackage, WaterClass};
use orbitx_vessel::{
    geocentric_lat_lon_deg_spin, GroundContact, GroundSample, GroundSampler, SurfaceKind,
};
use tracing::warn;

/// 发射场高程覆盖规格（lp 解析后由会话装配；数据已在加载期校验）。
#[derive(Clone)]
pub struct ElevationOverrideSpec {
    pub origin_lat_deg: f64,
    pub origin_lon_deg: f64,
    pub cell_m: f64,
    pub width: usize,
    pub height: usize,
    pub data: Vec<f32>,
}

impl std::fmt::Debug for ElevationOverrideSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ElevationOverrideSpec")
            .field("origin_lat_deg", &self.origin_lat_deg)
            .field("origin_lon_deg", &self.origin_lon_deg)
            .field("cell_m", &self.cell_m)
            .field("width", &self.width)
            .field("height", &self.height)
            .field("cells", &self.data.len())
            .finish()
    }
}

/// Configuration for opening / running a surface session.
#[derive(Debug, Clone)]
pub struct SurfaceSessionConfig {
    /// 地表包所属天体名（经纬度/半径校验都归属该天体；空 = 主天体）。
    pub body_name: String,
    pub package_root: PathBuf,
    pub working_max_level: u32,
    pub pause_alt_m: f64,
    pub resume_alt_m: f64,
    pub prefetch_every: u64,
    pub inject_every: u64,
    pub work_radius_m: f64,
    pub extra_tiles_along_vel: u32,
    pub max_cached_tiles: usize,
    /// 发射场局部高程覆盖（硬切换；足迹内权威替换全球包）。
    pub elevation_override: Option<ElevationOverrideSpec>,
    /// 发射场底部高程（m，相对 `R_elev`；lp `base_elevation_m`）。
    /// 有覆盖网格时，足迹内地形高截平为 `min(高程, 本值)`（水面不处理）；
    /// 无网格时不做面状对齐，本值仅用于台位点超差告警。
    pub launchpad_base_elevation_m: Option<f64>,
    /// 发射场各台位轴基点经纬度（加载期与全球包高程比对，超差告警）。
    pub launchpad_pad_points: Vec<(f64, f64)>,
}

impl Default for SurfaceSessionConfig {
    fn default() -> Self {
        Self {
            body_name: String::new(),
            package_root: PathBuf::new(),
            working_max_level: u32::MAX,
            pause_alt_m: 30_000.0,
            resume_alt_m: 20_000.0,
            prefetch_every: 5,
            inject_every: 1,
            work_radius_m: 1_500.0,
            extra_tiles_along_vel: 2,
            max_cached_tiles: 64,
            elevation_override: None,
            launchpad_base_elevation_m: None,
            launchpad_pad_points: Vec::new(),
        }
    }
}

/// Active surface scheduling state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleState {
    Paused,
    Active,
}

/// Runtime-owned surface package + schedule.
pub struct SurfaceSession {
    pub config: SurfaceSessionConfig,
    pub package: Arc<SurfacePackage>,
    pub elev_radius_m: f64,
    pub working_max_level: u32,
    pub schedule: ScheduleState,
    /// True after at least one successful prefetch while Active.
    pub workset_ready: bool,
    tick: u64,
    /// Last injected contact (retained when `inject_every` skips a tick).
    last_inject: GroundContact,
    /// 发射场高程覆盖网格（硬切换组合的覆盖侧）。
    grid: Option<Arc<LocalElevationGrid>>,
    /// 相对会话起点的 +Y 自转角 [rad]；每 tick 由 runtime 写入。
    spin_rad: f64,
}

struct PackageSampler {
    package: Arc<SurfacePackage>,
    level: u32,
    grid: Option<Arc<LocalElevationGrid>>,
    /// 发射场底部高程：足迹内地形截平上限（水面不处理）。
    base_elevation_m: Option<f64>,
}

/// ENU 法向分量 → 地心系向量（查询点当地基）。
fn enu_normal_to_world(lat_deg: f64, lon_deg: f64, n_enu: [f64; 3]) -> Vec3 {
    let (e, n, u) = enu_basis(lat_deg, lon_deg);
    Vec3::new(
        e[0] * n_enu[0] + n[0] * n_enu[1] + u[0] * n_enu[2],
        e[1] * n_enu[0] + n[1] * n_enu[1] + u[1] * n_enu[2],
        e[2] * n_enu[0] + n[2] * n_enu[1] + u[2] * n_enu[2],
    )
    .unit()
}

impl GroundSampler for PackageSampler {
    fn ground_radius_m(&self, lat_deg: f64, lon_deg: f64) -> f64 {
        self.sample_at(lat_deg, lon_deg).radius_m
    }

    fn surface_kind_at(&self, lat_deg: f64, lon_deg: f64) -> SurfaceKind {
        // 足迹内恒陆地；足迹外走全球包分类。
        if let Some(ref g) = self.grid {
            if g.sample(lat_deg, lon_deg).is_some() {
                return SurfaceKind::Land;
            }
        }
        match self
            .package
            .sample_water_class_at(lat_deg, lon_deg, self.level)
        {
            WaterClass::Ocean | WaterClass::InlandWater | WaterClass::River => SurfaceKind::Water,
            WaterClass::Land => SurfaceKind::Land,
        }
    }

    fn sample_at(&self, lat_deg: f64, lon_deg: f64) -> GroundSample {
        // 硬切换：先问覆盖（足迹内权威替换），None 落全球包。
        if let Some(ref g) = self.grid {
            if let Some(s) = g.sample(lat_deg, lon_deg) {
                // 发射场局部对齐：足迹内地形高于场坪则截平（只截高、不填低）。
                let height_m = match self.base_elevation_m {
                    Some(base) => s.height_m.min(base),
                    None => s.height_m,
                };
                return GroundSample {
                    radius_m: self.package.r_elev_m + height_m,
                    normal: enu_normal_to_world(lat_deg, lon_deg, s.normal_enu),
                    level_used: None,
                    kind: SurfaceKind::Land,
                };
            }
        }
        let d = self
            .package
            .sample_contact_detail_at(lat_deg, lon_deg, self.level);
        GroundSample {
            radius_m: self.package.r_elev_m + d.height_m,
            normal: enu_normal_to_world(lat_deg, lon_deg, d.normal_enu),
            level_used: Some(d.level_used),
            kind: match d.kind {
                WaterClass::Land => SurfaceKind::Land,
                WaterClass::Ocean | WaterClass::InlandWater | WaterClass::River => {
                    SurfaceKind::Water
                }
            },
        }
    }
}

impl SurfaceSession {
    /// Open package; clamp working level; check `|R_elev − body_size| ≤ 0.5`.
    pub fn open(config: SurfaceSessionConfig, body_size_m: f64) -> Result<Self, String> {
        let package = Arc::new(SurfacePackage::open_with_cache_cap(
            &config.package_root,
            config.max_cached_tiles,
        )?);
        let elev = package.r_elev_m;
        if (elev - body_size_m).abs() > 0.5 {
            return Err(format!(
                "surface R_elev_m={elev} vs body.size={body_size_m} (eps 0.5 m)"
            ));
        }
        let pkg_max = package
            .max_level("terrain")
            .max(package.max_level("water_surface"))
            .max(package.max_level("water_class"));
        let working_max_level = config.working_max_level.min(pkg_max);
        let grid = match config.elevation_override {
            Some(ref spec) => {
                let g = LocalElevationGrid::new(
                    spec.origin_lat_deg,
                    spec.origin_lon_deg,
                    elev,
                    spec.cell_m,
                    spec.width,
                    spec.height,
                    spec.data.clone(),
                )?;
                // 数据契约检查：最外圈应已向全球高程收敛；超差仅告警不拦截。
                let mut max_diff = 0.0f64;
                for (lat, lon, h) in g.border_nodes(256) {
                    let global = package.sample_contact_height_at(lat, lon, working_max_level);
                    max_diff = max_diff.max((h - global).abs());
                }
                if max_diff > 5.0 {
                    warn!(
                        max_diff_m = max_diff,
                        "elevation override border deviates from global package by >5 m (edge convergence is a data-generation contract)"
                    );
                }
                Some(Arc::new(g))
            }
            None => None,
        };
        // 发射场数据自检：各台位点全球高程与声明的场坪高超差仅告警（lp 可能填错）。
        if let Some(base) = config.launchpad_base_elevation_m {
            for &(lat, lon) in &config.launchpad_pad_points {
                let global = package.sample_contact_height_at(lat, lon, working_max_level);
                let diff = (global - base).abs();
                if diff > 5.0 {
                    warn!(
                        pad_lat = lat,
                        pad_lon = lon,
                        global_m = global,
                        base_m = base,
                        "launchpad pad point elevation deviates from base_elevation_m by >5 m (check lp data)"
                    );
                }
            }
        }
        Ok(Self {
            config,
            package,
            elev_radius_m: elev,
            working_max_level,
            schedule: ScheduleState::Active,
            workset_ready: false,
            tick: 0,
            last_inject: GroundContact::Sphere {
                radius_m: elev,
            },
            grid,
            spin_rad: 0.0,
        })
    }

    fn make_sampler(&self) -> Arc<PackageSampler> {
        Arc::new(PackageSampler {
            package: Arc::clone(&self.package),
            level: self.working_max_level,
            grid: self.grid.clone(),
            base_elevation_m: self.config.launchpad_base_elevation_m,
        })
    }

    /// 组合采样器（覆盖 + 全球包硬切换），供发射台摆放等步进外查询。
    /// 摆放在 `sim_t=0`，spin=0。
    pub fn placement_contact(&self) -> GroundContact {
        GroundContact::query(self.make_sampler(), 0.0)
    }

    /// 是否带发射场高程覆盖。
    pub fn has_override(&self) -> bool {
        self.grid.is_some()
    }

    pub fn package_root(&self) -> &Path {
        &self.config.package_root
    }

    /// Advance schedule clock; optionally prefetch from rocket state.
    ///
    /// `spin_rad`：相对会话起点的 +Y 自转角，预取/注入经纬均用体固系。
    pub fn on_tick(&mut self, pos: Vec3, vel: Vec3, spin_rad: f64) {
        self.spin_rad = spin_rad;
        self.tick = self.tick.wrapping_add(1);
        let r = pos.length().max(1e-9);
        let radial = pos * (1.0 / r);
        let alt = r - self.elev_radius_m;
        let v_radial = vel.x * radial.x + vel.y * radial.y + vel.z * radial.z;

        match self.schedule {
            ScheduleState::Active => {
                if alt > self.config.pause_alt_m && v_radial > 0.0 {
                    self.schedule = ScheduleState::Paused;
                    self.workset_ready = false;
                }
            }
            ScheduleState::Paused => {
                if alt < self.config.resume_alt_m || v_radial <= 0.0 {
                    self.schedule = ScheduleState::Active;
                    self.workset_ready = false;
                }
            }
        }

        if self.schedule != ScheduleState::Active {
            return;
        }

        let every = self.config.prefetch_every.max(1);
        let due = !self.workset_ready || self.tick % every == 0;
        if !due {
            return;
        }

        let (lat, lon) = geocentric_lat_lon_deg_spin(pos, self.spin_rad);
        let ring = self.config.extra_tiles_along_vel;
        if self
            .package
            .prefetch_around(lat, lon, self.working_max_level, ring)
            .is_ok()
        {
            self.workset_ready = true;
        }
        let speed = vel.length();
        if speed > 50.0 {
            let ahead = pos + vel * (self.config.work_radius_m / speed);
            let (alat, alon) = geocentric_lat_lon_deg_spin(ahead, self.spin_rad);
            let _ = self
                .package
                .prefetch_around(alat, alon, self.working_max_level, ring);
        }
    }

    /// Build GroundContact for this tick (three-state inject).
    ///
    /// When `inject_every > 1`, retains the previous sampler between inject ticks
    /// but always refreshes `spin_rad`；Off / Sphere transitions still apply immediately.
    pub fn inject(&mut self) -> GroundContact {
        let every = self.config.inject_every.max(1);
        let due = self.tick % every == 0;
        let need_query_refresh = self.workset_ready
            && !matches!(self.last_inject, GroundContact::Query { .. });
        let contact = match self.schedule {
            ScheduleState::Paused => GroundContact::Off,
            ScheduleState::Active if !self.workset_ready => GroundContact::Sphere {
                radius_m: self.elev_radius_m,
            },
            ScheduleState::Active if due || need_query_refresh => {
                GroundContact::query(self.make_sampler(), self.spin_rad)
            }
            ScheduleState::Active => {
                // 复用采样器，刷新 spin。
                let refreshed = match &self.last_inject {
                    GroundContact::Query { sampler, .. } => {
                        GroundContact::query(Arc::clone(sampler), self.spin_rad)
                    }
                    other => other.clone(),
                };
                self.last_inject = refreshed.clone();
                return refreshed;
            }
        };
        self.last_inject = contact.clone();
        contact
    }

    /// Sphere placeholder while active but caller wants explicit placeholder.
    pub fn sphere_placeholder(&self) -> GroundContact {
        GroundContact::Sphere {
            radius_m: self.elev_radius_m,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_vessel::GroundContact;

    fn pkg_low() -> Option<PathBuf> {
        let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../../simrocket_earth/packages/low");
        if p.join("manifest.toml").is_file() {
            Some(p)
        } else {
            None
        }
    }

    #[test]
    fn three_states_and_contact_height() {
        let Some(root) = pkg_low() else {
            eprintln!("skip three_states_and_contact_height: packages/low missing");
            return;
        };
        let elev = 6_371_010.0;
        let cfg = SurfaceSessionConfig {
            package_root: root,
            working_max_level: 0,
            prefetch_every: 1,
            inject_every: 1,
            ..Default::default()
        };
        // Match package R_elev
        let mut sess = SurfaceSession::open(cfg, elev).unwrap();
        assert!(!sess.workset_ready);

        // Near surface, outward radial vel small → stay Active; first tick prefetches.
        let pos = Vec3::new(elev + 100.0, 0.0, 0.0);
        let vel = Vec3::new(-10.0, 0.0, 0.0); // inward
        sess.on_tick(pos, vel, 0.0);
        assert_eq!(sess.schedule, ScheduleState::Active);
        assert!(sess.workset_ready);
        let g = sess.inject();
        assert!(
            matches!(g, GroundContact::Query { .. }),
            "expected Query after prefetch, got {g:?}"
        );
        let (lat, lon) = geocentric_lat_lon_deg_spin(pos, 0.0);
        let r_query = g.radius_at_world(pos).unwrap();
        let expect = elev + sess.package.sample_contact_height_at(lat, lon, 0);
        assert!(
            (r_query - expect).abs() < 1e-3,
            "Query radius {r_query} vs package {expect}"
        );

        // High altitude + outward → Paused → Off
        let pos_hi = Vec3::new(elev + 40_000.0, 0.0, 0.0);
        let vel_out = Vec3::new(100.0, 0.0, 0.0);
        sess.on_tick(pos_hi, vel_out, 0.0);
        assert_eq!(sess.schedule, ScheduleState::Paused);
        assert!(matches!(sess.inject(), GroundContact::Off));

        // Descend below resume → Active; Sphere path when not ready.
        let pos_mid = Vec3::new(elev + 15_000.0, 0.0, 0.0);
        let vel_in = Vec3::new(-50.0, 0.0, 0.0);
        sess.workset_ready = false;
        sess.schedule = ScheduleState::Paused;
        sess.on_tick(pos_mid, vel_in, 0.0);
        assert_eq!(sess.schedule, ScheduleState::Active);
        sess.workset_ready = false;
        assert!(matches!(
            sess.inject(),
            GroundContact::Sphere { .. }
        ));
    }

    #[test]
    fn corotating_body_fixed_sample_radius_stable() {
        // 共转位移后：体固 spin 采样高度应与起点一致（惯性 lon 已变）。
        let Some(root) = pkg_low() else {
            eprintln!("skip corotating_body_fixed: packages/low missing");
            return;
        };
        let elev = {
            let p = orbitx_surface::SurfacePackage::open(&root).unwrap();
            p.r_elev_m
        };
        let cfg = SurfaceSessionConfig {
            package_root: root,
            working_max_level: 0,
            prefetch_every: 1,
            inject_every: 1,
            ..Default::default()
        };
        let mut sess = SurfaceSession::open(cfg, elev).unwrap();
        let period = 86_164.0;
        let t = 60.0;
        let spin = orbitx_vessel::spin_y_rad(t, period);
        let pos0 = Vec3::new(elev + 50.0, 0.0, 0.0);
        let pos_t = orbitx_vessel::pos_inertial_from_body(pos0, spin);

        sess.on_tick(pos0, Vec3::new(-1.0, 0.0, 0.0), 0.0);
        let g0 = sess.inject();
        let r0 = g0.radius_at_world(pos0).unwrap();

        sess.on_tick(pos_t, Vec3::new(-1.0, 0.0, 0.0), spin);
        let g1 = sess.inject();
        let r1 = g1.radius_at_world(pos_t).unwrap();
        assert!(
            (r0 - r1).abs() < 1e-3,
            "body-fixed radius should match: r0={r0} r1={r1}"
        );
        // 若误用 spin=0 采惯性 lon，半径通常会偏（全球包经度变化）。
        let g_wrong = GroundContact::query(
            match &g1 {
                GroundContact::Query { sampler, .. } => Arc::clone(sampler),
                _ => panic!("expected Query"),
            },
            0.0,
        );
        let r_inertial_lon = g_wrong.radius_at_world(pos_t).unwrap();
        // 不强制不相等（光滑区可能碰巧相同），但体固路径必须自洽。
        let _ = r_inertial_lon;
    }

    #[test]
    fn override_hard_switch_inside_outside() {
        let Some(root) = pkg_low() else {
            eprintln!("skip override_hard_switch: packages/low missing");
            return;
        };
        let elev = 6_371_010.0;
        // 原点 (0°, 90°E) 印度洋：±500 m 足迹内恒 120 m 台面，足迹外全球包。
        let spec = ElevationOverrideSpec {
            origin_lat_deg: 0.0,
            origin_lon_deg: 90.0,
            cell_m: 100.0,
            width: 11,
            height: 11,
            data: vec![120.0; 121],
        };
        let cfg = SurfaceSessionConfig {
            package_root: root,
            working_max_level: 0,
            elevation_override: Some(spec),
            ..Default::default()
        };
        let sess = SurfaceSession::open(cfg, elev).unwrap();
        assert!(sess.has_override());
        let contact = sess.placement_contact();

        // 足迹内：覆盖值精确替换 + 陆地 + 无全球层级
        let inside = Vec3::new(0.0, 0.0, elev + 120.0);
        let s = contact.sample_at_world(inside).unwrap();
        assert!((s.radius_m - (elev + 120.0)).abs() < 1e-6, "radius {}", s.radius_m);
        assert_eq!(s.kind, SurfaceKind::Land);
        assert_eq!(s.level_used, None);
        assert!((s.normal.length() - 1.0).abs() < 1e-9);

        // 足迹外（东 ~11 km）：全球包值 + 层级回传
        let (lat_o, lon_o) = (0.0f64, 90.1f64);
        let dir = orbitx_vessel::geocentric_up(lat_o, lon_o);
        let outside = dir * elev;
        let s2 = contact.sample_at_world(outside).unwrap();
        let expect = elev + sess.package.sample_contact_height_at(lat_o, lon_o, 0);
        assert!(
            (s2.radius_m - expect).abs() < 1e-3,
            "outside radius {} vs global {expect}",
            s2.radius_m
        );
        assert!(s2.level_used.is_some());
    }

    #[test]
    fn base_elevation_clamps_inside_footprint_only() {
        let Some(root) = pkg_low() else {
            eprintln!("skip base_elevation_clamps: packages/low missing");
            return;
        };
        let elev = 6_371_010.0;
        let spec = ElevationOverrideSpec {
            origin_lat_deg: 0.0,
            origin_lon_deg: 90.0,
            cell_m: 100.0,
            width: 11,
            height: 11,
            data: vec![120.0; 121],
        };
        let inside = Vec3::new(0.0, 0.0, elev + 120.0);

        // base=50 < 网格 120：足迹内截平到 50
        let cfg = SurfaceSessionConfig {
            package_root: root.clone(),
            working_max_level: 0,
            elevation_override: Some(spec.clone()),
            launchpad_base_elevation_m: Some(50.0),
            ..Default::default()
        };
        let sess = SurfaceSession::open(cfg, elev).unwrap();
        let s = sess.placement_contact().sample_at_world(inside).unwrap();
        assert!(
            (s.radius_m - (elev + 50.0)).abs() < 1e-6,
            "clamped radius {}",
            s.radius_m
        );

        // base=200 > 网格 120：只截高不填低，足迹内仍 120
        let cfg = SurfaceSessionConfig {
            package_root: root.clone(),
            working_max_level: 0,
            elevation_override: Some(spec),
            launchpad_base_elevation_m: Some(200.0),
            ..Default::default()
        };
        let sess = SurfaceSession::open(cfg, elev).unwrap();
        let s = sess.placement_contact().sample_at_world(inside).unwrap();
        assert!(
            (s.radius_m - (elev + 120.0)).abs() < 1e-6,
            "unclamped radius {}",
            s.radius_m
        );

        // 无覆盖网格：不做面状对齐，采样与全球包一致（base 只作台位点告警）
        let cfg = SurfaceSessionConfig {
            package_root: root,
            working_max_level: 0,
            launchpad_base_elevation_m: Some(1.0),
            ..Default::default()
        };
        let sess = SurfaceSession::open(cfg, elev).unwrap();
        let contact = sess.placement_contact();
        let s = contact.sample_at_world(inside).unwrap();
        let expect = elev + sess.package.sample_contact_height_at(0.0, 90.0, 0);
        assert!(
            (s.radius_m - expect).abs() < 1e-3,
            "no-grid radius {} vs global {expect}",
            s.radius_m
        );
    }

    #[test]
    fn rejects_radius_mismatch() {
        let Some(root) = pkg_low() else {
            eprintln!("skip rejects_radius_mismatch: packages/low missing");
            return;
        };
        let cfg = SurfaceSessionConfig {
            package_root: root,
            ..Default::default()
        };
        match SurfaceSession::open(cfg, 6_000_000.0) {
            Ok(_) => panic!("expected radius mismatch error"),
            Err(err) => {
                assert!(err.contains("R_elev") || err.contains("body.size"), "{err}");
            }
        }
    }
}
