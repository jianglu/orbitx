//! P5.1 surface package session: hold package, schedule prefetch, inject GroundContact.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use orbitx_math::{dot, Matrix3, Vec3};
use orbitx_surface::{enu_basis, latlon_up, LocalElevationGrid, SurfacePackage, WaterClass};
use orbitx_vessel::{
    geocentric_lat_lon_deg_orient, GroundContact, GroundSample, GroundSampler, SurfaceKind,
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
    /// 有 `pad_area` 时本值升级为**填平域内强制地面高**（填平，非只截高）。
    pub launchpad_base_elevation_m: Option<f64>,
    /// 发射场填平域（lp `[pad_area]`；统一公式与 Godot shader 一字不差）。
    pub pad_area: Option<PadAreaSpec>,
    /// 发射场各台位轴基点经纬度（加载期与全球包高程比对，超差告警）。
    pub launchpad_pad_points: Vec<(f64, f64)>,
}

/// 发射场填平域（与 lp `[pad_area]` / Godot shader 同式统一公式）：
/// `t = clamp((dist − radius_m)/blend_m, 0, 1); h = lerp(base, h_global, t)`。
/// `dist` = 查询点相对圆心的 ENU 东/北范数（相对场原点切平面投影，同
/// `LocalElevationGrid` 投影法）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PadAreaSpec {
    /// 场原点经纬度（切平面投影锚点）。
    pub origin_lat_deg: f64,
    pub origin_lon_deg: f64,
    /// 圆心相对场原点 ENU 偏移（东、北，m）。
    pub center_enu_m: [f64; 2],
    /// 填平半径（m）。
    pub radius_m: f64,
    /// 过渡带宽度（m）。
    pub blend_m: f64,
}

impl PadAreaSpec {
    /// 查询点 → 圆心距离（m）。
    fn dist_m(&self, lat_deg: f64, lon_deg: f64, r_elev_m: f64) -> f64 {
        let (east, north, up) = enu_basis(self.origin_lat_deg, self.origin_lon_deg);
        let u = latlon_up(lat_deg, lon_deg);
        let p = [u[0] * r_elev_m, u[1] * r_elev_m, u[2] * r_elev_m];
        let o = [up[0] * r_elev_m, up[1] * r_elev_m, up[2] * r_elev_m];
        let d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
        let e = d[0] * east[0] + d[1] * east[1] + d[2] * east[2] - self.center_enu_m[0];
        let n = d[0] * north[0] + d[1] * north[1] + d[2] * north[2] - self.center_enu_m[1];
        (e * e + n * n).sqrt()
    }

    /// 查询点落在填平域内（t = 0）？
    fn is_inside(&self, lat_deg: f64, lon_deg: f64, r_elev_m: f64) -> bool {
        self.dist_m(lat_deg, lon_deg, r_elev_m) <= self.radius_m
    }
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
            pad_area: None,
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
    /// 体固→惯性旋转矩阵；每 tick 由 runtime 写入（与 `PlanetOrientation.rot` 同源）。
    rot: Matrix3,
}

impl SurfaceSession {
    /// Godot 切档热更新：钳在包 terrain / water_surface max_level 内。
    pub fn set_working_max_level(&mut self, level: u32) {
        let capped = level
            .min(self.package.max_level("terrain"))
            .min(self.package.max_level("water_surface"));
        self.working_max_level = capped;
        self.config.working_max_level = capped;
        self.workset_ready = false;
    }
}

struct PackageSampler {
    package: Arc<SurfacePackage>,
    level: u32,
    grid: Option<Arc<LocalElevationGrid>>,
    /// 发射场底部高程：足迹内地形截平上限；有填平域时 = 域内强制地面高。
    base_elevation_m: Option<f64>,
    /// 发射场填平域（统一公式，最终裁决层）。
    pad_area: Option<PadAreaSpec>,
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
        // 填平域内恒陆地（域内强制填平，水面一并覆盖）。
        if let Some(pa) = &self.pad_area {
            if self.base_elevation_m.is_some()
                && pa.is_inside(lat_deg, lon_deg, self.package.r_elev_m)
            {
                return SurfaceKind::Land;
            }
        }
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
        let mut s = if let Some(ref g) = self.grid {
            if let Some(s) = g.sample(lat_deg, lon_deg) {
                // 发射场局部对齐：足迹内地形高于场坪则截平（只截高、不填低）。
                let height_m = match self.base_elevation_m {
                    Some(base) => s.height_m.min(base),
                    None => s.height_m,
                };
                GroundSample {
                    radius_m: self.package.r_elev_m + height_m,
                    normal: enu_normal_to_world(lat_deg, lon_deg, s.normal_enu),
                    level_used: None,
                    kind: SurfaceKind::Land,
                }
            } else {
                self.sample_global(lat_deg, lon_deg)
            }
        } else {
            self.sample_global(lat_deg, lon_deg)
        };
        // 填平域最终裁决（统一公式，与 Godot shader 一字不差）：
        // t = clamp((dist − radius_m)/blend_m, 0, 1); h = lerp(base, h, t)。
        // 域内（t=0）强制 h = base（填平，含水面）、法向 = ENU up、分类恒陆地；
        // 过渡带法向/分类维持底层采样；t≥1 原样。
        if let (Some(pa), Some(base)) = (&self.pad_area, self.base_elevation_m) {
            let dist = pa.dist_m(lat_deg, lon_deg, self.package.r_elev_m);
            let t = if pa.blend_m > 0.0 {
                ((dist - pa.radius_m) / pa.blend_m).clamp(0.0, 1.0)
            } else if dist <= pa.radius_m {
                0.0
            } else {
                1.0
            };
            if t < 1.0 {
                let h_under = s.radius_m - self.package.r_elev_m;
                s.radius_m = self.package.r_elev_m + base + (h_under - base) * t;
            }
            if t <= 0.0 {
                s.normal = enu_normal_to_world(lat_deg, lon_deg, [0.0, 0.0, 1.0]);
                s.kind = SurfaceKind::Land;
                s.level_used = None;
            }
        }
        s
    }
}

impl PackageSampler {
    /// 全球包采样（无覆盖网格命中时）。
    fn sample_global(&self, lat_deg: f64, lon_deg: f64) -> GroundSample {
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
        // 填平域依赖场坪高：缺 base 时拒绝（语义不完整，静默回退会埋穿模）。
        if config.pad_area.is_some() && config.launchpad_base_elevation_m.is_none() {
            return Err("发射场 [pad_area] 需要 base_elevation_m 同时声明".into());
        }
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
            rot: Matrix3::IDENTITY,
        })
    }

    fn make_sampler(&self) -> Arc<PackageSampler> {
        Arc::new(PackageSampler {
            package: Arc::clone(&self.package),
            level: self.working_max_level,
            grid: self.grid.clone(),
            base_elevation_m: self.config.launchpad_base_elevation_m,
            pad_area: self.config.pad_area,
        })
    }

    /// 组合采样器（覆盖 + 全球包硬切换），供发射台摆放等步进外查询。
    /// 使用当前会话 `rot`（摆放前由宿主写入 `sim_t=0` 姿态）。
    pub fn placement_contact(&self) -> GroundContact {
        GroundContact::query(self.make_sampler(), self.rot)
    }

    /// 摆放前设置体固→惯性矩阵（与 `primary_surface().orientation.rot` 一致）。
    pub fn set_rot(&mut self, rot: Matrix3) {
        self.rot = rot;
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
    /// `rot`：体固→惯性；预取/注入经纬均用体固系（`Rᵀ · r`）。
    pub fn on_tick(&mut self, pos: Vec3, vel: Vec3, rot: Matrix3) {
        self.rot = rot;
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

        let (lat, lon) = geocentric_lat_lon_deg_orient(pos, self.rot);
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
            let (alat, alon) = geocentric_lat_lon_deg_orient(ahead, self.rot);
            let _ = self
                .package
                .prefetch_around(alat, alon, self.working_max_level, ring);
        }
    }

    /// Build GroundContact for this tick (three-state inject).
    ///
    /// When `inject_every > 1`, retains the previous sampler between inject ticks
    /// but always refreshes `rot`；Off / Sphere transitions still apply immediately.
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
                GroundContact::query(self.make_sampler(), self.rot)
            }
            ScheduleState::Active => {
                // 复用采样器，刷新 rot。
                let refreshed = match &self.last_inject {
                    GroundContact::Query { sampler, .. } => {
                        GroundContact::query(Arc::clone(sampler), self.rot)
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
            .join("../../../simrocket_earth/packages/physical");
        if p.join("manifest.toml").is_file() {
            Some(p)
        } else {
            None
        }
    }

    #[test]
    fn three_states_and_contact_height() {
        let Some(root) = pkg_low() else {
            eprintln!("skip three_states_and_contact_height: packages/physical missing");
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
        sess.on_tick(pos, vel, Matrix3::IDENTITY);
        assert_eq!(sess.schedule, ScheduleState::Active);
        assert!(sess.workset_ready);
        let g = sess.inject();
        assert!(
            matches!(g, GroundContact::Query { .. }),
            "expected Query after prefetch, got {g:?}"
        );
        let (lat, lon) = geocentric_lat_lon_deg_orient(pos, Matrix3::IDENTITY);
        let r_query = g.radius_at_world(pos).unwrap();
        let expect = elev + sess.package.sample_contact_height_at(lat, lon, 0);
        assert!(
            (r_query - expect).abs() < 1e-3,
            "Query radius {r_query} vs package {expect}"
        );

        // High altitude + outward → Paused → Off
        let pos_hi = Vec3::new(elev + 40_000.0, 0.0, 0.0);
        let vel_out = Vec3::new(100.0, 0.0, 0.0);
        sess.on_tick(pos_hi, vel_out, Matrix3::IDENTITY);
        assert_eq!(sess.schedule, ScheduleState::Paused);
        assert!(matches!(sess.inject(), GroundContact::Off));

        // Descend below resume → Active; Sphere path when not ready.
        let pos_mid = Vec3::new(elev + 15_000.0, 0.0, 0.0);
        let vel_in = Vec3::new(-50.0, 0.0, 0.0);
        sess.workset_ready = false;
        sess.schedule = ScheduleState::Paused;
        sess.on_tick(pos_mid, vel_in, Matrix3::IDENTITY);
        assert_eq!(sess.schedule, ScheduleState::Active);
        sess.workset_ready = false;
        assert!(matches!(
            sess.inject(),
            GroundContact::Sphere { .. }
        ));
    }

    #[test]
    fn corotating_body_fixed_sample_radius_stable() {
        // 共转位移后：体固 R 采样高度应与起点一致（惯性 lon 已变）。
        let Some(root) = pkg_low() else {
            eprintln!("skip corotating_body_fixed: packages/physical missing");
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
        let angle = orbitx_math::PI2 * 60.0 / 86_164.0;
        let c = angle.cos();
        let s = angle.sin();
        // R_y(+angle)：体固→惯性
        let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        let pos0 = Vec3::new(elev + 50.0, 0.0, 0.0);
        let pos_t = orbitx_dynamics::to_inertial(rot, pos0);

        sess.on_tick(pos0, Vec3::new(-1.0, 0.0, 0.0), Matrix3::IDENTITY);
        let g0 = sess.inject();
        let r0 = g0.radius_at_world(pos0).unwrap();

        sess.on_tick(pos_t, Vec3::new(-1.0, 0.0, 0.0), rot);
        let g1 = sess.inject();
        let r1 = g1.radius_at_world(pos_t).unwrap();
        assert!(
            (r0 - r1).abs() < 1e-3,
            "body-fixed radius should match: r0={r0} r1={r1}"
        );
        // 若误用 R=I 采惯性 lon，半径通常会偏（全球包经度变化）。
        let g_wrong = GroundContact::query(
            match &g1 {
                GroundContact::Query { sampler, .. } => Arc::clone(sampler),
                _ => panic!("expected Query"),
            },
            Matrix3::IDENTITY,
        );
        let r_inertial_lon = g_wrong.radius_at_world(pos_t).unwrap();
        let _ = r_inertial_lon;
    }

    #[test]
    fn override_hard_switch_inside_outside() {
        let Some(root) = pkg_low() else {
            eprintln!("skip override_hard_switch: packages/physical missing");
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
            eprintln!("skip base_elevation_clamps: packages/physical missing");
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
            eprintln!("skip rejects_radius_mismatch: packages/physical missing");
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

    /// 填平域统一公式：域内强制 base（填平，含水面）、过渡带 lerp、域外原样、
    /// 边界连续；法向域内 = ENU up、分类恒陆地。
    #[test]
    fn pad_area_flatten_unified_formula() {
        let Some(root) = pkg_low() else {
            eprintln!("skip pad_area_flatten: packages/physical missing");
            return;
        };
        let elev = 6_371_010.0;
        // 场原点 (0°, 90°)；圆心=场原点，radius=60，blend=20。
        let pa = PadAreaSpec {
            origin_lat_deg: 0.0,
            origin_lon_deg: 90.0,
            center_enu_m: [0.0, 0.0],
            radius_m: 60.0,
            blend_m: 20.0,
        };
        // ENU 偏移 → 经纬度（与 LocalElevationGrid 同投影）。
        // 经纬度采样经由世界向量（GroundContact::sample_at_world）。
        let sample_ll = |contact: &orbitx_vessel::ground::GroundContact,
                         lat: f64,
                         lon: f64|
         -> orbitx_vessel::ground::GroundSample {
            let dir = orbitx_vessel::geocentric_up(lat, lon);
            contact.sample_at_world(dir * elev).unwrap()
        };
        let latlon_at = |e_m: f64, n_m: f64| -> (f64, f64) {
            let (east, north, up) = enu_basis(0.0, 90.0);
            let p = [
                up[0] * elev + east[0] * e_m + north[0] * n_m,
                up[1] * elev + east[1] * e_m + north[1] * n_m,
                up[2] * elev + east[2] * e_m + north[2] * n_m,
            ];
            let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
            ((p[1] / r).asin().to_degrees(), p[2].atan2(p[0]).to_degrees())
        };
        let cfg = SurfaceSessionConfig {
            package_root: root,
            working_max_level: 0,
            launchpad_base_elevation_m: Some(50.0),
            pad_area: Some(pa),
            ..Default::default()
        };
        let sess = SurfaceSession::open(cfg, elev).unwrap();
        let contact = sess.placement_contact();
        let r_elev = sess.package.r_elev_m;

        // 域内（dist≈0）：h = base，与全球采样无关（填平，非截平）。
        let (lat, lon) = latlon_at(0.0, 0.0);
        let s = sample_ll(&contact, lat, lon);
        assert!(
            (s.radius_m - (r_elev + 50.0)).abs() < 1e-6,
            "in-domain h must equal base, got {}",
            s.radius_m - r_elev
        );
        assert_eq!(s.kind, SurfaceKind::Land);
        assert!(s.level_used.is_none());
        // 域内法向 = ENU up。
        let up_dir = orbitx_vessel::geocentric_up(lat, lon);
        assert!((dot(s.normal, up_dir) - 1.0).abs() < 1e-9, "normal must be up");

        // 过渡带中点（dist = radius + blend/2 = 70，沿东向）：h = lerp(50, h_g, 0.5)。
        let (lat, lon) = latlon_at(70.0, 0.0);
        let h_g = sess.package.sample_contact_height_at(lat, lon, 0);
        let s = sample_ll(&contact, lat, lon);
        let expect = 50.0 + (h_g - 50.0) * 0.5;
        assert!(
            (s.radius_m - (r_elev + expect)).abs() < 1e-6,
            "blend midpoint {} vs {expect}",
            s.radius_m - r_elev
        );

        // 域外（dist = radius + blend + 10 = 90）：原样。
        let (lat, lon) = latlon_at(90.0, 0.0);
        let h_g = sess.package.sample_contact_height_at(lat, lon, 0);
        let s = sample_ll(&contact, lat, lon);
        assert!(
            (s.radius_m - (r_elev + h_g)).abs() < 1e-6,
            "outside must be untouched"
        );

        // 边界连续：dist = radius 外沿（t=0，h=base）与 dist→radius⁺（t→0）连续；
        // blend 外沿（t=1）与全球一致。
        let (lat, lon) = latlon_at(60.0, 0.0);
        let s = sample_ll(&contact, lat, lon);
        assert!((s.radius_m - (r_elev + 50.0)).abs() < 1e-6, "t=0 boundary");
        let (lat, lon) = latlon_at(80.0, 0.0);
        let h_g = sess.package.sample_contact_height_at(lat, lon, 0);
        let s = sample_ll(&contact, lat, lon);
        assert!(
            (s.radius_m - (r_elev + h_g)).abs() < 1e-6,
            "t=1 boundary continuity"
        );
    }

    /// 无 base 的 pad_area 拒绝加载（语义不完整）。
    #[test]
    fn pad_area_requires_base_elevation() {
        let Some(root) = pkg_low() else {
            eprintln!("skip pad_area_requires_base: packages/physical missing");
            return;
        };
        let cfg = SurfaceSessionConfig {
            package_root: root,
            working_max_level: 0,
            pad_area: Some(PadAreaSpec {
                origin_lat_deg: 0.0,
                origin_lon_deg: 90.0,
                center_enu_m: [0.0, 0.0],
                radius_m: 60.0,
                blend_m: 20.0,
            }),
            ..Default::default()
        };
        let err = match SurfaceSession::open(cfg, 6_371_010.0) {
            Err(e) => e,
            Ok(_) => panic!("pad_area without base must be rejected"),
        };
        assert!(err.contains("base_elevation_m"), "{err}");
    }
}
