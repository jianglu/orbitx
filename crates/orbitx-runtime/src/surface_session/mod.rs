//! P5.1 surface package session: hold package, schedule prefetch, inject GroundContact.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use orbitx_math::{Matrix3, Vec3};
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
            last_inject: GroundContact::Sphere { radius_m: elev },
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
                // 仅按高度恢复：v_radial 在每个远地点都会过零转负——高空
                // 远地点曾把 Paused 翻回 Active、注入地面，连锁触发误判
                // 触地（2026-11-08 S3 于 10,698 km「坠入水面」根因①）。
                // 恢复语义 = 回落接近地表（alt < resume_alt_m），与瞬时
                // 径向速度趋势无关。
                if alt < self.config.resume_alt_m {
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
        let need_query_refresh =
            self.workset_ready && !matches!(self.last_inject, GroundContact::Query { .. });
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
mod tests;
