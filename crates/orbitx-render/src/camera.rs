//! 相机系统 — 移植自 Orbiter `Camera.cpp`（1,763 行）。
//!
//! 支持 6 种外部模式 + 3 种内部/驾驶舱模式，
//! 对数深度缓冲，动态近平面调整。

use orbitx_math::vec3::Vec3;
use orbitx_math::mat3::Matrix3;

/// 对数深度缓冲配置。
///
/// 太空模拟器必须使用对数深度分布，否则无法同时显示
/// 近处航天器（米级）和远处行星（AU 级）。
///
/// `z_ndc = log2(C * z_eye + 1) / log2(C * far + 1)`
///
/// 其中 C 为常数，典型值 1.0。近/远比可达 1e15。
#[derive(Clone, Debug)]
pub struct LogDepthConfig {
    pub enabled: bool,
    /// 远裁剪面距离（米）。
    pub far: f64,
    /// 近裁剪面距离（米）。
    pub near: f64,
}

impl Default for LogDepthConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            far: 1.0e14,  // ~670 AU
            near: 0.1,
        }
    }
}

impl LogDepthConfig {
    /// 对数深度常数 C（传给 WGSL shader）。
    pub fn constant(&self) -> f32 {
        1.0
    }

    /// 对数深度远裁剪面（f32，传给 shader uniform）。
    pub fn far_f32(&self) -> f32 {
        self.far as f32
    }
}

/// 外部相机模式（对应 Orbiter `ExtCamMode`，Camera.h）。
#[derive(Clone, Debug)]
pub enum ExternalCamMode {
    /// 相机绕目标天体轨道运动（极坐标）。
    /// 对应 Orbiter `CAMERA_TARGETRELATIVE`。
    TargetRelative {
        dist: f64,
        phi: f64,
        theta: f64,
    },

    /// 相机固定全局方向。
    /// 对应 Orbiter `CAMERA_ABSDIRECTION`。
    AbsDirection {
        dist: f64,
        gdir: Vec3,
    },

    /// 相机在全局坐标系中自由定位。
    /// 对应 Orbiter `CAMERA_GLOBALFRAME`。
    GlobalFrame {
        pos: Vec3,
        rot: Matrix3,
    },

    /// 相机方向始终从目标指向参考天体。
    /// 对应 Orbiter `CAMERA_TARGETTOOBJECT`。
    TargetToObject {
        dist: f64,
        dirref: usize,
    },

    /// 相机方向从参考天体指向目标。
    /// 对应 Orbiter `CAMERA_TARGETFROMOBJECT`。
    TargetFromObject {
        dist: f64,
        dirref: usize,
    },

    /// 地面观察者：固定在行星表面。
    /// 对应 Orbiter `CAMERA_GROUNDOBSERVER`。
    GroundObserver {
        lng: f64,
        lat: f64,
        alt: f64,
        terrain_follow: bool,
        target_lock: Option<usize>,
    },
}

impl ExternalCamMode {
    /// 模式名（用于 UI 显示）。
    pub fn name(&self) -> &'static str {
        match self {
            Self::TargetRelative { .. } => "TargetRelative",
            Self::AbsDirection { .. } => "AbsDirection",
            Self::GlobalFrame { .. } => "GlobalFrame",
            Self::TargetToObject { .. } => "TargetToObject",
            Self::TargetFromObject { .. } => "TargetFromObject",
            Self::GroundObserver { .. } => "GroundObserver",
        }
    }

    /// 短参数摘要（供右侧信息栏显示，一行内）。
    pub fn short_params(&self) -> String {
        match self {
            Self::TargetRelative { dist, phi, theta } => {
                format!("dist {:.2e}  φ {:+.1}° θ {:+.1}°",
                    dist, phi.to_degrees(), theta.to_degrees())
            }
            Self::AbsDirection { dist, gdir } => {
                format!("dist {:.2e}  gdir ({:+.2},{:+.2},{:+.2})",
                    dist, gdir.x, gdir.y, gdir.z)
            }
            Self::GlobalFrame { pos, .. } => {
                format!("pos ({:+.2e},{:+.2e},{:+.2e})", pos.x, pos.y, pos.z)
            }
            Self::TargetToObject { dist, dirref } => {
                format!("dist {:.2e}  ref #{}", dist, dirref)
            }
            Self::TargetFromObject { dist, dirref } => {
                format!("dist {:.2e}  ref #{}", dist, dirref)
            }
            Self::GroundObserver { lng, lat, alt, .. } => {
                format!("lng {:+.1}° lat {:+.1}° alt {:.0} m",
                    lng.to_degrees(), lat.to_degrees(), alt)
            }
        }
    }
}

impl Default for ExternalCamMode {
    fn default() -> Self {
        Self::TargetRelative {
            dist: 1.0e8,  // 100,000 km
            phi: 0.0,
            theta: std::f64::consts::FRAC_PI_4,
        }
    }
}

/// 内部/驾驶舱模式（对应 Orbiter `IntCamMode`）。
#[derive(Clone, Debug)]
pub enum InternalCamMode {
    /// 通用 2D 驾驶舱叠加。
    GenericCockpit,
    /// 2D 仪表面板。
    Panel2D,
    /// 3D 虚拟驾驶舱（可头部追踪）。
    VirtualCockpit {
        head_pos: Vec3,
        head_rot: Matrix3,
    },
}

/// 相机系统。
pub struct CameraSystem {
    /// 当前外部模式。
    pub ext_mode: ExternalCamMode,
    /// 当前内部模式（None = 外部视图）。
    pub int_mode: Option<InternalCamMode>,
    /// 是否处于内部/驾驶舱视图。
    pub is_internal: bool,
    /// 目标天体索引。
    pub target: usize,

    /// 对数深度缓冲配置。
    pub log_depth: LogDepthConfig,

    /// 动态近平面距离（米）。
    /// 根据最近行星距离调整，避免裁剪进行星表面。
    pub near_plane: f64,

    /// 当前相机位置（仿真坐标，f64 米）。
    cam_pos_sim: Vec3,

    /// 当前视图方向（仿真坐标，单位向量）。
    cam_dir_sim: Vec3,

    /// 垂直视场角（弧度）。
    fov_y: f64,

    /// 宽高比。
    aspect: f64,

    /// 坐标桥缩放因子：1 sim meter = render_scale render units.
    /// Set by the app from CoordinateBridge::scale(). Used to convert
    /// near/far planes from sim meters to render units for projection.
    pub render_scale: f64,
}

impl CameraSystem {
    /// 创建默认相机系统（TargetRelative 模式）。
    pub fn new() -> Self {
        Self {
            ext_mode: ExternalCamMode::default(),
            int_mode: None,
            is_internal: false,
            target: 0,
            log_depth: LogDepthConfig::default(),
            near_plane: 0.1,
            cam_pos_sim: Vec3::ZERO,
            cam_dir_sim: Vec3::new(0.0, -1.0, 0.0),
            fov_y: std::f64::consts::FRAC_PI_3,  // 60°
            aspect: 16.0 / 9.0,
            render_scale: 1.0,  // default: real_scale mode
        }
    }

    /// 获取相机仿真坐标（f64 米）。
    pub fn cam_pos_sim(&self) -> Vec3 {
        self.cam_pos_sim
    }

    /// 获取相机视图方向（仿真坐标，单位向量）。
    pub fn cam_dir_sim(&self) -> Vec3 {
        self.cam_dir_sim
    }

    /// 设置宽高比。
    pub fn fov_y(&self) -> f64 {
        self.fov_y
    }

    pub fn set_aspect(&mut self, aspect: f64) {
        self.aspect = aspect;
    }

    /// 设置垂直视场角。
    pub fn set_fov_y(&mut self, fov_y: f64) {
        self.fov_y = fov_y;
    }

    /// 设置坐标桥缩放因子（1 sim meter = scale render units）。
    pub fn set_render_scale(&mut self, scale: f64) {
        self.render_scale = scale;
    }

    /// 切换到外部模式。
    pub fn set_ext_mode(&mut self, mode: ExternalCamMode) {
        self.ext_mode = mode;
        self.is_internal = false;
        self.int_mode = None;
    }

    /// 切换到内部/驾驶舱模式。
    pub fn set_int_mode(&mut self, mode: InternalCamMode) {
        self.int_mode = Some(mode);
        self.is_internal = true;
    }

    /// 切换外部/内部视图（保留原有模式参数）。
    pub fn toggle_internal(&mut self) {
        if self.is_internal {
            self.is_internal = false;
        } else {
            if self.int_mode.is_none() {
                self.int_mode = Some(InternalCamMode::GenericCockpit);
            }
            self.is_internal = true;
        }
    }

    /// 循环切换外部模式（供 Tab 键使用）。
    ///
    /// 保留 TargetRelative 的 dist（当能取到时），新模式使用合理默认值。
    pub fn cycle_ext_mode(&mut self, forward: bool, dirref_hint: usize) {
        let cur = self.ext_mode_index();
        let n = 6_i32;
        let step = if forward { 1 } else { -1 };
        let next = ((cur as i32 + step).rem_euclid(n)) as usize;
        let dist = self.current_dist().unwrap_or(1.0e8);
        self.ext_mode = match next {
            0 => ExternalCamMode::TargetRelative {
                dist, phi: 0.0, theta: std::f64::consts::FRAC_PI_4,
            },
            1 => ExternalCamMode::AbsDirection {
                dist, gdir: Vec3::new(0.0, 0.0, -1.0),
            },
            2 => ExternalCamMode::GlobalFrame {
                pos: self.cam_pos_sim + Vec3::new(dist, 0.0, 0.0),
                rot: Matrix3::IDENTITY,
            },
            3 => ExternalCamMode::TargetToObject { dist, dirref: dirref_hint },
            4 => ExternalCamMode::TargetFromObject { dist, dirref: dirref_hint },
            5 => ExternalCamMode::GroundObserver {
                lng: 0.0, lat: 0.0, alt: 1.0e6,
                terrain_follow: false, target_lock: None,
            },
            _ => ExternalCamMode::default(),
        };
        self.is_internal = false;
    }

    /// 当前外部模式索引（0..6）。
    pub fn ext_mode_index(&self) -> usize {
        match self.ext_mode {
            ExternalCamMode::TargetRelative { .. } => 0,
            ExternalCamMode::AbsDirection { .. } => 1,
            ExternalCamMode::GlobalFrame { .. } => 2,
            ExternalCamMode::TargetToObject { .. } => 3,
            ExternalCamMode::TargetFromObject { .. } => 4,
            ExternalCamMode::GroundObserver { .. } => 5,
        }
    }

    /// 当前模式的距离（若模式含 dist 字段）。
    pub fn current_dist(&self) -> Option<f64> {
        match self.ext_mode {
            ExternalCamMode::TargetRelative { dist, .. } => Some(dist),
            ExternalCamMode::AbsDirection { dist, .. } => Some(dist),
            ExternalCamMode::TargetToObject { dist, .. } => Some(dist),
            ExternalCamMode::TargetFromObject { dist, .. } => Some(dist),
            _ => None,
        }
    }

    /// 设置 TargetToObject/TargetFromObject 的参考天体索引。
    pub fn set_dirref(&mut self, idx: usize) {
        match &mut self.ext_mode {
            ExternalCamMode::TargetToObject { dirref, .. }
            | ExternalCamMode::TargetFromObject { dirref, .. } => {
                *dirref = idx;
            }
            _ => {}
        }
    }

    /// 更新相机状态（每帧调用）。
    ///
    /// `body_positions` — 各天体位置（索引对应 target）。
    /// `dt` — 帧间隔（秒）。
    pub fn update(&mut self, body_positions: &[Vec3], dt: f64) {
        self.update_with_radii(body_positions, &[], dt);
    }

    /// 更新（含天体半径，用于动态近平面）。
    ///
    /// `body_radii` 若与 `body_positions` 长度一致则用于计算近平面：
    /// near = clamp(min(cam→表面距离) × 1e-3, 0.1, 1e6)。
    /// 长度不匹配时退化为静态 `LogDepthConfig::near`。
    pub fn update_with_radii(
        &mut self, body_positions: &[Vec3], body_radii: &[f64], _dt: f64,
    ) {
        if self.target >= body_positions.len() {
            return;
        }
        let target_pos = body_positions[self.target];

        match &self.ext_mode {
            ExternalCamMode::TargetRelative { dist, phi, theta } => {
                // 极坐标→笛卡尔（左手坐标系）
                let cp = phi.cos();
                let sp = phi.sin();
                let ct = theta.cos();
                let st = theta.sin();
                let offset = Vec3::new(
                    dist * ct * cp,
                    dist * st,
                    dist * ct * sp,
                );
                self.cam_pos_sim = target_pos + offset;
                self.cam_dir_sim = target_pos - self.cam_pos_sim;
                let len = self.cam_dir_sim.length();
                if len > 0.0 {
                    self.cam_dir_sim = self.cam_dir_sim.unit();
                }
            }
            ExternalCamMode::AbsDirection { dist, gdir } => {
                let d = gdir.length();
                let gdir_unit = if d > 0.0 { gdir.unit() } else { Vec3::new(0.0, -1.0, 0.0) };
                self.cam_pos_sim = target_pos - gdir_unit * *dist;
                self.cam_dir_sim = gdir_unit;
            }
            ExternalCamMode::GlobalFrame { pos, .. } => {
                self.cam_pos_sim = *pos;
                self.cam_dir_sim = target_pos - *pos;
                let len = self.cam_dir_sim.length();
                if len > 0.0 {
                    self.cam_dir_sim = self.cam_dir_sim.unit();
                }
            }
            ExternalCamMode::TargetToObject { dist, dirref } => {
                if *dirref < body_positions.len() {
                    let ref_pos = body_positions[*dirref];
                    let dir = ref_pos - target_pos;
                    let len = dir.length();
                    let dir_unit = if len > 0.0 { dir.unit() } else { Vec3::new(0.0, -1.0, 0.0) };
                    self.cam_pos_sim = target_pos - dir_unit * *dist;
                    self.cam_dir_sim = dir_unit;
                }
            }
            ExternalCamMode::TargetFromObject { dist, dirref } => {
                if *dirref < body_positions.len() {
                    let ref_pos = body_positions[*dirref];
                    let dir = target_pos - ref_pos;
                    let len = dir.length();
                    let dir_unit = if len > 0.0 { dir.unit() } else { Vec3::new(0.0, -1.0, 0.0) };
                    self.cam_pos_sim = ref_pos - dir_unit * *dist;
                    self.cam_dir_sim = dir_unit;
                }
            }
            ExternalCamMode::GroundObserver { lng, lat, alt, .. } => {
                // 简化：在目标天体表面的 (lng, lat, alt) 位置
                let r = alt;  // 相对于目标天体中心
                let clng = lng.cos();
                let slng = lng.sin();
                let clat = lat.cos();
                let slat = lat.sin();
                // 黄道坐标系中的表面位置（简化，忽略自转）
                let surface_pos = Vec3::new(r * clat * clng, r * slat, r * clat * slng);
                self.cam_pos_sim = target_pos + surface_pos;
                self.cam_dir_sim = Vec3::new(0.0, -1.0, 0.0);  // 简化：向下看
            }
        }

        // 更新动态近平面：找到相机到"任一天体表面"的最近距离
        if !body_radii.is_empty() && body_radii.len() == body_positions.len() {
            let mut nearest_surface = f64::INFINITY;
            for (pos, r) in body_positions.iter().zip(body_radii.iter()) {
                let d_center = (self.cam_pos_sim - *pos).length();
                let d_surface = (d_center - *r).max(1.0);
                if d_surface < nearest_surface {
                    nearest_surface = d_surface;
                }
            }
            // 近平面取到最近表面距离的 1e-3，限制在 [0.1, 1e6]，
            // 保证近/远比 <= 1e15（对数深度可容忍范围内）。
            let dyn_near = (nearest_surface * 1.0e-3).clamp(0.1, 1.0e6);
            self.near_plane = dyn_near;
        } else {
            self.near_plane = self.log_depth.near;
        }
    }

    /// 处理鼠标拖动（CAD 风格轨道旋转，方向为"抓取天体"式）。
    ///
    /// `dx`/`dy` 为原始像素位移（灵敏度在内部应用）：
    /// - 鼠标 X 移动 → phi（方位角）：向左拖天体向左转，向右拖向右转
    /// - 鼠标 Y 移动 → theta（仰角）：向下拖抬高视角，向上拖降低视角
    pub fn mouse_drag(&mut self, dx: f64, dy: f64) {
        const SENS: f64 = 0.005;
        if let ExternalCamMode::TargetRelative { dist, ref mut phi, ref mut theta } = self.ext_mode {
            *phi -= dx * SENS;
            *theta += dy * SENS;
            // 限制 theta 在 (-pi/2, pi/2) 避免翻转
            let half_pi = std::f64::consts::FRAC_PI_2 * 0.999;
            *theta = theta.clamp(-half_pi, half_pi);
            // 保持 dist 不变
            let _ = dist;
        }
    }

    /// 处理鼠标滚轮（缩放）。
    ///
    /// 使用指数缩放：`dist *= exp(-delta * k)`，让滚轮在跨越 15 数量级
    /// 距离（AU → 米）时保持可用。k=0.5 时每格约 ×1.65 / ÷1.65，
    /// 从 1 AU 缩到 1 m 约需 50 格（相较线性 0.1 系数的 250+ 格）。
    pub fn mouse_scroll(&mut self, delta: f64) {
        if let ExternalCamMode::TargetRelative { ref mut dist, .. } = self.ext_mode {
            let k = 0.5;
            *dist *= (-delta * k).exp();
            *dist = dist.max(1.0);  // 最小 1 米
        }
    }

    /// Build view matrix (glam f32, right-handed look-to).
    ///
    /// The CoordinateBridge sets the camera position as the floating-point
    /// origin, so in render space the camera IS at the origin. Scene node
    /// positions from `to_render()` are already camera-relative and scaled.
    /// Therefore eye = Vec3::ZERO, and only the look direction needs the
    /// handedness swap: render.x=sim.x, render.y=sim.y, render.z=-sim.z.
    pub fn view_matrix(&self) -> glam::Mat4 {
        // Camera is the floating-point origin in render space
        let eye = glam::Vec3::ZERO;

        // Convert sim direction to render space (handedness swap only).
        // Ecliptic north (sim.y) maps to render +y (up); flip z for handedness.
        let dir = glam::Vec3::new(
            self.cam_dir_sim.x as f32,
            self.cam_dir_sim.y as f32,
            -self.cam_dir_sim.z as f32,
        );

        let forward = dir.normalize();
        let up = glam::Vec3::Y;

        glam::Mat4::look_to_rh(eye, forward, up)
    }

    /// 构建投影矩阵（glam f32，用于 wgpu uniform）。
    ///
    /// Near/far planes are converted from sim meters to render units
    /// using `render_scale` so the depth buffer matches the coordinate
    /// system used by CoordinateBridge::to_render().
    pub fn projection_matrix(&self) -> glam::Mat4 {
        let near_render = (self.near_plane * self.render_scale) as f32;
        let far_render = (self.log_depth.far * self.render_scale) as f32;
        glam::Mat4::perspective_rh(self.fov_y as f32, self.aspect as f32,
            near_render.max(0.001), far_render)
    }
    /// Log-depth constant C in render units (for shader uniform).
    pub fn log_depth_constant_render(&self) -> f32 {
        self.log_depth.constant()
    }

    /// Log-depth far plane in render units (for shader uniform).
    pub fn log_depth_far_render(&self) -> f32 {
        (self.log_depth.far * self.render_scale) as f32
    }
}

impl Default for CameraSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_target_relative() {
        let cam = CameraSystem::new();
        assert!(!cam.is_internal);
        assert_eq!(cam.target, 0);
    }

    #[test]
    fn target_relative_update() {
        let mut cam = CameraSystem::new();
        let bodies = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0e11, 0.0, 0.0)];
        cam.update(&bodies, 0.016);
        // Camera should be at some distance from origin
        assert!(cam.cam_pos_sim().length() > 0.0);
    }

    #[test]
    fn mouse_drag_rotates_phi() {
        let mut cam = CameraSystem::new();
        let initial_phi = match cam.ext_mode {
            ExternalCamMode::TargetRelative { phi, .. } => phi,
            _ => 0.0,
        };
        cam.mouse_drag(100.0, 0.0);
        let new_phi = match cam.ext_mode {
            ExternalCamMode::TargetRelative { phi, .. } => phi,
            _ => 0.0,
        };
        // Dragging right (dx>0) decreases phi ("grab and rotate" convention).
        assert!(new_phi < initial_phi);
    }

    #[test]
    fn mouse_scroll_zooms() {
        let mut cam = CameraSystem::new();
        let initial_dist = match cam.ext_mode {
            ExternalCamMode::TargetRelative { dist, .. } => dist,
            _ => 0.0,
        };
        cam.mouse_scroll(1.0);  // scroll up = zoom in
        let new_dist = match cam.ext_mode {
            ExternalCamMode::TargetRelative { dist, .. } => dist,
            _ => 0.0,
        };
        assert!(new_dist < initial_dist);
    }

    #[test]
    fn log_depth_default_range() {
        let ld = LogDepthConfig::default();
        assert!(ld.enabled);
        assert!(ld.far / ld.near > 1e12, "near/far ratio should exceed 1e12");
    }

    #[test]
    fn switch_to_ground_observer() {
        let mut cam = CameraSystem::new();
        cam.set_ext_mode(ExternalCamMode::GroundObserver {
            lng: 0.0, lat: 0.0, alt: 6.371e6,
            terrain_follow: false, target_lock: None,
        });
        assert!(!cam.is_internal);
    }

    #[test]
    fn cycle_ext_mode_visits_all_six() {
        let mut cam = CameraSystem::new();
        let mut names = Vec::new();
        for _ in 0..6 {
            names.push(cam.ext_mode.name());
            cam.cycle_ext_mode(true, 1);
        }
        // 应恰好回到起点
        assert_eq!(names.len(), 6);
        // 应包含所有 6 个模式
        for expected in ["TargetRelative", "AbsDirection", "GlobalFrame",
                         "TargetToObject", "TargetFromObject", "GroundObserver"] {
            assert!(names.contains(&expected), "missing mode {}", expected);
        }
    }

    #[test]
    fn cycle_backward() {
        let mut cam = CameraSystem::new();
        let start = cam.ext_mode_index();
        cam.cycle_ext_mode(false, 0);
        assert_ne!(cam.ext_mode_index(), start);
        cam.cycle_ext_mode(true, 0);
        assert_eq!(cam.ext_mode_index(), start);
    }

    #[test]
    fn toggle_internal_flips_state() {
        let mut cam = CameraSystem::new();
        assert!(!cam.is_internal);
        cam.toggle_internal();
        assert!(cam.is_internal);
        assert!(cam.int_mode.is_some());
        cam.toggle_internal();
        assert!(!cam.is_internal);
    }

    #[test]
    fn dynamic_near_plane_scales_with_altitude() {
        let mut cam = CameraSystem::new();
        // 目标在原点，半径 6.371e6（地球）
        let bodies = vec![Vec3::ZERO];
        let radii = vec![6.371e6];

        // 拉远（TargetRelative dist=1e8 → 相机距表面 ~9.36e7）
        cam.ext_mode = ExternalCamMode::TargetRelative {
            dist: 1.0e8, phi: 0.0, theta: 0.0,
        };
        cam.update_with_radii(&bodies, &radii, 0.016);
        let far_near = cam.near_plane;

        // 拉近（TargetRelative dist=7e6 → 相机距表面 ~6.29e5）
        cam.ext_mode = ExternalCamMode::TargetRelative {
            dist: 7.0e6, phi: 0.0, theta: 0.0,
        };
        cam.update_with_radii(&bodies, &radii, 0.016);
        let close_near = cam.near_plane;

        assert!(close_near < far_near, "近距时 near 应更小");
        assert!(close_near >= 0.1 && close_near <= 1.0e6);
    }

    #[test]
    fn set_dirref_updates_target_to_object() {
        let mut cam = CameraSystem::new();
        cam.set_ext_mode(ExternalCamMode::TargetToObject { dist: 1e8, dirref: 1 });
        cam.set_dirref(3);
        if let ExternalCamMode::TargetToObject { dirref, .. } = cam.ext_mode {
            assert_eq!(dirref, 3);
        } else {
            panic!("mode did not stay TargetToObject");
        }
    }
}
