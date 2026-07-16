//! 浮点原点坐标桥 — 将 AU 尺度 f64 仿真坐标转换为 GPU 可用的 f32 渲染坐标。
//!
//! 移植自 `orbitx-scene/bridge.rs`（CameraFrame），但泛化支持任意焦点。
//! 与 Orbiter `Camera::Update()` 同理：相机始终位于视觉原点，
//! `p_visual = p_logical - p_camera`，避免单精度浮点在 AU 距离下丢失精度。

use orbitx_math::vec3::Vec3;
use orbitx_math::mat3::Matrix3;

/// AU（天文单位）转米。
const AU_M: f64 = 1.49597870700e11;

/// 浮点原点坐标桥。
///
/// 将仿真坐标系（左手黄道 J2000，f64 米）中的位置转换为
/// 渲染坐标系（右手 Y-up，f32 渲染单位）中的位置。
///
/// # 坐标系映射
///
/// orbitx 仿真坐标系（左手，黄道 J2000）：
/// - x = 春分方向
/// - y = 黄道北
/// - z = 左手正交（x×y 的左手方向）
///
/// wgpu 渲染坐标系（右手，Y-up）：
/// - render.x = sim.x
/// - render.y = sim.z   （z 变 y）
/// - render.z = -sim.y  （y 取反翻转行列式，左手→右手）
pub struct CoordinateBridge {
    /// 浮点原点：仿真坐标系中焦点位置（f64 米，左手黄道）。
    origin: Vec3,

    /// 缩放因子：1 仿真米 = scale 渲染单位。
    scale: f64,
}

impl CoordinateBridge {
    /// 太阳系视图模式：1 AU = `au_render_units` 渲染单位。
    ///
    /// 典型值：au_render_units = 20.0（太阳系缩放视图）。
    pub fn new_solar_system(au_render_units: f32) -> Self {
        Self {
            origin: Vec3::ZERO,
            scale: au_render_units as f64 / AU_M,
        }
    }

    /// 真实尺度模式：1 米 = 1 渲染单位。
    ///
    /// 用于近地/地面场景，需要浮点原点保持精度。
    pub fn new_real_scale() -> Self {
        Self {
            origin: Vec3::ZERO,
            scale: 1.0,
        }
    }

    /// 更新浮点原点（每帧调用，设为焦点天体位置）。
    pub fn set_origin(&mut self, new_origin: Vec3) {
        self.origin = new_origin;
    }

    /// 获取当前浮点原点。
    pub fn origin(&self) -> Vec3 {
        self.origin
    }

    /// 获取缩放因子。
    pub fn scale(&self) -> f64 {
        self.scale
    }

    /// 将仿真坐标（f64 米）转换为渲染坐标（f32 渲染单位）。
    ///
    /// `p_render = handedness((p_sim - origin) * scale)`
    ///
    /// 其中 handedness 执行左手→右手变换。
    pub fn to_render(&self, sim_pos: &Vec3) -> glam::Vec3 {
        let dx = (sim_pos.x - self.origin.x) * self.scale;
        let dy = (sim_pos.y - self.origin.y) * self.scale;
        let dz = (sim_pos.z - self.origin.z) * self.scale;
        // 左手→右手：render.x=sim.x, render.y=sim.z, render.z=-sim.y
        glam::Vec3::new(dx as f32, dz as f32, -dy as f32)
    }

    /// 将仿真半径（f64 米）转换为渲染半径（f32 渲染单位）。
    pub fn to_render_radius(&self, meters: f64) -> f32 {
        (meters * self.scale) as f32
    }

    /// 将仿真方向向量（f64，单位向量）转换为渲染方向（f32）。
    ///
    /// 不减去原点，仅做左手→右手变换。
    pub fn to_render_dir(&self, dir: &Vec3) -> glam::Vec3 {
        glam::Vec3::new(dir.x as f32, dir.z as f32, -dir.y as f32)
    }

    /// 将仿真旋转矩阵（f64，3×3）转换为渲染旋转矩阵（f32）。
    ///
    /// 左手→右手变换：R_render = H * R_sim * H^(-1)
    /// 其中 H = [[1,0,0],[0,0,-1],[0,1,0]]（y↔z 并取反）
    pub fn to_render_mat3(&self, m: &Matrix3) -> glam::Mat3 {
        // 简化：逐元素转换后做 handedness 变换
        // H * M * H^-1 其中 H = swap rows/cols 1,2 and negate
        // 结果：M' = [[m11, m13, -m12], [m31, m33, -m32], [-m21, -m23, m22]]
        glam::Mat3::from_cols(
            glam::Vec3::new(m.m11 as f32, m.m31 as f32, -m.m21 as f32),
            glam::Vec3::new(m.m13 as f32, m.m33 as f32, -m.m23 as f32),
            glam::Vec3::new(-m.m12 as f32, -m.m32 as f32, m.m22 as f32),
        )
    }

    /// 从渲染坐标反推仿真坐标（f64 米）。
    ///
    /// 用于拾取（picking）等需要从屏幕坐标反查仿真位置的场景。
    pub fn to_sim(&self, render_pos: &glam::Vec3) -> Vec3 {
        // 右手→左手逆变换：sim.x=render.x, sim.z=render.y, sim.y=-render.z
        let rx = render_pos.x as f64 / self.scale + self.origin.x;
        let ry = -render_pos.z as f64 / self.scale + self.origin.y;
        let rz = render_pos.y as f64 / self.scale + self.origin.z;
        Vec3::new(rx, ry, rz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::vec3::Vec3;

    #[test]
    fn solar_system_scale() {
        let bridge = CoordinateBridge::new_solar_system(20.0);
        // 1 AU should map to 20 render units from origin
        let pos_1au = Vec3::new(AU_M, 0.0, 0.0);
        let r = bridge.to_render(&pos_1au);
        assert!((r.x - 20.0).abs() < 1e-3, "1 AU = {} render units, expected 20", r.x);
    }

    #[test]
    fn origin_maps_to_zero() {
        let mut bridge = CoordinateBridge::new_solar_system(20.0);
        let origin = Vec3::new(1.0e11, 2.0e10, -3.0e10);
        bridge.set_origin(origin);
        let r = bridge.to_render(&origin);
        assert!(r.x.abs() < 1e-6 && r.y.abs() < 1e-6 && r.z.abs() < 1e-6,
            "origin should map to (0,0,0), got {:?}", r);
    }

    #[test]
    fn handedness_swap() {
        let bridge = CoordinateBridge::new_real_scale();
        // sim (1, 0, 0) → render (1, 0, 0)
        let r = bridge.to_render(&Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(r, glam::Vec3::new(1.0, 0.0, 0.0));
        // sim (0, 1, 0) → render (0, 0, -1)  (y→-z)
        let r = bridge.to_render(&Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(r, glam::Vec3::new(0.0, 0.0, -1.0));
        // sim (0, 0, 1) → render (0, 1, 0)   (z→y)
        let r = bridge.to_render(&Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(r, glam::Vec3::new(0.0, 1.0, 0.0));
    }

    #[test]
    fn roundtrip_sim_render_sim() {
        let mut bridge = CoordinateBridge::new_solar_system(20.0);
        let origin = Vec3::new(1.0e11, 0.0, 0.0);
        bridge.set_origin(origin);
        let sim_pos = Vec3::new(1.0e11 + 1000.0, 2000.0, -3000.0);
        let render_pos = bridge.to_render(&sim_pos);
        let recovered = bridge.to_sim(&render_pos);
        let err = (recovered - sim_pos).length();
        assert!(err < 0.01, "roundtrip error = {} m", err);
    }

    #[test]
    fn render_radius() {
        let bridge = CoordinateBridge::new_solar_system(20.0);
        // Earth radius ~6.371e6 m
        let r = bridge.to_render_radius(6.371e6);
        let expected = 6.371e6 * 20.0 / AU_M;
        assert!((r as f64 - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn to_render_dir_no_origin_offset() {
        let mut bridge = CoordinateBridge::new_real_scale();
        bridge.set_origin(Vec3::new(1.0e11, 0.0, 0.0));
        // Direction should not be affected by origin
        let dir = Vec3::new(0.0, 1.0, 0.0);
        let r = bridge.to_render_dir(&dir);
        assert_eq!(r, glam::Vec3::new(0.0, 0.0, -1.0));
    }
}
