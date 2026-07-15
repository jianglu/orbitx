//! 坐标/单位转换桥接：orbitx f64 左手系 → kiss3d f32 右手系 Y-up。
//!
//! orbitx 使用左手系（Orbiter 约定）：
//!   x = 春分点方向, y = 黄道北极, z = 正交（左手）
//! kiss3d 使用右手系 Y-up：
//!   x = 右, y = 上, z = 前（面向 -z）
//!
//! 转换：kiss3d.x = orbitx.x, kiss3d.y = orbitx.z, kiss3d.z = -orbitx.y
//! （取反 y 以保证行列式符号翻转，维持正确的旋转方向）
//!
//! 同时做 AU 缩放和浮点原点平移，避免 f32 在天文距离上的精度问题。

use kiss3d::prelude::Vec3 as Vec3f;

/// 1 天文单位 = 149597870700 米。
pub const AU_METERS: f64 = 299_792_458.0 * 499.004783806;

/// 渲染坐标系转换器。
pub struct CameraFrame {
    /// 浮点原点（关注中心的 f64 坐标，单位：米）。
    origin: [f64; 3],
    /// 缩放因子：1 米 = scale 个 kiss3d 单位。
    /// 通常设为 `render_scale / AU_METERS`，使 1 AU = render_scale 个单位。
    scale: f64,
}

impl CameraFrame {
    /// 创建以原点为中心、1 AU = `au_render_units` 个单位的转换器。
    pub fn new(au_render_units: f64) -> Self {
        Self {
            origin: [0.0; 3],
            scale: au_render_units / AU_METERS,
        }
    }

    /// 设置浮点原点（跟随的天体位置，单位：米）。
    pub fn set_origin(&mut self, origin: [f64; 3]) {
        self.origin = origin;
    }

    /// 获取浮点原点。
    pub fn origin(&self) -> [f64; 3] {
        self.origin
    }

    /// 将 orbitx f64 位置（米，左手系）转为 kiss3d f32 位置（右手系 Y-up）。
    pub fn to_render(&self, pos: [f64; 3]) -> Vec3f {
        let dx = (pos[0] - self.origin[0]) * self.scale;
        let dy = (pos[1] - self.origin[1]) * self.scale;
        let dz = (pos[2] - self.origin[2]) * self.scale;
        // 左手→右手 Y-up：kiss3d.x=x, kiss3d.y=z, kiss3d.z=-y
        Vec3f::new(dx as f32, dz as f32, -dy as f32)
    }

    /// 缩放因子。
    pub fn scale(&self) -> f64 {
        self.scale
    }
}

/// 将半径（米）转为渲染半径（kiss3d 单位）。
pub fn scale_radius(radius_m: f64, frame: &CameraFrame) -> f32 {
    (radius_m * frame.scale()) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_maps_to_zero() {
        let mut frame = CameraFrame::new(10.0); // 1 AU = 10 units
        let origin = [1.5e11, 0.0, 0.0];
        frame.set_origin(origin);
        let rendered = frame.to_render(origin);
        assert!(rendered.x.abs() < 1e-3);
        assert!(rendered.y.abs() < 1e-3);
        assert!(rendered.z.abs() < 1e-3);
    }

    #[test]
    fn au_distance_scales_correctly() {
        let frame = CameraFrame::new(10.0); // 1 AU = 10 units
                                            // 1 AU 沿 x 轴
        let pos = [AU_METERS, 0.0, 0.0];
        let rendered = frame.to_render(pos);
        assert!((rendered.x - 10.0).abs() < 0.01, "x = {}", rendered.x);
    }
}
