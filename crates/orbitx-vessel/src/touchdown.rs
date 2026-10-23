//! 着陆/接触（薄 re-export 层 + 布局构造器）。
//!
//! 物理算法（弹簧-阻尼-摩擦接触力）与物理数据类型 `TouchdownVertex` /
//! `SurfaceContact` / `compute_surface_forces` 已下沉到
//! `orbitx_dynamics::contact`。本模块保留 `make_landing_gear` 布局构造器
//! （装配关注点，非物理算法）并 re-export 物理项以保持公共 API 不变。

pub use orbitx_dynamics::contact::{
    compute_surface_forces, compute_surface_forces_at, SurfaceContact, TouchdownVertex,
};

use orbitx_math::Vec3;

/// 创建三点着陆架（三角布局）。
///
/// 典型配置：3 个触点在底部等间隔分布，模拟着陆三角架。
///
/// # 参数
/// - `radius`: 着陆架半径 [m]（触点到纵轴距离）
/// - `y_offset`: 着陆架在体坐标系 Y 的位置 [m]（负值 = 底部）
/// - `stiffness`: 弹簧常数 [N/m]
/// - `damping`: 阻尼系数 [N*s/m]
/// - `mu`: 摩擦系数
pub fn make_landing_gear(
    radius: f64,
    y_offset: f64,
    stiffness: f64,
    damping: f64,
    mu: f64,
) -> Vec<TouchdownVertex> {
    let mut pts = Vec::with_capacity(3);
    for i in 0..3 {
        let angle = i as f64 * std::f64::consts::TAU / 3.0;
        let x = radius * angle.cos();
        let z = radius * angle.sin();
        pts.push(TouchdownVertex::with_mu_lng(
            Vec3::new(x, y_offset, z),
            stiffness,
            damping,
            mu,
            mu * 0.8,
        ));
    }
    pts
}

/// 无配置触点时的默认底部触点（圆柱底环三点）。
///
/// 对齐 Orbiter 默认：`stiffness ≈ mass×20`、`damping ≈ 0.1×stiffness`。
/// 仅用于步进物理与坠毁/着陆判定，不写回用户配置。
pub fn default_bottom_touchdown(mass: f64, length: f64, radius: f64) -> Vec<TouchdownVertex> {
    let m = mass.max(1.0);
    let stiffness = (m * 20.0).max(1e5);
    let damping = stiffness * 0.1;
    let ring_r = radius.max(0.1);
    make_landing_gear(ring_r, -length * 0.5, stiffness, damping, 0.5)
}
