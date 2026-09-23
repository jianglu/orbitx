//! 气动力与大气模型（薄 re-export 层）。
//!
//! 物理算法与数据类型已下沉到 `orbitx_dynamics`：
//! - 气动力计算 → `dynamics::aero`
//! - 大气模型 → `dynamics::atmosphere`
//! - Cd(M) 查表插值 → `orbitx_math::piecewise_linear`
//!
//! 本模块仅做 re-export 以保持 `orbitx_vessel` 公共 API 不变。

pub use orbitx_dynamics::{
    compute_aero_forces, world_to_airvel_ship, AeroForces, Airfoil, AirfoilCoeffs,
    AirfoilOrientation, ControlSurface, CtrlAxis, CtrlType, DragElement,
};

pub use orbitx_dynamics::{
    atmosphere_from_config, Atmosphere, ExponentialAtmosphere, UsStd1976Atmosphere,
};

/// 分段线性 Cd(M) 查表插值（转发到 `orbitx_math::piecewise_linear`）。
///
/// 保留旧名以兼容既有调用方；新代码应直接用 `orbitx_math::piecewise_linear`。
pub fn interpolate_cd_mach(table: &[(f64, f64)], mach: f64, fallback: f64) -> f64 {
    orbitx_math::piecewise_linear(table, mach, fallback)
}
