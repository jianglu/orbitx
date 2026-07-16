//! 多级火箭系统：Vessel + Assembly + 对接/分离。
//!
//! 参照 Orbiter 的多模块设计：每个火箭级是一个独立的 Vessel 实体，
//! 通过对接端口连接，分离时解除对接。

pub mod aero;
pub mod assembly;
pub mod dock;
pub mod fuel;
pub mod rcs;
pub mod stage;
pub mod thruster;
pub mod touchdown;
pub mod vessel;

// 预设火箭配置。
pub mod presets;

#[cfg(test)]
mod tests;

pub use aero::{
    AeroForces, Airfoil, AirfoilCoeffs, AirfoilOrientation, Atmosphere,
    ControlSurface, CtrlAxis, CtrlType, DragElement, ExponentialAtmosphere,
    compute_aero_forces, world_to_airvel_ship,
};
pub use assembly::Assembly;
pub use dock::DockPort;
pub use fuel::PropellantTank;
pub use rcs::{
    ThrusterGroup, ThrusterGroupType, RotAxis, LinAxis,
    add_default_rcs, set_group_level, get_group_level,
    set_attitude_rot, set_attitude_lin,
};
pub use stage::StageSpec;
pub use thruster::{Thruster, G0};
pub use touchdown::{
    TouchdownVertex, SurfaceContact,
    compute_surface_forces, make_landing_gear,
};
pub use vessel::Vessel;
