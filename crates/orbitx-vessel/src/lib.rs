//! 多级火箭系统：Vessel + Assembly + 对接/分离。
//!
//! 参照 Orbiter 的多模块设计：每个火箭级是一个独立的 Vessel 实体，
//! 通过对接端口连接，分离时解除对接。

pub mod assembly;
pub mod dock;
pub mod stage;
pub mod thruster;
pub mod vessel;

// 预设火箭配置。
pub mod presets;

#[cfg(test)]
mod tests;

pub use assembly::Assembly;
pub use dock::DockPort;
pub use stage::StageSpec;
pub use thruster::{Thruster, G0};
pub use vessel::Vessel;
