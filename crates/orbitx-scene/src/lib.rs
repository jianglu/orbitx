//! 共享场景模块：历表加载 + 坐标转换桥接。
//!
//! 被 `orbitx-orrery`（太阳系仪）和 `orbitx-flight`（航天器飞行）共用。

pub mod bridge;
pub mod sim;

pub use bridge::{scale_radius, CameraFrame, AU_METERS};
pub use sim::{BodyState, Simulation, MJD2000};
