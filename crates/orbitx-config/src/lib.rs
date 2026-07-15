//! TOML 配置/场景系统。
//!
//! 用 TOML 格式实现 Orbiter 的三类配置文件：
//! - 火箭配置（rocket.toml）= Orbiter 的 vessel .cfg + clbkSetClassCaps
//! - 场景文件（scenario.toml）= Orbiter 的 .scn
//! - 天体配置（body.toml）= Orbiter 的 planet .cfg

pub mod rocket;
pub mod scenario;

pub use rocket::{RocketConfig, StageConfig};
pub use scenario::{CameraConfig, Environment, Focus, HudConfig, ScenarioConfig, ShipConfig};
