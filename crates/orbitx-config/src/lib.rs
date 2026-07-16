//! TOML 配置/场景系统。
//!
//! 用 TOML 格式实现 Orbiter 的三类配置文件：
//! - 火箭配置（rocket.toml）= Orbiter 的 vessel .cfg + clbkSetClassCaps
//! - 场景文件（scenario.toml）= Orbiter 的 .scn
//! - 天体配置（body.toml）= Orbiter 的 planet .cfg
//! - 太阳系配置（system.toml）= Orbiter 的 Sol.cfg

pub mod body;
pub mod rocket;
pub mod scenario;
pub mod system;

pub use body::{
    AtmosphereConfig, BodyConfig, EphemerisConfig, GravityConfig, RotationConfig,
};
pub use rocket::{RocketConfig, StageConfig};
pub use scenario::{CameraConfig, Environment, Focus, HudConfig, ScenarioConfig, ShipConfig};
pub use system::SystemConfig;
