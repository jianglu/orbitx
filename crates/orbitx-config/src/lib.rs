//! TOML 配置。
//!
//! 类型约束见 `docs/CONFIG_TOML.md`。环境文件是 `scenario_xxx.toml`（`PlanetaryScenario`）。
//! 火箭预设仍由 `RocketConfig` 加载，直到 `sc_xxx/sim.toml` 落地。

pub mod body;
pub mod planetary_scenario;
pub mod rocket;
pub mod scenario;
pub mod system;

pub use body::{
    AtmosphereConfig, AtmosphereModel, BodyConfig, EphemerisConfig, GravityConfig, RotationConfig,
};
pub use rocket::{
    builtin_aliases, builtin_rocket_toml, load_rocket_source, DockConfig, DockLinkConfig,
    RocketConfig, StageConfig, ThrusterConfig,
};
pub use planetary_scenario::{resolve_scenario_spec, PlanetaryScenario};
pub use scenario::{CameraConfig, Environment, Focus, HudConfig, ScenarioConfig, ShipConfig};
pub use system::SystemConfig;
