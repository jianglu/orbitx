//! 行星环境：按 `scenario_xxx.toml` 步进天体，并向积分器提供力学子集。
//!
//! 权威设计见 `docs/ENVIRONMENT.md`。引力、Pines、自转公式、大气模型留在 `orbitx-dynamics`。

pub mod body;
pub mod ephem_path;
pub mod frame;
pub mod system;

pub use body::{CelestialBody, EphemerisModel, GravityModel};
pub use ephem_path::resolve_ephemeris_data;
pub use frame::PrimarySurface;
pub use system::PlanetarySystem;
