//! HUD/MFD 叠加层 — 移植自 Orbiter `hud.cpp`（2,147 行）+ `Mfd.cpp`（1,290 行）。
//!
//! 使用 egui 即时模式 GUI 绘制 HUD 元素和 MFD 仪器面板。

pub mod flight_state;
pub mod hud;
pub mod mfd;

pub use flight_state::FlightState;
pub use hud::{HudState, HudMode, HudColor, HudElements};
pub use mfd::{MfdPanel, MfdType, MfdSize};
