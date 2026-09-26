//! orbitx-cli 库：Zenoh 客户端辅助、自动分离判据、TUI 绘制。
//!
//! 二进制 `main.rs` 负责 spawn `orbitx-runtime` 与交互循环；**不**持有/步进 Assembly。

pub mod auto_sep;
pub mod spawn;
pub mod ui;
pub mod zenoh_client;

// 旧进程内模块保留单测（control/crash/focus/telem）；产品路径不再使用。
pub mod control;
pub mod crash;
pub mod focus;
pub mod telem;
