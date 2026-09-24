//! 航天器控制策略（P4.1）。
//!
//! 分层（详见 crate `README.md` 与 `orbitx/docs/ARCHITECTURE.md`）：
//!
//! - [`base`]：`BaseController`——vessel 唯一全功能门面，既读遥测也写执行器。
//!   `TargetController` / `WorkFlow` 都不直接碰 `Assembly`，必经 `BaseController`。
//! - [`target`]：`TargetController`——目标导向控制算法（模式 b），经 `BaseController`
//!   读姿态/状态/部分遥测，算本步怎么飞，再经 `BaseController` 写执行器。不 own caps。
//! - [`workflow`]：`WorkFlow`（Target/Super）——Godot 编辑的工作流 toml 解析 + 编排。
//!   WorkFlow 模式下自管子控制器；SuperWorkFlow 实现入轨自动驾驶。
//! - [`capability`]：`ControlCapability` 契约 + `BodyRef`，按需 `for_primary` / `for_detached` 构建。
//! - [`throttle`] / [`tvc`] / [`separation`] / [`rcs`]：执行器原语算法，由 `BaseController` 调用。
//! - [`factory`]：`build_controller` / `build_workflow` + `ControllerAssignment`。
//!
//! 控制器不 own `ControlCapability`（caps 由 Runtime 或 WorkFlow 拥有，避免 tick 时
//! `&mut self` 与 `base.caps()` 借用冲突）；控制器只 own 算法状态。
//!
//! 显示遥测不经本 crate：Runtime 直接经 `orbitx_vessel::telemetry` 读 `Assembly`
//! → 切片 → zenoh → 客户端。`telemetry` trait 的两个消费者：`BaseController`
//! （控制环读）与 Runtime（显示切片读），都直读 `Assembly`。

pub mod base;
pub mod capability;
pub mod factory;
pub mod rcs;
pub mod separation;
pub mod target;
pub mod throttle;
pub mod tvc;
pub mod workflow;
