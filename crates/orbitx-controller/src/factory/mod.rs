//! 工厂与场景配置：上层按需求实例化控制器 / 工作流。
//!
//! Runtime（P4.2）顶层二选一（`Control`）：
//! - **手动模式（`Control::Controller`）**：Runtime 持 `Vec<ControllerEntry>` + 焦点 `BodyRef`，
//!   输入路由到焦点控制器；分离时 Runtime 重建主 caps + 按 [`ControllerAssignment`] 派生子控制器。
//! - **WorkFlow 模式（`Control::WorkFlow`）**：Runtime 持 `Box<dyn WorkFlow>` 一个；WorkFlow
//!   内部 own 子控制器 + 各 body caps，分离时 WorkFlow 自己重建 caps + 派生子控制器。
//!
//! `Control` 定义在本 crate（紧邻 factory），`build_control` 返回 `Control`；Runtime 持有并驱动。

use std::collections::BTreeMap;

use orbitx_vessel::Assembly;

use crate::base::Controller;
use crate::capability::BodyRef;
use crate::target::{TargetController, TargetMode};
use crate::workflow::{WorkFlow, WorkFlowDesc, WorkFlowKind};
use crate::workflow::super_wf::SuperWorkFlow;
use crate::workflow::target_wf::TargetWorkFlow;

/// 顶层控制约定：二选一闭集（穷尽 match，无顶层 vtable 间接）。
///
/// 叶控制器（`Box<dyn Controller>`）与 workflow（`Box<dyn WorkFlow>`）仍用 trait 对象；
/// 本枚举只在顶层把两模式收敛为闭集。
pub enum Control {
    /// 手动模式：多控制器按 `BodyRef` 索引（Runtime 持焦点 `BodyRef`，输入路由到焦点控制器）。
    Controller(Vec<ControllerEntry>),
    /// WorkFlow 模式：单一编排。
    WorkFlow(Box<dyn WorkFlow>),
}

/// 手动模式条目：可控体 + 其叶控制器。
pub struct ControllerEntry {
    pub body: BodyRef,
    pub controller: Box<dyn Controller>,
}

/// 构建叶目标控制器（模式 b）。
pub fn build_controller(mode: TargetMode) -> Box<dyn Controller> {
    Box::new(TargetController::new(mode))
}

/// 构建工作流（由解析后的 `WorkFlowDesc` + 初始 `Assembly` 投影 caps + 中心体引力参数）。
///
/// `mu` 用于 target workflow 的 apoapsis/periapsis 过渡条件；不需要时传 0。
/// caps 由工作流 own（target/super 均 `for_primary` 一次性构建；分离事件点由工作流重建）。
pub fn build_workflow(desc: &WorkFlowDesc, asm: &Assembly, mu: f64) -> Box<dyn WorkFlow> {
    match desc.kind {
        WorkFlowKind::Target => {
            let phases = desc
                .phases
                .clone()
                .expect("target 工作流需要 phases（schema 校验保证）");
            let caps = crate::capability::ControlCapability::for_primary(asm);
            Box::new(TargetWorkFlow::new(phases, caps, mu))
        }
        WorkFlowKind::Super => {
            let steps = desc
                .steps
                .clone()
                .expect("super 工作流需要 steps（schema 校验保证）");
            let caps = crate::capability::ControlCapability::for_primary(asm);
            Box::new(SuperWorkFlow::new(steps, caps))
        }
    }
}

/// 构建顶层 `Control`（WorkFlow 模式）。
pub fn build_control(desc: &WorkFlowDesc, asm: &Assembly, mu: f64) -> Control {
    Control::WorkFlow(build_workflow(desc, asm, mu))
}

/// 构建手动模式顶层 `Control`：单主 body 目标控制器（最简手动场景）。
pub fn build_manual_control(mode: TargetMode) -> Control {
    Control::Controller(vec![ControllerEntry {
        body: BodyRef::Primary,
        controller: build_controller(mode),
    }])
}

/// 场景配置：手动模式下分离出新 detached vessel 时，按 vessel 名派生子控制器（P4.1 建类型，P4.2 接线）。
///
/// Runtime 在分离事件后查 `derive_mode(vessel_name)`；命中则 `for_detached(asm, idx)` 构建 caps +
/// `build_controller(mode)` 派生子控制器并入 `Control::Controller` 列表。
#[derive(Debug, Clone, Default)]
pub struct ControllerAssignment {
    rules: BTreeMap<String, TargetMode>,
}

impl ControllerAssignment {
    pub fn new() -> Self {
        Self { rules: BTreeMap::new() }
    }
    /// 为指定 vessel 名登记派生模式（builder 风格）。
    pub fn for_vessel(mut self, name: &str, mode: TargetMode) -> Self {
        self.rules.insert(name.into(), mode);
        self
    }
    /// 查 vessel 名对应的派生模式（无匹配返回 `None` → 该 detached vessel 不派生控制器）。
    pub fn derive_mode(&self, vessel_name: &str) -> Option<TargetMode> {
        self.rules.get(vessel_name).copied()
    }
    /// 已登记规则数。
    pub fn len(&self) -> usize {
        self.rules.len()
    }
    pub fn is_empty(&self) -> bool {
        self.rules.is_empty()
    }
}

#[cfg(test)]
mod tests;
