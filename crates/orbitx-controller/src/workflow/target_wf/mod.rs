//! `TargetWorkFlow`（模式 c）：简易目标导向工作流。
//!
//! own 1 个 `TargetController`（主 body）+ 主 body caps。tick 按阶段条件切换
//! `TargetController` 目标配置，构造主 body `BaseController` 调 `target.tick`。
//!
//! 条件读上一步切片，不再用 `mu` 现算远近地点。`mu` 仍由工厂传入，避免改构造签名。

use orbitx_protocol::mem::Slice;
use orbitx_vessel::Assembly;

use crate::base::{BaseController, Controller};
use crate::capability::ControlCapability;
use crate::target::TargetController;
use crate::workflow::{PhaseDesc, TransitionDesc, WorkFlow};

/// 简易目标导向工作流（模式 c）。
pub struct TargetWorkFlow {
    target: TargetController,
    caps: ControlCapability,
    phases: Vec<PhaseDesc>,
    phase_idx: usize,
    phase_time: f64,
    /// 工厂仍传入。远近地点改读切片。
    #[allow(dead_code)]
    mu: f64,
    done: bool,
}

impl TargetWorkFlow {
    /// 由解析后的阶段列表 + 主 body caps + 中心体引力参数构建。
    /// `mu` 用于 apoapsis/periapsis 条件；不需要时传 0。
    pub fn new(phases: Vec<PhaseDesc>, caps: ControlCapability, mu: f64) -> Self {
        assert!(!phases.is_empty(), "TargetWorkFlow 需至少一个阶段");
        let first_mode = phases[0].mode.to_mode();
        Self {
            target: TargetController::new(first_mode),
            caps,
            phases,
            phase_idx: 0,
            phase_time: 0.0,
            mu,
            done: false,
        }
    }

    pub fn phase_idx(&self) -> usize {
        self.phase_idx
    }
    pub fn phase_time(&self) -> f64 {
        self.phase_time
    }

    /// 评估当前阶段是否应进入下一阶段。
    fn transition_met(&self, slice: &Slice) -> bool {
        let Some(t) = self.phases[self.phase_idx].transition.as_ref() else {
            return false;
        };
        t.is_met(slice, self.phase_time)
    }

    fn advance(&mut self) {
        if self.phase_idx + 1 < self.phases.len() {
            self.phase_idx += 1;
            let mode = self.phases[self.phase_idx].mode.to_mode();
            // 进入新阶段（尤其 GravityTurn）时重置累计俯仰。
            self.target.reset_turn();
            self.target.set_mode(mode);
            self.phase_time = 0.0;
        } else {
            // 末段 transition 满足 → 完成。
            self.done = true;
        }
    }
}

impl WorkFlow for TargetWorkFlow {
    fn tick(&mut self, asm: &mut Assembly, slice: &Slice, dt: f64) {
        if self.transition_met(slice) {
            self.advance();
        }
        // 2. 构造主 body BaseController，调 target.tick（分离借用 caps / target）。
        let caps = &self.caps;
        let target = &mut self.target;
        {
            let mut base = BaseController::new(asm, caps);
            target.tick(&mut base, dt);
        }
        // 3. 累计阶段时间。
        self.phase_time += dt;
    }

    fn throttle_cmd(&self) -> f64 {
        self.target.mode().throttle()
    }

    fn attitude_targets(&self) -> (f64, f64, f64) {
        crate::workflow::attitude_from_target(&self.target)
    }

    fn nose_target(&self) -> Option<[f64; 3]> {
        self.target.nose_cmd()
    }

    fn is_done(&self) -> bool {
        self.done
    }
}

impl TransitionDesc {
    /// 评估过渡条件是否满足。`phase_time` 为当前阶段累计时长；`mu` 为中心体引力参数。
    fn is_met(&self, slice: &Slice, phase_time: f64) -> bool {
        crate::workflow::pred::eval_term(self, slice, phase_time, "")
    }
}

#[cfg(test)]
mod tests;
