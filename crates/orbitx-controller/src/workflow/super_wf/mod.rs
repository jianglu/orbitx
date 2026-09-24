//! `SuperWorkFlow`（模式 d）：复杂自动控制工作流（入轨自动驾驶）。
//!
//! own 子控制器舰队 + 各 body caps + toml 编排状态。tick 按 `[[steps]]` 序列为每 body 构造
//! `BaseController`、经 base 命令各执行器；分离时重建主 caps。
//!
//! **P4.1 阶段 B 骨架实现**：基础步骤序列执行器——每 tick 执行当前步骤命令，按步骤完成
//! 条件推进。`Throttle` / `Tvc` / `Rcs` / `Separate` 为即时命令（执行一次，下一 tick 推进）；
//! `Wait` 持续 `duration` 后推进。完整的「子控制器舰队 + 连续姿态保持 + 分离派生子控制器 +
//! 入轨自动驾驶」留待 P4.2+（见 `ROADMAP.md`）；本骨架验证步骤调度与执行器接线。

use orbitx_vessel::{Assembly, RotAxis};

use crate::base::BaseController;
use crate::capability::ControlCapability;
use crate::workflow::{StepDesc, WorkFlow};

/// 复杂自动控制工作流（模式 d，P4.1 骨架）。
pub struct SuperWorkFlow {
    caps: ControlCapability,
    steps: Vec<StepDesc>,
    step_idx: usize,
    step_time: f64,
    done: bool,
}

impl SuperWorkFlow {
    /// 由解析后的步骤列表 + 主 body caps 构建。
    pub fn new(steps: Vec<StepDesc>, caps: ControlCapability) -> Self {
        assert!(!steps.is_empty(), "SuperWorkFlow 需至少一个步骤");
        Self {
            caps,
            steps,
            step_idx: 0,
            step_time: 0.0,
            done: false,
        }
    }

    pub fn step_idx(&self) -> usize {
        self.step_idx
    }

    /// 分离后重建主 body caps（事件驱动，由本工作流在 `Separate` 步骤后自动调）。
    fn rebuild_caps(&mut self, asm: &Assembly) {
        self.caps = ControlCapability::for_primary(asm);
    }

    /// 解析 rcs 轴字符串。
    fn parse_axis(s: &str) -> RotAxis {
        match s {
            "pitch" => RotAxis::Pitch,
            "yaw" => RotAxis::Yaw,
            "bank" => RotAxis::Bank,
            other => panic!("未知 rcs 轴: {other}（应为 pitch/yaw/bank）"),
        }
    }
}

impl WorkFlow for SuperWorkFlow {
    fn tick(&mut self, asm: &mut Assembly, dt: f64) {
        if self.done {
            return;
        }
        let step = self.steps[self.step_idx].clone();
        let mut advance = false;
        let mut need_rebuild = false;
        {
            let caps = &self.caps;
            let mut base = BaseController::new(asm, caps);
            match step {
                StepDesc::Throttle { group, level } => {
                    base.set_group_throttle(&group, level);
                    advance = true;
                }
                StepDesc::Tvc { group, pitch, yaw } => {
                    base.apply_tvc(&group, pitch, yaw, dt);
                    advance = true;
                }
                StepDesc::Rcs { group, axis, level } => {
                    let ax = Self::parse_axis(&axis);
                    base.set_rcs(&group, ax, level);
                    advance = true;
                }
                StepDesc::Separate { point } => {
                    base.separate(&point);
                    advance = true;
                    need_rebuild = true;
                }
                StepDesc::Wait { duration } => {
                    // 等待期间不命令执行器；累计时间到则推进。
                    if self.step_time >= duration {
                        advance = true;
                    }
                }
            }
        }
        if need_rebuild {
            self.rebuild_caps(asm);
        }
        if advance {
            if self.step_idx + 1 < self.steps.len() {
                self.step_idx += 1;
                self.step_time = 0.0;
            } else {
                self.done = true;
            }
        } else {
            self.step_time += dt;
        }
    }

    fn is_done(&self) -> bool {
        self.done
    }
}

#[cfg(test)]
mod tests;
