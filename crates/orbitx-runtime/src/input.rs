//! 飞行输入与会话命令（进程内契约；线上 protobuf 见 orbitx-protocol）。

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InputCmd {
    SetThrottle {
        level: f64,
    },
    SetAttitudeAxes {
        pitch: f64,
        yaw: f64,
        roll: f64,
    },
    Separate,
    SetGravityTurn {
        enabled: bool,
    },
    /// 图工作流点火（一次命令）。线性 phases/steps 工作流忽略。
    WorkflowIgnite,
    /// RCS 姿态参与开关（target 模式 R 键）：开时 TargetController tick 内按姿态
    /// 误差驱动 RCS 三轴（roll 仅 RCS 可执行）；关时只 TVC。
    SetRcsEnabled {
        enabled: bool,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SessionCmd {
    Pause,
    Resume,
    SetWarp { scale: f64 },
    Step { n: u32 },
    Shutdown,
    Reset,
    /// Godot 视觉档权威：热更新地表工作深度（钳在包 max_level）。
    SetSurfaceWorkingMaxLevel { level: u32 },
    /// 发射门禁解禁（预发射 hold：首帧输出后不步进，Launch 才开始步进）。
    Launch,
}
