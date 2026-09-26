//! 飞行输入与会话命令（进程内契约；线上 protobuf 见 orbitx-protocol）。

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InputCmd {
    SetThrottle { level: f64 },
    SetAttitudeAxes { pitch: f64, yaw: f64, roll: f64 },
    Separate,
    SetGravityTurn { enabled: bool },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SessionCmd {
    Pause,
    Resume,
    SetWarp { scale: f64 },
    Step { n: u32 },
    Shutdown,
    Reset,
}
