//! 飞行输入与会话命令（P4.2 进程内契约；网络编码 → P4.3）。

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InputCmd {
    SetThrottle { level: f64 },
    SetAttitudeAxes { pitch: f64, yaw: f64, roll: f64 },
    Separate,
    SetFocus { body: String },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SessionCmd {
    Pause,
    Resume,
    SetWarp { scale: f64 },
    Step { n: u32 },
    Shutdown,
}
