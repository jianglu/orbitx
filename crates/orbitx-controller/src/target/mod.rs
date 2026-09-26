//! `TargetController`——目标导向控制算法（模式 b）。
//!
//! 用户期望航天器朝哪飞、推力多大（即现 cli 控制能力）。内部根据目标 + 当前姿态/状态 +
//! 部分遥测，经 `BaseController` 控制本步怎么飞。**不 own caps**（caps 由 Runtime/WorkFlow
//! 拥有），只 own 算法状态（`TargetMode`、重力转向 kick 进度等）。caps 变化由拥有者换，
//! `TargetController` 经 `base` 观察新 caps，算法状态保留。
//!
//! `TargetMode` 枚举：
//! - `VerticalHold { throttle }` → `apply_tvc(0, 0)`，保竖直。
//! - `PitchTo { pitch, yaw, throttle }` → `apply_tvc(pitch, yaw)`，朝指定姿态。
//! - `ProgradeHold { throttle }` → 由速度方向反解 pitch/yaw 目标（保当前滚转）。
//! - `RetrogradeHold { throttle }` → 反向（`-prograde`）。
//! - `GravityTurn { throttle, kick_angle, kick_rate }` → 标准重力转向：速度过小时竖直；
//!   再 pitchover（按 `kick_rate` 倾到 `kick_angle`）；其后推力∥速度（同 ProgradeHold）。
//!
//! TVC 命令落在本体 caps 的第一个 tvc 组（单 body 通常仅一个）；无 tvc 组则跳过 TVC。
//! 油门策略按本体派生：`Primary` → `SyncPrimary`，`Detached` → `ActiveOnly`。

use crate::base::{BaseController, Controller};
use crate::capability::BodyRef;
use crate::throttle::ThrottlePolicy;
use orbitx_math::{cross, dot, Vec3};

/// 默认 kick 角 [rad]（≈5°）。
pub const DEFAULT_KICK_ANGLE: f64 = 0.087_266; // 5°
/// 默认 kick 速率 [rad/s]。
pub const DEFAULT_KICK_RATE: f64 = 0.05;
/// 速度低于此值时 GravityTurn 保持竖直（避免台架噪声）。
const GRAVITY_TURN_MIN_SPEED: f64 = 1.0;

/// 目标导向模式。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TargetMode {
    /// 保竖直（pitch/yaw 目标 = 0）。
    VerticalHold { throttle: f64 },
    /// 朝指定俯仰/偏航角 [rad]（竖直=0）。
    PitchTo { pitch: f64, yaw: f64, throttle: f64 },
    /// 沿速度方向（prograde）。
    ProgradeHold { throttle: f64 },
    /// 反速度方向（retrograde）。
    RetrogradeHold { throttle: f64 },
    /// 标准重力转向：kick 后推力∥速度（α≈0）。
    GravityTurn {
        throttle: f64,
        /// Pitchover 目标倾角 [rad]。
        kick_angle: f64,
        /// Pitchover 角速度 [rad/s]。
        kick_rate: f64,
    },
}

impl TargetMode {
    /// 该模式的油门开度。
    pub fn throttle(&self) -> f64 {
        match *self {
            TargetMode::VerticalHold { throttle }
            | TargetMode::PitchTo { throttle, .. }
            | TargetMode::ProgradeHold { throttle }
            | TargetMode::RetrogradeHold { throttle }
            | TargetMode::GravityTurn { throttle, .. } => throttle,
        }
    }

    /// 替换油门，其它字段不变。
    pub fn with_throttle(self, throttle: f64) -> Self {
        match self {
            TargetMode::VerticalHold { .. } => TargetMode::VerticalHold { throttle },
            TargetMode::PitchTo { pitch, yaw, .. } => TargetMode::PitchTo { pitch, yaw, throttle },
            TargetMode::ProgradeHold { .. } => TargetMode::ProgradeHold { throttle },
            TargetMode::RetrogradeHold { .. } => TargetMode::RetrogradeHold { throttle },
            TargetMode::GravityTurn {
                kick_angle,
                kick_rate,
                ..
            } => TargetMode::GravityTurn {
                throttle,
                kick_angle,
                kick_rate,
            },
        }
    }

    /// 默认参数的标准重力转向。
    pub fn gravity_turn(throttle: f64) -> Self {
        TargetMode::GravityTurn {
            throttle,
            kick_angle: DEFAULT_KICK_ANGLE,
            kick_rate: DEFAULT_KICK_RATE,
        }
    }
}

/// 目标导向控制器。
pub struct TargetController {
    mode: TargetMode,
    /// GravityTurn kick 段累计俯仰 [rad]。
    turn_pitch: f64,
    /// Kick 是否完成（此后锁 prograde）。
    kick_done: bool,
}

impl TargetController {
    pub fn new(mode: TargetMode) -> Self {
        Self {
            mode,
            turn_pitch: 0.0,
            kick_done: false,
        }
    }

    pub fn mode(&self) -> TargetMode {
        self.mode
    }
    pub fn set_mode(&mut self, mode: TargetMode) {
        self.mode = mode;
    }
    /// 重力转向 kick 累计俯仰角 [rad]。
    pub fn turn_pitch(&self) -> f64 {
        self.turn_pitch
    }
    pub fn kick_done(&self) -> bool {
        self.kick_done
    }
    /// 重置重力转向进度（切模式时由拥有者调）。
    pub fn reset_turn(&mut self) {
        self.turn_pitch = 0.0;
        self.kick_done = false;
    }
}

/// 由本体派生默认油门策略。
fn default_policy(base: &BaseController) -> ThrottlePolicy {
    match base.caps().body {
        BodyRef::Primary => ThrottlePolicy::SyncPrimary,
        BodyRef::Detached(_) => ThrottlePolicy::ActiveOnly,
    }
}

/// 反解速度方向对应的 (pitch, yaw) 目标 [rad]。
pub(crate) fn prograde_target_angles(base: &BaseController, vel: Vec3) -> (f64, f64) {
    let pos = base.position();
    let r_mag = pos.length();
    if r_mag < 1e-3 {
        return (0.0, 0.0);
    }
    let radial = pos * (1.0 / r_mag);
    let d = vel.unit();
    let (bx, _by, _bz) = base.body_axes();
    let mut tx = bx - d * dot(bx, d);
    if tx.length2() < 1e-18 {
        tx = radial;
    }
    tx = tx.unit();
    let tz = cross(tx, d).unit();
    let p = dot(radial, tz).clamp(-1.0, 1.0);
    let y = (-dot(radial, tx)).clamp(-1.0, 1.0);
    (p.asin(), y.asin())
}

impl Controller for TargetController {
    fn tick(&mut self, base: &mut BaseController, dt: f64) {
        let throttle = self.mode.throttle();
        let policy = default_policy(base);
        base.set_throttle(policy, throttle);

        let (pitch_t, yaw_t) = match self.mode {
            TargetMode::VerticalHold { .. } => (0.0, 0.0),
            TargetMode::PitchTo { pitch, yaw, .. } => (pitch, yaw),
            TargetMode::ProgradeHold { .. } => {
                let v = base.velocity();
                if v.length2() < 1e-12 {
                    (0.0, 0.0)
                } else {
                    prograde_target_angles(base, v)
                }
            }
            TargetMode::RetrogradeHold { .. } => {
                let v = base.velocity();
                if v.length2() < 1e-12 {
                    (0.0, 0.0)
                } else {
                    let (p, y) = prograde_target_angles(base, v);
                    (-p, -y)
                }
            }
            TargetMode::GravityTurn {
                kick_angle,
                kick_rate,
                ..
            } => {
                let v = base.velocity();
                let speed = v.length();
                if speed < GRAVITY_TURN_MIN_SPEED {
                    (0.0, 0.0)
                } else if !self.kick_done {
                    let kick = kick_angle.max(0.0);
                    self.turn_pitch =
                        (self.turn_pitch + kick_rate.max(0.0) * dt).clamp(0.0, kick.max(1e-9));
                    if self.turn_pitch >= kick - 1e-9 {
                        self.kick_done = true;
                    }
                    (self.turn_pitch, 0.0)
                } else {
                    prograde_target_angles(base, v)
                }
            }
        };

        if let Some(g) = base.caps().tvc_groups.first() {
            let id = g.id.clone();
            base.apply_tvc(&id, pitch_t, yaw_t, dt);
        }
    }
}

#[cfg(test)]
mod tests;
