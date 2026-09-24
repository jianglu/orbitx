//! `TargetController`——目标导向控制算法（模式 b）。
//!
//! 用户期望航天器朝哪飞、推力多大（即现 cli 控制能力）。内部根据目标 + 当前姿态/状态 +
//! 部分遥测，经 `BaseController` 控制本步怎么飞。**不 own caps**（caps 由 Runtime/WorkFlow
//! 拥有），只 own 算法状态（`TargetMode`、重力转向累计俯仰等）。caps 变化由拥有者换，
//! `TargetController` 经 `base` 观察新 caps，算法状态保留。
//!
//! `TargetMode` 枚举：
//! - `VerticalHold { throttle }` → `apply_tvc(0, 0)`，保竖直。
//! - `PitchTo { pitch, yaw, throttle }` → `apply_tvc(pitch, yaw)`，朝指定姿态。
//! - `ProgradeHold { throttle }` → 由速度方向反解 pitch/yaw 目标（保当前滚转）。
//! - `RetrogradeHold { throttle }` → 反向（`-prograde`）。
//! - `GravityTurn { throttle, pitch_rate }` → 渐进俯仰：每 tick 累计 `pitch_rate·dt`，
//!   保低迎角；`apply_tvc(turn_pitch, 0)`。
//!
//! TVC 命令落在本体 caps 的第一个 tvc 组（单 body 通常仅一个）；无 tvc 组则跳过 TVC。
//! 油门策略按本体派生：`Primary` → `SyncPrimary`，`Detached` → `ActiveOnly`。

use crate::base::{BaseController, Controller};
use crate::capability::BodyRef;
use crate::throttle::ThrottlePolicy;
use orbitx_math::{cross, dot, Vec3};

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
    /// 重力转向：俯仰以 `pitch_rate` [rad/s] 渐进，偏航保 0。
    GravityTurn { throttle: f64, pitch_rate: f64 },
}

impl TargetMode {
    /// 该模式的油门开度（GravityTurn 也带油门）。
    pub fn throttle(&self) -> f64 {
        match *self {
            TargetMode::VerticalHold { throttle }
            | TargetMode::PitchTo { throttle, .. }
            | TargetMode::ProgradeHold { throttle }
            | TargetMode::RetrogradeHold { throttle }
            | TargetMode::GravityTurn { throttle, .. } => throttle,
        }
    }
}

/// 目标导向控制器。
pub struct TargetController {
    mode: TargetMode,
    /// 重力转向累计俯仰角 [rad]（仅 `GravityTurn` 用）。
    turn_pitch: f64,
}

impl TargetController {
    pub fn new(mode: TargetMode) -> Self {
        Self {
            mode,
            turn_pitch: 0.0,
        }
    }

    pub fn mode(&self) -> TargetMode {
        self.mode
    }
    pub fn set_mode(&mut self, mode: TargetMode) {
        self.mode = mode;
    }
    /// 重力转向累计俯仰角 [rad]（仅 `GravityTurn` 模式有意义）。
    pub fn turn_pitch(&self) -> f64 {
        self.turn_pitch
    }
    /// 重置重力转向进度（切模式时由拥有者调）。
    pub fn reset_turn(&mut self) {
        self.turn_pitch = 0.0;
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
///
/// 始终按 prograde 方向（`dir = vel`）构造目标体轴：`target_y = prograde`；`target_x` 由
/// 当前体 +X 投影到 prograde 正交平面（保滚转连续），退化时回退 `+radial`。然后
/// `target_pitch = asin(clamp(radial·target_z))`、`target_yaw = asin(clamp(-radial·target_x·radial))`，
/// 与 `orbitx_dynamics::kinematics::pitch_yaw_angles` 同分解。
///
/// retrograde = prograde 的 180° 翻转：在 pitch/yaw 空间等价于双轴取负（小角度：body_z
/// 反向 → pitch 反号；大角度水平：yaw 从 -π/2 翻到 +π/2）。由 `sign`（+1 prograde / -1 retrograde）
/// 在调用侧乘回。
pub(crate) fn prograde_target_angles(base: &BaseController, vel: Vec3) -> (f64, f64) {
    let pos = base.position();
    let r_mag = pos.length();
    if r_mag < 1e-3 {
        return (0.0, 0.0);
    }
    let radial = pos * (1.0 / r_mag);
    let d = vel.unit();
    let (bx, _by, _bz) = base.body_axes();
    // target_x：当前 bx 去掉沿 d 的分量，再归一化；退化（bx∥d）时回退 +radial。
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
        // 先下油门（与 cli 同序：TVC 先于油门也可，二者作用于不同执行器，互不干扰）。
        base.set_throttle(policy, throttle);

        // TVC 目标。
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
                    // retrograde = prograde 的 180° 翻转：pitch/yaw 双轴取负。
                    let (p, y) = prograde_target_angles(base, v);
                    (-p, -y)
                }
            }
            TargetMode::GravityTurn { pitch_rate, .. } => {
                self.turn_pitch = (self.turn_pitch + pitch_rate * dt).clamp(0.0, std::f64::consts::FRAC_PI_2);
                (self.turn_pitch, 0.0)
            }
        };

        // 命令第一个 tvc 组（单 body 通常仅一个）；无则跳过。
        if let Some(g) = base.caps().tvc_groups.first() {
            let id = g.id.clone();
            base.apply_tvc(&id, pitch_t, yaw_t, dt);
        }
    }
}

#[cfg(test)]
mod tests;
