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
//! - `GravityTurn { throttle, kick_pitch, kick_yaw, kick_rate, min_alt, min_speed }` → 标准重力转向：
//!   高度或地面速度低于门槛时竖直；再按 `kick_rate` 把姿态指令爬到 `(kick_pitch, kick_yaw)`。
//!   播种段和其后都调用 `apply_tvc`：播种时跟随爬升指令，机头接近播种角且地速在播种方向同侧后，
//!   改为推力∥地面速度（无风时即空速，α≈0）。
//!
//! TVC 命令落在本体 caps 的第一个 tvc 组（单 body 通常仅一个）；无 tvc 组则跳过 TVC。
//! 油门策略按本体派生：`Primary` → `SyncPrimary`，`Detached` → `ActiveOnly`。

use crate::base::{BaseController, Controller};
use crate::capability::BodyRef;
use crate::throttle::ThrottlePolicy;
use orbitx_math::{cross, dot, Vec3};

/// 默认播种俯仰 [rad]。默认赤道台上向东落在偏航，俯仰保持 0。
pub const DEFAULT_KICK_PITCH: f64 = 0.0;
/// 默认播种偏航 [rad]（≈+5°）。默认赤道台上 body +X 为当地向东，正偏航把机头朝 +X。
pub const DEFAULT_KICK_YAW: f64 = 0.087_266;
/// 默认播种指令爬升速率 [rad/s]。
pub const DEFAULT_KICK_RATE: f64 = 0.05;
/// 开始 kick 的最低质心高度 [m]。150 m 箭长时尾部约在 425 m，高出塔身两百多米。
pub const DEFAULT_MIN_ALT: f64 = 500.0;
/// 开始 kick 的最低地面速度 [m/s]。
pub const DEFAULT_MIN_SPEED: f64 = 50.0;
/// 机头与最终播种角的接近阈值 [rad]（0.09°）。俯仰和偏航都落在此范围内才结束播种。
pub const SEED_NEAR: f64 = 0.09_f64.to_radians();

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
    /// 标准重力转向：过门槛后播种一次俯仰/偏航，再让推力对齐地面速度（α≈0）。
    GravityTurn {
        throttle: f64,
        /// 播种俯仰 [rad]。
        kick_pitch: f64,
        /// 播种偏航 [rad]。
        kick_yaw: f64,
        /// 播种指令沿 `(kick_pitch, kick_yaw)` 爬升的速率 [rad/s]。
        kick_rate: f64,
        /// 开始 kick 的最低质心高度 [m]。
        min_alt: f64,
        /// 开始 kick 的最低地面速度 [m/s]。
        min_speed: f64,
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
            TargetMode::PitchTo { pitch, yaw, .. } => TargetMode::PitchTo {
                pitch,
                yaw,
                throttle,
            },
            TargetMode::ProgradeHold { .. } => TargetMode::ProgradeHold { throttle },
            TargetMode::RetrogradeHold { .. } => TargetMode::RetrogradeHold { throttle },
            TargetMode::GravityTurn {
                kick_pitch,
                kick_yaw,
                kick_rate,
                min_alt,
                min_speed,
                ..
            } => TargetMode::GravityTurn {
                throttle,
                kick_pitch,
                kick_yaw,
                kick_rate,
                min_alt,
                min_speed,
            },
        }
    }

    /// 默认参数的标准重力转向。播种指向默认赤道台上的当地向东。
    pub fn gravity_turn(throttle: f64) -> Self {
        TargetMode::GravityTurn {
            throttle,
            kick_pitch: DEFAULT_KICK_PITCH,
            kick_yaw: DEFAULT_KICK_YAW,
            kick_rate: DEFAULT_KICK_RATE,
            min_alt: DEFAULT_MIN_ALT,
            min_speed: DEFAULT_MIN_SPEED,
        }
    }
}

/// 目标导向控制器。
pub struct TargetController {
    mode: TargetMode,
    /// GravityTurn 播种指令沿 `(kick_pitch, kick_yaw)` 已爬过的弧长 [rad]。
    turn_pitch: f64,
    /// Kick 是否完成（此后锁 prograde）。
    kick_done: bool,
    /// 本步实际下发的俯仰目标 [rad]（GravityTurn 按阶段更新）。
    pitch_cmd: f64,
    /// 本步实际下发的偏航目标 [rad]。
    yaw_cmd: f64,
}

impl TargetController {
    pub fn new(mode: TargetMode) -> Self {
        Self {
            mode,
            turn_pitch: 0.0,
            kick_done: false,
            pitch_cmd: 0.0,
            yaw_cmd: 0.0,
        }
    }

    pub fn mode(&self) -> TargetMode {
        self.mode
    }
    pub fn set_mode(&mut self, mode: TargetMode) {
        self.mode = mode;
    }
    /// 重力转向播种指令已爬过的弧长 [rad]。纯俯仰播种时等于俯仰指令。
    pub fn turn_pitch(&self) -> f64 {
        self.turn_pitch
    }
    pub fn kick_done(&self) -> bool {
        self.kick_done
    }
    /// 本步下发的俯仰目标 [rad]。
    pub fn pitch_cmd(&self) -> f64 {
        self.pitch_cmd
    }
    /// 本步下发的偏航目标 [rad]。
    pub fn yaw_cmd(&self) -> f64 {
        self.yaw_cmd
    }
    /// 重置重力转向进度（切模式时由拥有者调）。
    pub fn reset_turn(&mut self) {
        self.turn_pitch = 0.0;
        self.kick_done = false;
        self.pitch_cmd = 0.0;
        self.yaw_cmd = 0.0;
    }
}

/// 由本体派生默认油门策略。
fn default_policy(base: &BaseController) -> ThrottlePolicy {
    match base.caps().body {
        BodyRef::Primary => ThrottlePolicy::SyncPrimary,
        BodyRef::Detached(_) => ThrottlePolicy::ActiveOnly,
    }
}

/// 播种方向上的地速倾角 [rad]。同侧为正。
///
/// 水平地速在播种方向上的投影对垂直地速做 `atan2`。+俯仰沿 body −Z 的水平投影，
/// +偏航沿 body +X 的水平投影（默认赤道台上即当地向东）。
fn flight_path_along_seed(base: &BaseController, v_g: Vec3, kick_pitch: f64, kick_yaw: f64) -> f64 {
    if v_g.length2() < 1e-12 {
        return 0.0;
    }
    let pos = base.position();
    let r_mag = pos.length();
    if r_mag < 1e-3 {
        return 0.0;
    }
    let radial = pos * (1.0 / r_mag);
    let v_vert = dot(v_g, radial);
    let (bx, _by, bz) = base.body_axes();
    let pitch_h = horizontal(-bz, radial);
    let yaw_h = horizontal(bx, radial);
    let raw = pitch_h * kick_pitch + yaw_h * kick_yaw;
    if raw.length2() < 1e-18 {
        return 0.0;
    }
    let v_along = dot(v_g - radial * v_vert, raw.unit());
    v_along.atan2(v_vert)
}

fn horizontal(v: Vec3, radial: Vec3) -> Vec3 {
    v - radial * dot(v, radial)
}

fn attitude_near_seed(base: &BaseController, kick_pitch: f64, kick_yaw: f64) -> bool {
    let (p, y) = base.pitch_yaw_angles();
    (p - kick_pitch).abs() <= SEED_NEAR && (y - kick_yaw).abs() <= SEED_NEAR
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
                kick_pitch,
                kick_yaw,
                kick_rate,
                min_alt,
                min_speed,
                ..
            } => {
                let v_g = base.ground_velocity();
                let speed = v_g.length();
                if base.altitude() < min_alt || speed < min_speed {
                    (0.0, 0.0)
                } else if !self.kick_done {
                    let mag = (kick_pitch * kick_pitch + kick_yaw * kick_yaw).sqrt();
                    self.turn_pitch =
                        (self.turn_pitch + kick_rate.max(0.0) * dt).clamp(0.0, mag.max(0.0));
                    let scale = if mag < 1e-12 {
                        0.0
                    } else {
                        self.turn_pitch / mag
                    };
                    let fp = flight_path_along_seed(base, v_g, kick_pitch, kick_yaw);
                    if attitude_near_seed(base, kick_pitch, kick_yaw) && fp > 0.0 {
                        self.kick_done = true;
                    }
                    (kick_pitch * scale, kick_yaw * scale)
                } else if v_g.length2() < 1e-12 {
                    (0.0, 0.0)
                } else {
                    prograde_target_angles(base, v_g)
                }
            }
        };
        self.pitch_cmd = pitch_t;
        self.yaw_cmd = yaw_t;

        if let Some(g) = base.caps().tvc_groups.first() {
            let id = g.id.clone();
            base.apply_tvc(&id, pitch_t, yaw_t, dt);
        }
    }
}

#[cfg(test)]
mod tests;
