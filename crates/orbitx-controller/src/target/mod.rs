//! `TargetController`——目标导向控制算法（模式 b）。
//!
//! 用户期望航天器朝哪飞、推力多大（即现 cli 控制能力）。内部根据目标 + 当前姿态/状态 +
//! 部分遥测，经 `BaseController` 控制本步怎么飞。**不 own caps**（caps 由 Runtime/WorkFlow
//! 拥有），只 own 算法状态（`TargetMode`、重力转向 kick 进度等）。caps 变化由拥有者换，
//! `TargetController` 经 `base` 观察新 caps，算法状态保留。
//!
//! `TargetMode` 枚举：
//! - `VerticalHold { throttle }` → `apply_tvc(0, 0)`，保竖直。
//! - `PitchTo { pitch, yaw, roll, throttle }` → `apply_tvc(pitch, yaw)`，朝指定姿态；
//!   roll 仅为目标记录（TVC 无滚转轴），RCS 开启时由 tick 内 RCS 参与段执行。
//! - `ProgradeHold { throttle }` → 由速度方向反解 pitch/yaw 目标（保当前滚转）。
//! - `RetrogradeHold { throttle }` → 反向（`-prograde`）。
//! - `NoseTo { azimuth, elevation, roll, throttle }` → 罗盘方位/俯仰目标机头：
//!   azimuth 0=北顺时针（随星球自转每 tick 重投影当地罗盘架）、elevation
//!   仰角（±90° 极点塌缩就近极竖直）。每 tick 经
//!   `nose_dir_from_az_el` → `nose_target_angles_body` 反解 (pitch, yaw) 喂
//!   TVC——目标读数按当前体架相位对齐，「机头 = d̂ ⇔ 误差 = 0」对任意
//!   滚转（固定罗盘架目标会与体架读数相减漏进滚转，已弃用于控制路径）；
//!   roll 仅为目标记录（无滚转执行机构），派生方向经 `nose_cmd` 上行切片。
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
use orbitx_vessel::{attitude as att, RotAxis};

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
    /// 朝指定俯仰/偏航/滚转角 [rad]（竖直=0）。roll 仅为目标记录（TVC 无滚转轴），
    /// RCS 参与段开启时经 RCS 执行。
    PitchTo {
        pitch: f64,
        yaw: f64,
        roll: f64,
        throttle: f64,
    },
    /// 沿速度方向（prograde）。
    ProgradeHold { throttle: f64 },
    /// 反速度方向（retrograde）。
    RetrogradeHold { throttle: f64 },
    /// 罗盘机头指向：方位角 0=北顺时针 [rad]、仰角 [rad]（±90° 极点塌缩
    /// 就近极竖直）。随星球自转每 tick 重投影；roll 仅为目标记录。
    NoseTo {
        azimuth: f64,
        elevation: f64,
        roll: f64,
        throttle: f64,
    },
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
            | TargetMode::NoseTo { throttle, .. }
            | TargetMode::GravityTurn { throttle, .. } => throttle,
        }
    }

    /// 替换油门，其它字段不变。
    pub fn with_throttle(self, throttle: f64) -> Self {
        match self {
            TargetMode::VerticalHold { .. } => TargetMode::VerticalHold { throttle },
            TargetMode::PitchTo { pitch, yaw, roll, .. } => TargetMode::PitchTo {
                pitch,
                yaw,
                roll,
                throttle,
            },
            TargetMode::ProgradeHold { .. } => TargetMode::ProgradeHold { throttle },
            TargetMode::RetrogradeHold { .. } => TargetMode::RetrogradeHold { throttle },
            TargetMode::NoseTo {
                azimuth,
                elevation,
                roll,
                ..
            } => TargetMode::NoseTo {
                azimuth,
                elevation,
                roll,
                throttle,
            },
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
    /// NoseTo 模式本步派生的机头方向（惯性系单位向量；切片上行用）。
    /// 非 NoseTo 模式每 tick 清 None——无状态残留。
    nose_cmd: Option<[f64; 3]>,
    /// RCS 姿态参与开关（客户端 R 键；Runtime 经 `set_rcs_enabled` 下发）。
    /// 开启且船有 RCS 组时，tick 内按姿态误差驱动 RCS 三轴辅助收敛。
    rcs_enabled: bool,
}

impl TargetController {
    pub fn new(mode: TargetMode) -> Self {
        Self {
            mode,
            turn_pitch: 0.0,
            kick_done: false,
            pitch_cmd: 0.0,
            yaw_cmd: 0.0,
            nose_cmd: None,
            rcs_enabled: false,
        }
    }

    pub fn mode(&self) -> TargetMode {
        self.mode
    }
    pub fn set_mode(&mut self, mode: TargetMode) {
        self.mode = mode;
    }
    pub fn rcs_enabled(&self) -> bool {
        self.rcs_enabled
    }
    pub fn set_rcs_enabled(&mut self, enabled: bool) {
        self.rcs_enabled = enabled;
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
    /// NoseTo 模式本步派生的机头方向（惯性系单位向量）；无目标/退化 None。
    pub fn nose_cmd(&self) -> Option<[f64; 3]> {
        self.nose_cmd
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

        // nose_cmd 每 tick 重算：非 NoseTo 模式恒 None（切模式无残留）。
        self.nose_cmd = None;
        let (pitch_t, yaw_t, roll_t) = match self.mode {
            TargetMode::VerticalHold { .. } => (0.0, 0.0, 0.0),
            TargetMode::PitchTo {
                pitch, yaw, roll, ..
            } => (pitch, yaw, roll),
            TargetMode::ProgradeHold { .. } => {
                let v = base.velocity();
                if v.length2() < 1e-12 {
                    (0.0, 0.0, 0.0)
                } else {
                    let (p, y) = prograde_target_angles(base, v);
                    (p, y, 0.0)
                }
            }
            TargetMode::RetrogradeHold { .. } => {
                let v = base.velocity();
                if v.length2() < 1e-12 {
                    (0.0, 0.0, 0.0)
                } else {
                    let (p, y) = prograde_target_angles(base, v);
                    (-p, -y, 0.0)
                }
            }
            TargetMode::NoseTo {
                azimuth,
                elevation,
                roll,
                ..
            } => {
                // 罗盘 (az, el) → 当地惯性系 d̂（radial 随星球自转每 tick
                // 重投影）→ 体架相位对齐反解 TVC (pitch, yaw)。目标读数随
                // 当前滚转共轭旋转，保证「机头 = d̂ ⇔ PD 误差 = 0」对任意
                // 滚转——固定罗盘架目标（旧 `nose_target_angles`）与体架
                // 读数相减会把滚转漏进稳态方位/仰角（黑匣子 rocket_8562
                // 实测 −70°/−53° 伪平衡点，2026-11-09 修复）。
                // 位置退化（原点）无当地架，保竖直。
                let pos = base.position();
                let r_mag = pos.length();
                if r_mag < 1e-3 {
                    (0.0, 0.0, roll)
                } else {
                    let radial = pos * (1.0 / r_mag);
                    let d = att::nose_dir_from_az_el(radial, azimuth, elevation);
                    let (bx, _by, bz) = base.body_axes();
                    let (p, y) = att::nose_target_angles_body(radial, d, bx, bz);
                    self.nose_cmd = Some([d.x, d.y, d.z]);
                    (p, y, roll)
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
                    (0.0, 0.0, 0.0)
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
                    (kick_pitch * scale, kick_yaw * scale, 0.0)
                } else if v_g.length2() < 1e-12 {
                    (0.0, 0.0, 0.0)
                } else {
                    let (p, y) = prograde_target_angles(base, v_g);
                    (p, y, 0.0)
                }
            }
        };
        self.pitch_cmd = pitch_t;
        self.yaw_cmd = yaw_t;

        // 堆叠语义 TVC（v6.2/WP1 修复）：控制当前级 + 侧挂的全部引擎
        // （`ThrustReadout::lit_thrusting_indices` 范围，`max_gimbal>0` 者执行）
        // ——旧 `tvc_groups.first()` 恒命中会话启动首组，分离后舵打在已分离
        // 船上（新主级无 TVC：yaw 不收敛/喷管无动画实锤）。
        base.apply_tvc_stack(pitch_t, yaw_t, dt);

        // RCS 姿态参与段：开关开启且船有 RCS 组时，按姿态误差驱动 RCS 三轴辅助
        // 收敛（bang-bang 满档 + 死区）。roll 仅 RCS 可执行（无差动矢量/气动翼）。
        // 误差 = 目标 − 当前角（attitude_errors 返回当前偏角非差值）；roll 差按
        // ±π wrap。组为空时无操作（无 RCS 设施 → 只有 TVC 生效）。
        if self.rcs_enabled {
            const RCS_DEADZONE: f64 = 0.5_f64.to_radians();
            let (p_cur, y_cur) = base.attitude_errors();
            let mut r_err = roll_t - base.roll_angle();
            if r_err > std::f64::consts::PI {
                r_err -= std::f64::consts::TAU;
            } else if r_err < -std::f64::consts::PI {
                r_err += std::f64::consts::TAU;
            }
            let group_ids: Vec<String> = base
                .caps()
                .rcs_groups
                .iter()
                .map(|g| g.id.clone())
                .collect();
            let p_err = pitch_t - p_cur;
            let y_err = yaw_t - y_cur;
            for id in group_ids.iter() {
                if p_err.abs() > RCS_DEADZONE {
                    base.set_rcs(id, RotAxis::Pitch, p_err.signum());
                } else {
                    base.set_rcs(id, RotAxis::Pitch, 0.0);
                }
                if y_err.abs() > RCS_DEADZONE {
                    base.set_rcs(id, RotAxis::Yaw, y_err.signum());
                } else {
                    base.set_rcs(id, RotAxis::Yaw, 0.0);
                }
                if r_err.abs() > RCS_DEADZONE {
                    base.set_rcs(id, RotAxis::Bank, r_err.signum());
                } else {
                    base.set_rcs(id, RotAxis::Bank, 0.0);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
