//! TVC（推力矢量控制）双轴 PD。
//!
//! 增益 `TVC_KP` / `TVC_KD` 与 CLI 竖直保持 / 重力转向共用。按 tvc group id 对组内主推
//! 推力机调 `Thruster::slew_gimbal`（step 不代做，须在 step 前由控制器写入）。
//!
//! PD 误差取本 body 参考 vessel 的姿态（主组合体→active；detached→该 vessel），
//! `gimbal = -(Kp·err + Kd·ω)`：P/D 同号反对 tip 与 tip-rate。

use crate::capability::ControlCapability;
use crate::throttle::body_vessel_index;
use orbitx_vessel::{attitude as att, Assembly};

/// TVC PD 比例增益。
pub const TVC_KP: f64 = 1.0;
/// TVC PD 微分增益。
pub const TVC_KD: f64 = 2.0;

/// 双轴 TVC PD：按 `group_id` 选 tvc 组，对其推进器调 `slew_gimbal`。
///
/// `pitch_target` / `yaw_target` 为期望有符号 tip 角 [rad]（竖直=0）。组 id 未命中则无操作。
pub fn apply_tvc(
    asm: &mut Assembly,
    caps: &ControlCapability,
    group_id: &str,
    pitch_target: f64,
    yaw_target: f64,
    dt: f64,
) {
    let Some(group) = caps.tvc_groups.iter().find(|g| g.id == group_id) else {
        return;
    };
    let vi = body_vessel_index(asm, caps);
    let Some(v) = asm.vessels.get(vi) else {
        return;
    };
    let (p, y) = att::pitch_yaw_angles(&v.state);
    let err_p = p - pitch_target;
    let err_y = y - yaw_target;
    let w = v.state.omega;
    let cmd_p = TVC_KP * err_p + TVC_KD * w.x;
    let cmd_y = TVC_KP * err_y + TVC_KD * w.z;

    for &(tvvi, ti) in &group.thrusters {
        let Some(v) = asm.vessels.get_mut(tvvi) else { continue };
        let Some(t) = v.thrusters.get_mut(ti) else { continue };
        if t.max_gimbal > 0.0 {
            t.slew_gimbal(-cmd_p, -cmd_y, dt);
        }
    }
}

#[cfg(test)]
mod tests;
