//! TVC（推力矢量控制）：期望角加速度 → 所需力矩 → gimbal 角。
//!
//! 控制律：`α_des = K_ALPHA·err + K_OMEGA·ω`（err = 当前 − 目标角），所需力矩
//! `τ = I·α_des`，折算 gimbal 角 `cmd = τ/(T·L)`。T·L 为组内推进器「当前推力 ×
//! 轴向力臂」合计；力臂取推力轴（体 +Y）投影——推力偏转产生侧向力，力矩 ≈
//! 轴向距离 × 侧向力。权限随质量/燃烧实时归一：分离后惯量骤变不再等效放大
//! 增益（2026-10-27 侧挂分离后失控的根因之一）。
//!
//! 符号链继承旧直接律 `Kp·err + Kd·ω`（正指令压正误差，反打实验证实方向正确）；
//! T≈0（节流 0）、权限≈0 或 `max_gimbal=0` 时跳过。PD 无积分项，无抗 windup 需求。

use crate::capability::{BodyRef, ControlCapability};
use crate::throttle::body_vessel_index;
use orbitx_math::mul;
use orbitx_vessel::{attitude as att, Assembly};

/// 比例通道：单位姿态误差 [rad] 的期望角加速度 [rad/s²]。
/// 实测选值（sc_cache 侧挂分离回归）：1.0/2.0 峰值 |ω|=0.004 且衰减平滑；
/// 10/10 响应更硬但 0.9° 误差即满舵，噪声/风扰下易进入饱和+限速震荡带。
pub const TVC_K_ALPHA: f64 = 1.0;
/// 阻尼通道：单位角速度 [rad/s] 的期望角加速度 [rad/s²]。
pub const TVC_K_OMEGA: f64 = 2.0;

/// 双轴 TVC：按 `group_id` 选 tvc 组，对其推进器调 `slew_gimbal`。
///
/// `pitch_target` / `yaw_target` 为期望有符号 tip 角 [rad]（竖直=0）。组 id 未命中
/// 或权限不足则无操作。
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

    // 读阶段：姿态误差、角速度、惯量（按 body 参考）、组权限合计。
    let (err_p, err_y, w_x, w_z) = {
        let Some(v) = asm.vessels.get(vi) else {
            return;
        };
        let (p, y) = att::pitch_yaw_angles(&v.state);
        (
            p - pitch_target,
            y - yaw_target,
            v.state.omega.x,
            v.state.omega.z,
        )
    };
    let (i_pitch, i_yaw) = match caps.body {
        BodyRef::Primary => {
            let m = asm.total_mass();
            let pmi = asm.composite_pmi();
            (pmi.x * m, pmi.z * m)
        }
        BodyRef::Detached(_) => {
            let Some(v) = asm.vessels.get(vi) else {
                return;
            };
            let m = v.mass();
            (v.pmi.x * m, v.pmi.z * m)
        }
    };
    // 组权限：Σ 当前推力 × 轴向力臂（簇体坐标原点=质心，故 cg=0）。
    // 真空近似（气压 0）：权限估计偏保守 → 指令偏大 → 提前饱和但不进入失稳方向。
    let mut authority = 0.0_f64;
    for &(tvvi, ti) in &group.thrusters {
        let Some(v) = asm.vessels.get(tvvi) else {
            continue;
        };
        let Some(c) = asm.components.iter().find(|c| c.vessel_index == tvvi) else {
            continue;
        };
        let Some(t) = v.thrusters.get(ti) else {
            continue;
        };
        if t.max_gimbal <= 0.0 {
            continue;
        }
        let thrust = t.current_thrust(0.0);
        let r = c.rpos + mul(c.rrot, t.pos);
        authority += thrust * r.y.abs();
    }
    if authority <= 1e-6 {
        return;
    }

    let cmd_p = (TVC_K_ALPHA * err_p + TVC_K_OMEGA * w_x) * i_pitch / authority;
    let cmd_y = (TVC_K_ALPHA * err_y + TVC_K_OMEGA * w_z) * i_yaw / authority;
    if std::env::var("SC_TVC_DEBUG").is_ok() {
        eprintln!(
            "[TVC] err_p={err_p:.4} err_y={err_y:.4} w=({w_x:.4},{w_z:.4}) I_x={i_pitch:.3e} auth={authority:.3e} cmd_p={cmd_p:.4} cmd_y={cmd_y:.4}"
        );
    }

    // 写阶段：限速 slew 到组内推进器。
    for &(tvvi, ti) in &group.thrusters {
        let Some(v) = asm.vessels.get_mut(tvvi) else {
            continue;
        };
        let Some(t) = v.thrusters.get_mut(ti) else {
            continue;
        };
        if t.max_gimbal > 0.0 {
            t.slew_gimbal(cmd_p, cmd_y, dt);
        }
    }
}

/// 双轴 TVC（**堆叠语义**，TargetController 主路径）：控制当前级 + 侧挂的
/// **全部引擎**——范围 = `ThrustReadout::lit_thrusting_indices()`（active ∪
/// 侧挂叶，全部主推在册，固定与矢量皆在册；用户裁定 2026-10-29「TVC 控制
/// 当前级和侧挂的所有引擎，不只是矢量引擎」）；其中 `max_gimbal>0` 者执行
/// slew（固定引擎在册但不偏转）。err/惯量沿用组合体口径（分离后自动 = 新
/// 主级堆叠）。权责：vessel 出结构读数（`ThrustReadout`）、本模块做控制决策。
///
/// 与 [`apply_tvc`]（workflow 显式组指定）并存；堆叠内无 `max_gimbal>0`
/// 引擎或权限不足则无操作。
pub fn apply_tvc_stack(
    asm: &mut Assembly,
    caps: &ControlCapability,
    pitch_target: f64,
    yaw_target: f64,
    dt: f64,
) {
    use orbitx_vessel::ThrustReadout;

    let vi = body_vessel_index(asm, caps);

    // 读阶段：姿态误差、角速度、惯量（按 body 参考）。
    let (err_p, err_y, w_x, w_z) = {
        let Some(v) = asm.vessels.get(vi) else {
            return;
        };
        let (p, y) = att::pitch_yaw_angles(&v.state);
        (
            p - pitch_target,
            y - yaw_target,
            v.state.omega.x,
            v.state.omega.z,
        )
    };
    let (i_pitch, i_yaw) = match caps.body {
        BodyRef::Primary => {
            let m = asm.total_mass();
            let pmi = asm.composite_pmi();
            (pmi.x * m, pmi.z * m)
        }
        BodyRef::Detached(_) => {
            let Some(v) = asm.vessels.get(vi) else {
                return;
            };
            let m = v.mass();
            (v.pmi.x * m, v.pmi.z * m)
        }
    };

    // 范围：lit 集（active ∪ 侧挂叶，主推在册）的矢量推进器；authority =
    // Σ 推力 × 轴向力臂（真空近似，与组版同口径）。
    let mut targets: Vec<(usize, usize)> = Vec::new();
    let mut authority = 0.0_f64;
    for vi_t in asm.lit_thrusting_indices() {
        let Some(c) = asm.components.iter().find(|c| c.vessel_index == vi_t) else {
            continue;
        };
        let v = &asm.vessels[vi_t];
        let n_main = v.n_main_thrusters.min(v.thrusters.len());
        for ti in 0..n_main {
            let t = &v.thrusters[ti];
            if t.max_gimbal <= 0.0 {
                continue;
            }
            let thrust = t.current_thrust(0.0);
            let r = c.rpos + mul(c.rrot, t.pos);
            authority += thrust * r.y.abs();
            targets.push((vi_t, ti));
        }
    }
    if authority <= 1e-6 || targets.is_empty() {
        return;
    }

    let cmd_p = (TVC_K_ALPHA * err_p + TVC_K_OMEGA * w_x) * i_pitch / authority;
    let cmd_y = (TVC_K_ALPHA * err_y + TVC_K_OMEGA * w_z) * i_yaw / authority;
    if std::env::var("SC_TVC_DEBUG").is_ok() {
        eprintln!(
            "[TVC-stack] err_p={err_p:.4} err_y={err_y:.4} w=({w_x:.4},{w_z:.4}) I_x={i_pitch:.3e} auth={authority:.3e} cmd_p={cmd_p:.4} cmd_y={cmd_y:.4} n={}",
            targets.len()
        );
    }

    // 写阶段：限速 slew 到 lit 集矢量推进器（各台自身 max_gimbal/rate 钳制）。
    for &(tvvi, ti) in &targets {
        let Some(v) = asm.vessels.get_mut(tvvi) else {
            continue;
        };
        let Some(t) = v.thrusters.get_mut(ti) else {
            continue;
        };
        t.slew_gimbal(cmd_p, cmd_y, dt);
    }
}

#[cfg(test)]
mod tests;
