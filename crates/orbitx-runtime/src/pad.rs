//! 发射台钉死 / 松绑（P4.3 过渡；终态为发射台插件）。

use orbitx_math::{dot, Vec3};
use orbitx_vessel::{surface_inertial_velocity, Assembly, G0};

use crate::session::launch_attitude;

/// 首次点火后未释放台架的发射失败超时 [s]（仿真时间）。
pub const LAUNCH_FAIL_TIMEOUT_S: f64 = 10.0;

/// 会话台架状态。
#[derive(Debug, Clone)]
pub struct PadState {
    pub launched: bool,
    pub pad_pos: Vec3,
    /// 首次点火的仿真时刻 [s]；固定窗口起点，中途停推不重置（Reset 重建清除）。
    pub first_ignition_t: Option<f64>,
}

impl PadState {
    pub fn new(pad_pos: Vec3) -> Self {
        Self {
            launched: false,
            pad_pos,
            first_ignition_t: None,
        }
    }

    /// 回落台面后重新接管：未离地慢速回落台架时调用，下一 tick 起重新钉回，
    /// 点火窗口清零，可再次点火（不产生终局）。
    pub fn reengage(&mut self) {
        self.launched = false;
        self.first_ignition_t = None;
    }
}

/// 记录首次点火时刻；返回 true 表示点火满 10 s 仍未释放（发射失败）。
///
/// 只记第一次点火，窗口固定；中途停推不重置。已释放返回 false。
pub fn note_ignition_and_check_timeout(
    pad: &mut PadState,
    thrusting_level: f64,
    sim_t_s: f64,
) -> bool {
    if pad.launched {
        return false;
    }
    if pad.first_ignition_t.is_none() && thrusting_level > 1e-6 {
        pad.first_ignition_t = Some(sim_t_s);
    }
    match pad.first_ignition_t {
        Some(t0) => sim_t_s - t0 >= LAUNCH_FAIL_TIMEOUT_S,
        None => false,
    }
}

/// 未分离体上有效推力器的最大油门水平（0..1）。WorkFlow 写 `thruster.level` 后靠此松绑。
pub fn max_thrusting_level(asm: &Assembly) -> f64 {
    asm.vessels
        .iter()
        .filter(|v| !v.detached)
        .flat_map(|v| v.thrusters.iter())
        .filter(|t| t.max_thrust > 0.0)
        .map(|t| t.level)
        .fold(0.0_f64, f64::max)
}

/// 未分离体本步大气修正后的有效推力之和 [N]（步进 diagnostics 口径，与遥测一致）。
pub fn effective_thrust(asm: &Assembly) -> f64 {
    asm.vessels
        .iter()
        .filter(|v| !v.detached)
        .map(|v| v.diagnostics.thrust)
        .sum()
}

/// 步后：检测松绑；若仍在台架则钉死。
///
/// 释放门槛用**大气有效推力**（`effective_thrust_n`，diagnostics 口径）：
/// 有效推力 > 重量×1.05 才释放；真空额定达标但大气修正后不足时不释放、
/// 继续钉台，由点火 10 s 窗口判发射失败。
pub fn after_step(
    asm: &mut Assembly,
    pad: &mut PadState,
    thrusting_level: f64,
    effective_thrust_n: f64,
) {
    if !pad.launched && thrusting_level > 1e-6 {
        let weight = asm.total_mass() * G0;
        if effective_thrust_n > weight * 1.05 {
            pad.launched = true;
        } else {
            let pos = asm.state.pos;
            let vel = asm.state.vel;
            let r_mag = pos.length();
            if r_mag > 1e-3 {
                let v_radial = dot(vel, pos * (1.0 / r_mag));
                if v_radial > 0.5 {
                    pad.launched = true;
                }
            }
        }
    }

    if pad.launched {
        return;
    }

    let pad_pos = pad.pad_pos;
    let r_mag = pad_pos.length().max(1e-3);
    let radial_unit = pad_pos * (1.0 / r_mag);
    let pad_vel = if asm.sid_rot_period > 1e-9 {
        surface_inertial_velocity(pad_pos, asm.sid_rot_period)
    } else {
        Vec3::ZERO
    };
    let (lock_r, lock_q) = launch_attitude(radial_unit);
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.pos = pad_pos;
            v.state.vel = pad_vel;
            v.state.omega = Vec3::ZERO;
            v.state.q = lock_q;
            v.state.r = lock_r;
        }
    }
    let root = asm.root.min(asm.vessels.len().saturating_sub(1));
    asm.state = asm.vessels[root].state;
}

#[cfg(test)]
mod tests {
    use orbitx_math::{Quat, StateVectors, Vec3};
    use orbitx_vessel::{Assembly, StageSpec, ThrusterSpec};

    use super::{after_step, max_thrusting_level, PadState};

    fn booster_asm() -> Assembly {
        let stage = StageSpec {
            name: "core",
            dry_mass: 10_000.0,
            thrusters: vec![ThrusterSpec {
                pos: Vec3::new(0.0, -10.0, 0.0),
                dir: Vec3::new(0.0, 1.0, 0.0),
                thrust: 500_000.0,
                isp: 300.0,
                max_gimbal: 0.1,
                max_gimbal_rate: 1.0,
                gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
                ..Default::default()
            }],
            length: 20.0,
            radius: 1.5,
            ..Default::default()
        }
        .with_fuel(20_000.0);
        let pad = Vec3::new(0.0, 0.0, 6_371_000.0 + 44.0);
        Assembly::new(
            &[stage],
            StateVectors {
                pos: pad,
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: Default::default(),
                q: Quat::IDENTITY,
            },
        )
    }

    #[test]
    fn max_thrusting_level_reads_thruster_level() {
        let mut asm = booster_asm();
        assert_eq!(max_thrusting_level(&asm), 0.0);
        asm.vessels[0].thrusters[0].level = 0.8;
        assert!((max_thrusting_level(&asm) - 0.8).abs() < 1e-12);
    }

    #[test]
    fn after_step_releases_when_actual_level_high() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 1.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos);
        let level = max_thrusting_level(&asm);
        // 有效推力 500 kN > 重量（30 t）×1.05 → 松绑
        after_step(&mut asm, &mut pad, level, 500_000.0);
        assert!(pad.launched, "有效推力大于重量时应松绑");
    }

    #[test]
    fn after_step_holds_when_effective_thrust_insufficient() {
        // 真空额定 500 kN 达标，但大气修正后有效推力不足 → 不释放、继续钉台。
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 1.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos);
        let weight = asm.total_mass() * orbitx_vessel::G0;
        after_step(&mut asm, &mut pad, 1.0, weight * 1.02);
        assert!(!pad.launched, "有效推力未超重量 1.05 倍不应释放");
    }

    #[test]
    fn reengage_allows_second_release() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 1.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos);
        after_step(&mut asm, &mut pad, 1.0, 500_000.0);
        assert!(pad.launched);
        // 回落台面重新接管：清点火窗口，可再次释放
        pad.first_ignition_t = Some(42.0);
        pad.reengage();
        assert!(!pad.launched);
        assert_eq!(pad.first_ignition_t, None);
        after_step(&mut asm, &mut pad, 1.0, 500_000.0);
        assert!(pad.launched, "重新钉回后应可再次释放");
    }

    #[test]
    fn ignition_timeout_ten_seconds_fixed_window() {
        use super::note_ignition_and_check_timeout;
        let pad_pos = Vec3::new(0.0, 0.0, 6_371_000.0);
        let mut pad = PadState::new(pad_pos);
        // 点火前待命不判
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.0, 100.0));
        // 首次点火 t=100
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.5, 100.0));
        // 中途停推不重置窗口
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.0, 105.0));
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.5, 109.9));
        assert!(note_ignition_and_check_timeout(&mut pad, 0.5, 110.0));
        // 已释放不判
        pad.launched = true;
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.5, 200.0));
    }

    #[test]
    fn after_step_stays_pinned_when_level_zero() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 0.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos);
        after_step(&mut asm, &mut pad, 0.0, 0.0);
        assert!(!pad.launched);
        assert_eq!(asm.state.vel, Vec3::ZERO);
    }
}
