//! 发射台钉死 / 松绑（P4.3 过渡；终态为发射台插件）。

use orbitx_math::{dot, Vec3};
use orbitx_vessel::{surface_inertial_velocity, Assembly, G0};

use crate::session::launch_attitude;

/// 会话台架状态。
#[derive(Debug, Clone)]
pub struct PadState {
    pub launched: bool,
    pub pad_pos: Vec3,
}

impl PadState {
    pub fn new(pad_pos: Vec3) -> Self {
        Self {
            launched: false,
            pad_pos,
        }
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

/// 步后：检测松绑；若仍在台架则钉死。
pub fn after_step(asm: &mut Assembly, pad: &mut PadState, thrusting_level: f64) {
    if !pad.launched && thrusting_level > 1e-6 {
        let thrust: f64 = asm
            .vessels
            .iter()
            .filter(|v| !v.detached)
            .flat_map(|v| v.thrusters.iter())
            .filter(|t| t.max_thrust > 0.0)
            .map(|t| t.max_thrust * t.level)
            .sum();
        let weight = asm.total_mass() * G0;
        if thrust > weight * 1.05 {
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
        after_step(&mut asm, &mut pad, level);
        assert!(pad.launched, "推力大于重量时应松绑");
    }

    #[test]
    fn after_step_stays_pinned_when_level_zero() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 0.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos);
        after_step(&mut asm, &mut pad, 0.0);
        assert!(!pad.launched);
        assert_eq!(asm.state.vel, Vec3::ZERO);
    }
}
