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
