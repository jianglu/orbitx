//! 观察焦点的只读遥测快照（不改物理状态；受力/姿态只读 vessel.diagnostics）。

use crate::control::lit_thrusting_indices;
use crate::focus::{ViewFocus, ViewSubject};
use orbitx_dynamics::GravBody;
use orbitx_math::{dot, Vec3};
use orbitx_vessel::{surface_inertial_velocity, Assembly, StageSpec, G0};

/// 环境 / 气动诊断（来自焦点船 `Vessel::diagnostics` 缓存）。
#[derive(Debug, Clone)]
pub struct EnvTelem {
    pub a_grav: f64,
    pub g_multiple: f64,
    pub mach: f64,
    pub density: f64,
    pub dynamic_pressure: f64,
    pub pressure: f64,
    pub thrust_atm_scale: f64,
    pub isp_eff: f64,
    pub drag_force: f64,
    pub cd_eff: f64,
    pub temperature: f64,
    pub sound_speed: f64,
    pub load_factor: f64,
}

/// 供 TUI 绘制的焦点遥测。
#[derive(Debug, Clone)]
pub struct TelemSnapshot {
    pub is_primary: bool,
    pub display_name: String,
    pub vessel_index: usize,
    pub pos: Vec3,
    pub vel_inertial: Vec3,
    pub vel_ground: Vec3,
    pub altitude: f64,
    pub speed: f64,
    pub v_vert: f64,
    pub v_horiz: f64,
    pub mass: f64,
    pub fuel: f64,
    /// 剩余燃料相对该主体初始燃料 [%]（0..100）。
    pub fuel_pct: f64,
    pub thrust: f64,
    pub twr: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub tip: f64,
    pub env: EnvTelem,
}

/// 从 `Assembly` + 观察焦点构建 UI 快照（只读步进写回的 state / diagnostics）。
pub fn snapshot(
    asm: &Assembly,
    focus: ViewFocus,
    initial_stages: &[StageSpec],
    _grav_bodies: &[GravBody],
) -> TelemSnapshot {
    let is_primary = focus.is_primary();
    let display_name = focus.display_name(asm);
    let vessel_index = focus.vessel_index(asm);

    let initial_fuel_total: f64 = initial_stages.iter().map(|s| s.fuel_mass).sum();

    let (pos, vel_inertial, mass, fuel, fuel_pct) = match focus.subject {
        ViewSubject::Primary => {
            let pos = asm.state.pos;
            let vel = asm.state.vel;
            let mass = asm.total_mass();
            let fuel = asm.total_fuel();
            let fuel_pct = if initial_fuel_total > 0.0 {
                (fuel / initial_fuel_total * 100.0).clamp(0.0, 100.0)
            } else {
                0.0
            };
            (pos, vel, mass, fuel, fuel_pct)
        }
        ViewSubject::Detached(i) => {
            let i = i.min(asm.vessels.len().saturating_sub(1));
            let v = &asm.vessels[i];
            let pos = v.state.pos;
            let vel = v.state.vel;
            let mass = v.mass();
            let fuel = v.fuel_mass;
            let init_fuel = initial_stages
                .get(i)
                .map(|s| s.fuel_mass)
                .unwrap_or(0.0)
                .max(fuel);
            let fuel_pct = if init_fuel > 0.0 {
                (fuel / init_fuel * 100.0).clamp(0.0, 100.0)
            } else {
                0.0
            };
            (pos, vel, mass, fuel, fuel_pct)
        }
    };

    let thrust = if is_primary {
        lit_thrusting_indices(asm)
            .into_iter()
            .map(|i| asm.vessels[i].diagnostics.thrust)
            .sum()
    } else {
        let i = vessel_index.min(asm.vessels.len().saturating_sub(1));
        asm.vessels[i].diagnostics.thrust
    };

    let ground_wind = if asm.sid_rot_period > 1e-9 {
        surface_inertial_velocity(pos, asm.sid_rot_period)
    } else {
        Vec3::ZERO
    };
    let vel_ground = vel_inertial - ground_wind;
    let speed = vel_ground.length();
    let r_len = pos.length().max(1e-3);
    let r_unit = pos * (1.0 / r_len);
    let mut v_vert = dot(vel_ground, r_unit);
    if v_vert.abs() < 0.5 {
        v_vert = 0.0;
    }
    let v_horiz = (vel_ground - r_unit * v_vert).length();
    let altitude = r_len - asm.planet_radius;
    let twr = if mass > 0.0 {
        thrust / (mass * G0)
    } else {
        0.0
    };

    let focus_vi = if is_primary {
        asm.active.min(asm.vessels.len().saturating_sub(1))
    } else {
        vessel_index.min(asm.vessels.len().saturating_sub(1))
    };
    let d = &asm.vessels[focus_vi].diagnostics;
    let env = EnvTelem {
        a_grav: d.a_grav,
        g_multiple: d.g_multiple,
        mach: d.mach,
        density: d.density,
        dynamic_pressure: d.dynamic_pressure,
        pressure: d.pressure,
        thrust_atm_scale: d.thrust_atm_scale,
        isp_eff: d.isp_eff,
        drag_force: d.drag_force,
        cd_eff: d.cd_eff,
        temperature: d.temperature,
        sound_speed: d.sound_speed,
        load_factor: d.load_factor,
    };

    TelemSnapshot {
        is_primary,
        display_name,
        vessel_index,
        pos,
        vel_inertial,
        vel_ground,
        altitude,
        speed,
        v_vert,
        v_horiz,
        mass,
        fuel,
        fuel_pct,
        thrust,
        twr,
        pitch: d.pitch,
        yaw: d.yaw,
        roll: d.roll,
        tip: d.tip,
        env,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::{Matrix3, Quat, StateVectors};
    use orbitx_vessel::{presets, DragElement, ExponentialAtmosphere};

    fn earth() -> GravBody {
        GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![],
            rotation: None,
            pines: None,
        }
    }

    #[test]
    fn primary_snapshot_reads_active_diagnostics() {
        let stages = presets::falcon9();
        let init = StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 20_000.0),
            vel: Vec3::new(800.0, 0.0, 0.0),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
            ..Default::default()
        };
        let mut asm = Assembly::new(&stages, init);
        asm.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
        asm.planet_radius = 6_371_000.0;
        asm.step(0.05, &[earth()]);
        let snap = snapshot(&asm, ViewFocus::primary(), &stages, &[earth()]);
        assert!(snap.is_primary);
        assert!(snap.env.mach.is_finite());
        assert!((snap.env.mach - asm.diagnostics().mach).abs() < 1e-12);
        assert!((snap.pitch - asm.vessels[asm.active].diagnostics.pitch).abs() < 1e-12);
    }

    #[test]
    fn empty_fuel_after_step_reports_zero_thrust() {
        let stages = presets::falcon9();
        let init = StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 20_000.0),
            vel: Vec3::new(800.0, 0.0, 0.0),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
            ..Default::default()
        };
        let mut asm = Assembly::new(&stages, init);
        asm.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
        asm.planet_radius = 6_371_000.0;
        asm.set_throttle(1.0);
        asm.vessels[asm.active].fuel_mass = 0.0;
        for t in &mut asm.vessels[asm.active].tanks {
            t.mass = 0.0;
        }
        asm.step(0.05, &[earth()]);
        assert!(asm.vessels[asm.active].diagnostics.thrust.abs() < 1e-9);
        let snap = snapshot(&asm, ViewFocus::primary(), &stages, &[earth()]);
        assert!(snap.thrust.abs() < 1e-9);
        assert!(snap.twr.abs() < 1e-9);
        assert!(snap.env.isp_eff.abs() < 1e-9);
    }

    #[test]
    fn detached_snapshot_reads_vessel_cache_not_recompute() {
        let stages = presets::falcon9();
        let init = StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 20_000.0),
            vel: Vec3::new(800.0, 0.0, 0.0),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
            ..Default::default()
        };
        let mut asm = Assembly::new(&stages, init);
        asm.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0]
            .dragels
            .push(DragElement::constant(Vec3::ZERO, 0.5, 8.0));
        asm.separate_stage();
        asm.step(0.05, &[earth()]);

        let focus = ViewFocus {
            subject: ViewSubject::Detached(0),
        };
        let cached_mach = asm.vessels[0].diagnostics.mach;
        let cached_thrust = asm.vessels[0].diagnostics.thrust;
        let snap = snapshot(&asm, focus, &stages, &[earth()]);
        assert!(!snap.is_primary);
        assert!((snap.env.mach - cached_mach).abs() < 1e-12);
        assert!((snap.thrust - cached_thrust).abs() < 1e-12);
        assert!(snap.env.density > 0.0);
        assert!(snap.env.a_grav > 5.0);
    }
}
