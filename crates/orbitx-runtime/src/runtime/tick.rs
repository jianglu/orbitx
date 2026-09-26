//! 步进编排：Control → StepEnv → Assembly::step → 台架/坠毁 → 富 Slice。

use std::collections::HashSet;

use orbitx_math::{dot, Elements, GGRAV, Vec3};
use orbitx_vessel::{
    pitch_yaw_angles, roll_angle, tip_angle, surface_inertial_velocity, Assembly, G0,
};

use crate::crash;
use crate::ephem::{earth_centered_grav_env, earth_mass_kg};
use crate::pad;
use crate::runtime::clock::Clock;
use crate::session::SimBundle;
use crate::slice::{
    AttitudeTelem, EnvTelem, FocusTelem, LaunchpadTelem, OrbitTelem, Slice, StageTelem,
};
use orbitx_vessel::StepEnv;

pub enum TickOutcome {
    Stepped { slice: Slice },
    Skipped,
}

/// 完成一步固定 `sim_dt`（物理秒）。
pub fn tick(clock: &mut Clock, sim: &mut SimBundle) -> TickOutcome {
    if clock.paused() {
        return TickOutcome::Skipped;
    }

    let dt = clock.sim_dt_secs();
    let thr_cmd = sim.control.throttle_cmd();

    // 1 Control @ T0
    sim.control.tick(&mut sim.asm, dt);

    // 2 环境 @ T0
    sim.psys.update_positions();
    let (grav, primary) = earth_centered_grav_env(&sim.psys);
    if let Some(b) = grav.get(primary) {
        sim.earth_radius = b.size;
        sim.asm.planet_radius = b.size;
    }

    // 3 航空器
    sim.asm.step(dt, StepEnv::new(&grav, primary));

    // 4 台架
    pad::after_step(&mut sim.asm, &mut sim.pad, thr_cmd);

    // 5 坠毁
    if let Some((name, impact_speed)) = crash::apply_crash_checks(&mut sim.asm, sim.pad.launched)
    {
        if sim.crash_msg.is_empty() {
            sim.crash_msg = format!("{name} 撞击地面，速度 {impact_speed:.0} m/s");
            clock.set_paused(true);
        }
    }

    // 6 环境 → T1
    sim.psys.advance(dt / 86_400.0);
    clock.advance_fixed_step();

    let slice = build_slice(clock, sim);
    TickOutcome::Stepped { slice }
}

fn build_slice(clock: &Clock, sim: &SimBundle) -> Slice {
    let asm = &sim.asm;
    let pos = asm.state.pos;
    let vel = asm.state.vel;
    let r_mag = pos.length().max(1e-3);
    let radial = pos * (1.0 / r_mag);
    let v_vert_i = dot(vel, radial);

    let primary: HashSet<usize> = asm.components.iter().map(|c| c.vessel_index).collect();

    let stages: Vec<StageTelem> = sim
        .stage_display_order
        .iter()
        .copied()
        .filter(|&i| i < asm.vessels.len())
        .map(|i| {
            let v = &asm.vessels[i];
            let init_f = sim.initial_fuel.get(i).copied().unwrap_or(v.fuel_mass);
            let fp = if init_f > 1e-9 {
                (v.fuel_mass / init_f * 100.0).clamp(0.0, 100.0)
            } else {
                0.0
            };
            let firing = v.thrusters.iter().any(|t| t.max_thrust > 0.0 && t.level > 1e-3);
            let strap_on = !primary.contains(&i)
                || (primary.contains(&i)
                    && i != asm.active
                    && v.thrusters.iter().any(|t| t.max_thrust > 0.0));
            StageTelem {
                name: v.name.clone(),
                fuel_pct: fp,
                active: i == asm.active && primary.contains(&i),
                detached: v.detached,
                crashed: v.crashed,
                firing,
                empty_fuel: v.fuel_mass < 1.0,
                strap_on: strap_on && primary.contains(&i) && i != asm.active,
                fuel: v.fuel_mass,
                vessel_index: i as u32,
            }
        })
        .collect();

    let active_vi = asm.active.min(asm.vessels.len().saturating_sub(1));
    let focus = build_focus_telem(asm, active_vi, true, &sim.initial_fuel, sim.earth_radius);

    let mut detached = Vec::new();
    for i in 0..asm.vessels.len() {
        if asm.vessels[i].detached || !primary.contains(&i) {
            detached.push(build_focus_telem(
                asm,
                i,
                false,
                &sim.initial_fuel,
                sim.earth_radius,
            ));
        }
    }

    let (pitch_t, yaw_t, roll_t) = sim.control.attitude_targets();

    let mu = GGRAV * earth_mass_kg(&sim.psys);
    let el = Elements::calculate(pos, vel, mu, 0.0);
    let energy = vel.length2() * 0.5 - mu / r_mag;
    let escaping = energy >= 0.0;
    let pe_r = if el.e < 1.0 {
        el.a * (1.0 - el.e)
    } else {
        0.0
    };
    let ap_r = if el.e < 1.0 {
        el.a * (1.0 + el.e)
    } else {
        f64::INFINITY
    };
    let suborbital = !escaping && pe_r < sim.earth_radius;
    let period = if el.e < 1.0 && el.a > 0.0 {
        std::f64::consts::TAU * (el.a.powi(3) / mu).sqrt()
    } else {
        0.0
    };

    let speed_inertial = vel.length();
    let v_circular = (mu / r_mag).sqrt();
    let energy_margin = mu / r_mag * 0.01;
    let v_horiz_inertial = (vel - radial * v_vert_i).length();
    let hud_mode: u32 = if v_horiz_inertial > v_circular * 0.5 && energy < -energy_margin {
        1
    } else if energy > energy_margin && speed_inertial > 100.0 {
        2
    } else {
        0
    };

    let lat = (pos.y / r_mag).asin().to_degrees();
    let lng = pos.z.atan2(pos.x).to_degrees();
    let alt = r_mag - sim.earth_radius;

    Slice {
        sim_t: clock.sim_t_ms(),
        step_index: clock.step_index(),
        paused: clock.paused(),
        warp: clock.warp(),
        rocket_name: sim.rocket_name.clone(),
        active_name: asm.active_name().to_string(),
        launched: sim.pad.launched,
        crash_msg: sim.crash_msg.clone(),
        pitch_target: pitch_t,
        yaw_target: yaw_t,
        roll_target: roll_t,
        throttle_cmd: sim.control.throttle_cmd(),
        gravity_turn: sim.control.gravity_turn_enabled(),
        focus: focus.clone(),
        detached,
        launchpad: LaunchpadTelem {
            lat_deg: lat,
            lng_deg: lng,
            alt_m: alt,
            launched: sim.pad.launched,
        },
        orbit: OrbitTelem {
            escaping,
            suborbital,
            ap_alt: if ap_r.is_finite() {
                ap_r - sim.earth_radius
            } else {
                0.0
            },
            pe_alt: pe_r - sim.earth_radius,
            period_s: period,
            energy_mj_kg: energy / 1e6,
            hud_mode,
        },
        stages,
        attitude: AttitudeTelem {
            pitch: focus.pitch,
            yaw: focus.yaw,
            roll: focus.roll,
            pitch_target: pitch_t,
            yaw_target: yaw_t,
            roll_target: roll_t,
            omega: focus.omega,
            throttle: focus.throttle,
            gimbal_pitch: focus.gimbal_pitch,
            gimbal_yaw: focus.gimbal_yaw,
        },
    }
}

fn build_focus_telem(
    asm: &Assembly,
    vessel_idx: usize,
    is_primary: bool,
    initial_fuel: &[f64],
    earth_radius: f64,
) -> FocusTelem {
    let vi = vessel_idx.min(asm.vessels.len().saturating_sub(1));
    let v = &asm.vessels[vi];

    let (pos, vel_inertial, mass, fuel, fuel_pct, thrust, display_name) = if is_primary {
        let mass = asm.total_mass();
        let fuel = asm.total_fuel();
        let init: f64 = initial_fuel.iter().sum();
        let fuel_pct = if init > 1e-9 {
            (fuel / init * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        let thrust: f64 = asm
            .vessels
            .iter()
            .filter(|vv| !vv.detached)
            .map(|vv| vv.diagnostics.thrust)
            .sum();
        (
            asm.state.pos,
            asm.state.vel,
            mass,
            fuel,
            fuel_pct,
            thrust,
            asm.active_name().to_string(),
        )
    } else {
        let init = initial_fuel.get(vi).copied().unwrap_or(v.fuel_mass).max(v.fuel_mass);
        let fuel_pct = if init > 1e-9 {
            (v.fuel_mass / init * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        (
            v.state.pos,
            v.state.vel,
            v.mass(),
            v.fuel_mass,
            fuel_pct,
            v.diagnostics.thrust,
            v.name.clone(),
        )
    };

    let surf = if asm.sid_rot_period > 1e-9 {
        surface_inertial_velocity(pos, asm.sid_rot_period)
    } else {
        Vec3::ZERO
    };
    let vel_ground = vel_inertial - surf;
    let r_mag = pos.length().max(1e-3);
    let radial = pos * (1.0 / r_mag);
    let mut v_vert = dot(vel_ground, radial);
    if v_vert.abs() < 0.5 {
        v_vert = 0.0;
    }
    let v_horiz = (vel_ground - radial * v_vert).length();
    let alt = r_mag - earth_radius;
    let twr = if mass > 1e-9 {
        thrust / (mass * G0)
    } else {
        0.0
    };

    let d = &v.diagnostics;
    let (pitch, yaw, roll, tip) = if is_primary {
        let (p, y) = pitch_yaw_angles(&asm.state);
        (p, y, roll_angle(&asm.state), tip_angle(&asm.state))
    } else {
        (d.pitch, d.yaw, d.roll, d.tip)
    };

    let (gimbal_p, gimbal_y, thr_level) = {
        let t = v.thrusters.iter().find(|t| t.max_thrust > 0.0);
        match t {
            Some(t) => (t.gimbal_pitch, t.gimbal_yaw, t.level),
            None => (0.0, 0.0, 0.0),
        }
    };
    let omega = v.state.omega;

    FocusTelem {
        is_primary,
        display_name,
        vessel_index: vi as u32,
        pos: [pos.x, pos.y, pos.z],
        vel: [vel_ground.x, vel_ground.y, vel_ground.z],
        altitude: alt,
        speed: vel_ground.length(),
        v_vert,
        v_horiz,
        mass,
        fuel,
        fuel_pct,
        thrust,
        twr,
        pitch,
        yaw,
        roll,
        tip,
        env: EnvTelem {
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
        },
        omega: [omega.x, omega.y, omega.z],
        gimbal_pitch: gimbal_p,
        gimbal_yaw: gimbal_y,
        throttle: thr_level,
    }
}
