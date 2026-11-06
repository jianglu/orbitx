//! 步进编排：Control → StepEnv → Assembly::step → 台架/坠毁 → 富 Slice。

use std::collections::HashSet;

use orbitx_math::mat3::Matrix3;
use orbitx_math::{dot, Elements, Vec3, GGRAV};
use orbitx_protocol::mem::mat3_from_orbitx;
use orbitx_vessel::{
    atmosphere_from_config, geocentric_lat_lon_deg_orient, surface_inertial_velocity, Assembly, G0,
};

use crate::crash;
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

/// 完成一步固定 `sim_dt`（物理秒）。`stepping` = 发射门禁状态（切片回显；
/// false = 预发射 hold，客户端据此停发 Step）。
pub fn tick(clock: &mut Clock, sim: &mut SimBundle, stepping: bool) -> TickOutcome {
    if clock.paused() {
        return TickOutcome::Skipped;
    }

    let dt = clock.sim_dt_secs();

    // 1 Control @ T0（读上一步切片；第一步尚无切片）
    sim.control.tick(&mut sim.asm, &sim.last_slice, dt);

    // 2 环境 @ T0
    sim.world.psys.update();
    let surface = sim.world.psys.primary_surface();
    sim.asm.atmosphere = atmosphere_from_config(surface.atmosphere.as_ref());
    sim.asm.sid_rot_period = surface.sid_rot_period;
    sim.asm.planet_orientation = surface.orientation;
    sim.asm.planet_radius = surface.radius;
    sim.earth_radius = surface.radius;

    // P5.1：SurfaceSession 分频预取 + 三态注入；无会话时保持光滑球（与 P4.3 一致）。
    // 体固经纬经 Rᵀ 采样，与共转 ω×r 同源（PlanetOrientation）。
    let sim_t_s = clock.sim_t_ms() as f64 / 1000.0;
    let pos0 = sim.asm.state.pos;
    let vel0 = sim.asm.state.vel;
    if let Some(ref mut surf) = sim.surface {
        surf.on_tick(pos0, vel0, surface.orientation.rot);
        sim.asm.ground = surf.inject();
    } else {
        sim.asm.ground = orbitx_vessel::GroundContact::Sphere {
            radius_m: surface.radius,
        };
    }

    let grav = sim.world.psys.grav_bodies();
    let primary = sim.world.psys.primary_grav_index();

    // 3 航天器（触点力读 asm.ground）
    sim.asm.step(dt, StepEnv::new(&grav, primary));

    // 4 台架（用装配实际油门；WorkFlow 下 throttle_cmd 恒 0）
    // 释放门槛用大气有效推力（diagnostics 口径），非真空额定。
    let thr_level = pad::max_thrusting_level(&sim.asm);
    let eff_thrust = pad::effective_thrust(&sim.asm);
    pad::after_step(&mut sim.asm, &mut sim.pad, thr_level, eff_thrust);

    // 5 触地终局（坠毁 runtime 暂停；着陆成功/发射失败只上报，CLI 负责暂停）
    // 已落定：每拍体固钉姿共转（对齐 Orbiter LANDED / land_rot），不再走接触终局。
    if sim.landed.active {
        crash::apply_landed_nail(&mut sim.asm, &sim.landed);
    }
    let terminal = !sim.crash_msg.is_empty()
        || !sim.landed_msg.is_empty()
        || !sim.launch_failed_msg.is_empty();
    if !terminal && !sim.landed.active {
        if thr_level > 1e-6 {
            sim.thrust_attempted = true;
        }
        // 离地解锁：底部相对**地形**净间隙曾 >1 m（径向高度在丘陵地形下会误解锁）。
        if sim.pad.launched && !sim.was_airborne {
            if let Some(clearance) = sim.asm.bottom_clearance() {
                if clearance > 1.0 {
                    sim.was_airborne = true;
                }
            }
        }
        let pad_timeout = pad::note_ignition_and_check_timeout(&mut sim.pad, thr_level, sim_t_s);
        if pad_timeout {
            let name = sim.asm.active_name().to_string();
            sim.launch_failed_msg = format!(
                "{name} 发射失败：点火 10 秒推力未超过重量的 1.05 倍，台架未释放（T/W {:.2}）",
                current_twr(&sim.asm)
            );
        } else {
            match crash::apply_contact_checks(
                &mut sim.asm,
                sim.pad.launched,
                sim.was_airborne,
                sim.thrust_attempted,
                eff_thrust,
                &mut sim.land_dwell,
                &mut sim.landed,
                sim_t_s,
            ) {
                crash::ContactOutcome::Crash { water, force_n } => {
                    let name = sim.asm.active_name().to_string();
                    sim.crash_msg = if water {
                        format!("{name} 坠入水面")
                    } else {
                        format!("{name} 撞击地面，接触力 {force_n:.0} N")
                    };
                    clock.set_paused(true);
                }
                crash::ContactOutcome::Landed { force_n } => {
                    let name = sim.asm.active_name().to_string();
                    sim.landed_msg = format!(
                        "{name} 着陆成功（无动力稳定 {land_stable:.0} s），接触力 {force_n:.0} N",
                        land_stable = crash::LAND_STABLE_S
                    );
                }
                crash::ContactOutcome::ReturnedToPad => {
                    // 已释放但从未离地、慢速回落：台架重新接管，可再次点火，非终局。
                    sim.pad.reengage();
                    sim.land_dwell.clear();
                }
                crash::ContactOutcome::None => {}
            }
        }
    }

    // 6 环境 → T1
    sim.world.psys.advance(dt / 86_400.0);
    clock.advance_fixed_step();

    // 7 分离事件（若有）：取走即清，随当步切片下发。
    let separated = sim.asm.take_pending_separations();
    let slice = build_slice(clock, sim, stepping, separated);
    sim.last_slice = slice.clone();
    TickOutcome::Stepped { slice }
}

/// 富 Slice 组装**单一来源**：步进切片（tick）与非步进状态切片（run_loop
/// 状态回显，P3.13）共用。**纯读契约**——只读 `clock` / `sim` 当前态，无任何
/// 步进副作用（`last_slice` 写权归 tick()）；`separated` 仅分离步非空，
/// 状态切片恒传空。
pub(crate) fn build_slice(
    clock: &Clock,
    sim: &mut SimBundle,
    stepping: bool,
    separated: Vec<usize>,
) -> Slice {
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
            let init_f = sim.initial_fuel.get(i).copied().unwrap_or(v.fuel_mass());
            let fp = if init_f > 1e-9 {
                (v.fuel_mass() / init_f * 100.0).clamp(0.0, 100.0)
            } else {
                0.0
            };
            let (throttle, firing) = visual_throttle(v);
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
                empty_fuel: v.fuel_mass() < 1.0,
                strap_on: strap_on && primary.contains(&i) && i != asm.active,
                fuel: v.fuel_mass(),
                throttle,
                vessel_index: i as u32,
            }
        })
        .collect();

    let active_vi = asm.active.min(asm.vessels.len().saturating_sub(1));
    let mut focus = build_focus_telem(asm, active_vi, true, &sim.initial_fuel, sim.earth_radius);

    let mut detached = Vec::new();
    for i in 0..asm.vessels.len() {
        if asm.vessels[i].detached || !primary.contains(&i) {
            let mut ft = build_focus_telem(asm, i, false, &sim.initial_fuel, sim.earth_radius);
            // 分离体 throttle 用视觉口径（dtbl 喂焰效）：燃空/关机 → 0；
            // 主船 focus 保留 level 口径作滑条指令回显。
            ft.throttle = visual_throttle(&asm.vessels[i]).0;
            detached.push(ft);
        }
    }

    let (pitch_t, yaw_t, roll_t) = sim.control.attitude_targets();
    let (rcs_enabled, has_rcs) = sim.control.rcs_state();

    let mu = GGRAV * sim.world.psys.primary_surface().mass;
    let el = Elements::calculate(pos, vel, mu, 0.0);
    let energy = vel.length2() * 0.5 - mu / r_mag;
    let escaping = energy >= 0.0;
    let pe_r = if el.e < 1.0 { el.a * (1.0 - el.e) } else { 0.0 };
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

    // HUD 经纬与地表采样同口径（体固系），共转时不爬经度。
    let (lat, lng) = geocentric_lat_lon_deg_orient(pos, asm.planet_orientation.rot);
    let alt = bottom_elevation_m(asm, sim.earth_radius);

    let (sun_direction, earth_obl, earth_lan, earth_rot, starfield) =
        solar_orientation_fields(sim, pos);
    // 地球相对焦点（root 船原点，= focus.pos）；装配 CG 与 root 原点不再重合
    // （刚体钉死保持 rest 布局），勿用 asm.state.pos（CG）口径。
    let earth_position_rel_focus = [-focus.pos[0], -focus.pos[1], -focus.pos[2]];
    let planet_rot = flatten_mat3(asm.planet_orientation.rot);

    // 对地速率增减率（HUD ACC 真值，P3.11b）：权威侧逐切片差分 d|speed|/dt
    // （与 focus.speed 地速同源）；仅 primary 焦点差分，detached 恒 0.0
    // （未实现差分，勿消费）。asm 借用到 planet_rot 止，此处才可 &mut tracker。
    focus.accel = sim
        .acc_tracker
        .diff(focus.vessel_index, clock.sim_t_ms(), focus.speed);

    Slice {
        sim_t: clock.sim_t_ms(),
        step_index: clock.step_index(),
        paused: clock.paused(),
        warp: clock.warp(),
        rocket_name: sim.world.rocket_name.clone(),
        active_name: asm.active_name().to_string(),
        launched: sim.pad.launched,
        crash_msg: sim.crash_msg.clone(),
        landed_msg: sim.landed_msg.clone(),
        launch_failed_msg: sim.launch_failed_msg.clone(),
        pitch_target: pitch_t,
        yaw_target: yaw_t,
        roll_target: roll_t,
        throttle_cmd: sim.control.throttle_cmd(),
        gravity_turn: sim.control.gravity_turn_enabled(),
        stepping,
        rcs_enabled: rcs_enabled,
        has_rcs: has_rcs,
        focus: focus.clone(),
        detached,
        separated_vessels: separated.iter().map(|&i| i as u32).collect(),
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
        ready: true,
        sun_direction,
        earth_position_rel_focus,
        earth_obliquity_deg: earth_obl,
        earth_lan_deg: earth_lan,
        earth_rotation_deg: earth_rot,
        starfield_alignment: starfield,
        planet_rot,
    }
}

fn flatten_mat3(m: Matrix3) -> [f64; 9] {
    mat3_from_orbitx(
        m.m11, m.m12, m.m13, m.m21, m.m22, m.m23, m.m31, m.m32, m.m33,
    )
}

fn solar_orientation_fields(
    sim: &SimBundle,
    focus_pos: orbitx_math::vec3::Vec3,
) -> ([f64; 3], f64, f64, f64, [f64; 4]) {
    let rad2deg = 180.0 / std::f64::consts::PI;
    let sun_pos = sim
        .world
        .psys
        .bodies()
        .iter()
        .find(|b| b.name == "Sun")
        .map(|b| b.pos);
    let sun_direction = match sun_pos {
        Some(sp) => {
            let d = sp - focus_pos;
            let n = d.length();
            if n > 1e-9 {
                [d.x / n, d.y / n, d.z / n]
            } else {
                [1.0, 0.0, 0.0]
            }
        }
        None => [1.0, 0.0, 0.0],
    };

    // 主天体 = 有自转且 dynamics 的体（地心系下为 Earth）。
    let (obl_rad, lan_rad, rot_rad) = sim
        .world
        .psys
        .bodies()
        .iter()
        .find(|b| b.dynamics && b.rotation.is_some())
        .and_then(|b| b.rotation.as_ref())
        .map(|r| (r.obliquity_rel(), r.lan_ecl(), r.rotation()))
        .unwrap_or((0.409_092_802_3, 0.0, 0.0));

    let half = obl_rad * 0.5;
    let starfield = [half.sin(), 0.0, 0.0, half.cos()];

    (
        sun_direction,
        obl_rad * rad2deg,
        lan_rad * rad2deg,
        rot_rad * rad2deg,
        starfield,
    )
}

/// 当前推重比（大气有效推力 / 重量），供发射失败文案；与遥测 T/W 同口径。
fn current_twr(asm: &Assembly) -> f64 {
    let weight = asm.total_mass() * G0;
    if weight > 1e-9 {
        pad::effective_thrust(asm) / weight
    } else {
        0.0
    }
}

/// 显示用底部海拔 [m]：主组合体底点相对高程球（`earth_radius`）的高度。
/// 台上待命 ≈ 场坪海拔 + 台架高；与触地判定用的底部净间隙（相对地形）是两个量。
fn bottom_elevation_m(asm: &Assembly, earth_radius: f64) -> f64 {
    asm.primary_bottom_point()
        .map(|p| p.length() - earth_radius)
        .unwrap_or_else(|| asm.state.pos.length() - earth_radius)
}

/// 焦点遥测构建。
///
/// # 口径契约（P4.5 修订）
/// `pos`/`vel`/`r` 恒取焦点船（is_primary = `asm.active` 主级，否则 detached 船）的
/// **船原点状态**——`Assembly.state` 在飞行阶段是簇质心、pad 钉死阶段被 `pad.rs`
/// 覆盖为主级原点，双语义禁止外泄到遥测。模型根挂在主级原点上（scene.toml 级 0），
/// 消费端免 CG 换算；级间分离切焦点时参考系随之切换（集成阶段处理级 0 基准变换）。
/// 视觉/点火口径：本步实际推力占该级主推真空总推力比 ∈ [0,1]。
/// 取 `diagnostics.thrust`（步进冻结，燃空/气压关断为 0），与焰效/点火显示同源；
/// 与油门指令 `level`（控制回显口径）区分。返回 (throttle, firing)。
/// 视觉/点火口径推力级（黑匣子帧 `thrust_level` 同源复用，禁止第二份实现）。
pub(crate) fn visual_throttle(v: &orbitx_vessel::Vessel) -> (f64, bool) {
    let max_total: f64 = v
        .thrusters
        .iter()
        .filter(|t| t.max_thrust > 0.0)
        .map(|t| t.max_thrust)
        .sum();
    if max_total <= 1e-9 {
        return (0.0, false);
    }
    let ratio = (v.diagnostics.thrust / max_total).clamp(0.0, 1.0);
    (ratio, ratio > 0.02)
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
            v.state.pos,
            v.state.vel,
            mass,
            fuel,
            fuel_pct,
            thrust,
            asm.active_name().to_string(),
        )
    } else {
        let init = initial_fuel
            .get(vi)
            .copied()
            .unwrap_or(v.fuel_mass())
            .max(v.fuel_mass());
        let fuel_pct = if init > 1e-9 {
            (v.fuel_mass() / init * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        (
            v.state.pos,
            v.state.vel,
            v.mass(),
            v.fuel_mass(),
            fuel_pct,
            v.diagnostics.thrust,
            v.name.clone(),
        )
    };

    let surf = surface_inertial_velocity(pos, asm.planet_orientation.omega);
    let vel_ground = vel_inertial - surf;
    let r_mag = pos.length().max(1e-3);
    let radial = pos * (1.0 / r_mag);
    let mut v_vert = dot(vel_ground, radial);
    if v_vert.abs() < 0.5 {
        v_vert = 0.0;
    }
    let v_horiz = (vel_ground - radial * v_vert).length();
    // 显示高度 = 底部海拔（相对高程球），非离地间隙；判定口径不受影响。
    let alt = if is_primary {
        asm.primary_bottom_point()
    } else {
        Some(Assembly::vessel_bottom_point(&asm.vessels[vi]))
    }
    .map(|p| p.length() - earth_radius)
    .unwrap_or_else(|| pos.length() - earth_radius);
    let twr = if mass > 1e-9 {
        thrust / (mass * G0)
    } else {
        0.0
    };

    let d = &v.diagnostics;
    // 主栈/分离体姿态一律读步进写入的 diagnostics，禁止对 live state 再算一套。
    let (pitch, yaw, roll, tip) = (d.pitch, d.yaw, d.roll, d.tip);

    let (gimbal_p, gimbal_y, thr_level) = {
        let t = v.thrusters.iter().find(|t| t.max_thrust > 0.0);
        match t {
            Some(t) => (t.gimbal_pitch, t.gimbal_yaw, t.level),
            None => (0.0, 0.0, 0.0),
        }
    };

    // 推力方向（WP1）：跨全簇推进器推力加权合成（含固定与万向喷管）——
    // 混合簇里只读单台会漏掉 gimbal 打舵（2026-10-29 实锤：首台固定 lch4
    // → gimbal telem 恒 0，重力转弯段真实打舵被掩盖）。零推力回退 +Y。
    let mut dir_acc = Vec3::ZERO;
    let mut thrust_w_sum = 0.0;
    for t in &v.thrusters {
        let w = t.current_thrust(d.pressure);
        if w > 0.0 {
            dir_acc = dir_acc + t.current_dir() * w;
            thrust_w_sum += w;
        }
    }
    let thrust_dir = if thrust_w_sum > 1e-9 {
        (dir_acc * (1.0 / thrust_w_sum)).unit()
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let omega = v.state.omega;

    // 姿态恒取主级（焦点）船参考系：与 pos 同源，pad 钉死/飞行全阶段一致。
    let r = v.state.r;

    FocusTelem {
        is_primary,
        display_name,
        vessel_index: vi as u32,
        pos: [pos.x, pos.y, pos.z],
        vel: [vel_ground.x, vel_ground.y, vel_ground.z],
        altitude: alt,
        speed: vel_ground.length(),
        // primary 由 build_slice 尾部 diff 覆盖；detached 恒 0.0（未差分）。
        accel: 0.0,
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
        aoa: d.aoa,
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
        thrust_dir: [thrust_dir.x, thrust_dir.y, thrust_dir.z],
        rot: flatten_mat3(r),
    }
}

#[cfg(test)]
mod tick_tests;
