//! Assembly 对接 / 分离局部不变量（同轴栈）+ GG/多机诊断。

use super::*;
use crate::pad::surface_inertial_velocity;
use crate::presets;
use crate::stage::{StageSpec, ThrusterSpec};
use crate::thruster::P_REF_SL;
use orbitx_dynamics::GravBody;
use orbitx_math::{Matrix3, Quat, StateVectors, Vec3};

#[test]
fn coaxial_new_docks_adjacent_top_bottom() {
    let asm = Assembly::new(&presets::falcon9(), StateVectors::default());
    assert!(asm.vessels[0].docks[1].connected_to.is_some());
    assert!(asm.vessels[1].docks[0].connected_to.is_some());
    assert_eq!(asm.components.len(), 3);
}

#[test]
fn separate_stage_detaches_bottom_keeps_upper() {
    let mut asm = Assembly::new(&presets::falcon9(), StateVectors::default());
    let next = asm.separate_stage();
    assert!(asm.vessels[0].detached);
    assert!(!asm.vessels[1].detached);
    assert_eq!(asm.active, next);
    assert_eq!(asm.active_name(), "F9-S2");
    assert_eq!(asm.components.len(), 2);
}

#[test]
fn undock_records_pending_separations_take_once() {
    let mut asm = Assembly::new(&presets::falcon9(), StateVectors::default());
    assert!(asm.take_pending_separations().is_empty(), "完整栈无分离事件");

    // 无效 undock（不存在的 vessel）不产生事件。
    let _ = asm.undock(999, 0, 0.5);
    assert!(asm.take_pending_separations().is_empty());

    asm.separate_stage();
    let ev = asm.take_pending_separations();
    assert_eq!(ev, vec![0], "falcon9 底级 = vessel 0");
    assert!(asm.take_pending_separations().is_empty(), "取走即清，事件只下发一次");
}

#[test]
fn separation_velocity_points_away_from_keep() {
    // port.dir 是对接接近方向（指向对方 = keep 侧）；leave 必须取反向，
    // 否则分离体朝 keep 撞（径向助推贴箭体、轴向抛离级怼上级）。
    let mut asm = Assembly::new(&presets::falcon9(), StateVectors::default());
    asm.separate_stage();
    assert!(asm.vessels[0].detached);
    // 默认姿态 r = I：竖直链底级 leave 应向下（−Y）弹离；keep 反冲记录在
    // 组合体 state（undock 写 asm.state.vel，投影回各船发生在 writeback）。
    let v_leave = asm.vessels[0].state.vel;
    let v_keep = asm.state.vel;
    assert!(v_leave.y < -1e-6, "leave 应向下弹离，实际 {:?}", v_leave);
    assert!(v_keep.y > 1e-6, "keep 反冲应向上，实际 {:?}", v_keep);
}

#[test]
fn detached_step_keeps_origin_frame_without_com_jump() {
    // 非零质心构型：falcon9 满载原点约定下 com_body=0，口径转换退化为
    // 恒等变换、掩盖泄漏。leave 级干质心偏 20 m、罐在原点 → com_body≈10 m：
    // 修复前每步漂移 +R·com_body（分离体瞬移根因）。
    let i_cyl = crate::stage::cylinder_inertia(1000.0, 1.0, 10.0);
    let s1 = StageSpec {
        name: "OC-S1",
        dry_mass: 1000.0,
        dry_center: Vec3::new(0.0, 20.0, 0.0),
        dry_inertia: i_cyl,
        tanks: vec![crate::stage::TankSpec {
            id: 1,
            max_mass: 1000.0,
            mass: 1000.0,
            pos: Vec3::ZERO,
            inertia: i_cyl,
            efficiency: 1.0,
        }],
        length: 10.0,
        radius: 1.0,
        separation_impulse: 2.0,
        ..Default::default()
    };
    let s2 = StageSpec {
        name: "OC-S2",
        dry_mass: 1000.0,
        dry_inertia: i_cyl,
        length: 10.0,
        radius: 1.0,
        separation_impulse: 2.0,
        ..Default::default()
    };
    let mut asm = Assembly::new(&[s1, s2], StateVectors::default());
    asm.ground = GroundContact::Off;
    // 抬到轨道高度：pos=(0,0,0) 在行星半径内，会触发默认底环触地修正，
    // 污染「静止体无位移」断言。
    asm.state.pos = Vec3::new(6.8e6, 0.0, 0.0);
    asm.separate_stage();
    assert!(asm.vessels[0].detached);
    let com0 = asm.vessels[0].com_body();
    assert!(com0.y > 5.0, "测试前提：leave 级 com_body 非零，实际 {:?}", com0);
    // 清分离冲量：纯口径回归只关心静态质点一步不产生位移。
    asm.vessels[0].state.vel = Vec3::ZERO;
    asm.vessels[0].state.omega = Vec3::ZERO;
    let p0 = asm.vessels[0].state.pos;
    asm.step_detached(0.05, StepEnv::primary0(&[]));
    let drift = (asm.vessels[0].state.pos - p0).length();
    assert!(
        drift < 1e-6,
        "静止分离体一步漂移 {} m（com_body 口径泄漏）",
        drift
    );
}

#[test]
fn with_dock_links_empty_is_single_components() {
    let stages = presets::falcon9();
    let asm = Assembly::with_dock_links(&stages[..1], StateVectors::default(), &[]);
    assert_eq!(asm.components.len(), 1);
    assert_eq!(asm.total_mass(), stages[0].total_mass());
}

#[test]
fn leo_slender_gg_nonzero_via_dynamics() {
    let tau = orbitx_dynamics::gravity_gradient_torque(
        Vec3::new(4.5e6, 4.5e6, 0.0),
        5.972e24,
        Vec3::new(1e5, 1e3, 1e5),
        Matrix3::IDENTITY,
        Vec3::ZERO,
        0.0,
        1.0,
        false,
    );
    assert!(tau.length() > 1e-9, "LEO slender PMI GG torque = {tau:?}");
}

#[test]
fn tidaldamp_changes_torque_vs_zero() {
    let pmi = Vec3::new(1e5, 1e3, 1e5);
    let rel = Vec3::new(4.5e6, 4.5e6, 0.0);
    let omega = Vec3::new(0.01, 0.0, 0.0);
    let tau0 = orbitx_dynamics::gravity_gradient_torque(
        rel,
        5.972e24,
        pmi,
        Matrix3::IDENTITY,
        omega,
        0.0,
        0.1,
        false,
    );
    let tau_d = orbitx_dynamics::gravity_gradient_torque(
        rel,
        5.972e24,
        pmi,
        Matrix3::IDENTITY,
        omega,
        1.0,
        0.1,
        false,
    );
    assert!(
        (tau_d - tau0).length() > 1e-12,
        "tidaldamp should alter torque: {:?} vs {:?}",
        tau_d,
        tau0
    );
}

#[test]
fn vessel_tidaldamp_wired_from_spec() {
    let spec = StageSpec {
        name: "T",
        dry_mass: 100.0,
        thrusters: vec![],
        length: 10.0,
        radius: 1.0,
        separation_impulse: 0.0,
        tidaldamp: 0.5,
        ..Default::default()
    }
    .with_fuel(0.0);
    let v = crate::Vessel::from_spec(0, &spec, StateVectors::default());
    assert!((v.tidaldamp - 0.5).abs() < 1e-15);
}

#[test]
fn canted_thrusters_produce_torque() {
    let spec = StageSpec {
        name: "Canted",
        dry_mass: 1000.0,
        thrusters: vec![ThrusterSpec {
            pos: Vec3::new(1.0, -5.0, 0.0),
            dir: Vec3::new(0.2, 1.0, 0.0).unit(),
            thrust: 100_000.0,
            isp: 300.0,
            ..Default::default()
        }],
        length: 10.0,
        radius: 1.0,
        separation_impulse: 0.0,
        ..Default::default()
    }
    .with_fuel(500.0);
    let mut asm = Assembly::new(&[spec], StateVectors::default());
    asm.set_throttle(1.0);
    let p = 0.0;
    let t = &asm.vessels[0].thrusters[0];
    let f = t.current_dir() * t.current_thrust(p);
    let m = orbitx_math::cross(f, t.pos);
    assert!(
        m.length() > 1.0,
        "canted thrust should yield torque, got {m:?}"
    );
}

#[test]
fn pfac_scales_thrust_at_sl_and_vacuum() {
    let spec = StageSpec {
        name: "P",
        dry_mass: 100.0,
        thrusters: vec![ThrusterSpec {
            pos: Vec3::ZERO,
            dir: Vec3::new(0.0, 1.0, 0.0),
            thrust: 1000.0,
            isp: 300.0,
            thrust_sl: Some(800.0),
            isp_sl: Some(240.0),
            ..Default::default()
        }],
        length: 5.0,
        radius: 1.0,
        separation_impulse: 0.0,
        ..Default::default()
    }
    .with_fuel(100.0);
    let mut asm = Assembly::new(&[spec], StateVectors::default());
    asm.set_throttle(1.0);
    let t = &asm.vessels[0].thrusters[0];
    assert!((t.current_thrust(0.0) - 1000.0).abs() < 1.0);
    let thr_sl = t.current_thrust(P_REF_SL);
    assert!((thr_sl - 800.0).abs() < 5.0, "SL thrust = {thr_sl}");
}

#[test]
fn falcon9_preset_has_thrusters() {
    let stages = presets::falcon9();
    assert!(!stages[0].thrusters.is_empty());
    assert!(!stages[1].thrusters.is_empty());
    assert!(stages[2].thrusters.is_empty());
    assert_eq!(stages[0].thrusters.len(), 9);
}

#[test]
fn corotating_atmosphere_zero_airspeed_on_pad() {
    // 台位：v = ω×r，大气共转 → 阻力≈0、Ma≈0
    let stages = presets::falcon9();
    let r = 6.37101e6 + 50.0;
    let period = 86_164.1;
    let omega = Vec3::new(0.0, orbitx_math::PI2 / period, 0.0);
    let pos = Vec3::new(0.0, 0.0, r);
    let vel = surface_inertial_velocity(pos, omega);
    let state = StateVectors {
        pos,
        vel,
        ..Default::default()
    };
    let mut asm = Assembly::new(&stages, state);
    asm.atmosphere = Some(Box::new(crate::UsStd1976Atmosphere::new()));
    asm.planet_radius = 6.37101e6;
    asm.sid_rot_period = period;
    asm.planet_orientation.omega = omega;
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    asm.step(0.05, StepEnv::primary0(&[earth]));
    let d = asm.diagnostics();
    assert!(d.mach < 0.05, "pad Ma should be ~0, got {}", d.mach);
    assert!(
        d.drag_force < 1e4,
        "pad drag should be tiny, got {}",
        d.drag_force
    );
    assert!(d.density > 0.5);
    assert!(d.a_grav > 5.0);
}

#[test]
fn relative_airspeed_produces_mach() {
    // 相对共转大气有 50 m/s → 应有小但非零 Ma
    let stages = presets::falcon9();
    let r = 6.37101e6 + 50.0;
    let period = 86_164.1;
    let omega = Vec3::new(0.0, orbitx_math::PI2 / period, 0.0);
    let pos = Vec3::new(0.0, 0.0, r);
    let vel = surface_inertial_velocity(pos, omega) + Vec3::new(50.0, 0.0, 0.0);
    let state = StateVectors {
        pos,
        vel,
        ..Default::default()
    };
    let mut asm = Assembly::new(&stages, state);
    asm.atmosphere = Some(Box::new(crate::UsStd1976Atmosphere::new()));
    asm.planet_radius = 6.37101e6;
    asm.sid_rot_period = period;
    asm.planet_orientation.omega = omega;
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    asm.step(0.05, StepEnv::primary0(&[earth]));
    let d = asm.diagnostics();
    assert!(d.mach > 0.05 && d.mach < 0.5);
}

#[test]
fn each_vessel_keeps_independent_diagnostics_after_sep() {
    let stages = presets::falcon9();
    let r = 6.37101e6 + 20_000.0;
    let pos = Vec3::new(0.0, 0.0, r);
    let state = StateVectors {
        pos,
        vel: Vec3::new(800.0, 0.0, 0.0),
        ..Default::default()
    };
    let mut asm = Assembly::new(&stages, state);
    asm.atmosphere = Some(Box::new(crate::UsStd1976Atmosphere::new()));
    asm.planet_radius = 6.37101e6;
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let _ = asm.separate_stage();
    asm.step(0.05, StepEnv::primary0(&[earth]));

    assert!(asm.vessels[0].detached);
    assert!(asm.vessels[0].diagnostics.a_grav > 5.0);
    assert!(asm.vessels[0].diagnostics.density > 0.0);
    assert!(asm.vessels[0].diagnostics.mach.is_finite());

    let active = asm.active;
    assert!(!asm.vessels[active].detached);
    assert!(asm.vessels[active].diagnostics.a_grav > 5.0);
    assert!(asm.vessels[active].diagnostics.density > 0.0);
    assert!((asm.diagnostics().mach - asm.vessels[active].diagnostics.mach).abs() < 1e-12);

    // 主栈与分离体各有独立快照，互不覆盖。
    let d0 = asm.vessels[0].diagnostics.mach;
    let da = asm.vessels[active].diagnostics.mach;
    assert!(d0.is_finite() && da.is_finite());
}

#[test]
fn mark_crashed_detached_skips_integration() {
    let stages = presets::falcon9();
    let r = 6.37101e6 + 50_000.0;
    let state = StateVectors {
        pos: Vec3::new(0.0, 0.0, r),
        vel: Vec3::new(100.0, 0.0, 0.0),
        ..Default::default()
    };
    let mut asm = Assembly::new(&stages, state);
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let _ = asm.separate_stage();
    assert!(asm.vessels[0].detached);
    asm.mark_crashed(0);
    let pos0 = asm.vessels[0].state.pos;
    let bodies = [earth];
    for _ in 0..20 {
        asm.step(0.05, StepEnv::primary0(&bodies));
    }
    assert!(asm.vessels[0].crashed);
    assert!((asm.vessels[0].state.pos - pos0).length() < 1e-9);
    assert!(asm.vessels[0].state.vel.length() < 1e-12);
}

#[test]
fn mark_crashed_primary_skips_but_detached_still_steps() {
    let stages = presets::falcon9();
    let r = 6.37101e6 + 50_000.0;
    let state = StateVectors {
        pos: Vec3::new(0.0, 0.0, r),
        vel: Vec3::new(100.0, 0.0, 0.0),
        ..Default::default()
    };
    let mut asm = Assembly::new(&stages, state);
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let _ = asm.separate_stage();
    let active = asm.active;
    asm.mark_crashed(active);
    let primary_pos = asm.state.pos;
    let detached_pos = asm.vessels[0].state.pos;
    let bodies = [earth];
    for _ in 0..20 {
        asm.step(0.05, StepEnv::primary0(&bodies));
    }
    assert!((asm.state.pos - primary_pos).length() < 1e-9);
    assert!(
        (asm.vessels[0].state.pos - detached_pos).length() > 1.0,
        "uncrashed detached should still integrate"
    );
}

#[test]
fn assembly_uses_vessel_tidaldamp() {
    let mut spec = StageSpec {
        name: "T",
        dry_mass: 1000.0,
        thrusters: vec![],
        length: 20.0,
        radius: 1.0,
        separation_impulse: 0.0,
        tidaldamp: 2.0,
        ..Default::default()
    }
    .with_fuel(0.0);
    let r_leo = 6.771e6;
    let state = StateVectors {
        pos: Vec3::new(r_leo * 0.707, r_leo * 0.707, 0.0),
        vel: Vec3::ZERO,
        omega: Vec3::new(0.05, 0.0, 0.0),
        r: Matrix3::IDENTITY,
        q: Quat::IDENTITY,
        ..Default::default()
    };
    let mut asm = Assembly::new(&[spec], state);
    assert!((asm.vessels[0].tidaldamp - 2.0).abs() < 1e-15);
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let w0 = asm.state.omega.x;
    asm.step(0.1, StepEnv::primary0(&[earth]));
    assert!(asm.state.omega.x.is_finite());
    let _ = w0;
}

/// 分离体在稠密大气中应受气动减速（相对无大气同初值）。
#[test]
fn detached_vessel_gets_aero_drag() {
    use crate::aero::{DragElement, ExponentialAtmosphere};

    let lower = StageSpec::with_single_thruster(
        "L",
        5_000.0,
        0.0,
        0.0,
        300.0,
        Vec3::new(0.0, -5.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        10.0,
        2.0,
        2.0,
    );
    let upper = StageSpec::with_single_thruster(
        "U",
        1_000.0,
        0.0,
        0.0,
        300.0,
        Vec3::new(0.0, -2.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        5.0,
        1.0,
        0.0,
    );
    let init = StateVectors {
        pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 20_000.0),
        vel: Vec3::new(800.0, 0.0, 0.0),
        omega: Vec3::ZERO,
        r: Matrix3::IDENTITY,
        q: Quat::IDENTITY,
        ..Default::default()
    };
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };

    let mut with_atm = Assembly::new(&[lower.clone(), upper.clone()], init);
    with_atm.vessels[0]
        .dragels
        .push(DragElement::constant(Vec3::ZERO, 0.5, 8.0));
    with_atm.vessels[0].cross_section = Vec3::new(2.0, 8.0, 2.0);
    with_atm.vessels[0].rdrag = Vec3::new(1.0, 0.1, 1.0);
    with_atm.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
    with_atm.planet_radius = 6_371_000.0;
    with_atm.separate_stage();
    assert!(with_atm.vessels[0].detached);

    let mut no_atm = Assembly::new(&[lower, upper], init);
    no_atm.vessels[0]
        .dragels
        .push(DragElement::constant(Vec3::ZERO, 0.5, 8.0));
    no_atm.vessels[0].cross_section = Vec3::new(2.0, 8.0, 2.0);
    no_atm.vessels[0].rdrag = Vec3::new(1.0, 0.1, 1.0);
    no_atm.planet_radius = 6_371_000.0;
    no_atm.separate_stage();

    let dt = 0.05;
    for _ in 0..40 {
        with_atm.step(dt, StepEnv::primary0(&[earth.clone()]));
        no_atm.step(dt, StepEnv::primary0(&[earth.clone()]));
    }
    let v_atm = with_atm.vessels[0].state.vel.length();
    let v_vac = no_atm.vessels[0].state.vel.length();
    assert!(
        v_atm < v_vac - 1.0,
        "detached with aero should be slower: {v_atm} vs {v_vac}"
    );
}

/// 分离体若仍开节流阀，应继续耗油（与主栈同一结算路径）。
#[test]
fn detached_vessel_burns_fuel_when_throttled() {
    let lower = StageSpec::with_single_thruster(
        "L",
        2_000.0,
        500.0,
        50_000.0,
        300.0,
        Vec3::new(0.0, -4.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        8.0,
        1.0,
        1.0,
    );
    let upper = StageSpec::with_single_thruster(
        "U",
        500.0,
        0.0,
        0.0,
        300.0,
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        3.0,
        0.5,
        0.0,
    );
    let init = StateVectors {
        pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 100_000.0),
        vel: Vec3::new(0.0, 0.0, 100.0),
        ..Default::default()
    };
    let mut asm = Assembly::new(&[lower, upper], init);
    asm.vessels[0].set_throttle(1.0);
    // 瞬时节流：无斜坡时一步后 level==cmd
    for t in &mut asm.vessels[0].thrusters {
        t.level = 1.0;
        t.level_cmd = 1.0;
    }
    asm.separate_stage();
    assert!(asm.vessels[0].detached);
    let fuel0 = asm.vessels[0].fuel_mass();
    assert!(fuel0 > 10.0);

    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    for _ in 0..20 {
        asm.step(0.1, StepEnv::primary0(&[earth.clone()]));
    }
    assert!(
        asm.vessels[0].fuel_mass() < fuel0 - 0.1,
        "detached should burn fuel: {} -> {}",
        fuel0,
        asm.vessels[0].fuel_mass()
    );
}

#[test]
fn step_env_primary_not_list_first() {
    let sun = GravBody {
        pos: Vec3::new(1.496e11, 0.0, 0.0),
        mass: 1.9885e30,
        size: 6.96e8,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let bodies = [sun, earth];
    let env = StepEnv::new(&bodies, 1);
    let b = env.primary_body().expect("Earth primary");
    assert!(b.pos.length() < 1.0);
    assert!((b.mass - 5.972e24).abs() < 1e15);
    assert!((b.size - 6_371_000.0).abs() < 1.0);

    // Wrong primary (Sun): mass/pos must differ — host must not leave primary=0 on sol().
    let wrong = StepEnv::primary0(&bodies).primary_body().unwrap();
    assert!(wrong.pos.length() > 1e10);
    assert!(wrong.mass > 1e30);
}

#[test]
fn step_subtracts_primary_acceleration() {
    fn vel_after(bodies: &[GravBody], primary: usize) -> f64 {
        let mut state = StateVectors::default();
        state.pos = Vec3::new(6.771e6, 0.0, 0.0);
        let mut asm = Assembly::new(&presets::falcon9(), state);
        for v in &mut asm.vessels {
            for t in &mut v.thrusters {
                t.level = 0.0;
            }
        }
        asm.step(1.0, StepEnv::new(bodies, primary));
        asm.state.vel.length()
    }

    let earth = GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let sun = GravBody {
        pos: Vec3::new(1.4959787e11, 0.0, 0.0),
        mass: 1.9885e30,
        size: 6.96e8,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    };
    let earth_only = vel_after(&[earth.clone()], 0);
    let with_sun = vel_after(&[earth, sun], 0);
    assert!(
        (with_sun - earth_only).abs() < 1e-3,
        "sun should cancel: earth {earth_only}, both {with_sun}"
    );
}

#[test]
fn rocket_from_spec_skips_origin_dragel() {
    let spec = StageSpec {
        name: "R",
        dry_mass: 1000.0,
        length: 10.0,
        radius: 1.0,
        separation_impulse: 0.0,
        cd_mach: crate::stage::default_rocket_cd_mach(),
        ..Default::default()
    };
    let v = crate::Vessel::from_spec(0, &spec, StateVectors::default());
    assert!(v.rocket_body.is_some());
    assert!(v.dragels.is_empty());
}

#[test]
fn rocket_and_aircraft_aero_paths_mutually_exclusive() {
    use crate::aero::{Airfoil, AirfoilCoeffs, AirfoilOrientation};

    let mut v = crate::Vessel::from_spec(
        0,
        &StageSpec {
            name: "mixed",
            dry_mass: 1000.0,
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            cd_mach: crate::stage::default_rocket_cd_mach(),
            ..Default::default()
        },
        StateVectors::default(),
    );
    let comps = [SubVesselData {
        vessel_index: 0,
        rpos: Vec3::ZERO,
        rrot: Matrix3::IDENTITY,
        rq: Quat::IDENTITY,
    }];
    assert!(
        super::cluster_uses_rocket_aero(std::slice::from_ref(&v), &comps),
        "cd_mach rocket body without airfoils → rocket path"
    );
    v.airfoils.push(Airfoil {
        ref_pos: Vec3::ZERO,
        orientation: AirfoilOrientation::LiftVertical,
        chord: 1.0,
        area: 2.0,
        aspect_ratio: 4.0,
        coeffs: AirfoilCoeffs::LinearLift {
            cl_alpha: 5.0,
            cl0: 0.0,
            cd0: 0.02,
        },
    });
    assert!(
        !super::cluster_uses_rocket_aero(std::slice::from_ref(&v), &comps),
        "airfoils force P1.1 path"
    );
}

#[test]
fn rocket_aero_moment_about_cluster_cg() {
    use crate::aero::{
        compute_rocket_aero, moment_about_cg, RocketAeroInput, RocketBodyAero, TriaxialAreas,
    };

    let body = RocketBodyAero {
        cd_mach: vec![(0.0, 0.3)],
        cd0: 0.3,
        cn_alpha: 2.0,
        ..RocketBodyAero::default()
    };
    let areas = TriaxialAreas {
        x: 40.0,
        y: std::f64::consts::PI,
        z: 40.0,
    };
    let body_cop = Vec3::new(0.0, 5.0, 0.0);
    let cg = Vec3::new(0.0, 2.0, 0.0);
    let aero = compute_rocket_aero(&RocketAeroInput {
        airvel_body: Vec3::new(0.0, -200.0, 0.0),
        omega_body: Vec3::ZERO,
        rho: 1.2,
        sound_speed: 340.0,
        areas,
        body_cop,
        cg,
        body: &body,
        surfaces: &[],
    });
    let expected = moment_about_cg(aero.force, body_cop, cg);
    assert!((aero.torque - expected).length() < 1e-9);
    // 纯轴向力 + CoP 与 cg 仅 Y 差 → 力矩应为 0（力沿 Y，臂沿 Y）。
    assert!(
        aero.torque.length() < 1e-6,
        "axial force gives zero moment: {:?}",
        aero.torque
    );

    let aero_side = compute_rocket_aero(&RocketAeroInput {
        airvel_body: Vec3::new(50.0, -200.0, 0.0),
        omega_body: Vec3::ZERO,
        rho: 1.2,
        sound_speed: 340.0,
        areas,
        body_cop,
        cg,
        body: &body,
        surfaces: &[],
    });
    assert!(aero_side.torque.length() > 1e-3);
    let expected_side = moment_about_cg(aero_side.force, body_cop, cg);
    assert!((aero_side.torque - expected_side).length() < 1e-9);
}

/// CZ-2F 包络：100 m/s、15° 侧滑时法向力远小于起飞推力。
#[test]
fn cz2f_aero_diag() {
    use crate::aero::{compute_rocket_aero, RocketAeroInput};
    use crate::vessel::stage_spec_from_config;
    use orbitx_config::RocketConfig;

    const THRUST: f64 = 5_923_200.0;
    let toml = orbitx_config::builtin_rocket_toml("lm2f").expect("lm2f");
    let config = RocketConfig::from_toml_str(toml).expect("parse");
    let stages: Vec<StageSpec> = config.stages.iter().map(stage_spec_from_config).collect();
    let links: Vec<(usize, usize, usize, usize)> = config
        .dock_links
        .expect("dock_links")
        .iter()
        .map(|l| (l.stage, l.port, l.remote_stage, l.remote_port))
        .collect();
    let asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let comps = asm.components.clone();
    let geom = super::aero_geom::compute_cluster_aero_geom(&asm.vessels, &comps);
    let masses: Vec<f64> = comps
        .iter()
        .map(|c| asm.vessels[c.vessel_index].mass())
        .collect();
    let cg = center_of_mass(&comps, &masses);

    let body = asm.vessels[0].rocket_body.clone().expect("S1 rocket_body");
    let rho = 1.225;
    let a_snd = 340.0;
    let speed = 100.0;
    for &alpha_deg in &[1.0_f64, 15.0_f64] {
        let alpha = alpha_deg.to_radians();
        // 前飞 −Y，侧风 +X：alpha = atan2(|vx|, |vy|)
        let vy = -speed * alpha.cos();
        let vx = speed * alpha.sin();
        let airvel = Vec3::new(vx, vy, 0.0);
        let aero = compute_rocket_aero(&RocketAeroInput {
            airvel_body: airvel,
            omega_body: Vec3::ZERO,
            rho,
            sound_speed: a_snd,
            areas: geom.areas,
            body_cop: geom.body_cop,
            cg,
            body: &body,
            surfaces: &[],
        });
        let fn_ratio = aero.lift_force / THRUST;
        // 15° 含横流（S_lat），仍应远小于起飞推力。
        if (alpha_deg - 15.0).abs() < 1e-9 {
            assert!(
                fn_ratio < 0.10,
                "Fn/Fth={fn_ratio} at 15deg should stay well below liftoff thrust"
            );
        }
    }
}

/// Demo 四翼：小攻角滚转力矩小于抬头力矩，滚转角速度产生反向力矩。
#[test]
fn demo_fin_roll_diag() {
    use crate::aero::{
        compute_rocket_aero, moment_about_cg, FinKind, LiftingSurface, RocketAeroInput,
        RocketBodyAero, TriaxialAreas,
    };

    fn surf(ref_pos: [f64; 3], normal: [f64; 3], chord: [f64; 3]) -> LiftingSurface {
        LiftingSurface {
            ref_pos: Vec3::new(ref_pos[0], ref_pos[1], ref_pos[2]),
            normal: Vec3::new(normal[0], normal[1], normal[2]),
            chord_dir: Vec3::new(chord[0], chord[1], chord[2]),
            area: 2.22,
            aspect_ratio: 0.87,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: 0.314,
            kind: FinKind::Fixed,
            deploy: 1.0,
            leeward_sheltered: false,
        }
    }

    // 数字来自用户 Demo sim.toml，不把整份文件提交进仓库。
    let surfaces = [
        surf(
            [
                -0.05103094531470312,
                -11.833156783226245,
                -2.4397016474909323,
            ],
            [0.999781370064751, 0.0, -0.020909616721727895],
            [0.0, -1.0, 0.0],
        ),
        surf(
            [0.04821011872615155, -11.833156952707943, 2.3048722339154764],
            [-0.9997813694027929, 0.0, 0.02090964837285109],
            [0.0, -1.0, 0.0],
        ),
        surf(
            [2.423873110677899, -11.833156909008821, -0.05064764506180314],
            [0.020909611135985103, 0.0, 0.9997813701815722],
            [0.0, -1.0, 0.0],
        ),
        surf(
            [
                -2.4231060803776048,
                -11.833157157170307,
                0.05073549070291458,
            ],
            [-0.020909605550492767, 0.0, -0.9997813702983882],
            [0.0, -1.0, 0.0],
        ),
    ];

    let body = RocketBodyAero {
        cd_mach: vec![
            (0.0, 0.3),
            (0.6, 0.32),
            (0.9, 0.55),
            (1.1, 0.95),
            (1.5, 0.7),
            (2.5, 0.45),
            (5.0, 0.35),
        ],
        cd0: 0.3,
        cn_alpha: 2.0,
        ..RocketBodyAero::default()
    };
    let r = 3.2173105103072976;
    let length = 28.797005255346455;
    let areas = TriaxialAreas {
        x: length * 2.0 * r,
        y: std::f64::consts::PI * r * r,
        z: length * 2.0 * r,
    };
    let cg = Vec3::ZERO;
    let body_cop = Vec3::new(0.0, -3.5163613853111695, 0.0);
    let rho = 1.225;
    let a_snd = 340.0;
    let speed = 342.0;
    let pitch = (-2.0_f64).to_radians();
    let omega_y = (-1.93_f64).to_radians();
    let air_a = Vec3::new(0.0, speed, 0.0);
    let air_b = Vec3::new(0.0, speed * pitch.cos(), speed * pitch.sin());
    let omega_c = Vec3::new(0.0, omega_y, 0.0);

    let run = |airvel: Vec3, omega: Vec3| {
        let all = compute_rocket_aero(&RocketAeroInput {
            airvel_body: airvel,
            omega_body: omega,
            rho,
            sound_speed: a_snd,
            areas,
            body_cop,
            cg,
            body: &body,
            surfaces: &surfaces,
        });
        all.torque.y
    };

    let tau_a = run(air_a, Vec3::ZERO);
    let tau_b = run(air_b, Vec3::ZERO);
    let tau_c = run(air_a, omega_c);
    let tau_d = run(air_b, omega_c);

    let eng_pos = Vec3::new(
        0.000000853330683142639,
        -16.063695699390593,
        0.0000551006440698369,
    );
    let dir0 = Vec3::new(0.0, 0.9999999999970044, -0.0000024477585611381867);
    let thrust = 723_000.0;
    let g_p = 2.20_f64.to_radians();
    let g_y = (-0.51_f64).to_radians();
    let dir = orbitx_dynamics::current_dir(dir0, Vec3::new(1.0, 0.0, 0.0), g_p, g_y);
    let f_th = dir * thrust;
    let tau_th = moment_about_cg(f_th, eng_pos, cg);
    assert!(
        tau_th.y.abs() < tau_b.abs(),
        "TVC |τy|={} should be < aero B |τy|={}",
        tau_th.y.abs(),
        tau_b.abs()
    );

    assert!(
        tau_a.abs() < tau_b.abs() * 0.25 + 1.0,
        "A |τy|={tau_a} should be << B |τy|={tau_b}"
    );
    assert!(
        tau_c * omega_y < 0.0,
        "C damping τy={tau_c} should oppose ωy={omega_y}"
    );
    let _ = tau_d;
}

#[test]
fn rigid_aero_static_ignores_active_and_invalidates_on_undock() {
    let s0 = StageSpec {
        name: "heavy",
        dry_mass: 10_000.0,
        length: 20.0,
        radius: 1.5,
        cn_alpha: Some(2.0),
        cd_mach: vec![(0.0, 0.30)],
        ..Default::default()
    };
    let s1 = StageSpec {
        name: "light",
        dry_mass: 2_000.0,
        length: 8.0,
        radius: 1.0,
        cn_alpha: Some(1.0),
        cd_mach: vec![(0.0, 0.90)],
        ..Default::default()
    };

    let mut asm = Assembly::new(&[s0, s1], StateVectors::default());
    assert_eq!(asm.components.len(), 2);

    let comps = asm.components.clone();
    let (geom0, body0) = asm.rigid_aero_static(&comps);
    assert!(
        (body0.cd0 - 0.30).abs() < 1e-12,
        "bake heaviest dry_mass, cd0={}",
        body0.cd0
    );
    assert!((body0.cn_alpha - 2.0).abs() < 1e-12);

    asm.active = 1;
    let (geom1, body1) = asm.rigid_aero_static(&comps);
    assert_eq!(geom0.areas.y, geom1.areas.y);
    assert_eq!(geom0.areas.x, geom1.areas.x);
    assert!((body1.cd0 - body0.cd0).abs() < 1e-12);
    assert!((body1.cn_alpha - body0.cn_alpha).abs() < 1e-12);

    // 恢复 active 到底级后再拆顶口，使轻级离开
    asm.active = 0;
    let leave = asm.undock(0, 1, 1.0);
    assert_eq!(leave, vec![1]);
    let primary = asm.components.clone();
    let (g_p, b_p) = asm.rigid_aero_static(&primary);
    let det_comps = [super::aero_geom::test_comp(
        1,
        Vec3::ZERO,
        Matrix3::IDENTITY,
    )];
    let (g_d, b_d) = asm.rigid_aero_static(&det_comps);
    assert!((b_p.cd0 - 0.30).abs() < 1e-12, "primary keeps heavy body");
    assert!((b_d.cd0 - 0.90).abs() < 1e-12, "detached uses light body");
    assert!(
        (g_p.areas.y - g_d.areas.y).abs() > 0.5,
        "geometries should differ after undock"
    );
}

/// 底部口径契约：`vessel_bottom_point` 用真实外观 `bottom_offset_m`（geometry），
/// 不再是 `length/2` 筒体近似。摆放归零 / 触地 / 海拔 / 坠毁同此口径。
#[test]
fn bottom_point_uses_geometry_not_length_half() {
    use crate::stage::StageGeometry;
    let spec = StageSpec {
        name: "g",
        dry_mass: 1000.0,
        length: 20.0,
        radius: 1.0,
        geometry: Some(StageGeometry {
            bottom_offset_m: 12.0,
            top_offset_m: 8.0,
            max_radius_m: 1.0,
        }),
        ..Default::default()
    }
    .with_fuel(0.0);
    let asm = Assembly::new(&[spec], StateVectors::default());
    let bp = Assembly::vessel_bottom_point(&asm.vessels[0]);
    assert!(
        (bp.y - (-12.0)).abs() < 1e-9,
        "bottom y must be -bottom_offset_m"
    );
}

/// geometry 缺省回退：无 geometry 段（schema 1 旧文件）时 = 筒体对称近似 ±length/2。
#[test]
fn bottom_point_falls_back_to_length_half() {
    let spec = StageSpec {
        name: "legacy",
        dry_mass: 1000.0,
        length: 20.0,
        radius: 1.0,
        ..Default::default()
    }
    .with_fuel(0.0);
    assert!((spec.geometry().bottom_offset_m - 10.0).abs() < 1e-12);
    let asm = Assembly::new(&[spec], StateVectors::default());
    let bp = Assembly::vessel_bottom_point(&asm.vessels[0]);
    assert!((bp.y - (-10.0)).abs() < 1e-9, "fallback bottom y");
}

// ==== 2026-10-27 分离冲量复现（镜像用户真实装配 sc_c0afd38b/sim.toml）====

use crate::dock::DockPort;
use crate::stage::{StageGeometry, TankSpec};
use orbitx_math::dot;

/// S1+S2 同轴级间（真实质量/几何/dock；S1 sep=2.0，S2 sep=0.0）。
fn user_s1_s2() -> Vec<StageSpec> {
    let s1 = StageSpec {
        name: "S1",
        dry_mass: 4390.0,
        dry_center: Vec3::new(0.0, -5.4001903, 0.0),
        dry_inertia: Vec3::new(149371.43, 5243.58, 149372.48),
        tanks: vec![TankSpec {
            id: 0,
            max_mass: 37680.0,
            mass: 37680.0,
            pos: Vec3::new(0.0, 0.6291623, 0.0),
            inertia: Vec3::new(645286.33, 59692.66, 645286.33),
            efficiency: 1.0,
        }],
        length: 21.62396,
        radius: 2.50084,
        geometry: Some(StageGeometry {
            bottom_offset_m: 10.99479,
            top_offset_m: 10.62916,
            max_radius_m: 2.50084,
        }),
        separation_impulse: 2.0,
        cd_mach: vec![(0.0, 0.3)],
        docks: Some(vec![DockPort::with_rot(
            Vec3::new(0.0, 10.62916, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )]),
        ..Default::default()
    };
    let s2 = StageSpec {
        name: "S2",
        dry_mass: 3179.71,
        dry_center: Vec3::new(0.0, -0.95949, 0.0),
        dry_inertia: Vec3::new(191707.63, 3441.67, 191707.63),
        tanks: vec![TankSpec {
            id: 0,
            max_mass: 37680.0,
            mass: 37680.0,
            pos: Vec3::new(0.0, 0.08097, 0.0),
            inertia: Vec3::new(645286.33, 59692.66, 645286.33),
            efficiency: 1.0,
        }],
        length: 26.83208,
        radius: 2.51391,
        geometry: Some(StageGeometry {
            bottom_offset_m: 12.14804,
            top_offset_m: 14.68405,
            max_radius_m: 2.51391,
        }),
        separation_impulse: 0.0,
        cd_mach: vec![(0.0, 0.3)],
        docks: Some(vec![DockPort::with_rot(
            Vec3::new(0.0, -12.09803, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )]),
        ..Default::default()
    };
    vec![s1, s2]
}

/// 竖直排布的 S1 顶沿 / S2 底沿世界 y。
fn s1_top_s2_bottom_y(asm: &Assembly) -> (f64, f64) {
    let s1 = &asm.vessels[0];
    let s2 = &asm.vessels[1];
    let top = s1.state.pos.y + mul(s1.state.r, Vec3::new(0.0, 10.62916, 0.0)).y;
    let bot = s2.state.pos.y + mul(s2.state.r, Vec3::new(0.0, -12.14804, 0.0)).y;
    (top, bot)
}

#[test]
fn reproduce_interstage_separation_s1_s2() {
    let stages = user_s1_s2();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &[(1, 0, 0, 0)]);
    asm.writeback_primary_states();
    let (top0, bot0) = s1_top_s2_bottom_y(&asm);
    let gap0 = bot0 - top0;
    println!("[复现] 分离前 S1顶/S2底 间距 gap0 = {gap0:.6} m（S2 dock pos 与 bottom_offset 差 5cm=数据口径）");

    let active = asm.separate_stage();
    assert_eq!(active, 1, "active 应切到 S2");
    assert!(asm.vessels[0].detached, "S1 应被拆走");

    let v_s1 = asm.vessels[0].state.vel;
    let v_s2 = asm.state.vel;
    let dv = v_s1 - v_s2;
    println!(
        "[复现] 分离瞬间: v_S1=({:.4},{:.4},{:.4}) v_S2=({:.4},{:.4},{:.4}) Δv=({:.4},{:.4},{:.4}) |Δv|={:.4}",
        v_s1.x, v_s1.y, v_s1.z, v_s2.x, v_s2.y, v_s2.z, dv.x, dv.y, dv.z, dv.length()
    );
    assert!(dv.y < 0.0, "S1 应朝下离开（Δv.y<0）: dv.y={}", dv.y);
    // 相对分离速度 = vsep = 2.0（动量守恒分配到两侧）。
    let m_s1 = asm.vessels[0].mass();
    let m_s2 = asm.vessels[1].mass();
    let v_leave = 2.0 * m_s2 / (m_s1 + m_s2);
    println!(
        "[复现] 质量 m_S1={m_s1:.1} m_S2={m_s2:.1} 期望 v_leave={v_leave:.4} 实际|v_S1|={:.4}（|Δv| 应=2.0）",
        (v_s1 - Vec3::ZERO).length()
    );
    assert!((dv.length() - 2.0).abs() < 0.05, "相对分离速度应=vsep=2.0: {}", dv.length());
    let om_s1 = asm.vessels[0].state.omega;
    println!(
        "[复现] S1 omega=({:.4},{:.4},{:.4})（tumble=0 应无扰动）",
        om_s1.x, om_s1.y, om_s1.z
    );
    assert!(om_s1.length() < 1e-9, "级间 tumble=0 不应翻滚");

    let env = StepEnv::primary0(&[]);
    for _ in 0..100 {
        asm.step(0.02, env);
    }
    let (top, bot) = s1_top_s2_bottom_y(&asm);
    let gap = bot - top;
    println!("[复现] 步进 2s 后 S2底−S1顶 间距 = {gap:.4} m");
    assert!(gap > 0.0, "S2 底沿必须在 S1 顶沿之上（无穿模）: gap={gap}");
}

/// S1 + B1 侧挂（真实 dock 几何与质量；B1 sep=4.0）。
fn user_s1_b1() -> Vec<StageSpec> {
    let s1 = StageSpec {
        name: "S1",
        dry_mass: 4390.0,
        dry_center: Vec3::new(0.0, -5.4001903, 0.0),
        dry_inertia: Vec3::new(149371.43, 5243.58, 149372.48),
        tanks: vec![TankSpec {
            id: 0,
            max_mass: 37680.0,
            mass: 37680.0,
            pos: Vec3::new(0.0, 0.6291623, 0.0),
            inertia: Vec3::new(645286.33, 59692.66, 645286.33),
            efficiency: 1.0,
        }],
        length: 21.62396,
        radius: 2.50084,
        geometry: Some(StageGeometry {
            bottom_offset_m: 10.99479,
            top_offset_m: 10.62916,
            max_radius_m: 2.50084,
        }),
        separation_impulse: 2.0,
        cd_mach: vec![(0.0, 0.3)],
        docks: Some(vec![DockPort::with_rot(
            Vec3::new(-0.39319, -4.20705, 2.99087),
            Vec3::new(-0.13034, 0.0, 0.99147),
            Vec3::new(0.0, -0.000005, 1.0),
        )]),
        ..Default::default()
    };
    let b1 = StageSpec {
        name: "B1",
        dry_mass: 11400.0,
        dry_center: Vec3::new(0.0, 0.09879, 0.0),
        dry_inertia: Vec3::new(75880.09, 8890.90, 75880.09),
        tanks: vec![TankSpec {
            id: 0,
            max_mass: 68400.0,
            mass: 68400.0,
            pos: Vec3::new(0.0, -0.01646, 0.0),
            inertia: Vec3::new(543312.03, 53437.5, 543312.03),
            efficiency: 1.0,
        }],
        length: 16.77185,
        radius: 1.76221,
        geometry: Some(StageGeometry {
            bottom_offset_m: 6.91902,
            top_offset_m: 9.85283,
            max_radius_m: 1.76221,
        }),
        separation_impulse: 4.0,
        separation_tumble_rad_s: 0.5,
        cd_mach: vec![(0.0, 0.3)],
        docks: Some(vec![DockPort::with_rot(
            Vec3::new(0.0, -0.01646, 0.0),
            Vec3::new(-1.0, 0.0, 0.0000015),
            Vec3::new(0.99147, -0.000005, -0.13034),
        )]),
        ..Default::default()
    };
    vec![s1, b1]
}

#[test]
fn reproduce_radial_separation_s1_b1() {
    let stages = user_s1_b1();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &[(0, 0, 1, 0)]);
    asm.writeback_primary_states();
    let p0 = asm.vessels[0].state.pos;
    let b0 = asm.vessels[1].state.pos;
    println!(
        "[复现] 分离前: S1=({:.3},{:.3},{:.3}) B1=({:.3},{:.3},{:.3})",
        p0.x, p0.y, p0.z, b0.x, b0.y, b0.z
    );
    // 外向真值 = S1 dock dir（约定：指向对方 B1，即径向外侧）。
    let radial = asm.vessels[0].docks[0].dir.unit();
    println!(
        "[复现] 分离前: S1=({:.3},{:.3},{:.3}) B1=({:.3},{:.3},{:.3}) 外向单位向量=({:.4},{:.4},{:.4})",
        p0.x, p0.y, p0.z, b0.x, b0.y, b0.z, radial.x, radial.y, radial.z
    );

    let b1_id = asm.vessels[1].id;
    let leave = asm.undock(b1_id, 0, 4.0);
    assert_eq!(leave, vec![1], "B1 应被拆走");

    let v_s1 = asm.state.vel;
    let v_b1 = asm.vessels[1].state.vel;
    let dv = v_b1 - v_s1;
    let align = dot(v_b1, radial) / v_b1.length().max(1e-12);
    let m_b1 = asm.vessels[1].mass();
    let m_keep = asm.vessels[0].mass();
    let expect = 4.0 * m_keep / (m_b1 + m_keep);
    println!(
        "[复现] 分离瞬间: |v_B1|={:.4} 期望v_leave={:.4} 相对|Δv|={:.4}(应=4.0) 方向对齐度(·外向)={:.4}",
        v_b1.length(),
        expect,
        dv.length(),
        align
    );
    println!(
        "[复现] Δv=({:.4},{:.4},{:.4}) v_B1=({:.4},{:.4},{:.4})",
        dv.x, dv.y, dv.z, v_b1.x, v_b1.y, v_b1.z
    );
    assert!((v_b1.length() - expect).abs() < 0.05, "v_B1 应=动量分配 v_leave");
    assert!(align > 0.999, "B1 分离速度应沿几何外向: align={align}");

    // 姿态扰动：分离瞬间 |ω|≈tumble；鼻端世界速度比质心更朝外（外甩）。
    let om = asm.vessels[1].state.omega;
    println!(
        "[复现] B1 omega=({:.4},{:.4},{:.4}) |ω|={:.4}",
        om.x,
        om.y,
        om.z,
        om.length()
    );
    assert!(om.length() > 0.4, "应有姿态扰动 ~0.5 rad/s: {}", om.length());

    // 绕切向轴翻滚：步进 0.5s 后鼻端姿态应偏离径向（外翻可见）。
    let nose_radial = |a: &Assembly| {
        let b = &a.vessels[1];
        dot(mul(b.state.r, Vec3::new(0.0, 9.85283, 0.0)).unit(), radial)
    };
    let align0 = nose_radial(&asm);
    let env = StepEnv::primary0(&[]);
    for _ in 0..25 {
        asm.step(0.02, env);
    }
    let align1 = nose_radial(&asm);
    println!("[复现] 鼻端·径向: 分离瞬间={align0:.4} → 0.5s 后={align1:.4}（应朝外翻正增长）");
    assert!(align1 > align0 + 0.02, "鼻端姿态应朝径向外翻: {align0}→{align1}");

    let env = StepEnv::primary0(&[]);
    for _ in 0..50 {
        asm.step(0.02, env);
    }
    let d = (asm.vessels[1].state.pos - asm.vessels[0].state.pos).length();
    println!("[复现] 步进 1s 后 S1↔B1 质心距 = {d:.3} m（初始 {:.3}）", (b0 - p0).length());
}
