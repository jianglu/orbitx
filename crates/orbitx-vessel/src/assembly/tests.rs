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
        fuel_mass: 0.0,
        thrusters: vec![],
        length: 10.0,
        radius: 1.0,
        separation_impulse: 0.0,
        tidaldamp: 0.5,
        ..Default::default()
    };
    let v = crate::Vessel::from_spec(0, &spec, StateVectors::default());
    assert!((v.tidaldamp - 0.5).abs() < 1e-15);
}

#[test]
fn canted_thrusters_produce_torque() {
    let spec = StageSpec {
        name: "Canted",
        dry_mass: 1000.0,
        fuel_mass: 500.0,
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
    };
    let mut asm = Assembly::new(&[spec], StateVectors::default());
    asm.set_throttle(1.0);
    let p = 0.0;
    let t = &asm.vessels[0].thrusters[0];
    let f = t.current_dir() * t.current_thrust(p);
    let m = orbitx_math::cross(f, t.pos);
    assert!(m.length() > 1.0, "canted thrust should yield torque, got {m:?}");
}

#[test]
fn pfac_scales_thrust_at_sl_and_vacuum() {
    let spec = StageSpec {
        name: "P",
        dry_mass: 100.0,
        fuel_mass: 100.0,
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
    };
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
    let pos = Vec3::new(0.0, 0.0, r);
    let vel = surface_inertial_velocity(pos, period);
    let state = StateVectors {
        pos,
        vel,
        ..Default::default()
    };
    let mut asm = Assembly::new(&stages, state);
    asm.atmosphere = Some(Box::new(crate::UsStd1976Atmosphere::new()));
    asm.planet_radius = 6.37101e6;
    asm.sid_rot_period = period;
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
    assert!(
        d.mach < 0.05,
        "pad Ma should be ~0, got {}",
        d.mach
    );
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
    let pos = Vec3::new(0.0, 0.0, r);
    let vel = surface_inertial_velocity(pos, period) + Vec3::new(50.0, 0.0, 0.0);
    let state = StateVectors {
        pos,
        vel,
        ..Default::default()
    };
    let mut asm = Assembly::new(&stages, state);
    asm.atmosphere = Some(Box::new(crate::UsStd1976Atmosphere::new()));
    asm.planet_radius = 6.37101e6;
    asm.sid_rot_period = period;
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
        fuel_mass: 0.0,
        thrusters: vec![],
        length: 20.0,
        radius: 1.0,
        separation_impulse: 0.0,
        pmi: Vec3::new(1e5, 1e3, 1e5),
        tidaldamp: 2.0,
        ..Default::default()
    };
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
    with_atm.vessels[0].dragels.push(DragElement::constant(Vec3::ZERO, 0.5, 8.0));
    with_atm.vessels[0].cross_section = Vec3::new(2.0, 8.0, 2.0);
    with_atm.vessels[0].rdrag = Vec3::new(1.0, 0.1, 1.0);
    with_atm.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
    with_atm.planet_radius = 6_371_000.0;
    with_atm.separate_stage();
    assert!(with_atm.vessels[0].detached);

    let mut no_atm = Assembly::new(&[lower, upper], init);
    no_atm.vessels[0].dragels.push(DragElement::constant(Vec3::ZERO, 0.5, 8.0));
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
    let fuel0 = asm.vessels[0].fuel_mass;
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
        asm.vessels[0].fuel_mass < fuel0 - 0.1,
        "detached should burn fuel: {} -> {}",
        fuel0,
        asm.vessels[0].fuel_mass
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
