use super::*;
use orbitx_dynamics::PlanetOrientation;
use orbitx_math::{Matrix3, StateVectors, PI2};
use orbitx_vessel::{presets, GroundContact, GroundSampler, SurfaceKind};
use std::sync::Arc;

const SID: f64 = 86_164.0;
fn omega_y() -> Vec3 {
    Vec3::new(0.0, PI2 / SID, 0.0)
}
fn orient_y() -> PlanetOrientation {
    PlanetOrientation {
        rot: Matrix3::IDENTITY,
        omega: omega_y(),
    }
}

struct WaterSampler;
impl GroundSampler for WaterSampler {
    fn ground_radius_m(&self, _lat: f64, _lon: f64) -> f64 {
        6_371_000.0
    }
    fn surface_kind_at(&self, _lat: f64, _lon: f64) -> SurfaceKind {
        SurfaceKind::Water
    }
}

fn pad_asm(pos: Vec3, vel: Vec3) -> Assembly {
    let stages = presets::falcon9();
    let r = 6_371_000.0;
    let mut asm = Assembly::new(
        &stages,
        StateVectors {
            pos,
            vel,
            ..Default::default()
        },
    );
    asm.planet_radius = r;
    asm.sid_rot_period = SID;
    asm.planet_orientation = orient_y();
    asm.ground = GroundContact::Sphere { radius_m: r };
    asm
}

fn vel_down(pos: Vec3, down_mps: f64) -> Vec3 {
    surface_inertial_velocity(pos, omega_y()) + Vec3::new(-down_mps, 0.0, 0.0)
}

fn soft_contact(force_n: f64) -> SurfaceContact {
    SurfaceContact {
        force: Vec3::ZERO,
        torque: Vec3::ZERO,
        in_contact: true,
        max_penetration: -0.01,
        max_normal_force: force_n,
    }
}

fn hard_contact(force_n: f64, penet: f64) -> SurfaceContact {
    SurfaceContact {
        force: Vec3::ZERO,
        torque: Vec3::ZERO,
        in_contact: true,
        max_penetration: -penet,
        max_normal_force: force_n,
    }
}

#[test]
fn under_five_seconds_does_not_land() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = surface_inertial_velocity(pos, omega_y());
    let mut asm = pad_asm(pos, vel);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    for i in 0..20 {
        let t = i as f64 * 0.2; // 0..3.8 s < 5 s
        let out =
            apply_contact_checks(&mut asm, true, true, true, 0.0, &mut dwell, &mut landed, t);
        assert_eq!(out, ContactOutcome::None, "t={t}");
    }
    assert!(dwell.testing);
    assert!((3.8 - dwell.since.unwrap()) < LAND_STABLE_S);
}

#[test]
fn soft_force_high_v_imp_does_not_crash() {
    // 大撞击速度但接触力/穿透未超门限 → 不 Crash（无 v_imp 兜底）。
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = vel_down(pos, 50.0);
    let mut asm = pad_asm(pos, vel);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out = apply_contact_checks(
        &mut asm,
        true,
        true,
        true,
        0.0,
        &mut dwell,
        &mut landed,
        1.0,
    );
    assert!(!matches!(out, ContactOutcome::Crash { .. }), "got {out:?}");
}

#[test]
fn thrust_on_never_lands() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = surface_inertial_velocity(pos, omega_y());
    let mut asm = pad_asm(pos, vel);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    for i in 0..300 {
        let t = i as f64 * 0.02;
        let out =
            apply_contact_checks(&mut asm, true, true, true, 1e5, &mut dwell, &mut landed, t);
        assert_eq!(out, ContactOutcome::None, "t={t}");
    }
    assert!(!dwell.testing);
}

/// 绕 +Y 推进 `rot`，惯性 pos/姿态随共转（体固钉死量不变）。
fn set_corotating_pose(asm: &mut Assembly, pos_body: Vec3, r_body: Matrix3, ang_rad: f64) {
    let (c, s) = (ang_rad.cos(), ang_rad.sin());
    let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
    let omega = omega_y();
    asm.planet_orientation = PlanetOrientation { rot, omega };
    let pos = to_inertial(rot, pos_body);
    let vel = surface_inertial_velocity(pos, omega);
    let lock_r = rot.matmul(r_body);
    let lock_q = Quat::from_matrix(lock_r);
    asm.state.pos = pos;
    asm.state.vel = vel;
    asm.state.r = lock_r;
    asm.state.q = lock_q;
    asm.state.omega = Vec3::ZERO;
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.pos = pos;
            v.state.vel = vel;
            v.state.r = lock_r;
            v.state.q = lock_q;
            v.state.omega = Vec3::ZERO;
        }
    }
}

#[test]
fn stable_five_seconds_lands_while_corotating() {
    // 共转 + 推进 rot、不冻惯性 pos：体固相对静止 → 满 ~5 s 应 Landed。
    let r = 6_371_000.0;
    let pos_body = Vec3::new(r + 10.0, 0.0, 0.0);
    let r_body = Matrix3::IDENTITY;
    let mut asm = pad_asm(pos_body, Vec3::ZERO);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed_st = LandedState::default();
    let mut got_landed = false;
    let omega = PI2 / SID;
    for i in 0..40 {
        let t = i as f64 * 0.2;
        set_corotating_pose(&mut asm, pos_body, r_body, omega * t);
        let out = apply_contact_checks(
            &mut asm,
            true,
            true,
            true,
            0.0,
            &mut dwell,
            &mut landed_st,
            t,
        );
        if matches!(out, ContactOutcome::Landed { .. }) {
            got_landed = true;
            break;
        }
    }
    assert!(got_landed, "expected Landed after ~5s body-relative stable");
    assert!(landed_st.active, "LandedState should be active");
    let clearance = asm.bottom_clearance().unwrap();
    assert!(
        clearance.abs() < 1.0,
        "after InitLanded clearance {clearance}"
    );
}

#[test]
fn body_slide_resets_dwell() {
    // 体固平移超门限 → since 重置（相对滑动，不是惯性共转）。
    let r = 6_371_000.0;
    let mut pos_body = Vec3::new(r - 1.0, 0.0, 0.0);
    let r_body = Matrix3::IDENTITY;
    let mut asm = pad_asm(pos_body, Vec3::ZERO);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let omega = PI2 / SID;
    for i in 0..10 {
        let t = i as f64 * 0.2;
        set_corotating_pose(&mut asm, pos_body, r_body, omega * t);
        let _ =
            apply_contact_checks(&mut asm, true, true, true, 0.0, &mut dwell, &mut landed, t);
    }
    assert!(dwell.testing);
    let t0 = dwell.since.unwrap();
    pos_body.x += 10.0;
    let t_slide = 2.2;
    set_corotating_pose(&mut asm, pos_body, r_body, omega * t_slide);
    let _ = apply_contact_checks(
        &mut asm,
        true,
        true,
        true,
        0.0,
        &mut dwell,
        &mut landed,
        t_slide,
    );
    assert!(dwell.since.unwrap() >= t0 + 0.1);
    assert!(!landed.active);
}

#[test]
fn never_airborne_large_force_does_not_crash() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = vel_down(pos, 1.0);
    let mut asm = pad_asm(pos, vel);
    let mass = asm.total_mass();
    asm.last_surface_contact = hard_contact(crash_force_limit_n(mass) * 2.0, 0.8);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out = apply_contact_checks(
        &mut asm,
        true,
        false,
        true,
        0.0,
        &mut dwell,
        &mut landed,
        1.0,
    );
    assert_eq!(out, ContactOutcome::ReturnedToPad);
    assert!(!asm.vessels[asm.active].crashed);
}

#[test]
fn large_force_crashes_without_speed() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    // 共转 ≈ 零撞击速度
    let vel = surface_inertial_velocity(pos, omega_y());
    let mut asm = pad_asm(pos, vel);
    let mass = asm.total_mass();
    let huge = crash_force_limit_n(mass) * 2.0;
    asm.last_surface_contact = hard_contact(huge, 0.05);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out = apply_contact_checks(
        &mut asm,
        true,
        true,
        true,
        0.0,
        &mut dwell,
        &mut landed,
        1.0,
    );
    assert!(
        matches!(out, ContactOutcome::Crash { water: false, .. }),
        "got {out:?}"
    );
    assert!(asm.vessels[asm.active].crashed);
}

#[test]
fn deep_penetration_crashes() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = surface_inertial_velocity(pos, omega_y());
    let mut asm = pad_asm(pos, vel);
    asm.last_surface_contact = hard_contact(1.0, CRASH_MAX_PENETRATION_M + 0.1);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out = apply_contact_checks(
        &mut asm,
        true,
        true,
        true,
        0.0,
        &mut dwell,
        &mut landed,
        1.0,
    );
    assert!(matches!(out, ContactOutcome::Crash { .. }), "got {out:?}");
}

#[test]
fn never_airborne_slow_sink_returns_to_pad() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = vel_down(pos, 1.0);
    let mut asm = pad_asm(pos, vel);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out = apply_contact_checks(
        &mut asm,
        true,
        false,
        true,
        0.0,
        &mut dwell,
        &mut landed,
        1.0,
    );
    assert_eq!(out, ContactOutcome::ReturnedToPad);
}

#[test]
fn never_airborne_ascending_is_not_returned() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = vel_down(pos, -4.0);
    let mut asm = pad_asm(pos, vel);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    assert_eq!(
        apply_contact_checks(
            &mut asm,
            true,
            false,
            true,
            0.0,
            &mut dwell,
            &mut landed,
            1.0,
        ),
        ContactOutcome::None
    );
}

#[test]
fn water_touch_is_crash() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let vel = vel_down(pos, 0.5);
    let mut asm = pad_asm(pos, vel);
    asm.ground = GroundContact::query(Arc::new(WaterSampler), Matrix3::IDENTITY);
    asm.last_surface_contact = soft_contact(1e3);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out = apply_contact_checks(
        &mut asm,
        true,
        true,
        true,
        0.0,
        &mut dwell,
        &mut landed,
        1.0,
    );
    assert!(
        matches!(out, ContactOutcome::Crash { water: true, .. }),
        "got {out:?}"
    );
}

#[test]
fn above_ground_never_crashes() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r + 100.0, 0.0, 0.0);
    let vel = vel_down(pos, 200.0);
    let mut asm = pad_asm(pos, vel);
    asm.last_surface_contact = SurfaceContact::default();
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    assert_eq!(
        apply_contact_checks(
            &mut asm,
            true,
            true,
            true,
            0.0,
            &mut dwell,
            &mut landed,
            1.0,
        ),
        ContactOutcome::None
    );
}

#[test]
fn off_ground_skips() {
    let r = 6_371_000.0;
    let pos = Vec3::new(r - 1.0, 0.0, 0.0);
    let mut asm = pad_asm(pos, Vec3::new(-100.0, 0.0, 0.0));
    asm.ground = GroundContact::Off;
    asm.last_surface_contact = soft_contact(1e6);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    assert_eq!(
        apply_contact_checks(
            &mut asm,
            true,
            true,
            true,
            0.0,
            &mut dwell,
            &mut landed,
            1.0,
        ),
        ContactOutcome::None
    );
}

fn mat3_close(a: Matrix3, b: Matrix3, eps: f64) -> bool {
    (a.m11 - b.m11).abs() < eps
        && (a.m12 - b.m12).abs() < eps
        && (a.m13 - b.m13).abs() < eps
        && (a.m21 - b.m21).abs() < eps
        && (a.m22 - b.m22).abs() < eps
        && (a.m23 - b.m23).abs() < eps
        && (a.m31 - b.m31).abs() < eps
        && (a.m32 - b.m32).abs() < eps
        && (a.m33 - b.m33).abs() < eps
}

#[test]
fn init_landed_preserves_attitude_and_corotates() {
    // 非竖直落地朝向 → InitLanded 不得硬掰；推进 rot 后体固相对姿态不变。
    let r = 6_371_000.0;
    let pos = Vec3::new(r + 20.0, 0.0, 0.0);
    let vel = surface_inertial_velocity(pos, omega_y());
    let mut asm = pad_asm(pos, vel);
    // 绕 +X（当地径向）转 30°。
    let a = 30.0_f64.to_radians();
    let (ca, sa) = (a.cos(), a.sin());
    let r0 = Matrix3::new(1.0, 0.0, 0.0, 0.0, ca, -sa, 0.0, sa, ca);
    let q0 = Quat::from_matrix(r0);
    asm.state.r = r0;
    asm.state.q = q0;
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.r = r0;
            v.state.q = q0;
            v.state.pos = pos;
            v.state.vel = vel;
        }
    }

    let mut landed = LandedState::default();
    init_landed(&mut asm, &mut landed);
    assert!(landed.active);
    assert!(
        mat3_close(asm.state.r, r0, 1e-9),
        "attitude wiped at InitLanded"
    );
    let r_body0 = landed.r_body;

    let ang = 15.0_f64.to_radians();
    let (c, s) = (ang.cos(), ang.sin());
    let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
    asm.planet_orientation = PlanetOrientation {
        rot,
        omega: omega_y(),
    };
    apply_landed_nail(&mut asm, &landed);
    let expect = rot.matmul(r_body0);
    assert!(
        mat3_close(asm.state.r, expect, 1e-9),
        "corotation attitude mismatch"
    );
    assert!(mat3_close(landed.r_body, r_body0, 1e-15));
    let pos_i = asm.state.pos;
    let v_expect = surface_inertial_velocity(pos_i, omega_y());
    assert!((asm.state.vel - v_expect).length() < 1e-6);
    assert_eq!(asm.state.omega, Vec3::ZERO);
}

#[test]
fn off_ground_marks_underground_detached_crashed() {
    // 2026-11-08 误坠毁根因③回归：ground=Off（高空 Paused）期间地下
    // 独立体须即刻终局；此前 crash=0 滞留数十分钟，直到注入恢复的
    // 瞬间被 detached_is_crash 集体标毁并污染主栈判定。
    let r = 6_371_000.0;
    let mut asm = pad_asm(Vec3::new(r + 10_000_000.0, 0.0, 0.0), Vec3::ZERO);
    asm.ground = GroundContact::Off;
    asm.separate_stage();
    asm.separate_stage();
    asm.vessels[0].state.pos = Vec3::new(5.4e6, 0.0, 0.0);
    asm.vessels[1].state.pos = Vec3::new(5.9e6, 0.0, 0.0);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out =
        apply_contact_checks(&mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0);
    assert_eq!(out, ContactOutcome::None, "Off 分支不产生任务级终局");
    assert!(asm.vessels[0].crashed, "地下残骸 0 应标毁");
    assert!(asm.vessels[1].crashed, "地下残骸 1 应标毁");
    assert!(!asm.vessels[2].crashed, "高空主栈不得连带标毁");
}

#[test]
fn off_ground_keeps_orbiting_detached_flying() {
    // Off 期间只终局「坠入行星内部」的独立体；在轨/高空分离体照常飞行。
    let r = 6_371_000.0;
    let mut asm = pad_asm(Vec3::new(r + 10_000_000.0, 0.0, 0.0), Vec3::ZERO);
    asm.ground = GroundContact::Off;
    asm.separate_stage();
    asm.vessels[0].state.pos = Vec3::new(r + 500_000.0, 0.0, 0.0);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out =
        apply_contact_checks(&mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0);
    assert_eq!(out, ContactOutcome::None);
    assert!(!asm.vessels[0].crashed, "在轨分离体不得误标毁");
    assert!(!asm.vessels[1].crashed);
    assert!(!asm.vessels[2].crashed);
}

#[test]
fn active_ground_crashes_underground_detached() {
    // 注入恢复后（Sphere）地下残骸走 detached_is_crash：穿透远超门限；
    // 高空主栈不受影响（对照 2026-11-08 事故形态）。
    let r = 6_371_000.0;
    let mut asm = pad_asm(Vec3::new(r + 10_000_000.0, 0.0, 0.0), Vec3::ZERO);
    asm.separate_stage();
    asm.vessels[0].state.pos = Vec3::new(5.4e6, 0.0, 0.0);
    let mut dwell = LandDwellState::default();
    let mut landed = LandedState::default();
    let out =
        apply_contact_checks(&mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0);
    assert_eq!(out, ContactOutcome::None, "高空主栈不产生终局");
    assert!(asm.vessels[0].crashed, "地下残骸应标毁");
    assert!(!asm.vessels[2].crashed);
}
