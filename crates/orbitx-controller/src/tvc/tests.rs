use crate::base::BaseController;
use crate::capability::ControlCapability;
use crate::throttle::ThrottlePolicy;
use orbitx_dynamics::GravBody;
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{Assembly, StageSpec, StepEnv, ThrusterSpec};

fn hold_spec() -> StageSpec {
    StageSpec {
        name: "hold",
        dry_mass: 10_000.0,
        thrusters: vec![ThrusterSpec {
            pos: Vec3::new(0.0, -15.0, 0.0),
            dir: Vec3::new(0.0, 1.0, 0.0),
            thrust: 800_000.0,
            isp: 300.0,
            max_gimbal: 0.15,
            max_gimbal_rate: 1.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        }],
        length: 30.0,
        radius: 1.5,
        ..Default::default()
    }
    .with_fuel(40_000.0)
}

fn earth() -> GravBody {
    let earth_r = 6_371_000.0;
    GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: earth_r,
        jcoeff: vec![],
        rotation: None,
        pines: None,
    }
}

fn launch_asm(spec: StageSpec, omega: Vec3) -> (Assembly, f64) {
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 20.0);
    let up = pos * (1.0 / pos.length());
    let ref_axis = Vec3::new(0.0, 1.0, 0.0);
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let by = up;
    let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(rot);
    let asm = Assembly::new(
        &[spec],
        StateVectors {
            pos,
            vel: Vec3::ZERO,
            omega,
            r: rot,
            q,
        },
    );
    (asm, earth_r)
}

/// 无操作竖直上升：有符号双轴 TVC 保持 tip 有界。
#[test]
fn vertical_hold_tip_stays_bounded() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::new(0.03, 0.0, -0.02));
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let dt = 0.05;
    let mut max_tip = 0.0_f64;
    for _ in 0..(40.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        base.set_throttle(ThrottlePolicy::SyncPrimary, 1.0);
        base.apply_tvc("hold-tvc", 0.0, 0.0, dt);
        drop(base);
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
        max_tip = max_tip.max(tip_of(&asm));
    }
    let tip_deg = max_tip.to_degrees();
    assert!(
        tip_deg < 8.0,
        "竖直保持 tip 应 < 8°，实际峰值 {tip_deg:.2}°"
    );
    let h = asm.vessels[asm.active].state.pos.length() - earth_r;
    assert!(h > 100.0, "应明显离地，高度={h:.1} m");
}

/// 非零俯仰目标：稳态 pitch 角应逼近目标，而非 asin(目标弧度)。
#[test]
fn pitch_target_tracks_angle_not_sin() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let pitch_tgt = 30.0_f64.to_radians();
    let dt = 0.05;
    for _ in 0..(25.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        base.set_throttle(ThrottlePolicy::SyncPrimary, 1.0);
        base.apply_tvc("hold-tvc", pitch_tgt, 0.0, dt);
        drop(base);
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
    }
    let (p, y) = orbitx_vessel::attitude::pitch_yaw_angles(&asm.vessels[asm.active].state);
    let p_deg = p.to_degrees();
    let wrong_eq = pitch_tgt.asin().to_degrees();
    assert!(
        (p_deg - 30.0).abs() < 3.0,
        "稳态俯仰应≈30°，实际 {p_deg:.2}°"
    );
    assert!(y.abs().to_degrees() < 5.0, "偏航应保持近 0");
    assert!(
        (p_deg - wrong_eq).abs() > 0.5,
        "不应停在旧 asin 平衡点 {wrong_eq:.2}°"
    );
}

#[test]
fn apply_tvc_unknown_group_id_no_op() {
    let (mut asm, _r) = launch_asm(hold_spec(), Vec3::ZERO);
    let caps = ControlCapability::for_primary(&asm);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        // 未命中 group id：不应 panic，无副作用。
        base.apply_tvc("no-such-tvc", 0.1, 0.0, 0.05);
    }
    for t in &asm.vessels[0].thrusters {
        assert!(t.gimbal_pitch.abs() < 1e-9);
    }
}

fn tip_of(asm: &Assembly) -> f64 {
    orbitx_vessel::attitude::tip_angle(&asm.vessels[asm.active].state)
}

// ── 堆叠语义（apply_tvc_stack）回归 ──────────────────────────────
// 用户裁定 2026-10-29：TVC 控制当前级 + 侧挂的全部引擎（不按矢量/固定
// 预过滤）；固定引擎在册但不偏转；分离后范围自动收缩到新主级。

use crate::capability::SeparationKind;
use orbitx_vessel::DockPort;

/// 芯级：固定 + 矢量双推进器（同位异台），顶部同轴口 + 侧向口。
fn stack_core_spec() -> StageSpec {
    StageSpec {
        name: "Core",
        dry_mass: 1000.0,
        thrusters: vec![
            ThrusterSpec {
                pos: Vec3::new(0.0, -10.0, 0.0),
                dir: Vec3::new(0.0, 1.0, 0.0),
                thrust: 500_000.0,
                isp: 300.0,
                max_gimbal: 0.0, // 固定
                ..Default::default()
            },
            ThrusterSpec {
                pos: Vec3::new(0.0, -10.0, 0.0),
                dir: Vec3::new(0.0, 1.0, 0.0),
                thrust: 500_000.0,
                isp: 300.0,
                max_gimbal: 0.2,
                max_gimbal_rate: 1.0,
                gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
                ..Default::default()
            },
        ],
        length: 20.0,
        radius: 1.5,
        docks: Some(vec![
            DockPort::with_rot(
                Vec3::new(0.0, 10.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
        ]),
        ..Default::default()
    }
    .with_fuel(20_000.0)
}

/// 矢量侧挂助推（径向口对接）。
fn strap_vector_spec() -> StageSpec {
    StageSpec {
        name: "BoosterV",
        dry_mass: 500.0,
        thrusters: vec![ThrusterSpec {
            pos: Vec3::new(0.0, -4.0, 0.0),
            dir: Vec3::new(0.0, 1.0, 0.0),
            thrust: 300_000.0,
            isp: 300.0,
            max_gimbal: 0.2,
            max_gimbal_rate: 1.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        }],
        length: 8.0,
        radius: 1.0,
        docks: Some(vec![DockPort::with_rot(
            Vec3::new(-0.5, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )]),
        ..Default::default()
    }
    .with_fuel(5_000.0)
}

/// 矢量二级（同轴堆叠在上）。
fn upper_vector_spec() -> StageSpec {
    StageSpec {
        name: "S2",
        dry_mass: 800.0,
        thrusters: vec![ThrusterSpec {
            pos: Vec3::new(0.0, -6.0, 0.0),
            dir: Vec3::new(0.0, 1.0, 0.0),
            thrust: 400_000.0,
            isp: 320.0,
            max_gimbal: 0.2,
            max_gimbal_rate: 1.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        }],
        length: 12.0,
        radius: 1.5,
        docks: Some(vec![DockPort::with_rot(
            Vec3::new(0.0, -6.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )]),
        ..Default::default()
    }
    .with_fuel(8_000.0)
}

#[test]
fn tvc_stack_covers_active_and_strap_on() {
    // 分离前：lit 集（active ∪ 侧挂叶）内全部矢量引擎 slew；固定引擎在册
    // 但恒 0。同轴二级（S2，非 active 非侧挂）不在范围内。
    let core = stack_core_spec();
    let booster = strap_vector_spec();
    let upper = upper_vector_spec();
    let mut asm = Assembly::with_dock_links(
        &[core, booster, upper],
        StateVectors::default(),
        &[(0, 1, 1, 0), (0, 0, 2, 0)],
    );
    asm.vessels[0].set_throttle(1.0);
    asm.vessels[1].set_throttle(1.0);
    let caps = ControlCapability::for_primary(&asm);

    for _ in 0..3 {
        super::apply_tvc_stack(&mut asm, &caps, 0.2, 0.0, 0.05);
    }

    let core_t = &asm.vessels[0].thrusters;
    assert!(
        core_t[0].gimbal_pitch == 0.0 && core_t[0].gimbal_yaw == 0.0,
        "固定引擎在册但不偏转: {:?}",
        core_t[0].gimbal_pitch
    );
    assert!(
        core_t[1].gimbal_pitch.abs() > 0.01,
        "芯级矢量引擎应被 slew: {:?}",
        core_t[1].gimbal_pitch
    );
    let boost_t = &asm.vessels[1].thrusters[0];
    assert!(
        boost_t.gimbal_pitch.abs() > 0.01,
        "侧挂矢量助推应被 slew（堆叠语义）: {:?}",
        boost_t.gimbal_pitch
    );
    let upper_t = &asm.vessels[2].thrusters[0];
    assert!(
        upper_t.gimbal_pitch == 0.0,
        "同轴二级非 active 非侧挂，不在 TVC 范围: {:?}",
        upper_t.gimbal_pitch
    );
}

#[test]
fn tvc_stack_follows_active_after_separation() {
    // 分离后：范围自动收缩到新主级——S1（已分离）不再被写，S2 被 slew。
    let s1 = stack_core_spec();
    let s2 = upper_vector_spec();
    let mut asm = Assembly::with_dock_links(
        &[s1, s2],
        StateVectors::default(),
        &[(0, 0, 1, 0)],
    );
    asm.vessels[0].set_throttle(1.0);
    let caps = ControlCapability::for_primary(&asm);

    for _ in 0..3 {
        super::apply_tvc_stack(&mut asm, &caps, 0.2, 0.0, 0.05);
    }
    let s1_gimbal_before = asm.vessels[0].thrusters[1].gimbal_pitch;
    assert!(s1_gimbal_before.abs() > 0.01, "分离前 S1 矢量应被 slew");

    // 同轴分离：底级 S1 脱离，S2 成为新主级。
    let sep = crate::separation::perform_separate(&mut asm, &caps, "stage-sep");
    assert!(!sep.is_empty(), "同轴分离应产出 detached");
    assert_eq!(asm.active, 1, "分离后 active 应切到 S2");

    asm.vessels[1].set_throttle(1.0);
    for _ in 0..10 {
        super::apply_tvc_stack(&mut asm, &caps, 0.2, 0.0, 0.05);
    }

    assert!(
        (asm.vessels[0].thrusters[1].gimbal_pitch - s1_gimbal_before).abs() < 1e-12,
        "已分离 S1 的 gimbal 不得再被写"
    );
    // S2 单级 I/auth 比小 → 稳态 cmd 小（~5e-3 rad，物理正确：α_des×I/authority）。
    // 断言：非零 + 符号与 err 同向（err=−0.2 → cmd 负）+ 续写收敛稳定。
    let s2_g = asm.vessels[1].thrusters[0].gimbal_pitch;
    assert!(
        s2_g.abs() > 1e-4,
        "新主级 S2 的矢量引擎应被 slew: {s2_g}"
    );
    assert!(s2_g < 0.0, "符号应与 err_p 同向: {s2_g}");
    super::apply_tvc_stack(&mut asm, &caps, 0.2, 0.0, 0.05);
    assert!(
        (asm.vessels[1].thrusters[0].gimbal_pitch - s2_g).abs() < 1e-12,
        "稳态收敛后续写应稳定"
    );
}

// SeparationKind 仅用于类型引用守卫（防止误删枚举导出）。
const _: fn() -> SeparationKind = || SeparationKind::CoaxialStage;
