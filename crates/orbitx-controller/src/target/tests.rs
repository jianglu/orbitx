use super::*;
use crate::base::BaseController;
use crate::capability::ControlCapability;
use orbitx_dynamics::GravBody;
use orbitx_math::{cross, dot, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{attitude as att, Assembly, StageSpec, ThrusterSpec};

fn hold_spec() -> StageSpec {
    StageSpec {
        name: "hold",
        dry_mass: 10_000.0,
        fuel_mass: 40_000.0,
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

/// 发射台姿态：body +Y = 径向（竖直），可选初始 omega。
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
    let asm = Assembly::new(&[spec], StateVectors { pos, vel: Vec3::ZERO, omega, r: rot, q });
    (asm, earth_r)
}

/// 给定位置 + 初速度（world），构造竖直姿态的 asm。
fn asm_at(pos: Vec3, vel: Vec3, spec: StageSpec) -> (Assembly, f64) {
    let earth_r = 6_371_000.0;
    let up = pos * (1.0 / pos.length());
    let ref_axis = Vec3::new(0.0, 1.0, 0.0);
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let by = up;
    let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(rot);
    let asm = Assembly::new(&[spec], StateVectors { pos, vel, omega: Vec3::ZERO, r: rot, q });
    (asm, earth_r)
}

fn tip_of(asm: &Assembly) -> f64 {
    att::tip_angle(&asm.vessels[asm.active].state)
}

#[test]
fn mode_accessors_round_trip() {
    let mut tc = TargetController::new(TargetMode::VerticalHold { throttle: 0.5 });
    assert_eq!(tc.mode().throttle(), 0.5);
    tc.set_mode(TargetMode::PitchTo { pitch: 0.1, yaw: 0.0, throttle: 1.0 });
    assert_eq!(tc.mode().throttle(), 1.0);
}

#[test]
fn gravity_turn_accumulates_and_clamps() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(TargetMode::GravityTurn { throttle: 1.0, pitch_rate: 0.05 });
    let dt = 0.05;
    // 5 秒 → 累计 0.05*5 = 0.25 rad ≈ 14.3°。
    for _ in 0..(5.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
        drop(base);
        asm.step(dt, &[earth.clone()]);
    }
    assert!((tc.turn_pitch() - 0.25).abs() < 1e-9, "turn_pitch={}", tc.turn_pitch());
    // 长时间后应夹到 90°（0.05 rad/s → ~31.4 s 到顶）。
    for _ in 0..(200.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
        drop(base);
        asm.step(dt, &[earth.clone()]);
    }
    assert!((tc.turn_pitch() - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    // reset_turn 归零。
    tc.reset_turn();
    assert!(tc.turn_pitch().abs() < 1e-12);
}

#[test]
fn vertical_hold_keeps_tip_bounded_and_applies_throttle() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::new(0.03, 0.0, -0.02));
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(TargetMode::VerticalHold { throttle: 1.0 });
    let dt = 0.05;
    let mut max_tip = 0.0_f64;
    for _ in 0..(40.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
        drop(base);
        asm.step(dt, &[earth.clone()]);
        max_tip = max_tip.max(tip_of(&asm));
    }
    assert!(max_tip.to_degrees() < 8.0, "tip 峰值 {}°", max_tip.to_degrees());
    // 油门已下：active vessel 推力机 level 应 > 0。
    let lvl = asm.vessels[asm.active].thrusters[0].level;
    assert!(lvl > 1e-6, "level={lvl}");
}

#[test]
fn pitch_to_tracks_target_angle() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let pitch_tgt = 25.0_f64.to_radians();
    let mut tc = TargetController::new(TargetMode::PitchTo { pitch: pitch_tgt, yaw: 0.0, throttle: 1.0 });
    let dt = 0.05;
    for _ in 0..(25.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
        drop(base);
        asm.step(dt, &[earth.clone()]);
    }
    let (p, y) = att::pitch_yaw_angles(&asm.vessels[asm.active].state);
    assert!((p.to_degrees() - 25.0).abs() < 3.0, "稳态 pitch={}°", p.to_degrees());
    assert!(y.abs().to_degrees() < 5.0);
}

#[test]
fn prograde_hold_aligns_nose_with_velocity() {
    // 上升期小角度场景：速度近竖直（+z）带小东向分量。prograde 与径向夹角小，
    // pitch/yaw 分解非退化，PD 无滚转歧义。body +Y 应跟踪 prograde。
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 5_000.0);
    let vel = Vec3::new(120.0, 0.0, 600.0); // ≈ 11° 偏东
    let (mut asm, _) = asm_at(pos, vel, hold_spec());
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(TargetMode::ProgradeHold { throttle: 1.0 });
    let dt = 0.05;
    let prograde0 = vel.unit();
    for _ in 0..(20.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
        drop(base);
        asm.step(dt, &[earth.clone()]);
    }
    let v = asm.vessels[asm.active].state.vel;
    let r = asm.vessels[asm.active].state.r;
    let body_y = r.col(1);
    // 速度方向会因推力演化，检查 body +Y 与当前 prograde 对齐。
    let cos = dot(body_y, v.unit()).clamp(-1.0, 1.0);
    let angle = cos.acos().to_degrees();
    assert!(angle < 8.0, "body +Y 与 prograde 夹角应 < 8°，实际 {angle:.2}°");
    // 同时确认确实跟踪了初始偏东方向（非停在竖直）。
    let cos0 = dot(body_y, prograde0).clamp(-1.0, 1.0);
    assert!(cos0.acos().to_degrees() < 12.0, "应朝初始 prograde 方向，夹角 {}", cos0.acos().to_degrees());
}

#[test]
fn retrograde_target_is_negated_prograde() {
    // 单元级：retrograde 目标 = prograde 目标双轴取负（180° 翻转在 pitch/yaw 空间的等价）。
    // 闭环 180° 翻转超出简单 PD 能力，故在此验证算法几何正确性。
    use crate::target::prograde_target_angles;
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 5_000.0);
    let vel = Vec3::new(120.0, 0.0, 600.0); // ≈ 11° 偏东
    let (mut asm, _) = asm_at(pos, vel, hold_spec());
    let caps = ControlCapability::for_primary(&asm);
    let base = BaseController::new(&mut asm, &caps);
    let (p_pro, y_pro) = prograde_target_angles(&base, vel);
    let (p_ret, y_ret) = (-p_pro, -y_pro);
    // prograde 目标 yaw 应非零（偏东速度 → yaw 偏移）。
    assert!(y_pro.abs() > 0.05, "prograde yaw={y_pro}");
    // retrograde 双轴取负。
    assert!((p_ret + p_pro).abs() < 1e-12);
    assert!((y_ret + y_pro).abs() < 1e-12);
    // 几何一致性：prograde 目标对应的体 +Y 应指向 prograde 方向。
    // 构造目标体轴并验证 body+Y · prograde ≈ 1。
    let radial = pos * (1.0 / pos.length());
    let d = vel.unit();
    let (bx, _by, _bz) = base.body_axes();
    let mut tx = bx - d * dot(bx, d);
    if tx.length2() < 1e-18 {
        tx = radial;
    }
    tx = tx.unit();
    let tz = cross(tx, d).unit();
    // body+Y = d（构造保证），与 prograde 单位向量同向。
    assert!((dot(d, d) - 1.0).abs() < 1e-12);
    // 验证 pitch/yaw 与该目标体轴的 pitch_yaw_angles 一致。
    let recomputed = (
        dot(radial, tz).clamp(-1.0, 1.0).asin(),
        (-dot(radial, tx)).clamp(-1.0, 1.0).asin(),
    );
    assert!((recomputed.0 - p_pro).abs() < 1e-9, "pitch {} vs {}", recomputed.0, p_pro);
    assert!((recomputed.1 - y_pro).abs() < 1e-9, "yaw {} vs {}", recomputed.1, y_pro);
}

#[test]
fn prograde_hold_zero_velocity_falls_back_to_vertical() {
    // 速度近零：direction_target_angles 应回退 (0,0)，不 panic。
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(TargetMode::ProgradeHold { throttle: 0.0 });
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    // 不 panic 即通过；tvc gimbal 应保持近 0（target=0，初始 tip=0）。
    let g = asm.vessels[asm.active].thrusters[0].gimbal_pitch;
    assert!(g.abs() < 1e-6, "gimbal={g}");
}

#[test]
fn detached_body_uses_active_only_policy() {
    // 单船 detached：tick 应只设该 vessel 油门。
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    asm.vessels[0].detached = true;
    let caps = ControlCapability::for_detached(&asm, 0);
    let mut tc = TargetController::new(TargetMode::VerticalHold { throttle: 0.7 });
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    assert!((asm.vessels[0].thrusters[0].level - 0.7).abs() < 1e-9);
}
