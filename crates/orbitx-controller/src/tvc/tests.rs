use crate::base::BaseController;
use crate::capability::ControlCapability;
use crate::throttle::ThrottlePolicy;
use orbitx_dynamics::GravBody;
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{Assembly, StepEnv, StageSpec, ThrusterSpec};

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
    assert!(tip_deg < 8.0, "竖直保持 tip 应 < 8°，实际峰值 {tip_deg:.2}°");
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
    assert!((p_deg - 30.0).abs() < 3.0, "稳态俯仰应≈30°，实际 {p_deg:.2}°");
    assert!(y.abs().to_degrees() < 5.0, "偏航应保持近 0");
    assert!((p_deg - wrong_eq).abs() > 0.5, "不应停在旧 asin 平衡点 {wrong_eq:.2}°");
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
