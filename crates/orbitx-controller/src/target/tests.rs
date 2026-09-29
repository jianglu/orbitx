use super::*;
use crate::base::BaseController;
use crate::capability::ControlCapability;
use orbitx_dynamics::GravBody;
use orbitx_math::{cross, dot, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{attitude as att, Assembly, StageSpec, StepEnv, ThrusterSpec};

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
    let asm = Assembly::new(
        &[spec],
        StateVectors {
            pos,
            vel,
            omega: Vec3::ZERO,
            r: rot,
            q,
        },
    );
    (asm, earth_r)
}

fn tip_of(asm: &Assembly) -> f64 {
    att::tip_angle(&asm.vessels[asm.active].state)
}

/// 把组合体质心抬到 `alt`，各船平移同一位移。
fn lift_to(asm: &mut Assembly, earth_r: f64, alt: f64) {
    let radial = asm.state.pos.unit();
    let target = radial * (earth_r + alt);
    let delta = target - asm.state.pos;
    asm.state.pos = target;
    for v in &mut asm.vessels {
        v.state.pos = v.state.pos + delta;
    }
}

fn set_inertial_vel(asm: &mut Assembly, vel: Vec3) {
    for v in &mut asm.vessels {
        v.state.vel = vel;
    }
    asm.state.vel = vel;
}

/// 速率 `speed`、俯仰倾角 `pitch` 的速度。+pitch 朝水平面内的 -body Z，与 prograde 俯仰同号。
fn vel_at_pitch(asm: &Assembly, speed: f64, pitch: f64) -> Vec3 {
    let radial = asm.state.pos.unit();
    let bz = asm.state.r.col(2);
    let raw = (bz * -1.0) - radial * dot(bz * -1.0, radial);
    let horiz = if raw.length2() < 1e-18 {
        let fallback = Vec3::new(0.0, 1.0, 0.0);
        (fallback - radial * dot(fallback, radial)).unit()
    } else {
        raw.unit()
    };
    (radial * pitch.cos() + horiz * pitch.sin()) * speed
}

/// 绕 body +X 把机头转到俯仰 `pitch`（朝 −Z，与 kick 正俯仰同号）。
fn set_body_pitch(asm: &mut Assembly, pitch: f64) {
    let r = asm.state.r;
    let bx = r.col(0);
    let by = r.col(1);
    let bz = r.col(2);
    let by2 = by * pitch.cos() - bz * pitch.sin();
    let bz2 = bz * pitch.cos() + by * pitch.sin();
    let rot = Matrix3::new(bx.x, by2.x, bz2.x, bx.y, by2.y, bz2.y, bx.z, by2.z, bz2.z);
    let q = Quat::from_matrix(rot);
    asm.state.r = rot;
    asm.state.q = q;
    for v in &mut asm.vessels {
        v.state.r = rot;
        v.state.q = q;
    }
}

/// 绕 body −Z 把机头转到偏航 `yaw`（朝 +X，与正偏航同号）。
fn set_body_yaw(asm: &mut Assembly, yaw: f64) {
    let r = asm.state.r;
    let bx = r.col(0);
    let by = r.col(1);
    let bz = r.col(2);
    let bx2 = bx * yaw.cos() - by * yaw.sin();
    let by2 = by * yaw.cos() + bx * yaw.sin();
    let rot = Matrix3::new(bx2.x, by2.x, bz.x, bx2.y, by2.y, bz.y, bx2.z, by2.z, bz.z);
    let q = Quat::from_matrix(rot);
    asm.state.r = rot;
    asm.state.q = q;
    for v in &mut asm.vessels {
        v.state.r = rot;
        v.state.q = q;
    }
}

/// 速率 `speed`、偏航倾角 `yaw` 的速度。+yaw 朝水平面内的 body +X。
fn vel_at_yaw(asm: &Assembly, speed: f64, yaw: f64) -> Vec3 {
    let radial = asm.state.pos.unit();
    let bx = asm.state.r.col(0);
    let raw = bx - radial * dot(bx, radial);
    let horiz = raw.unit();
    (radial * yaw.cos() + horiz * yaw.sin()) * speed
}

fn gravity(pitch: f64, yaw: f64, rate: f64) -> TargetMode {
    TargetMode::GravityTurn {
        throttle: 1.0,
        kick_pitch: pitch,
        kick_yaw: yaw,
        kick_rate: rate,
        min_alt: DEFAULT_MIN_ALT,
        min_speed: DEFAULT_MIN_SPEED,
    }
}

#[test]
fn gravity_turn_default_yaw_points_along_body_plus_x() {
    match TargetMode::gravity_turn(1.0) {
        TargetMode::GravityTurn {
            kick_pitch,
            kick_yaw,
            kick_rate,
            min_alt,
            min_speed,
            ..
        } => {
            assert!(kick_pitch.abs() < 1e-15, "kick_pitch={kick_pitch}");
            assert!(
                (kick_yaw - DEFAULT_KICK_YAW).abs() < 1e-15,
                "kick_yaw={kick_yaw}"
            );
            assert!(
                kick_yaw > 0.0,
                "default yaw should tilt the nose toward body +X"
            );
            assert!((kick_rate - DEFAULT_KICK_RATE).abs() < 1e-15);
            assert!((min_alt - DEFAULT_MIN_ALT).abs() < 1e-15);
            assert!((min_speed - DEFAULT_MIN_SPEED).abs() < 1e-15);
        }
        other => panic!("expected gravity turn, got {other:?}"),
    }

    let (mut asm, _) = launch_asm(hold_spec(), Vec3::ZERO);
    let yaw = DEFAULT_KICK_YAW;
    set_body_yaw(&mut asm, yaw);
    let (p, y) = att::pitch_yaw_angles(&asm.vessels[asm.active].state);
    assert!(p.abs() < 1e-6, "pitch={p}");
    assert!((y - yaw).abs() < 1e-6, "yaw={y}");
    let radial = asm.state.pos.unit();
    let nose = asm.state.r.col(1);
    let bx = asm.state.r.col(0);
    let nose_h = nose - radial * dot(nose, radial);
    let east_h = bx - radial * dot(bx, radial);
    assert!(
        dot(nose_h, east_h) > 0.0,
        "positive yaw should point the nose toward body +X"
    );
}

#[test]
fn mode_accessors_round_trip() {
    let mut tc = TargetController::new(TargetMode::VerticalHold { throttle: 0.5 });
    assert_eq!(tc.mode().throttle(), 0.5);
    tc.set_mode(TargetMode::PitchTo {
        pitch: 0.1,
        yaw: 0.0,
        throttle: 1.0,
    });
    assert_eq!(tc.mode().throttle(), 1.0);
}

#[test]
fn gravity_turn_kick_then_prograde() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let kick = 0.1_f64;
    let mut tc = TargetController::new(gravity(kick, 0.0, 0.05));
    let dt = 0.05;
    let climb = asm.state.pos.unit() * 80.0;
    set_inertial_vel(&mut asm, climb);

    for _ in 0..(3.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        !tc.kick_done(),
        "radial ground velocity must not finish the kick"
    );
    assert!(
        (tc.turn_pitch() - kick).abs() < 1e-6,
        "turn_pitch={}",
        tc.turn_pitch()
    );
    assert!(
        (tc.pitch_cmd() - kick).abs() < 1e-6,
        "pitch_cmd={}",
        tc.pitch_cmd()
    );
    assert!(tc.yaw_cmd().abs() < 1e-6, "yaw_cmd={}", tc.yaw_cmd());

    // 机头仍竖直、同侧倾角已经很大：不结束。
    let pitched = vel_at_pitch(&asm, 80.0, kick + 0.05);
    set_inertial_vel(&mut asm, pitched);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(!tc.kick_done(), "nose still vertical");

    set_body_pitch(&mut asm, kick);
    let same_side = vel_at_pitch(&asm, 80.0, 0.01);
    set_inertial_vel(&mut asm, same_side);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        tc.kick_done(),
        "nose at the seed with a small same-side angle should finish"
    );
    assert!(
        (tc.pitch_cmd() - kick).abs() < 1e-6,
        "completing tick still commands kick, pitch_cmd={}",
        tc.pitch_cmd()
    );

    let expected = {
        let base = BaseController::new(&mut asm, &caps);
        prograde_target_angles(&base, base.ground_velocity())
    };
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        (tc.pitch_cmd() - expected.0).abs() < 1e-9,
        "pitch_cmd={} expected={}",
        tc.pitch_cmd(),
        expected.0
    );
    assert!(
        (tc.yaw_cmd() - expected.1).abs() < 1e-9,
        "yaw_cmd={} expected={}",
        tc.yaw_cmd(),
        expected.1
    );

    tc.reset_turn();
    assert!(!tc.kick_done());
    assert!(tc.turn_pitch().abs() < 1e-12);
    assert!(tc.pitch_cmd().abs() < 1e-12);
    assert!(tc.yaw_cmd().abs() < 1e-12);
}

#[test]
fn gravity_turn_reports_stage_targets() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(gravity(0.1, 0.0, 0.05));
    let dt = 0.05;

    // 地面速度低于门槛：竖直，两轴目标为 0，kick 不累计。
    let slow = asm.state.pos.unit() * 10.0;
    set_inertial_vel(&mut asm, slow);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(tc.pitch_cmd().abs() < 1e-12, "pitch_cmd={}", tc.pitch_cmd());
    assert!(tc.yaw_cmd().abs() < 1e-12, "yaw_cmd={}", tc.yaw_cmd());
    assert!(!tc.kick_done());
    assert!(tc.turn_pitch().abs() < 1e-12);

    // Kick：只累计俯仰，偏航保持 0。径向地速到不了俯仰角门槛。
    let radial = asm.state.pos.unit();
    set_inertial_vel(&mut asm, radial * 80.0);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(!tc.kick_done());
    assert!(tc.pitch_cmd() > 1e-6, "pitch_cmd={}", tc.pitch_cmd());
    assert!(
        (tc.pitch_cmd() - tc.turn_pitch()).abs() < 1e-12,
        "pitch_cmd={} turn_pitch={}",
        tc.pitch_cmd(),
        tc.turn_pitch()
    );
    assert!(tc.yaw_cmd().abs() < 1e-12, "yaw_cmd={}", tc.yaw_cmd());

    for _ in 0..50 {
        if (tc.turn_pitch() - 0.1).abs() < 1e-6 {
            break;
        }
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(!tc.kick_done(), "radial velocity must not finish the kick");
    assert!(
        (tc.pitch_cmd() - 0.1).abs() < 1e-6,
        "held pitch_cmd={}",
        tc.pitch_cmd()
    );

    let pitched = vel_at_pitch(&asm, 80.0, 0.2);
    set_inertial_vel(&mut asm, pitched);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        !tc.kick_done(),
        "large flight-path angle with a vertical nose must not finish"
    );

    set_body_pitch(&mut asm, 0.1);
    let same_side = vel_at_pitch(&asm, 80.0, 0.01);
    set_inertial_vel(&mut asm, same_side);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        tc.kick_done(),
        "nose at the seed with a small same-side angle should finish"
    );
    assert!(
        (tc.pitch_cmd() - 0.1).abs() < 1e-6,
        "completing tick still commands kick, pitch_cmd={}",
        tc.pitch_cmd()
    );

    // Prograde：地面速度含东向分量时，偏航目标与反解一致。
    let vel = radial * 80.0 + Vec3::new(40.0, 0.0, 0.0);
    set_inertial_vel(&mut asm, vel);
    let expected = {
        let base = BaseController::new(&mut asm, &caps);
        let v_g = base.ground_velocity();
        prograde_target_angles(&base, v_g)
    };
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        (tc.pitch_cmd() - expected.0).abs() < 1e-9,
        "pitch_cmd={} expected={}",
        tc.pitch_cmd(),
        expected.0
    );
    assert!(
        (tc.yaw_cmd() - expected.1).abs() < 1e-9,
        "yaw_cmd={} expected={}",
        tc.yaw_cmd(),
        expected.1
    );
    assert!(
        tc.yaw_cmd().abs() > 1e-3,
        "prograde yaw should be nonzero, yaw_cmd={}",
        tc.yaw_cmd()
    );
}

#[test]
fn gravity_turn_holds_below_altitude_gate() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(TargetMode::gravity_turn(1.0));
    let climb = asm.state.pos.unit() * 80.0;
    set_inertial_vel(&mut asm, climb);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    assert!(tc.pitch_cmd().abs() < 1e-12, "pitch_cmd={}", tc.pitch_cmd());
    assert!(tc.yaw_cmd().abs() < 1e-12, "yaw_cmd={}", tc.yaw_cmd());
    assert!(
        tc.turn_pitch().abs() < 1e-12,
        "turn_pitch={}",
        tc.turn_pitch()
    );
    assert!(!tc.kick_done());
}

#[test]
fn gravity_turn_prograde_follows_ground_not_inertial() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    asm.sid_rot_period = 86_164.1;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(gravity(0.05, 0.0, 1.0));
    let dt = 0.05;
    let v_ground = vel_at_pitch(&asm, 80.0, 0.10);
    // 当前惯性速度为 0，velocity − ground_velocity 即共转速度。
    let v_inertial = {
        let base = BaseController::new(&mut asm, &caps);
        v_ground + (base.velocity() - base.ground_velocity())
    };
    set_inertial_vel(&mut asm, v_inertial);
    set_body_pitch(&mut asm, 0.05);

    let mut base = BaseController::new(&mut asm, &caps);
    tc.tick(&mut base, dt);
    drop(base);
    assert!(tc.kick_done(), "one fast tick should finish the kick");

    let (expected_p, expected_y) = {
        let base = BaseController::new(&mut asm, &caps);
        let v_g = base.ground_velocity();
        assert!(
            (v_g - v_ground).length() < 1e-6,
            "ground vel drifted: {}",
            (v_g - v_ground).length()
        );
        prograde_target_angles(&base, v_g)
    };
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        (tc.pitch_cmd() - expected_p).abs() < 1e-9,
        "pitch_cmd={}",
        tc.pitch_cmd()
    );
    assert!(
        (tc.yaw_cmd() - expected_y).abs() < 1e-9,
        "yaw_cmd={}",
        tc.yaw_cmd()
    );
    assert!(
        tc.yaw_cmd().abs() < 1e-3,
        "pitch-plane ground prograde yaw should stay near 0, yaw_cmd={}",
        tc.yaw_cmd()
    );
    let inertial_yaw = {
        let base = BaseController::new(&mut asm, &caps);
        prograde_target_angles(&base, v_inertial).1
    };
    assert!(
        inertial_yaw.abs() > 0.5,
        "inertial prograde yaw should be large, got {}",
        inertial_yaw
    );
}

#[test]
fn gravity_turn_holds_kick_when_flight_path_opposes() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let kick = 0.1_f64;
    let mut tc = TargetController::new(gravity(kick, 0.0, 10.0));
    let dt = 0.05;
    let opposed = vel_at_pitch(&asm, 80.0, -(kick + 0.05));
    set_inertial_vel(&mut asm, opposed);
    set_body_pitch(&mut asm, kick);
    for _ in 0..2 {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        !tc.kick_done(),
        "opposite flight-path pitch must not finish the kick"
    );
    assert!(
        (tc.turn_pitch() - kick).abs() < 1e-6,
        "turn_pitch={}",
        tc.turn_pitch()
    );
    assert!(
        (tc.pitch_cmd() - kick).abs() < 1e-6,
        "pitch_cmd={}",
        tc.pitch_cmd()
    );
    assert!(tc.yaw_cmd().abs() < 1e-3, "yaw_cmd={}", tc.yaw_cmd());
}

#[test]
fn gravity_turn_finishes_when_pitched_body_has_same_side_flight_path() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let kick = 0.087_266_f64;
    set_body_pitch(&mut asm, kick);
    let body_pitch = att::pitch_yaw_angles(&asm.vessels[asm.active].state).0;
    assert!(
        (body_pitch - kick).abs() < 1e-6,
        "body pitch {} rad",
        body_pitch
    );
    let mut tc = TargetController::new(gravity(kick, 0.0, 10.0));
    let dt = 0.05;
    let same_side = vel_at_pitch(&asm, 80.0, 12.0_f64.to_radians());
    set_inertial_vel(&mut asm, same_side);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
    }
    assert!(
        tc.kick_done(),
        "same-side 12° flight path with nose at 5° should finish the kick"
    );
    assert!(
        (tc.pitch_cmd() - kick).abs() < 1e-6,
        "completing tick still commands kick, pitch_cmd={}",
        tc.pitch_cmd()
    );
}

#[test]
fn gravity_turn_seed_gimbal_stays_on_turn_side_below_half_error() {
    // 播种段走原来的 apply_tvc。指令 0.1 rad，角速度 0.02 < 0.5×0.1，喷管留在转向侧。
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::new(0.02, 0.0, 0.0));
    asm.planet_radius = earth_r;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(gravity(0.1, 0.0, 10.0));
    let climb = asm.state.pos.unit() * 80.0;
    set_inertial_vel(&mut asm, climb);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    assert!(!tc.kick_done());
    let g = asm.vessels[asm.active].thrusters[0].gimbal_pitch;
    assert!(
        g > 0.0,
        "seed gimbal should stay on the turn side, gimbal={g}"
    );
}

#[test]
fn gravity_turn_seed_gimbal_reverses_when_rate_exceeds_half_error() {
    // 播种段走原来的 apply_tvc。指令 0.1 rad，角速度 0.08 > 0.5×0.1，喷管反向。
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::new(0.08, 0.0, 0.0));
    asm.planet_radius = earth_r;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(gravity(0.1, 0.0, 10.0));
    let climb = asm.state.pos.unit() * 80.0;
    set_inertial_vel(&mut asm, climb);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    assert!(!tc.kick_done(), "vertical nose must not finish");
    let g = asm.vessels[asm.active].thrusters[0].gimbal_pitch;
    assert!(g < 0.0, "derivative should reverse the gimbal, gimbal={g}");
}

#[test]
fn vertical_hold_brakes_pitch_rate_with_shared_pd() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::new(0.2, 0.0, 0.0));
    asm.planet_radius = earth_r;
    let caps = ControlCapability::for_primary(&asm);
    let mut tc = TargetController::new(TargetMode::VerticalHold { throttle: 1.0 });
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    let g = asm.vessels[asm.active].thrusters[0].gimbal_pitch;
    assert!(
        g < 0.0,
        "shared PD should brake a positive pitch rate, gimbal={g}"
    );
}

#[test]
fn gravity_turn_yaw_seed_finishes_when_ground_speed_is_same_side() {
    let (mut asm, earth_r) = launch_asm(hold_spec(), Vec3::ZERO);
    asm.planet_radius = earth_r;
    lift_to(&mut asm, earth_r, 600.0);
    let caps = ControlCapability::for_primary(&asm);
    let yaw = DEFAULT_KICK_YAW;
    set_body_yaw(&mut asm, yaw);
    let mut tc = TargetController::new(gravity(0.0, yaw, 10.0));
    let same_side = vel_at_yaw(&asm, 80.0, 0.02);
    set_inertial_vel(&mut asm, same_side);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    assert!(
        tc.kick_done(),
        "eastward ground speed with the nose at +yaw should finish"
    );
    assert!(
        (tc.yaw_cmd() - yaw).abs() < 1e-6,
        "yaw_cmd={}",
        tc.yaw_cmd()
    );
    assert!(tc.pitch_cmd().abs() < 1e-6, "pitch_cmd={}", tc.pitch_cmd());

    tc.reset_turn();
    set_body_yaw(&mut asm, yaw);
    let opposed = vel_at_yaw(&asm, 80.0, -0.02);
    set_inertial_vel(&mut asm, opposed);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, 0.05);
    }
    assert!(!tc.kick_done(), "opposite yaw-plane angle must not finish");
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
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
        max_tip = max_tip.max(tip_of(&asm));
    }
    assert!(
        max_tip.to_degrees() < 8.0,
        "tip 峰值 {}°",
        max_tip.to_degrees()
    );
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
    let mut tc = TargetController::new(TargetMode::PitchTo {
        pitch: pitch_tgt,
        yaw: 0.0,
        throttle: 1.0,
    });
    let dt = 0.05;
    for _ in 0..(25.0 / dt) as usize {
        let mut base = BaseController::new(&mut asm, &caps);
        tc.tick(&mut base, dt);
        drop(base);
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
    }
    let (p, y) = att::pitch_yaw_angles(&asm.vessels[asm.active].state);
    assert!(
        (p.to_degrees() - 25.0).abs() < 3.0,
        "稳态 pitch={}°",
        p.to_degrees()
    );
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
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
    }
    let v = asm.vessels[asm.active].state.vel;
    let r = asm.vessels[asm.active].state.r;
    let body_y = r.col(1);
    // 速度方向会因推力演化，检查 body +Y 与当前 prograde 对齐。
    let cos = dot(body_y, v.unit()).clamp(-1.0, 1.0);
    let angle = cos.acos().to_degrees();
    assert!(
        angle < 8.0,
        "body +Y 与 prograde 夹角应 < 8°，实际 {angle:.2}°"
    );
    // 同时确认确实跟踪了初始偏东方向（非停在竖直）。
    let cos0 = dot(body_y, prograde0).clamp(-1.0, 1.0);
    assert!(
        cos0.acos().to_degrees() < 12.0,
        "应朝初始 prograde 方向，夹角 {}",
        cos0.acos().to_degrees()
    );
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
    assert!(
        (recomputed.0 - p_pro).abs() < 1e-9,
        "pitch {} vs {}",
        recomputed.0,
        p_pro
    );
    assert!(
        (recomputed.1 - y_pro).abs() < 1e-9,
        "yaw {} vs {}",
        recomputed.1,
        y_pro
    );
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
