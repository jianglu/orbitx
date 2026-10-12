use super::*;
use crate::capability::ControlCapability;
use crate::workflow::{PhaseDesc, PredOp, TargetModeDesc, TransitionDesc, WorkFlow};
use orbitx_dynamics::GravBody;
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_protocol::mem::{FocusTelem, OrbitTelem, Slice};
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

fn mu_earth() -> f64 {
    // G * M：用标准 μ_earth。
    3.986_004_418e14
}

fn launch_asm() -> (Assembly, f64) {
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
        &[hold_spec()],
        StateVectors {
            pos,
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: rot,
            q,
        },
    );
    (asm, earth_r)
}

fn alt(asm: &Assembly) -> f64 {
    asm.state.pos.length() - asm.planet_radius
}

fn slice_alt(altitude: f64) -> Slice {
    Slice {
        ready: true,
        focus: FocusTelem {
            altitude,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn slice_fuel(fuel_pct: f64) -> Slice {
    Slice {
        ready: true,
        focus: FocusTelem {
            fuel_pct,
            ..Default::default()
        },
        ..Default::default()
    }
}

#[test]
fn advances_phase_on_altitude_threshold() {
    let (mut asm, earth_r) = launch_asm();
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let phases = vec![
        PhaseDesc {
            mode: TargetModeDesc::VerticalHold { throttle: 1.0 },
            transition: Some(TransitionDesc {
                kind: "altitude".into(),
                op: PredOp::Gt,
                value: 500.0,
            }),
        },
        PhaseDesc {
            mode: TargetModeDesc::ProgradeHold { throttle: 1.0 },
            transition: None,
        },
    ];
    let mut wf = TargetWorkFlow::new(phases, caps, 0.0);
    let dt = 0.05;
    let mut advanced = false;
    for _ in 0..(40.0 / dt) as usize {
        let slice = slice_alt(alt(&asm));
        wf.tick(&mut asm, &slice, dt);
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
        if wf.phase_idx() == 1 && !advanced {
            advanced = true;
            // 推进后 phase_time 归零，本 tick 末再 += dt → 约一个 dt（新阶段首 tick）。
            assert!(
                wf.phase_time() <= dt + 1e-9,
                "phase_time 应在新阶段首 tick ≈ dt，实际 {}",
                wf.phase_time()
            );
        }
    }
    assert!(advanced, "应在高度 > 500m 时推进到第二阶段");
    assert!(!wf.is_done(), "末段无 transition → 永驻，is_done=false");
}

#[test]
fn done_when_last_phase_transition_met() {
    let (mut asm, earth_r) = launch_asm();
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let phases = vec![
        PhaseDesc {
            mode: TargetModeDesc::VerticalHold { throttle: 1.0 },
            transition: Some(TransitionDesc {
                kind: "altitude".into(),
                op: PredOp::Gt,
                value: 300.0,
            }),
        },
        PhaseDesc {
            mode: TargetModeDesc::VerticalHold { throttle: 1.0 },
            transition: Some(TransitionDesc {
                kind: "altitude".into(),
                op: PredOp::Gt,
                value: 5_000.0,
            }),
        },
    ];
    let mut wf = TargetWorkFlow::new(phases, caps, 0.0);
    let dt = 0.05;
    for _ in 0..(60.0 / dt) as usize {
        if wf.is_done() {
            break;
        }
        let slice = slice_alt(alt(&asm));
        wf.tick(&mut asm, &slice, dt);
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
    }
    assert!(wf.is_done(), "末段 transition 满足后应 done");
    assert!(alt(&asm) > 5_000.0);
}

#[test]
fn time_transition_advances_after_duration() {
    let (mut asm, earth_r) = launch_asm();
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    let phases = vec![
        PhaseDesc {
            mode: TargetModeDesc::VerticalHold { throttle: 1.0 },
            transition: Some(TransitionDesc {
                kind: "time".into(),
                op: PredOp::Gt,
                value: 2.0,
            }),
        },
        PhaseDesc {
            mode: TargetModeDesc::VerticalHold { throttle: 0.3 },
            transition: None,
        },
    ];
    let mut wf = TargetWorkFlow::new(phases, caps, 0.0);
    let dt = 0.1;
    for _ in 0..(10.0 / dt) as usize {
        let slice = slice_alt(alt(&asm));
        wf.tick(&mut asm, &slice, dt);
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
    }
    assert_eq!(wf.phase_idx(), 1);
    // 第二阶段油门 0.3：active vessel level 应为 0.3。
    let lvl = asm.vessels[asm.active].thrusters[0].level;
    assert!((lvl - 0.3).abs() < 1e-9, "level={lvl}");
}

#[test]
fn fuel_pct_lt_transition() {
    let (mut asm, earth_r) = launch_asm();
    asm.planet_radius = earth_r;
    let earth = earth();
    let caps = ControlCapability::for_primary(&asm);
    // 高油门快速耗油，fuel_pct 下降。
    let phases = vec![
        PhaseDesc {
            mode: TargetModeDesc::VerticalHold { throttle: 1.0 },
            transition: Some(TransitionDesc {
                kind: "fuel_pct".into(),
                op: PredOp::Lt,
                value: 50.0,
            }),
        },
        PhaseDesc {
            mode: TargetModeDesc::VerticalHold { throttle: 0.0 },
            transition: None,
        },
    ];
    let mut wf = TargetWorkFlow::new(phases, caps, 0.0);
    let dt = 0.05;
    let mut advanced = false;
    for _ in 0..(200.0 / dt) as usize {
        let slice = slice_fuel(asm.fuel_percent());
        wf.tick(&mut asm, &slice, dt);
        asm.step(dt, StepEnv::primary0(&[earth.clone()]));
        if wf.phase_idx() == 1 {
            advanced = true;
            break;
        }
    }
    assert!(advanced, "应在 fuel_pct < 50% 时推进");
}

#[test]
fn apoapsis_transition_with_mu() {
    // 给水平速度接近圆轨道，apoapsis 应接近当前半径。
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 200_000.0);
    let r_mag = pos.length();
    let mu = mu_earth();
    let v_circ = (mu / r_mag).sqrt(); // 圆轨道速度
    let vel = Vec3::new(v_circ, 0.0, 0.0);
    let up = pos * (1.0 / r_mag);
    let ref_axis = Vec3::new(0.0, 1.0, 0.0);
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let by = up;
    let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(rot);
    let mut asm = Assembly::new(
        &[hold_spec()],
        StateVectors {
            pos,
            vel,
            omega: Vec3::ZERO,
            r: rot,
            q,
        },
    );
    asm.planet_radius = earth_r;
    let caps = ControlCapability::for_primary(&asm);
    // 切片远地点是距地表高度。200 km 圆轨道的 ap_alt 高于 100 km 即推进。
    let alt_m = r_mag - earth_r;
    let phases = vec![
        PhaseDesc {
            mode: TargetModeDesc::ProgradeHold { throttle: 0.0 },
            transition: Some(TransitionDesc {
                kind: "apoapsis".into(),
                op: PredOp::Gt,
                value: 100_000.0,
            }),
        },
        PhaseDesc {
            mode: TargetModeDesc::ProgradeHold { throttle: 0.0 },
            transition: None,
        },
    ];
    let mut wf = TargetWorkFlow::new(phases, caps, mu);
    let slice = Slice {
        ready: true,
        orbit: OrbitTelem {
            ap_alt: alt_m,
            ..Default::default()
        },
        ..Default::default()
    };
    wf.tick(&mut asm, &slice, 0.05);
    assert_eq!(wf.phase_idx(), 1, "切片远地点高于阈值时应推进");
}
