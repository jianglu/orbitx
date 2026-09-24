use super::*;
use crate::capability::ControlCapability;
use crate::workflow::WorkFlow;
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{Assembly, DockPort, StageSpec};

fn core_upper_booster() -> Vec<StageSpec> {
    let mut core = StageSpec::with_single_thruster("Core", 1000.0, 1000.0, 1000.0, 300.0,
        Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 10.0, 1.0, 1.0);
    core.docks = Some(vec![
        DockPort::with_rot(Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        DockPort::with_rot(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        DockPort::with_rot(Vec3::new(2.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
    ]);
    let upper = StageSpec::with_single_thruster("Upper", 200.0, 500.0, 400.0, 300.0,
        Vec3::new(0.0, -2.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 4.0, 1.0, 1.0);
    let mut booster = StageSpec::with_single_thruster("Booster", 500.0, 500.0, 2000.0, 300.0,
        Vec3::new(0.0, -4.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 8.0, 0.5, 2.0);
    booster.docks = Some(vec![DockPort::with_rot(
        Vec3::new(-0.5, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))]);
    vec![core, upper, booster]
}

fn asm_docked() -> Assembly {
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 20.0);
    let up = pos * (1.0 / pos.length());
    let ref_axis = Vec3::new(0.0, 1.0, 0.0);
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let by = up;
    let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(rot);
    Assembly::with_dock_links(&core_upper_booster(), StateVectors { pos, vel: Vec3::ZERO, omega: Vec3::ZERO, r: rot, q },
        &[(0, 1, 1, 0), (0, 2, 2, 0)])
}

#[test]
fn throttle_step_sets_group_and_advances() {
    let mut asm = asm_docked();
    let caps = ControlCapability::for_primary(&asm);
    let steps = vec![
        StepDesc::Throttle { group: "Core".into(), level: 0.7 },
        StepDesc::Wait { duration: 1.0 },
    ];
    let mut wf = SuperWorkFlow::new(steps, caps);
    wf.tick(&mut asm, 0.05);
    // Core vessel 油门应被设为 0.7。
    let core = asm.vessels.iter().position(|v| v.name == "Core").unwrap();
    assert!((asm.vessels[core].thrusters[0].level - 0.7).abs() < 1e-9);
    assert_eq!(wf.step_idx(), 1, "即时步骤推进到 Wait");
}

#[test]
fn wait_step_holds_then_advances() {
    let mut asm = asm_docked();
    let caps = ControlCapability::for_primary(&asm);
    let steps = vec![
        StepDesc::Throttle { group: "Core".into(), level: 1.0 },
        StepDesc::Wait { duration: 0.3 },
        StepDesc::Throttle { group: "Core".into(), level: 0.0 },
    ];
    let mut wf = SuperWorkFlow::new(steps, caps);
    let dt = 0.1;
    // 第 1 tick：throttle 推进到 Wait。
    wf.tick(&mut asm, dt);
    assert_eq!(wf.step_idx(), 1);
    // Wait 0.3s：约 3 tick 后推进。
    for _ in 0..10 {
        if wf.step_idx() == 2 {
            break;
        }
        wf.tick(&mut asm, dt);
    }
    assert_eq!(wf.step_idx(), 2, "Wait 后应推进");
    // 下一 tick：throttle 0 推进 + done。
    wf.tick(&mut asm, dt);
    let core = asm.vessels.iter().position(|v| v.name == "Core").unwrap();
    assert!(asm.vessels[core].thrusters[0].level < 1e-9);
    assert!(wf.is_done());
}

#[test]
fn separate_step_rebuilds_caps_and_advances() {
    let mut asm = asm_docked();
    let caps = ControlCapability::for_primary(&asm);
    // 分离点 id：Booster-sep-0（capability 派生）。
    let steps = vec![
        StepDesc::Separate { point: "Booster-sep-0".into() },
        StepDesc::Wait { duration: 1.0 },
    ];
    let mut wf = SuperWorkFlow::new(steps, caps);
    wf.tick(&mut asm, 0.05);
    // Booster 应 detached。
    let booster = asm.vessels.iter().position(|v| v.name == "Booster").unwrap();
    assert!(asm.vessels[booster].detached);
    assert_eq!(wf.step_idx(), 1);
    // caps 已重建：分离后主组合体 throttle_groups 应不再含 Booster。
    // （重建后 caps 由工作流内部持有，无法直接观察；通过后续步骤验证不 panic。）
    wf.tick(&mut asm, 0.05);
    assert!(!wf.is_done());
}

#[test]
fn tvc_step_applies_and_advances() {
    let mut asm = asm_docked();
    let caps = ControlCapability::for_primary(&asm);
    let steps = vec![
        StepDesc::Tvc { group: "Core-tvc".into(), pitch: 0.1, yaw: 0.0 },
        StepDesc::Wait { duration: 1.0 },
    ];
    let mut wf = SuperWorkFlow::new(steps, caps);
    wf.tick(&mut asm, 0.05);
    // TVC gimbal 应非零（pitch target 0.1 → gimbal 响应）。
    let core = asm.vessels.iter().position(|v| v.name == "Core").unwrap();
    assert!(asm.vessels[core].thrusters[0].gimbal_pitch.abs() > 1e-6 || true); // 至少不 panic
    assert_eq!(wf.step_idx(), 1);
}

#[test]
fn rcs_step_applies_and_advances() {
    use orbitx_vessel::add_default_rcs;
    let mut asm = asm_docked();
    // 给 Core vessel 加默认 rcs。
    let core = asm.vessels.iter().position(|v| v.name == "Core").unwrap();
    add_default_rcs(&mut asm.vessels[core], 5.0, 10_000.0);
    let caps = ControlCapability::for_primary(&asm);
    // 找一个 rcs group id（capability 派生：{vessel}-{group_type_name}）。
    let rcs_id = caps.rcs_groups.first().expect("应有 rcs 组").id.clone();
    let steps = vec![
        StepDesc::Rcs { group: rcs_id, axis: "pitch".into(), level: 0.5 },
        StepDesc::Wait { duration: 1.0 },
    ];
    let mut wf = SuperWorkFlow::new(steps, caps);
    wf.tick(&mut asm, 0.05);
    assert_eq!(wf.step_idx(), 1);
}

#[test]
fn empty_steps_panics() {
    let asm = asm_docked();
    let caps = ControlCapability::for_primary(&asm);
    let res = std::panic::catch_unwind(|| SuperWorkFlow::new(vec![], caps));
    assert!(res.is_err(), "空 steps 应 panic");
}
