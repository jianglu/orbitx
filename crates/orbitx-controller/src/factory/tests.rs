use super::*;
use crate::workflow::{from_toml_str, WorkFlowKind};
use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::{Assembly, StageSpec};

fn single_stage() -> Vec<StageSpec> {
    vec![StageSpec::with_single_thruster(
        "solo", 1000.0, 1000.0, 1000.0, 300.0,
        Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 10.0, 1.0, 1.0,
    )]
}

fn asm() -> Assembly {
    Assembly::new(&single_stage(), StateVectors::default())
}

#[test]
fn build_controller_returns_target() {
    let c = build_controller(TargetMode::VerticalHold { throttle: 1.0 });
    // 仅验证可构造为 Box<dyn Controller>；tick 行为由 target 模块测试覆盖。
    let _ = c;
}

#[test]
fn build_manual_control_primary_entry() {
    let ctrl = build_manual_control(TargetMode::VerticalHold { throttle: 0.5 });
    match ctrl {
        Control::Controller(entries) => {
            assert_eq!(entries.len(), 1);
            assert_eq!(entries[0].body, BodyRef::Primary);
        }
        Control::WorkFlow(_) => panic!("应为手动模式"),
    }
}

#[test]
fn build_control_target_workflow() {
    let s = r#"
kind = "target"
name = "ascent"

[[phases]]
mode = "vertical_hold"
throttle = 1.0
transition = { altitude_gt = 1000.0 }

[[phases]]
mode = "prograde_hold"
throttle = 1.0
"#;
    let desc = from_toml_str(s).expect("解析");
    let asm = asm();
    let ctrl = build_control(&desc, &asm, 0.0);
    match ctrl {
        Control::WorkFlow(_) => {}
        Control::Controller(_) => panic!("应为 workflow 模式"),
    }
}

#[test]
fn build_control_super_workflow() {
    let s = r#"
kind = "super"
name = "seq"

[[steps]]
action = "throttle"
group = "solo"
level = 1.0

[[steps]]
action = "wait"
duration = 1.0
"#;
    let desc = from_toml_str(s).expect("解析");
    let asm = asm();
    let ctrl = build_control(&desc, &asm, 0.0);
    assert!(matches!(ctrl, Control::WorkFlow(_)));
}

#[test]
fn build_workflow_target_uses_mu() {
    let s = r#"
kind = "target"
name = "orbit"

[[phases]]
mode = "prograde_hold"
throttle = 1.0
transition = { apoapsis_gt = 7000000.0 }

[[phases]]
mode = "prograde_hold"
throttle = 0.0
"#;
    let desc = from_toml_str(s).expect("解析");
    let asm = asm();
    // mu > 0 应被接受（不 panic）。
    let wf = build_workflow(&desc, &asm, 3.986e14);
    assert!(!wf.is_done());
}

#[test]
fn controller_assignment_lookup() {
    let a = ControllerAssignment::new()
        .for_vessel("Booster", TargetMode::VerticalHold { throttle: 0.0 })
        .for_vessel("Upper", TargetMode::ProgradeHold { throttle: 1.0 });
    assert_eq!(a.len(), 2);
    assert!(!a.is_empty());
    assert_eq!(
        a.derive_mode("Booster"),
        Some(TargetMode::VerticalHold { throttle: 0.0 })
    );
    assert_eq!(
        a.derive_mode("Upper"),
        Some(TargetMode::ProgradeHold { throttle: 1.0 })
    );
    assert_eq!(a.derive_mode("Unknown"), None);
}

#[test]
fn controller_assignment_default_empty() {
    let a = ControllerAssignment::new();
    assert!(a.is_empty());
    assert_eq!(a.derive_mode("anything"), None);
}

#[test]
fn control_workflow_kind_round_trip() {
    // 验证 WorkFlowKind 与 build_workflow 分派一致。
    let s = r#"
kind = "super"
name = "x"

[[steps]]
action = "wait"
duration = 0.5
"#;
    let desc = from_toml_str(s).expect("解析");
    assert_eq!(desc.kind, WorkFlowKind::Super);
    let asm = asm();
    let _ = build_workflow(&desc, &asm, 0.0);
}

#[test]
fn build_workflow_owns_primary_caps() {
    // 验证 build_workflow 构造的 workflow 持有主 caps（通过 tick 不 panic 间接验证）。
    let s = r#"
kind = "target"
name = "x"

[[phases]]
mode = "vertical_hold"
throttle = 0.0
"#;
    let desc = from_toml_str(s).expect("解析");
    let mut asm = asm();
    let mut wf = build_workflow(&desc, &asm, 0.0);
    wf.tick(&mut asm, 0.05);
    assert!(!wf.is_done());
}
