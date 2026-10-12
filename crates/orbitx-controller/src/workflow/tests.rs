use super::*;

#[test]
fn parse_target_workflow_basic() {
    let s = r#"
kind = "target"
name = "ascent"

[[phases]]
mode = "vertical_hold"
throttle = 1.0
transition = { kind = "altitude", op = "gt", value = 10000.0 }

[[phases]]
mode = "gravity_turn"
throttle = 1.0
kick_pitch = 0.0
kick_yaw = 0.087
kick_rate = 0.05
transition = { kind = "altitude", op = "gt", value = 80000.0 }

[[phases]]
mode = "prograde_hold"
throttle = 1.0
"#;
    let d = from_toml_str(s).expect("解析");
    assert_eq!(d.kind, WorkFlowKind::Target);
    assert_eq!(d.name, "ascent");
    let phases = d.phases.expect("phases");
    assert_eq!(phases.len(), 3);
    assert_eq!(
        phases[0].mode,
        TargetModeDesc::VerticalHold { throttle: 1.0 }
    );
    assert_eq!(
        phases[1].mode,
        TargetModeDesc::GravityTurn {
            throttle: 1.0,
            kick_pitch: 0.0,
            kick_yaw: 0.087,
            kick_rate: 0.05,
            min_alt: crate::target::DEFAULT_MIN_ALT,
            min_speed: crate::target::DEFAULT_MIN_SPEED,
        }
    );
    assert_eq!(phases[2].transition, None);
    let t0 = phases[0].transition.as_ref().unwrap();
    assert_eq!(t0.kind, "altitude");
    assert_eq!(t0.op, PredOp::Gt);
    assert_eq!(t0.value, 10000.0);
}

#[test]
fn gravity_turn_toml_defaults_point_east() {
    let s = r#"
kind = "target"
name = "ascent"

[[phases]]
mode = "gravity_turn"
throttle = 1.0
"#;
    let d = from_toml_str(s).expect("解析");
    let phases = d.phases.expect("phases");
    assert_eq!(
        phases[0].mode,
        TargetModeDesc::GravityTurn {
            throttle: 1.0,
            kick_pitch: crate::target::DEFAULT_KICK_PITCH,
            kick_yaw: crate::target::DEFAULT_KICK_YAW,
            kick_rate: crate::target::DEFAULT_KICK_RATE,
            min_alt: crate::target::DEFAULT_MIN_ALT,
            min_speed: crate::target::DEFAULT_MIN_SPEED,
        }
    );
}

#[test]
fn parse_super_workflow_basic() {
    let s = r#"
kind = "super"
name = "full"

[[steps]]
action = "throttle"
group = "Core"
level = 1.0

[[steps]]
action = "tvc"
group = "Core-tvc"
pitch = 0.0
yaw = 0.0

[[steps]]
action = "wait"
duration = 5.0

[[steps]]
action = "separate"
point = "Booster-sep-0"
"#;
    let d = from_toml_str(s).expect("解析");
    assert_eq!(d.kind, WorkFlowKind::Super);
    let steps = d.steps.expect("steps");
    assert_eq!(steps.len(), 4);
    assert_eq!(
        steps[0],
        StepDesc::Throttle {
            group: "Core".into(),
            level: 1.0
        }
    );
    assert_eq!(
        steps[1],
        StepDesc::Tvc {
            group: "Core-tvc".into(),
            pitch: 0.0,
            yaw: 0.0
        }
    );
    assert_eq!(steps[2], StepDesc::Wait { duration: 5.0 });
    assert_eq!(
        steps[3],
        StepDesc::Separate {
            point: "Booster-sep-0".into()
        }
    );
}

#[test]
fn target_mode_desc_to_mode_round_trip() {
    let d = TargetModeDesc::PitchTo {
        pitch: 0.1,
        yaw: -0.05,
        throttle: 0.8,
    };
    let m: TargetMode = d.into();
    assert!(matches!(m, TargetMode::PitchTo { pitch, yaw, throttle }
        if (pitch - 0.1).abs() < 1e-12 && (yaw + 0.05).abs() < 1e-12 && (throttle - 0.8).abs() < 1e-12));
}

#[test]
fn reject_target_missing_phases() {
    let s = r#"
kind = "target"
name = "x"
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(_)));
}

#[test]
fn reject_target_with_steps() {
    let s = r#"
kind = "target"
name = "x"

[[phases]]
mode = "vertical_hold"
throttle = 1.0

[[steps]]
action = "wait"
duration = 1.0
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(m) if m.contains("不应含 [[steps]]")));
}

#[test]
fn reject_super_with_phases() {
    let s = r#"
kind = "super"
name = "x"

[[steps]]
action = "wait"
duration = 1.0

[[phases]]
mode = "vertical_hold"
throttle = 1.0
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(m) if m.contains("不应含 [[phases]]")));
}

#[test]
fn reject_non_last_phase_without_transition() {
    let s = r#"
kind = "target"
name = "x"

[[phases]]
mode = "vertical_hold"
throttle = 1.0

[[phases]]
mode = "prograde_hold"
throttle = 1.0
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(m) if m.contains("非末段必须 transition")));
}

#[test]
fn reject_transition_missing_fields() {
    let s = r#"
kind = "target"
name = "x"

[[phases]]
mode = "vertical_hold"
throttle = 1.0
transition = {}

[[phases]]
mode = "prograde_hold"
throttle = 1.0
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Parse(_)));
}

#[test]
fn reject_transition_unknown_kind() {
    let s = r#"
kind = "target"
name = "x"

[[phases]]
mode = "vertical_hold"
throttle = 1.0
transition = { kind = "altitude_gt", op = "gt", value = 100.0 }

[[phases]]
mode = "prograde_hold"
throttle = 1.0
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(m) if m.contains("未知谓词")));
}

#[test]
fn reject_throttle_level_out_of_range() {
    let s = r#"
kind = "super"
name = "x"

[[steps]]
action = "throttle"
group = "Core"
level = 1.5
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(m) if m.contains("[0,1]")));
}

#[test]
fn reject_negative_wait() {
    let s = r#"
kind = "super"
name = "x"

[[steps]]
action = "wait"
duration = -1.0
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(m) if m.contains("不能为负")));
}

#[test]
fn reject_empty_name() {
    let s = r#"
kind = "target"
name = ""

[[phases]]
mode = "vertical_hold"
throttle = 1.0
"#;
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Validate(m) if m.contains("name")));
}

#[test]
fn reject_bad_toml() {
    let s = "kind = ";
    let err = from_toml_str(s).unwrap_err();
    assert!(matches!(err, SchemaError::Parse(_)));
}

#[test]
fn transition_term_validate() {
    let t = TransitionDesc {
        kind: "altitude".into(),
        op: PredOp::Gt,
        value: 1.0,
    };
    assert!(t.validate("t").is_ok());
    let bad = TransitionDesc {
        kind: "altitude_gt".into(),
        op: PredOp::Gt,
        value: 1.0,
    };
    assert!(bad.validate("t").unwrap_err().contains("未知谓词"));
}
