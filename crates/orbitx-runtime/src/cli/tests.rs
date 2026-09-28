use std::io::Write;

use clap::Parser;
use orbitx_controller::workflow::WorkFlowKind;

use super::test_runtime_args;
use super::{validate_zenoh_endpoint, ControlKindArg, RuntimeArgs, SessionControl};

#[test]
fn rejects_missing_flags() {
    assert!(RuntimeArgs::try_parse_from(["orbitx-runtime"]).is_err());
}

#[test]
fn explicit_args_validate() {
    let args = test_runtime_args("falcon9");
    assert_eq!(args.scenario, "earth");
    assert_eq!(args.sim_dt, 20);
    args.validate().unwrap();
    match args.resolve_control().unwrap() {
        SessionControl::Control(ControlKindArg::Target) => {}
        other => panic!("expected Control(Target), got {other:?}"),
    }
}

#[test]
fn control_without_value_is_rejected() {
    let err = RuntimeArgs::try_parse_from(["orbitx-runtime", "--control"]).unwrap_err();
    let _ = err;
}

#[test]
fn control_base() {
    let mut args = test_runtime_args("falcon9");
    args.control = Some(ControlKindArg::Base);
    match args.resolve_control().unwrap() {
        SessionControl::Control(ControlKindArg::Base) => {}
        other => panic!("expected Control(Base), got {other:?}"),
    }
}

#[test]
fn workflow_loads_target_toml() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("ascent.toml");
    let mut f = std::fs::File::create(&path).unwrap();
    write!(
        f,
        r#"
kind = "target"
name = "ascent"

[[phases]]
mode = "vertical_hold"
throttle = 1.0
"#
    )
    .unwrap();

    let mut args = test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(path);
    let ctrl = args.resolve_control().unwrap();
    match ctrl {
        SessionControl::WorkFlow { desc, .. } => {
            assert_eq!(desc.kind, WorkFlowKind::Target);
            assert_eq!(desc.name, "ascent");
        }
        other => panic!("expected WorkFlow, got {other:?}"),
    }
    args.validate().unwrap();
}

#[test]
fn workflow_missing_file_errors() {
    let mut args = test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some("no-such-wf.toml".into());
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("不存在") || err.contains("workflow"));
}

#[test]
fn workflow_invalid_toml_errors() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bad.toml");
    std::fs::write(&path, "kind = \"nope\"\nname = \"x\"\n").unwrap();
    let mut args = test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(path);
    assert!(args.resolve_control().is_err());
}

#[test]
fn control_and_workflow_conflict() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("ascent.toml");
    std::fs::write(
        &path,
        r#"
kind = "target"
name = "ascent"
[[phases]]
mode = "vertical_hold"
throttle = 1.0
"#,
    )
    .unwrap();
    let mut args = test_runtime_args("falcon9");
    args.workflow = Some(path);
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("互斥"));
    assert!(args.validate().is_err());
}

#[test]
fn rejects_non_positive_sim_dt() {
    let mut args = test_runtime_args("falcon9");
    args.sim_dt = 0;
    assert!(args.validate().is_err());
}

#[test]
fn neither_control_nor_workflow_fails() {
    let mut args = test_runtime_args("falcon9");
    args.control = None;
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("--control"));
}

#[test]
fn accepts_local_and_loopback() {
    validate_zenoh_endpoint("local").unwrap();
    validate_zenoh_endpoint("tcp/127.0.0.1:7447").unwrap();
    validate_zenoh_endpoint("tcp/localhost:9").unwrap();
    validate_zenoh_endpoint("unixpipe:///tmp/orbitx.sock").unwrap();
}

#[test]
fn rejects_cross_device_tcp() {
    let err = validate_zenoh_endpoint("tcp/192.168.1.10:7447").unwrap_err();
    assert!(err.contains("cross-device") || err.contains("not supported"));
}

#[test]
fn loads_builtin_rocket_on_validate() {
    let args = test_runtime_args("saturnv");
    let session = args.load_session().unwrap();
    assert_eq!(session.rocket.class, "SaturnV");
    assert!(matches!(
        session.control,
        SessionControl::Control(ControlKindArg::Target)
    ));
}

#[test]
fn rejects_unknown_rocket() {
    let args = test_runtime_args("nope");
    assert!(args.validate().is_err());
}
