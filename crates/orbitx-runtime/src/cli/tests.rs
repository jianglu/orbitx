use std::io::Write;

use clap::Parser;
use orbitx_controller::workflow::WorkFlowKind;

use super::{
    validate_zenoh_endpoint, ControlKindArg, DriveModeArg, RuntimeArgs, SessionControl,
};

#[test]
fn parses_defaults() {
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime"]).expect("parse");
    assert!(args.scenario.is_none());
    assert!(args.control.is_none());
    assert!(args.workflow.is_none());
    assert_eq!(args.rocket, "falcon9");
    assert_eq!(args.drive, DriveModeArg::SelfPaced);
    assert_eq!(args.sim_dt, 20);
    assert_eq!(args.zenoh_endpoint, "local");
    args.validate().unwrap();
    match args.resolve_control().unwrap() {
        SessionControl::Control(ControlKindArg::Target) => {}
        other => panic!("expected Control(Target), got {other:?}"),
    }
}

#[test]
fn control_flag_defaults_to_target() {
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--control"]).unwrap();
    assert_eq!(args.control, Some(ControlKindArg::Target));
    match args.resolve_control().unwrap() {
        SessionControl::Control(ControlKindArg::Target) => {}
        other => panic!("expected Control(Target), got {other:?}"),
    }
}

#[test]
fn control_base() {
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--control", "base"]).unwrap();
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

    let args =
        RuntimeArgs::try_parse_from(["orbitx-runtime", "--workflow", path.to_str().unwrap()])
            .unwrap();
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
    let args =
        RuntimeArgs::try_parse_from(["orbitx-runtime", "--workflow", "no-such-wf.toml"]).unwrap();
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("不存在") || err.contains("workflow"));
}

#[test]
fn workflow_invalid_toml_errors() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bad.toml");
    std::fs::write(&path, "kind = \"nope\"\nname = \"x\"\n").unwrap();
    let args =
        RuntimeArgs::try_parse_from(["orbitx-runtime", "--workflow", path.to_str().unwrap()])
            .unwrap();
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
    let args = RuntimeArgs::try_parse_from([
        "orbitx-runtime",
        "--control",
        "target",
        "--workflow",
        path.to_str().unwrap(),
    ])
    .unwrap();
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("互斥"));
    assert!(args.validate().is_err());
}

#[test]
fn rejects_non_positive_sim_dt() {
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--sim-dt", "0"]).unwrap();
    assert!(args.validate().is_err());
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
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--rocket", "saturnv"]).unwrap();
    let session = args.load_session().unwrap();
    assert_eq!(session.rocket.class, "SaturnV");
    assert!(matches!(
        session.control,
        SessionControl::Control(ControlKindArg::Target)
    ));
}

#[test]
fn rejects_unknown_rocket() {
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--rocket", "nope"]).unwrap();
    assert!(args.validate().is_err());
}
