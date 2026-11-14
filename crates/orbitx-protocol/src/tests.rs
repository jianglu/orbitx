//! 线协议往返测试（crate 根，覆盖对外编码契约）。

use super::*;
use std::collections::BTreeMap;

#[test]
fn inbound_round_trip_throttle() {
    let msg = Inbound {
        payload: Some(inbound::Payload::Input(InputCmd {
            kind: Some(input_cmd::Kind::SetThrottle(SetThrottle { level: 0.7 })),
        })),
    };
    let bytes = encode_message(&msg);
    let back = decode_inbound(&bytes).unwrap();
    match back.payload {
        Some(inbound::Payload::Input(InputCmd {
            kind: Some(input_cmd::Kind::SetThrottle(SetThrottle { level })),
        })) => assert!((level - 0.7).abs() < 1e-12),
        other => panic!("unexpected {other:?}"),
    }
}

#[test]
fn inbound_round_trip_set_attitude_nose() {
    // NoseTo 场景手动指令：罗盘方位/俯仰 + 目标记录 roll，逐字段往返。
    let msg = Inbound {
        payload: Some(inbound::Payload::Input(InputCmd {
            kind: Some(input_cmd::Kind::SetAttitudeNose(SetAttitudeNose {
                azimuth: 2.345,
                elevation: -0.617,
                roll: 0.25,
            })),
        })),
    };
    let bytes = encode_message(&msg);
    let back = decode_inbound(&bytes).unwrap();
    match back.payload {
        Some(inbound::Payload::Input(InputCmd {
            kind:
                Some(input_cmd::Kind::SetAttitudeNose(SetAttitudeNose {
                    azimuth,
                    elevation,
                    roll,
                })),
        })) => {
            assert!((azimuth - 2.345).abs() < 1e-12, "azimuth={azimuth}");
            assert!((elevation + 0.617).abs() < 1e-12, "elevation={elevation}");
            assert!((roll - 0.25).abs() < 1e-12, "roll={roll}");
        }
        other => panic!("unexpected {other:?}"),
    }
}

#[test]
fn slice_round_trip() {
    let msg = Slice {
        sim_t: 1000,
        step_index: 50,
        paused: false,
        warp: 2.0,
        rocket_name: "falcon9".into(),
        launched: true,
        nose_target: vec![0.1, -0.2, 0.975],
        ..Default::default()
    };
    let bytes = encode_message(&msg);
    let back = decode_slice(&bytes).unwrap();
    assert_eq!(back.sim_t, 1000);
    assert_eq!(back.rocket_name, "falcon9");
    assert!(back.launched);
    assert!((back.warp - 2.0).abs() < 1e-12);
    // nose_target：有目标 = 3 元素惯性系单位向量原样透传。
    assert_eq!(back.nose_target.len(), 3);
    assert!((back.nose_target[2] - 0.975).abs() < 1e-12);
}

#[test]
fn slice_round_trip_no_nose_target_is_empty() {
    // 无目标（VerticalHold 等）：字段缺省 = 空数组（消费端「空 = 无目标」）。
    let msg = Slice {
        sim_t: 1,
        ..Default::default()
    };
    let bytes = encode_message(&msg);
    let back = decode_slice(&bytes).unwrap();
    assert!(back.nose_target.is_empty());
}

#[test]
fn configure_session_context_and_schema_expected_round_trip() {
    // C3：通用会话上下文标签（BTreeMap）+ 黑匣子 schema 版本协商字段逐字段往返。
    let msg = Inbound {
        payload: Some(inbound::Payload::Session(SessionCmd {
            kind: Some(session_cmd::Kind::Configure(Configure {
                rocket: "falcon9".into(),
                control: "target".into(),
                session_context: BTreeMap::from([(
                    "recorder_subtree".to_string(),
                    "competition/orbital_insertion/senior/orbit".to_string(),
                )]),
                recorder_schema_expected: 1,
                has_recorder_schema_expected: true,
                ..Default::default()
            })),
        })),
    };
    let bytes = encode_message(&msg);
    let back = decode_inbound(&bytes).unwrap();
    let inbound::Payload::Session(SessionCmd {
        kind: Some(session_cmd::Kind::Configure(back)),
    }) = back.payload.expect("payload")
    else {
        panic!("expected Configure");
    };
    assert_eq!(
        back.session_context
            .get("recorder_subtree")
            .map(String::as_str),
        Some("competition/orbital_insertion/senior/orbit")
    );
    assert!(back.has_recorder_schema_expected);
    assert_eq!(back.recorder_schema_expected, 1);
}

#[test]
fn configure_defaults_are_empty() {
    // 未填上下文/协商字段 = 缺省（旧客户端语义零变化）。
    let msg = Inbound {
        payload: Some(inbound::Payload::Session(SessionCmd {
            kind: Some(session_cmd::Kind::Configure(Configure {
                rocket: "falcon9".into(),
                control: "base".into(),
                ..Default::default()
            })),
        })),
    };
    let bytes = encode_message(&msg);
    let back = decode_inbound(&bytes).unwrap();
    let inbound::Payload::Session(SessionCmd {
        kind: Some(session_cmd::Kind::Configure(back)),
    }) = back.payload.expect("payload")
    else {
        panic!("expected Configure");
    };
    assert!(back.session_context.is_empty());
    assert!(!back.has_recorder_schema_expected);
    assert_eq!(back.recorder_schema_expected, 0);
}

#[test]
fn outbound_round_trip_ack() {
    // ack 变体：ok + 实际 schema 版本回带。
    let msg = Outbound {
        payload: Some(outbound::Payload::Ack(ConfigureAck {
            ok: true,
            error: String::new(),
            recorder_schema_version: 1,
        })),
    };
    let bytes = encode_message(&msg);
    let back = decode_outbound(&bytes).unwrap();
    match back.payload {
        Some(outbound::Payload::Ack(ack)) => {
            assert!(ack.ok);
            assert!(ack.error.is_empty());
            assert_eq!(ack.recorder_schema_version, 1);
        }
        other => panic!("unexpected {other:?}"),
    }
}

#[test]
fn outbound_round_trip_halted() {
    // halted 变体：会话标识 + 黑匣子引用 + schema 版本。
    let msg = Outbound {
        payload: Some(outbound::Payload::Halted(Halted {
            session_id: "falcon9_a1b2_20261114_101500".into(),
            recorder_session_dir: "C:/records/competition/pack/g/task/falcon9_a1b2_20261114_101500"
                .into(),
            recorder_schema_version: 1,
        })),
    };
    let bytes = encode_message(&msg);
    let back = decode_outbound(&bytes).unwrap();
    match back.payload {
        Some(outbound::Payload::Halted(h)) => {
            assert_eq!(h.session_id, "falcon9_a1b2_20261114_101500");
            assert!(h.recorder_session_dir.contains("competition"));
            assert_eq!(h.recorder_schema_version, 1);
        }
        other => panic!("unexpected {other:?}"),
    }
}

#[test]
fn outbound_halted_without_recorder_has_empty_dir() {
    // 未启用录制：仅发事件、目录为空（RUNTIME.md 停机收尾第 3 步）。
    let msg = Outbound {
        payload: Some(outbound::Payload::Halted(Halted {
            session_id: "s".into(),
            recorder_session_dir: String::new(),
            recorder_schema_version: 1,
        })),
    };
    let bytes = encode_message(&msg);
    let back = decode_outbound(&bytes).unwrap();
    match back.payload {
        Some(outbound::Payload::Halted(h)) => {
            assert!(h.recorder_session_dir.is_empty());
        }
        other => panic!("unexpected {other:?}"),
    }
}
