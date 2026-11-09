//! 线协议往返测试（crate 根，覆盖对外编码契约）。

use super::*;

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
            kind: Some(input_cmd::Kind::SetAttitudeNose(SetAttitudeNose {
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
