//! orbitx Zenoh 线协议（protobuf）与 keyexpr 辅助。

pub mod keyexpr;

pub use prost::Message;

include!(concat!(env!("OUT_DIR"), "/orbitx.v1.rs"));

/// 编码任意 prost Message 为字节。
pub fn encode_message<M: Message>(msg: &M) -> Vec<u8> {
    let mut buf = Vec::with_capacity(msg.encoded_len());
    msg.encode(&mut buf).expect("prost encode");
    buf
}

/// 解码 Inbound。
pub fn decode_inbound(bytes: &[u8]) -> Result<Inbound, prost::DecodeError> {
    Inbound::decode(bytes)
}

/// 解码 Slice。
pub fn decode_slice(bytes: &[u8]) -> Result<Slice, prost::DecodeError> {
    Slice::decode(bytes)
}

#[cfg(test)]
mod tests {
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
    fn slice_round_trip() {
        let msg = Slice {
            sim_t: 1000,
            step_index: 50,
            paused: false,
            warp: 2.0,
            rocket_name: "falcon9".into(),
            launched: true,
            ..Default::default()
        };
        let bytes = encode_message(&msg);
        let back = decode_slice(&bytes).unwrap();
        assert_eq!(back.sim_t, 1000);
        assert_eq!(back.rocket_name, "falcon9");
        assert!(back.launched);
        assert!((back.warp - 2.0).abs() < 1e-12);
    }
}
