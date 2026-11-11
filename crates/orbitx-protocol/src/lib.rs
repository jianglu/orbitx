//! orbitx Zenoh 线协议（protobuf）与 keyexpr 辅助。

pub mod keyexpr;
pub mod mem;

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

/// 解码下行回执（ack 通道）。
pub fn decode_ack(bytes: &[u8]) -> Result<ConfigureAck, prost::DecodeError> {
    ConfigureAck::decode(bytes)
}

#[cfg(test)]
mod tests;

