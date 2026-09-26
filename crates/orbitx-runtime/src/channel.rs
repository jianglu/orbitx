//! Comms ↔ Runtime / Runtime ↔ Comms 有界 flume 通道。

use std::sync::Arc;

use flume::{Receiver, Sender};

use crate::input::{InputCmd, SessionCmd};
use crate::slice::Slice;

pub const CMD_CAPACITY: usize = 256;
pub const SLICE_CAPACITY: usize = 8;

#[derive(Debug, Clone)]
pub enum RuntimeInbound {
    Input(InputCmd),
    Session(SessionCmd),
}

pub struct RuntimeChannels {
    pub cmd_tx: Sender<RuntimeInbound>,
    pub cmd_rx: Receiver<RuntimeInbound>,
    pub slice_tx: Sender<Arc<Slice>>,
    pub slice_rx: Receiver<Arc<Slice>>,
}

impl RuntimeChannels {
    pub fn bounded() -> Self {
        let (cmd_tx, cmd_rx) = flume::bounded(CMD_CAPACITY);
        let (slice_tx, slice_rx) = flume::bounded(SLICE_CAPACITY);
        Self {
            cmd_tx,
            cmd_rx,
            slice_tx,
            slice_rx,
        }
    }
}

/// Slice 通道满时尽量投递；骨架阶段满则丢弃本帧投递（Comms stub 应及时 drain）。
/// 阶段 B 可改为双端协作「丢旧留新」。
pub fn send_slice_keep_latest(tx: &Sender<Arc<Slice>>, slice: Arc<Slice>) {
    let _ = tx.try_send(slice);
}
