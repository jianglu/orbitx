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

/// Slice 通道满时丢本帧（Comms 侧 `try_recv` 排空取最新，实现 keep-latest）。
pub fn send_slice_keep_latest(tx: &Sender<Arc<Slice>>, slice: Arc<Slice>) {
    let _ = tx.try_send(slice);
}
