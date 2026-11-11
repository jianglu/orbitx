//! Comms ↔ Runtime / Runtime ↔ Comms 有界 flume 通道。

use std::sync::Arc;

use flume::{Receiver, Sender};

use crate::input::{InputCmd, SessionCmd};
use crate::slice::Slice;
use orbitx_protocol::ConfigureAck;

pub const CMD_CAPACITY: usize = 256;
pub const SLICE_CAPACITY: usize = 8;
pub const ACK_CAPACITY: usize = 8;

#[derive(Debug, Clone)]
pub enum RuntimeInbound {
    Input(InputCmd),
    Session(SessionCmd),
}

pub struct RuntimeChannels {
    pub cmd_tx: Sender<RuntimeInbound>,
    pub cmd_rx: Receiver<RuntimeInbound>,
    pub slice_bus: SliceBus,
    pub slice_rx: Receiver<Arc<Slice>>,
    /// 回执通道（host 层 pre-configure 循环发，Comms publish 到 ack key）。
    pub ack_tx: Sender<ConfigureAck>,
    pub ack_rx: Receiver<ConfigureAck>,
}

impl RuntimeChannels {
    pub fn bounded() -> Self {
        let (cmd_tx, cmd_rx) = flume::bounded(CMD_CAPACITY);
        let (slice_tx, slice_rx) = flume::bounded(SLICE_CAPACITY);
        let (ack_tx, ack_rx) = flume::bounded(ACK_CAPACITY);
        Self {
            cmd_tx,
            cmd_rx,
            slice_bus: SliceBus {
                tx: slice_tx,
                latest: Arc::new(std::sync::Mutex::new(None)),
            },
            slice_rx,
            ack_tx,
            ack_rx,
        }
    }
}

/// 推送切片。Comms 侧 `try_recv` 排空只 publish **队列中最新**。
/// 满时：丢弃队头占位（经同线程 `Receiver` 窥视不可行）——改为先 `try_send`，
/// 失败则写入旁路 `latest` 槽，由 Comms 在排空后合并（见 `SliceBus`）。
pub fn send_slice_keep_latest(bus: &SliceBus, slice: Arc<Slice>) {
    match bus.tx.try_send(slice.clone()) {
        Ok(()) => {}
        Err(flume::TrySendError::Full(_)) => {
            // 队列满：只更新 latest，保证最新态不丢；Comms 排空队列后合并。
            *bus.latest.lock().unwrap() = Some(slice);
        }
        Err(flume::TrySendError::Disconnected(_)) => {}
    }
}

/// Slice 出站：有界队列 + latest 旁路（满时覆盖）。
#[derive(Clone)]
pub struct SliceBus {
    pub tx: Sender<Arc<Slice>>,
    pub latest: Arc<std::sync::Mutex<Option<Arc<Slice>>>>,
}

impl SliceBus {
    /// 取出并清空旁路槽（满时写入的最新帧）。
    pub fn take_latest_if_any(&self) -> Option<Arc<Slice>> {
        self.latest.lock().unwrap().take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slice_at(step: u64) -> Arc<Slice> {
        Arc::new(Slice {
            step_index: step,
            ..Default::default()
        })
    }

    #[test]
    fn keep_latest_overwrites_bypass_slot_when_full() {
        let channels = RuntimeChannels::bounded();
        // SLICE_CAPACITY 内正常入队（latest 槽不动）。
        for step in 0..SLICE_CAPACITY as u64 {
            send_slice_keep_latest(&channels.slice_bus, slice_at(step));
        }
        assert!(channels.slice_bus.take_latest_if_any().is_none());
        // 队列满后：只更新 latest，保证最新态不丢。
        for step in (SLICE_CAPACITY as u64)..(SLICE_CAPACITY as u64 + 5) {
            send_slice_keep_latest(&channels.slice_bus, slice_at(step));
        }
        let latest = channels
            .slice_bus
            .take_latest_if_any()
            .expect("full 时必须写入 latest 槽");
        assert_eq!(
            latest.step_index,
            SLICE_CAPACITY as u64 + 4,
            "满时覆盖为最新帧"
        );
        // take 清槽：不残留旧帧。
        assert!(channels.slice_bus.take_latest_if_any().is_none());
    }

    #[test]
    fn comms_merge_prefers_slot_over_queue_tail() {
        // 模拟 Comms 排空语义：队列尾 vs 满时写入的 slot——slot 更新，取 slot。
        let channels = RuntimeChannels::bounded();
        for step in 0..(SLICE_CAPACITY as u64 + 2) {
            send_slice_keep_latest(&channels.slice_bus, slice_at(step));
        }
        let mut latest = None;
        while let Ok(s) = channels.slice_rx.try_recv() {
            latest = Some(s);
        }
        if let Some(slot) = channels.slice_bus.take_latest_if_any() {
            latest = Some(slot);
        }
        let got = latest.expect("必须有帧");
        assert_eq!(
            got.step_index,
            SLICE_CAPACITY as u64 + 1,
            "Comms 合并应取 slot（最新）而非队列尾"
        );
    }
}
