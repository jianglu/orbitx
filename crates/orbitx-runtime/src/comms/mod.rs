//! CommsService：本机 Zenoh + SHM（P4.3）。

pub mod zenoh_svc;

use crate::channel::{RuntimeChannels, SliceBus};
use crate::shutdown::ShutdownFlag;

pub struct CommsHandles {
    pub cmd_tx: flume::Sender<crate::channel::RuntimeInbound>,
    pub slice_rx: flume::Receiver<std::sync::Arc<crate::slice::Slice>>,
    pub slice_bus: SliceBus,
    /// 回执（ConfigureAck 等）：host 层发 → Comms publish 到 ack key。
    pub ack_rx: flume::Receiver<orbitx_protocol::ConfigureAck>,
}

impl CommsHandles {
    pub fn from_channels(ch: &RuntimeChannels) -> Self {
        Self {
            cmd_tx: ch.cmd_tx.clone(),
            slice_rx: ch.slice_rx.clone(),
            slice_bus: ch.slice_bus.clone(),
            ack_rx: ch.ack_rx.clone(),
        }
    }
}

pub async fn run_comms(shutdown: ShutdownFlag, handles: CommsHandles, zenoh_endpoint: String) {
    zenoh_svc::run(shutdown, handles, zenoh_endpoint).await;
}
