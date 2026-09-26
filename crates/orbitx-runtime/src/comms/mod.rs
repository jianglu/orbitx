//! CommsService：本机 Zenoh + SHM（P4.3）。

pub mod zenoh_svc;

use crate::channel::RuntimeChannels;
use crate::shutdown::ShutdownFlag;

pub struct CommsHandles {
    pub cmd_tx: flume::Sender<crate::channel::RuntimeInbound>,
    pub slice_rx: flume::Receiver<std::sync::Arc<crate::slice::Slice>>,
}

impl CommsHandles {
    pub fn from_channels(ch: &RuntimeChannels) -> Self {
        Self {
            cmd_tx: ch.cmd_tx.clone(),
            slice_rx: ch.slice_rx.clone(),
        }
    }
}

pub async fn run_comms(shutdown: ShutdownFlag, handles: CommsHandles, zenoh_endpoint: String) {
    zenoh_svc::run(shutdown, handles, zenoh_endpoint).await;
}
