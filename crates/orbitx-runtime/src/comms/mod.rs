//! CommsService：P4.2 stub；P4.3 换本机 Zenoh + SHM。

pub mod stub;

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

pub async fn run_comms_stub(shutdown: ShutdownFlag, handles: CommsHandles, zenoh_endpoint: String) {
    stub::run(shutdown, handles, zenoh_endpoint).await;
}
