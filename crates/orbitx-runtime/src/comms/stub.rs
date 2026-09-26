//! 本地 stub：不接 Zenoh；drain Slice；仅本机 SHM 语义占位。

use std::time::Duration;

use tracing::{debug, info};

use crate::comms::CommsHandles;
use crate::shutdown::ShutdownFlag;

pub async fn run(shutdown: ShutdownFlag, handles: CommsHandles, zenoh_endpoint: String) {
    info!(
        endpoint = %zenoh_endpoint,
        mode = "local-shm-only",
        "CommsService stub started (no Zenoh yet; cross-device unsupported)"
    );
    while !shutdown.is_requested() {
        while let Ok(slice) = handles.slice_rx.try_recv() {
            debug!(sim_t = slice.sim_t, step = slice.step_index, "stub drop slice");
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    while handles.slice_rx.try_recv().is_ok() {}
    info!("CommsService stub stopped");
}
