//! FlightRecorder：L1 入队 + IO tokio L2/L3 stub（格式见 FLIGHT_RECORDER.md）。

mod cache;

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use flume::{Receiver, Sender};
use tracing::{info, warn};

use crate::shutdown::ShutdownFlag;
use crate::slice::Slice;
use crate::world::World;

pub use cache::{RecorderEvent, RECORDER_L1_CAPACITY};

/// Runtime 热路径持有的入队句柄（非阻塞）。
#[derive(Clone)]
pub struct RecorderEnqueue {
    tx: Sender<RecorderEvent>,
    dropped: Arc<AtomicU64>,
}

impl RecorderEnqueue {
    pub fn try_enqueue_session_start(&self, sim_t: u64, world: &World) -> bool {
        self.try_send(RecorderEvent::SessionStart {
            sim_t,
            world_label: world.label.clone(),
        })
    }

    pub fn try_enqueue_step(&self, slice: Arc<Slice>) -> bool {
        self.try_send(RecorderEvent::Step { slice })
    }

    fn try_send(&self, ev: RecorderEvent) -> bool {
        match self.tx.try_send(ev) {
            Ok(()) => true,
            Err(flume::TrySendError::Full(_)) => {
                // 不丢帧策略：L1 满时由调用方应反压会话；骨架阶段计数并仍尝试阻塞短送会堵热路径。
                // 文档：满则 L2 放大 / 会话反压。此处用 `send` 会阻塞 Runtime——改为记 pending 并 warn，
                // 完整 L2 spill → P6。当前用独立大容量 L1，测试验证 drain。
                self.dropped.fetch_add(1, Ordering::Relaxed);
                warn!("recorder L1 full (skeleton counts; stage B adds L2 spill)");
                false
            }
            Err(flume::TrySendError::Disconnected(_)) => false,
        }
    }

    pub fn dropped_count(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

pub struct RecorderIo {
    rx: Receiver<RecorderEvent>,
    out_dir: PathBuf,
    written: Arc<AtomicU64>,
}

impl RecorderIo {
    pub fn pair(out_dir: PathBuf) -> (RecorderEnqueue, Self) {
        let (tx, rx) = flume::bounded(RECORDER_L1_CAPACITY);
        let written = Arc::new(AtomicU64::new(0));
        (
            RecorderEnqueue {
                tx,
                dropped: Arc::new(AtomicU64::new(0)),
            },
            Self {
                rx,
                out_dir,
                written,
            },
        )
    }

    pub fn written_count(&self) -> u64 {
        self.written.load(Ordering::Relaxed)
    }

    /// IO tokio 任务：drain 事件（当前：计数 stub；**P6**：CBOR+zstd 段文件）。
    pub async fn run(self, shutdown: ShutdownFlag) {
        let _ = std::fs::create_dir_all(&self.out_dir);
        info!(dir = %self.out_dir.display(), "FlightRecorder IO task started");
        loop {
            tokio::select! {
                biased;
                _ = async {
                    while !shutdown.is_requested() {
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                } => {
                    while let Ok(ev) = self.rx.try_recv() {
                        self.handle(ev);
                    }
                    break;
                }
                msg = self.rx.recv_async() => {
                    match msg {
                        Ok(ev) => self.handle(ev),
                        Err(_) => break,
                    }
                }
            }
        }
        info!(
            written = self.written.load(Ordering::Relaxed),
            "FlightRecorder IO task stopped"
        );
    }

    fn handle(&self, ev: RecorderEvent) {
        match &ev {
            RecorderEvent::SessionStart { .. } | RecorderEvent::Step { .. } => {}
        }
        self.written.fetch_add(1, Ordering::Relaxed);
        let _ = ev;
    }
}
