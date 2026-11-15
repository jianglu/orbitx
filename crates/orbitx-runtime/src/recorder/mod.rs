//! FlightRecorder：L1 非阻塞入队（水位前置反压）→ IO tokio 落盘。
//!
//! 架构见 `docs/RUNTIME.md`；磁盘格式见 `docs/FLIGHT_RECORDER.md`。
//! - L1：`RecorderEnqueue`——Runtime 热路径在 **tick 前**查水位（[`RecorderEnqueue::accepts`]），
//!   超限则推迟积步（待步计数保留，不丢步）；载荷直接是
//!   [`orbitx_recorder::RecorderFrame`]（格式层枚举，无第二层事件抽象）。
//! - IO：[`RecorderIo`]——纯接线（事件 → `SessionWriter`），无格式知识；
//!   写失败置 [`RecorderHealth`]，Runtime 侧据此暂停积步（极端耗尽显式失败）。
//! - `--recorder-enabled=false` 时入队恒 no-op，不启 IO 任务、不建目录。

pub mod frame;

use std::hash::{BuildHasher, Hasher};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use flume::{Receiver, Sender};
use orbitx_recorder::{RecorderFrame, SessionMeta, SessionWriter};
use tracing::{error, info, warn};

use crate::shutdown::ShutdownFlag;

/// L1 有界容量。
pub const RECORDER_L1_CAPACITY: usize = 4096;
/// 反压高水位：占用超过该值时 Runtime 推迟积步（保留待步数，不丢步）。
pub const RECORDER_HIGH_WATERMARK: usize = RECORDER_L1_CAPACITY * 3 / 4;

/// L1 队列消息：载荷帧 + 停机收尾控制命令（C3）。
///
/// **控制消息不进磁盘格式枚举 [`orbitx_recorder::RecorderFrame`]**（格式层
/// 纯度，`SCHEMA_VERSION` 不动）；`HaltFlush` 处理 = drain 残余帧（保序）→
/// **评分快照帧落盘（C4；随本命令携带，drain 之后、`finish` 之前——不可被
/// 反压丢弃 + 保序：step 帧先落）** → `SessionWriter::finish` →
/// 置 `writer = None`（后续步帧走「无会话头丢弃」warn 路径）→ `reply` 回
/// 实际会话目录。
#[derive(Debug)]
pub enum RecorderCmd {
    Frame(RecorderFrame),
    HaltFlush {
        /// 应答通道（bounded(1)）：回实际会话目录（绝对路径字符串）。
        reply: Sender<String>,
        /// 评分快照帧（C4）：`kind="scoring"` 至多一帧；未启用录制为 `None`
        /// （Runtime 侧照常构建，载荷随丢弃路径自然作废）。
        scoring: Option<RecorderFrame>,
    },
}

/// IO 侧健康标志（写盘失败置位；Runtime tick 前读，N1 机制）。
#[derive(Debug, Default)]
pub struct RecorderHealth {
    failed: AtomicBool,
}

impl RecorderHealth {
    fn fail(&self) {
        self.failed.store(true, Ordering::Relaxed);
    }

    pub fn failed(&self) -> bool {
        self.failed.load(Ordering::Relaxed)
    }
}

/// Runtime 热路径持有的入队句柄（非阻塞）。
#[derive(Clone)]
pub struct RecorderEnqueue {
    tx: Option<Sender<RecorderCmd>>,
    health: Arc<RecorderHealth>,
    dropped: Arc<AtomicU64>,
}

impl RecorderEnqueue {
    /// 关闭黑匣子：热路径入队恒为 no-op，水位恒放行；停机收尾直接回空目录。
    pub fn disabled() -> Self {
        Self {
            tx: None,
            health: Arc::new(RecorderHealth::default()),
            dropped: Arc::new(AtomicU64::new(0)),
        }
    }

    /// tick 前水位检查：true = 可继续步进入队；false = 推迟本步（不丢帧、不丢步）。
    pub fn accepts(&self) -> bool {
        self.queued_len() < RECORDER_HIGH_WATERMARK
    }

    /// L1 当前积压帧数（测试与诊断用）。
    pub fn queued_len(&self) -> usize {
        self.tx.as_ref().map_or(0, Sender::len)
    }

    /// IO 写盘失败（极端耗尽）：Runtime 应暂停积步并显式上报。
    pub fn healthy(&self) -> bool {
        !self.health.failed()
    }

    /// 入队一帧（session_start / step 通用）。队满（竞态兜底）返回 false。
    pub fn try_enqueue(&self, frame: RecorderFrame) -> bool {
        self.try_send_cmd(RecorderCmd::Frame(frame))
    }

    /// 停机收尾控制消息：**绕过水位检查**（控制消息不可被反压丢弃）；
    /// 队满（竞态兜底）返回 false 由调用方重试。未启用录制时直接回空目录
    /// （评分快照载荷随丢弃路径作废）。重试路径载荷按值重发——调用方 `clone`。
    pub fn request_halt_flush(&self, reply: Sender<String>, scoring: Option<RecorderFrame>) -> bool {
        let Some(tx) = self.tx.as_ref() else {
            let _ = reply.send(String::new());
            return true;
        };
        match tx.try_send(RecorderCmd::HaltFlush { reply, scoring }) {
            Ok(()) => true,
            Err(flume::TrySendError::Full(_)) => false,
            Err(flume::TrySendError::Disconnected(_)) => false,
        }
    }

    fn try_send_cmd(&self, cmd: RecorderCmd) -> bool {
        let Some(tx) = self.tx.as_ref() else {
            return true;
        };
        match tx.try_send(cmd) {
            Ok(()) => true,
            Err(flume::TrySendError::Full(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
                false
            }
            Err(flume::TrySendError::Disconnected(_)) => false,
        }
    }

    pub fn dropped_count(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    pub fn is_enabled(&self) -> bool {
        self.tx.is_some()
    }
}

/// 会话目录基名：`<火箭ID>_<hex4>_<UTC 年-月-日_时分秒>`（例
/// `Test_1_a3f2_2026-11-03_191345`）。火箭名净化为目录安全字符；同秒冲突由
/// `SessionWriter::create` 追加 `-2`… 后缀兜底。
pub fn new_session_id(rocket_name: &str, now_unix_ms: u64) -> String {
    let secs = now_unix_ms / 1000;
    let (y, m, d) = civil_from_days((secs / 86_400) as i64);
    let s = secs % 86_400;
    let entropy = std::collections::hash_map::RandomState::new()
        .build_hasher()
        .finish();
    let rocket = rocket_name.trim();
    let rocket = if rocket.is_empty() { "rocket" } else { rocket };
    let mut safe = String::with_capacity(rocket.len());
    for c in rocket.chars() {
        if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
            safe.push(c);
        } else {
            safe.push('_');
        }
    }
    format!(
        "{safe}_{hex:04x}_{y:04}-{m:02}-{d:02}_{h:02}{mi:02}{ss:02}",
        hex = (entropy & 0xffff) as u16,
        h = s / 3600,
        mi = s % 3600 / 60,
        ss = s % 60,
    )
}

/// Howard Hinnant `civil_from_days`（天数 → (年, 月, 日)，proleptic Gregorian）。
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (if m <= 2 { y + 1 } else { y }, m as u32, d)
}

/// IO tokio 持有的落盘端：drain L1 → `SessionWriter`（建/封段/manifest 全在 lib）。
pub struct RecorderIo {
    rx: Receiver<RecorderCmd>,
    root: PathBuf,
    session_base: String,
    meta: SessionMeta,
    writer: Option<SessionWriter>,
    health: Arc<RecorderHealth>,
    written: Arc<AtomicU64>,
}

impl RecorderIo {
    /// 构建入队/落盘对。`meta` 携带 manifest 头信息（drive / sim_dt），帧事件不携带。
    /// `root` 已含子树前缀（host 层按 `recorder_subtree` 拼接）。
    pub fn pair(root: PathBuf, session_base: String, meta: SessionMeta) -> (RecorderEnqueue, Self) {
        let (tx, rx) = flume::bounded(RECORDER_L1_CAPACITY);
        let health = Arc::new(RecorderHealth::default());
        (
            RecorderEnqueue {
                tx: Some(tx),
                health: Arc::clone(&health),
                dropped: Arc::new(AtomicU64::new(0)),
            },
            Self {
                rx,
                root,
                session_base,
                meta,
                writer: None,
                health,
                written: Arc::new(AtomicU64::new(0)),
            },
        )
    }

    pub fn written_count(&self) -> u64 {
        self.written.load(Ordering::Relaxed)
    }

    /// IO tokio 任务：drain 事件；shutdown 后清空尾巴并 `finish`（终版 manifest）。
    pub async fn run(mut self, shutdown: ShutdownFlag) {
        info!(
            root = %self.root.display(),
            "FlightRecorder IO task started"
        );
        loop {
            tokio::select! {
                biased;
                _ = async {
                    while !shutdown.is_requested() {
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                } => {
                    while let Ok(cmd) = self.rx.try_recv() {
                        self.handle_cmd(cmd);
                    }
                    break;
                }
                msg = self.rx.recv_async() => {
                    match msg {
                        Ok(cmd) => self.handle_cmd(cmd),
                        Err(_) => break,
                    }
                }
            }
        }
        self.finalize();
        info!(
            written = self.written.load(Ordering::Relaxed),
            "FlightRecorder IO task stopped"
        );
    }

    /// 收尾：终版 manifest（幂等）。run 退出与测试直接调用。
    fn finalize(&mut self) {
        if let Some(w) = self.writer.as_mut() {
            if let Err(e) = w.finish() {
                error!(error = %e, "FlightRecorder finish failed");
                self.health.fail();
            } else {
                info!(
                    dir = %w.dir().display(),
                    frames = w.frames_written(),
                    "FlightRecorder session finalized"
                );
            }
        }
    }

    /// 消息 → 状态机。Frame 走既有逻辑；HaltFlush = 停机收尾五步
    /// （drain 残余帧保序 → 评分快照帧落盘（C4，drain 后 finish 前——
    /// step 帧先落 + 快照帧不可被反压丢弃）→ finish → 置 None → reply 回
    /// 实际目录）。
    fn handle_cmd(&mut self, cmd: RecorderCmd) {
        match cmd {
            RecorderCmd::Frame(frame) => {
                self.handle_frame(frame);
            }
            RecorderCmd::HaltFlush { reply, scoring } => {
                // 保序：drain 队内残余步帧先落盘（此前 step 帧先于收尾）。
                while let Ok(next) = self.rx.try_recv() {
                    match next {
                        RecorderCmd::Frame(f) => {
                            self.handle_frame(f);
                        }
                        RecorderCmd::HaltFlush {
                            reply: r2,
                            scoring: s2,
                        } => {
                            // 并发第二收尾（不该发生）：回空目录兜底。
                            drop(s2);
                            let _ = r2.send(String::new());
                            break;
                        }
                    }
                }
                // C4：评分快照帧（至多一帧）在 step 帧之后、finish 之前落盘。
                if let Some(sf) = scoring {
                    if !self.handle_frame(sf) {
                        warn!("FlightRecorder scoring frame not written (no session or failed)");
                    }
                }
                let dir = match self.writer.take() {
                    Some(mut w) => {
                        let dir = w.dir().to_string_lossy().into_owned();
                        if let Err(e) = w.finish() {
                            error!(error = %e, "FlightRecorder halt flush finish failed");
                            self.health.fail();
                        } else {
                            info!(
                                dir = %dir,
                                frames = w.frames_written(),
                                "FlightRecorder halt flush finalized"
                            );
                        }
                        dir
                    }
                    None => String::new(),
                };
                // writer 已置 None：后续步帧走「无会话头丢弃」warn 路径，
                // 不再打已关闭 writer 的 WriterClosed。
                let _ = reply.send(dir);
            }
        }
    }

    /// 帧 → 写状态机。`session_start` 开新会话目录（Reset 时旧会话先 finish）。
    /// 返回帧是否被落盘（false = 无会话头丢弃 / 写失败）。
    fn handle_frame(&mut self, frame: RecorderFrame) -> bool {
        match &frame {
            RecorderFrame::SessionStart(_) => {
                if let Some(mut old) = self.writer.take() {
                    // Reset：同进程重开会话 → 旧会话收尾，新目录见 create。
                    if let Err(e) = old.finish() {
                        error!(error = %e, "FlightRecorder previous session finish failed");
                        self.health.fail();
                    }
                }
                match SessionWriter::create(
                    &self.root,
                    &self.session_base,
                    self.meta.clone(),
                    Default::default(),
                ) {
                    Ok(mut w) => {
                        info!(dir = %w.dir().display(), "FlightRecorder session opened");
                        if let Err(e) = w.push_frame(&frame) {
                            error!(error = %e, "FlightRecorder session_start write failed");
                            self.health.fail();
                            return false;
                        }
                        self.writer = Some(w);
                    }
                    Err(e) => {
                        error!(error = %e, "FlightRecorder session create failed");
                        self.health.fail();
                        return false;
                    }
                }
            }
            RecorderFrame::Step(_) => match self.writer.as_mut() {
                Some(w) => {
                    if let Err(e) = w.push_frame(&frame) {
                        error!(error = %e, "FlightRecorder frame write failed");
                        self.health.fail();
                        return false;
                    }
                }
                None => {
                    // 无会话头的步帧：session_start 未入队，或停机收尾后不再
                    // 落盘（HaltFlush 置 None；优雅降级，非错误）。
                    warn!("FlightRecorder step frame without session_start; dropped");
                    return false;
                }
            },
            // C4 评分快照帧：与步帧同写路径（会话头存在则落盘）；无会话头 =
            // 停机收尾后才开会话（不该发生）或写失败，丢弃留痕。
            RecorderFrame::Scoring(_) => match self.writer.as_mut() {
                Some(w) => {
                    if let Err(e) = w.push_frame(&frame) {
                        error!(error = %e, "FlightRecorder scoring frame write failed");
                        self.health.fail();
                        return false;
                    }
                }
                None => {
                    warn!("FlightRecorder scoring frame without session_start; dropped");
                    return false;
                }
            },
            // 未知 kind（旧 IO 读新格式不该发生——IO 只写本进程产生的帧）：
            // 不落盘、不报错（与读方「跳过」铁律同语义）。
            RecorderFrame::Unknown => {
                warn!("FlightRecorder unknown frame kind; skipped");
                return false;
            }
        }
        self.written.fetch_add(1, Ordering::Relaxed);
        true
    }
}

#[cfg(test)]
mod tests;
