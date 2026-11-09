//! 本机 Zenoh + SHM Comms（官方 peer API；禁跨设备）。
//!
//! 事件驱动主循环：阻塞在「命令订阅」与「切片通道」两路接收上（`tokio::select!`），
//! 热路径无轮询 sleep——命令到达即刻转发、切片入队即刻发布。100ms 兜底心跳仅在
//! 空闲期触发（响应 shutdown + 周期诊断报告）。背景：原轮询版的
//! `tokio::time::sleep(2ms)` 在 Windows 上被 ~15.6ms 系统定时器 tick 钳制
//! （实测圈距 15.4ms），把 ClientStep 往返拖过 20ms 物理周期导致丢步 0.5x；
//! 事件驱动后 RTT 与定时器粒度解耦。

use std::sync::Arc;
use std::time::{Duration, Instant};

use tracing::{debug, error, info, warn};
use zenoh::bytes::ZBytes;
use zenoh::Config;

use crate::channel::RuntimeInbound;
use crate::comms::CommsHandles;
use crate::diag::{self, DurationWindow};
use crate::input::{InputCmd, SessionCmd};
use crate::shutdown::ShutdownFlag;
use crate::slice::Slice;
use orbitx_protocol::{
    self, encode_message, inbound::Payload as InboundPayload, input_cmd::Kind as InputKind,
    keyexpr, session_cmd::Kind as SessionKind, Inbound,
};

/// 空闲兜底心跳周期：shutdown 响应延迟上限。
const IDLE_TICK: Duration = Duration::from_millis(100);
/// 诊断窗口报告周期。
const REPORT_PERIOD: Duration = Duration::from_secs(5);
/// 命令处理延迟预算（decode + 转发；超阈说明编码/通道异常）。
const CMD_BUDGET: Duration = Duration::from_millis(1);
/// 切片发布延迟预算（排空 + 编码 + put；超阈说明发布路径异常）。
const SLICE_BUDGET: Duration = Duration::from_millis(2);
/// 诊断窗口容量（5s 报告周期内事件数上限的数倍，防驱逐失真）。
const WINDOW_CAP: usize = 8192;

/// 由 `--zenoh-endpoint` 得到本机 listen/connect locator。
pub fn local_locator(endpoint: &str) -> String {
    let ep = endpoint.trim();
    if ep.is_empty() || ep.eq_ignore_ascii_case("local") {
        return "tcp/127.0.0.1:17447".into();
    }
    let lower = ep.to_ascii_lowercase();
    if lower.starts_with("tcp/") || lower.starts_with("udp/") {
        return ep.to_string();
    }
    // 会话 id → 固定本机端口（哈希），避免跨设备。
    let mut h: u32 = 2166136261;
    for b in ep.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(16777619);
    }
    let port = 17000 + (h % 2000);
    format!("tcp/127.0.0.1:{port}")
}

pub fn build_host_config(endpoint: &str) -> Result<Config, String> {
    let locator = local_locator(endpoint);
    let mut config = Config::default();
    config
        .insert_json5("mode", r#""peer""#)
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("listen/endpoints", &format!(r#"["{locator}"]"#))
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|e| e.to_string())?;
    apply_local_transport(&mut config)?;
    Ok(config)
}

pub fn build_client_config(endpoint: &str) -> Result<Config, String> {
    let locator = local_locator(endpoint);
    let mut config = Config::default();
    config
        .insert_json5("mode", r#""peer""#)
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("connect/endpoints", &format!(r#"["{locator}"]"#))
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|e| e.to_string())?;
    apply_local_transport(&mut config)?;
    Ok(config)
}

/// TCP 回环上的本机传输：低延迟、共享内存在建会话时建好，大切片走共享内存。
fn apply_local_transport(config: &mut Config) -> Result<(), String> {
    let threshold = orbitx_protocol::keyexpr::SHM_MESSAGE_SIZE_THRESHOLD.to_string();
    config
        .insert_json5("transport/unicast/lowlatency", "true")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("transport/unicast/qos/enabled", "false")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("transport/shared_memory/enabled", "true")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("transport/shared_memory/mode", r#""init""#)
        .map_err(|e| e.to_string())?;
    config
        .insert_json5(
            "transport/shared_memory/transport_optimization/message_size_threshold",
            &threshold,
        )
        .map_err(|e| e.to_string())?;
    Ok(())
}

fn inbound_from_proto(msg: Inbound) -> Option<RuntimeInbound> {
    match msg.payload? {
        InboundPayload::Input(input) => {
            let cmd = match input.kind? {
                InputKind::SetThrottle(t) => InputCmd::SetThrottle { level: t.level },
                InputKind::SetAttitudeAxes(a) => InputCmd::SetAttitudeAxes {
                    pitch: a.pitch,
                    yaw: a.yaw,
                    roll: a.roll,
                },
                InputKind::Separate(_) => InputCmd::Separate,
                InputKind::SetGravityTurn(g) => InputCmd::SetGravityTurn { enabled: g.enabled },
                InputKind::WorkflowIgnite(_) => InputCmd::WorkflowIgnite,
                InputKind::SetRcsEnabled(r) => InputCmd::SetRcsEnabled { enabled: r.enabled },
                InputKind::SetAttitudeNose(n) => InputCmd::SetAttitudeNose {
                    azimuth: n.azimuth,
                    elevation: n.elevation,
                    roll: n.roll,
                },
            };
            Some(RuntimeInbound::Input(cmd))
        }
        InboundPayload::Session(session) => {
            let cmd = match session.kind? {
                SessionKind::Pause(_) => SessionCmd::Pause,
                SessionKind::Resume(_) => SessionCmd::Resume,
                SessionKind::SetWarp(w) => SessionCmd::SetWarp { scale: w.scale },
                SessionKind::Step(s) => SessionCmd::Step { n: s.n },
                SessionKind::Shutdown(_) => SessionCmd::Shutdown,
                SessionKind::Reset(_) => SessionCmd::Reset,
                SessionKind::SetSurfaceWorkingMaxLevel(s) => {
                    SessionCmd::SetSurfaceWorkingMaxLevel { level: s.level }
                }
                SessionKind::Launch(_) => SessionCmd::Launch,
            };
            Some(RuntimeInbound::Session(cmd))
        }
    }
}

/// keep-latest 收集：以 `head`（select 醒来时已持有的第一片）为起点排空切片
/// 队列，再合并旁路 latest 槽（队列 Full 时 runtime 写入的最新帧，优先于队尾）。
fn collect_latest_slice(handles: &CommsHandles, head: Option<Arc<Slice>>) -> Option<Arc<Slice>> {
    let mut latest = head;
    while let Ok(slice) = handles.slice_rx.try_recv() {
        latest = Some(slice);
    }
    if let Some(slot) = handles.slice_bus.take_latest_if_any() {
        latest = Some(slot);
    }
    latest
}

/// P4.5 门控不变量：同一 Arc（无新步）不得重发——Godot 以「收到切片」清
/// in_flight，重发旧片会把 50Hz Step 门控打穿（步进速率失控 + 渲染冻结）。
fn is_new_slice(prev: &Option<Arc<Slice>>, slice: &Arc<Slice>) -> bool {
    prev.as_ref()
        .map(|p| !Arc::ptr_eq(p, slice))
        .unwrap_or(true)
}

pub async fn run(shutdown: ShutdownFlag, handles: CommsHandles, zenoh_endpoint: String) {
    let config = match build_host_config(&zenoh_endpoint) {
        Ok(c) => c,
        Err(e) => {
            error!(error = %e, "zenoh host config failed");
            return;
        }
    };
    let session = match zenoh::open(config).await {
        Ok(s) => s,
        Err(e) => {
            error!(error = %e, "zenoh open failed");
            return;
        }
    };

    let cmd_key = keyexpr::cmd_key(&zenoh_endpoint);
    let slice_key = keyexpr::slice_key(&zenoh_endpoint);
    info!(
        endpoint = %zenoh_endpoint,
        listen = %local_locator(&zenoh_endpoint),
        cmd = %cmd_key,
        slice = %slice_key,
        "CommsService Zenoh started (local SHM)"
    );

    let subscriber = match session.declare_subscriber(cmd_key).await {
        Ok(s) => s,
        Err(e) => {
            error!(error = %e, "declare cmd subscriber failed");
            return;
        }
    };
    let publisher = match session.declare_publisher(slice_key).await {
        Ok(p) => p,
        Err(e) => {
            error!(error = %e, "declare slice publisher failed");
            return;
        }
    };
    let mut last_published: Option<Arc<Slice>> = None;

    // 诊断窗口：handler 延迟实测 + 事件计数（窗口随报告 reset，count 即事件数）。
    let mut cmd_us = DurationWindow::new(WINDOW_CAP, CMD_BUDGET);
    let mut slice_us = DurationWindow::new(WINDOW_CAP, SLICE_BUDGET);
    let mut last_report = Instant::now();
    // 兜底心跳：空闲期（无命令/无切片）唯一唤醒源。
    let mut idle_tick = tokio::time::interval(IDLE_TICK);
    // slice_rx 断开后禁用该 select 分支——flume 断开通道的 recv_async 立即返回
    // Err，若继续参与 select 会退化为忙循环；cmd 路径仍存活，comms 不退出。
    let mut slices_alive = true;

    loop {
        tokio::select! {
            // ① 命令到达：即刻解码转发（轮询版要等下一圈，现零等待）。
            sample = subscriber.recv_async() => {
                match sample {
                    Ok(sample) => {
                        let t0 = Instant::now();
                        let bytes = sample.payload().to_bytes();
                        match orbitx_protocol::decode_inbound(&bytes) {
                            Ok(msg) => {
                                if let Some(inbound) = inbound_from_proto(msg) {
                                    if let Err(e) = handles.cmd_tx.send(inbound) {
                                        warn!(error = %e, "cmd channel closed");
                                    }
                                }
                            }
                            Err(e) => warn!(error = %e, "bad inbound protobuf"),
                        }
                        cmd_us.push(t0.elapsed());
                    }
                    Err(e) => {
                        // 订阅端关闭 = zenoh 会话终止：comms 使命结束。
                        info!(error = %e, "cmd subscriber closed; comms exiting");
                        break;
                    }
                }
            }
            // ② 切片入队：即刻排空至最新并发布（原 sleep(2ms)/15.4ms 等待已移除）。
            head = handles.slice_rx.recv_async(), if slices_alive => {
                match head {
                    Ok(first) => {
                        let t0 = Instant::now();
                        let latest = collect_latest_slice(&handles, Some(first));
                        if let Some(slice) = latest {
                            if is_new_slice(&last_published, &slice) {
                                last_published = Some(slice.clone());
                                let proto = slice.to_proto();
                                let bytes = encode_message(&proto);
                                if let Err(e) = publisher.put(ZBytes::from(bytes)).await {
                                    debug!(error = %e, "slice put failed");
                                }
                            }
                        }
                        slice_us.push(t0.elapsed());
                    }
                    Err(_) => slices_alive = false,
                }
            }
            // ③ 兜底心跳：仅在空闲期触发——防睡死 + 周期诊断报告。
            _ = idle_tick.tick() => {
                if shutdown.is_requested() {
                    break;
                }
                if last_report.elapsed() >= REPORT_PERIOD {
                    last_report = Instant::now();
                    if let Some(st) = cmd_us.snapshot() {
                        diag::log_window("comms cmd handler window (5s)", &st, CMD_BUDGET);
                    }
                    if let Some(st) = slice_us.snapshot() {
                        diag::log_window("comms slice publish window (5s)", &st, SLICE_BUDGET);
                    }
                    cmd_us.reset();
                    slice_us.reset();
                }
            }
        }
    }

    // 收尾：清空残留切片。
    while handles.slice_rx.try_recv().is_ok() {}
    let _ = handles.slice_bus.take_latest_if_any();
    info!("CommsService Zenoh stopped");
}

#[cfg(test)]
mod tests;
