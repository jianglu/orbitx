//! 节奏/延迟诊断工具（crate 级，runtime 与 comms 平级共用）。
//!
//! - `DurationWindow`：环形滑动窗口时长统计（count/avg/max/over_threshold）；
//! - `probe_sleep`：一次性实测 sleep 真实粒度——Windows 定时器精度按进程隔离
//!   （Win10 2004+ / Win11 / Server 2022+），进程未调 `timeBeginPeriod` 时
//!   tokio 等待原语的 1ms 可能实睡 ~15.6ms；
//! - `log_window`：统一窗口报告出口，各埋点站点共用同一 `info!` 格式，
//!   报告职责收敛于本模块（避免各处复制同构的 snapshot→info!→reset 块）。

use std::collections::VecDeque;
use std::time::Duration;

use tracing::info;

/// 环形滑动窗口时长统计：push O(1)（触顶驱逐最老样本并同步修正聚合值）。
#[derive(Debug)]
pub struct DurationWindow {
    cap: usize,
    samples: VecDeque<Duration>,
    sum: Duration,
    max: Duration,
    over: u64,
    threshold: Duration,
}

/// 一次窗口快照（count/avg/max/over 均为窗口内语义）。
#[derive(Debug, Clone, Copy)]
pub struct WindowStats {
    pub count: u64,
    pub avg: Duration,
    pub max: Duration,
    /// 超过窗口阈值（如步进预算 sim_dt）的样本数。
    pub over: u64,
}

impl DurationWindow {
    pub fn new(cap: usize, threshold: Duration) -> Self {
        let cap = cap.max(1);
        Self {
            cap,
            samples: VecDeque::with_capacity(cap),
            sum: Duration::ZERO,
            max: Duration::ZERO,
            over: 0,
            threshold,
        }
    }

    pub fn push(&mut self, d: Duration) {
        if self.samples.len() >= self.cap {
            if let Some(old) = self.samples.pop_front() {
                self.sum = self.sum.saturating_sub(old);
                if old > self.threshold {
                    self.over = self.over.saturating_sub(1);
                }
                if old == self.max {
                    self.max = self.samples.iter().copied().max().unwrap_or(Duration::ZERO);
                }
            }
        }
        if d > self.max {
            self.max = d;
        }
        if d > self.threshold {
            self.over += 1;
        }
        self.sum += d;
        self.samples.push_back(d);
    }

    /// 快照；窗口无样本返回 None。快照不清空，须显式 `reset`。
    pub fn snapshot(&self) -> Option<WindowStats> {
        let count = self.samples.len() as u64;
        if count == 0 {
            return None;
        }
        Some(WindowStats {
            count,
            avg: self.sum / count as u32,
            max: self.max,
            over: self.over,
        })
    }

    pub fn reset(&mut self) {
        self.samples.clear();
        self.sum = Duration::ZERO;
        self.max = Duration::ZERO;
        self.over = 0;
    }
}

/// 统一窗口报告出口：一条 `info!` 输出 count/avg/max/over 与阈值。
/// `label` 承载站点语义（如 "step cost window (5s)"）。
pub fn log_window(label: &str, stats: &WindowStats, threshold: Duration) {
    info!(
        count = stats.count,
        avg_us = stats.avg.as_micros() as u64,
        max_us = stats.max.as_micros() as u64,
        over = stats.over,
        over_threshold_us = threshold.as_micros() as u64,
        "{label}"
    );
}

/// sleep 粒度探针结果：`target` 时长的真实耗时均值 / 最大值。
#[derive(Debug, Clone, Copy)]
pub struct SleepProbe {
    pub avg: Duration,
    pub max: Duration,
}

/// 实测 `target` 时长 sleep 的真实耗时（`iters` 次，取 avg/max）。
/// `sleep` 为注入点（生产传 `std::thread::sleep`，测试可注入节流版）。
pub fn probe_sleep(sleep: impl FnMut(Duration), target: Duration, iters: u32) -> SleepProbe {
    let mut window = DurationWindow::new(iters.max(1) as usize, target);
    let mut sleep = sleep;
    for _ in 0..iters.max(1) {
        let t0 = std::time::Instant::now();
        sleep(target);
        window.push(t0.elapsed());
    }
    let st = window.snapshot().expect("probe pushed >= 1 sample");
    SleepProbe {
        avg: st.avg,
        max: st.max,
    }
}

#[cfg(test)]
mod tests;
