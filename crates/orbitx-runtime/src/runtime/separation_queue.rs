//! 分离事件 TTL 队列（P45 拆级视觉可靠性）：Assembly 产生的分离事件不再
//! 「取走即随当步切片一次性下发」，而是入队后每片切片携带全量快照，直至
//! 出队窗口（[`dequeue_window_steps`]）超时。
//!
//! 根因与设计（2026-11-07）：P3.14 ClientStep 倍速引入 burst × 三层
//! keep-latest 单槽（`slice_bus` / 桥 ready 槽 / `last_slice` replace），
//! 瞬态事件片可被覆盖丢失 → Godot 拆级视觉缺失。TTL 重发 + 消费端 vi 哨兵
//! 去重（单一容器，事件源与翻沿兜底两序等价），**无握手无 ack**：队列语义
//! 「有就返回，没有是空」，消费 = 消息确定送达 Godot（不被倍速切片吞掉）。
//!
//! 判据锚**物理步进时间戳**（步号差），不锚墙钟：暂停期步不走表，窗口不
//! 推进，事件不蒸发；倍速（warp）只影响窗口折算的墙钟时长。

use orbitx_protocol::mem::SeparationEventTelem;

use super::clock::Clock;

/// 单次分离事件：一次 undock 拆走的 vessel 集合 + 产生物理步。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeparationEvent {
    /// 事件产生物理步步号（`Clock::step_index` 口径）。
    pub born_step: u64,
    /// 被拆走的 vessel 下标（`Assembly.vessels` 下标口径）。
    pub vessels: Vec<usize>,
}

/// 队列容量上限：正常路径一步至多一个事件（一次 undock）；cap 满说明消费
/// 端长期失联，新事件并入最早事件（vessels 并集、born 取最早）——事件不丢、
/// 队列不无界增长，且最早 born 仍是最早产生时刻（消费端哨兵按 vi 去重，
/// 合并不产生重复拆级）。
const QUEUE_CAP: usize = 8;

#[derive(Debug, Default)]
pub struct SeparationQueue {
    events: Vec<SeparationEvent>,
}

impl SeparationQueue {
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }

    /// 新产生事件入队。空事件忽略；同 born_step 合并（一步多次 undock）；
    /// cap 满并入最早事件（born 取最早、vessels 并集）。
    pub fn enqueue(&mut self, born_step: u64, vessels: Vec<usize>) {
        if vessels.is_empty() {
            return;
        }
        let mut vessels = vessels;
        vessels.sort_unstable();
        vessels.dedup();
        // 事件按 born_step 升序追加（步号单调递增）：last = 最新产生。
        if let Some(last) = self.events.last_mut() {
            if last.born_step == born_step {
                merge_vessels(&mut last.vessels, &vessels);
                return;
            }
        }
        if self.events.len() >= QUEUE_CAP {
            let first = &mut self.events[0];
            merge_vessels(&mut first.vessels, &vessels);
            return;
        }
        self.events.push(SeparationEvent { born_step, vessels });
    }

    /// 出队：`current_step - born_step` 达窗口（[`dequeue_window_steps`]）即
    /// 移除。步进时间戳差判据（非墙钟过期）——暂停期窗口不推进。
    pub fn prune(&mut self, current_step: u64, warp: f64) {
        let window = dequeue_window_steps(warp);
        self.events
            .retain(|e| current_step.saturating_sub(e.born_step) < window);
    }

    /// 切片快照（协议遥测）：born_step → born_t_ms 由 [`Clock`] 权威派生。
    pub fn telem(&self, clock: &Clock) -> Vec<SeparationEventTelem> {
        self.events
            .iter()
            .map(|e| SeparationEventTelem {
                born_t_ms: born_t_ms_of(e.born_step, clock),
                vessels: e.vessels.iter().map(|&i| i as u32).collect(),
            })
            .collect()
    }

    /// 旧字段（`separated_vessels`）口径：队列全部事件 vessels 并集（升序
    /// 去重）。旧 bridge 拿并集一次拆全部，行为优于旧「仅当步」瞬态语义。
    pub fn all_vessels(&self) -> Vec<u32> {
        let mut out: Vec<u32> = Vec::new();
        for e in &self.events {
            for &i in &e.vessels {
                let vi = i as u32;
                if !out.contains(&vi) {
                    out.push(vi);
                }
            }
        }
        out.sort_unstable();
        out
    }

    /// 队列快照（黑匣子产生步判定 / 诊断用）。
    pub fn snapshot(&self) -> &[SeparationEvent] {
        &self.events
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// 会话 Reset 清空（新会话无历史事件）。
    pub fn clear(&mut self) {
        self.events.clear();
    }
}

/// 出队窗口（步）：覆盖 Godot 端每帧最大欠债钳制（8×warp 步，P3.14 倍速
/// burst）+ 余量 8 步——窗口恒 ≥ 欠债钳 1 帧，任一片丢失必有一片后续补发。
/// 折算墙钟时长 = 8×sim_dt + 8×sim_dt/warp：warp=1 约 320ms，高倍速收敛
/// ≈160ms（余量项主导）；低速档更长无妨——窗口只决定「多久后放弃重发」，
/// TTL 重发语义下宁长勿短。
pub fn dequeue_window_steps(warp: f64) -> u64 {
    (8.0 * warp.max(1.0)).ceil() as u64 + 8
}

/// born_step → 产生时刻 [ms]（`Clock` 权威派生：sim_t = sim_dt × step 线性
/// 走表，saturating 防回绕）。
pub fn born_t_ms_of(born_step: u64, clock: &Clock) -> u64 {
    clock
        .sim_t_ms()
        .saturating_sub(clock.step_index().saturating_sub(born_step) * clock.sim_dt_ms())
}

/// vessels 并集（就地合并，保持 dst 升序去重）。
fn merge_vessels(dst: &mut Vec<usize>, src: &[usize]) {
    for &v in src {
        if !dst.contains(&v) {
            dst.push(v);
        }
    }
    dst.sort_unstable();
    dst.dedup();
}

#[cfg(test)]
mod tests;
