//! `SeparationQueue` 行为契约：入队合并 / TTL 窗口出队 / 快照派生 / 清空。

use super::{born_t_ms_of, dequeue_window_steps, SeparationQueue};
use crate::runtime::clock::Clock;

/// helpers：入队并返回队列（链式断言可读性）。
fn q(enqueue: &[(u64, Vec<usize>)]) -> SeparationQueue {
    let mut q = SeparationQueue::new();
    for (step, vessels) in enqueue {
        q.enqueue(*step, vessels.clone());
    }
    q
}

#[test]
fn empty_enqueue_is_ignored() {
    let mut queue = SeparationQueue::new();
    queue.enqueue(5, Vec::new());
    assert!(queue.is_empty());
    assert!(queue.all_vessels().is_empty());
}

/// 同一物理步的多次 undock 合并为单事件（vessels 并集去重）。
#[test]
fn same_step_events_merge_with_dedup() {
    let mut queue = SeparationQueue::new();
    queue.enqueue(10, vec![1, 3]);
    queue.enqueue(10, vec![3, 2]);
    let snap = queue.snapshot();
    assert_eq!(snap.len(), 1);
    assert_eq!(snap[0].born_step, 10);
    // 升序去重口径。
    assert_eq!(snap[0].vessels, vec![1, 2, 3]);
}

/// 不同步不合并：事件按产生步独立保留（TTL 窗口各自计算）。
#[test]
fn distinct_steps_stay_separate() {
    let queue = q(&[(10, vec![1]), (11, vec![2])]);
    let snap = queue.snapshot();
    assert_eq!(snap.len(), 2);
    assert_eq!(snap[0].vessels, vec![1]);
    assert_eq!(snap[1].vessels, vec![2]);
}

/// cap 满并入最早事件：born 取最早、vessels 并集——事件不丢、队列不增长。
#[test]
fn cap_full_merges_into_earliest() {
    let mut queue = SeparationQueue::new();
    for step in 0..8u64 {
        queue.enqueue(step, vec![step as usize]);
    }
    assert_eq!(queue.snapshot().len(), 8);
    // 第 9 个事件：并入最早（born_step=0），队列长度不变。
    queue.enqueue(8, vec![100]);
    let snap = queue.snapshot();
    assert_eq!(snap.len(), 8, "cap must not grow past 8");
    assert_eq!(snap[0].born_step, 0, "merged event keeps earliest born");
    assert!(snap[0].vessels.contains(&100));
}

/// 窗口出队：`current - born < window` 保留，`>= window` 出队（步进判据）。
#[test]
fn prune_dequeues_after_window() {
    let mut queue = q(&[(0, vec![1]), (5, vec![2])]);
    // warp=1 → 窗口 16：born=0 在 current=15 保留，current=16 出队。
    queue.prune(15, 1.0);
    assert_eq!(queue.snapshot().len(), 2);
    queue.prune(16, 1.0);
    let snap = queue.snapshot();
    assert_eq!(snap.len(), 1);
    assert_eq!(snap[0].born_step, 5);
}

/// 暂停语义：步不走表 → current 不变 → 窗口不推进，事件不蒸发。
#[test]
fn prune_at_same_step_keeps_events() {
    let mut queue = q(&[(10, vec![1])]);
    for _ in 0..100 {
        queue.prune(10, 1.0);
    }
    assert_eq!(queue.snapshot().len(), 1);
}

/// 窗口随 warp 折算：warp=32 → 8×32+8；warp 下限按 1 钳（0.125 慢速 burst
/// 更小，用 warp=1 的窗口宁长勿短）。
#[test]
fn window_scales_with_warp() {
    assert_eq!(dequeue_window_steps(1.0), 16);
    assert_eq!(dequeue_window_steps(4.0), 40);
    assert_eq!(dequeue_window_steps(32.0), 264);
    assert_eq!(dequeue_window_steps(0.125), 16);
    assert_eq!(dequeue_window_steps(2.5), 28);
}

/// born_t_ms 权威派生：sim_t = sim_dt × step 线性走表，回溯精确。
#[test]
fn telem_derives_born_t_ms_from_clock() {
    let mut clock = Clock::new(20);
    for _ in 0..10 {
        clock.advance_fixed_step();
    }
    assert_eq!(clock.sim_t_ms(), 200);
    assert_eq!(clock.step_index(), 10);

    let mut queue = SeparationQueue::new();
    queue.enqueue(3, vec![0, 2]);
    queue.enqueue(10, vec![1]);
    // 派生函数直证（telem 同源）。
    assert_eq!(born_t_ms_of(3, &clock), 60);
    let telem = queue.telem(&clock);
    assert_eq!(telem.len(), 2);
    assert_eq!(telem[0].born_t_ms, 60); // 3 × 20
    assert_eq!(telem[0].vessels, vec![0, 2]);
    assert_eq!(telem[1].born_t_ms, 200); // 产生步 = 当前步
}

/// 旧字段口径：全事件 vessels 并集（升序去重）。
#[test]
fn all_vessels_is_union_of_queue() {
    let mut queue = q(&[(10, vec![3, 1]), (11, vec![1, 2]), (12, vec![0])]);
    assert_eq!(queue.all_vessels(), vec![0, 1, 2, 3]);
    queue.clear();
    assert!(queue.all_vessels().is_empty());
}

/// Reset 清空。
#[test]
fn clear_empties_queue() {
    let mut queue = q(&[(10, vec![1, 2])]);
    queue.clear();
    assert!(queue.is_empty());
    assert!(queue.snapshot().is_empty());
}
