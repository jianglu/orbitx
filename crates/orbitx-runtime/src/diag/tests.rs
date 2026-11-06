use super::{probe_sleep, DurationWindow};
use std::time::Duration;

#[test]
fn snapshot_empty_is_none() {
    let w = DurationWindow::new(8, Duration::from_millis(20));
    assert!(w.snapshot().is_none());
}

#[test]
fn avg_max_over_window_semantics() {
    let mut w = DurationWindow::new(8, Duration::from_millis(20));
    for ms in [10u64, 30, 10, 15] {
        w.push(Duration::from_millis(ms));
    }
    let st = w.snapshot().expect("samples");
    assert_eq!(st.count, 4);
    assert_eq!(st.avg, Duration::from_nanos(16_250_000)); // (10+30+10+15)/4 = 16.25ms
    assert_eq!(st.max, Duration::from_millis(30));
    assert_eq!(st.over, 1);
}

#[test]
fn eviction_keeps_aggregates_consistent() {
    let mut w = DurationWindow::new(4, Duration::from_millis(20));
    for ms in [10u64, 30, 10, 10, 10, 10] {
        w.push(Duration::from_millis(ms));
    }
    let st = w.snapshot().expect("samples");
    // 只留最后 4 个（全 10ms）；30ms 已被驱逐，over 归零。
    assert_eq!(st.count, 4);
    assert_eq!(st.avg, Duration::from_millis(10));
    assert_eq!(st.max, Duration::from_millis(10));
    assert_eq!(st.over, 0);
}

#[test]
fn reset_clears_all() {
    let mut w = DurationWindow::new(8, Duration::from_millis(20));
    w.push(Duration::from_millis(30));
    w.reset();
    assert!(w.snapshot().is_none());
}

#[test]
fn probe_sleep_reports_at_least_target() {
    // 真 sleep（截到 1ms 上限防慢测）：sleep 保证 >= target。
    let probe = probe_sleep(
        |d| std::thread::sleep(d.min(Duration::from_millis(1))),
        Duration::from_millis(1),
        4,
    );
    assert!(probe.avg >= Duration::from_millis(1));
    assert!(probe.max >= probe.avg);
}
