use super::gate_allows;

/// hold 期：首步放行（出初始切片），第二步起拒绝。
#[test]
fn hold_allows_only_first_step() {
    let mut stepped_once = false;
    assert!(gate_allows(false, &mut stepped_once));
    assert!(stepped_once);
    assert!(!gate_allows(false, &mut stepped_once));
    assert!(!gate_allows(false, &mut stepped_once));
}

/// Launch 后全放行；已放行首步的 hold 态再次确认无副作用。
#[test]
fn launch_opens_gate_permanently() {
    let mut stepped_once = false;
    assert!(gate_allows(false, &mut stepped_once));
    assert!(gate_allows(true, &mut stepped_once));
    assert!(gate_allows(true, &mut stepped_once));
}

/// gate 状态初始即 open（self-paced 等价场景）时首步与后续全放行。
#[test]
fn open_gate_from_start_allows_all() {
    let mut stepped_once = false;
    assert!(gate_allows(true, &mut stepped_once));
    assert!(gate_allows(true, &mut stepped_once));
}
