use super::ShutdownFlag;

#[test]
fn flag_starts_clear_and_latches() {
    let f = ShutdownFlag::new();
    assert!(!f.is_requested());
    f.request();
    assert!(f.is_requested());
    f.request();
    assert!(f.is_requested());
}
