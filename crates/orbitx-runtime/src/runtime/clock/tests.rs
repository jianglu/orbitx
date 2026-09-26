use super::{Clock, WARP_MAX, WARP_MIN};

#[test]
fn advance_does_not_scale_dt_by_warp() {
    let mut c = Clock::new(20);
    c.set_warp(10.0);
    c.advance_fixed_step();
    assert_eq!(c.sim_t_ms(), 20);
    assert_eq!(c.step_index(), 1);
    assert!((c.sim_dt_secs() - 0.02).abs() < 1e-12);
}

#[test]
fn set_warp_clamps_to_max() {
    let mut c = Clock::new(20);
    c.set_warp(1e9);
    assert!((c.warp() - WARP_MAX).abs() < 1e-12);
}

#[test]
fn set_warp_clamps_to_min() {
    let mut c = Clock::new(20);
    c.set_warp(1e-9);
    assert!((c.warp() - WARP_MIN).abs() < 1e-12);
}

#[test]
fn set_warp_invalid_falls_back_to_one() {
    let mut c = Clock::new(20);
    c.set_warp(0.0);
    assert!((c.warp() - 1.0).abs() < 1e-12);
    c.set_warp(f64::NAN);
    assert!((c.warp() - 1.0).abs() < 1e-12);
    c.set_warp(-2.0);
    assert!((c.warp() - 1.0).abs() < 1e-12);
}
