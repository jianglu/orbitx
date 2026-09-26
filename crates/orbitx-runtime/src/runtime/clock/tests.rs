use super::Clock;

#[test]
fn advance_does_not_scale_dt_by_warp() {
    let mut c = Clock::new(20);
    c.set_warp(10.0);
    c.advance_fixed_step();
    assert_eq!(c.sim_t_ms(), 20);
    assert_eq!(c.step_index(), 1);
    assert!((c.sim_dt_secs() - 0.02).abs() < 1e-12);
}
