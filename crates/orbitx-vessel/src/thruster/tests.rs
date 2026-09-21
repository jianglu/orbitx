use super::*;

#[test]
fn no_gimbal_returns_base_dir() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0);
    t.max_gimbal = 0.1;
    let d = t.current_dir();
    assert!((d - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-12);
}

#[test]
fn gimbal_pitch_rotates_about_x() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0)
        .with_tvc(std::f64::consts::FRAC_PI_6, 0.0, Vec3::new(1.0, 0.0, 0.0));
    t.gimbal_pitch = std::f64::consts::FRAC_PI_2;
    let d = t.current_dir();
    assert!((d.length() - 1.0).abs() < 1e-9, "not unit: {:?}", d);
    assert!(d.x.abs() < 1e-9, "should stay in YZ plane: {:?}", d);
}

#[test]
fn gimbal_yaw_rotates_about_z() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 1000.0, 300.0)
        .with_tvc(std::f64::consts::FRAC_PI_6, 0.0, Vec3::new(1.0, 0.0, 0.0));
    t.gimbal_yaw = std::f64::consts::FRAC_PI_2;
    let d = t.current_dir();
    assert!((d.length() - 1.0).abs() < 1e-9);
    // +Y 绕 +Z 转 +90° → −X
    assert!((d.x + 1.0).abs() < 1e-9 && d.y.abs() < 1e-9, "got {:?}", d);
}

#[test]
fn set_gimbal_clamps() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0)
        .with_tvc(0.1, 0.0, Vec3::new(1.0, 0.0, 0.0));
    t.set_gimbal_2(5.0, -5.0);
    assert!((t.gimbal_pitch - 0.1).abs() < 1e-12);
    assert!((t.gimbal_yaw + 0.1).abs() < 1e-12);
}

#[test]
fn slew_rate_limited() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0)
        .with_tvc(1.0, 1.0, Vec3::new(1.0, 0.0, 0.0));
    t.slew_gimbal(1.0, 1.0, 0.1);
    assert!((t.gimbal_pitch - 0.1).abs() < 1e-9, "got {}", t.gimbal_pitch);
    assert!((t.gimbal_yaw - 0.1).abs() < 1e-9, "got {}", t.gimbal_yaw);
}

#[test]
fn slew_throttle_rate_limited() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 1000.0, 300.0)
        .with_throttle_rate(0.8);
    t.level_cmd = 1.0;
    t.slew_throttle(0.1);
    assert!((t.level - 0.08).abs() < 1e-12, "got {}", t.level);
}

#[test]
fn slew_throttle_instant_when_rate_zero() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 1000.0, 300.0);
    t.level_cmd = 0.75;
    t.slew_throttle(0.01);
    assert!((t.level - 0.75).abs() < 1e-12);
}

#[test]
fn pfac_vacuum_full_thrust() {
    let t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 1000.0, 300.0)
        .with_pfac(pfac_from_isp_sl(300.0, 270.0));
    let mut t = t;
    t.level = 1.0;
    assert!((t.current_thrust(0.0) - 1000.0).abs() < 1e-9);
    assert!((t.effective_isp(0.0) - 300.0).abs() < 1e-9);
}

#[test]
fn pfac_sl_matches_isp_sl() {
    let isp_vac = 311.0;
    let isp_sl = 282.0;
    let pfac = pfac_from_isp_sl(isp_vac, isp_sl);
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 914_000.0, isp_vac).with_pfac(pfac);
    t.level = 1.0;
    let isp_e = t.effective_isp(P_REF_SL);
    assert!((isp_e - isp_sl).abs() < 0.05, "got {isp_e} want {isp_sl}");
}

#[test]
fn pfac_zero_unaffected_by_pressure() {
    let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 1000.0, 300.0);
    t.level = 1.0;
    assert!((t.current_thrust(P_REF_SL) - 1000.0).abs() < 1e-9);
}
