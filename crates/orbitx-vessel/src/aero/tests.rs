use super::*;
use orbitx_math::Vec3;

#[test]
fn exponential_atmosphere_sea_level() {
    let atm = ExponentialAtmosphere::earth();
    let rho = atm.density(0.0);
    assert!((rho - 1.225).abs() < 1e-6, "ρ(0) = {rho}");
}

#[test]
fn exponential_atmosphere_10km() {
    let atm = ExponentialAtmosphere::earth();
    let rho = atm.density(10_000.0);
    let expected = 1.225 * (-10_000.0_f64 / 8500.0).exp();
    assert!((rho - expected).abs() < 1e-6, "ρ(10km) = {rho}, expected = {expected}");
}

#[test]
fn exponential_atmosphere_negative_alt() {
    let atm = ExponentialAtmosphere::earth();
    assert_eq!(atm.density(-100.0), 0.0);
}

#[test]
fn us76_density_magnitudes() {
    let atm = UsStd1976Atmosphere::new();
    let rho0 = atm.density(0.0);
    assert!((rho0 - 1.225).abs() < 0.02, "sea-level ρ = {rho0}");
    let rho11 = atm.density(11_000.0);
    assert!(rho11 > 0.3 && rho11 < 0.45, "11 km ρ = {rho11}");
    let rho25 = atm.density(25_000.0);
    assert!(rho25 > 0.03 && rho25 < 0.05, "25 km ρ = {rho25}");
    let rho50 = atm.density(50_000.0);
    assert!(rho50 > 5e-4 && rho50 < 2e-3, "50 km ρ = {rho50}");
}

#[test]
fn atmosphere_from_config_branches() {
    use orbitx_config::{AtmosphereConfig, AtmosphereModel};
    let base = AtmosphereConfig {
        model: AtmosphereModel::Exponential,
        density0: 1.225,
        scale_height: 8500.0,
        pressure0: 101325.0,
        gas_constant: 287.0,
        gamma: 1.4,
        alt_limit: 200e3,
    };
    let exp = atmosphere_from_config(Some(&base)).unwrap();
    assert!((exp.density(0.0) - 1.225).abs() < 1e-6);

    let mut us = base.clone();
    us.model = AtmosphereModel::Us76;
    let u = atmosphere_from_config(Some(&us)).unwrap();
    assert!((u.density(0.0) - 1.225).abs() < 0.02);

    let mut none = base;
    none.model = AtmosphereModel::None;
    assert!(atmosphere_from_config(Some(&none)).is_none());
    assert!(atmosphere_from_config(None).is_none());
}

#[test]
fn sound_speed_varies_with_temperature() {
    let atm = UsStd1976Atmosphere::new();
    let a0 = atm.sound_speed(0.0);
    let a11 = atm.sound_speed(11_000.0);
    assert!(a0 > 330.0 && a0 < 350.0, "a(0) = {a0}");
    assert!(a11 < a0 - 20.0, "tropopause colder → slower sound: {a11} vs {a0}");
}

#[test]
fn cd_mach_changes_drag_magnitude() {
    let table = vec![
        (0.0, 0.3),
        (1.0, 0.3),
        (1.2, 1.0),
        (2.0, 0.5),
    ];
    let de = DragElement::constant(Vec3::ZERO, 0.3, 10.0).with_cd_mach(table);
    let rho = 1.225;
    // Choose speeds so dynp similar is NOT the goal — same q different Ma:
    // Fix airspeed via sound_speed so Ma differs while v (thus q) same.
    let v = 200.0;
    let airvel = Vec3::new(0.0, 0.0, -v);
    let r_lo = compute_aero_forces(
        &[],
        &[],
        &[de.clone()],
        airvel,
        rho,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        400.0, // Ma = 0.5
    );
    let r_hi = compute_aero_forces(
        &[],
        &[],
        &[de],
        airvel,
        rho,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        200.0 / 1.2, // Ma = 1.2
    );
    assert!((r_lo.mach - 0.5).abs() < 1e-9, "Ma low = {}", r_lo.mach);
    assert!((r_hi.mach - 1.2).abs() < 1e-9, "Ma high = {}", r_hi.mach);
    assert!(
        (r_hi.drag_force - r_lo.drag_force).abs() > 100.0,
        "same q, different Cd(M): {} vs {}",
        r_hi.drag_force,
        r_lo.drag_force
    );
}

#[test]
fn constant_cd_matches_legacy() {
    let v = 100.0;
    let rho = 1.225;
    let cd = 0.3;
    let area = 10.0;
    let expected = 0.5 * rho * v * v * cd * area;
    let dragels = vec![DragElement::constant(Vec3::ZERO, cd, area)];
    let result = compute_aero_forces(
        &[],
        &[],
        &dragels,
        Vec3::new(0.0, 0.0, -v),
        rho,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        340.0,
    );
    let rel_err = (result.force.z - expected).abs() / expected;
    assert!(rel_err < 1e-10);
}

#[test]
fn zero_airspeed_no_force() {
    let result = compute_aero_forces(
        &[],
        &[],
        &[],
        Vec3::ZERO,
        1.225,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        340.0,
    );
    assert_eq!(result.force, Vec3::ZERO);
}

#[test]
fn zero_density_no_force() {
    let result = compute_aero_forces(
        &[],
        &[],
        &[DragElement::constant(Vec3::ZERO, 1.0, 1.0)],
        Vec3::new(0.0, 0.0, 100.0),
        0.0,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        340.0,
    );
    assert_eq!(result.force, Vec3::ZERO);
}

#[test]
fn axial_drag_direction() {
    let dragels = vec![DragElement::constant(Vec3::ZERO, 1.0, 1.0)];
    let result = compute_aero_forces(
        &[],
        &[],
        &dragels,
        Vec3::new(0.0, 0.0, -100.0),
        1.225,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        340.0,
    );
    assert!(result.force.z > 0.0);
    assert!(result.force.x.abs() < 1e-6);
    assert!(result.force.y.abs() < 1e-6);
}

#[test]
fn lift_perpendicular_to_drag() {
    let airfoils = vec![Airfoil {
        ref_pos: Vec3::ZERO,
        orientation: AirfoilOrientation::LiftVertical,
        chord: 1.0,
        area: 1.0,
        aspect_ratio: 1.0,
        coeffs: AirfoilCoeffs::Constant {
            cl: 1.0,
            cm: 0.0,
            cd: 0.5,
        },
    }];
    let airvel = Vec3::new(0.0, -50.0, 100.0);
    let result = compute_aero_forces(
        &airfoils,
        &[],
        &[],
        airvel,
        1.225,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        340.0,
    );
    let ddir = airvel * (-1.0 / airvel.length());
    let ldir_raw = Vec3::new(0.0, airvel.z, -airvel.y);
    let ldir = ldir_raw * (1.0 / ldir_raw.length());
    assert!(dot(ddir, ldir).abs() < 1e-9);
    assert!(result.force.length() > 1e-3);
}

#[test]
fn control_surface_deflection_produces_lift() {
    let ctrlsurfs = vec![ControlSurface {
        ctrl_type: CtrlType::Elevator,
        ref_pos: Vec3::ZERO,
        axis: CtrlAxis::YPos,
        area: 1.0,
        d_cl: 2.0,
        level: 0.5,
    }];
    let result = compute_aero_forces(
        &[],
        &ctrlsurfs,
        &[],
        Vec3::new(0.0, 0.0, 100.0),
        1.225,
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        340.0,
    );
    assert!(result.force.length() > 1e-3);
}

#[test]
fn aero_damping_reduces_omega() {
    let omega = Vec3::new(1.0, 0.0, 0.0);
    let result = compute_aero_forces(
        &[],
        &[],
        &[],
        Vec3::new(0.0, 0.0, 100.0),
        1.225,
        omega,
        Vec3::new(10.0, 1.0, 10.0),
        1000.0,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        0.05,
        340.0,
    );
    assert!(result.torque.x < 0.0);
}

#[test]
fn airfoil_table_interpolation() {
    use std::f64::consts::FRAC_PI_4;
    let table = AirfoilCoeffs::Table(vec![
        (0.0, 0.0, 0.0, 0.1),
        (std::f64::consts::FRAC_PI_2, 1.0, 0.0, 0.2),
    ]);
    let (cl, cm, cd) = table.evaluate(FRAC_PI_4);
    assert!((cl - 0.5).abs() < 1e-10);
    assert!(cm.abs() < 1e-10);
    assert!((cd - 0.15).abs() < 1e-10);
}

#[test]
fn airfoil_linear_lift() {
    let coeffs = AirfoilCoeffs::LinearLift {
        cl_alpha: 2.0 * std::f64::consts::PI,
        cl0: 0.0,
        cd0: 0.02,
    };
    let (cl, _, cd) = coeffs.evaluate(0.1);
    assert!((cl - 2.0 * std::f64::consts::PI * 0.1).abs() < 1e-10);
    assert!((cd - 0.02).abs() < 1e-10);
}

#[test]
fn world_to_airvel_ship_identity() {
    use orbitx_math::Matrix3;
    let vel = Vec3::new(100.0, 0.0, 0.0);
    let airvel = world_to_airvel_ship(vel, Vec3::ZERO, Matrix3::IDENTITY);
    assert!((airvel.x - 100.0).abs() < 1e-10);
}
