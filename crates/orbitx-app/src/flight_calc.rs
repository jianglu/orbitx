//! Convert a UserVessel + parent-body context into a FlightState.
//!
//! Computes orbital elements from the (rel_pos, rel_vel) state vectors in the
//! parent's inertial frame, plus altitude/speed decomposition, a simple
//! atmosphere model (Earth only in this pass), and copies vessel-carried
//! attitude / propulsion fields.

use orbitx_gfx_hud::FlightState;
use orbitx_math::vec3::Vec3;

use crate::vessel::UserVessel;

/// Parent-body context needed to interpret the vessel state.
#[derive(Clone, Debug)]
pub struct ParentBody {
    pub name: String,
    /// Absolute sim-space position of the parent (m, J2000).
    pub abs_pos: Vec3,
    /// GM (m^3/s^2).
    pub gm: f64,
    /// Physical radius (m).
    pub radius: f64,
}

/// Reference plane normal for inclination (ecliptic north in sim coords).
const ECL_NORTH: Vec3 = Vec3 { x: 0.0, y: 1.0, z: 0.0 };

/// Build a FlightState from vessel + parent + wall-clock context.
///
/// `sim_time` is elapsed simulation seconds, `mjd` the current modified
/// Julian date, `time_warp` the current speed-up factor.
pub fn compute_flight_state(
    v: &UserVessel,
    parent: &ParentBody,
    sim_time: f64,
    mjd: f64,
    time_warp: f64,
) -> FlightState {
    let r_vec = v.rel_pos;
    let v_vec = v.rel_vel;
    let r = r_vec.length();
    let speed = v_vec.length();

    // Radial (vertical) and tangential (horizontal) components.
    let r_hat = if r > 0.0 { r_vec * (1.0 / r) } else { Vec3::ZERO };
    let vertical_speed = v_vec.x * r_hat.x + v_vec.y * r_hat.y + v_vec.z * r_hat.z;
    let horizontal_speed = (speed * speed - vertical_speed * vertical_speed).max(0.0).sqrt();

    // Orbital elements (two-body, parent-centered).
    let mu = parent.gm.max(1.0);
    let eps = 0.5 * speed * speed - mu / r.max(1.0);
    let sma = if eps.abs() > 1e-12 { -mu / (2.0 * eps) } else { r };

    // Angular momentum h = r x v (manual cross to avoid depending on Vec3 op).
    let hx = r_vec.y * v_vec.z - r_vec.z * v_vec.y;
    let hy = r_vec.z * v_vec.x - r_vec.x * v_vec.z;
    let hz = r_vec.x * v_vec.y - r_vec.y * v_vec.x;
    let h = (hx * hx + hy * hy + hz * hz).sqrt();
    // Inclination: angle between h and reference plane normal (ecliptic north).
    let inclination = if h > 0.0 {
        let cos_i = (hx * ECL_NORTH.x + hy * ECL_NORTH.y + hz * ECL_NORTH.z) / h;
        cos_i.clamp(-1.0, 1.0).acos()
    } else {
        0.0
    };

    // Eccentricity: e = ((v^2 - mu/r) r - (r . v) v) / mu.
    let rv_dot = r_vec.x * v_vec.x + r_vec.y * v_vec.y + r_vec.z * v_vec.z;
    let coeff_r = speed * speed - mu / r.max(1.0);
    let ex = (coeff_r * r_vec.x - rv_dot * v_vec.x) / mu;
    let ey = (coeff_r * r_vec.y - rv_dot * v_vec.y) / mu;
    let ez = (coeff_r * r_vec.z - rv_dot * v_vec.z) / mu;
    let eccentricity = (ex * ex + ey * ey + ez * ez).sqrt();

    let rp = sma * (1.0 - eccentricity);
    let ra = sma * (1.0 + eccentricity);
    let periapsis_alt = (rp - parent.radius).max(-parent.radius);
    let apoapsis_alt = (ra - parent.radius).max(-parent.radius);

    // Period (elliptical only).
    let period = if sma > 0.0 && sma.is_finite() {
        std::f64::consts::TAU * (sma.powi(3) / mu).sqrt()
    } else {
        f64::INFINITY
    };

    let altitude = (r - parent.radius).max(0.0);

    // Simple exponential atmosphere: Earth only.
    let (air_density, mach) = atmosphere_for(&parent.name, altitude, speed);
    let dynamic_pressure = 0.5 * air_density * speed * speed;

    let thrust = v.thrust();
    let total_mass = v.total_mass();
    let tw_ratio = if total_mass > 0.0 { thrust / (total_mass * 9.80665) } else { 0.0 };

    let abs_pos = parent.abs_pos + r_vec;

    FlightState {
        position: abs_pos,
        velocity: v_vec,
        speed,
        vertical_speed,
        horizontal_speed,

        semi_major_axis: sma,
        eccentricity,
        inclination,
        periapsis_alt,
        apoapsis_alt,
        period,
        specific_energy: eps,

        pitch: v.pitch,
        yaw: v.yaw,
        bank: v.bank,

        thrust,
        tw_ratio,
        throttle: v.throttle,
        fuel_mass: v.fuel_mass,
        total_mass,

        focus_dist: r,
        altitude,
        focus_name: parent.name.clone(),
        air_density,
        dynamic_pressure,
        mach,

        sim_time,
        mjd,
        time_warp,

        is_thrusting: thrust > 0.0,
        is_crashed: false,
        on_ground: altitude <= 0.0,
    }
}

/// Simple atmosphere model. Returns (density kg/m^3, mach).
fn atmosphere_for(name: &str, altitude_m: f64, speed_mps: f64) -> (f64, f64) {
    match name {
        // Earth: exponential with scale height ~8.5 km, sea-level density 1.225.
        "Earth" => {
            let rho0 = 1.225_f64;
            let h_scale = 8_500.0_f64;
            let rho = rho0 * (-altitude_m / h_scale).exp();
            // Approximate speed of sound: decreases with altitude (rough).
            let c = 340.0 * ((rho / rho0).max(0.05)).sqrt();
            (rho, speed_mps / c)
        }
        _ => (0.0, 0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EARTH_R: f64 = 6.371e6;
    const EARTH_GM: f64 = 3.986004418e14;

    fn earth() -> ParentBody {
        ParentBody {
            name: "Earth".into(),
            abs_pos: Vec3::ZERO,
            gm: EARTH_GM,
            radius: EARTH_R,
        }
    }

    #[test]
    fn leo_gives_expected_elements() {
        let v = UserVessel::leo(0, EARTH_R, EARTH_GM);
        let fs = compute_flight_state(&v, &earth(), 0.0, 51544.5, 1.0);
        assert!(fs.eccentricity < 1e-6, "e={}", fs.eccentricity);
        let expected_a = EARTH_R + 400_000.0;
        assert!((fs.semi_major_axis - expected_a).abs() / expected_a < 1e-6);
        let expected_period = std::f64::consts::TAU * (expected_a.powi(3) / EARTH_GM).sqrt();
        assert!((fs.period - expected_period).abs() / expected_period < 1e-6);
        assert!((fs.altitude - 400_000.0).abs() < 1.0);
    }

    #[test]
    fn radial_component_zero_for_circular_orbit() {
        let v = UserVessel::leo(0, EARTH_R, EARTH_GM);
        let fs = compute_flight_state(&v, &earth(), 0.0, 51544.5, 1.0);
        assert!(fs.vertical_speed.abs() < 1e-6);
        assert!((fs.horizontal_speed - fs.speed).abs() / fs.speed < 1e-6);
    }

    #[test]
    fn elliptical_orbit_apsides() {
        // Prograde boost from circular: raises apoapsis.
        let mut v = UserVessel::leo(0, EARTH_R, EARTH_GM);
        let boost = 500.0;
        // v.rel_vel is along +z initially (see UserVessel::leo).
        v.rel_vel = Vec3::new(0.0, 0.0, v.rel_vel.z + boost);
        let fs = compute_flight_state(&v, &earth(), 0.0, 51544.5, 1.0);
        assert!(fs.eccentricity > 0.0);
        // Apoapsis should now be above the circular altitude.
        assert!(fs.apoapsis_alt > 400_000.0);
        // Periapsis is below apoapsis.
        assert!(fs.periapsis_alt < fs.apoapsis_alt);
    }

    #[test]
    fn earth_atmosphere_density_decays() {
        let (rho0, _) = atmosphere_for("Earth", 0.0, 0.0);
        let (rho10, _) = atmosphere_for("Earth", 10_000.0, 0.0);
        let (rho100, _) = atmosphere_for("Earth", 100_000.0, 0.0);
        assert!(rho0 > rho10);
        assert!(rho10 > rho100);
        assert!(rho100 < 1e-4);
    }

    #[test]
    fn non_earth_has_no_atmosphere() {
        let (rho, _) = atmosphere_for("Moon", 0.0, 100.0);
        assert_eq!(rho, 0.0);
    }
}
