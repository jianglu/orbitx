//! User vessel state and 2-body RK4 propagator.
//!
//! The vessel state is kept in the PARENT body's inertial frame (J2000
//! orientation, origin at the parent's center). This decouples propagation
//! from the parent body's own motion in the solar system, and matches the
//! frame in which orbital elements are naturally computed.
//!
//! Simplifications:
//! - Point-mass gravity of the parent only (no third-body, no oblateness).
//!   Accurate enough for low-orbit HUD/MFD display.
//! - Attitude (pitch/yaw/bank) is carried as user Euler angles; it does not
//!   feed back into dynamics in this pass.
//! - Throttle applies prograde thrust for simple orbit changes.

use orbitx_math::vec3::Vec3;

/// User-controlled vessel state in the parent body's inertial frame.
#[derive(Clone, Debug)]
pub struct UserVessel {
    /// Parent body index in the scene/planetary system.
    pub parent_idx: usize,
    /// Position relative to parent body center (m).
    pub rel_pos: Vec3,
    /// Velocity relative to the parent (m/s).
    pub rel_vel: Vec3,
    /// Attitude (rad): pitch above horizon, yaw compass heading, bank roll.
    pub pitch: f64,
    pub yaw: f64,
    pub bank: f64,
    /// Throttle in [0, 1].
    pub throttle: f64,
    /// Dry mass (kg).
    pub dry_mass: f64,
    /// Fuel mass remaining (kg).
    pub fuel_mass: f64,
    /// Vacuum specific impulse (s).
    pub isp: f64,
    /// Maximum thrust (N).
    pub thrust_max: f64,
}

/// Standard gravity used to convert Isp to exhaust velocity (m/s^2).
const G0: f64 = 9.80665;

impl UserVessel {
    /// Default LEO around the given parent body.
    ///
    /// `parent_radius` in meters, `parent_gm` = GM in m^3/s^2.
    /// Places the vessel at 400 km circular equatorial orbit, prograde in +z.
    pub fn leo(parent_idx: usize, parent_radius: f64, parent_gm: f64) -> Self {
        let alt = 400_000.0;
        let r = parent_radius + alt;
        let v = (parent_gm / r).sqrt();
        Self {
            parent_idx,
            rel_pos: Vec3::new(r, 0.0, 0.0),
            rel_vel: Vec3::new(0.0, 0.0, v),
            pitch: 0.0,
            yaw: 0.0,
            bank: 0.0,
            throttle: 0.0,
            dry_mass: 5_000.0,
            fuel_mass: 5_000.0,
            isp: 320.0,
            thrust_max: 30_000.0,
        }
    }

    /// Total mass (kg) = dry + fuel.
    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.fuel_mass
    }

    /// Current commanded thrust (N).
    pub fn thrust(&self) -> f64 {
        if self.fuel_mass > 0.0 {
            self.throttle.clamp(0.0, 1.0) * self.thrust_max
        } else {
            0.0
        }
    }

    /// Propagate one RK4 step of `dt` seconds under 2-body gravity plus a
    /// prograde thrust term. `parent_gm` = GM of the parent body (m^3/s^2).
    pub fn propagate(&mut self, parent_gm: f64, dt: f64) {
        let thrust = self.thrust();
        let mass = self.total_mass();
        let mdot = if thrust > 0.0 && self.isp > 0.0 {
            thrust / (self.isp * G0)
        } else {
            0.0
        };

        // Acceleration at (p, v, m): point-mass gravity + prograde thrust.
        let acc = |p: Vec3, v: Vec3, m: f64| -> Vec3 {
            let r = p.length();
            let grav = if r > 1.0 {
                p * (-parent_gm / (r * r * r))
            } else {
                Vec3::ZERO
            };
            let vlen = v.length();
            let thrust_acc = if thrust > 0.0 && vlen > 1e-6 && m > 1e-6 {
                v * (thrust / (m * vlen))
            } else {
                Vec3::ZERO
            };
            grav + thrust_acc
        };

        // Classic RK4 in (position, velocity) with linear mass depletion.
        let p0 = self.rel_pos;
        let v0 = self.rel_vel;
        let m0 = mass;

        let a1 = acc(p0, v0, m0);
        let k1p = v0;
        let k1v = a1;

        let m2 = m0 - mdot * dt * 0.5;
        let a2 = acc(p0 + k1p * (dt * 0.5), v0 + k1v * (dt * 0.5), m2);
        let k2p = v0 + k1v * (dt * 0.5);
        let k2v = a2;

        let a3 = acc(p0 + k2p * (dt * 0.5), v0 + k2v * (dt * 0.5), m2);
        let k3p = v0 + k2v * (dt * 0.5);
        let k3v = a3;

        let m4 = m0 - mdot * dt;
        let a4 = acc(p0 + k3p * dt, v0 + k3v * dt, m4);
        let k4p = v0 + k3v * dt;
        let k4v = a4;

        self.rel_pos = p0 + (k1p + k2p * 2.0 + k3p * 2.0 + k4p) * (dt / 6.0);
        self.rel_vel = v0 + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
        self.fuel_mass = (self.fuel_mass - mdot * dt).max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    // Earth-like parameters for tests.
    const EARTH_R: f64 = 6.371e6;
    const EARTH_GM: f64 = 3.986004418e14;

    #[test]
    fn leo_defaults_are_circular() {
        let v = UserVessel::leo(0, EARTH_R, EARTH_GM);
        let r = v.rel_pos.length();
        let s = v.rel_vel.length();
        // v = sqrt(mu/r) for a circular orbit.
        let expected = (EARTH_GM / r).sqrt();
        assert!((s - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn circular_orbit_closes_after_one_period() {
        let mut v = UserVessel::leo(0, EARTH_R, EARTH_GM);
        let r0 = v.rel_pos;
        let a = v.rel_pos.length();
        let period = TAU * (a * a * a / EARTH_GM).sqrt();
        // Propagate one full period with a small step.
        let dt = period / 20_000.0;
        let steps = 20_000;
        for _ in 0..steps {
            v.propagate(EARTH_GM, dt);
        }
        // Position should return within a small fraction of the radius.
        let err = (v.rel_pos - r0).length() / a;
        assert!(err < 1e-3, "closure error {err}");
    }

    #[test]
    fn zero_thrust_conserves_energy() {
        let mut v = UserVessel::leo(0, EARTH_R, EARTH_GM);
        // Specific orbital energy: eps = v^2/2 - mu/r.
        let eps0 = 0.5 * v.rel_vel.length().powi(2) - EARTH_GM / v.rel_pos.length();
        for _ in 0..5000 {
            v.propagate(EARTH_GM, 0.5);
        }
        let eps1 = 0.5 * v.rel_vel.length().powi(2) - EARTH_GM / v.rel_pos.length();
        let rel = (eps1 - eps0).abs() / eps0.abs();
        assert!(rel < 1e-4, "energy drift {rel}");
    }

    #[test]
    fn thrust_consumes_fuel() {
        let mut v = UserVessel::leo(0, EARTH_R, EARTH_GM);
        v.throttle = 1.0;
        let f0 = v.fuel_mass;
        for _ in 0..1000 {
            v.propagate(EARTH_GM, 0.1);
        }
        assert!(v.fuel_mass < f0);
    }
}
