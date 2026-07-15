//! Ephemeris interpolation node and cubic Hermite spline interpolation.
//!
//! Mirrors the `struct Sample` and `Interpolate()` function from Orbiter's
//! `CelBodyAPI.h` / `Vsop87.cpp:282`. The same interpolation code is duplicated
//! across Vsop87.cpp, Moon.cpp, and Satsat.cpp in Orbiter — here it lives once.

/// An interpolation node used by the fast-ephemeris sliding window.
///
/// Corresponds to `struct Sample` in CelBodyAPI.h:42-46:
/// ```text
/// struct Sample {
///     double t;           // simulation time [s]
///     double rad;         // radial distance
///     double param[6];    // position [0..2] + velocity [3..5]
/// };
/// ```
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    /// Simulation time in seconds.
    pub t: f64,
    /// Radial distance (used for interpolation checks, not by Hermite itself).
    pub rad: f64,
    /// Position (`param[0..2]`) and velocity (`param[3..5]`).
    pub param: [f64; 6],
}

impl Default for Sample {
    fn default() -> Self {
        Self {
            t: -1e20,
            rad: 0.0,
            param: [0.0; 6],
        }
    }
}

/// Cubic Hermite spline interpolation of position and velocity.
///
/// Exactly mirrors `VSOPOBJ::Interpolate` / `Interpolate` in Vsop87.cpp:282
/// (also Moon.cpp:99, Satsat.cpp). Given two samples `s0` (at time `s0.t`) and
/// `s1` (at time `s1.t`), interpolates at time `t` using the position and
/// velocity at both endpoints.
///
/// - `data[0..2]` ← interpolated position
/// - `data[3..5]` ← interpolated velocity
///
/// The endpoint velocities are scaled by `dt` inside the Hermite basis, then
/// un-scaled for the velocity output — matching the C++ code exactly.
pub fn interpolate(t: f64, data: &mut [f64; 6], s0: &Sample, s1: &Sample) {
    let dt = s1.t - s0.t;

    if dt == 0.0 {
        data.copy_from_slice(&s0.param);
        return;
    }

    // Normalized time u ∈ [0, 1] between the two sample points.
    let u = (t - s0.t) / dt;
    let u2 = u * u;
    let u3 = u2 * u;

    // Hermite basis functions for position.
    let h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    let h10 = u3 - 2.0 * u2 + u;
    let h01 = -2.0 * u3 + 3.0 * u2;
    let h11 = u3 - u2;

    // Derivatives of the Hermite basis for velocity.
    let dh00 = 6.0 * u2 - 6.0 * u;
    let dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    let dh01 = -6.0 * u2 + 6.0 * u;
    let dh11 = 3.0 * u2 - 2.0 * u;

    for i in 0..3 {
        let p0 = s0.param[i];
        let p1 = s1.param[i];

        // Scale endpoint velocities by dt (matching C++).
        let v0 = s0.param[i + 3] * dt;
        let v1 = s1.param[i + 3] * dt;

        data[i] = h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1;
        data[i + 3] = (dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1) / dt;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolate_endpoints_exact() {
        // At u=0, result = s0's params exactly.
        let s0 = Sample {
            t: 0.0,
            rad: 1.0,
            param: [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
        };
        let s1 = Sample {
            t: 10.0,
            rad: 2.0,
            param: [4.0, 5.0, 6.0, 0.4, 0.5, 0.6],
        };
        let mut data = [0.0; 6];
        interpolate(0.0, &mut data, &s0, &s1);
        for (d, &s) in data.iter().zip(&s0.param) {
            assert!((d - s).abs() < 1e-15);
        }

        // At u=1, result = s1's params exactly.
        interpolate(10.0, &mut data, &s0, &s1);
        for (d, &s) in data.iter().zip(&s1.param) {
            assert!((d - s).abs() < 1e-15);
        }
    }

    #[test]
    fn interpolate_midpoint() {
        // Two samples with identical position (0) but nonzero velocity.
        // The Hermite interpolation at the midpoint can be computed by hand.
        let s0 = Sample {
            t: 0.0,
            rad: 1.0,
            param: [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        };
        let s1 = Sample {
            t: 2.0,
            rad: 1.0,
            param: [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        };
        let mut data = [0.0; 6];
        interpolate(1.0, &mut data, &s0, &s1);

        // At u=0.5, dt=2, p0=p1=0, v0=v1=1, so v0_scaled=v1_scaled=1*2=2.
        // Position: h00*0 + h10*2 + h01*0 + h11*2 = 2*(h10+h11)
        // h10(0.5) = 0.125 - 0.5 + 0.5 = 0.125
        // h11(0.5) = 0.125 - 0.25 = -0.125
        // => 2*(0.125-0.125) = 0
        for (i, d) in data.iter().take(3).enumerate() {
            assert!(d.abs() < 1e-15, "pos[{i}] = {d}");
        }

        // Velocity: (dh00*0 + dh10*2 + dh01*0 + dh11*2) / dt
        // dh10(0.5) = 0.75 - 2 + 1 = -0.25
        // dh11(0.5) = 0.75 - 1 = -0.25
        // => (-0.25*2 + -0.25*2) / 2 = (-0.5 - 0.5) / 2 = -0.5
        for (i, d) in data.iter().take(6).skip(3).enumerate() {
            assert!((d - (-0.5)).abs() < 1e-14, "vel[{i}] = {d}");
        }
    }

    #[test]
    fn interpolate_dt_zero() {
        // When dt==0, result = s0's params.
        let s0 = Sample {
            t: 5.0,
            rad: 0.0,
            param: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        };
        let s1 = Sample {
            t: 5.0,
            rad: 0.0,
            param: [7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
        };
        let mut data = [0.0; 6];
        interpolate(5.0, &mut data, &s0, &s1);
        for (&d, &s) in data.iter().zip(&s0.param) {
            assert_eq!(d, s);
        }
    }
}
