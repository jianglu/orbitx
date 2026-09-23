//! Mathematical and astronomical constants.
//!
//! All values mirror `Vecmat.h` (lines 14-21) and `Astro.h` / `OrbiterAPI.h`
//! exactly. Units are SI unless noted.

// --- Angle constants (Vecmat.h:14-20) ---
// These intentionally replicate Orbiter's C++ literal values exactly (rather
// than using std::f64::consts) so that downstream computations match the C++
// oracle bit-for-bit. The crate-level `#![allow(clippy::approx_constant)]` in
// lib.rs suppresses the lint.

/// π
pub const PI: f64 = 3.14159265358979323846;
/// 2π
pub const PI2: f64 = 6.28318530717958647693;
/// π/2
pub const PI05: f64 = 1.57079632679489661923;
/// 3π/2
pub const PI15: f64 = 4.71238898038468985769;
/// π/4
pub const PI025: f64 = 0.785398163397448309615;
/// Degrees → radians factor (`_RAD_`, Vecmat.h:19)
pub const RAD: f64 = PI / 180.0;
/// Radians → degrees factor (`_DEG_`, Vecmat.h:20)
pub const DEG: f64 = 180.0 / PI;
/// Reciprocal of ln(2) (`LOG2`, Vecmat.h:21)
pub const LOG2: f64 = 1.44269504088896340736;

// --- Astronomical constants (OrbiterAPI.h:69-72, Astro.h:23-26) ---
/// Speed of light in vacuum [m/s] (`C0`, OrbiterAPI.h:69)
pub const C0: f64 = 299_792_458.0;
/// Light time for 1 AU [s] (`TAUA`, OrbiterAPI.h:70)
pub const TAUA: f64 = 499.004783806;
/// Astronomical unit [m] (`AU = C0*TAUA`, OrbiterAPI.h:71)
pub const AU: f64 = C0 * TAUA;
/// Reciproal of the AU [1/m] (`iAU`, Astro.h:14)
pub const IAU: f64 = 1.0 / AU;
/// Gravitational constant [m³ kg⁻¹ s⁻²] (`Ggrav`, OrbiterAPI.h:72 / Astro.h:23)
pub const GGRAV: f64 = 6.67259e-11;
/// Modified Julian Date of the J2000 epoch (`MJD2000`, Astro.h:26)
pub const MJD2000: f64 = 51544.5;
/// Seconds per Julian day reciprocal — multiplying seconds by this yields days
/// (`day = 1/86400`, Astro.h:34)
pub const DAY: f64 = 1.0 / 86400.0;
/// Parsec [m] (`parsec`, Astro.h:17): the distance at which 1 AU subtends 1 arcsec.
pub const PARSEC: f64 = 3.08567758075545e16;
/// Reciprocal of the parsec [1/m] (`iparsec`, Astro.h:20)
pub const IPARSEC: f64 = 1.0 / PARSEC;
/// UTC − Coordinate Time offset [s] (`UTC_CT_diff`, Astro.h:37)
pub const UTC_CT_DIFF: f64 = 66.184;

// --- Convenience angle conversions (Vecmat.h:23-24) ---
/// Convert degrees to radians (`Rad`, Vecmat.h:23).
#[inline]
pub const fn rad(deg: f64) -> f64 {
    deg * RAD
}

/// Convert radians to degrees (`Deg`, Vecmat.h:24).
#[inline]
pub const fn deg(rad: f64) -> f64 {
    rad * DEG
}

/// `a1 - a2` with the 2π wraparound removed (`diffangle`, Vecmat.h:51).
///
/// Result lies in `(-2π, 2π)` and represents the shortest signed angular
/// separation consistent with the C++ implementation.
#[inline]
pub fn diff_angle(a1: f64, a2: f64) -> f64 {
    // Wrap both into [0, 2π).
    let mut a1 = a1 % PI2;
    let mut a2 = a2 % PI2;
    if a1 < 0.0 {
        a1 += PI2;
    }
    if a2 < 0.0 {
        a2 += PI2;
    }
    if a1 - a2 > PI {
        a2 += PI2;
    } else if a2 - a1 > PI {
        a1 += PI2;
    }
    a1 - a2
}

/// Wrap angle to [0, 2π) (`posangle` scalar form, used by Element.cpp).
///
/// Takes any signed angle and returns the equivalent angle in `[0, 2π)`.
#[inline]
pub fn pos_angle(a: f64) -> f64 {
    let mut a = a % PI2;
    if a < 0.0 {
        a += PI2;
    }
    a
}

/// Two-argument atan2 returning a value in [0, 2π) (`posangle(y, x)` macro form,
/// used by Celbody.cpp). `atan2` returns `(-π, π]`; this lifts negatives into
/// `[0, 2π)`.
#[inline]
pub fn atan2_0_2pi(y: f64, x: f64) -> f64 {
    let a = y.atan2(x);
    if a < 0.0 { a + PI2 } else { a }
}

/// Inverse hyperbolic sine.
///
/// Orbiter (`Vecmat.h:60`) hand-rolled `(x + sqrt(x²+1)).ln()` as a portability
/// workaround for pre-C++11 MSVC lacking `std::asinh`. We delegate to the
/// platform `std::f64::asinh` instead — it is the correct, libm-optimized
/// implementation (and avoids the small-`x` cancellation of the naive form).
/// Kept here so dynamics consumes math through a single entry point.
#[inline]
pub fn asinh(x: f64) -> f64 {
    x.asinh()
}

/// Inverse hyperbolic cosine.
///
/// Orbiter (`Vecmat.h:65`) hand-rolled `(x + sqrt(x²-1)).ln()` as a portability
/// workaround (its `// note: sign undefined` comment acknowledges the formula's
/// weakness). That naive form cancels catastrophically near `x = 1` — exactly
/// where hyperbolic near-periapsis evaluation lands, since the argument
/// `(e + cos tra)/(1 + e·cos tra)` equals 1 at periapsis. We delegate to the
/// platform `std::f64::acosh` instead, which is the correct, libm-optimized
/// implementation. Returns NaN for `x < 1` (domain error). Kept here so dynamics
/// consumes math through a single entry point.
#[inline]
pub fn acosh(x: f64) -> f64 {
    x.acosh()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rad_deg_roundtrip() {
        let v = 45.0;
        assert!((deg(rad(v)) - v).abs() < 1e-12);
    }

    #[test]
    fn diff_angle_wraps() {
        // 350° vs 10° → shortest separation is 20° (not 340°).
        let d = diff_angle(rad(10.0), rad(350.0));
        assert!((d - rad(20.0)).abs() < 1e-9, "got {}", deg(d));
    }

    #[test]
    fn au_value() {
        assert!((AU - 1.495978707e11).abs() < 1.0e3);
    }

    #[test]
    fn pos_angle_wraps() {
        assert!((pos_angle(-0.1) - (PI2 - 0.1)).abs() < 1e-12);
        assert!((pos_angle(0.0)).abs() < 1e-12);
        assert!((pos_angle(PI05) - PI05).abs() < 1e-12);
        assert!((pos_angle(PI2 + 0.5) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn atan2_0_2pi_range() {
        // (1,0) → 0; (0,1) → π/2; (-1,0) → π; (0,-1) → 3π/2
        assert!((atan2_0_2pi(0.0, 1.0)).abs() < 1e-12);
        assert!((atan2_0_2pi(1.0, 0.0) - PI05).abs() < 1e-12);
        assert!((atan2_0_2pi(0.0, -1.0) - PI).abs() < 1e-12);
        assert!((atan2_0_2pi(-1.0, 0.0) - (PI + PI05)).abs() < 1e-12);
    }

    #[test]
    fn acosh_matches_stdlib() {
        // acosh delegates to std::f64::acosh — exact by construction.
        for &x in &[1.0_f64, 1.5, 2.0, 10.0, 1.0e6, 1.0e12] {
            assert_eq!(acosh(x), x.acosh());
        }
    }

    #[test]
    fn acosh_near_one_accurate() {
        // The naive Orbiter formula cancels here; the stdlib delegate stays
        // exact (it IS stdlib) down to 1+1e-12.
        for &eps in &[1e-2_f64, 1e-4, 1e-6, 1e-8, 1e-12] {
            let x = 1.0 + eps;
            assert_eq!(acosh(x), x.acosh(), "acosh(1+{eps})");
        }
    }

    #[test]
    fn acosh_domain() {
        assert!(acosh(0.5).is_nan());
        assert!(acosh(-1.0).is_nan());
        assert_eq!(acosh(1.0), 0.0);
    }

    #[test]
    fn asinh_matches_stdlib() {
        // asinh delegates to std::f64::asinh — exact by construction.
        for &x in &[0.0_f64, 1e-8, 1e-4, 0.5, 1.0, 10.0, 1.0e6, -1.0, -1e-4] {
            assert_eq!(asinh(x), x.asinh(), "asinh({x})");
        }
    }
}
