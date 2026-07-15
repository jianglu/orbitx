//! Astronomical time and coordinate conversions — mirrors `Astro.h` /
//! `Astro.cpp`.
//!
//! All routines are faithful ports of the C++ source. Date conversions use the
//! [`CivilDate`] struct in lieu of C's `struct tm`; note the field semantics
//! match `struct tm` (`tm_year = year - 1900`, `tm_mon = 0-11`).

use crate::consts::{AU, IAU, MJD2000, PI, PI05, PI2};

/// Gregorian date/time fields, mirroring the relevant subset of C `struct tm`.
///
/// `year` is the full 4-digit year (unlike `tm_year`); `month` is 1-12 (unlike
/// `tm_mon` which is 0-11). The conversion functions translate accordingly.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CivilDate {
    pub year: i32,
    pub month: i32, // 1-12
    pub day: i32,
    pub hour: i32,
    pub min: i32,
    pub sec: i32,
}

/// Modified Julian Date from Unix seconds (`MJD`, Astro.h:50).
#[inline]
pub fn mjd_from_unix(unix_secs: i64) -> f64 {
    40587.0 + (unix_secs as f64) / 86400.0
}

/// Julian Date from Unix seconds (`JD`, Astro.h:45).
#[inline]
pub fn jd_from_unix(unix_secs: i64) -> f64 {
    2440587.5 + (unix_secs as f64) / 86400.0
}

/// Julian epoch → MJD (`Jepoch2MJD`, Astro.h:55).
#[inline]
pub fn jepoch_to_mjd(j: f64) -> f64 {
    (j - 2000.0) * 365.25 + MJD2000
}

/// MJD → Julian epoch (`MJD2Jepoch`, Astro.h:59).
#[inline]
pub fn mjd_to_jepoch(mjd: f64) -> f64 {
    2000.0 + (mjd - MJD2000) / 365.25
}

/// Julian century → MJD (`JC2MJD`, Astro.h:63).
#[inline]
pub fn jc_to_mjd(jc: f64) -> f64 {
    jc * 36525.0 + MJD2000
}

/// MJD → Julian century (`MJD2JC`, Astro.h:67).
#[inline]
pub fn mjd_to_jc(mjd: f64) -> f64 {
    (mjd - MJD2000) / 36525.0
}

/// Convert a Gregorian date to MJD (`date2mjd`, Astro.cpp:12).
///
/// Uses the Julian/Gregorian split at `10000*y + 100*m + d <= 15821004.1`,
/// matching the C++ exactly (integer division semantics).
///
/// **Note**: Orbiter's `date2mjd` uses a `tm_mon` convention where months are
/// 1-12 (NOT the standard C `struct tm` 0-11). The companion [`mjd_to_date`]
/// produces matching 1-based months. This matches the C++ round-trip exactly.
pub fn date_to_mjd(date: CivilDate) -> f64 {
    let mut y = date.year;
    let mut m = date.month; // Orbiter's tm_mon is 1-12, matching CivilDate.month
    let d = date.day;
    let a = 10000.0 * (y as f64) + 100.0 * (m as f64) + (d as f64);
    if m <= 2 {
        m += 12;
        y -= 1;
    }
    let b: i32 = if a <= 15821004.1 {
        (y + 4716) / 4 - 1181
    } else {
        y / 400 - y / 100 + y / 4
    };
    365.0 * (y as f64) - 679004.0
        + (b as f64)
        + (30.6001 * (m + 1) as f64).floor()
        + (d as f64)
        + (date.hour as f64) / 24.0
        + (date.min as f64) / 1440.0
        + (date.sec as f64) / 86400.0
}

/// Convert an MJD to a Gregorian date (`mjddate`, Astro.cpp:27).
pub fn mjd_to_date(mjd: f64) -> CivilDate {
    let ijd = mjd.floor();
    let h = 24.0 * (mjd - ijd);
    let ijd_i = ijd as i64;

    let c = if ijd_i < -100840 {
        ijd + 2401525.0
    } else {
        let b = (((ijd + 532784.75) / 36524.25) as i64) as i32;
        ijd + 2401526.0 + (b - b / 4) as f64
    };
    let a = (((c - 122.1) / 365.25) as i64) as i32;
    let e = 365.0 * (a as f64) + ((a / 4) as f64);
    let f = (((c - e) / 30.6001) as i64) as i32;

    let wday = (((ijd_i + 3) % 7) as i32).rem_euclid(7);
    let mday = ((c - e + 0.5) as i64) as i32 - (30.6001 * (f as f64)) as i32;
    let mon = f - 1 - 12 * (f / 14);
    let year = a - 4715 - ((7 + mon) / 10);
    let hour = h as i32;
    let h2 = 60.0 * (h - (hour as f64));
    let min = h2 as i32;
    let h3 = 60.0 * (h2 - (min as f64));
    let sec = h3 as i32;

    // wday is not stored in CivilDate currently; kept for parity via debug only.
    let _ = wday;
    CivilDate {
        year,
        month: mon,
        day: mday,
        hour,
        min,
        sec,
    }
}

/// Earth's obliquity [rad] at Julian century `jc` (`Obliquity`, Astro.cpp:86).
#[inline]
pub fn obliquity(jc: f64) -> f64 {
    0.4090928042 - (2.269655248e-4 + (2.860400719e-9 - 8.789672039e-9 * jc) * jc) * jc
}

/// Equatorial → ecliptic conversion (`Equ2Ecl`, Astro.cpp:91).
///
/// Arguments: `cosob`, `sinob` (cosine/sine of obliquity — **cosine first**),
/// right ascension `ra`, declination `dc`. Returns `(longitude, latitude)`.
pub fn equ_to_ecl(cosob: f64, sinob: f64, ra: f64, dc: f64) -> (f64, f64) {
    let sinra = ra.sin();
    let cosra = ra.cos();
    let sindc = dc.sin();
    let cosdc = dc.cos();
    let l = (sinra * cosdc * cosob + sindc * sinob).atan2(cosra * cosdc);
    let b = (sindc * cosob - sinra * cosdc * sinob).asin();
    (l, b)
}

/// Ecliptic → equatorial conversion (`Ecl2Equ`, Astro.cpp:99).
///
/// Arguments: `cosob`, `sinob` (cosine first), longitude `l`, latitude `b`.
/// Returns `(right ascension, declination)`.
pub fn ecl_to_equ(cosob: f64, sinob: f64, l: f64, b: f64) -> (f64, f64) {
    let sinl = l.sin();
    let cosl = l.cos();
    let sinb = b.sin();
    let cosb = b.cos();
    let ra = (sinl * cosb * cosob - sinb * sinob).atan2(cosb * cosl);
    let dc = (sinb * cosob + cosb * sinl * sinob).asin();
    (ra, dc)
}

/// Great-circle distance [rad] between two points (`Orthodrome` dist only,
/// Astro.cpp:131). All angles in radians.
pub fn orthodrome_dist(lng1: f64, lat1: f64, lng2: f64, lat2: f64) -> f64 {
    let cosa = lat2.sin() * lat1.sin() + lat2.cos() * lat1.cos() * (lng2 - lng1).cos();
    cosa.clamp(-1.0, 1.0).acos()
}

/// Great-circle distance [rad] and initial bearing [rad, in `[0, 2π)`]
/// (`Orthodrome` dist+dir, Astro.cpp:107).
pub fn orthodrome(lng1: f64, lat1: f64, lng2: f64, lat2: f64) -> (f64, f64) {
    let a = lng2 - lng1;
    let dlng = a.abs();
    let dlat = (lat2 - lat1).abs();
    if dlat < 1e-14 {
        let dir = if lng2 > lng1 { PI05 } else { 3.0 * PI05 };
        return (dlng, dir);
    } else if dlng < 1e-14 {
        let dir = if lat2 > lat1 { 0.0 } else { PI };
        return (dlat, dir);
    }
    let sina = a.sin();
    let cosa = a.cos();
    let slat1 = lat1.sin();
    let clat1 = lat1.cos();
    let slat2 = lat2.sin();
    let clat2 = lat2.cos();
    let cosa2 = slat2 * slat1 + clat2 * clat1 * cosa;
    let dist = cosa2.clamp(-1.0, 1.0).acos();
    let mut dir = (sina * clat2).atan2(clat1 * slat2 - slat1 * clat2 * cosa);
    if dir < 0.0 {
        dir += PI2;
    }
    (dist, dir)
}

/// Format a distance with SI/AU suffixes (`DistStr`, Astro.cpp:142). Returns an
/// owned `String` (the C++ version uses a static buffer).
pub fn dist_str(dist: f64, precision: i32) -> String {
    let absd = dist.abs();
    // Helper: right-justify in width 8 with given decimal precision, optional suffix.
    let fmt = |val: f64, prec: i32, suffix: &str| -> String {
        format!("{:8.*}{}", prec.max(0) as usize, val, suffix)
    };
    if absd < 1e4 {
        if absd < 1e2 {
            fmt(dist, precision - 2, "")
        } else if absd < 1e3 {
            fmt(dist, precision - 3, "")
        } else {
            fmt(dist * 1e-3, precision - 1, "k")
        }
    } else if absd < 1e7 {
        if absd < 1e5 {
            fmt(dist * 1e-3, precision - 2, "k")
        } else if absd < 1e6 {
            fmt(dist * 1e-3, precision - 3, "k")
        } else {
            fmt(dist * 1e-6, precision - 1, "M")
        }
    } else if absd < 1e10 {
        if absd < 1e8 {
            fmt(dist * 1e-6, precision - 2, "M")
        } else if absd < 1e9 {
            fmt(dist * 1e-6, precision - 3, "M")
        } else {
            fmt(dist * 1e-9, precision - 1, "G")
        }
    } else if absd < 1e2 * AU {
        if absd < 1e11 {
            fmt(dist * 1e-9, precision - 2, "G")
        } else if absd < 1e12 {
            fmt(dist * 1e-9, precision - 3, "G")
        } else {
            fmt(dist * IAU, precision - 2, "AU")
        }
    } else {
        format!("{:8.0}AU", dist * IAU)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn j2000_epoch_roundtrip() {
        // J2000.0 = MJD 51544.5.
        assert!((jepoch_to_mjd(2000.0) - 51544.5).abs() < 1e-9);
        assert!((mjd_to_jepoch(51544.5) - 2000.0).abs() < 1e-9);
        assert!((mjd_to_jc(51544.5)).abs() < 1e-9);
    }

    #[test]
    fn date_to_mjd_j2000() {
        // J2000 epoch: 2000-01-01 12:00:00 TT = MJD 51544.5.
        // Orbiter's date2mjd uses 1-based tm_mon; mjddate round-trips correctly.
        let d = CivilDate {
            year: 2000,
            month: 1,
            day: 1,
            hour: 12,
            min: 0,
            sec: 0,
        };
        let mjd = date_to_mjd(d);
        assert!((mjd - 51544.5).abs() < 1e-6, "got {mjd}");
    }

    #[test]
    fn obliquity_j2000() {
        // At J2000, obliquity ≈ 23.439291° = 0.40909280 rad.
        let obl = obliquity(0.0);
        assert!((obl - 0.4090928042).abs() < 1e-9, "got {obl}");
    }

    #[test]
    fn orthodrome_antipode() {
        // Antipodal points are π apart.
        let (dist, _) = orthodrome(0.0, 0.0, std::f64::consts::PI, 0.0);
        assert!((dist - std::f64::consts::PI).abs() < 1e-9, "got {dist}");
    }
}
