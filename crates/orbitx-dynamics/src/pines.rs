//! Pines spherical-harmonic gravity model.
//!
//! Mirrors `PinesGravProp` (`Src/Orbiter/PinesGrav.h` / `PinesGrav.cpp`).
//! Implements Samuel Pines' uniform representation of the gravitational field
//! (AIAA Journal, Nov 1973) using normalized associated Legendre functions.
//!
//! **Important**: this module works in **km** units (matching the C++ which
//! converts m→km before calling Pines). The caller is responsible for unit
//! conversion at the boundary.

use std::io::{BufRead, BufReader};

/// Triangular index: maps `(n, m)` to a flat array index.
#[inline]
pub fn nm(n: usize, m: usize) -> usize {
    (n * n + n) / 2 + m
}

/// Pines spherical-harmonic gravity model.
///
/// Coefficients `c` and `s` are packed via the triangular index `NM(n,m)`.
/// `c[0] = s[0] = 0` deliberately so only the perturbation (non-point-mass)
/// acceleration is returned.
pub struct PinesModel {
    /// Reference radius [km].
    pub ref_rad: f64,
    /// GM [km³/s²] (from model file header).
    pub gm: f64,
    /// Maximum degree loaded from file.
    pub degree: usize,
    /// Maximum order loaded from file.
    pub order: usize,
    /// Whether coefficients are normalized.
    pub normalized: bool,
    /// Cnm cosine coefficients (packed via NM).
    pub c: Vec<f64>,
    /// Snm sine coefficients (packed via NM).
    pub s: Vec<f64>,
}

impl PinesModel {
    /// Read a gravity model from a `.tab`/`.sha` file reader.
    ///
    /// Mirrors `PinesGravProp::readGravModel` (PinesGrav.cpp:110).
    ///
    /// File format:
    /// - Header: `refRad, GM, <ignored>, order, degree, normalized, refLat, refLon`
    /// - Each line: `n, m, Cnm, Snm, <ignored>, <ignored>`
    /// - `cutoff` limits the degree/order actually loaded.
    pub fn from_reader<R: BufRead>(reader: R, cutoff: usize) -> std::io::Result<Self> {
        let mut lines = BufReader::new(reader).lines();
        let header = lines
            .next()
            .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidData, "empty file"))??;

        // Parse header: comma-separated.
        let parts: Vec<&str> = header.split(',').collect();
        if parts.len() < 6 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "bad header format",
            ));
        }
        let ref_rad = parts[0].trim().parse::<f64>().map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, format!("refRad: {e}"))
        })?;
        let gm = parts[1].trim().parse::<f64>().map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, format!("GM: {e}"))
        })?;
        let file_order = parts[3].trim().parse::<usize>().unwrap_or(0);
        let file_degree = parts[4].trim().parse::<usize>().unwrap_or(0);
        let normalized = parts.get(5).map(|s| s.trim() == "1").unwrap_or(true);

        let degree = cutoff.min(file_degree);
        let order = cutoff.min(file_order);

        // Allocate coefficient arrays.
        let array_size = nm(degree + 2, degree + 2) + 1;
        let mut c = vec![0.0_f64; array_size];
        let mut s = vec![0.0_f64; array_size];

        // Read coefficient lines.
        for line in lines {
            let line = line?;
            let fields: Vec<&str> = line.split(',').collect();
            if fields.len() < 4 {
                continue;
            }
            let n: usize = match fields[0].trim().parse() {
                Ok(v) => v,
                Err(_) => continue,
            };
            let m: usize = match fields[1].trim().parse() {
                Ok(v) => v,
                Err(_) => continue,
            };
            if n > degree || m > order {
                continue;
            }
            let cnm: f64 = fields[2].trim().parse().unwrap_or(0.0);
            let snm: f64 = fields[3].trim().parse().unwrap_or(0.0);
            let idx = nm(n, m);
            if idx < c.len() {
                c[idx] = cnm;
                s[idx] = snm;
            }
        }

        // C[0,0] and S[0,0] forced to zero (point mass handled separately).
        c[nm(0, 0)] = 0.0;
        s[nm(0, 0)] = 0.0;

        Ok(PinesModel {
            ref_rad,
            gm,
            degree,
            order,
            normalized,
            c,
            s,
        })
    }

    /// Compute the perturbation acceleration at position `rpos` [km].
    ///
    /// Returns acceleration [km/s²]. The point-mass GM/r² term is NOT included.
    ///
    /// Mirrors `PinesGravProp::GetPinesGrav` (PinesGrav.cpp:185).
    pub fn accel(&self, rpos: Vec3Pines, max_degree: usize, max_order: usize) -> Vec3Pines {
        let r = rpos.length();
        let s = rpos.x / r; // direction cosines
        let t = rpos.y / r;
        let u = rpos.z / r;

        let mut rho = self.gm / (r * self.ref_rad);
        let rhop = self.ref_rad / r;

        // Real and imaginary parts of (s + it)^m via recurrence.
        let max_m = max_order + 1;
        let mut re = vec![0.0_f64; max_m + 2];
        let mut im = vec![0.0_f64; max_m + 2];
        re[0] = 0.0;
        im[0] = 0.0;
        re[1] = 1.0;
        im[1] = 0.0;
        for m in 2..=max_m {
            re[m] = s * re[m - 1] - t * im[m - 1];
            im[m] = s * im[m - 1] + t * re[m - 1];
        }

        // Generate associated Legendre matrix.
        let a = generate_assoc_legendre(u, max_degree);

        let mut g1 = 0.0_f64;
        let mut g2 = 0.0_f64;
        let mut g3 = 0.0_f64;
        let mut g4 = 0.0_f64;

        for n in 0..=max_degree {
            let mut g1t = 0.0;
            let mut g2t = 0.0;
            let mut g3t = 0.0;
            let mut g4t = 0.0;

            let mut sm = 0.5_f64;
            let nmodel = if n > max_order { max_order } else { n };

            for m in 0..=nmodel {
                let idx = nm(n, m);
                let d = self.c[idx] * re[m + 1] + self.s[idx] * im[m + 1];
                let e = self.c[idx] * re[m] + self.s[idx] * im[m];
                let f = self.s[idx] * re[m] - self.c[idx] * im[m];

                let alpha = (sm * (n as f64 - m as f64) * (n as f64 + m as f64 + 1.0)).sqrt();

                g1t += a[nm(n, m)] * m as f64 * e;
                g2t += a[nm(n, m)] * m as f64 * f;
                g3t += alpha * a[nm(n, m + 1)] * d;
                g4t +=
                    ((n as f64 + m as f64 + 1.0) * a[nm(n, m)] + alpha * u * a[nm(n, m + 1)]) * d;

                if m == 0 {
                    sm = 1.0;
                }
            }
            rho *= rhop;

            g1 += rho * g1t;
            g2 += rho * g2t;
            g3 += rho * g3t;
            g4 += rho * g4t;
        }

        Vec3Pines {
            x: g1 - g4 * s,
            y: g2 - g4 * t,
            z: g3 - g4 * u,
        }
    }
}

/// Generate the normalized associated Legendre function matrix.
///
/// Mirrors `GenerateAssocLegendreMatrix` (PinesGrav.cpp:75).
///
/// Returns a flat array indexed by `NM(n, m)`, for `0 <= n,m <= maxDegree+2`.
fn generate_assoc_legendre(u: f64, max_degree: usize) -> Vec<f64> {
    let md2 = max_degree + 2;
    let mut a = vec![0.0_f64; nm(md2, md2) + 1];

    a[nm(0, 0)] = 2.0_f64.sqrt();

    for m in 0..=md2 {
        if m != 0 {
            // Diagonal terms.
            a[nm(m, m)] = (1.0 + 1.0 / (2.0 * m as f64)).sqrt() * a[nm(m - 1, m - 1)];
        }
        if m != md2 {
            // Off-diagonal terms.
            a[nm(m + 1, m)] = (2.0 * m as f64 + 3.0).sqrt() * u * a[nm(m, m)];
        }
        if m < max_degree + 1 {
            // Column recurrence.
            for n in (m + 2)..=md2 {
                let alpha_num = (2.0 * n as f64 + 1.0) * (2.0 * n as f64 - 1.0);
                let alpha_den = (n as f64 - m as f64) * (n as f64 + m as f64);
                let alpha = (alpha_num / alpha_den).sqrt();

                let beta_num = (2.0 * n as f64 + 1.0)
                    * (n as f64 - m as f64 - 1.0)
                    * (n as f64 + m as f64 - 1.0);
                let beta_den =
                    (2.0 * n as f64 - 3.0) * (n as f64 + m as f64) * (n as f64 - m as f64);
                let beta = (beta_num / beta_den).sqrt();

                a[nm(n, m)] = alpha * u * a[nm(n - 1, m)] - beta * a[nm(n - 2, m)];
            }
        }
    }

    // Scale m=0 column by sqrt(0.5).
    for n in 0..=md2 {
        a[nm(n, 0)] *= 0.5_f64.sqrt();
    }

    a
}

/// Lightweight 3D vector for the Pines module (works in km units, independent
/// of orbitx_math's Vec3 which operates in meters).
#[derive(Clone, Copy, Debug, Default)]
pub struct Vec3Pines {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3Pines {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn length(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_simple_model() {
        // A minimal spherical-harmonic model with just C(2,0) = J2.
        // refRad=6378.1363 km, GM=398600.4415 km³/s²
        let data = "6378.1363, 398600.4415, 0, 2, 2, 1, 0, 0\n\
                    2, 0, -0.00108263, 0.0, 0.0, 0.0\n\
                    2, 1, 0.0, 0.0, 0.0, 0.0\n\
                    2, 2, 0.0, 0.0, 0.0, 0.0\n";
        let model = PinesModel::from_reader(data.as_bytes(), 2).unwrap();
        assert!((model.ref_rad - 6378.1363).abs() < 1e-4);
        assert!((model.gm - 398600.4415).abs() < 1e-4);
        assert_eq!(model.degree, 2);
        assert_eq!(model.order, 2);
        // C(2,0) should be loaded.
        assert!((model.c[nm(2, 0)] - (-0.00108263)).abs() < 1e-10);
    }

    #[test]
    fn accel_at_pole_nonzero() {
        // At the pole, the J2 perturbation should be nonzero.
        let data = "6378.1363, 398600.4415, 0, 2, 2, 1, 0, 0\n\
                    2, 0, -0.00108263, 0.0, 0.0, 0.0\n";
        let model = PinesModel::from_reader(data.as_bytes(), 2).unwrap();
        // Position at 7000 km on the z-axis (pole in Pines' right-handed frame).
        let rpos = Vec3Pines::new(0.0, 0.0, 7000.0);
        let acc = model.accel(rpos, 2, 2);
        // The perturbation acceleration should be nonzero.
        let mag = (acc.x * acc.x + acc.y * acc.y + acc.z * acc.z).sqrt();
        assert!(mag > 1e-6, "Pines accel at pole = {mag}");
    }

    #[test]
    fn accel_decreases_with_distance() {
        let data = "6378.1363, 398600.4415, 0, 2, 2, 1, 0, 0\n\
                    2, 0, -0.00108263, 0.0, 0.0, 0.0\n";
        let model = PinesModel::from_reader(data.as_bytes(), 2).unwrap();
        let acc_near = model.accel(Vec3Pines::new(0.0, 0.0, 7000.0), 2, 2);
        let acc_far = model.accel(Vec3Pines::new(0.0, 0.0, 20000.0), 2, 2);
        let mag_near =
            (acc_near.x * acc_near.x + acc_near.y * acc_near.y + acc_near.z * acc_near.z).sqrt();
        let mag_far =
            (acc_far.x * acc_far.x + acc_far.y * acc_far.y + acc_far.z * acc_far.z).sqrt();
        assert!(mag_near > mag_far, "near={mag_near}, far={mag_far}");
    }
}
