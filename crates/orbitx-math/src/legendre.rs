//! Normalized associated Legendre functions and complex-power recurrence.
//!
//! Pure special-function math (no gravity physics); used by the Pines
//! spherical-harmonic gravity model in `orbitx-dynamics`. Mirrors
//! `GenerateAssocLegendreMatrix` (PinesGrav.cpp:75) and the `(s + it)^m`
//! recurrence inside `GetPinesGrav` (PinesGrav.cpp:185).

/// Triangular index: maps `(n, m)` to a flat array index.
///
/// Packs the lower-triangular `(n, m)` layout used by the Pines coefficient
/// arrays and the associated Legendre matrix.
#[inline]
pub fn nm(n: usize, m: usize) -> usize {
    (n * n + n) / 2 + m
}

/// Generate the normalized associated Legendre function matrix.
///
/// Mirrors `GenerateAssocLegendreMatrix` (PinesGrav.cpp:75).
///
/// Returns a flat array indexed by `nm(n, m)`, for `0 <= n, m <= max_degree + 2`.
/// The argument `u` is the polar direction cosine of the evaluation point.
pub fn generate_assoc_legendre(u: f64, max_degree: usize) -> Vec<f64> {
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

/// Real and imaginary parts of `(s + i·t)^m` for `m = 0..=max_m` via recurrence.
///
/// Mirrors the recurrence inside `GetPinesGrav` (PinesGrav.cpp:152-163).
/// Returns `(re, im)`, each of length `max_m + 2`, with
/// `re[0] = 0, im[0] = 0, re[1] = 1, im[1] = 0`, and for `m = 2..=max_m`:
/// `re[m] = s·re[m-1] − t·im[m-1]`, `im[m] = s·im[m-1] + t·re[m-1]`.
///
/// `s` and `t` are the in-plane direction cosines of the evaluation point.
pub fn complex_power_recurrence(s: f64, t: f64, max_m: usize) -> (Vec<f64>, Vec<f64>) {
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
    (re, im)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nm_triangular() {
        // (0,0)->0, (1,0)->1, (1,1)->2, (2,0)->3, (2,1)->4, (2,2)->5
        assert_eq!(nm(0, 0), 0);
        assert_eq!(nm(1, 0), 1);
        assert_eq!(nm(1, 1), 2);
        assert_eq!(nm(2, 0), 3);
        assert_eq!(nm(2, 1), 4);
        assert_eq!(nm(2, 2), 5);
    }

    #[test]
    fn assoc_legendre_a00_is_one_after_m0_scaling() {
        // a[nm(0,0)] starts at sqrt(2), then the m=0 column is scaled by
        // sqrt(0.5), so the final value is sqrt(2)*sqrt(0.5) = 1.
        let a = generate_assoc_legendre(0.5, 2);
        assert!((a[nm(0, 0)] - 1.0).abs() < 1e-12, "got {}", a[nm(0, 0)]);
    }

    #[test]
    fn complex_power_recurrence_unit_real() {
        // Index m holds (s + it)^(m-1): for s=1, t=0 this is 1 for all m >= 1.
        let (re, im) = complex_power_recurrence(1.0, 0.0, 4);
        assert_eq!(re[0], 0.0);
        assert_eq!(re[1], 1.0);
        for m in 2..=4 {
            assert!((re[m] - 1.0).abs() < 1e-12, "re[{m}]={}", re[m]);
            assert!(im[m].abs() < 1e-12, "im[{m}]={}", im[m]);
        }
    }

    #[test]
    fn complex_power_recurrence_imaginary() {
        // s=0, t=1: (s + it) = i. Index m holds (s+it)^(m-1), so:
        //   index 1 → i^0 = 1,  index 2 → i^1 = i,  index 3 → i^2 = -1,
        //   index 4 → i^3 = -i.
        let (re, im) = complex_power_recurrence(0.0, 1.0, 4);
        assert!((re[1] - 1.0).abs() < 1e-12 && im[1].abs() < 1e-12);
        assert!((re[2]).abs() < 1e-12 && (im[2] - 1.0).abs() < 1e-12);
        assert!((re[3] + 1.0).abs() < 1e-12 && im[3].abs() < 1e-12);
        assert!((re[4]).abs() < 1e-12 && (im[4] + 1.0).abs() < 1e-12);
    }
}
