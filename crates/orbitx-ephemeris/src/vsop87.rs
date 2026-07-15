//! VSOP87 planetary ephemeris: parser, evaluator, and fast-ephemeris driver.
//!
//! Mirrors Orbiter's `VSOPOBJ` class (Src/Celbody/Vsop87/Vsop87.cpp), supporting
//! series B (spherical: longitude, latitude, radius in AU) and series E
//! (rectangular: x, y, z in meters). The data layout and evaluation algorithm
//! are symbol-for-symbol replicas of the C++ implementation.

use crate::sample::{interpolate, Sample};
use std::io::{BufRead, BufReader};

/// Maximum power of time in the VSOP87 series (matches `VSOP_MAXALPHA`).
pub const VSOP_MAXALPHA: usize = 5;

/// VSOP87 series identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Series {
    /// Spherical heliocentric coordinates: longitude [rad], latitude [rad], radius [AU].
    /// Sets `EPHEM_POLAR`.
    B,
    /// Rectangular coordinates referenced to parent barycentre. Sets `EPHEM_PARENTBARY`.
    E,
}

impl Series {
    /// Character used in data-file names (e.g. `'B'` for `Vsop87B.dat`).
    pub fn char(self) -> char {
        match self {
            Series::B => 'B',
            Series::E => 'E',
        }
    }

    /// True when this series outputs spherical (polar) coordinates.
    pub fn is_polar(self) -> bool {
        matches!(self, Series::B)
    }
}

/// A single VSOP87 term: `a * cos(b + c * t)`.
#[derive(Clone, Copy, Debug, Default)]
pub struct VsopTerm {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

/// Complete VSOP87 model loaded from a `.dat` file.
pub struct VsopModel {
    /// Maximum power of time (typically 5).
    pub nalpha: usize,
    /// Series identifier (B or E).
    pub series: Series,
    /// Semi-major axis [AU], used to normalise the radius coordinate.
    pub a0: f64,
    /// Precision tolerance used during term filtering.
    pub prec: f64,
    /// Sampling interval [s] for the fast-ephemeris sliding window.
    pub interval: f64,
    /// Flat array of all retained terms.
    pub terms: Vec<VsopTerm>,
    /// `termidx[alpha][cooidx]` = start offset into `terms`.
    pub termidx: [[usize; 3]; VSOP_MAXALPHA + 1],
    /// `termlen[alpha][cooidx]` = number of terms used (sentinel row `nalpha+1` = 0).
    pub termlen: [[usize; 3]; VSOP_MAXALPHA + 2],
    /// Two interpolation samples for fast ephemeris.
    pub sp: [Sample; 2],
}

// --- Physical constants (from Vsop87.cpp:148-153) ---
const MJD2000: f64 = 51544.5;
const A1000: f64 = 365_250.0; // days per millennium
const RSEC: f64 = 1.0 / (A1000 * 86400.0); // 1/seconds per millennium
const C0: f64 = 299_792_458.0; // speed of light [m/s]
const TAU_A: f64 = 499.004783806; // light time for 1 AU [s]
const AU_METERS: f64 = C0 * TAU_A; // 1 AU in meters
const PI: f64 = std::f64::consts::PI;

impl VsopModel {
    /// Read VSOP87 data from a reader, applying precision-based term filtering.
    ///
    /// Exactly mirrors `VSOPOBJ::ReadData` (Vsop87.cpp:60).
    pub fn from_reader<R: BufRead>(
        reader: R,
        series: Series,
        a0: f64,
        prec: f64,
        interval: f64,
    ) -> std::io::Result<Self> {
        let mut tokens = TokenStream::new(reader);

        let nalpha: usize = tokens.next_int()? as usize;

        // Temporary storage: per (cooidx, alpha) group of raw terms.
        let mut groups: Vec<Vec<VsopTerm>> = vec![Vec::new(); 3 * (nalpha + 1)];

        let mut termidx = [[0usize; 3]; VSOP_MAXALPHA + 1];
        let mut termlen = [[0usize; 3]; VSOP_MAXALPHA + 2];

        let mut nused = 0usize;

        for cooidx in 0..3 {
            let mut tfac = 1.0_f64;
            for alpha in 0..=nalpha {
                let nterm = tokens.next_int()? as usize;
                let mut group = Vec::with_capacity(nterm);
                let mut iused = nterm;

                for i in 0..nterm {
                    let a = tokens.next_f64()?;
                    let b = tokens.next_f64()?;
                    let c = tokens.next_f64()?;

                    if iused == nterm {
                        group.push(VsopTerm { a, b, c });
                        // Compute the effective amplitude for error estimation.
                        // cooidx==2 (radius) is normalised by a0 before the check.
                        let a_eff = if cooidx == 2 { a / a0 } else { a };
                        let err = 2.0 * ((i as f64) + 1.0).sqrt() * a_eff * tfac;
                        if err < prec {
                            iused = i;
                        }
                    }
                }

                // Only keep `iused` terms (the rest were truncated).
                group.truncate(iused);

                termlen[alpha][cooidx] = iused;
                termidx[alpha][cooidx] = nused;
                nused += iused;

                let gi = cooidx * (nalpha + 1) + alpha;
                groups[gi] = group;

                tfac *= 5.0; // don't ask
            }
            // Sentinel: mark the alpha just beyond the last as 0 for the evaluation loop.
            termlen[nalpha + 1][cooidx] = 0;
        }

        // Flatten into a single array.
        let mut terms = Vec::with_capacity(nused);
        for cooidx in 0..3 {
            for alpha in 0..=nalpha {
                let gi = cooidx * (nalpha + 1) + alpha;
                // Normalise radius terms by a0 (cooidx == 2).
                for t in &groups[gi] {
                    let a = if cooidx == 2 { t.a / a0 } else { t.a };
                    terms.push(VsopTerm { a, b: t.b, c: t.c });
                }
            }
        }

        let mut model = VsopModel {
            nalpha,
            series,
            a0,
            prec,
            interval,
            terms,
            termidx,
            termlen,
            sp: [Sample::default(), Sample::default()],
        };
        model.init_samples();
        Ok(model)
    }

    /// Initialise the two fast-ephemeris samples (mirrors `VSOPOBJ::Init`).
    fn init_samples(&mut self) {
        self.sp[0].t = 0.0;
        self.sp[1].t = self.interval;
        let mjd1 = self.sp[1].t / 86400.0 + MJD2000;
        vsop_eval_raw(
            MJD2000,
            self.series,
            &self.terms,
            &self.termidx,
            &self.termlen,
            &mut self.sp[0].param,
        );
        vsop_eval_raw(
            mjd1,
            self.series,
            &self.terms,
            &self.termidx,
            &self.termlen,
            &mut self.sp[1].param,
        );
        self.sp[0].rad = radius(&self.sp[0].param);
        self.sp[1].rad = radius(&self.sp[1].param);
    }

    /// Evaluate the VSOP87 series at MJD `mjd` and write position+velocity to
    /// `ret[0..5]`.
    ///
    /// Mirrors `VSOPOBJ::VsopEphem` (Vsop87.cpp:141).
    pub fn eval_into(&self, mjd: f64, ret: &mut [f64; 6]) {
        vsop_eval_raw(
            mjd,
            self.series,
            &self.terms,
            &self.termidx,
            &self.termlen,
            ret,
        );
    }

    /// Evaluate at MJD and return a fresh `[f64; 6]`.
    pub fn eval(&self, mjd: f64) -> [f64; 6] {
        let mut ret = [0.0; 6];
        self.eval_into(mjd, &mut ret);
        ret
    }

    /// Sliding-window fast ephemeris interpolation.
    ///
    /// Mirrors `VSOPOBJ::VsopFastEphem` (Vsop87.cpp:216). `simt` is simulation
    /// time in seconds (with `MJD_ref = 51544.5` assumed, i.e. `simt` is
    /// seconds since J2000).
    pub fn fast_eval(&mut self, simt: f64) -> [f64; 6] {
        let mut ret = [0.0; 6];
        self.fast_eval_into(simt, &mut ret);
        ret
    }

    /// Same as [`fast_eval`](Self::fast_eval) but writes into a provided array.
    pub fn fast_eval_into(&mut self, simt: f64, ret: &mut [f64; 6]) {
        // Order the two samples so s0 is the earlier one.
        let (i0, i1) = if self.sp[0].t < self.sp[1].t {
            (0, 1)
        } else {
            (1, 0)
        };

        let s0_t = self.sp[i0].t;
        let s1_t = self.sp[i1].t;

        if simt >= s0_t && simt <= s1_t {
            // Interpolate within the window.
            interpolate(simt, ret, &self.sp[i0], &self.sp[i1]);
        } else if simt > s1_t {
            if simt <= s1_t + self.interval {
                // Advance the older sample by one interval.
                let new_t = s1_t + self.interval;
                self.sp[i0].t = new_t;
                let mjd = new_t / 86400.0 + MJD2000;
                vsop_eval_raw(
                    mjd,
                    self.series,
                    &self.terms,
                    &self.termidx,
                    &self.termlen,
                    &mut self.sp[i0].param,
                );
                if self.series.is_polar() {
                    // Check for phase wrap in longitude.
                    let diff = self.sp[i0].param[0] - self.sp[i1].param[0];
                    if diff > PI {
                        self.sp[i1].param[0] += 2.0 * PI;
                    } else if diff < -PI {
                        self.sp[i1].param[0] -= 2.0 * PI;
                    }
                } else {
                    self.sp[i0].rad = radius(&self.sp[i0].param);
                }
                interpolate(simt, ret, &self.sp[i1], &self.sp[i0]);
            } else {
                // Too far ahead: recompute at simt directly.
                self.sp[i0].t = simt;
                let mjd = simt / 86400.0 + MJD2000;
                vsop_eval_raw(
                    mjd,
                    self.series,
                    &self.terms,
                    &self.termidx,
                    &self.termlen,
                    &mut self.sp[i0].param,
                );
                if !self.series.is_polar() {
                    self.sp[i0].rad = radius(&self.sp[i0].param);
                }
                ret.copy_from_slice(&self.sp[i0].param);
            }
        } else {
            // simt < s0_t: looking backward.
            if simt >= s0_t - self.interval {
                let new_t = s0_t - self.interval;
                self.sp[i1].t = new_t;
                let mjd = new_t / 86400.0 + MJD2000;
                vsop_eval_raw(
                    mjd,
                    self.series,
                    &self.terms,
                    &self.termidx,
                    &self.termlen,
                    &mut self.sp[i1].param,
                );
                if self.series.is_polar() {
                    let diff = self.sp[i1].param[0] - self.sp[i0].param[0];
                    if diff > PI {
                        self.sp[i0].param[0] += 2.0 * PI;
                    } else if diff < -PI {
                        self.sp[i0].param[0] -= 2.0 * PI;
                    }
                } else {
                    self.sp[i1].rad = radius(&self.sp[i1].param);
                }
                interpolate(simt, ret, &self.sp[i1], &self.sp[i0]);
            } else {
                self.sp[i1].t = simt;
                let mjd1 = simt / 86400.0 + MJD2000;
                vsop_eval_raw(
                    mjd1,
                    self.series,
                    &self.terms,
                    &self.termidx,
                    &self.termlen,
                    &mut self.sp[i1].param,
                );
                let new_t0 = simt + self.interval;
                self.sp[i0].t = new_t0;
                let mjd0 = new_t0 / 86400.0 + MJD2000;
                vsop_eval_raw(
                    mjd0,
                    self.series,
                    &self.terms,
                    &self.termidx,
                    &self.termlen,
                    &mut self.sp[i0].param,
                );
                if self.series.is_polar() {
                    let diff = self.sp[i0].param[0] - self.sp[i1].param[0];
                    if diff > PI {
                        self.sp[i1].param[0] += 2.0 * PI;
                    } else if diff < -PI {
                        self.sp[i1].param[0] -= 2.0 * PI;
                    }
                } else {
                    self.sp[i0].rad = radius(&self.sp[i0].param);
                    self.sp[i1].rad = radius(&self.sp[i1].param);
                }
                ret.copy_from_slice(&self.sp[i1].param);
            }
        }
    }
}

/// Core VSOP87 series evaluation (mirrors `VSOPOBJ::VsopEphem`, Vsop87.cpp:141).
///
/// Extracted as a free function so that `fast_eval_into` can call it without
/// borrowing `self` while also mutating `self.sp[...]`.
fn vsop_eval_raw(
    mjd: f64,
    series: Series,
    terms: &[VsopTerm],
    termidx: &[[usize; 3]; VSOP_MAXALPHA + 1],
    termlen: &[[usize; 3]; VSOP_MAXALPHA + 2],
    ret: &mut [f64; 6],
) {
    // Zero result.
    *ret = [0.0; 6];

    // Time powers.
    let mut t = [0.0_f64; VSOP_MAXALPHA + 1];
    t[0] = 1.0;
    t[1] = (mjd - MJD2000) / A1000;
    for i in 2..=VSOP_MAXALPHA {
        t[i] = t[i - 1] * t[1];
    }

    // Term summation.
    for cooidx in 0..3 {
        let mut alpha = 0;
        while termlen[alpha][cooidx] != 0 {
            let start = termidx[alpha][cooidx];
            let len = termlen[alpha][cooidx];

            let mut tm = 0.0;
            let mut termdot = 0.0;
            for i in 0..len {
                let term = terms[start + i];
                let arg = term.b + term.c * t[1];
                tm += term.a * arg.cos();
                termdot -= term.c * term.a * arg.sin();
            }

            ret[cooidx] += t[alpha] * tm;
            ret[cooidx + 3] += t[alpha] * termdot
                + if alpha > 0 {
                    alpha as f64 * t[alpha - 1] * tm
                } else {
                    0.0
                };

            alpha += 1;
        }
    }

    // Output scaling.
    if series.is_polar() {
        // Polar: convert millennium rate to second rate for velocity.
        for v in ret.iter_mut().take(6).skip(3) {
            *v *= RSEC;
        }
        // Radius stays in AU (polar output).
    } else {
        // Rectangular: scale position to meters, velocity to m/s.
        for v in ret.iter_mut().take(3) {
            *v *= AU_METERS;
        }
        for v in ret.iter_mut().take(6).skip(3) {
            *v *= AU_METERS * RSEC;
        }
        // Swap y and z to map to Orbiter's left-handed convention.
        swap(ret, 1, 2);
        swap(ret, 4, 5);
    }
}

/// Compute radial distance from a position array (mirrors `Radius()` in Vsop87.cpp).
fn radius(data: &[f64; 6]) -> f64 {
    (data[0] * data[0] + data[1] * data[1] + data[2] * data[2]).sqrt()
}

fn swap(arr: &mut [f64; 6], i: usize, j: usize) {
    arr.swap(i, j);
}

// --- Whitespace-token stream parser ---

/// A simple whitespace-delimited token reader (mirrors C++ `ifstream >>`).
struct TokenStream<R> {
    lines: std::io::Lines<BufReader<R>>,
    current_line: String,
    pos: usize,
}

impl<R: BufRead> TokenStream<R> {
    fn new(reader: R) -> Self {
        TokenStream {
            lines: BufReader::new(reader).lines(),
            current_line: String::new(),
            pos: 0,
        }
    }

    fn next_token(&mut self) -> std::io::Result<&str> {
        loop {
            // Try to extract a token from the current line.
            let trimmed = self.current_line[self.pos..].trim_start();
            let start = self.current_line.len() - trimmed.len();
            if let Some(end) = trimmed.find(char::is_whitespace) {
                self.pos = start + end;
                return Ok(&self.current_line[start..start + end]);
            } else if !trimmed.is_empty() {
                self.pos = self.current_line.len();
                return Ok(&self.current_line[start..]);
            }
            // Need a new line.
            match self.lines.next() {
                Some(Ok(line)) => {
                    self.current_line = line;
                    self.pos = 0;
                }
                Some(Err(e)) => return Err(e),
                None => {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "unexpected end of VSOP87 data",
                    ));
                }
            }
        }
    }

    fn next_f64(&mut self) -> std::io::Result<f64> {
        let tok = self.next_token()?;
        tok.parse::<f64>()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }

    fn next_int(&mut self) -> std::io::Result<i64> {
        let tok = self.next_token()?;
        tok.parse::<i64>()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tiny synthetic VSOP87B dataset with known results.
    ///
    /// nalpha=1, so 3 coords × 2 alphas = 6 groups.
    /// We use a single term: a=1.0, b=0.0, c=1.0 for L,alpha0.
    fn make_simple_data() -> String {
        // nalpha
        let mut s = String::from("1\n");
        // coord 0 (L): alpha 0 — 1 term, alpha 1 — 0 terms
        s.push_str("1\n1.0 0.0 1.0\n0\n");
        // coord 1 (B): alpha 0 — 0 terms, alpha 1 — 0 terms
        s.push_str("0\n0\n");
        // coord 2 (R): alpha 0 — 1 term (a=1.0 in AU), alpha 1 — 0 terms
        s.push_str("1\n1.0 0.0 0.0\n0\n");
        s
    }

    #[test]
    fn parse_simple_model() {
        let data = make_simple_data();
        let model = VsopModel::from_reader(
            data.as_bytes(),
            Series::B,
            1.0,
            1e-30, // very tight precision so no terms get filtered
            10.0,
        )
        .unwrap();

        assert_eq!(model.nalpha, 1);
        // L(alpha0): 1 term; R(alpha0): 1 term; total 2.
        assert_eq!(model.terms.len(), 2);
    }

    #[test]
    fn eval_at_j2000_polar() {
        // At J2000 (mjd=51544.5), t[1]=0.0.
        // L = 1.0*cos(0+1.0*0) = 1.0*cos(0) = 1.0 rad
        // B = 0.0
        // R = 1.0/a0 * a0 = 1.0 AU (normalised)
        // Velocity: termdot = -c*a*sin(b+c*t1) = -1*1*sin(0) = 0
        let data = make_simple_data();
        let model = VsopModel::from_reader(data.as_bytes(), Series::B, 1.0, 1e-30, 10.0).unwrap();

        let ret = model.eval(51544.5);
        assert!((ret[0] - 1.0).abs() < 1e-15, "L = {}", ret[0]);
        assert!(ret[1].abs() < 1e-15, "B = {}", ret[1]);
        assert!((ret[2] - 1.0).abs() < 1e-15, "R = {}", ret[2]);
        assert!(ret[3].abs() < 1e-15, "dL/dt = {}", ret[3]);
    }

    #[test]
    fn eval_velocity_nonzero() {
        // At a time where the argument b + c*t1 = PI/2, cos(arg)=0 and sin(arg)=1.
        // With a=1, b=0, c=1, we need t1 = PI/2, so mjd = 51544.5 + PI/2 * 365250.
        let data = make_simple_data();
        let model = VsopModel::from_reader(data.as_bytes(), Series::B, 1.0, 1e-30, 10.0).unwrap();

        let mjd = 51544.5 + std::f64::consts::FRAC_PI_2 * A1000;
        let ret = model.eval(mjd);
        // L = 1.0*cos(PI/2) ≈ 0
        assert!(ret[0].abs() < 1e-15, "L = {}", ret[0]);
        // dL/dt = -c*a*sin(arg) * rsec = -1.0*1.0*sin(PI/2) * rsec = -rsec
        let expected_vel = -RSEC;
        assert!(
            (ret[3] - expected_vel).abs() < 1e-25,
            "dL/dt = {} vs {}",
            ret[3],
            expected_vel
        );
    }
}
