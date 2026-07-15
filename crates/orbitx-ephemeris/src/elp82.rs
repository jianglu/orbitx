//! ELP2000-82 lunar ephemeris: parser and evaluator.
//!
//! Mirrors Orbiter's `ELP82.cpp` (Src/Celbody/Moon/ELP82.cpp). Only the "main
//! problem" terms are supported (tidal/planetary perturbations —
//! `INCLUDE_TIDAL_PERT` — are disabled by default in Orbiter as well).
//!
//! The implementation is a symbol-for-symbol replica of the C++ code: the
//! fundamental-argument constants (`ELP82_init`), the data-file parser
//! (`ELP82_read`), and the evaluation function (`ELP82`).

use std::io::{BufRead, BufReader};

// ===========================================================
// Constants (ELP82.cpp:16-37)
// ===========================================================

const CPI: f64 = 3.141592653589793;
#[allow(dead_code)]
const CPI2: f64 = 2.0 * CPI;
const PIS2: f64 = CPI / 2.0;
const RAD_CONST: f64 = 648_000.0 / CPI; // arcseconds per radian
const DEG: f64 = CPI / 180.0;
const C1: f64 = 60.0;
const C2: f64 = 3600.0;
const ATH: f64 = 384_747.980_674_316_5;
const A0: f64 = 384_747.980_644_895_4;
const AM: f64 = 0.074_801_329_518;
const ALPHA: f64 = 0.002_571_881_335;
const DTASM: f64 = 2.0 * ALPHA / (3.0 * AM);
const MJD2000: f64 = 51_544.5;
const SC: f64 = 36_525.0; // days per Julian century
const PRECESS: f64 = 5029.0966 / RAD_CONST;

// ===========================================================
// Fundamental arguments (ELP82_init, ELP82.cpp:49-127)
// ===========================================================

/// All the constant fundamental arguments and correction values, computed
/// once in `ELP82_init`. Mirrors the C++ file-scope static variables.
#[derive(Clone, Debug)]
pub struct ElpConstants {
    // Lunar arguments: w[3][5] — mean longitude, arg of perigee, long asc node
    pub w: [[f64; 5]; 3],
    // Planetary arguments: p[8][2]
    pub p: [[f64; 2]; 8],
    // Earth's mean longitude: eart[5]
    pub eart: [f64; 5],
    // Earth's perihelion: peri[5]
    pub peri: [f64; 5],
    // Delaunay's arguments: del[4][5]
    pub del: [[f64; 5]; 4],
    // zeta[2]
    pub zeta: [f64; 2],
    // Corrections (fit to DE200/LE200)
    pub delnu: f64,
    pub dele: f64,
    pub delg: f64,
    pub delnp: f64,
    pub delep: f64,
    // Precession coefficients
    pub p1: f64,
    pub p2: f64,
    pub p3: f64,
    pub p4: f64,
    pub p5: f64,
    pub q1: f64,
    pub q2: f64,
    pub q3: f64,
    pub q4: f64,
    pub q5: f64,
}

impl Default for ElpConstants {
    fn default() -> Self {
        Self::new()
    }
}

impl ElpConstants {
    /// Initialise all fundamental arguments (mirrors `ELP82_init`).
    pub fn new() -> Self {
        let mut w = [[0.0_f64; 5]; 3];
        let mut p = [[0.0_f64; 2]; 8];
        let mut eart = [0.0_f64; 5];
        let mut peri = [0.0_f64; 5];
        let mut del = [[0.0_f64; 5]; 4];
        let mut zeta = [0.0_f64; 2];

        // Lunar arguments
        w[0][0] = (218.0 + 18.0 / C1 + 59.95571 / C2) * DEG;
        w[1][0] = (83.0 + 21.0 / C1 + 11.67475 / C2) * DEG;
        w[2][0] = (125.0 + 2.0 / C1 + 40.39816 / C2) * DEG;
        eart[0] = (100.0 + 27.0 / C1 + 59.22059 / C2) * DEG;
        peri[0] = (102.0 + 56.0 / C1 + 14.42753 / C2) * DEG;
        w[0][1] = 1_732_559_343.736_04 / RAD_CONST;
        w[1][1] = 14_643_420.263_2 / RAD_CONST;
        w[2][1] = -6_967_919.362_2 / RAD_CONST;
        eart[1] = 129_597_742.275_8 / RAD_CONST;
        peri[1] = 1161.2283 / RAD_CONST;
        w[0][2] = -5.8883 / RAD_CONST;
        w[1][2] = -38.2776 / RAD_CONST;
        w[2][2] = 6.3622 / RAD_CONST;
        eart[2] = -0.0202 / RAD_CONST;
        peri[2] = 0.5327 / RAD_CONST;
        w[0][3] = 0.6604e-2 / RAD_CONST;
        w[1][3] = -0.45047e-1 / RAD_CONST;
        w[2][3] = 0.7625e-2 / RAD_CONST;
        eart[3] = 0.9e-5 / RAD_CONST;
        peri[3] = -0.138e-3 / RAD_CONST;
        w[0][4] = -0.3169e-4 / RAD_CONST;
        w[1][4] = 0.21301e-3 / RAD_CONST;
        w[2][4] = -0.3586e-4 / RAD_CONST;
        eart[4] = 0.15e-6 / RAD_CONST;
        peri[4] = 0.0;

        // Planetary arguments
        p[0][0] = (252.0 + 15.0 / C1 + 3.25986 / C2) * DEG;
        p[1][0] = (181.0 + 58.0 / C1 + 47.28305 / C2) * DEG;
        p[2][0] = eart[0];
        p[3][0] = (355.0 + 25.0 / C1 + 59.78866 / C2) * DEG;
        p[4][0] = (34.0 + 21.0 / C1 + 5.34212 / C2) * DEG;
        p[5][0] = (50.0 + 4.0 / C1 + 38.89694 / C2) * DEG;
        p[6][0] = (314.0 + 3.0 / C1 + 18.01841 / C2) * DEG;
        p[7][0] = (304.0 + 20.0 / C1 + 55.19575 / C2) * DEG;
        p[0][1] = 538_101_628.688_98 / RAD_CONST;
        p[1][1] = 210_664_136.433_55 / RAD_CONST;
        p[2][1] = eart[1];
        p[3][1] = 68_905_077.592_84 / RAD_CONST;
        p[4][1] = 10_925_660.428_61 / RAD_CONST;
        p[5][1] = 4_399_609.659_32 / RAD_CONST;
        p[6][1] = 1_542_481.193_93 / RAD_CONST;
        p[7][1] = 786_550.320_74 / RAD_CONST;

        // Corrections of the constants (fit to DE200/LE200)
        let delnu = 0.55604 / RAD_CONST / w[0][1];
        let dele = 0.01789 / RAD_CONST;
        let delg = -0.08066 / RAD_CONST;
        let delnp = -0.06424 / RAD_CONST / w[0][1];
        let delep = -0.12879 / RAD_CONST;

        // Delaunay's arguments
        for i in 0..5 {
            del[0][i] = w[0][i] - eart[i];
            del[3][i] = w[0][i] - w[2][i];
            del[2][i] = w[0][i] - w[1][i];
            del[1][i] = eart[i] - peri[i];
        }
        del[0][0] += CPI;
        zeta[0] = w[0][0];
        zeta[1] = w[0][1] + PRECESS;

        // Precession coefficients
        let p1 = 0.101_803_91e-4;
        let p2 = 0.470_204_39e-6;
        let p3 = -0.541_736_7e-9;
        let p4 = -0.250_794_8e-11;
        let p5 = 0.463_486e-14;
        let q1 = -0.113_469_002e-3;
        let q2 = 0.123_726_74e-6;
        let q3 = 0.126_541_7e-8;
        let q4 = -0.137_180_8e-11;
        let q5 = -0.320_334e-14;

        ElpConstants {
            w,
            p,
            eart,
            peri,
            del,
            zeta,
            delnu,
            dele,
            delg,
            delnp,
            delep,
            p1,
            p2,
            p3,
            p4,
            p5,
            q1,
            q2,
            q3,
            q4,
            q5,
        }
    }
}

// ===========================================================
// Term structure
// ===========================================================

/// A single ELP82 main-problem term (the `SEQ6` in C++).
///
/// - `coef[0]`: amplitude (arcseconds for L,B; km for R, after corrections)
/// - `coef[1]`: phase base
/// - `coef[2..5]`: phase time-derivative components for `t^1..t^4`
#[derive(Clone, Copy, Debug, Default)]
pub struct ElpTerm {
    pub coef: [f64; 6],
}

/// Complete ELP2000-82 model (main problem only).
pub struct ElpModel {
    /// Fundamental-argument constants.
    pub consts: ElpConstants,
    /// Three main sequences: `[L, B, R]`, each a vector of terms.
    pub series: [Vec<ElpTerm>; 3],
}

impl ElpModel {
    /// Read ELP82 data from a reader with the given precision.
    ///
    /// Mirrors `ELP82_read` (ELP82.cpp:158, main problem only).
    pub fn from_reader<R: BufRead>(reader: R, prec: f64) -> std::io::Result<Self> {
        let consts = ElpConstants::new();
        let mut tokens = TokenStream::new(reader);

        // Precision parameters (ELP82.cpp:189-192).
        let pre = [prec * RAD_CONST, prec * RAD_CONST, prec * ATH];

        let mut series: [Vec<ElpTerm>; 3] = [Vec::new(), Vec::new(), Vec::new()];

        for ific in 0..3 {
            let m = tokens.next_int()? as usize;
            // Read all terms first, filtering by precision.
            // C++ reads into block[], counts mm (terms used), then assembles pc.
            let mut block: Vec<([i64; 4], [f64; 7])> = Vec::with_capacity(m);
            let mut used_terms: Vec<([i64; 4], [f64; 7])> = Vec::new();

            for _ in 0..m {
                let ilu = [
                    tokens.next_int()?,
                    tokens.next_int()?,
                    tokens.next_int()?,
                    tokens.next_int()?,
                ];
                let coef = [
                    tokens.next_f64()?,
                    tokens.next_f64()?,
                    tokens.next_f64()?,
                    tokens.next_f64()?,
                    tokens.next_f64()?,
                    tokens.next_f64()?,
                    tokens.next_f64()?,
                ];
                block.push((ilu, coef));
                if coef[0].abs() >= pre[ific] {
                    used_terms.push((ilu, coef));
                }
            }

            let mm = used_terms.len();
            series[ific].reserve(mm);

            for (ilu, coef) in &used_terms {
                // C++ reads `xx = lin.coef[0]` but mutates `lin.coef[0]` for ific==2.
                let mut coef0 = coef[0];
                let tgv = coef[1] + DTASM * coef[5];
                if ific == 2 {
                    coef0 -= 2.0 * coef0 * consts.delnu / 3.0;
                }
                let xx = coef0
                    + tgv * (consts.delnp - AM * consts.delnu)
                    + coef[2] * consts.delg
                    + coef[3] * consts.dele
                    + coef[4] * consts.delep;

                let mut zone = [0.0_f64; 6];
                zone[0] = xx;
                for k in 0..=4 {
                    let y: f64 = ilu
                        .iter()
                        .zip(consts.del.iter().map(|d| d[k]))
                        .map(|(&i, d)| i as f64 * d)
                        .sum();
                    zone[k + 1] = y;
                }
                if ific == 2 {
                    zone[1] += PIS2;
                }
                series[ific].push(ElpTerm { coef: zone });
            }
        }

        Ok(ElpModel { consts, series })
    }

    /// Evaluate the ELP82 ephemeris at MJD `mjd`, writing position+velocity to
    /// `ret[0..5]`.
    ///
    /// Mirrors `ELP82()` (ELP82.cpp:309).
    pub fn eval_into(&self, mjd: f64, ret: &mut [f64; 6]) {
        let c = &self.consts;

        // Time powers (Julian centuries).
        let mut t = [0.0_f64; 5];
        t[0] = 1.0;
        t[1] = (mjd - MJD2000) / SC;
        t[2] = t[1] * t[1];
        t[3] = t[2] * t[1];
        t[4] = t[3] * t[1];

        for iv in 0..3 {
            ret[iv] = 0.0;
            ret[iv + 3] = 0.0;

            // Main sequence summation.
            for term in &self.series[iv] {
                let x = term.coef[0];
                let x_dot = 0.0;
                let mut y = term.coef[1];
                let mut y_dot = 0.0;
                for k in 1..=4 {
                    y += term.coef[k + 1] * t[k];
                    y_dot += term.coef[k + 1] * t[k - 1] * k as f64;
                }
                ret[iv] += x * y.sin();
                ret[iv + 3] += x_dot * y.sin() + x * y.cos() * y_dot;
            }
        }

        // Change of coordinates.

        ret[0] = ret[0] / RAD_CONST
            + c.w[0][0]
            + c.w[0][1] * t[1]
            + c.w[0][2] * t[2]
            + c.w[0][3] * t[3]
            + c.w[0][4] * t[4];
        ret[3] = ret[3] / RAD_CONST
            + c.w[0][1]
            + 2.0 * c.w[0][2] * t[1]
            + 3.0 * c.w[0][3] * t[2]
            + 4.0 * c.w[0][4] * t[3];
        ret[1] /= RAD_CONST;
        ret[4] /= RAD_CONST;
        ret[2] *= A0 / ATH;
        ret[5] *= A0 / ATH;

        let cosr0 = ret[0].cos();
        let sinr0 = ret[0].sin();
        let cosr1 = ret[1].cos();
        let sinr1 = ret[1].sin();
        let mut x1 = ret[2] * cosr1;
        let mut x1_dot = ret[5] * cosr1 - ret[2] * sinr1 * ret[4];
        let x2 = x1 * sinr0;
        let x2_dot = x1_dot * sinr0 + x1 * cosr0 * ret[3];
        x1_dot = x1_dot * cosr0 - x1 * sinr0 * ret[3];
        x1 *= cosr0;
        let x3 = ret[2] * sinr1;
        let x3_dot = ret[5] * sinr1 + ret[2] * cosr1 * ret[4];

        let pw = (c.p1 + c.p2 * t[1] + c.p3 * t[2] + c.p4 * t[3] + c.p5 * t[4]) * t[1];
        let pw_dot =
            c.p1 + 2.0 * c.p2 * t[1] + 3.0 * c.p3 * t[2] + 4.0 * c.p4 * t[3] + 5.0 * c.p5 * t[4];
        let qw = (c.q1 + c.q2 * t[1] + c.q3 * t[2] + c.q4 * t[3] + c.q5 * t[4]) * t[1];
        let qw_dot =
            c.q1 + 2.0 * c.q2 * t[1] + 3.0 * c.q3 * t[2] + 4.0 * c.q4 * t[3] + 5.0 * c.q5 * t[4];
        let ra = 2.0 * (1.0 - pw * pw - qw * qw).sqrt();
        let ra_dot = -4.0 * (pw + qw) / ra;
        let pwqw = 2.0 * pw * qw;
        let pwqw_dot = 2.0 * (pw_dot * qw + pw * qw_dot);
        let pw2 = 1.0 - 2.0 * pw * pw;
        let pw2_dot = -4.0 * pw;
        let qw2 = 1.0 - 2.0 * qw * qw;
        let qw2_dot = -4.0 * qw;
        // C++ rescales pw/qw in-place: pw = pw*ra; pw_dot = pw_dot*ra + pw*ra_dot
        let pw_s = pw * ra;
        let pw_dot_s = pw_dot * ra + pw * ra_dot;
        let qw_s = qw * ra;
        let qw_dot_s = qw_dot * ra + qw * ra_dot;

        // y and z components are swapped to conform with Orbiter convention.
        // (r[1] <-> r[2] and r[4] <-> r[5])
        ret[0] = pw2 * x1 + pwqw * x2 + pw_s * x3;
        ret[3] = pw2_dot * x1
            + pw2 * x1_dot
            + pwqw_dot * x2
            + pwqw * x2_dot
            + pw_dot_s * x3
            + pw_s * x3_dot;
        ret[2] = pwqw * x1 + qw2 * x2 - qw_s * x3;
        ret[5] = pwqw_dot * x1 + pwqw * x1_dot + qw2_dot * x2 + qw2 * x2_dot
            - qw_dot_s * x3
            - qw_s * x3_dot;
        ret[1] = -pw_s * x1 + qw_s * x2 + (pw2 + qw2 - 1.0) * x3;
        ret[4] = -pw_dot_s * x1 - pw_s * x1_dot
            + qw_dot_s * x2
            + qw_s * x2_dot
            + (pw2_dot + qw2_dot) * x3
            + (pw2 + qw2 - 1.0) * x3_dot;

        // Convert to meters and m/s.
        let pscale = 1e3;
        let vscale = 1e3 / (86400.0 * SC);
        for i in 0..3 {
            ret[i] *= pscale;
            ret[i + 3] *= vscale;
        }
    }

    /// Evaluate at MJD and return a fresh `[f64; 6]`.
    pub fn eval(&self, mjd: f64) -> [f64; 6] {
        let mut ret = [0.0; 6];
        self.eval_into(mjd, &mut ret);
        ret
    }
}

// ===========================================================
// Whitespace-token stream parser (shared pattern)
// ===========================================================

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
            let trimmed = self.current_line[self.pos..].trim_start();
            let start = self.current_line.len() - trimmed.len();
            if let Some(end) = trimmed.find(char::is_whitespace) {
                self.pos = start + end;
                return Ok(&self.current_line[start..start + end]);
            } else if !trimmed.is_empty() {
                self.pos = self.current_line.len();
                return Ok(&self.current_line[start..]);
            }
            match self.lines.next() {
                Some(Ok(line)) => {
                    self.current_line = line;
                    self.pos = 0;
                }
                Some(Err(e)) => return Err(e),
                None => {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "unexpected end of ELP82 data",
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

    #[test]
    fn constants_initialized() {
        let c = ElpConstants::new();
        // Mean longitude of the Moon at J2000 should be ~218.46° in radians.
        let expected = (218.0 + 18.0 / 60.0 + 59.95571 / 3600.0) * DEG;
        assert!((c.w[0][0] - expected).abs() < 1e-15);
        // Delaunay arg D = w[0]-eart+pi
        assert!((c.del[0][0] - (c.w[0][0] - c.eart[0] + CPI)).abs() < 1e-15);
    }

    #[test]
    fn parse_minimal_elp() {
        // 3 groups: L (1 term), B (0 terms), R (0 terms).
        // Term: ilu=0 0 1 0, coef = 22639.55 0 0 412529.61 0 0 0
        // (the dominant l' term)
        let data = "1\n0 0 1 0 22639.55 0 0 412529.61 0 0 0\n0\n0\n";
        let model = ElpModel::from_reader(data.as_bytes(), 1e-10).unwrap();
        // L should have 1 term, B and R should have 0.
        assert_eq!(model.series[0].len(), 1);
        assert_eq!(model.series[1].len(), 0);
        assert_eq!(model.series[2].len(), 0);
    }
}
