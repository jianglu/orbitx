//! Kepler orbit solver: classical orbital elements, Kepler equation solver,
//! and 2-body analytic propagation.
//!
//! Mirrors Orbiter's `Elements` class (`Src/Orbiter/Element.h` / `Element.cpp`).
//! All formulas are symbol-for-symbol replicas, including the left-handed
//! coordinate system convention (`H = V × R`).
//!
//! This is pure analytic two-body math (closed-form coordinate conversions
//! and Kepler's-equation root-finding) — no force evaluation, no numerical
//! integration — so it lives in `orbitx-math`, not `orbitx-dynamics`. The
//! numerical propagators / Encke correction (which consume forces) stay in
//! `orbitx-dynamics`.

use crate::consts::acosh;
use crate::consts::{pos_angle, PI, PI2};
use crate::vec3::{cross, dot, Vec3};

const E_CIRCLE_LIMIT: f64 = 1e-8;
const I_NOINC_LIMIT: f64 = 1e-8;

/// Classical orbital elements and derived quantities.
///
/// Mirrors `class Elements` (Element.h). The primary elements (`a`, `e`, `i`,
/// `theta`, `omegab`, `L`) are public; derived quantities are accessible via
/// methods.
pub struct Elements {
    // --- Primary elements ---
    /// Semi-major axis [m].
    pub a: f64,
    /// Numerical eccentricity.
    pub e: f64,
    /// Inclination [rad].
    pub i: f64,
    /// Longitude of ascending node [rad].
    pub theta: f64,
    /// Longitude of periapsis [rad].
    pub omegab: f64,
    /// Mean longitude at epoch.
    pub l: f64,

    // --- Derived shape parameters ---
    mu: f64,     // G*(M+m)
    muh: f64,    // sqrt(mu/p)
    omega: f64,  // argument of periapsis
    p: f64,      // semi-latus rectum
    n: f64,      // mean motion 2pi/T
    period: f64, // orbit period [s]
    #[allow(dead_code)]
    le: f64, // linear eccentricity
    pd: f64,     // periapsis distance
    ad: f64,     // apoapsis distance
    smi: f64,    // semi-minor axis
    tpe: f64,    // time to next periapsis [s]
    tap: f64,    // time to next apoapsis [s]
    tau: f64,    // periapsis passage time [s]

    // --- Cached trig values ---
    pub sint: f64,
    pub cost: f64,
    pub sini: f64,
    pub cosi: f64,
    pub sino: f64,
    pub coso: f64,

    // --- Time-varying (last Calculate) ---
    ma: f64,  // mean anomaly
    tra: f64, // true anomaly
    ea: f64,  // eccentric anomaly

    // --- Newton iteration seed cache ---
    ea0: f64,
    ma0: f64,

    #[allow(dead_code)]
    mjd_epoch: f64,
}

impl Elements {
    /// Calculate orbital elements from position and velocity state vectors.
    ///
    /// Mirrors `Elements::Calculate` (Element.cpp:376+).
    ///
    /// **Left-handed**: uses `H = V × R` (not the standard `R × V`).
    pub fn calculate(r: Vec3, v: Vec3, mu: f64, simt: f64) -> Self {
        let v2 = v.length2();
        let rmag = r.length();
        let h_vec = cross(v, r); // left-handed coordinates!
        let h = h_vec.length();
        let rv = dot(r, v);

        // Semi-major axis (vis-viva).
        let a = rmag * mu / (2.0 * mu - rmag * v2);

        // Eccentricity vector (Laplace-Runge-Lenz).
        let e_vec = r * (1.0 / rmag - 1.0 / a) - v * (rv / mu);
        let e = e_vec.length();
        let closed_orbit = e < 1.0;

        // Inclination.
        let inc = (h_vec.y / h).acos();
        let sini = inc.sin();
        let cosi = inc.cos();

        // Derived shape parameters.
        let le = a * e;
        let pd = a - le;
        let p = (a * (1.0 - e * e)).max(0.0);
        let muh = (mu / p).sqrt();

        let (ad, smi, n, period) = if closed_orbit {
            let ad = a + le;
            let smi = a * (1.0 - e * e).sqrt();
            let n = (mu / (a * a * a)).sqrt();
            let period = PI2 / n;
            (ad, smi, n, period)
        } else {
            let smi = a * (e * e - 1.0).sqrt();
            let n = (mu / (-a * a * a)).sqrt();
            (0.0, smi, n, 0.0)
        };

        // Longitude of ascending node.
        let (theta, n_vec) = if inc > I_NOINC_LIMIT {
            let tmp = 1.0 / (h_vec.z * h_vec.z + h_vec.x * h_vec.x).sqrt();
            let n_vec = Vec3::new(-h_vec.z * tmp, 0.0, h_vec.x * tmp);
            let mut theta = n_vec.x.acos();
            if n_vec.z < 0.0 {
                theta = PI2 - theta;
            }
            (theta, n_vec)
        } else {
            (0.0, Vec3::new(1.0, 0.0, 0.0))
        };
        let sint = theta.sin();
        let cost = theta.cos();

        // Argument of periapsis.
        let omega = if e > E_CIRCLE_LIMIT {
            if inc > I_NOINC_LIMIT {
                let arg = dot(n_vec, e_vec) / e;
                let mut om = if arg < -1.0 {
                    PI
                } else if arg > 1.0 {
                    0.0
                } else {
                    arg.acos()
                };
                if e_vec.y < 0.0 {
                    om = PI2 - om;
                }
                om
            } else {
                let mut om = e_vec.z.atan2(e_vec.x);
                if om < 0.0 {
                    om += PI2;
                }
                om
            }
        } else {
            0.0
        };
        let sino = omega.sin();
        let coso = omega.cos();

        // Longitude of periapsis.
        let mut omegab = theta + omega;
        if omegab >= PI2 {
            omegab -= PI2;
        }

        // True anomaly.
        let tra = if e > E_CIRCLE_LIMIT {
            let mut ta = (dot(e_vec, r) / (e * rmag)).acos();
            if rv < 0.0 {
                ta = PI2 - ta;
            }
            ta
        } else if inc > I_NOINC_LIMIT {
            let mut ta = (dot(n_vec, r) / rmag).acos();
            if dot(n_vec, v) > 0.0 {
                ta = PI2 - ta;
            }
            ta
        } else {
            let mut ta = (r.x / rmag).acos();
            if v.x > 0.0 {
                ta = PI2 - ta;
            }
            ta
        };

        // Eccentric and mean anomaly.
        let (ea, ma) = if closed_orbit {
            let ea = if e > E_CIRCLE_LIMIT {
                (rv * (a / mu).sqrt()).atan2(a - rmag)
            } else {
                tra
            };
            let ma = ea - e * ea.sin();
            (ea, ma)
        } else {
            let costra = tra.cos();
            let mut ea = acosh((e + costra) / (1.0 + e * costra));
            if tra >= PI {
                ea = -ea;
            }
            let ma = e * ea.sinh() - ea;
            (ea, ma)
        };

        // Time to periapsis/apoapsis.
        let (tpe, tap) = if closed_orbit {
            let mut tpe = -ma / n;
            if tpe < 0.0 {
                tpe += period;
            }
            let mut tap = tpe - 0.5 * period;
            if tap < 0.0 {
                tap += period;
            }
            (tpe, tap)
        } else {
            let tpe = -ma / n;
            (tpe, 0.0)
        };

        // Periapsis passage time.
        let mut tau = simt + tpe;
        if closed_orbit {
            tau -= period;
        }

        // Mean longitude at epoch.
        let l = if closed_orbit {
            pos_angle(omegab + n * (-tau))
        } else {
            omegab + n * (-tau)
        };

        Elements {
            a,
            e,
            i: inc,
            theta,
            omegab,
            l,
            mu,
            muh,
            omega,
            p,
            n,
            period,
            le,
            pd,
            ad,
            smi,
            tpe,
            tap,
            tau,
            sint,
            cost,
            sini,
            cosi,
            sino,
            coso,
            ma,
            tra,
            ea,
            ea0: ea,
            ma0: ma,
            mjd_epoch: 0.0,
        }
    }

    /// Solve Kepler's equation for eccentric anomaly from mean anomaly.
    ///
    /// Mirrors `Elements::EccAnomaly` (Element.cpp:195). Newton-Raphson
    /// iteration with step clamping to [-1, 1]. Uses cached seed for nearby
    /// mean anomalies.
    pub fn ecc_anomaly(&self, ma: f64) -> f64 {
        const NITER: usize = 16;
        const TOL: f64 = 1e-14;

        let mut ea = if (ma - self.ma0).abs() < 1e-2 {
            self.ea0
        } else {
            ma
        };

        if self.e < 1.0 {
            // Closed orbit: M = E - e*sin(E)
            let mut res = ma - ea + self.e * ea.sin();
            if res.abs() > ma.abs() {
                ea = 0.0;
                res = ma;
            }
            let mut i = 0;
            while res.abs() > TOL && i < NITER {
                ea += step_clamp(res / (1.0 - self.e * ea.cos()));
                res = ma - ea + self.e * ea.sin();
                i += 1;
            }
        } else {
            // Open orbit: M = e*sinh(E) - E
            let mut res = ma - self.e * ea.sinh() + ea;
            if res.abs() > ma.abs() {
                ea = 0.0;
                res = ma;
            }
            let mut i = 0;
            while res.abs() > TOL && i < NITER {
                ea += step_clamp(res / (self.e * ea.cosh() - 1.0));
                res = ma - self.e * ea.sinh() + ea;
                i += 1;
            }
        }

        ea
    }

    /// Calculate position and velocity relative to the reference body at time `t`.
    ///
    /// Mirrors `Elements::PosVel` (Element.cpp:281).
    pub fn pos_vel(&self, t: f64) -> (Vec3, Vec3) {
        let (r, ta) = self.rel_pos(t);
        let pos = self.pol2crt(r, ta);

        // Velocity in the orbital plane.
        let vx = -self.muh * ta.sin();
        let vz = self.muh * (self.e + ta.cos());
        let thetav = vz.atan2(vx);
        let rv = (vx * vx + vz * vz).sqrt();
        let sinto = (thetav + self.omega).sin();
        let costo = (thetav + self.omega).cos();
        let vel = Vec3::new(
            rv * (self.cost * costo - self.sint * sinto * self.cosi),
            rv * sinto * self.sini,
            rv * (self.sint * costo + self.cost * sinto * self.cosi),
        );

        (pos, vel)
    }

    /// Relative position (radius, true anomaly) at time `t`.
    ///
    /// Mirrors `Elements::RelPos` (Element.cpp).
    fn rel_pos(&self, t: f64) -> (f64, f64) {
        let ma = self.mean_anomaly(t);
        let ea = self.ecc_anomaly(ma);

        // True anomaly from eccentric anomaly.
        let ta = if self.e < 1.0 {
            // Closed orbit: ta = 2*atan2(sqrt((1+e)/(1-e)) * sin(E/2), cos(E/2))
            2.0 * (((1.0 + self.e) / (1.0 - self.e)).sqrt() * (ea / 2.0).sin())
                .atan2((ea / 2.0).cos())
        } else {
            // Open orbit: ta = 2*atan2(sqrt((e+1)/(e-1)) * sinh(E/2), cosh(E/2))
            2.0 * (((self.e + 1.0) / (self.e - 1.0)).sqrt() * (ea / 2.0).sinh())
                .atan2((ea / 2.0).cosh())
        };

        let r = self.p / (1.0 + self.e * ta.cos());
        (r, ta)
    }

    /// Convert polar orbital position to cartesian.
    ///
    /// Mirrors `Elements::Pol2Crt` (Element.cpp).
    fn pol2crt(&self, r: f64, ta: f64) -> Vec3 {
        let cosargo_plus_ta = (self.omega + ta).cos();
        let sinargo_plus_ta = (self.omega + ta).sin();

        Vec3::new(
            r * (self.cost * cosargo_plus_ta - self.sint * sinargo_plus_ta * self.cosi),
            r * sinargo_plus_ta * self.sini,
            r * (self.sint * cosargo_plus_ta + self.cost * sinargo_plus_ta * self.cosi),
        )
    }

    /// Mean anomaly at time `t`.
    pub fn mean_anomaly(&self, t: f64) -> f64 {
        self.n * (t - self.tau)
    }

    /// 纬度幅角 u = ω + ν [rad]，归一化 [0, 2π)（C3b）。
    ///
    /// 轨道面内自升交点沿运动方向量到当前位置的角度。圆轨（e ≤
    /// `E_CIRCLE_LIMIT`）时 ω 无定义、引擎置 0（Element.cpp 同源口径），
    /// u = ν 自升交点起算，与椭圆口径连续（见 tests）；e ≥ 1（逃逸）时
    /// ν 仍有定义、u 可计算——消费方按 escaping/suborbital 自行取舍。
    /// 任务二圈判定消费：切片 `arg_latitude_deg` 逐帧最小角差累积。
    pub fn arg_latitude(&self) -> f64 {
        let u = self.omega + self.tra;
        if u >= PI2 {
            u - PI2
        } else {
            u
        }
    }

    // --- Accessors for derived elements ---
    pub fn arg_per(&self) -> f64 {
        self.omega
    }
    pub fn smi(&self) -> f64 {
        self.smi
    }
    pub fn ap_dist(&self) -> f64 {
        self.ad
    }
    pub fn pe_dist(&self) -> f64 {
        self.pd
    }
    pub fn orbit_t(&self) -> f64 {
        self.period
    }
    pub fn pe_t(&self) -> f64 {
        self.tpe
    }
    pub fn ap_t(&self) -> f64 {
        self.tap
    }
    pub fn p(&self) -> f64 {
        self.p
    }
    pub fn mu(&self) -> f64 {
        self.mu
    }
    pub fn mean_anm(&self) -> f64 {
        self.ma
    }
    pub fn true_anm(&self) -> f64 {
        self.tra
    }
    pub fn ecc_anm(&self) -> f64 {
        self.ea
    }
}

/// Clamp Newton step to [-1, 1].
fn step_clamp(x: f64) -> f64 {
    x.clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests;
