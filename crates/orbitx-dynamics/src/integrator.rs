//! Numerical integrators: Runge-Kutta (RK2-RK8) and Yoshida symplectic
//! (SY2/SY4/SY6/SY8).
//!
//! Mirrors Orbiter's `BodyIntegrator.cpp`. The integrators operate on a
//! `StateVectors` bundle (position, velocity, quaternion, angular velocity),
//! using a caller-provided force function to evaluate accelerations.
//!
//! **Design**: Orbiter's integrators are member methods of `RigidBody` that
//! directly mutate `rpos_add`/`rvel_add` and `s1->Q`/`s1->omega`. orbitx
//! extracts them as pure functions that take a `StateVectors` and a force
//! closure, returning a new `StateVectors`. This decouples the integrators
//! from the engine.
//!
//! **Angular dynamics**: The C++ integrators call `EulerInv_full(tau, omega)`
//! to convert torque to angular acceleration. orbitx's `ForceFn` directly
//! returns angular acceleration, so the Euler equation is the caller's
//! responsibility.

use orbitx_math::{StateVectors, Vec3};

/// Force function: given a state and a fractional time `(isub + c_i)/nsub`,
/// return `(linear_acceleration, angular_acceleration)`.
pub type ForceFn = dyn FnMut(&StateVectors, f64) -> (Vec3, Vec3);

/// Butcher tableau coefficients for a Runge-Kutta method.
pub struct RkCoeffs {
    /// Number of stages.
    pub n: usize,
    /// Node positions c_i (length n-1, c_0=0 implied).
    pub alpha: &'static [f64],
    /// Stage coupling matrix a_ij, row-major lower-triangular (length (n-1)*(n-1)).
    /// Row i (0-indexed) has i+1 entries for stages 1..n-1.
    pub beta: &'static [f64],
    /// Weight coefficients b_i (length n).
    pub gamma: &'static [f64],
}

// ===========================================================
// Butcher tableaux (BodyIntegrator.cpp:59-147)
// ===========================================================

/// RK5 — 6 stages (Dormand-Prince).
pub const RK5: RkCoeffs = RkCoeffs {
    n: 6,
    alpha: &[1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0],
    beta: &[
        1.0 / 5.0,
        0.0,
        0.0,
        0.0,
        0.0,
        3.0 / 40.0,
        9.0 / 40.0,
        0.0,
        0.0,
        0.0,
        44.0 / 45.0,
        -56.0 / 15.0,
        32.0 / 9.0,
        0.0,
        0.0,
        19_372.0 / 6561.0,
        -25_360.0 / 2187.0,
        64_448.0 / 6561.0,
        -212.0 / 729.0,
        0.0,
        9017.0 / 3168.0,
        -355.0 / 33.0,
        46_732.0 / 5247.0,
        49.0 / 176.0,
        -5103.0 / 18_656.0,
    ],
    gamma: &[
        35.0 / 384.0,
        0.0,
        500.0 / 1113.0,
        125.0 / 192.0,
        -2187.0 / 6784.0,
        11.0 / 84.0,
    ],
};

/// RK6 — 8 stages (Lawson).
pub const RK6: RkCoeffs = RkCoeffs {
    n: 8,
    alpha: &[
        1.0 / 6.0,
        4.0 / 15.0,
        2.0 / 3.0,
        5.0 / 6.0,
        1.0,
        1.0 / 15.0,
        1.0,
    ],
    beta: &[
        1.0 / 6.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        4.0 / 75.0,
        16.0 / 75.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        5.0 / 6.0,
        -8.0 / 3.0,
        5.0 / 2.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -165.0 / 64.0,
        55.0 / 6.0,
        -425.0 / 64.0,
        85.0 / 96.0,
        0.0,
        0.0,
        0.0,
        12.0 / 5.0,
        -8.0,
        4015.0 / 612.0,
        -11.0 / 36.0,
        88.0 / 255.0,
        0.0,
        0.0,
        -8263.0 / 15_000.0,
        124.0 / 75.0,
        -643.0 / 680.0,
        -81.0 / 250.0,
        2484.0 / 10_625.0,
        0.0,
        0.0,
        3501.0 / 1720.0,
        -300.0 / 43.0,
        297_275.0 / 52_632.0,
        -319.0 / 2322.0,
        24_068.0 / 84_065.0,
        0.0,
        3850.0 / 26_703.0,
    ],
    gamma: &[
        3.0 / 40.0,
        0.0,
        875.0 / 2244.0,
        23.0 / 72.0,
        264.0 / 1955.0,
        0.0,
        125.0 / 11_592.0,
        43.0 / 616.0,
    ],
};

/// RK7 — 11 stages (Prince-Dormand 7(5)).
pub const RK7: RkCoeffs = RkCoeffs {
    n: 11,
    alpha: &[
        2.0 / 27.0,
        1.0 / 9.0,
        1.0 / 6.0,
        5.0 / 12.0,
        0.5,
        5.0 / 6.0,
        1.0 / 6.0,
        2.0 / 3.0,
        1.0 / 3.0,
        1.0,
    ],
    beta: &[
        2.0 / 27.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0 / 36.0,
        1.0 / 12.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0 / 24.0,
        0.0,
        1.0 / 8.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        5.0 / 12.0,
        0.0,
        -25.0 / 16.0,
        25.0 / 16.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0 / 20.0,
        0.0,
        0.0,
        1.0 / 4.0,
        1.0 / 5.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -25.0 / 108.0,
        0.0,
        0.0,
        125.0 / 108.0,
        -65.0 / 27.0,
        125.0 / 54.0,
        0.0,
        0.0,
        0.0,
        0.0,
        31.0 / 300.0,
        0.0,
        0.0,
        0.0,
        61.0 / 225.0,
        -2.0 / 9.0,
        13.0 / 900.0,
        0.0,
        0.0,
        0.0,
        2.0,
        0.0,
        0.0,
        -53.0 / 6.0,
        704.0 / 45.0,
        -107.0 / 9.0,
        67.0 / 90.0,
        3.0,
        0.0,
        0.0,
        -91.0 / 108.0,
        0.0,
        0.0,
        23.0 / 108.0,
        -976.0 / 135.0,
        311.0 / 54.0,
        -19.0 / 60.0,
        17.0 / 6.0,
        -1.0 / 12.0,
        0.0,
        2383.0 / 4100.0,
        0.0,
        0.0,
        -341.0 / 164.0,
        4496.0 / 1025.0,
        -301.0 / 82.0,
        2133.0 / 4100.0,
        45.0 / 82.0,
        45.0 / 164.0,
        18.0 / 41.0,
    ],
    gamma: &[
        41.0 / 840.0,
        0.0,
        0.0,
        0.0,
        0.0,
        34.0 / 105.0,
        9.0 / 35.0,
        9.0 / 35.0,
        9.0 / 280.0,
        9.0 / 280.0,
        41.0 / 840.0,
    ],
};

/// RK8 — 13 stages (Prince-Dormand 8(7)).
pub const RK8: RkCoeffs = RkCoeffs {
    n: 13,
    alpha: &[
        2.0 / 27.0,
        1.0 / 9.0,
        1.0 / 6.0,
        5.0 / 12.0,
        0.5,
        5.0 / 6.0,
        1.0 / 6.0,
        2.0 / 3.0,
        1.0 / 3.0,
        1.0,
        0.0,
        1.0,
    ],
    beta: &[
        2.0 / 27.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0 / 36.0,
        1.0 / 12.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0 / 24.0,
        0.0,
        1.0 / 8.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        5.0 / 12.0,
        0.0,
        -25.0 / 16.0,
        25.0 / 16.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0 / 20.0,
        0.0,
        0.0,
        1.0 / 4.0,
        1.0 / 5.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -25.0 / 108.0,
        0.0,
        0.0,
        125.0 / 108.0,
        -65.0 / 27.0,
        125.0 / 54.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        31.0 / 300.0,
        0.0,
        0.0,
        0.0,
        61.0 / 225.0,
        -2.0 / 9.0,
        13.0 / 900.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        2.0,
        0.0,
        0.0,
        -53.0 / 6.0,
        704.0 / 45.0,
        -107.0 / 9.0,
        67.0 / 90.0,
        3.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -91.0 / 108.0,
        0.0,
        0.0,
        23.0 / 108.0,
        -976.0 / 135.0,
        311.0 / 54.0,
        -19.0 / 60.0,
        17.0 / 6.0,
        -1.0 / 12.0,
        0.0,
        0.0,
        0.0,
        2383.0 / 4100.0,
        0.0,
        0.0,
        -341.0 / 164.0,
        4496.0 / 1025.0,
        -301.0 / 82.0,
        2133.0 / 4100.0,
        45.0 / 82.0,
        45.0 / 164.0,
        18.0 / 41.0,
        0.0,
        0.0,
        3.0 / 205.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -6.0 / 41.0,
        -3.0 / 205.0,
        -3.0 / 41.0,
        3.0 / 41.0,
        6.0 / 41.0,
        0.0,
        0.0,
        -1777.0 / 4100.0,
        0.0,
        0.0,
        -341.0 / 164.0,
        4496.0 / 1025.0,
        -289.0 / 82.0,
        2193.0 / 4100.0,
        51.0 / 82.0,
        33.0 / 164.0,
        12.0 / 41.0,
        0.0,
        1.0,
    ],
    gamma: &[
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        34.0 / 105.0,
        9.0 / 35.0,
        9.0 / 35.0,
        9.0 / 280.0,
        9.0 / 280.0,
        0.0,
        41.0 / 840.0,
        41.0 / 840.0,
    ],
};

// ===========================================================
// Yoshida symplectic coefficients (BodyIntegrator.cpp:322-425)
// ===========================================================

pub struct SyCoeffs {
    /// Drift fractions (length ndrift = nkick+1).
    pub c: &'static [f64],
    /// Kick fractions (length nkick).
    pub d: &'static [f64],
}

/// SY2 — order 2 (velocity Verlet / leapfrog).
pub const SY2: SyCoeffs = SyCoeffs {
    c: &[0.5, 0.5],
    d: &[1.0],
};

/// SY4 — order 4 (Forest-Ruth / Yoshida).
pub const SY4: SyCoeffs = {
    const B: f64 = 1.25992104989487319066654436028; // 2^(1/3)
    const A: f64 = 2.0 - B;
    const X0: f64 = -B / A;
    const X1: f64 = 1.0 / A;
    SyCoeffs {
        c: &[X1 / 2.0, (X0 + X1) / 2.0, (X0 + X1) / 2.0, X1 / 2.0],
        d: &[X1, X0, X1],
    }
};

/// SY6 — order 6 (Yoshida solution set 1).
pub const SY6: SyCoeffs = {
    const W0: f64 = 1.0 - 2.0 * (-0.117767998417887E1 + 0.235573213359357E0 + 0.784513610477560E0);
    SyCoeffs {
        c: &[
            0.784513610477560E0 / 2.0,
            (0.784513610477560E0 + 0.235573213359357E0) / 2.0,
            (0.235573213359357E0 + -0.117767998417887E1) / 2.0,
            (-0.117767998417887E1 + W0) / 2.0,
            (-0.117767998417887E1 + W0) / 2.0,
            (0.235573213359357E0 + -0.117767998417887E1) / 2.0,
            (0.784513610477560E0 + 0.235573213359357E0) / 2.0,
            0.784513610477560E0 / 2.0,
        ],
        d: &[
            0.784513610477560E0,
            0.235573213359357E0,
            -0.117767998417887E1,
            W0,
            -0.117767998417887E1,
            0.235573213359357E0,
            0.784513610477560E0,
        ],
    }
};

/// SY8 — order 8 (Yoshida solution set 3).
pub const SY8: SyCoeffs = {
    const W1: f64 = 0.311790812418427e0;
    const W2: f64 = -0.155946803821447e1;
    const W3: f64 = -0.167896928259640e1;
    const W4: f64 = 0.166335809963315e1;
    const W5: f64 = -0.106458714789183e1;
    const W6: f64 = 0.136934946416871e1;
    const W7: f64 = 0.629030650210433e0;
    const W0: f64 = 1.0 - 2.0 * (W1 + W2 + W3 + W4 + W5 + W6 + W7);
    SyCoeffs {
        c: &[
            W7 / 2.0,
            (W7 + W6) / 2.0,
            (W6 + W5) / 2.0,
            (W5 + W4) / 2.0,
            (W4 + W3) / 2.0,
            (W3 + W2) / 2.0,
            (W2 + W1) / 2.0,
            (W1 + W0) / 2.0,
            (W1 + W0) / 2.0,
            (W2 + W1) / 2.0,
            (W3 + W2) / 2.0,
            (W4 + W3) / 2.0,
            (W5 + W4) / 2.0,
            (W6 + W5) / 2.0,
            (W7 + W6) / 2.0,
            W7 / 2.0,
        ],
        d: &[W7, W6, W5, W4, W3, W2, W1, W0, W1, W2, W3, W4, W5, W6, W7],
    }
};

// ===========================================================
// Integrator step functions
// ===========================================================

/// RK2 midpoint method step (mirrors RK2_LinAng, BodyIntegrator.cpp:157).
///
/// Advances state `s1` by one step `h`. The `force` closure is called once at
/// the midpoint.
pub fn rk2_step(s1: StateVectors, h: f64, force: &mut ForceFn) -> StateVectors {
    let h05 = h * 0.5;

    // Initial acceleration (passed in via force at s1, t_frac=0).
    let (acc0, arot0) = force(&s1, 0.0);

    // Midpoint state.
    let mut sm = StateVectors {
        pos: s1.pos + s1.vel * h05,
        vel: s1.vel + acc0 * h05,
        omega: s1.omega + arot0 * h05,
        ..s1
    };
    sm.set_rot_quat(s1.q.rotated(s1.omega * h05));

    // Evaluate force at midpoint.
    let (acc1, arot1) = force(&sm, 0.5);

    // Final update.
    let mut result = s1;
    result.pos += sm.vel * h;
    result.vel += acc1 * h;
    result.omega += arot1 * h;
    result.q.rotate(sm.omega * h);
    result.r = orbitx_math::Matrix3::from_quat(result.q);
    result
}

/// RK4 classical 4th-order step (mirrors RK4_LinAng, BodyIntegrator.cpp:180).
pub fn rk4_step(s1: StateVectors, h: f64, force: &mut ForceFn) -> StateVectors {
    let h05 = h * 0.5;
    let hi6 = h / 6.0;

    let (acc0, arot0) = force(&s1, 0.0);

    // Stage A (midpoint from s1).
    let mut sa = StateVectors {
        pos: s1.pos + s1.vel * h05,
        vel: s1.vel + acc0 * h05,
        omega: s1.omega + arot0 * h05,
        ..s1
    };
    sa.set_rot_quat(s1.q.rotated(s1.omega * h05));
    let (acc1, aacc1) = force(&sa, 0.5);

    // Stage B (midpoint from A).
    let mut sb = StateVectors {
        pos: s1.pos + sa.vel * h05,
        vel: s1.vel + acc1 * h05,
        omega: s1.omega + aacc1 * h05,
        ..s1
    };
    sb.set_rot_quat(s1.q.rotated(sa.omega * h05));
    let (acc2, aacc2) = force(&sb, 0.5);

    // Stage C (full step from B).
    let mut sc = StateVectors {
        pos: s1.pos + sb.vel * h,
        vel: s1.vel + acc2 * h,
        omega: s1.omega + aacc2 * h,
        ..s1
    };
    sc.set_rot_quat(s1.q.rotated(sb.omega * h));
    let (acc3, aacc3) = force(&sc, 1.0);

    // Weighted combination.
    let mut result = s1;
    result.vel += (acc0 + (acc1 + acc2) * 2.0 + acc3) * hi6;
    result.pos += (s1.vel + (sa.vel + sb.vel) * 2.0 + sc.vel) * hi6;
    result.omega += (arot0 + (aacc1 + aacc2) * 2.0 + aacc3) * hi6;
    result
        .q
        .rotate((s1.omega + (sa.omega + sb.omega) * 2.0 + sc.omega) * hi6);
    result.r = orbitx_math::Matrix3::from_quat(result.q);
    result
}

/// Generic RK driver for RK5-RK8 (mirrors RKdrv_LinAng, BodyIntegrator.cpp:219).
///
/// Uses the Butcher tableau `coeffs` to perform an n-stage RK step.
///
/// The `beta` array is packed lower-triangular: row `i` (for stage `i+1`) has
/// `i+1` elements starting at index `i*(i+1)/2`.
pub fn rk_drv(s1: StateVectors, h: f64, coeffs: &RkCoeffs, force: &mut ForceFn) -> StateVectors {
    let n = coeffs.n;
    let mut stages: Vec<StateVectors> = Vec::with_capacity(n);
    let mut accs: Vec<Vec3> = Vec::with_capacity(n);
    let mut arots: Vec<Vec3> = Vec::with_capacity(n);

    // Stage 0: initial state.
    stages.push(s1);
    let (a0, ar0) = force(&s1, 0.0);
    accs.push(a0);
    arots.push(ar0);

    // Stages 1..n-1.
    // Beta is stored as a full (n-1)×(n-1) lower-triangular matrix (row-major,
    // matching the C++ layout). Row i-1 starts at index (i-1)*(n-1).
    for i in 1..n {
        let mut si = s1;
        let beta_row_start = (i - 1) * (n - 1);
        for j in 0..i {
            let beta = coeffs.beta[beta_row_start + j];
            si.advance(beta * h, accs[j], stages[j].vel, arots[j], stages[j].omega);
        }
        let t_frac = if i <= coeffs.alpha.len() {
            coeffs.alpha[i - 1]
        } else {
            0.0
        };
        let (ai, ari) = force(&si, t_frac);
        stages.push(si);
        accs.push(ai);
        arots.push(ari);
    }

    // Final weighted combination.
    let mut result = s1;
    for i in 0..n {
        let bh = coeffs.gamma[i] * h;
        result.vel += accs[i] * bh;
        result.pos += stages[i].vel * bh;
        result.omega += arots[i] * bh;
        result.q.rotate(stages[i].omega * bh);
    }
    result.r = orbitx_math::Matrix3::from_quat(result.q);
    result
}

/// Dispatch RK step by method (RK2/4 hand-written, RK5-8 via driver).
pub fn rk_step(method: &str, s1: StateVectors, h: f64, force: &mut ForceFn) -> StateVectors {
    match method {
        "RK2" => rk2_step(s1, h, force),
        "RK4" => rk4_step(s1, h, force),
        "RK5" => rk_drv(s1, h, &RK5, force),
        "RK6" => rk_drv(s1, h, &RK6, force),
        "RK7" => rk_drv(s1, h, &RK7, force),
        "RK8" => rk_drv(s1, h, &RK8, force),
        _ => panic!("unknown RK method: {method}"),
    }
}

/// Symplectic step (drift-kick-drift, mirrors SY*_LinAng, BodyIntegrator.cpp:301-425).
///
/// Uses the Yoshida `coeffs` to perform a composition step.
pub fn sy_step(s1: StateVectors, h: f64, coeffs: &SyCoeffs, force: &mut ForceFn) -> StateVectors {
    let ndrift = coeffs.c.len();
    let mut state = s1;
    let mut sec = 0.0_f64;

    for i in 0..ndrift {
        // Drift.
        let step = h * coeffs.c[i];
        state.pos += state.vel * step;
        state.q.rotate(state.omega * step);
        state.r = orbitx_math::Matrix3::from_quat(state.q);
        sec += coeffs.c[i];

        // Kick (skip after last drift).
        if i < ndrift - 1 {
            let (acc, arot) = force(&state, sec);
            state.vel += acc * (h * coeffs.d[i]);
            state.omega += arot * (h * coeffs.d[i]);
        }
    }

    state
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::Vec3;

    /// 2-body gravitational acceleration (simple inverse-square).
    fn make_gravity_force(gm: f64) -> impl FnMut(&StateVectors, f64) -> (Vec3, Vec3) {
        move |s: &StateVectors, _t: f64| {
            let r = s.pos;
            let d = r.length();
            let acc = r * (-gm / (d * d * d));
            (acc, Vec3::ZERO)
        }
    }

    #[test]
    fn rk4_circular_orbit_stable() {
        // Circular orbit at r=7000 km, mu=3.986e14.
        let gm: f64 = 3.986e14;
        let r0: f64 = 7.0e6;
        let v0 = (gm / r0).sqrt();
        let period = 2.0 * std::f64::consts::PI * r0 / v0;

        let s0 = StateVectors {
            pos: Vec3::new(r0, 0.0, 0.0),
            vel: Vec3::new(0.0, 0.0, v0),
            ..StateVectors::default()
        };

        let n_steps = 1000;
        let dt = period / n_steps as f64;
        let mut state = s0;

        for _ in 0..n_steps {
            state = rk4_step(state, dt, &mut make_gravity_force(gm));
        }

        // After one full orbit, position should be close to start.
        let err = (state.pos - s0.pos).length() / r0;
        assert!(err < 1e-3, "RK4 orbit error = {err}");
    }

    #[test]
    fn sy2_circular_orbit_stable() {
        let gm: f64 = 3.986e14;
        let r0: f64 = 7.0e6;
        let v0 = (gm / r0).sqrt();
        let period = 2.0 * std::f64::consts::PI * r0 / v0;

        let s0 = StateVectors {
            pos: Vec3::new(r0, 0.0, 0.0),
            vel: Vec3::new(0.0, 0.0, v0),
            ..StateVectors::default()
        };

        let n_steps = 1000;
        let dt = period / n_steps as f64;
        let mut state = s0;

        for _ in 0..n_steps {
            state = sy_step(state, dt, &SY2, &mut make_gravity_force(gm));
        }

        let err = (state.pos - s0.pos).length() / r0;
        assert!(err < 1e-3, "SY2 orbit error = {err}");
    }

    #[test]
    fn rk8_higher_order_better() {
        // RK8 should be more accurate than RK4 for the same step size.
        let gm: f64 = 3.986e14;
        let r0: f64 = 7.0e6;
        let v0 = (gm / r0).sqrt();
        let period = 2.0 * std::f64::consts::PI * r0 / v0;

        let s0 = StateVectors {
            pos: Vec3::new(r0, 0.0, 0.0),
            vel: Vec3::new(0.0, 0.0, v0),
            ..StateVectors::default()
        };

        let n_steps = 100;
        let dt = period / n_steps as f64;

        let mut state4 = s0;
        let mut state8 = s0;
        let mut f4 = make_gravity_force(gm);
        let mut f8 = make_gravity_force(gm);

        for _ in 0..n_steps {
            state4 = rk4_step(state4, dt, &mut f4);
            state8 = rk_drv(state8, dt, &RK8, &mut f8);
        }

        let err4 = (state4.pos - s0.pos).length() / r0;
        let err8 = (state8.pos - s0.pos).length() / r0;
        assert!(err8 < err4, "RK8 err={err8} should be < RK4 err={err4}");
    }
}
