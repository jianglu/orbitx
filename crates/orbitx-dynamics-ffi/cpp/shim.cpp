// Self-contained C++ oracle for the dynamics property tests.
//
// Re-implements key algorithms from Orbiter's dynamics system as free functions:
//   - SingleGacc (Psys.cpp:668) — point-mass gravity
//   - J2/J3/J4 zonal perturbation (Psys.cpp:619-664)
//   - Pines spherical harmonics (PinesGrav.cpp:185)
//   - Kepler EccAnomaly (Element.cpp:195)
//
// These are verbatim copies of the algorithms, adapted to use the Vec3d struct.

#include "oracle.h"
#include <vector>
#include <cstring>

// ===========================================================
// Point-mass gravity (Psys.cpp:668)
// ===========================================================

extern "C" void ox_single_gacc(double rx, double ry, double rz, double gm,
                                double *ax, double *ay, double *az) {
    Vec3d rpos = {rx, ry, rz};
    double d = v3_length(rpos);
    double f = gm / (d*d*d);
    *ax = rx * f;
    *ay = ry * f;
    *az = rz * f;
}

// ===========================================================
// J2/J3/J4 zonal perturbation (Psys.cpp:619-664)
// ===========================================================

extern "C" void ox_jcoeff_pert(double rx, double ry, double rz,
                                double body_size, double gm,
                                const double *jcoeff, int nj,
                                double *ax, double *ay, double *az) {
    const double eps = 1e-10;
    Vec3d rpos = {rx, ry, rz};
    double d = v3_length(rpos);
    double Rr = body_size / d;
    double Rrn = Rr * Rr;

    double Jn_Rrn = jcoeff[0] * Rrn;
    if (fabs(Jn_Rrn) <= eps) { *ax = *ay = *az = 0.0; return; }

    Vec3d er = v3_unit(rpos);
    double slat = -rpos.y / d;
    double clat = sqrt(1.0 - slat*slat);

    double gacc_r = 1.5 * Jn_Rrn * (1.0 - 3.0*slat*slat);
    double gacc_p = 3.0 * Jn_Rrn * clat * slat;

    if (nj > 1) {
        double J3_Rr3 = jcoeff[1] * Rr * Rrn;
        if (fabs(J3_Rr3) > eps) {
            gacc_r += 2.0 * J3_Rr3 * slat * (3.0 - 5.0*slat*slat);
            gacc_p += 1.5 * J3_Rr3 * clat * (-1.0 + 5.0*slat*slat);
        }
    }
    if (nj > 2) {
        double J4_Rr4 = jcoeff[2] * Rrn * Rrn;
        if (fabs(J4_Rr4) > eps) {
            gacc_r += -0.625 * J4_Rr4 * (3.0 + slat*slat*(-30.0 + 35.0*slat*slat));
            gacc_p += 2.5 * J4_Rr4 * clat * slat * (-3.0 + 7.0*slat*slat);
        }
    }

    double T0 = gm / (d*d);

    // Polar unit vector: perpendicular to er, toward y-axis projection.
    Vec3d ey = {0.0, 1.0, 0.0};
    double dot_er_ey = v3_dot(er, ey);
    Vec3d proj = v3_scale(er, dot_er_ey);
    Vec3d ep_unnorm = v3_sub(ey, proj);
    double ep_len = v3_length(ep_unnorm);
    Vec3d ep = (ep_len > eps) ? v3_scale(ep_unnorm, 1.0/ep_len) : ey;

    Vec3d result = v3_add(v3_scale(er, T0*gacc_r), v3_scale(ep, T0*gacc_p));
    *ax = result.x; *ay = result.y; *az = result.z;
}

// ===========================================================
// Pines spherical harmonic gravity (PinesGrav.cpp)
// ===========================================================

static inline unsigned int NM(unsigned int n, unsigned int m) { return (n*n+n)/2+m; }

static void GenerateAssocLegendreMatrix(double u, int maxDegree, std::vector<double> &A) {
    A[NM(0,0)] = sqrt(2.0);
    int md2 = maxDegree + 2;
    for (int m = 0; m <= md2; m++) {
        if (m != 0)
            A[NM(m,m)] = sqrt(1.0 + 1.0/(2.0*(double)m)) * A[NM(m-1,m-1)];
        if (m != md2)
            A[NM(m+1,m)] = sqrt(2.0*(double)m+3.0) * u * A[NM(m,m)];
        if (m < maxDegree+1) {
            for (int n = m+2; n <= md2; n++) {
                double ALPHA_NUM = (2.0*(double)n+1.0)*(2.0*(double)n-1.0);
                double ALPHA_DEN = ((double)n-(double)m)*((double)n+(double)m);
                double ALPHA = sqrt(ALPHA_NUM/ALPHA_DEN);
                double BETA_NUM = (2.0*(double)n+1.0)*((double)n-(double)m-1.0)*((double)n+(double)m-1.0);
                double BETA_DEN = (2.0*(double)n-3.0)*((double)n+(double)m)*((double)n-(double)m);
                double BETA = sqrt(BETA_NUM/BETA_DEN);
                A[NM(n,m)] = ALPHA*u*A[NM(n-1,m)] - BETA*A[NM(n-2,m)];
            }
        }
    }
    for (int n = 0; n <= md2; n++)
        A[NM(n,0)] *= sqrt(0.5);
}

extern "C" void ox_pines_accel(double rx, double ry, double rz,
                               double refRad, double GM,
                               const double *C, const double *S, int csLen,
                               int maxDegree, int maxOrder,
                               double *ax, double *ay, double *az) {
    Vec3d rpos = {rx, ry, rz};
    double r = v3_length(rpos);
    double s = rpos.x / r;
    double t = rpos.y / r;
    double u = rpos.z / r;

    double rho = GM / (r * refRad);
    double rhop = refRad / r;

    int maxM = maxOrder + 1;
    std::vector<double> R(maxM+2, 0.0), I(maxM+2, 0.0);
    R[1] = 1.0;
    for (int m = 2; m <= maxM; m++) {
        R[m] = s*R[m-1] - t*I[m-1];
        I[m] = s*I[m-1] + t*R[m-1];
    }

    std::vector<double> A(NM(maxDegree+2, maxDegree+2)+1, 0.0);
    GenerateAssocLegendreMatrix(u, maxDegree, A);

    double g1=0, g2=0, g3=0, g4=0;

    for (int n = 0; n <= maxDegree; n++) {
        double g1t=0, g2t=0, g3t=0, g4t=0;
        double SM = 0.5;
        int nmodel = (n > maxOrder) ? maxOrder : n;
        for (int m = 0; m <= nmodel; m++) {
            unsigned int idx = NM(n,m);
            double D = C[idx]*R[m+1] + S[idx]*I[m+1];
            double E = C[idx]*R[m]   + S[idx]*I[m];
            double F = S[idx]*R[m]   - C[idx]*I[m];
            double ALPHA = sqrt(SM * ((double)n-(double)m) * ((double)n+(double)m+1));
            g1t += A[NM(n,m)] * (double)m * E;
            g2t += A[NM(n,m)] * (double)m * F;
            g3t += ALPHA * A[NM(n,m+1)] * D;
            g4t += (((double)n+(double)m+1)*A[NM(n,m)] + ALPHA*u*A[NM(n,m+1)]) * D;
            if (m == 0) SM = 1.0;
        }
        rho = rhop * rho;
        g1 += rho*g1t; g2 += rho*g2t; g3 += rho*g3t; g4 += rho*g4t;
    }

    *ax = g1 - g4*s;
    *ay = g2 - g4*t;
    *az = g3 - g4*u;
    (void)csLen;
}

// ===========================================================
// Kepler EccAnomaly (Element.cpp:195)
// ===========================================================

extern "C" double ox_ecc_anomaly(double ma, double e, double ea0_in, double ma0_in) {
    const int niter = 16;
    const double tol = 1e-14;

    double E = (fabs(ma - ma0_in) < 1e-2) ? ea0_in : ma;

    if (e < 1.0) {
        double res = ma - E + e * sin(E);
        if (fabs(res) > fabs(ma)) { E = 0.0; res = ma; }
        for (int i = 0; fabs(res) > tol && i < niter; i++) {
            double step = res / (1.0 - e*cos(E));
            if (step > 1.0) step = 1.0; else if (step < -1.0) step = -1.0;
            E += step;
            res = ma - E + e*sin(E);
        }
    } else {
        double res = ma - e*sinh(E) + E;
        if (fabs(res) > fabs(ma)) { E = 0.0; res = ma; }
        for (int i = 0; fabs(res) > tol && i < niter; i++) {
            double step = res / (e*cosh(E) - 1.0);
            if (step > 1.0) step = 1.0; else if (step < -1.0) step = -1.0;
            E += step;
            res = ma - e*sinh(E) + E;
        }
    }
    return E;
}

// ===========================================================
// RK integrator step (linear only, BodyIntegrator.cpp RKdrv_LinAng)
// ===========================================================

// General RK driver for linear dynamics (pos/vel only).
// force_fn: called with (x, y, z, vx, vy, vz, t_frac) → (ax, ay, az)
// Stored as a global function pointer to avoid closure complexity in C ABI.

typedef void (*ForceCallback)(double x, double y, double z,
                              double vx, double vy, double vz,
                              double tfrac,
                              double *ax, double *ay, double *az);

static ForceCallback g_force_cb = nullptr;

extern "C" void ox_set_force_callback(ForceCallback cb) { g_force_cb = cb; }

// RK4 step for linear dynamics.
extern "C" void ox_rk4_step(double x, double y, double z,
                            double vx, double vy, double vz,
                            double h,
                            double *ox, double *oy, double *oz,
                            double *ovx, double *ovy, double *ovz) {
    double h05 = h*0.5;
    double hi6 = h/6.0;

    double ax1, ay1, az1;
    g_force_cb(x, y, z, vx, vy, vz, 0.0, &ax1, &ay1, &az1);

    // Stage a (midpoint)
    double ax_p = x + vx*h05;
    double ay_p = y + vy*h05;
    double az_p = z + vz*h05;
    double avx_p = vx + ax1*h05;
    double avy_p = vy + ay1*h05;
    double avz_p = vz + az1*h05;
    double ax2, ay2, az2;
    g_force_cb(ax_p, ay_p, az_p, avx_p, avy_p, avz_p, 0.5, &ax2, &ay2, &az2);

    // Stage b (midpoint from a)
    double bx_p = x + avx_p*h05;
    double by_p = y + avy_p*h05;
    double bz_p = z + avz_p*h05;
    double bvx_p = vx + ax2*h05;
    double bvy_p = vy + ay2*h05;
    double bvz_p = vz + az2*h05;
    double ax3, ay3, az3;
    g_force_cb(bx_p, by_p, bz_p, bvx_p, bvy_p, bvz_p, 0.5, &ax3, &ay3, &az3);

    // Stage c (full step from b)
    double cx_p = x + bvx_p*h;
    double cy_p = y + bvy_p*h;
    double cz_p = z + bvz_p*h;
    double cvx_p = vx + ax3*h;
    double cvy_p = vy + ay3*h;
    double cvz_p = vz + az3*h;
    double ax4, ay4, az4;
    g_force_cb(cx_p, cy_p, cz_p, cvx_p, cvy_p, cvz_p, 1.0, &ax4, &ay4, &az4);

    *ox  = x  + (vx  + (avx_p + bvx_p)*2.0 + cvx_p)*hi6;
    *oy  = y  + (vy  + (avy_p + bvy_p)*2.0 + cvy_p)*hi6;
    *oz  = z  + (vz  + (avz_p + bvz_p)*2.0 + cvz_p)*hi6;
    *ovx = vx + (ax1 + (ax2   + ax3  )*2.0 + az4  )*hi6;
    *ovy = vy + (ay1 + (ay2   + ay3  )*2.0 + ay4  )*hi6;
    *ovz = vz + (az1 + (az2   + az3  )*2.0 + az4  )*hi6;
}

// ===========================================================
// Rigid-body angular dynamics (Rigidbody.cpp:458-511)
// Verbatim copies of RigidBody::Euler_full / EulerInv_full /
// EulerInv_simple. pmi is the diagonal inertia tensor; tau is the
// mass-normalised (specific) torque; omega is the body-frame angular
// velocity.
// ===========================================================

extern "C" void ox_euler_inv_full(
    double taux, double tauy, double tauz,
    double wx,   double wy,   double wz,
    double px,   double py,   double pz,
    double *ax,  double *ay,  double *az) {
    // EulerInv_full, Rigidbody.cpp:477-481
    *ax = (taux - (py - pz) * wy * wz) / px;
    *ay = (tauy - (pz - px) * wz * wx) / py;
    *az = (tauz - (px - py) * wx * wy) / pz;
}

extern "C" void ox_euler_inv_simple(
    double taux, double tauy, double tauz,
    double px,   double py,   double pz,
    double *ax,  double *ay,  double *az) {
    // EulerInv_simple, Rigidbody.cpp:496
    *ax = taux / px;
    *ay = tauy / py;
    *az = tauz / pz;
}

extern "C" void ox_euler_full(
    double odx, double ody, double odz,
    double wx,  double wy,  double wz,
    double px,  double py,  double pz,
    double *tx, double *ty, double *tz) {
    // Euler_full, Rigidbody.cpp:460-463
    *tx = odx * px + (py - pz) * wy * wz;
    *ty = ody * py + (pz - px) * wz * wx;
    *tz = odz * pz + (px - py) * wx * wy;
}
