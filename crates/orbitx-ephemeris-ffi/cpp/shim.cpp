// Self-contained C++ oracle for the ephemeris property tests.
//
// Rather than compiling Orbiter's VSOPOBJ/ELP82 class hierarchy (which drags in
// CELBODY, CELBODY2, ATMOSPHERE, and the entire engine), this file re-implements
// the evaluation algorithms as free functions, copied verbatim from:
//   - Src/Celbody/Vsop87/Vsop87.cpp  (VsopEphem, VsopFastEphem, Interpolate)
//   - Src/Celbody/Moon/ELP82.cpp    (ELP82_init, ELP82_read, ELP82)
//
// The data structures and formulas are identical to the originals; only the
// class wrapper is replaced with a C struct + free functions.

#include "oracle.h"
#include <cstring>
#include <fstream>
#include <vector>

// Global MJD reference time (J2000 by default).
double ox_MJD_ref = 51544.5;

// ===========================================================
// VSOP87
// ===========================================================

#define VSOP_MAXALPHA 5

struct VsopData {
    int nalpha;
    int fmtflag;     // EPHEM_POLAR, EPHEM_PARENTBARY
    double a0;
    double prec;
    double interval;
    // termidx[alpha][cooidx], termlen[alpha][cooidx], term[]
    // Using flat arrays indexed by [alpha*3+cooidx]
    int termidx[6][3];    // start offset
    int termlen[7][3];    // count (sentinel row [nalpha+1] = 0)
    std::vector<double> terms; // flat: triples of (a,b,c)
    Sample sp[2];
};

// Radius helper (from Vsop87.cpp:16)
inline double VsopRadius(const double *data) {
    return sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]);
}

// Interpolate (from Vsop87.cpp:282, also Moon.cpp) — verbatim
static void VsopInterpolate(double t, double *data, const Sample *s0, const Sample *s1) {
    int i;
    double dt = s1->t - s0->t;
    if (dt == 0.0) {
        for (i = 0; i < 6; i++) data[i] = s0->param[i];
        return;
    }
    double u = (t - s0->t) / dt;
    double u2 = u*u, u3 = u2*u;
    double h00 = 2.0*u3 - 3.0*u2 + 1.0;
    double h10 = u3 - 2.0*u2 + u;
    double h01 = -2.0*u3 + 3.0*u2;
    double h11 = u3 - u2;
    double dh00 = 6.0*u2 - 6.0*u;
    double dh10 = 3.0*u2 - 4.0*u + 1.0;
    double dh01 = -6.0*u2 + 6.0*u;
    double dh11 = 3.0*u2 - 2.0*u;
    for (i = 0; i < 3; i++) {
        double p0 = s0->param[i], p1 = s1->param[i];
        double v0 = s0->param[i+3]*dt, v1 = s1->param[i+3]*dt;
        data[i] = h00*p0 + h10*v0 + h01*p1 + h11*v1;
        data[i+3] = (dh00*p0 + dh10*v0 + dh01*p1 + dh11*v1)/dt;
    }
}

// VsopEphem (from Vsop87.cpp:141) — verbatim, adapted to VsopData
static void VsopEphem(const VsopData *vd, double mjd, double *ret) {
    static const double mjd2000 = 51544.5;
    static const double a1000   = 365250.0;
    static const double rsec    = 1.0/(a1000*86400.0);
    static const double c0   = 299792458.0;
    static const double tauA = 499.004783806;
    static const double AU   = c0*tauA;
    static const double pscl = AU;
    static const double vscl = AU*rsec;

    int i, cooidx, alpha;
    for (i = 0; i < 6; i++) ret[i] = 0.0;

    double t[VSOP_MAXALPHA+1];
    t[0] = 1.0;
    t[1] = (mjd-mjd2000)/a1000;
    for (i = 2; i <= VSOP_MAXALPHA; ++i) t[i] = t[i-1] * t[1];

    for (cooidx = 0; cooidx < 3; ++cooidx) {
        for (alpha = 0; vd->termlen[alpha][cooidx]; ++alpha) {
            int start = vd->termidx[alpha][cooidx];
            int len = vd->termlen[alpha][cooidx];
            double tm = 0.0, termdot = 0.0;
            for (i = 0; i < len; ++i) {
                double a = vd->terms[(start+i)*3+0];
                double b = vd->terms[(start+i)*3+1];
                double c = vd->terms[(start+i)*3+2];
                double arg = b + c*t[1];
                tm      += a*cos(arg);
                termdot -= c*a*sin(arg);
            }
            ret[cooidx] += t[alpha]*tm;
            ret[cooidx+3] += t[alpha]*termdot +
                (alpha > 0 ? alpha*t[alpha-1]*tm : 0.0);
        }
    }

    if (vd->fmtflag & EPHEM_POLAR) {
        for (i = 3; i < 6; i++) ret[i] *= rsec;
    } else {
        double tmp;
        for (i = 0; i < 3; i++) ret[i] *= pscl;
        for (     ; i < 6; i++) ret[i] *= vscl;
        tmp = ret[1]; ret[1] = ret[2]; ret[2] = tmp;
        tmp = ret[4]; ret[4] = ret[5]; ret[5] = tmp;
    }
}

// VsopFastEphem (from Vsop87.cpp:216) — verbatim, adapted to VsopData
static void VsopFastEphem(VsopData *vd, double simt, double *ret) {
    Sample *s0, *s1;
    double interval = vd->interval;
    if (vd->sp[0].t < vd->sp[1].t) { s0 = vd->sp+0; s1 = vd->sp+1; }
    else                           { s0 = vd->sp+1; s1 = vd->sp+0; }

    if (simt >= s0->t && simt <= s1->t) {
        VsopInterpolate(simt, ret, s0, s1);
    } else if (simt > s1->t) {
        if (simt <= s1->t + interval) {
            s0->t = s1->t + interval;
            VsopEphem(vd, oapiTime2MJD(s0->t), s0->param);
            if (vd->fmtflag & EPHEM_POLAR) {
                if      (s0->param[0]-s1->param[0] >  PI) s1->param[0] += 2.0*PI;
                else if (s0->param[0]-s1->param[0] < -PI) s1->param[0] -= 2.0*PI;
            } else {
                s0->rad = VsopRadius(s0->param);
            }
            VsopInterpolate(simt, ret, s1, s0);
        } else {
            s0->t = simt;
            VsopEphem(vd, oapiTime2MJD(s0->t), s0->param);
            if (!(vd->fmtflag & EPHEM_POLAR))
                s0->rad = VsopRadius(s0->param);
            for (int i = 0; i < 6; i++) ret[i] = s0->param[i];
        }
    } else {
        if (simt >= s0->t - interval) {
            s1->t = s0->t - interval;
            VsopEphem(vd, oapiTime2MJD(s1->t), s1->param);
            if (vd->fmtflag & EPHEM_POLAR) {
                if      (s1->param[0]-s0->param[0] >  PI) s0->param[0] += 2.0*PI;
                else if (s1->param[0]-s0->param[0] < -PI) s0->param[0] -= 2.0*PI;
            } else {
                s1->rad = VsopRadius(s1->param);
            }
            VsopInterpolate(simt, ret, s1, s0);
        } else {
            s1->t = simt;
            VsopEphem(vd, oapiTime2MJD(s1->t), s1->param);
            s0->t = simt + interval;
            VsopEphem(vd, oapiTime2MJD(s0->t), s0->param);
            if (vd->fmtflag & EPHEM_POLAR) {
                if      (s0->param[0]-s1->param[0] >  PI) s1->param[0] += 2.0*PI;
                else if (s0->param[0]-s1->param[0] < -PI) s1->param[0] -= 2.0*PI;
            } else {
                s0->rad = VsopRadius(s0->param);
                s1->rad = VsopRadius(s1->param);
            }
            for (int i = 0; i < 6; i++) ret[i] = s1->param[i];
        }
    }
}

// ReadData (from Vsop87.cpp:60) — verbatim, adapted to VsopData
// Returns true on success.
static bool VsopReadData(VsopData *vd, const char *path, char sid) {
    std::ifstream ifs(path);
    if (!ifs) return false;

    int nterm, cooidx, alpha, i, iused, nused = 0;
    double a, b, c, tfac, err;

    // Temporary per-group storage
    std::vector<std::vector<double>> groups(3 * (VSOP_MAXALPHA+1) * 3);
    // groups[gi] contains a flat list of (a,b,c) triples for group gi
    // We store as a flat double vector per group.

    std::vector<std::vector<double>> grp(3 * (VSOP_MAXALPHA+1));

    ifs >> vd->nalpha;

    for (cooidx = 0; cooidx < 3; cooidx++) {
        tfac = 1.0;
        for (alpha = 0; alpha <= vd->nalpha; alpha++) {
            ifs >> nterm;
            auto &g = grp[cooidx*(vd->nalpha+1)+alpha];
            g.clear();
            g.reserve(nterm * 3);
            iused = nterm;
            for (i = 0; i < nterm; i++) {
                ifs >> a >> b >> c;
                if (iused == nterm) {
                    g.push_back(a); g.push_back(b); g.push_back(c);
                    if (cooidx == 2) a /= vd->a0;
                    err = 2.0*sqrt((double)(i+1))*a*tfac;
                    if (err < vd->prec) iused = i;
                }
            }
            // Truncate to iused terms
            g.resize(iused * 3);
            vd->termlen[alpha][cooidx] = iused;
            vd->termidx[alpha][cooidx] = nused;
            nused += iused;
            tfac *= 5.0;
        }
        vd->termlen[alpha][cooidx] = 0; // sentinel
    }

    // Flatten
    vd->terms.clear();
    vd->terms.reserve(nused * 3);
    for (cooidx = 0; cooidx < 3; cooidx++) {
        for (alpha = 0; alpha <= vd->nalpha; alpha++) {
            auto &g = grp[cooidx*(vd->nalpha+1)+alpha];
            for (size_t j = 0; j < g.size(); j += 3) {
                double ta = g[j], tb = g[j+1], tc = g[j+2];
                if (cooidx == 2) ta /= vd->a0;
                vd->terms.push_back(ta);
                vd->terms.push_back(tb);
                vd->terms.push_back(tc);
            }
        }
    }

    // Init samples
    vd->sp[0].t = 0;
    vd->sp[1].t = vd->interval;
    VsopEphem(vd, oapiTime2MJD(vd->sp[0].t), vd->sp[0].param);
    VsopEphem(vd, oapiTime2MJD(vd->sp[1].t), vd->sp[1].param);
    vd->sp[0].rad = VsopRadius(vd->sp[0].param);
    vd->sp[1].rad = VsopRadius(vd->sp[1].param);

    (void)nused; // silence unused warning in some configs
    return true;
}

// ===========================================================
// ELP82
// ===========================================================

// Constants (from ELP82.cpp:16-37)
static const double elp_cpi     = 3.141592653589793;
static const double elp_cpi2    = 2.0*elp_cpi;
static const double elp_pis2    = elp_cpi/2.0;
static const double elp_rad     = 648000.0/elp_cpi;
static const double elp_deg     = elp_cpi/180.0;
static const double elp_c1      = 60.0;
static const double elp_c2      = 3600.0;
static const double elp_ath     = 384747.9806743165;
static const double elp_a0      = 384747.9806448954;
static const double elp_am      = 0.074801329518;
static const double elp_alpha   = 0.002571881335;
static const double elp_dtasm   = 2.0*elp_alpha/(3.0*elp_am);
static const double elp_mjd2000 = 51544.5;
static const double elp_sc      = 36525.0;
static const double elp_precess = 5029.0966/elp_rad;

// Global state (from ELP82.cpp:39-47)
static double elp_delnu, elp_dele, elp_delg, elp_delnp, elp_delep;
static double elp_p1, elp_p2, elp_p3, elp_p4, elp_p5;
static double elp_q1, elp_q2, elp_q3, elp_q4, elp_q5;
static double elp_w[3][5], elp_p[8][2], elp_eart[5], elp_peri[5];
static double elp_del[4][5], elp_zeta[2];
static int    elp_nterm[3];
static std::vector<double> elp_pc[3]; // each: flat array of SEQ6 (6 doubles per term)
static double elp_cur_prec = -1.0;

// ELP82_init (from ELP82.cpp:49-127) — verbatim
static void ELP82_init() {
    elp_w[0][0] = (218.0+18.0/elp_c1+59.95571/elp_c2)*elp_deg;
    elp_w[1][0] = (83.0+21.0/elp_c1+11.67475/elp_c2)*elp_deg;
    elp_w[2][0] = (125.0+2.0/elp_c1+40.39816/elp_c2)*elp_deg;
    elp_eart[0] = (100.0+27.0/elp_c1+59.22059/elp_c2)*elp_deg;
    elp_peri[0] = (102.0+56.0/elp_c1+14.42753/elp_c2)*elp_deg;
    elp_w[0][1] = 1732559343.73604/elp_rad;
    elp_w[1][1] = 14643420.2632/elp_rad;
    elp_w[2][1] = -6967919.3622/elp_rad;
    elp_eart[1] = 129597742.2758/elp_rad;
    elp_peri[1] = 1161.2283/elp_rad;
    elp_w[0][2] = -5.8883/elp_rad;
    elp_w[1][2] = -38.2776/elp_rad;
    elp_w[2][2] = 6.3622/elp_rad;
    elp_eart[2] = -0.0202/elp_rad;
    elp_peri[2] = 0.5327/elp_rad;
    elp_w[0][3] = 0.6604e-2/elp_rad;
    elp_w[1][3] = -0.45047e-1/elp_rad;
    elp_w[2][3] = 0.7625e-2/elp_rad;
    elp_eart[3] = 0.9e-5/elp_rad;
    elp_peri[3] = -0.138e-3/elp_rad;
    elp_w[0][4] = -0.3169e-4/elp_rad;
    elp_w[1][4] = 0.21301e-3/elp_rad;
    elp_w[2][4] = -0.3586e-4/elp_rad;
    elp_eart[4] = 0.15e-6/elp_rad;
    elp_peri[4] = 0.0;

    elp_p[0][0] = (252.0+15.0/elp_c1+3.25986/elp_c2)*elp_deg;
    elp_p[1][0] = (181.0+58.0/elp_c1+47.28305/elp_c2)*elp_deg;
    elp_p[2][0] = elp_eart[0];
    elp_p[3][0] = (355.0+25.0/elp_c1+59.78866/elp_c2)*elp_deg;
    elp_p[4][0] = (34.0+21.0/elp_c1+5.34212/elp_c2)*elp_deg;
    elp_p[5][0] = (50.0+4.0/elp_c1+38.89694/elp_c2)*elp_deg;
    elp_p[6][0] = (314.0+3.0/elp_c1+18.01841/elp_c2)*elp_deg;
    elp_p[7][0] = (304.0+20.0/elp_c1+55.19575/elp_c2)*elp_deg;
    elp_p[0][1] = 538101628.68898/elp_rad;
    elp_p[1][1] = 210664136.43355/elp_rad;
    elp_p[2][1] = elp_eart[1];
    elp_p[3][1] = 68905077.59284/elp_rad;
    elp_p[4][1] = 10925660.42861/elp_rad;
    elp_p[5][1] = 4399609.65932/elp_rad;
    elp_p[6][1] = 1542481.19393/elp_rad;
    elp_p[7][1] = 786550.32074/elp_rad;

    elp_delnu = +0.55604/elp_rad/elp_w[0][1];
    elp_dele  = +0.01789/elp_rad;
    elp_delg  = -0.08066/elp_rad;
    elp_delnp = -0.06424/elp_rad/elp_w[0][1];
    elp_delep = -0.12879/elp_rad;

    for (int i = 0; i < 5; i++) {
        elp_del[0][i] = elp_w[0][i] - elp_eart[i];
        elp_del[3][i] = elp_w[0][i] - elp_w[2][i];
        elp_del[2][i] = elp_w[0][i] - elp_w[1][i];
        elp_del[1][i] = elp_eart[i] - elp_peri[i];
    }
    elp_del[0][0] = elp_del[0][0] + elp_cpi;
    elp_zeta[0]   = elp_w[0][0];
    elp_zeta[1]   = elp_w[0][1] + elp_precess;

    elp_p1 =  0.10180391e-4;
    elp_p2 =  0.47020439e-6;
    elp_p3 = -0.5417367e-9;
    elp_p4 = -0.2507948e-11;
    elp_p5 =  0.463486e-14;
    elp_q1 = -0.113469002e-3;
    elp_q2 =  0.12372674e-6;
    elp_q3 =  0.1265417e-8;
    elp_q4 = -0.1371808e-11;
    elp_q5 = -0.320334e-14;
}

// ELP82_read (from ELP82.cpp:158, main problem only) — verbatim
static int ELP82_read(const char *path, double prec) {
    if (elp_cur_prec >= 0.0) {
        if (prec == elp_cur_prec) return 0;
        for (int i = 0; i < 3; i++) elp_pc[i].clear();
    }

    int ific, itab, m, mm, i, im, ir, k;
    double tgv, xx, y, pre[3], zone[6];

    pre[0] = prec*elp_rad;
    pre[1] = prec*elp_rad;
    pre[2] = prec*elp_ath;

    std::ifstream ifs(path);
    if (!ifs) return -1;

    for (ific = 0; ific < 3; ific++) {
        ifs >> m;
        // Read all terms into a temp buffer
        struct MainBin { int ilu[4]; double coef[7]; };
        std::vector<MainBin> block(m);
        for (ir = mm = 0; ir < m; ir++) {
            for (i = 0; i < 4; i++) ifs >> block[ir].ilu[i];
            for (i = 0; i < 7; i++) ifs >> block[ir].coef[i];
            if (fabs(block[ir].coef[0]) >= pre[ific]) mm++;
        }
        elp_pc[ific].clear();
        elp_pc[ific].reserve(mm * 6);

        for (im = ir = 0; im < m; im++) {
            MainBin &lin = block[im];
            xx = lin.coef[0];
            if (fabs(xx) < pre[ific]) continue;
            tgv = lin.coef[1] + elp_dtasm*lin.coef[5];
            if (ific == 2) lin.coef[0] -= 2.0*lin.coef[0]*elp_delnu/3.0;
            xx = lin.coef[0] + tgv*(elp_delnp-elp_am*elp_delnu) + lin.coef[2]*elp_delg +
                 lin.coef[3]*elp_dele + lin.coef[4]*elp_delep;
            zone[0] = xx;
            for (k = 0; k <= 4; k++) {
                y = 0.0;
                for (i = 0; i < 4; i++) y += lin.ilu[i]*elp_del[i][k];
                zone[k+1] = y;
            }
            if (ific == 2) zone[1] += elp_pis2;
            for (i = 0; i < 6; i++) elp_pc[ific].push_back(zone[i]);
            ir++;
        }
        elp_nterm[ific] = ir;
        (void)itab;
    }

    elp_cur_prec = prec;
    return 0;
}

// ELP82 (from ELP82.cpp:309) — verbatim
static int ELP82_eval(double mjd, double *r) {
    int k, iv, nt;
    double t[5];
    double x, y, x1, x2, x3, pw, qw, ra, pwqw, pw2, qw2;
    double x_dot, y_dot, x1_dot, x2_dot, x3_dot, pw_dot, qw_dot;
    double ra_dot, pwqw_dot, pw2_dot, qw2_dot;
    double cosr0, sinr0, cosr1, sinr1;

    t[0] = 1.0;
    t[1] = (mjd-elp_mjd2000)/elp_sc;
    t[2] = t[1]*t[1];
    t[3] = t[2]*t[1];
    t[4] = t[3]*t[1];

    for (iv = 0; iv < 3; iv++) {
        r[iv] = r[iv+3] = 0.0;
        const double *pciv = elp_pc[iv].data();

        for (nt = 0; nt < elp_nterm[iv]; nt++) {
            x = pciv[nt*6+0];     x_dot = 0.0;
            y = pciv[nt*6+1];     y_dot = 0.0;
            for (k = 1; k <= 4; k++) {
                y     += pciv[nt*6+k+1] * t[k];
                y_dot += pciv[nt*6+k+1] * t[k-1] * k;
            }
            r[iv]   += x*sin(y);
            r[iv+3] += x_dot*sin(y) + x*cos(y)*y_dot;
        }
    }

    r[0] = r[0]/elp_rad + elp_w[0][0] + elp_w[0][1]*t[1] + elp_w[0][2]*t[2] + elp_w[0][3]*t[3] +
                          elp_w[0][4]*t[4];
    r[3] = r[3]/elp_rad + elp_w[0][1] + 2*elp_w[0][2]*t[1] + 3*elp_w[0][3]*t[2] + 4*elp_w[0][4]*t[3];
    r[1] = r[1]/elp_rad;
    r[4] = r[4]/elp_rad;
    r[2] = r[2]*elp_a0/elp_ath;
    r[5] = r[5]*elp_a0/elp_ath;
    cosr0 = cos(r[0]), sinr0 = sin(r[0]);
    cosr1 = cos(r[1]), sinr1 = sin(r[1]);
    x1       = r[2]*cosr1;
    x1_dot   = r[5]*cosr1 - r[2]*sinr1*r[4];
    x2       = x1*sinr0;
    x2_dot   = x1_dot*sinr0 + x1*cosr0*r[3];
    x1_dot   = x1_dot*cosr0 - x1*sinr0*r[3];
    x1       = x1*cosr0;
    x3       = r[2]*sinr1;
    x3_dot   = r[5]*sinr1 + r[2]*cosr1*r[4];
    pw       = (elp_p1+elp_p2*t[1]+elp_p3*t[2]+elp_p4*t[3]+elp_p5*t[4])*t[1];
    pw_dot   = elp_p1 + 2*elp_p2*t[1] + 3*elp_p3*t[2] + 4*elp_p4*t[3] + 5*elp_p5*t[4];
    qw       = (elp_q1+elp_q2*t[1]+elp_q3*t[2]+elp_q4*t[3]+elp_q5*t[4])*t[1];
    qw_dot   = elp_q1 + 2*elp_q2*t[1] + 3*elp_q3*t[2] + 4*elp_q4*t[3] + 5*elp_q5*t[4];
    ra       = 2.0*sqrt(1-pw*pw-qw*qw);
    ra_dot   = -4.0*(pw+qw)/ra;
    pwqw     = 2.0*pw*qw;
    pwqw_dot = 2.0*(pw_dot*qw + pw*qw_dot);
    pw2      = 1-2.0*pw*pw;
    pw2_dot  = -4.0*pw;
    qw2      = 1-2.0*qw*qw;
    qw2_dot  = -4.0*qw;
    pw       = pw*ra;
    pw_dot   = pw_dot*ra + pw*ra_dot;
    qw       = qw*ra;
    qw_dot   = qw_dot*ra + qw*ra_dot;
    r[0] = pw2*x1+pwqw*x2+pw*x3;
    r[3] = pw2_dot*x1 + pw2*x1_dot + pwqw_dot*x2 + pwqw*x2_dot + pw_dot*x3 + pw*x3_dot;
    r[2] = pwqw*x1+qw2*x2-qw*x3;
    r[5] = pwqw_dot*x1 + pwqw*x1_dot + qw2_dot*x2 + qw2*x2_dot - qw_dot*x3 - qw*x3_dot;
    r[1] = -pw*x1+qw*x2+(pw2+qw2-1)*x3;
    r[4] = -pw_dot*x1 - pw*x1_dot + qw_dot*x2 + qw*x2_dot + (pw2_dot+qw2_dot)*x3 + (pw2+qw2-1)*x3_dot;

    static double pscale = 1e3;
    static double vscale = 1e3/(86400.0*elp_sc);
    r[0] *= pscale; r[1] *= pscale; r[2] *= pscale;
    r[3] *= vscale; r[4] *= vscale; r[5] *= vscale;
    return 0;
}

// ===========================================================
// C ABI exports
// ===========================================================

extern "C" {

// --- VSOP87 ---

VsopData* ox_vsop_create(char sid, double a0, double prec, double interval) {
    VsopData *vd = new VsopData();
    std::memset(vd, 0, sizeof(*vd));
    vd->a0 = a0;
    vd->prec = prec;
    vd->interval = interval;
    char su = (char)toupper((int)sid);
    vd->fmtflag = 0;
    if (su == 'B' || su == 'D') vd->fmtflag |= EPHEM_POLAR;
    if (su == 'E') vd->fmtflag |= EPHEM_PARENTBARY;
    vd->sp[0].t = vd->sp[1].t = -1e20;
    return vd;
}

void ox_vsop_destroy(VsopData *vd) { delete vd; }

int ox_vsop_read(VsopData *vd, const char *path) {
    char su = 0;
    if (vd->fmtflag & EPHEM_POLAR) su = 'B';
    else if (vd->fmtflag & EPHEM_PARENTBARY) su = 'E';
    return VsopReadData(vd, path, su) ? 1 : 0;
}

void ox_vsop_eval(const VsopData *vd, double mjd, double *ret) {
    VsopEphem(vd, mjd, ret);
}

void ox_vsop_fast_eval(VsopData *vd, double simt, double *ret) {
    VsopFastEphem(vd, simt, ret);
}

void ox_vsop_set_mjd_ref(double ref) { ox_MJD_ref = ref; }
double ox_vsop_get_mjd_ref() { return ox_MJD_ref; }

// --- ELP82 ---

int ox_elp_read(const char *path, double prec) {
    ELP82_init();
    return ELP82_read(path, prec);
}

void ox_elp_eval(double mjd, double *ret) {
    ELP82_eval(mjd, ret);
}

// --- Interpolate ---

void ox_interpolate(double t, double *data, const Sample *s0, const Sample *s1) {
    VsopInterpolate(t, data, s0, s1);
}

// ===========================================================
// TASS17 (Saturn moons) — verbatim from Tass17.cpp
// ===========================================================

typedef double TasTerm[3];
typedef int TasIks[8];

struct TasSeriesData {
    int ntr[5];
    std::vector<double> terms[4]; // flat: 3 doubles per term
    std::vector<int> iks[4];      // flat: 8 ints per term
    double al0, an0;
};

struct TasHyperion {
    int nbtp, nbtq, nbtz, nbtzt;
    double t0, cstp, cstq, amm7;
    std::vector<double> serp, fap, frp;
    std::vector<double> serq, faq, frq;
    std::vector<double> serz, faz, frz;
    std::vector<double> serzt, fazt, frzt;
};

struct TasModel {
    TasSeriesData sats[8];
    TasHyperion hyp;
    double gk1, aia, oma;
    double aam[9], tmas[9];
};

static const double TASS_AU = 299792458.0 * 499.004783806;
static const double TASS_AUy = TASS_AU / (86400.0 * 365.25);
static const double TASS_EPOCH = 2444240.0;

// calclon (Tass17.cpp:157)
static void TasCalclon(double dj, const TasModel *m, double *dlo) {
    double t = (dj - TASS_EPOCH) / 365.25;
    for (int is = 0; is < 8; is++) {
        if (is != 6) {
            const TasSeriesData *sd = &m->sats[is];
            const std::vector<double> &tm = sd->terms[1];
            double s = 0;
            for (int i = 0; i < sd->ntr[4]; i++) {
                s += tm[i*3+0] * sin(tm[i*3+1] + t * tm[i*3+2]);
            }
            dlo[is] = s;
        } else {
            dlo[is] = 0;
        }
    }
}

// calcelem (Tass17.cpp:99)
static void TasCalcelem(double dj, int is, double *elem, const TasModel *m, const double *dlo) {
    double t = (dj - TASS_EPOCH) / 365.25;
    const TasSeriesData *sd = &m->sats[is];
    double phas, s;
    int i, jk;

    // elem[0]
    s = 0;
    for (i = 0; i < sd->ntr[0]; i++) {
        phas = sd->terms[0][i*3+1];
        for (jk = 0; jk < 8; jk++) phas += sd->iks[0][i*8+jk] * dlo[jk];
        s += sd->terms[0][i*3+0] * cos(phas + t * sd->terms[0][i*3+2]);
    }
    elem[0] = s;

    // elem[1]
    s = dlo[is] + sd->al0;
    for (i = sd->ntr[4]; i < sd->ntr[1]; i++) {
        phas = sd->terms[1][i*3+1];
        for (jk = 0; jk < 8; jk++) phas += sd->iks[1][i*8+jk] * dlo[jk];
        s += sd->terms[1][i*3+0] * sin(phas + t * sd->terms[1][i*3+2]);
    }
    s += sd->an0 * t;
    elem[1] = atan2(sin(s), cos(s));

    // elem[2,3]
    double s1 = 0, s2 = 0;
    for (i = 0; i < sd->ntr[2]; i++) {
        phas = sd->terms[2][i*3+1];
        for (jk = 0; jk < 8; jk++) phas += sd->iks[2][i*8+jk] * dlo[jk];
        s1 += sd->terms[2][i*3+0] * cos(phas + t * sd->terms[2][i*3+2]);
        s2 += sd->terms[2][i*3+0] * sin(phas + t * sd->terms[2][i*3+2]);
    }
    elem[2] = s1; elem[3] = s2;

    // elem[4,5]
    s1 = 0; s2 = 0;
    for (i = 0; i < sd->ntr[3]; i++) {
        phas = sd->terms[3][i*3+1];
        for (jk = 0; jk < 8; jk++) phas += sd->iks[3][i*8+jk] * dlo[jk];
        s1 += sd->terms[3][i*3+0] * cos(phas + t * sd->terms[3][i*3+2]);
        s2 += sd->terms[3][i*3+0] * sin(phas + t * sd->terms[3][i*3+2]);
    }
    elem[4] = s1; elem[5] = s2;
}

// edered (Tass17.cpp:235)
static void TasEdered(const double *elem, double *xyz, double *vxyz, int isat, const TasModel *m) {
    double amo = m->aam[isat] * (elem[0] + 1);
    double rmu = m->gk1 * (m->tmas[isat] + 1);
    double dga = pow(rmu / (amo*amo), .33333333333333331);
    double rl = elem[1], rk = elem[2], rh = elem[3];
    double fle = rl - rk*sin(rl) + rh*cos(rl);
    double cf, sf, corf;
    do {
        cf = cos(fle); sf = sin(fle);
        corf = (rl - fle + rk*sf - rh*cf) / (1 - rk*cf - rh*sf);
        fle += corf;
    } while (fabs(corf) >= 1e-14);
    cf = cos(fle); sf = sin(fle);
    double dlf = -rk*sf + rh*cf;
    double rsam1 = -rk*cf - rh*sf;
    double asr = 1. / (rsam1 + 1);
    double phi = sqrt(1 - rk*rk - rh*rh);
    double psi = 1. / (phi + 1);
    double x1 = dga * (cf - rk - psi*rh*dlf);
    double y1 = dga * (sf - rh + psi*rk*dlf);
    double vx1 = amo * asr * dga * (-sf - psi*rh*rsam1);
    double vy1 = amo * asr * dga * (cf + psi*rk*rsam1);
    double dwho = sqrt(1 - elem[5]*elem[5] - elem[4]*elem[4]) * 2;
    double rtp = 1 - elem[5]*2*elem[5];
    double rtq = 1 - elem[4]*2*elem[4];
    double rdg = elem[5]*2*elem[4];
    double xyz2[3], vxyz2[3];
    xyz2[0] = x1*rtp + y1*rdg;
    xyz2[1] = x1*rdg + y1*rtq;
    xyz2[2] = (-x1*elem[5] + y1*elem[4]) * dwho;
    vxyz2[0] = vx1*rtp + vy1*rdg;
    vxyz2[1] = vx1*rdg + vy1*rtq;
    vxyz2[2] = (-vx1*elem[5] + vy1*elem[4]) * dwho;
    double ci = cos(m->aia), si = sin(m->aia);
    double co = cos(m->oma), so = sin(m->oma);
    xyz[0] = co*xyz2[0] - so*ci*xyz2[1] + so*si*xyz2[2];
    xyz[1] = so*xyz2[0] + co*ci*xyz2[1] - co*si*xyz2[2];
    xyz[2] = si*xyz2[1] + ci*xyz2[2];
    vxyz[0] = co*vxyz2[0] - so*ci*vxyz2[1] + so*si*vxyz2[2];
    vxyz[1] = so*vxyz2[0] + co*ci*vxyz2[1] - co*si*vxyz2[2];
    vxyz[2] = si*vxyz2[1] + ci*vxyz2[2];
}

// elemhyp (Tass17.cpp:319)
static void TasElemhyp(double dj, double *elem, const TasModel *m) {
    const TasHyperion *h = &m->hyp;
    double t = dj - h->t0;
    double p = h->cstp;
    for (int i = 0; i < h->nbtp; i++) {
        double wt = t * h->frp[i] + h->fap[i];
        p += h->serp[i] * cos(wt);
    }
    double q = h->cstq;
    for (int i = 0; i < h->nbtq; i++) {
        double wt = t * h->frq[i] + h->faq[i];
        q += h->serq[i] * sin(wt);
    }
    double zr = 0, zi = 0;
    for (int i = 0; i < h->nbtz; i++) {
        double wt = t * h->frz[i] + h->faz[i];
        zr += h->serz[i] * cos(wt);
        zi += h->serz[i] * sin(wt);
    }
    double ztr = 0, zti = 0;
    for (int i = 0; i < h->nbtzt; i++) {
        double wt = t * h->frzt[i] + h->fazt[i];
        ztr += h->serzt[i] * cos(wt);
        zti += h->serzt[i] * sin(wt);
    }
    double vl = fmod(h->amm7*t + q, 6.2831853071795862);
    if (vl < 0) vl += 6.2831853071795862;
    elem[0] = p; elem[1] = vl;
    elem[2] = zr; elem[3] = zi;
    elem[4] = ztr; elem[5] = zti;
}

// ReadData (Tass17.cpp:181)
static void TasReadData(TasModel *m, FILE *f) {
    static double radsdg = atan(1.) / 45.;
    double gk, tas, tam[9], am[9];
    int i, j, k, n, is, ieq, nt1, nt2, nt;
    double tm[3];
    int ik[8];

    fscanf(f, "%lf", &gk);
    fscanf(f, "%lf", &tas);
    m->gk1 = pow(gk*365.25, 2.0) / tas;
    fscanf(f, "%lf", &m->aia);
    fscanf(f, "%lf", &m->oma);
    m->aia *= radsdg;
    m->oma *= radsdg;
    for (i = 0; i < 9; i++) { fscanf(f, "%lf", tam+i); m->tmas[i] = 1./tam[i]; }
    for (i = 0; i < 9; i++) fscanf(f, "%lf", am+i);
    for (i = 0; i < 9; i++) m->aam[i] = am[i] * 365.25;

    for (i = 0; i < 8; i++) {
        if (i == 6) continue;
        for (j = 0; j < 4; j++) {
            fscanf(f, "%d%d%d%d", &is, &ieq, &nt1, &nt2);
            nt = nt2;
            m->sats[i].ntr[j] = nt;
            if (ieq == 2) {
                fscanf(f, "%d%lf%lf", &k, &m->sats[i].al0, &m->sats[i].an0);
                m->sats[i].ntr[4] = nt1;
            }
            for (k = 0; k < nt2; k++) {
                fscanf(f, "%d%lf%lf%lf", &n, tm+0, tm+1, tm+2);
                fscanf(f, "%d%d%d%d%d%d%d%d",
                    ik+0, ik+1, ik+2, ik+3, ik+4, ik+5, ik+6, ik+7);
                if (k < nt) {
                    for (int mm = 0; mm < 3; mm++) m->sats[i].terms[j].push_back(tm[mm]);
                    for (int mm = 0; mm < 8; mm++) m->sats[i].iks[j].push_back(ik[mm]);
                }
            }
        }
    }

    // Hyperion
    TasHyperion *h = &m->hyp;
    fscanf(f, "%lf", &h->t0);
    fscanf(f, "%lf", &h->amm7);
    fscanf(f, "%d", &h->nbtp);
    fscanf(f, "%lf", &h->cstp);
    for (i = 0; i < h->nbtp; i++) {
        double a, b, c; fscanf(f, "%lf%lf%lf", &a, &b, &c);
        h->serp.push_back(a); h->fap.push_back(b); h->frp.push_back(c);
    }
    fscanf(f, "%d", &h->nbtq);
    fscanf(f, "%lf", &h->cstq);
    for (i = 0; i < h->nbtq; i++) {
        double a, b, c; fscanf(f, "%lf%lf%lf", &a, &b, &c);
        h->serq.push_back(a); h->faq.push_back(b); h->frq.push_back(c);
    }
    fscanf(f, "%d", &h->nbtz);
    for (i = 0; i < h->nbtz; i++) {
        double a, b, c; fscanf(f, "%lf%lf%lf", &a, &b, &c);
        h->serz.push_back(a); h->faz.push_back(b); h->frz.push_back(c);
    }
    fscanf(f, "%d", &h->nbtzt);
    for (i = 0; i < h->nbtzt; i++) {
        double a, b, c; fscanf(f, "%lf%lf%lf", &a, &b, &c);
        h->serzt.push_back(a); h->fazt.push_back(b); h->frzt.push_back(c);
    }
}

// --- TASS17 C exports ---

TasModel* ox_tass17_create() {
    TasModel *m = new TasModel();
    memset(m, 0, sizeof(*m));
    return m;
}

void ox_tass17_destroy(TasModel *m) { delete m; }

int ox_tass17_read(TasModel *m, const char *path) {
    FILE *f = fopen(path, "rt");
    if (!f) return 0;
    TasReadData(m, f);
    fclose(f);
    return 1;
}

void ox_tass17_eval(const TasModel *m, double jd, int isat, double *ret) {
    double elem[6], dlo[8];
    if (isat == 6) {
        TasElemhyp(jd, elem, m);
    } else {
        TasCalclon(jd, m, dlo);
        TasCalcelem(jd, isat, elem, m, dlo);
    }
    double xyz[3], vxyz[3];
    TasEdered(elem, xyz, vxyz, isat, m);
    // Unit conversion: AU->m, AU/year->m/s, xzy swap
    ret[0] = xyz[0] * TASS_AU;
    ret[1] = xyz[2] * TASS_AU;
    ret[2] = xyz[1] * TASS_AU;
    ret[3] = vxyz[0] * TASS_AUy;
    ret[4] = vxyz[2] * TASS_AUy;
    ret[5] = vxyz[1] * TASS_AUy;
}

} // extern "C"
