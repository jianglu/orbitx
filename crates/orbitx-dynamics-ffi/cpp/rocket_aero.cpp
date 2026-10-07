// Product rocket aero C++ oracle.
//
// - ox_wave_drag / ox_induced_drag: verbatim from Orbiter OrbiterAPI.cpp:873-884
//   (+ Rust guard A/eps <= 1e-9 -> 0 on induced).
// - Remaining synthesis: second implementation of
//   crates/orbitx-dynamics/src/aero/rocket.rs (AERO.md). Not from Vessel.cpp.
// Used only by orbitx-dynamics FFI property tests.

#include "oracle.h"
#include <algorithm>
#include <cmath>

namespace {

inline double signum(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

inline double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
}

inline double clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double piecewise_linear_cd(const double *m, const double *cd, int n,
                           double x, double fallback) {
    if (n <= 0) return fallback;
    if (n == 1) return cd[0];
    if (x <= m[0]) return cd[0];
    if (x >= m[n - 1]) return cd[n - 1];
    int lo = 0;
    int hi = n - 1;
    while (hi - lo > 1) {
        int mid = lo + (hi - lo) / 2;
        if (m[mid] <= x) lo = mid;
        else hi = mid;
    }
    double x0 = m[lo], y0 = cd[lo];
    double x1 = m[hi], y1 = cd[hi];
    double t = (x - x0) / (x1 - x0);
    return y0 + (y1 - y0) * t;
}

// OrbiterAPI.cpp:878-884 oapiGetWaveDrag
double wave_drag_impl(double M, double M1, double M2, double M3, double cmax) {
    if (M < M1) return 0.0;
    if (M < M2) return cmax * (M - M1) / (M2 - M1);
    if (M < M3) return cmax;
    return cmax * std::sqrt((M3 * M3 - 1.0) / (M * M - 1.0));
}

// OrbiterAPI.cpp:873-876 oapiGetInducedDrag + Rust A/eps guard
double induced_drag_impl(double cl, double A, double eps) {
    if (A <= 1e-9 || eps <= 1e-9) return 0.0;
    return (cl * cl) / (PI * A * eps);
}

double alpha_stall_mach_impl(double alpha_stall0, double mach) {
    if (mach < 0.8) return alpha_stall0;
    if (mach < 1.2) {
        double t = (mach - 0.8) / 0.4;
        return alpha_stall0 * (1.0 - 0.35 * t);
    }
    return alpha_stall0 * 0.65;
}

double grid_eta_impl(double mach) {
    if (mach < 0.8) return 1.0;
    if (mach < 1.5) return 1.0 - 0.4 * (mach - 0.8) / 0.7;
    return 0.6;
}

double slew_deploy_impl(double deploy, double deploy_target, double deploy_rate,
                        double dt) {
    double cmd = clamp01(deploy_target);
    if (deploy_rate <= 0.0) return cmd;
    double max_step = deploy_rate * dt;
    return clamp01(deploy + clamp(cmd - deploy, -max_step, max_step));
}

double weighted_area_impl(double ax, double ay, double az, Vec3d vhat) {
    return std::fabs(vhat.x) * ax + std::fabs(vhat.y) * ay + std::fabs(vhat.z) * az;
}

double side_area_impl(double ax, double ay, double az, Vec3d airvel) {
    (void)ay;
    double lat = std::sqrt(airvel.x * airvel.x + airvel.z * airvel.z);
    if (lat < 1e-12) return 0.5 * (ax + az);
    return (std::fabs(airvel.x) * ax + std::fabs(airvel.z) * az) / lat;
}

double cl_of_alpha(double alpha, double cl_alpha, double alpha_stall) {
    double a = std::fabs(alpha);
    double sign = alpha >= 0.0 ? 1.0 : -1.0;
    if (a <= alpha_stall) return cl_alpha * alpha;
    double cl_peak = cl_alpha * alpha_stall;
    double denom = std::max(PI * 0.5 - alpha_stall, 1e-6);
    double over = clamp((a - alpha_stall) / denom, 0.0, 1.0);
    return sign * cl_peak * std::max(1.0 - over, 0.0);
}

double fin_local_alpha_impl(Vec3d airvel, Vec3d normal, Vec3d chord_dir) {
    double nl = v3_length(normal);
    if (nl < 1e-12) return 0.0;
    Vec3d n = v3_scale(normal, 1.0 / nl);
    double clen = v3_length(chord_dir);
    if (clen < 1e-12) return 0.0;
    Vec3d c = v3_scale(chord_dir, 1.0 / clen);
    double v_c = v3_dot(airvel, c);
    double v_n = v3_dot(airvel, n);
    return std::atan2(v_n, -v_c);
}

Vec3d moment_about_cg_impl(Vec3d force, Vec3d point, Vec3d cg) {
    return v3_cross(force, v3_sub(point, cg));
}

void surface_force(const OxLiftingSurface *surf, Vec3d airvel, double q, double mach,
                   Vec3d *force_out, double *lift_out, double *drag_out) {
    double area = surf->area * clamp01(surf->deploy);
    if (area < 1e-12 || q < 1e-18) {
        force_out->x = force_out->y = force_out->z = 0.0;
        *lift_out = 0.0;
        *drag_out = 0.0;
        return;
    }
    double q_eff = surf->leeward_sheltered ? q * OX_LEEWARD_Q_FACTOR : q;

    Vec3d normal;
    normal.x = surf->normal_x;
    normal.y = surf->normal_y;
    normal.z = surf->normal_z;
    Vec3d chord;
    chord.x = surf->chord_x;
    chord.y = surf->chord_y;
    chord.z = surf->chord_z;
    double alpha = fin_local_alpha_impl(airvel, normal, chord);
    double a_stall = alpha_stall_mach_impl(surf->alpha_stall0, mach);
    double cl_a = surf->cl_alpha;
    if (surf->kind == OX_FIN_GRID) {
        cl_a *= grid_eta_impl(mach);
        double stall0 = std::max(surf->alpha_stall0, OX_DEFAULT_ALPHA_STALL_GRID);
        a_stall = alpha_stall_mach_impl(stall0, mach);
    }
    double cl = cl_of_alpha(alpha, cl_a, a_stall);
    double cd = surf->cd0
        + induced_drag_impl(cl, std::max(surf->aspect_ratio, 0.1), 0.7)
        + wave_drag_impl(mach, 0.75, 1.0, 1.1, 0.04);

    double nl = v3_length(normal);
    if (nl < 1e-12) {
        force_out->x = force_out->y = force_out->z = 0.0;
        *lift_out = 0.0;
        *drag_out = 0.0;
        return;
    }
    Vec3d n = v3_scale(normal, 1.0 / nl);
    double speed = v3_length(airvel);
    Vec3d vhat;
    vhat.x = vhat.y = vhat.z = 0.0;
    if (speed > 1e-12) {
        vhat = v3_scale(airvel, 1.0 / speed);
    }
    Vec3d ddir = v3_scale(vhat, -1.0);
    Vec3d n_lift = v3_sub(n, v3_scale(vhat, v3_dot(n, vhat)));
    double lift = cl * q_eff * area;
    double drag = cd * q_eff * area;
    *force_out = v3_add(v3_scale(n_lift, lift), v3_scale(ddir, drag));
    *lift_out = std::fabs(lift);
    *drag_out = drag;
}

void compute_body_aero_impl(
    Vec3d airvel_body, Vec3d omega_body, double rho, double sound_speed,
    double area_x, double area_y, double area_z,
    Vec3d body_cop, Vec3d cg,
    double cd0, double cn_alpha,
    double pitch_damp, double yaw_damp, double roll_damp,
    const double *cd_mach_m, const double *cd_mach_cd, int n_cd,
    OxAeroForces *out) {
    out->force_x = out->force_y = out->force_z = 0.0;
    out->torque_x = out->torque_y = out->torque_z = 0.0;
    out->mach = 0.0;
    out->dynamic_pressure = 0.0;
    out->drag_force = 0.0;
    out->cd_eff = 0.0;
    out->lift_force = 0.0;
    if (rho < 1e-15) return;
    double speed = v3_length(airvel_body);
    if (speed < 1e-6) return;
    double q = 0.5 * rho * speed * speed;
    out->dynamic_pressure = q;
    double a = std::max(sound_speed, 1.0);
    double mach = speed / a;
    out->mach = mach;

    Vec3d vhat = v3_scale(airvel_body, 1.0 / speed);

    double cd = piecewise_linear_cd(cd_mach_m, cd_mach_cd, n_cd, mach, cd0);
    double axial_w = std::fabs(vhat.y);
    double f_axial_mag = cd * q * area_y * axial_w;
    double f_y = -signum(vhat.y) * f_axial_mag;

    Vec3d v_lat;
    v_lat.x = airvel_body.x;
    v_lat.y = 0.0;
    v_lat.z = airvel_body.z;
    double v_lat_mag = v3_length(v_lat);
    double alpha = std::atan2(v_lat_mag, std::max(std::fabs(airvel_body.y), 1e-12));
    double cn = cn_alpha * alpha;
    // Slender-body CN_alpha is referenced to frontal disk area_y, not side area.
    double f_n_mag = cn * q * area_y;
    Vec3d f_lat;
    f_lat.x = f_lat.y = f_lat.z = 0.0;
    if (v_lat_mag > 1e-12) {
        f_lat = v3_scale(v_lat, -f_n_mag / v_lat_mag);
    }

    Vec3d force;
    force.x = f_lat.x;
    force.y = f_y;
    force.z = f_lat.z;
    out->force_x = force.x;
    out->force_y = force.y;
    out->force_z = force.z;
    out->drag_force = f_axial_mag;
    out->lift_force = f_n_mag;
    out->cd_eff = cd;
    Vec3d torque = moment_about_cg_impl(force, body_cop, cg);
    // Pitch ω_x → Sx (YZ); yaw ω_z → Sz (XY); roll → Sy.
    out->torque_x = torque.x - q * area_x * pitch_damp * omega_body.x;
    out->torque_y = torque.y - q * area_y * roll_damp * omega_body.y;
    out->torque_z = torque.z - q * area_z * yaw_damp * omega_body.z;
}

}  // namespace

extern "C" double ox_wave_drag(double mach, double m1, double m2, double m3,
                               double cmax) {
    return wave_drag_impl(mach, m1, m2, m3, cmax);
}

extern "C" double ox_induced_drag(double cl, double aspect_ratio, double oswald) {
    return induced_drag_impl(cl, aspect_ratio, oswald);
}

extern "C" double ox_alpha_stall_mach(double alpha_stall0, double mach) {
    return alpha_stall_mach_impl(alpha_stall0, mach);
}

extern "C" double ox_grid_eta(double mach) { return grid_eta_impl(mach); }

extern "C" double ox_slew_deploy(double deploy, double deploy_target,
                                 double deploy_rate, double dt) {
    return slew_deploy_impl(deploy, deploy_target, deploy_rate, dt);
}

extern "C" double ox_weighted_area(double area_x, double area_y, double area_z,
                                   double vx, double vy, double vz) {
    Vec3d vhat;
    vhat.x = vx;
    vhat.y = vy;
    vhat.z = vz;
    return weighted_area_impl(area_x, area_y, area_z, vhat);
}

extern "C" double ox_side_area(double area_x, double area_y, double area_z,
                               double avx, double avy, double avz) {
    Vec3d airvel;
    airvel.x = avx;
    airvel.y = avy;
    airvel.z = avz;
    return side_area_impl(area_x, area_y, area_z, airvel);
}

extern "C" double ox_fin_local_alpha(double avx, double avy, double avz,
                                     double nx, double ny, double nz,
                                     double cx, double cy, double cz) {
    Vec3d airvel;
    airvel.x = avx;
    airvel.y = avy;
    airvel.z = avz;
    Vec3d normal;
    normal.x = nx;
    normal.y = ny;
    normal.z = nz;
    Vec3d chord;
    chord.x = cx;
    chord.y = cy;
    chord.z = cz;
    return fin_local_alpha_impl(airvel, normal, chord);
}

extern "C" void ox_moment_about_cg(double fx, double fy, double fz,
                                   double px, double py, double pz,
                                   double cgx, double cgy, double cgz,
                                   double *tx, double *ty, double *tz) {
    Vec3d force;
    force.x = fx;
    force.y = fy;
    force.z = fz;
    Vec3d point;
    point.x = px;
    point.y = py;
    point.z = pz;
    Vec3d cg;
    cg.x = cgx;
    cg.y = cgy;
    cg.z = cgz;
    Vec3d t = moment_about_cg_impl(force, point, cg);
    *tx = t.x;
    *ty = t.y;
    *tz = t.z;
}

extern "C" void ox_compute_body_aero(
    double avx, double avy, double avz,
    double wx, double wy, double wz,
    double rho, double sound_speed,
    double area_x, double area_y, double area_z,
    double copx, double copy, double copz,
    double cgx, double cgy, double cgz,
    double cd0, double cn_alpha,
    double pitch_damp, double yaw_damp, double roll_damp,
    const double *cd_mach_m, const double *cd_mach_cd, int n_cd,
    OxAeroForces *out) {
    Vec3d airvel;
    airvel.x = avx;
    airvel.y = avy;
    airvel.z = avz;
    Vec3d omega;
    omega.x = wx;
    omega.y = wy;
    omega.z = wz;
    Vec3d cop;
    cop.x = copx;
    cop.y = copy;
    cop.z = copz;
    Vec3d cg;
    cg.x = cgx;
    cg.y = cgy;
    cg.z = cgz;
    compute_body_aero_impl(airvel, omega, rho, sound_speed, area_x, area_y, area_z,
                           cop, cg, cd0, cn_alpha, pitch_damp, yaw_damp, roll_damp,
                           cd_mach_m, cd_mach_cd, n_cd, out);
}

extern "C" void ox_compute_rocket_aero(
    double avx, double avy, double avz,
    double wx, double wy, double wz,
    double rho, double sound_speed,
    double area_x, double area_y, double area_z,
    double copx, double copy, double copz,
    double cgx, double cgy, double cgz,
    double cd0, double cn_alpha,
    double pitch_damp, double yaw_damp, double roll_damp,
    const double *cd_mach_m, const double *cd_mach_cd, int n_cd,
    const OxLiftingSurface *surfaces, int n_surf,
    OxAeroForces *out) {
    Vec3d airvel;
    airvel.x = avx;
    airvel.y = avy;
    airvel.z = avz;
    Vec3d omega;
    omega.x = wx;
    omega.y = wy;
    omega.z = wz;
    Vec3d cop;
    cop.x = copx;
    cop.y = copy;
    cop.z = copz;
    Vec3d cg;
    cg.x = cgx;
    cg.y = cgy;
    cg.z = cgz;
    compute_body_aero_impl(airvel, omega, rho, sound_speed, area_x, area_y, area_z,
                           cop, cg, cd0, cn_alpha, pitch_damp, yaw_damp, roll_damp,
                           cd_mach_m, cd_mach_cd, n_cd, out);
    if (rho < 1e-15) return;
    double speed = v3_length(airvel);
    if (speed < 1e-6) return;
    double q = out->dynamic_pressure;
    double mach = out->mach;
    double lift_sum = out->lift_force;
    double drag_sum = out->drag_force;
    for (int i = 0; i < n_surf; ++i) {
        Vec3d f;
        double lift = 0.0, drag = 0.0;
        Vec3d ref;
        ref.x = surfaces[i].ref_pos_x;
        ref.y = surfaces[i].ref_pos_y;
        ref.z = surfaces[i].ref_pos_z;
        Vec3d v_pt = v3_add(airvel, v3_cross(omega, v3_sub(ref, cg)));
        surface_force(&surfaces[i], v_pt, q, mach, &f, &lift, &drag);
        out->force_x += f.x;
        out->force_y += f.y;
        out->force_z += f.z;
        Vec3d t = moment_about_cg_impl(f, ref, cg);
        out->torque_x += t.x;
        out->torque_y += t.y;
        out->torque_z += t.z;
        lift_sum += lift;
        drag_sum += drag;
    }
    out->lift_force = lift_sum;
    out->drag_force = drag_sum;
}
