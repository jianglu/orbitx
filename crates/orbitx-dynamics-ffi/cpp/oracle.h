// Stub header for the dynamics FFI oracle.
//
// Replaces OrbiterAPI.h/Astro.h with minimal definitions for the dynamics
// algorithms, without any windows.h or engine dependencies.
#pragma once

#include <cmath>
#include <cstdint>

// --- Physical constant ---
static const double GGRAV = 6.67259e-11;

// --- Minimal Vector/StateVectors for the oracle ---
struct Vec3d { double x, y, z; };

inline Vec3d v3_add(Vec3d a, Vec3d b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3d v3_sub(Vec3d a, Vec3d b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3d v3_scale(Vec3d a, double s) { return {a.x*s, a.y*s, a.z*s}; }
inline double v3_dot(Vec3d a, Vec3d b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3d v3_cross(Vec3d a, Vec3d b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline double v3_length(Vec3d a) { return sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }
inline double v3_length2(Vec3d a) { return a.x*a.x + a.y*a.y + a.z*a.z; }
inline Vec3d v3_unit(Vec3d a) { double d = v3_length(a); return {a.x/d, a.y/d, a.z/d}; }

static const double PI = 3.14159265358979323846;
static const double PI2 = 2.0 * PI;

// --- Product rocket aero (oracle for AERO.md / rocket.rs) ---
// wave/induced: OrbiterAPI.cpp; body+fin synthesis: second impl of rocket.rs.

static const double OX_LEEWARD_Q_FACTOR = 0.5;
static const double OX_DEFAULT_ALPHA_STALL_GRID = 28.0 * PI / 180.0;

enum OxFinKind { OX_FIN_FIXED = 0, OX_FIN_GRID = 1 };

struct OxLiftingSurface {
    double ref_pos_x, ref_pos_y, ref_pos_z;
    double normal_x, normal_y, normal_z;
    double chord_x, chord_y, chord_z;
    double area;
    double aspect_ratio;
    double cl_alpha;
    double cd0;
    double alpha_stall0;
    int kind;                 // OxFinKind
    double deploy;
    int leeward_sheltered;    // 0/1
};

struct OxAeroForces {
    double force_x, force_y, force_z;
    double torque_x, torque_y, torque_z;
    double mach;
    double dynamic_pressure;
    double drag_force;
    double cd_eff;
    double lift_force;
};
