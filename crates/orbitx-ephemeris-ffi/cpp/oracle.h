// Stub header for the ephemeris FFI oracle.
//
// Replaces OrbiterAPI.h and CelBodyAPI.h with just the definitions the VSOP87
// and ELP82 algorithms need, without any windows.h or engine dependencies.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdint>

// --- Types (from OrbiterAPI.h, minimal subset) ---
using OBJHANDLE = void *;
using FILEHANDLE = void *;
using DWORD = uint32_t;

// --- EPHEM bitflags (from CelBodyAPI.h:32-38) ---
#define EPHEM_TRUEPOS     0x01
#define EPHEM_TRUEVEL     0x02
#define EPHEM_BARYPOS     0x04
#define EPHEM_BARYVEL     0x08
#define EPHEM_BARYISTRUE  0x10
#define EPHEM_PARENTBARY  0x20
#define EPHEM_POLAR       0x40

// --- Sample struct (from CelBodyAPI.h:42-46) ---
struct Sample {
    double t;
    double rad;
    double param[6];
};

// --- Math/physics constants (from OrbiterAPI.h / Vsop87.cpp / ELP82.cpp) ---
static const double PI   = 3.14159265358979323846;
static const double PI05 = 1.57079632679489661923;
static const double PI2  = 6.28318530717958647693;
static const double RAD  = PI / 180.0;
static const double DEG  = 180.0 / PI;
static const double C0   = 299792458.0;
static const double TAUA = 499.004783806;
static const double AU   = C0 * TAUA;
static const double GGRAV = 6.67259e-11;

// --- API stubs ---
// oapiTime2MJD: MJD_ref + t/86400. MJD_ref defaults to 51544.5 (J2000).
// Declared as extern here; defined in shim.cpp.
extern double ox_MJD_ref;
inline double oapiTime2MJD(double t) { return ox_MJD_ref + t / 86400.0; }

// No-op stubs for logging/config (never called by the evaluation algorithms).
inline void oapiWriteLogV(const char *fmt, ...) {}
inline void oapiWriteLogError(const char *fmt, ...) {}
inline bool oapiReadItem_float(FILEHANDLE cfg, const char *tag, double &val) { return false; }
