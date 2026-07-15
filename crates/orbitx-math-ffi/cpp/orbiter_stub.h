// Minimal stub replacing OrbiterAPI.h for FFI test compilation.
//
// Astro.h includes OrbiterAPI.h for just a handful of constants (AU, C0, TAUA,
// GGRAV). The real OrbiterAPI.h pulls in <windows.h>, which is unavailable on
// macOS/Linux. This stub provides only what Vecmat.cpp and Astro.cpp need.
#pragma once

// --- Physical constants (copied from OrbiterAPI.h:69-74) ---
static const double C0    = 299792458.0;   // speed of light [m/s]
static const double TAUA  = 499.004783806; // light time for 1 AU [s]
static const double AU    = C0 * TAUA;     // astronomical unit [m]
static const double GGRAV = 6.67259e-11;   // gravitational constant
static const double G     = 9.81;
static const double ATMP  = 101.4e3;
static const double ATMD  = 1.293;
