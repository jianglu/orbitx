// Drop-in replacement for OrbiterAPI.h that provides ONLY the constants Astro.h
// and Vecmat.h reference, without pulling in <windows.h>. build.rs places this
// directory first on the include path so it shadows the real OrbiterAPI.h.
#pragma once
#include "orbiter_stub.h"

// --- Math constants (copied from OrbiterAPI.h:64-68) ---
// Astro.cpp uses the uppercase variants; Vecmat.h defines the lowercase ones.
static const double PI   = 3.14159265358979323846;
static const double PI05 = 1.57079632679489661923;
static const double PI2  = 6.28318530717958647693;
static const double RAD  = PI / 180.0;
static const double DEG  = 180.0 / PI;
