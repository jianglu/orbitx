# orbitx

A cross-platform space-flight simulation engine, derived from the [Orbiter Space Flight
Simulator](https://github.com/orbitersim/orbiter) and rewritten in Rust.

This project does not use Orbiter source code directly. Instead, it re-implements the
physics, mathematics, and ephemeris subsystems from the published technical reference,
validating correctness against the original C++ implementation via FFI property tests.

## Coordinate system

orbitx preserves Orbiter's **left-handed** ecliptic J2000 frame (`ẑ = ŷ × x̂`) so that
legacy scenario data, mesh assets, and ephemeris series can be consumed without
conversion. The handedness is hard-coded (no runtime switch) and isolated to the math
layer; the graphics layer applies the left-handed projection at its boundary.

## Project layout

```
crates/
├── orbitx-math/        Phase 1: Vec3/Matrix3/Quaternion/Astro (this crate)
├── orbitx-math-ffi/    C++ oracle compiled via FFI for property tests
```

## Status

Phase 1 — Math library + FFI test infrastructure. See `CLAUDE.md` / project plan.

## License

MIT.
