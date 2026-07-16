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
├── orbitx-math/           Vec3/Matrix3/Quaternion/Astro (2,218 行) ✅
├── orbitx-math-ffi/       C++ oracle for property tests
├── orbitx-dynamics/       Gravity, Pines harmonics, RK/SY integrators, rigid body (2,130 行) ✅
├── orbitx-dynamics-ffi/   C++ oracle for property tests
├── orbitx-ephemeris/      VSOP87, ELP82, TASS17, GALSAT/Lieske (2,629 行) ✅
├── orbitx-ephemeris-ffi/  C++ oracle for property tests
├── orbitx-vessel/         Multi-stage rocket, Assembly, TVC (1,614 行) 🟡
├── orbitx-config/         TOML scenario/vehicle config (479 行) 🟡
├── orbitx-cli/            Terminal UI launch simulator (1,154 行)
├── orbitx-flight/         kiss3d 3-D flight viewer (593 行)
├── orbitx-launch/         Legacy launch app (641 行)
├── orbitx-scene/          3-D scene graph (528 行)
└── orbitx-orrery/         Solar-system orrery (184 行)
```

## Verification strategy

Every core numerical algorithm is verified against the original Orbiter C++ implementation
via **FFI property tests** (proptest). The C++ oracle re-implements each algorithm as a
free function (verbatim copy from Orbiter source), compiled into the test binary. Rust and
C++ results are compared to ~1e-10 relative tolerance.

| Module | Tests | Coverage |
|--------|------:|----------|
| `orbitx-math` | 18 | Vec3, Matrix3, Quat, Astro constants |
| `orbitx-dynamics` | 20 | Gravity (point-mass, J2, Pines), Euler equations, RK2/4/5/8, SY2/4/6/8 |
| `orbitx-ephemeris` | 7 | VSOP87 (Earth), ELP82 (Moon), TASS17 (Saturn moons), GALSAT (Jupiter moons) |

## Current status

### ✅ Complete — Math, Dynamics, Ephemeris

- **Math library**: Full Vec3/Matrix3/Quaternion + astro constants, symbol-by-symbol verified
- **Dynamics**: Point-mass & J2/J3/J4 gravity, Pines spherical harmonics, rigid-body Euler
  equations, all 10 integrators (RK2–RK8, SY2–SY8), TVC closed-loop control
- **Ephemeris**: VSOP87 (Earth), ELP82 (Moon), TASS17 (8 Saturn moons), GALSAT/Lieske
  (4 Galilean moons) — including the Jupiter–Saturn great-inequality correction (`revizg_`)

### 🟡 Partial — Vessel, Config

- **Vessel**: Multi-stage rocket rigid body (~10% of Orbiter's `Vessel.cpp`); missing
  aerodynamics, RCS, touchdown points, fuel crossfeed
- **Config**: TOML-based, not compatible with Orbiter's `.cfg`/`.scn` format

### 🔴 Skeleton — Rendering/UI

- kiss3d placeholder + ratatui TUI; MFD, mesh/texture, elevation LOD not yet started

## Roadmap

See [`docs/ROADMAP.md`](docs/ROADMAP.md) for the full migration roadmap and priority order:

```
P0 闭合测试缺口        ✅ Done
P1 航天器物理          ← Next (aerodynamics, RCS, touchdown, fuel)
P2 天体/场景完整性
P3 渲染/UI
P4 架构整合
```

## Building

```bash
cargo build
cargo test -p orbitx-math -p orbitx-dynamics -p orbitx-ephemeris
```

The FFI oracle tests require the Orbiter data files (VSOP87, ELP82, TASS17, GALSAT) at
the sibling path `../orbiter/Src/Celbody/`. Set `ORBITER_SRC` if the path differs.

## License

MIT.
