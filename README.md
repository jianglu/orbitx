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
├── orbitx-vessel/         Multi-stage rocket, aero, RCS, landing, fuel (3,558 行) 🟡
├── orbitx-config/         TOML scenario/vehicle config (479 行) 🟡
├── orbitx-cli/            Terminal UI launch simulator (1,138 行)
├── orbitx-demo-aero/      Atmospheric reentry demo (217 行)
├── orbitx-demo-landing/   Touchdown/landing demo (430 行)
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
| `orbitx-vessel` | 64 | Multi-stage assembly, TVC, aerodynamics, RCS, touchdown, fuel, determinism |

## Current status

### ✅ Complete — Math, Dynamics, Ephemeris

- **Math library**: Full Vec3/Matrix3/Quaternion + astro constants, symbol-by-symbol verified
- **Dynamics**: Point-mass & J2/J3/J4 gravity, Pines spherical harmonics, rigid-body Euler
  equations, all 10 integrators (RK2–RK8, SY2–SY8), TVC closed-loop control
- **Ephemeris**: VSOP87 (Earth), ELP82 (Moon), TASS17 (8 Saturn moons), GALSAT/Lieske
  (4 Galilean moons) — including the Jupiter–Saturn great-inequality correction (`revizg_`)

### 🟡 Partial — Vessel, Config

- **Vessel** (~39% of Orbiter's `Vessel.cpp`):
  - ✅ Multi-stage rocket rigid body, Assembly, TVC gimbal control
  - ✅ Aerodynamics: airfoil (constant/linear/table CL/CD), control surfaces, drag elements,
    aero damping, exponential atmosphere model — integrated into Assembly physics step
  - ✅ RCS: 12-thruster default思考已达上限，停止思考，开始回答。</think><tool_call>Write<arg_key>content</arg_key><arg_value># orbitx

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
├── orbitx-vessel/         Multi-stage rocket, aero, RCS, landing, fuel (3,558 行) 🟡
├── orbitx-config/         TOML scenario/vehicle config (479 行) 🟡
├── orbitx-cli/            Terminal UI launch simulator (1,138 行)
├── orbitx-demo-aero/      Atmospheric reentry demo (217 行)
├── orbitx-demo-landing/   Touchdown/landing demo (430 行)
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
| `orbitx-vessel` | 64 | Multi-stage assembly, TVC, aerodynamics, RCS, touchdown, fuel, determinism |

## Current status

### ✅ Complete — Math, Dynamics, Ephemeris

- **Math library**: Full Vec3/Matrix3/Quaternion + astro constants, symbol-by-symbol verified
- **Dynamics**: Point-mass & J2/J3/J4 gravity, Pines spherical harmonics, rigid-body Euler
  equations, all 10 integrators (RK2–RK8, SY2–SY8), TVC closed-loop control
- **Ephemeris**: VSOP87 (Earth), ELP82 (Moon), TASS17 (8 Saturn moons), GALSAT/Lieske
  (4 Galilean moons) — including the Jupiter–Saturn great-inequality correction (`revizg_`)

### 🟡 Partial — Vessel, Config

- **Vessel** (~39% of Orbiter's `Vessel.cpp`):
  - ✅ Multi-stage rocket rigid body, Assembly, TVC gimbal control
  - ✅ Aerodynamics: airfoil (constant/linear/table CL/CD), control surfaces, drag elements,
    aero damping, exponential atmosphere model — integrated into Assembly physics step
  - ✅ RCS: 12-thruster default layout, 15 standard thruster groups (THGROUP_MAIN .. ATT_BACK),
    attitude rotation / translation control API
  - ✅ Touchdown: spring-damper-friction contact model (3+ touchdown vertices),
    force limiting to prevent velocity reversal, landing gear helper
  - ✅ Fuel: multi-tank PropellantTank with thruster↔tank association,
    backward-compatible single `fuel_mass` path
  - ❌ General dock tree (SuperVessel arbitrary assembly), Isp pressure correction
- **Config**: TOML-based, not compatible with Orbiter's `.cfg`/`.scn` format

### 🔴 Skeleton — Rendering/UI

- kiss3d placeholder + ratatui TUI; MFD, mesh/texture, elevation LOD not yet started

## Demos

| Demo | Run | Description |
|------|-----|-------------|
| **CLI launch** | `cargo run -p orbitx-cli` | Terminal UI Falcon 9 / Saturn V launch with gravity turn |
| **Aero reentry** | `cargo run -p orbitx-demo-aero` | Atmospheric reentry with aero vs no-aero comparison |
| **Landing** | `cargo run -p orbitx-demo-landing` | Soft/hard landing with spring-damper touchdown forces |
| **3-D flight** | `cargo run -p orbitx-flight` | kiss3d orbital flight viewer |

## Roadmap

See [`docs/ROADMAP.md`](docs/ROADMAP.md) for the full migration roadmap and priority order:

```
P0 闭合测试缺口        ✅ Done
P1 航天器物理          ✅ Done (aerodynamics, RCS, touchdown, fuel)
P2 天体/场景完整性      ← Next (planet config, multi-body, rotation, J2/J4)
P3 渲染/UI
P4 架构整合
```

## Building

```bash
cargo build
cargo test -p orbitx-math -p orbitx-dynamics -p orbitx-ephemeris -p orbitx-vessel
```

The FFI oracle tests require the Orbiter data files (VSOP87, ELP82, TASS17, GALSAT) at
the sibling path `../orbiter/Src/Celbody/`. Set `ORBITER_SRC` if the path differs.

## License

MIT.
