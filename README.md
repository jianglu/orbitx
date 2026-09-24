# orbitx

A cross-platform space-flight simulation engine, derived from the [Orbiter Space Flight
Simulator](https://github.com/orbitersim/orbiter) and rewritten in Rust.

This project does not use Orbiter source code directly. Instead, it re-implements the
physics, mathematics, and ephemeris subsystems from the published technical reference,
validating correctness against the original C++ implementation via FFI property tests.

**Product target** (SimRocket): Godot is a front-end; orbitx runs as an independent process
with a **Runtime** (clock + stepping) and **Controller**. See
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md). Workspace IPC boundaries:
[`../AGENTS.md`](../AGENTS.md).

## Coordinate system

orbitx preserves Orbiter's **left-handed** ecliptic J2000 frame (`ẑ = ŷ × x̂`) so that
legacy scenario data, mesh assets, and ephemeris series can be consumed without
conversion. The handedness is hard-coded (no runtime switch) and isolated to the math
layer; the graphics layer applies the left-handed projection at its boundary.

## Project layout

```
crates/
├── orbitx-math/           Vec3/Matrix3/Quaternion/Astro ✅
├── orbitx-math-ffi/       C++ oracle for property tests
├── orbitx-dynamics/       Gravity, Pines, RK/SY, rigid body, planetary ✅
├── orbitx-dynamics-ffi/   C++ oracle for property tests
├── orbitx-ephemeris/      VSOP87, ELP82, TASS17, GALSAT ✅
├── orbitx-ephemeris-ffi/  C++ oracle for property tests
├── orbitx-vessel/         Multi-stage, aero, RCS, touchdown primitives, fuel 🟡
├── orbitx-config/         TOML body/system/rocket/scenario 🟡
├── orbitx-cli/            Terminal UI launch (control logic → migrates to controller)
├── orbitx-app/            Local wgpu GUI viewer (not product host; name kept)
├── orbitx-controller/     Control strategies (Base / Target / WorkFlow) 🟡 skeleton (P4.1 阶段 A)
├── orbitx-demo-aero/      Atmospheric reentry demo
├── orbitx-demo-landing/   Touchdown demo (forces applied outside Assembly step)
├── orbitx-demo-orrery/    Solar system body config viewer
├── orbitx-flight/         Legacy kiss3d viewer (parked; outside P4.1–P4.3)
├── orbitx-launch/         Legacy launch app (parked; outside P4.1–P4.3)
├── orbitx-scene/          3-D scene graph
└── orbitx-orrery/         Solar-system orrery

Planned: orbitx-runtime (product main: IPC + Runtime, no GUI); orbitx-controller skeleton in place (P4.1 阶段 A)
```

## Verification strategy

Every core numerical algorithm is verified against the original Orbiter C++ implementation
via **FFI property tests** (proptest). The C++ oracle re-implements each algorithm as a
free function (verbatim copy from Orbiter source), compiled into the test binary. Rust and
C++ results are compared to ~1e-10 relative tolerance.

| Module | Tests | Coverage |
|--------|------:|----------|
| `orbitx-math` | 18 | Vec3, Matrix3, Quat, Astro constants |
| `orbitx-dynamics` | 34 | Gravity (point-mass, J2, Pines), Euler equations, RK2/4/5/8, SY2/4/6/8, rotation, planetary |
| `orbitx-ephemeris` | 7 | VSOP87 (Earth), ELP82 (Moon), TASS17 (Saturn moons), GALSAT (Jupiter moons) |
| `orbitx-vessel` | 64 | Multi-stage assembly, TVC, aerodynamics, RCS, touchdown, fuel, determinism |
| `orbitx-config` | 17 | BodyConfig defaults, TOML roundtrip, SystemConfig |

## Current status

### ✅ Complete — Math, Dynamics, Ephemeris, Celestial Bodies

- **Math library**: Full Vec3/Matrix3/Quaternion + astro constants, symbol-by-symbol verified
- **Dynamics**: Point-mass & J2/J3/J4 gravity (with body rotation), Pines spherical harmonics,
  rigid-body Euler equations, all 10 integrators (RK2–RK8, SY2–SY8), TVC closed-loop control
- **Ephemeris**: VSOP87 (Earth), ELP82 (Moon), TASS17 (8 Saturn moons), GALSAT/Lieske
  (4 Galilean moons) — including the Jupiter–Saturn great-inequality correction (`revizg_`)
- **Celestial bodies**: PlanetarySystem multi-body container, rotation + precession model
  (Celbody.cpp port), J-coeff with body-frame rotation, Pines perturbation branch,
  BodyConfig with Orbiter-quality defaults (14 bodies)

### 🟡 Partial — Vessel, Config, Product host

- **Vessel** (~39% of Orbiter's `Vessel.cpp`):
  - ✅ Multi-stage rocket rigid body, Assembly, TVC gimbal control
  - ✅ Aerodynamics integrated into `Assembly::step`
  - ✅ RCS: default layout + thruster groups + attitude API
  - 🟡 Touchdown: spring-damper-friction **primitives** exist; **not** yet in `Assembly::step`;
    shared elevation + in-loop contact is ROADMAP **P5**
  - ✅ Fuel: multi-tank + thruster↔tank association
  - ✅ Lateral / hard dock SuperVessel subset (CZ-2F; see ROADMAP P1.4)
  - ❌ Full dock-tree split, SoftDock, Attachment, Isp pressure correction (P1.4b–e)
- **Config**: TOML body/system/rocket/scenario — see [`docs/CONFIG_TOML.md`](docs/CONFIG_TOML.md)
- **Runtime / Controller**: not built yet — product host crate **`orbitx-runtime`**
  ([`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md), ROADMAP **P4**); existing **`orbitx-app`** stays the local GUI
- **Local GUI**: `orbitx-app` wgpu viewer works; **`UserVessel`** is a **non-authoritative** bypass (removal **outside P4.1–P4.3**)

## Demos

| Demo | Run | Description |
|------|-----|-------------|
| **Main app (GUI)** | `cargo run -p orbitx-app` | Local wgpu viewer (**not** `orbitx-runtime`) |
| **CLI launch** | `cargo run -p orbitx-cli` | Terminal UI Falcon 9 / Saturn V; control → future controller |
| **Aero reentry** | `cargo run -p orbitx-demo-aero` | Atmospheric reentry with aero vs no-aero comparison |
| **Landing** | `cargo run -p orbitx-demo-landing` | Soft/hard landing (forces outside Assembly; P5 will in-loop) |
| **Orrery** | `cargo run -p orbitx-demo-orrery` | Solar system body config viewer (14 bodies) |
| **3-D flight** | `cargo run -p orbitx-flight` | kiss3d orbital flight viewer (legacy) |

## Running the local GUI (`orbitx-app`)

```bash
cargo run -p orbitx-app        # local wgpu viewer (product host will be orbitx-runtime)
```

主 app 提供：太阳系 14 天体（历表驱动 + 纹理球 + 大气层 + 土星环 + 地球云层）
+ LEO 用户飞船（**简化** `UserVessel` 传播，非 Assembly 权威）
+ HUD/MFD + 相机模式。详见 [`docs/RENDERING.md`](docs/RENDERING.md)、[`docs/KEYBINDINGS.md`](docs/KEYBINDINGS.md)。

**TOML 配置** — [`docs/CONFIG_TOML.md`](docs/CONFIG_TOML.md)。

## Roadmap

See [`docs/ROADMAP.md`](docs/ROADMAP.md) and [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md):

```
P0 闭合测试缺口              ✅ Done
P1 航天器物理                🟡 主能力 Done；触点入环 / P1.4b–e 后续
P2 天体/场景完整性            ✅ Done
P3 本地渲染 `orbitx-app`     🟡 可用；产品主进程为 `orbitx-runtime`（P4）
P4 Controller→Runtime→CLI↔Zenoh→Godot  🟡 **P4.1 阶段 A 完成**（骨架 + 设计文档）；阶段 B → P4.2 → P4.3 → P4.4
P5 共用高程地表 + 近距级间碰撞  🔲（羽流撞击本期不做）
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
