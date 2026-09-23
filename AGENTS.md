# AGENTS — orbitx/

Rust 重写的航天飞行仿真引擎局部约束。**工作区架构边界、Godot / zenoh 集成、禁止 path-link 进展示端**以根 [`AGENTS.md`](../AGENTS.md) 为准；本文件仅描述 `orbitx/` 子树的目录结构、crate 分层、开发约束与技术约定。

进度、demo 清单与运行命令见 [`README.md`](README.md)；移植与 P4/P5 排期见 [`docs/ROADMAP.md`](docs/ROADMAP.md)；**产品架构（Runtime / Controller / 接触）**见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。本文件不复述完成度百分比或教程步骤。

orbitx 按 Orbiter **技术参考**重写物理 / 数学 / 历表；目标是以 **独立仿真进程**（app 持有 **Runtime**）经 zenoh 服务 `sim-rocket`。`orbiter/` 仅对照与 FFI oracle，**不是**运行时依赖。本树为独立 git / 独立 Cargo workspace（edition 2021，MSRV 1.75），勿与 `sim-rocket/rust` 混用同一工具链假设。

---

## 目录结构

```text
orbitx/
├── AGENTS.md              # 本文件
├── README.md              # 完成度、验证策略、demo / 构建入口
├── Cargo.toml             # workspace 根
├── rust-toolchain.toml    # stable + rustfmt / clippy
├── clippy.toml / rustfmt.toml
├── crates/                # workspace members（见分层）
├── docs/                  # 本子树技术文档
│   ├── ARCHITECTURE.md    # Runtime / Controller / 接触（产品闭环）
│   ├── ROADMAP.md
│   ├── RENDERING.md
│   ├── CONFIG_TOML.md
│   ├── KEYBINDINGS.md
│   └── ORBITER_QUIRKS.md
├── assets/                # 运行时资源
│   ├── keybindings.toml
│   ├── orbiter-data/      # 捆绑历表数据
│   └── textures/
└── .github/               # CI
```

---

## Crate 分层

依赖方向单向：下层不依赖上层；`*-ffi` 仅被对应 crate 的**测试**使用，不作产品运行时依赖。

```text
orbitx-math
    ├── orbitx-dynamics ← orbitx-dynamics-ffi（测试）
    ├── orbitx-ephemeris ← orbitx-ephemeris-ffi（测试）
    ├── orbitx-vessel  ← orbitx-dynamics, orbitx-config
    ├── orbitx-config
    └── orbitx-render
            └── orbitx-app（本地 GUI，非产品主进程）

规划中：
  orbitx-controller ← orbitx-vessel
  orbitx-runtime    ← vessel, dynamics, controller
       └── bin：产品主进程（通信 + Runtime 编排，无 GUI）← zenoh

现有：
  orbitx-app ← render, gfx-hud, …   # 本地 wgpu GUI，保留原名
```

| 层 | Crate | 职责 |
|----|-------|------|
| 数学 | `orbitx-math` | Vec3 / Matrix3 / Quaternion / Astro；**左手** ecliptic J2000 |
| 物理 | `orbitx-dynamics` | 引力、Pines、积分器、刚体、旋转、多体容器 |
| 历表 | `orbitx-ephemeris` | VSOP87、ELP82、TASS17、GALSAT |
| 航天器 | `orbitx-vessel` | 多级装配、气动、RCS、着陆触点、燃料、（规划）级间代理碰撞 |
| 配置 | `orbitx-config` | 原生 TOML（body / system / rocket / scenario）；**非** Orbiter `.cfg` / `.scn` |
| 控制 | `orbitx-controller`（规划） | 四档控制 / Base·Target·WorkFlow；见 ARCHITECTURE |
| 运行时 / 主程序 | `orbitx-runtime`（规划） | 时钟、步进编排、切片、input、zenoh；**产品主进程**（无 GUI） |
| 渲染桥 | `orbitx-render` | f64→f32 `CoordinateBridge`、相机、场景图 |
| HUD | `orbitx-gfx-hud` | egui HUD / MFD |
| 本地 GUI | `orbitx-app`（现有） | winit + wgpu + egui 可视化；**非**产品主进程，名称保留以免与 `orbitx-runtime` 冲突 |
| Oracle | `orbitx-math-ffi` / `dynamics-ffi` / `ephemeris-ffi` | C++ oracle，仅测试 |
| 遗留 / 演示 | `orbitx-cli`；`demo-*`；`flight` / `launch`（kiss3d 暂搁） | **P4.1–P4.2 只建 crate**；**P4.3** cli↔zenoh；demo 不强制；flight/launch 另开清理 |

**Controller**：上层业务下发的自动控制（油门组合、分离时序、GNC、工作流）。依赖 `orbitx-vessel` 原语，不反向依赖。当前过渡实现于 `orbitx-cli`（`control` 模块）。四档与类层次见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

**客户端约定**：**P4.1** 建 `orbitx-controller`、**P4.2** 建 `orbitx-runtime`（均不改 cli）；**P4.3** 起 `orbitx-cli` 经 **zenoh** 连 Runtime（与 Godot 同形态）。禁止新产品路径进程内直调 `Assembly`。`demo-*` 不强制。`flight` / `launch` 暂搁。`UserVessel` 废除另排。

---

## 开发约束

### 单一职责（crate / 模块）

- 每个 crate 只承担上表一行职责；新能力先判断归属层，再落文件，禁止「方便起见」塞进邻近 crate。
- crate 内按主题拆模块（如 `vessel` 的 `aero` / `rcs` / `fuel` / `touchdown`）；同类逻辑集中一处，抽公共类型 / 函数，禁止碎片化复制。
- 命名与可见性清晰：对外 API 稳定，内部细节 `pub(crate)` 或私有；避免跨 crate 泄漏实现细节。

### 架构优先，禁止补丁式改动

- 功能延伸、问题修复、移植缺口：**禁止**在调用方 / demo / `orbitx-app` 上打局部补丁绕过权威层。
- 应在正确 crate 内按现有分层实现（气动 → `orbitx-vessel`，积分器 → `orbitx-dynamics`，坐标转换 → `orbitx-render`）。
- 发现重复物理或配置逻辑时，收敛到权威 crate，而不是再加第三份拷贝。

### 结构清晰

- 保持依赖图单向、可测试：核心数值逻辑不拖 wgpu / egui；渲染 / HUD 不内嵌步进权威。
- 配置解析只在 `orbitx-config`；积木步进在 `dynamics` / `vessel`；**时钟与对外步进入口在 Runtime**；帧循环与 GPU 只在展示 crate。
- 改动触及边界时，先核对本文件分层表与 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)，再改代码；跨层新依赖须有明确理由并更新本文件。

### 编码一致性

- 遵循 Rust 惯例与 workspace `clippy` / `rustfmt`；与邻近模块风格一致。
- 公开 API 与错误路径可读；数值路径优先可复现（固定步长等既有约定）。

### 单元测试与改动验证

**文件结构（统一）**

单元测试放在**各自模块目录内的独立文件**，由实现侧用 `mod` 引入，禁止在实现文件底部内联大块 `mod tests { … }`（新代码与改动触及的模块须按此结构；历史内联随改动迁出）。

```text
src/<module>/
├── mod.rs       # 实现（原 <module>.rs 迁入）
└── tests.rs     # 单元测试
```

在 `mod.rs`（或该模块根）末尾：

```rust
#[cfg(test)]
mod tests;
```

- `tests.rs` 与实现同目录，仅含测试；可 `use super::*` 访问模块私有项。
- crate 级 / 跨模块 / FFI oracle 仍放在 `crates/<name>/tests/`（如 `ffi_oracle.rs`），与模块内单元测试分工：模块测局部不变量，`tests/` 测对外契约与对照。
- 单文件模块若尚未拆目录，改测试时一并改为 `src/<module>/{mod.rs,tests.rs}`，避免 `#[path = …]` 特例。

**覆盖范围**

- 有逻辑的 crate / 模块须具备可运行的单元测试，覆盖对外行为与关键不变量。
- 核心数值层（`orbitx-math` / `dynamics` / `ephemeris` / `vessel` / `config`）：单元测试与既有 FFI / proptest 路径一并维护；改数值算法时优先扩展对应测试，而非只靠手工跑 app。
- 可抽离的纯逻辑（如 `orbitx-render` 的坐标桥、`orbitx-app` 的 `flight_calc` / 输入映射）：测逻辑本身，不强制 GPU / 窗口集成测试。
- 遗留 demo / 纯展示入口：新增业务逻辑仍须测试；仅接线、无新算法时可依赖所调用权威 crate 的测试。

**与改动绑定的验证流程**

1. 改代码前确认触及模块已有测试；若缺口，**同改动内先补测试再改实现**（或先写失败用例再实现）。
2. 改动完成后在 `orbitx/` 下对受影响 crate 执行 `cargo test -p <crate>…`（核心数值改动至少覆盖 `orbitx-math` / `dynamics` / `ephemeris` / `vessel` / `config` 中被改者）。
3. **以测试结果为正确性门禁**：全部通过才视为改动完成；失败则修实现或测试，禁止未通过即结束。
4. 交付说明中须指出跑过哪些 `-p` 以及结果（通过 / 失败原因）。

---

## 技术栈

| 用途 | 选型 |
|------|------|
| 语言 / workspace | Rust 2021，stable，MSRV 1.75；`clippy` + `rustfmt` |
| 仿真数学 | `orbitx-math`（f64）；GPU 侧 `glam` f32 |
| 配置 | `serde` + `toml` |
| 验证 | `proptest` + `*-ffi`（`cc` 编 C++ oracle） |
| 主 UI 图形 | `wgpu` 29 + `winit` 0.30 + `egui` 0.35 |
| 辅助 UI | `ratatui` / `crossterm`（CLI）；kiss3d 路径属遗留 viewer |

---

## 核心约定

1. **坐标系**：左手 ecliptic J2000；手性只在 math 层硬编码，渲染边界做投影翻转（见 [`docs/RENDERING.md`](docs/RENDERING.md)）。
2. **精度分界**：仿真 f64；渲染经 `CoordinateBridge`（相机为浮点原点）再进 f32。
3. **配置真相源**：`orbitx-config` TOML + [`docs/CONFIG_TOML.md`](docs/CONFIG_TOML.md)；不引入 Orbiter cfg 解析作为默认路径。
4. **数值正确性**：核心算法改动须有 FFI / 属性测试对照；默认忠实 Orbiter 行为，偏离须记入 [`docs/ORBITER_QUIRKS.md`](docs/ORBITER_QUIRKS.md)。
5. **历表数据**：运行可用 `assets/orbiter-data`；oracle 测试可走 `../orbiter/Src/Celbody/` 或 `ORBITER_SRC`。
6. **物理权威**：积木步进在 core crates；产品主进程为 **`orbitx-runtime`**（P4.2）；客户端（cli / Godot）经 **zenoh**（P4.3 / P4.4）。P4.3 前 cli 可暂直调 Assembly。勿把 `UserVessel` 或 kiss3d 遗留路径当权威。

---

## 反模式（禁止）

1. **错误层打补丁** — 调用方特判、demo 硬编码物理、复制「临时」公式而不收回权威 crate。
2. **违反单一职责** — 向 `math` 塞 IO、向 `dynamics` 塞 wgpu、向 `config` 塞积分步进、向 `app` 塞新物理子系统。
3. **碎片代码** — 同目的逻辑散落多处而不抽公共模块。
4. **绕过 Runtime / Assembly** — 在展示 crate / demo 内复制 vessel 物理闭环（含 `UserVessel` 类旁路）；或客户端绕过 Runtime 直调积木作为产品路径。
5. **运行时依赖 oracle** — 把 `*-ffi` / Orbiter C++ 编进发布产物或 Godot 进程。
6. **散落手性转换** — 运行时切换左右手，或在多处重复做坐标手性翻转。
7. **Orbiter 配置主格式** — 用 `.cfg` / `.scn` 作为 orbitx 配置默认路径。
8. **绕过 IPC 边界** — 在本树实现集成时用 Cargo path 把物理核心链进 `sim-rocket/rust`（以根 AGENTS 为准）。
9. **双真相源进度** — 在本文件堆砌与 ROADMAP / README 重复的完成度百分比。
10. **无测试合入** — 新模块或新公开 API 无对应单元测试。
11. **改完不测** — 只改实现不跑 / 不更新相关测试；或用手动开窗口代替可自动化的单元断言。
12. **红灯交付** — 相关 `cargo test` 失败仍宣称改动完成。
13. **测试内联实现文件** — 在 `.rs` 实现文件底部堆 `#[cfg(test)] mod tests { … }`，而不使用同目录独立 `tests.rs` + `mod tests;`。

---

## 文档权威

| 主题 | 权威来源 |
|------|----------|
| 工作区 / IPC / Godot 边界 | 根 [`AGENTS.md`](../AGENTS.md) |
| 本树目录、分层、开发约束、禁止事项 | **本文件** |
| 产品架构（Runtime / Controller / 接触 / Godot 会话） | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| 完成度、demo、构建命令 | [`README.md`](README.md) |
| 移植优先级与 P4/P5 | [`docs/ROADMAP.md`](docs/ROADMAP.md) |
| wgpu / egui 渲染架构 | [`docs/RENDERING.md`](docs/RENDERING.md) |
| rocket / scenario TOML | [`docs/CONFIG_TOML.md`](docs/CONFIG_TOML.md) |
| 键位 | [`docs/KEYBINDINGS.md`](docs/KEYBINDINGS.md) |
| 忠实 vs 修正 Orbiter 怪异行为 | [`docs/ORBITER_QUIRKS.md`](docs/ORBITER_QUIRKS.md) |

---

## 构建入口

构建见 [`README.md`](README.md)。本地 GUI：`cargo run -p orbitx-app`；产品主进程落地后为 `cargo run -p orbitx-runtime`。任何代码改动须按上文「单元测试与改动验证」对受影响 crate 执行 `cargo test -p …` 并以通过为门禁；本文件不维护逐步教程。
