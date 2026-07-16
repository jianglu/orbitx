# orbitx 移植路线图

基于 Orbiter C++ 源码（`/Users/jianglu/devel/johnson/orbiter/Src/Orbiter/`，~130 文件）
与 orbitx Rust 移植版（13 crate，~13,756 行）的系统对比，整理出以下差异清单与推进顺序。

---

## 当前完成度总览

| 领域 | Orbiter C++ | orbitx (Rust) | 状态 |
|------|-------------|---------------|------|
| 数学库 | `Vecmat.h`/`Astro.h` | `orbitx-math` (2,218 行) | ✅ 完整（逐符号 + FFI 验证） |
| 物理核心 | `BodyIntegrator`/`Rigidbody`/`Psys`/`PinesGrav` | `orbitx-dynamics` (2,130 行) | ✅ 完整（含刚体/TVC，2026-07 新增） |
| 历表 | VSOP87/ELP82/TASS17/GALSAT | `orbitx-ephemeris` (2,452 行) | ✅ 完整（GALSAT 有小缺口） |
| 航天器 | `Vessel.cpp` 9,030 行 | `orbitx-vessel` (1,614 行) | 🟡 部分（多级火箭刚体，~10% 覆盖） |
| 渲染/UI | D3D7 + Win32 + ImGui（~40 文件） | `orbitx-scene`/`orrery`/`flight`/`launch`/`cli` | 🔴 骨架（kiss3d 占位 + ratatui TUI） |
| 配置 | `.cfg`/`.scn` 格式 | `orbitx-config` (479 行) | 🟡 部分（改用 TOML，不兼容旧格式） |

**本次会话已完成**（commit `947be7a`–`7996dc2`）：
- 刚体物理建模（Euler 方程 + 力矩 + TVC 闭环），逐符号移植 `Rigidbody.cpp`
- PMI 归一化约定统一、engine_dir 修正、launch_attitude 对齐
- 可复现模式（固定步长，默认开启）
- 10 个 bit-equal 可复现性测试

---

## P0 — 闭合测试缺口（低风险、高置信度）

这些是**已有代码但缺验证**的缺口。shim 已就绪，只需补测试代码。

### P0.1 积分器 FFI oracle 测试
- **现状**：`orbitx-dynamics-ffi/cpp/shim.cpp` 已实现 `ox_rk4_step` + force callback 机制，
  但 `tests/ffi_oracle.rs` 没有 `prop_rk4_*` 测试。RK2/4/5/8/SY2-8 的正确性仅靠内部单元测试覆盖。
- **差距**：这是**唯一未验证的核心数值路径**。
- **任务**：加 RK2/4/5/8 + SY2/4/6/8 多步轨迹对照测试（圆轨道、椭圆轨道场景）。
- **预估**：半天。
- **涉及文件**：`crates/orbitx-dynamics-ffi/cpp/shim.cpp`（扩展 callback 支持 omega/q）、
  `crates/orbitx-dynamics/tests/ffi_oracle.rs`。

### P0.2 GALSAT oracle 测试
- **现状**：Rust `GalModel`（`galsat.rs`，547 行）已移植，但无 `prop_galsat_eval`。
- **任务**：加木卫历表（4 个伽利略卫星）对照测试。shim 的 C++ Lieskie 实现可用。
- **预估**：2 小时。

### P0.3 GALSAT `revizg_` 大不等修正
- **现状**：`galsat.rs:388` 是全仓库唯一的"未实现"标记——木星-土星大不等修正的 packed-code 解码器。
  对 ±50 年范围精度影响很小。
- **任务**：实现 packed-code 解码器，补全全精度 Lieskie。
- **预估**：半天。

---

## P1 — 扩展航天器物理（orbitx-vessel 最大短板）

orbitx-vessel 当前是**聚焦多级火箭的刚体模型**，对比 `Vessel.cpp`（9,030 行）缺大量子系统。

### P1.1 气动力模型
- **现状**：Orbiter `Vessel::AddSurfaceForces`（`Vessel.cpp:4289`）实现完整气动——
  动压 `sp.dynp`、参考面积 `S`、升阻系数 `CL/CD`、方向 `ldir/ddir/sdir`、控制面 `ctrlsurf`、阻力元件 `dragel`。
  orbitx 在 CLI/launch 里**外部硬编码** `0.5·ρ·v²·CD`，不在 vessel crate 内。
- **任务**：移植 Orbiter airfoil/lift-drag 模型到 `orbitx-vessel`，使物理自洽。
- **关键源文件**：`Vessel.cpp:4150-4222`（气动力）、`Vessel.h`（`AirfoilDef`/`ctrlsurf`/`dragel`）。
- **预估**：2-3 天。

### P1.2 RCS / 姿态推进器
- **现状**：仅主发动机 TVC。Orbiter 有 `THGROUP_ATTITUDE`/`THGROUP_RETRO` 等推进器组。
- **任务**：加 RCS 推进器组（轨道姿态控制、交会对接所需）。
- **关键源文件**：`Vessel.h:484-679`（`THGROUP_*` 定义、`CreateThrusterGroup`）。
- **预估**：1-2 天。

### P1.3 着陆/碰撞检测
- **现状**：Orbiter `SetTouchdownPoints`（`Vessel.cpp:1137`）支持 3+ 个带刚度/阻尼/摩擦的触地点，
  `AddSurfaceForces` 计算地面接触力/力矩。orbitx CLI 用径向速度约束伪造"发射台支撑"。
- **任务**：移植 touchdown point 模型（支持着陆、碰撞、倾斜地面）。
- **关键源文件**：`Vessel.cpp:371-386`（默认触地点）、`4289+`（接触力计算）。
- **预估**：2 天。

### P1.4 通用对接组合体
- **现状**：Orbiter `SuperVessel`（`SuperVessel.cpp`，1,173 行）支持任意 dock 树 + 相对旋转。
  orbitx `Assembly` 假设同轴堆叠。
- **任务**：扩展为任意 dock 树（空间站组装场景）。
- **关键源文件**：`SuperVessel.cpp`（`CalcPMI` 已移植，缺 `Add`/`Split`/`ComponentStateVectors`）。
- **预估**：2-3 天。

### P1.5 燃料系统
- **现状**：每级单 `fuel_mass` 标量。Orbiter 有多 tank、优先级、crossfeed。
- **任务**：加多 tank/资源定义。
- **关键源文件**：`Vessel.h`（`PROPELLANT_HANDLE`/`CreatePropellantResource`/`SetFuelMass`）。
- **预估**：1-2 天。

---

## P2 — 天体/场景完整性（从"单地球"到"真实太阳系"）

### P2.1 行星物理参数配置
- **现状**：Orbiter `Planet.cfg`（大小/质量/J系数/自转周期/大气）。orbitx 在 CLI/flight 里
  **硬编码** `GravBody{mass, size}` 常量。
- **任务**：加 `body.toml` 配置 + `Planet` 结构（自转/大气/J系数）。
- **预估**：1-2 天。

### P2.2 多体场景容器
- **现状**：Orbiter `Psys`（`PlanetarySystem`）是恒星→行星→卫星的树形容器，
  含引力场聚合（`Gacc`/`Gacc_intermediate`/`ScanGFieldSources`）。
  orbitx CLI 单地球、flight 10 个简化 GravBody。
- **任务**：统一用 `PlanetarySystem` 容器 + 历表驱动天体位置。
- **关键源文件**：`Psys.cpp/.h`（容器与引力场）。
- **预估**：2-3 天。

### P2.3 行星自转/姿态
- **现状**：Orbiter `Celbody` 有完整自转模型（`rotation`/`rot_T`/`rot_omega`、岁差、分点）。
  orbitx 地球固定不自转。
- **任务**：移植 `UpdateRotation`/`GetRotation(t)`。
- **关键源文件**：`Celbody.cpp/.h`（`UpdateRotation`/`UpdatePrecession`/`GetRotation`）。
- **预估**：2 天。

### P2.4 非球形重力场景接入
- **现状**：Pines 球谐模型已移植（`pines.rs`，完整），但 CLI/flight 用空 `jcoeff`，未启用。
- **任务**：在场景中启用 J2-J4 / Pines（地球扁率摄动）。
- **预估**：半天（接入 + 测试）。

---

## P3 — 渲染与 UI（高成本，建议重新设计）

Orbiter 渲染/UI 深度绑定 D3D7/Win32/ImGui（~40+ 文件）。**不建议直译**，应选 Rust 原生图形栈。

| 差异 | Orbiter | orbitx | 说明 |
|------|---------|--------|------|
| 摄像机 | `Camera.cpp`（1,763 行） | kiss3d OrbitCamera3d | 轨道/地面/跟踪模式 |
| HUD | `hud.cpp`（2,147 行） | ratatui TUI 表格 | 轨道/地面/对接模式 |
| MFD（11 种） | Orbit/Map/Docking/Landing/Hsi/Sync/Transfer… | 无 | 多功能显示器 |
| 网格/纹理 | `.msh` 加载、纹理管理、阴影 | 无 | — |
| 行星地表 | `elevmgr` + `ZTreeMgr`（高程 LOD） | 球体 | 四叉树瓦片 |

**前置依赖**：应先完成 P1-P2，渲染才有意义。选定 Rust 图形栈（如 `wgpu` + 渲染抽象层）后重新设计。

---

## P4 — 架构整合（消除重复）

三个图形 app 有重复物理代码：
- `orbitx-launch` 有自己的 `Rocket` 结构，**不使用 orbitx-vessel**（已被 CLI 功能性取代）
- `orbitx-flight` 用自有 force 闭包，不经 Assembly

**任务**：统一到 `orbitx-vessel` + `orbitx-scene`，合并/废弃 `launch`→`cli`、`flight` 复用 vessel。

---

## 推荐执行顺序

```
P0（闭合测试缺口）   ←  低风险、高置信度，立即动手
  ↓
P1（航天器物理）      ←  vessel crate 最大短板
  ↓
P2（天体/场景）       ←  从单地球走向真实太阳系
  ↓
P3（渲染/UI）         ←  选定 Rust 图形栈后重做
```

### 最值得立即动手的 3 件事

1. **积分器 FFI oracle 测试**（P0.1）—— shim 已就绪只差测试代码，半天工作量，闭合唯一未验证的核心数值路径
2. **气动力模型移植**（P1.1）—— 当前阻力是 app 层硬编码，移入 vessel crate 让物理自洽
3. **行星配置 + 多体场景**（P2.1 + P2.2）—— 从单地球升级到历表驱动的太阳系

---

## 附录：Orbiter 核心架构（移植参考）

### 主仿真循环（`Orbiter.cpp`）
```
SingleFrame()
  ├─ BeginTimeStep()      // td 时间推进（真实时间 → SimDT，含 warp）
  ├─ UpdateWorld()        // 物理阶段
  │    ├─ ModulePreStep() // 插件 pre-step 回调（td.SimT0）
  │    ├─ g_psys->Update()// 行星系推进（所有 body 的 RK4 积分）
  │    └─ ModulePostStep()// 插件 post-step 回调（td.SimT1）
  ├─ EndTimeStep()        // T1→T0 状态拷贝
  ├─ UserInput()          // 输入处理
  └─ Render3DEnvironment()// 渲染 + Output2DData（HUD/MFD/Panel）
```

### 时间管理（`TimeData`）
- `SysT0/SysT1/SysDT`：系统（真实）时间
- `SimT0/SimT1/SimDT`：仿真时间步（SimDT 含 warp）
- `MJD0/MJD1`：修正儒略日
- `TWarp/TWarpTarget`：时间加速倍率

### 类层级
```
Body → RigidBody → CelestialBody → Planet / Star
                   └→ VesselBase → Vessel / SuperVessel
```

### 代码规模 Top 10（Orbiter C++）
| 行数 | 文件 | orbitx 对应 |
|------|------|------------|
| 9,030 | Vessel.cpp | orbitx-vessel (1,614 行，~18%) |
| 3,563 | Baseobj.cpp | — |
| 2,792 | Orbiter.cpp | —（运行时核心） |
| 2,661 | OrbiterAPI.cpp | —（插件 API） |
| 2,147 | hud.cpp | ratatui TUI |
| 1,763 | Camera.cpp | kiss3d |
| 1,578 | Config.cpp | orbitx-config (479 行) |
| 1,548 | VectorMap.cpp | — |
| 1,290 | Mfd.cpp | — |
| 1,228 | Mesh.cpp / SuperVessel.cpp | SuperVessel.CalcPMI 已移植 |
