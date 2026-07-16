# orbitx 移植路线图

基于 Orbiter C++ 源码（`/Users/jianglu/devel/johnson/orbiter/Src/Orbiter/`，~130 文件）
与 orbitx Rust 移植版（13 crate，~13,756 行）的系统对比，整理出以下差异清单与推进顺序。

---

## 当前完成度总览

| 领域 | Orbiter C++ | orbitx (Rust) | 状态 |
|------|-------------|---------------|------|
| 数学库 | `Vecmat.h`/`Astro.h` | `orbitx-math` (2,218 行) | ✅ 完整（逐符号 + FFI 验证） |
| 物理核心 | `BodyIntegrator`/`Rigidbody`/`Psys`/`PinesGrav` | `orbitx-dynamics` (3,200+ 行) | ✅ 完整（含刚体/TVC/旋转/多体容器，2026-07 新增） |
| 历表 | VSOP87/ELP82/TASS17/GALSAT | `orbitx-ephemeris` (2,452 行) | ✅ 完整（含 GALSAT 大不等修正） |
| 航天器 | `Vessel.cpp` 9,030 行 | `orbitx-vessel` (~3,500 行) | 🟡 部分（含气动/RCS/着陆/多储箱，~39% 覆盖） |
| 天体/场景 | `Psys`/`Celbody`/`Planet.cfg` | `orbitx-dynamics`/`orbitx-config` | ✅ 完整（含旋转/岁差/J2/Pines/多体容器） |
| 渲染/UI | D3D7 + Win32 + ImGui（~40 文件） | `orbitx-render`/`orbitx-gfx-hud`/`orbitx-app` | ✅ P3A++ 完成（历表驱动+3D球体+billboard fallback） |
| 配置 | `.cfg`/`.scn` 格式 | `orbitx-config` (700+ 行) | 🟡 部分（改用 TOML，含 body/system/rocket/scenario） |

**本次会话已完成**（commit `947be7a`–`7996dc2`）：
- 刚体物理建模（Euler 方程 + 力矩 + TVC 闭环），逐符号移植 `Rigidbody.cpp`
- PMI 归一化约定统一、engine_dir 修正、launch_attitude 对齐
- 可复现模式（固定步长，默认开启）
- 10 个 bit-equal 可复现性测试

**P0 已全部完成**（积分器 FFI oracle + GALSAT oracle + revizg_ 大不等修正）：
- P0.1：RK2/4/5/8 + SY2/4/6/8 多步轨迹对照测试（20 个 proptest case）
- P0.2：GALSAT 4 卫星 + barycentre oracle 测试（已存在并验证通过）
- P0.3：`revizg_` 木土大不等修正完整实现（packed-code 解码器 `unkod` + `updat` + `revizg`）

---

## P0 — 闭合测试缺口（✅ 已完成）

P0 的三个子任务已全部完成，消除了全仓库唯一的"未实现"标记（`revizg_`），
并闭合了所有核心数值路径的 FFI oracle 验证。

### P0.1 积分器 FFI oracle 测试 ✅
- **结果**：RK2/4/5/8 + SY2/4/6/8 多步轨迹对照测试（圆轨道 + 椭圆轨道 + J2 扰动场景），
  共 20 个 proptest case 全部通过。
- **涉及文件**：`orbitx-dynamics-ffi/cpp/shim.cpp`（新增 RK2/RKdrv/SY 步进函数）、
  `orbitx-dynamics-ffi/src/lib.rs`（新增 FFI 绑定）、
  `orbitx-dynamics/tests/ffi_oracle.rs`（新增 12 个测试）。

### P0.2 GALSAT oracle 测试 ✅
- **结果**：4 卫星（Io/Europa/Ganymede/Callisto）+ barycentre oracle 测试已存在并通过。
  额外补充了 `prop_galsat_barycentre`（ksat=0）测试。
- **涉及文件**：`orbitx-ephemeris/tests/ffi_oracle.rs`。

### P0.3 GALSAT `revizg_` 大不等修正 ✅
- **结果**：实现了 packed-code 解码器 `unkod` + 级数参数更新 `updat` +
  木土大不等修正 `revizg`。Rust 和 C++ oracle 逐位一致。
- **涉及文件**：`orbitx-ephemeris/src/galsat.rs`（新增 `SatKod`/`unkod`/`updat_series`/`revizg`）、
  `orbitx-ephemeris-ffi/cpp/shim.cpp`（新增 `GalUnkod`/`GalUpdat`/`GalRevizg`）。

---

## P1 — 扩展航天器物理（✅ 已完成）

orbitx-vessel 从 1,614 行（~10% Orbiter 覆盖）扩展到 ~3,500 行（~39% 覆盖），
新增气动力模型、RCS 推进器组、着陆接触力、多储箱燃料系统。

### P1.1 气动力模型 ✅
- **结果**：移植 Orbiter `UpdateAerodynamicForces`（`Vessel.cpp:4099-4226`）到 `aero.rs`。
  支持：空气翼面（常量/线性/查表升阻系数）、控制面、变阻力元件、气动阻尼、
  指数衰减大气模型。CLI 外部硬编码阻力已移除，改用 vessel 内置。
- **涉及文件**：`aero.rs`（新增）、`vessel.rs`（新增 airfoils/ctrlsurfs/dragels 字段）、
  `assembly.rs`（step 中集成气动力）、`main.rs`（CLI 重构）。
- **测试**：13 个（大气模型、零速、方向、升阻正交、控制面、阻尼、手算验证、查表插值）。

### P1.2 RCS / 姿态推进器 ✅
- **结果**：实现 `ThrusterGroup`/`ThrusterGroupType`（15 个标准组），
  `add_default_rcs`（12 推进器默认布局，移植 `CreateDefaultAttitudeSet`），
  `set_attitude_rot`/`set_attitude_lin`（姿态/平移控制接口）。
  Assembly::step 现在包含所有级的所有推进器（主发动机 + RCS）。
- **涉及文件**：`rcs.rs`（新增）、`vessel.rs`（新增 thruster_groups 字段）、
  `assembly.rs`（推力收集扩展到所有级）。
- **测试**：7 个（布局、力矩、平移无力矩、限幅、姿态/平移控制）。

### P1.3 着陆/碰撞检测 ✅
- **结果**：实现 `TouchdownVertex`（弹簧+阻尼+摩擦触点）和
  `compute_surface_forces`（移植 `Vessel.cpp:4289-4590` 简化版），
  `make_landing_gear`（三点着陆架辅助函数）。
- **涉及文件**：`touchdown.rs`（新增）、`vessel.rs`（新增 touchdown_points 字段）。
- **测试**：7 个（无接触、弹簧力、阻尼、摩擦、三点架、硬着陆、空触点）。

### P1.5 燃料系统 ✅
- **结果**：实现 `PropellantTank`（多储箱，移植 `TankSpec`），
  推进器↔储箱关联（`Thruster::tank_id`），向后兼容旧式 `fuel_mass`。
- **涉及文件**：`fuel.rs`（新增）、`thruster.rs`（新增 tank_id）、
  `vessel.rs`（新增 tanks 字段）、`assembly.rs`（多储箱燃料消耗）。
- **测试**：7 个（创建、消耗、限幅、流率、效率、快照、向后兼容）。

### P1.4 通用对接组合体（延后）
- 现有 `Assembly` 同轴堆叠已覆盖发射场景。完整 SuperVessel dock 树留到空间站组装需求时再做。

### 集成测试 ✅
- `falcon9_full_ascent_with_aero`：F9 含气动上升不崩溃
- `reentry_deceleration`：有阻力 vs 无阻力对照
- `rcs_attitude_hold`：RCS 俯仰产生角速度
- `multi_tank_independent_consumption`：多储箱独立消耗
- `landing_touchdown_stops_descent`：着陆触点使下沉停止

### Demo ✅
- `orbitx-demo-aero`：再入气动减速演示（有/无气动对照）
- `orbitx-demo-landing`：着陆接触力演示（软/硬着陆）

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

## P2 — 天体/场景完整性（从"单地球"到"真实太阳系"）✅

### P2.1 行星物理参数配置 ✅
- **新增** `orbitx-config/src/body.rs`：`BodyConfig`（serde TOML），含 `EphemerisConfig`、
  `RotationConfig`、`GravityConfig`（Jcoeff/Pines）、`AtmosphereConfig`。
- **新增** `orbitx-config/src/system.rs`：`SystemConfig`（太阳系树形配置 + 父子关系）。
- **内置默认**：`BodyConfig::earth()`/`moon()`/`jupiter()` 等 14 个函数，
  数值与 Orbiter Planet.cfg 一致（如 Earth mass=5.973698968e24）。
- **测试**：7 个（earth_default_mass/gravity/rotation, moon_obliquity, jupiter_j2, toml_roundtrip, parse）。

### P2.2 多体场景容器 ✅
- **新增** `orbitx-dynamics/src/planetary.rs`：
  - `CelestialBody`：带完整物理参数的天体（mass/size/pos/rotation/gravity/atmosphere/ephemeris）。
  - `PlanetarySystem`：树形容器 + 引力场聚合（`gacc`含 J-coeff + Pines 分支）。
  - `EphemerisModel`：统一封装 VSOP87/ELP82/GALSAT/TASS17。
  - `from_config()`：从 `SystemConfig` 构建，加载历表和重力模型。
  - `update_positions()`：历表驱动天体位置更新（含卫星→父天体偏移）。
  - `to_grav_bodies()`：向后兼容旧接口。
- **测试**：4 个（celestials_sorted_by_mass, gacc_point_mass_only, gacc_with_jcoeff, to_grav_bodies_backward_compat）。

### P2.3 行星自转/姿态 ✅
- **新增** `orbitx-dynamics/src/rotation.rs`：`RotationState` 结构。
  - 移植 `CelestialBody::UpdatePrecession()`（Celbody.cpp:493-518）。
  - 移植 `CelestialBody::UpdateRotation()`（Celbody.cpp:521-534）。
  - 移植 `CelestialBody::GetRotation(t)`（Celbody.cpp:537-548）。
  - 岁差矩阵 R_ecl、旋转轴 R_axis、旋转角 rotation 全部实现。
- **测试**：8 个（earth_rotation_period/angle_advances/obliquity, moon_obliquity,
  no_precession_simplifies, get_rotation_matches_update, rotation_matrix_orthonormal, jupiter_rotation）。

### P2.4 非球形重力场景接入 ✅
- **J-coeff 修复**：`jcoeff_perturbation_with_rot()` 使用体坐标系旋转矩阵计算纬度
  （匹配 Orbiter `tmul(GRot(), er)` 约定），修复了 y 轴硬编码 bug。
- **Pines 分支**：`pines_perturbation()` 完整实现（旋转→体坐标系→m→km→左手→右手→accel→右手→左手→km→m→旋转回）。
- **GravBody 扩展**：新增 `rotation: Option<Matrix3>` 和 `pines: Option<(Arc<PinesModel>, usize)>` 字段。
- **gacc_nbody 升级**：自动使用旋转矩阵（若有）和 Pines 模型（若有）。
- **CLI 改造**：用 `BodyConfig::earth()` 质量替代硬编码，启用 Earth J2=1.0826e-3。
- **flight 改造**：用 `body_config()` 替代 `body_mass()` 硬编码，自动启用 J-coeff（Jupiter J2=0.01475）。
- **FFI oracle 修复**：C++ jcoeff oracle 改用 `crossp` 公式（匹配 Psys.cpp:658-661）。
- **测试**：7 个（jcoeff_with_rot_matches_identity, pines_perturbation_at_pole/decreases_with_distance,
  gacc_nbody_with_jcoeff_and_rotation, jcoeff_with_rot_matches_no_rotation, jcoeff_with_tilted_rotation_differs, pines_perturbation_at_pole）。

### Demo ✅
- `orbitx-demo-orrery`：终端 UI 显示太阳系 14 个天体配置（名称/质量/半径/重力/自转/大气/父天体）。

---

## P3 — 渲染与 UI：Rust 原生图形栈 ✅ P3A+ 已完成

Orbiter 渲染/UI 深度绑定 D3D7/Win32/ImGui（~40+ 文件）。**不建议直译**，改用 wgpu + egui + glam Rust 原生图形栈。

### 技术选型

| 层 | 选型 | 理由 |
|----|------|------|
| GPU 抽象 | wgpu 29 (Vulkan/Metal/D3D12/WebGPU) | WebGPU spec 稳定，跨平台 |
| 窗口/输入 | winit 0.30 | Rust 生态标准，与 egui 深度集成 |
| HUD/MFD | egui 0.35 (egui-wgpu + egui-winit) | 即时模式适配仪表盘，API 稳定 |
| GPU 数学 | glam 0.29 (f32) | SIMD 优化，与 orbitx-math f64 互补 |

### 新 Crate 架构

| Crate | 职责 | 状态 |
|-------|------|------|
| `orbitx-render` | CoordinateBridge / CameraSystem / RenderGraph / SceneNode / SceneManager | ✅ P3A |
| `orbitx-gfx-hud` | HUD/MFD egui 面板（3 种 HUD 模式 + 10 种 MFD） | ✅ P3A |
| `orbitx-app` | winit 窗口 + wgpu + 仿真主循环 + egui 集成 | ✅ P3A |
| `orbitx-gfx-planet` | 四叉树瓦片 LOD + 大气散射 + 云层 + 环系 | 🔲 P3B |
| `orbitx-gfx-vessel` | .msh/glTF 加载 + PBR + 尾焰粒子 | 🔲 P3C |

### P3A — 渲染基础 ✅（2026-07 完成）

**orbitx-render**（5 模块，21 测试通过）：
- `coord.rs` — `CoordinateBridge`：f64 浮点原点→f32 渲染坐标转换
  - 太阳系缩放模式（1 AU = N 渲染单位）与真实尺度模式
  - 左手→右手映射：render.x=sim.x, render.y=sim.z, render.z=-sim.y
- `camera.rs` — `CameraSystem`：6 外部模式 + 3 内部模式
  - TargetRelative / AbsDirection / GlobalFrame / TargetToObject / TargetFromObject / GroundObserver
  - 对数深度缓冲：z=log₂(C·w+1)/log₂(C·far+1)，近/远比 1e15
- `render_graph.rs` — `RenderGraph`：10 标准渲染 pass + 依赖追踪
- `scene.rs` — `SceneNode` / `Transform64` / `SceneManager`：f64→f32 逐帧转换 + 远距 fallback

**orbitx-gfx-hud**（3 模块，8 测试通过）：
- `flight_state.rs` — `FlightState`：位置/速度/轨道根数/姿态/推进/环境
- `hud.rs` — `HudState`：3 种 HUD 模式 × 4 种颜色，egui::Painter 自定义绘制
- `mfd.rs` — `MfdPanel`：10 种 MFD 类型，CRT 绿色美学，6 按钮布局

**orbitx-app**（完整编译通过）：
- `app.rs` — `App` + `ApplicationHandler`：`egui_wgpu::winit::Painter` 管理全部 GPU 生命周期
- `input.rs` — 30 键盘 Action 映射
- `lib.rs` / `main.rs` — winit EventLoop 入口
- 渲染流水线：`take_egui_input` → `ctx.run_ui` → `tessellate` → `paint_and_update_textures`

**全工作区 242 测试通过，零回归。**

### P3A+ — 第一个 3D 画面 ✅（2026-07 完成）

在 P3A 骨架基础上，实现 wgpu 3D 球体渲染管线，太阳系天体以彩色球体出现在屏幕上，
egui HUD 叠加在 3D 场景上方。

**关键架构发现**：`egui_wgpu::CallbackTrait` 允许在 egui 的 RenderPass 内插入自定义
wgpu 绘制命令，3D 与 egui 共享同一个 CommandEncoder/RenderPass，无需独立管理 GPU 生命周期。

**新增文件**：
- `scene_renderer.rs` — `SceneRenderer`：wgpu 管线 + `FrameScene` + `SceneCallback`（CallbackTrait）
  - Uniforms 结构（MVP、model、base_color、light_dir、log_depth）
  - 深度缓冲 `Depth32Float`，对数深度输出
  - 逐天体 uniform 上传 + `draw_indexed`
- `sphere.rs` — UV 球体几何生成（24×16 segments，5 测试通过）
- `shader/planet.wgsl` — WGSL 顶点+片元着色器（对数深度 + 半 Lambert 光照）

**修改文件**：
- `app.rs` — Painter 启用深度、从 `render_state()` 创建 SceneRenderer、
  `Callback::new_paint_callback` 注入 3D 渲染、`callback_resources` 注入
- `camera.rs` — 新增 `view_matrix()`（sim→render 坐标映射 + `look_to_rh`）

**全工作区 246+ 测试通过，零回归。**

### P3A++ — 活着的太阳系 ✅（2026-07 完成）

在 P3A+ 基础上，接入历表驱动天体位置 + 远距 fallback billboard 渲染，
从"编译通过但什么都看不到"到"可见的动态太阳系"。

**历表驱动位置**：
- 新增 `ephem_bridge.rs`：`PlanetarySystem` ↔ `SceneManager` 同步
  - `create_planetary_system()` — 从 `SystemConfig::sol()` 加载，fallback 到 no-ephemeris
  - `create_scene_from_psys()` — 从 PlanetarySystem 动态创建 SceneNode
  - `sync_positions()` — 每帧将历表位置写入场景节点
  - `sim_time_to_mjd()` — 仿真时间 → MJD 转换
- `app.rs` 接入 `PlanetarySystem`：每帧推进 MJD → `update_positions()` → `sync_positions()`
- 支持 `ORBITER_SRC` 环境变量指定 `.dat` 文件路径

**远距 fallback billboard 渲染**：
- 新增 `shader/billboard.wgsl`：camera-facing disc/glow 着色器
- `scene_renderer.rs` 扩展为双管线架构：
  - `BodyDraw` 枚举：`Sphere`（3D 球体） / `Billboard`（2D 光点/圆盘）
  - `FrameScene::from_scene()` 根据 `screen_size < min_render_radius` 决定渲染方式
  - 恒星始终 billboard（glow 效果），行星远距 billboard、近距 sphere
  - 太阳相对光照方向（替代硬编码 `[0.3, 1.0, 0.5]`）
- `camera.rs` 新增 `fov_y()` 公开 getter

**全工作区 250+ 测试通过，零回归。**

### P3B — 行星渲染 🔲
- 四叉树瓦片 LOD（移植 TileManager2 / ZTreeMgr / elevmgr）
- 行星纹理 + 高程数据（DDS + ZTreeMgr 格式）
- 大气散射（移植 VPlanetAtmo.cpp Rayleigh/Mie）
- 云层 + 环系

### P3C — 航天器渲染 🔲
- .msh 格式兼容（移植 Mesh.cpp 解析器）+ glTF 2.0 新模型
- PBR metallic-roughness shader
- 尾焰粒子 + 阴影

### P3D — HUD/MFD 完善 🔲
- 3 种 HUD 模式完整绘制（姿态梯、航向带、速度/高度标尺）
- 4 种核心 MFD（Orbit/Map/Docking/Landing）

### P3E — 相机完善 🔲
- 6 外部模式完整交互
- 驾驶舱模式（GenericCockpit / Panel2D / VirtualCockpit）
- 动态近平面调整

### P3F — 集成优化 🔲
- InputMap 配置化
- 视锥裁剪 + 实例化渲染
- 文档

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
