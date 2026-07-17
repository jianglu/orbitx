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
| 渲染/UI | D3D7 + Win32 + ImGui（~40 文件） | `orbitx-render`/`orbitx-gfx-hud`/`orbitx-app` | ✅ P3A+++ 完成（历表驱动+3D球体+billboard+黄道面/轨道/垂线，数据自包含） |
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

### P3A+++ — 可见性修复 + 场景可视化 + 数据自包含 ✅（2026-07 完成）

在 P3A++ 基础上，通过 8 个独立验证 demo（`crates/orbitx-app/examples/`）逐子系统定位并
修复了一系列导致"只显示 HUD / 只有一个点"的真实 bug，并把太阳系场景可视化做完整。

**验证 demo（逐子系统隔离验证）**：
- `demo_callback_triangle` — egui_wgpu CallbackTrait 回调渲染
- `demo_callback_billboard` — billboard.wgsl 管线
- `demo_callback_sphere` — planet.wgsl + 对数深度
- `demo_coord_camera` — CoordinateBridge + Camera 完整变换链
- `demo_ephemeris`（headless）— 历表真实 J2000 位置 + MJD 推进（5/5 断言）
- `demo_lod_transition` — 球体↔billboard LOD 平滑切换
- `demo_camera_interaction` — 拖拽/缩放/焦点 + 黄道面网格/轨道环
- `demo_full_scene` — 真实历表 + SceneManager 完整集成 + 黄道面/轨道/垂线

**修复的核心 bug**：
- **共享 uniform buffer**（"一个点"根因）：14 天体 uniform 全写同一 buffer，
  `queue.write_buffer` 提交前统一刷新，最后一个覆盖全部 → 改为每天体独立 buffer 池
- **透明面板**：CentralPanel 不透明背景遮挡 3D 场景 → Frame 透明
- **screen_px 单位不匹配**：渲染单位半径 ÷ 米制距离恒为 0，LOD 失效 → 统一渲染单位
- **坐标系约定**：黄道北 sim.y → render +y（黄道面水平，原映射使其竖直）
- **默认相机取景**：原 dist=1e8 使相机位于太阳内部 → 拉远框住内太阳系
- **CAD 拖拽**：X 绕转/Y 俯仰，修正 mouse_drag 双重缩放（2.5e-5）

**主 app 场景可视化**（`build_scene_lines` + 线渲染管线 LineList + 对数深度）：
- 水平黄道面网格（同心圆 + 辐条）
- 各行星轨道环（真实日心距离）
- 各行星到黄道面的垂线（黄道纬度可视化）

**数据自包含**（免设 `ORBITER_SRC`）：
- 12 个历表 `.dat`（2.1MB）内置 `assets/orbiter-data`，镜像 Orbiter `Src/Celbody` 结构
- `resolve_orbiter_src()` 候选搜索：环境变量 > 项目内（编译期路径）> cwd 相对 > 遗留
- gravity 加载非致命：模型文件缺失回退 PointMass（渲染只需位置，避免内置 10MB 重力场）

### P3B — 行星渲染 🟡（P3B-1/2/3/4a/4b 已完成）

#### P3B-1 行星表面纹理 ✅（2026-07 完成）
- 球体顶点加等距柱状 UV（u=经度/2π, v=余纬/π）
- 修复球体三角形绕序 bug（原朝内，被 FrontFace::Ccw+cull Back 剔除成里朝外）
- scene_renderer 双 bind group 纹理支持（group0 uniform + group1 纹理+采样器），
  planet.wgsl 采样 + use_texture 开关；未贴图天体白色回退
- 高清纹理内置 `assets/textures/planets/`：Mercury..Neptune + Moon 2048×1024
  （Solar System Scope, CC BY 4.0），小卫星保留 Orbiter 低分图
- image 依赖（png+jpeg）；demo_textured_planet 验证
- 拉近行星（Tab 聚焦 + 滚轮缩放）即显示真实表面纹理

#### P3B-2 大气壳层 ✅（2026-07 完成）
- atmosphere.wgsl 边缘辉光壳层（Fresnel rim + 昼侧渐隐，预乘 alpha）
- 有大气天体（地球/金星/火星/土卫六/气态巨行星）sphere 后叠加 ×1.03 辉光壳
- 逐天体大气色（PlanetRenderState.atmosphere_color）；demo_atmosphere 验证

#### P3B-3 土星环 ✅（2026-07 完成）
- sphere::generate_ring 赤道面环带（径向 UV）+ ring.wgsl 采样径向纹理（alpha 环缝）
- 内置 Saturn_ring.png（Solar System Scope CC BY 4.0）；双面、直式 alpha、不写深度
- Saturn 在 sphere 后画环（固定 26.7° 倾角）；demo_saturn_ring 验证

#### P3B-4a 云层 ✅（2026-07 完成）
- cloud.wgsl: 等距柱状云图亮度作 opacity, 昼侧受光, discard 空白区
- 内置 Earth_clouds.jpg（Solar System Scope CC BY 4.0）
- 地球 surface 后叠加 ×1.01 云壳, 绕轴缓慢漂移（FrameScene.time 驱动）
- 渲染层次: 地表 → 云层(1.01) → 大气(1.03)

#### P3B-4b 太阳发射球 ✅（2026-07 完成）
- planet.wgsl emissive 标志（light_dir.w）: 发射天体全亮不受光照
- 内置 Sun.jpg（Solar System Scope CC BY 4.0）; Star 渲染为发射纹理球
- 远距仍黄色辉光光点; 复用现有球体管线

#### P3B-4c 🔲（后续）
- 四叉树瓦片 LOD（移植 TileManager2 / ZTreeMgr / elevmgr）+ 高程数据
- 大气 Rayleigh/Mie 物理散射（移植 VPlanetAtmo.cpp）


### P3C — 航天器渲染 🟡（P3C-1/2/3 已完成，P3C-4 后续）

#### P3C-1 航天器场景节点 + 可见性 ✅
- `orbitx-render/scene.rs`：`VesselRenderState` 加 `throttle: f32`；
  `SceneNode::new_vessel(id, scale, mesh_name, color)` 辅助函数
- `orbitx-app/ephem_bridge.rs`：`add_vessel_node(scene, mesh, color, scale_m)` +
  `sync_vessel_position(scene, vessel, psys, node_idx)` 每帧同步
  `abs_pos = parent.pos + vessel.rel_pos`
- `scene_renderer.rs::FrameScene::from_scene`：新增 `NodeType::Vessel` 分支
  （4 像素最低半径、emissive 亮色，远距 billboard、近距 sphere）
- `app.rs`：`vessel_node_idx: Option<usize>` + 初始化时插入 vessel 节点（cyan，40 m 特征长度）
  + 每帧在 propagate 后 sync 到 scene（保证本帧渲染最新位置）
- `body_radii` 长度与 scene.len() 不匹配时（含 vessel）退回不带半径的 update

#### P3C-2 尾焰（最小可行） ✅
- 每帧从 vessel.throttle 同步到 scene 节点的 `VesselRenderState.throttle`
- `from_scene`：Vessel 节点若 throttle > 0.01，额外发射一个亮橙色 billboard
  （像素半径 = base × (1.5 + 6 × throttle)，颜色由 throttle 加深）
- 输入：↑/↓ 增减 5% 油门；0 全推、` 切断
- 右侧信息栏显示 `Thr xx%  fuel xx kg`

#### P3C-3 程序化火箭 mesh + 姿态驱动 ✅
- `orbitx-app/src/vessel_mesh.rs`：`generate_rocket(radial_segments)` 生成程序化
  火箭几何（圆柱身 + 鼻锥 + 收敛发动机锥 + 底板 + 4 片十字尾翼），
  局部 +Y 为鼻锥方向；顶点复用 `sphere::Vertex`（position + normal + uv）；
  u16 索引（vertex count ≪ 65k）。4 tests（顶点/索引/尺度/绕周细分）
- `BodyDraw::VesselMesh { position, scale, orientation: [[f32;3];3], color }`
  新变体；`FrameScene::from_scene` 中 Vessel 节点当 screen_px ≥ 4 时（近距）
  emit VesselMesh 而非 Sphere，orientation 从 `node.render_data.rotation`（已由
  CoordinateBridge f64→f32 转换）取
- `SceneRenderer`：新增 `vessel_vertex_buffer` / `vessel_index_buffer` /
  `vessel_index_count` / `vessel_slots` 池；`set_frame` 同步扩容；
  `draw_vessel_mesh` 方法（复用 planet.wgsl 管线 + Uniforms + 白色 fallback 纹理，
  自发光标志 emissive=1 使飞船在阴影侧仍可见）
- `app.rs`：每帧 `Matrix3::from_euler(Vec3::new(pitch, yaw, bank))` →
  `node.transform.rotation`；姿态变化通过 CoordinateBridge 传到 render 层
- 兼容 P3C-1 远距 fallback（billboard）—— 拉近才切到 mesh

#### P3C-4 🔲（后续）
- .msh 格式兼容（移植 Mesh.cpp 解析器）+ glTF 2.0 新模型
- PBR 完整流程（贴图）+ 尾焰粒子系统 + 阴影

### P3D — HUD/MFD 完善 ✅（2026-07 完成）

#### P3D-1 飞行状态数据管线 ✅
- `orbitx-app/src/vessel.rs`：`UserVessel` + RK4 Kepler 传播器（父体引力）；
  默认地球 LEO 400km 圆轨道（v ≈ 7.67 km/s）；油门推进消耗燃料；
  4 tests（圆轨闭合 / 能量守恒 / 燃料消耗 / 默认参数）
- `orbitx-app/src/flight_calc.rs`：状态矢量 → 轨道要素（a, e, i, Pe/Ap, T, ε）+
  径向/水平速度分量 + Earth 指数大气（密度/动压/马赫）+ 姿态直读 + T/W 比；
  5 tests（LEO 一致性 / 椭圆轨道 / 大气模型示例值 / 姿态 / 推进）
- `app.rs`：每帧 vessel.propagate（高时间加速时子步）+ compute_flight_state 填充 FlightState

#### P3D-2 HUD 三模式细化 ✅
- **Orbit**：飞行路径梯（随 bank 旋转，负俯仰虚线 + 端部钩 + 数字）+
  Prograde/Retrograde 向量标 + 左右速度/高度侧带（大刻度 + 中心指针框）+
  左上 REF/Pe/Ap/Ecc/Inc/T + 右上 E/m/fuel/thr/T-W + 底部轨道分类
- **Surface**：滚动地平线（bank + pitch 驱动）+ 俯仰梯 +
  顶部航向带（±45° 视窗、N/E/S/W 大刻度、当前航向大字）+
  滚转指示器（顶部弧线 + 三角指针）+ 侧速/高带 + V/S + Mach/q/ρ
- **Docking**：目标框（4 角刻度 + 内环 + 十字）+ 相对速度矢量线 +
  RNG/V∥/V⊥/TGO/STAT + 姿态占位
- 顶栏统一：模式名 + 时间加速

#### P3D-3 四核心 MFD 细化 ✅
- **Orbit**：数值列 + 真实椭圆（96 段折线，焦点原点，中心 a·e 偏移）+
  Pe/Ap 标注 + 由 focus_dist 反解真近点角 ν + 当前航天器亮点
- **Map**：等距圆柱投影（2:1）+ 30° 经纬网 + 赤道加粗 +
  前 60 min 地面轨迹（mean_motion + 恒星日修正）+ 当前位置亮点 + LON/LAT/HDG/SPD
- **Docking**：三层距离环 + 十字 + 5° 接近走廊 +
  (V⊥, V∥) 目标点（200 m/s 满量程）+ TGT/RNG/V∥/V⊥/TGO/STAT
- **Landing**：ILS 双针（垂直=航向偏差 ±10°，水平=下滑道 ±5°，参考 3°）+
  飞机中心符号 + 高度剖面 + NAV/ALT/ΔHDG/V-S/FPA/SPD/GS

**全工作区测试：orbitx-gfx-hud 12 tests、workspace 全绿。**

### P3E — 相机完善 ✅（2026-07 完成）

#### P3E-1 6 外部模式完整运行时切换 ✅
- `camera.rs` 新增 `cycle_ext_mode(forward, dirref_hint)`、`ext_mode_index()`、
  `current_dist()`、`set_dirref()`、`toggle_internal()`、`ExternalCamMode::name()` /
  `short_params()`
- `input.rs` 扩充：`CamModeSet(u8)`（数字键 1-6 直接选择模式）、
  `CamToggleInternal`（V）、`CamCycleDirref`（R）；Tab 循环、Shift+Tab 反向、G 快进 GroundObserver
- `app.rs::handle_action` 完整实现（跳到指定模式 / 循环参考天体 / 内外切换）

#### P3E-2 动态近平面 ✅
- `CameraSystem::update_with_radii(positions, radii, dt)`：
  near = clamp(min(cam→表面距离) × 1e-3, [0.1 m, 1e6 m])，
  保证近/远比 ≤ 1e15（对数深度可容忍范围内）
- app.rs 每帧从 `PlanetarySystem` 采集 `radius_m` 并传入相机
- 单元测试：`dynamic_near_plane_scales_with_altitude`

#### P3E-3 驾驶舱模式 ✅（GenericCockpit）
- V 键切换内外视图；`InternalCamMode` 已定义 3 种（GenericCockpit / Panel2D /
  VirtualCockpit），本阶段激活 GenericCockpit 简易叠加
- 半透明黑色边框 + 视窗轮廓 + "COCKPIT · GENERIC" 标签，明确"内视图"状态
- Panel2D / VirtualCockpit 需要仪表面板资源，留到 P3C（航天器渲染）随 mesh 一起做

#### P3E-4 右侧信息栏 ✅
- 显示 `Cam[EXT/INT] <ModeName>` + 模式参数摘要（dist/φ/θ 或 pos/gdir/lng-lat-alt 等）
- 显示动态 Near/Far 数值（便于验证近平面自适应）
- 操作提示补充：1-6 / V / R 键说明

**测试：orbitx-render 26 tests（新增 5 个：cycle_ext_mode_visits_all_six /
cycle_backward / toggle_internal_flips_state / dynamic_near_plane_scales_with_altitude /
set_dirref_updates_target_to_object）；workspace 全绿。**

### P3F — 集成优化 🟡（P3F-1/2/4 已完成，P3F-3 后续）

#### P3F-1 InputMap 配置化 ✅
- `orbitx-app/src/input.rs`：新增 `KeyMap` + `Action::from_name` + `parse_key` +
  `KeyMap::from_toml` / `baked_default` / `hardcoded_default` / `resolve()`
- 加载顺序：`$ORBITX_KEYBINDINGS` env → `$HOME/.config/orbitx/keybindings.toml` →
  编译时嵌入 `assets/keybindings.toml`
- TOML 语法极简：`KeyName = "ActionName"`；支持整行与行尾 `#` 注释；
  未识别键/动作静默丢弃保留默认（无 panic）
- `App::key_map` 字段 + 事件循环调用 `key_map.get(key)` 替代原静态 `key_to_action`
- 单元测试：6 tests（action_from_name / parse_key / baked_default / from_toml
  / hardcoded_fallback 覆盖）

#### P3F-2 视锥剔除 ✅
- `FrameScene::from_scene`：behind-camera 剔除
  `dot(pos_render, forward_render) + scale < 0 → skip`
- 保留 `scale` 边缘作安全余量；侧向裁切留给 GPU 硬件（14 天体规模 CPU 侧收益微小）

#### P3F-3 🔲（后续）
- 实例化渲染（consolidate per-body draw calls → indirect / instanced）
- 目前 per-draw uniform pool + N ≤ 30 draw calls 的 CPU 开销不是瓶颈；
  这一步需重写 shader + 管线，风险高、收益小，故暂缓

#### P3F-4 文档 ✅
- `docs/KEYBINDINGS.md`：完整键位速查 + TOML 自定义示例 +
  动作/KeyCode 枚举清单
- `docs/RENDERING.md`：渲染栈架构（wgpu + egui CallbackTrait、CoordinateBridge、
  相机 6+3 模式、动态近平面、每 draw uniform pool、双 BindGroup、
  atmosphere/cloud/ring 层次、太阳双管线、视锥剔除、常见"看不见"7 大 bug）
- `README.md`：新增"Running the main app"节；Demos 表加入 `orbitx-app`；
  Roadmap 更新为 P3 各阶段状态

**测试：orbitx-app 25 tests（新增 6 个 input 测试）；workspace 无回归；
runtime smoke 正常启动。**

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
