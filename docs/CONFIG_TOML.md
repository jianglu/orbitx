# orbitx TOML 配置结构

配置类型跨 orbitx 与 Godot，实现仍分属各自目录。下表是命名约束。`rocket.toml` / `scenario.toml` / `system.toml` 作为公开分类名退役，不另起第三套名字。下文仍记录**当前**火箭预设与旧混合场景文件的字段，直到对应新路径落地。`launch_attitude` 是代码里的起飞姿态函数，不是配置文件。

本格式为 orbitx 原生 TOML，与 Orbiter 的 `.cfg` / `.scn` **不兼容**。

## 配置文件类型

| 类型 | 路径 | 谁加载 | 内容 |
|------|------|--------|------|
| 航天器仿真 | `sc_xxx/sim.toml` | orbitx | 级、质量、推进、对接 |
| 航天器展示场景 | `sc_xxx/scene.toml` | Godot | 场景里加载的模型 |
| 航天器设计 | `sc_xxx/design.toml` | Godot | 设计器里加载的模型 |
| 工作流 | `sc_xxx/wf_<t\|s>_xxx.toml` | orbitx | `t` = TargetWorkFlow，`s` = SuperWorkFlow |
| 环境 | `scenario_xxx.toml` | orbitx 与 Godot | 行星系统：天体、自转、历表或固定位置、力学是否参与 |
| 发射台 | `lp_xxx.toml` | orbitx 与 Godot | 发射台 |
| 任务 | `task_xxx.toml` | 仅 Godot | 选用哪份航天器、发射台、环境，以及任务目标 |

本阶段只实现环境文件。字段与步进见 [`ENVIRONMENT.md`](ENVIRONMENT.md)。默认别名 `earth` 指向 [`crates/orbitx-config/presets/scenario_earth.toml`](../crates/orbitx-config/presets/scenario_earth.toml)。任务、发射台本轮不实现；`sc_xxx/` 三文件由 sim-rocket 侧 `AssemblyExporter` 投影写出（见下节与 [`sim-rocket/docs/assembly_model.md`](../../sim-rocket/docs/assembly_model.md)），仓库里的火箭预设仍由 `RocketConfig` 加载。

### 用户数据根（Godot `user://`）

sim-rocket 工程设置 `application/config/use_custom_user_dir=true`、`custom_user_dir_name="WLCY/SimRocket"`。`user://` 展开为：

- macOS：`~/Library/Application Support/WLCY/SimRocket`
- Windows：`%APPDATA%\WLCY\SimRocket`
- Linux：`~/.local/share/WLCY/SimRocket`

`user://` 是项目唯一根 `{UserData}/WLCY/SimRocket`，编辑器与正式包读同一份；禁止再写 `user://WLCY/SimRocket/...`（会叠成 `SimRocket/WLCY/SimRocket`），也禁止使用 Godot 默认 `app_userdata/<工程名>`。

```text
{UserData}/WLCY/SimRocket/          # Godot user://；custom_user_dir_name
├── settings/                       # 显示等设置，含 display.cfg
└── spacecraft/sc_<class>/
    ├── design.toml                 # 设计拓扑（唯一可手改、可回读源）
    ├── scene.toml                  # 飞行场景拓扑（按级聚合，渲染用）
    └── sim.toml                    # orbitx 仿真描述（RocketConfig 形状）
```

`class` 首次保存时生成 8 位十六进制（目录为 `spacecraft/sc_<class>/`），改显示名不换目录。工作流 `wf_*.toml`、任务 `task_*.toml`、发射台 `lp_*.toml` 本轮不生成。

### 配置文件版本通用规则

工作区所有 TOML 配置文件（含 `design.toml` / `scene.toml` / `sim.toml`、`scenario_*.toml`、`wf_*.toml`、`lp_*.toml`、`task_*.toml`、`display.cfg` 等）都必须带 `schema` 字段表示文件版本。读写方统一按以下三档加载：

- `schema` 等于当前支持版本：正常读。
- `schema` 小于当前版本（老版本文件）：按已知迁移规则迁到当前版本再读；迁移失败则拒绝并提示。
- `schema` 大于当前版本（文件比软件新）：拒绝加载，UI 提示「文件版本更高，请升级程序后再读取」。
- `schema` 缺失：视为非法文件，拒绝加载。本轮尚未发布，不背无版本旧文件包袱。

本轮 `schema = 1` 是首个版本，多数文件无实际迁移规则；实现时预留版本比较与迁移入口的代码路径，后续升版本时补迁移表。该规则是配置文件通用约束，不只适用于航天器三文件。

权威类型：

- [`crates/orbitx-config/src/rocket/mod.rs`](../crates/orbitx-config/src/rocket/mod.rs)
- [`crates/orbitx-config/src/planetary_scenario.rs`](../crates/orbitx-config/src/planetary_scenario.rs)
- [`crates/orbitx-config/src/body.rs`](../crates/orbitx-config/src/body.rs)
- 预设：[`crates/orbitx-config/presets/`](../crates/orbitx-config/presets/)

航天器组织（当前预设，对应未来 `sc_xxx/sim.toml`）= **火箭类定义**（`RocketConfig`）+ 运行时放置。旧 `ScenarioConfig` 混合文件不再作为环境入口。

```
RocketConfig.stages[]  ──►  StageSpec  ──►  Vessel
ShipConfig.class       ──►  选择哪个 RocketConfig
Assembly               =  Vec<Vessel>（底→顶）+ active
```

---

## rocket.toml

### 顶层 `RocketConfig`

```toml
name = "Falcon 9"
class = "Falcon9"

[[stages]]
# ... StageConfig，可重复多次
```

| 字段 | 类型 | 必填 | 描述 |
|------|------|------|------|
| `name` | string | 是 | 火箭名称 |
| `class` | string | 是 | 类名；场景 `ShipConfig.class` 引用此值 |
| `stages` | `StageConfig` 数组 | 是 | 级列表；同轴时通常底→顶；侧挂助推可插在列表中由 `dock_links` 连接 |
| `dock_links` | `DockLinkConfig[]`? | 否 | 显式对接边；缺省则运行时对相邻级做顶口↔底口自动对接 |

API：`RocketConfig::from_toml_str` / `to_toml_string` / `from_file` / `to_file`。

### 单级 `StageConfig`

| 字段 | 类型 | 必填 | 单位 | 默认 | 描述 |
|------|------|------|------|------|------|
| `name` | string | 是 | — | — | 级名称 |
| `dry_mass` | float | 是 | kg | — | 空重（不含燃料） |
| `fuel_mass` | float | 是 | kg | — | 燃料质量 |
| `thrusters` | `ThrusterConfig[]` | 否 | — | `[]` | 推进器列表；有动力级非空，载荷为空 |
| `length` | float | 是 | m | — | 级长度 |
| `radius` | float | 是 | m | — | 级半径 |
| `separation_impulse` | float | 是 | m/s | — | 分离时施加的脉冲速度 |
| `inertia` | `[Ixx, Iyy, Izz]` | 否 | kg·m² | 圆柱体公式推断 | 主惯量张量对角线，真实惯量（非归一化） |
| `tidaldamp` | float | 否 | — | `0` | 重力梯度阻尼（Orbiter `tidaldamp`） |
| `cd_mach` | `[[mach, cd], …]` | 否 | — | 运行时默认火箭表 | 轴向阻力 Cd(M) 查表 |
| `docks` | `DockConfig[]`? | 否 | — | 自动顶/底 | 自定义对接口 |

**已删除（无兼容路径）**：级级 `thrust` / `isp` / `engine_pos` / `engine_dir` / `max_gimbal*`。推进一律写在 `[[stages.thrusters]]`。

### 单台 `ThrusterConfig`（`[[stages.thrusters]]`）

| 字段 | 类型 | 必填 | 单位 | 默认 | 描述 |
|------|------|------|------|------|------|
| `pos` | `[x,y,z]` | 是 | m | — | 体坐标系位置 |
| `dir` | `[x,y,z]` | 是 | — | — | 推力方向（单位向量） |
| `thrust` | float | 是 | N | — | **真空**最大推力 |
| `isp` | float | 是 | s | — | **真空**比冲 |
| `thrust_sl` | float? | 否 | N | — | 海平面推力（与 `isp_sl` 用于推导 `pfac`） |
| `isp_sl` | float? | 否 | s | — | 海平面比冲 |
| `max_gimbal` | float | 否 | rad | `0` | TVC 最大偏转角 |
| `max_gimbal_rate` | float | 否 | rad/s | `0` | TVC 最大偏转角速率 |
| `gimbal_axis` | `[x,y,z]` | 否 | — | `[1,0,0]` | TVC 偏转轴 |
| `throttle_rate` | float | 否 | 1/s | `0` | 节流斜坡最大速率（开度分数/秒）；`0` = 瞬时。液体缺公开数据时预设用标准代理 `0.8`（≈ CECE）；**固体**用 `0`（开/关 only） |

加载时由真空/海平面双点收成 Orbiter 式 `pfac`：`Isp(p)=Isp₀·(1−p·pfac)`。仅真空级可省略双点（`pfac=0`）。

### `DockConfig`

| 字段 | 类型 | 描述 |
|------|------|------|
| `pos` | `[x,y,z]` | 体坐标系口位置 [m] |
| `dir` | `[x,y,z]` | 接近方向（单位向量，指向外） |
| `rot` | `[x,y,z]` | 滚转对齐参考（单位向量） |

### `DockLinkConfig`（火箭顶层 `dock_links`）

| 字段 | 类型 | 描述 |
|------|------|------|
| `stage` | usize | 本侧级在 `stages` 中的下标 |
| `port` | usize | 本侧端口下标 |
| `remote_stage` | usize | 对方级下标 |
| `remote_port` | usize | 对方端口下标 |

### 约定

- **体坐标**：火箭纵轴沿 **+Y（朝顶）**。发动机通常在底部（`pos.y < 0`），`dir` 常为 `[0, 1, 0]`。
- **多机**：每台发动机一条 `[[stages.thrusters]]`；**侧挂助推应单独成级 + `dock_links`**（见 CZ-2F）。
- **侧挂**：径向 `dir` 的 `docks` + `dock_links`；运行时走 Dock→组合体刚体（`Assembly`）。
- **载荷级**：`thrusters = []`（及通常 `fuel_mass = 0`）。
- **惯量**：配置里的 `inertia` 是真实惯量；加载到 `Vessel` 后会归一化为 PMI（m²）。
- **大气**（`body.toml` / `AtmosphereConfig`）：`model = "us76" | "exponential" | "none"`；地球默认 `us76`。

### 模板

```toml
name = "Example Rocket"
class = "ExampleRocket"

[[stages]]
name = "S1"
dry_mass = 1000.0
fuel_mass = 5000.0
length = 10.0
radius = 1.0
separation_impulse = 2.0
cd_mach = [
  [0.0, 0.30],
  [0.9, 0.55],
  [1.1, 0.95],
  [5.0, 0.35],
]

[[stages.thrusters]]
pos = [0.0, -5.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 110000.0      # 真空
isp = 310.0
thrust_sl = 100000.0   # 海平面（推导 pfac）
isp_sl = 280.0
max_gimbal = 0.087
max_gimbal_rate = 0.17
throttle_rate = 0.8    # [1/s] 液体标准代理；固体用 0
```

---

## scenario.toml

### 顶层 `ScenarioConfig`

```toml
[environment]
system = "Sol"
mjd = 52345.5

[focus]
ship = "Falcon-9"

[camera]   # 可选
[hud]      # 可选

[[ships]]
# ... ShipConfig，可重复多次
```

| 字段 | 类型 | 必填 | 描述 |
|------|------|------|------|
| `environment` | `Environment` | 是 | 模拟环境（行星系与起始时间） |
| `focus` | `Focus` | 是 | 相机焦点（跟随的飞船） |
| `camera` | `CameraConfig` | 否 | 相机配置 |
| `hud` | `HudConfig` | 否 | HUD 配置 |
| `ships` | `ShipConfig` 数组 | 是 | 飞船列表 |

### `Environment` / `Focus`

| 结构 | 字段 | 类型 | 单位 | 描述 |
|------|------|------|------|------|
| Environment | `system` | string | — | 行星系名称（如 `"Sol"`） |
| | `mjd` | float | MJD | 模拟开始时间 |
| Focus | `ship` | string | — | 相机跟随的飞船名称；须匹配某个 `ships[].name` |

### `CameraConfig`（可选）

| 字段 | 类型 | 默认 | 单位 | 描述 |
|------|------|------|------|------|
| `target` | string | （必填） | — | 相机目标（天体或飞船名） |
| `mode` | string | `"external"` | — | 相机模式 |
| `distance` | float | `300` | m | 距离 |
| `azimuth` | float | `0` | rad | 方位角 |
| `elevation` | float | `0` | rad | 仰角 |
| `fov` | float | `45` | deg | 视场角 |

### `HudConfig`（可选）

| 字段 | 类型 | 描述 |
|------|------|------|
| `mode` | string | HUD 模式：`"surface"` \| `"orbit"` \| `"docking"` |

### `ShipConfig`

场景**不内嵌**级参数；通过 `class` 选择火箭类文件。

| 字段 | 类型 | 条件 | 单位 | 描述 |
|------|------|------|------|------|
| `name` | string | 必填 | — | 飞船名称 |
| `class` | string | 必填 | — | 类名（对应 `RocketConfig.class`） |
| `status` | string | 必填 | — | 状态：`"landed"` \| `"orbiting"` |
| `body` | string | 必填 | — | 参考天体 |
| `longitude` | float? | landed | deg | 着陆经度 |
| `latitude` | float? | landed | deg | 着陆纬度 |
| `heading` | float? | landed | deg | 着陆朝向 |
| `altitude` | float? | landed | m | 着陆地面高度 |
| `rpos` | `[x,y,z]`? | orbiting | m | 轨道位置（相对参考天体） |
| `rvel` | `[x,y,z]`? | orbiting | m/s | 轨道速度 |
| `arot` | `[x,y,z]`? | 可选 | deg | 姿态欧拉角 |
| `fuel_level` | float[]? | 可选 | 0..1 | 各燃料罐/级液位（按级顺序） |
| `dock_info` | `DockInfo[]`? | 可选 | — | 对接信息列表 |

### `DockInfo`

```toml
dock_info = [
  { port = 0, remote_port = 1, vessel = "ISS" },
]
```

| 字段 | 类型 | 描述 |
|------|------|------|
| `port` | u32 | 本方端口索引 |
| `remote_port` | u32 | 对方端口索引 |
| `vessel` | string | 对方飞船名称 |

---

## 配置 → 运行时边界

| 配置侧 | 运行时（`orbitx-vessel`） |
|--------|---------------------------|
| `RocketConfig` | 无直接类型；展开为 `Vec<StageSpec>` |
| `StageConfig` | `StageSpec` → `Vessel` |
| `thrust` + 引擎几何 | **一个**带 TVC 的主推（`thrust > 0`）；否则无发动机 |
| `inertia`（kg·m²） | `Vessel.pmi`（归一化 m²） |
| （自动） | 按 `length` 生成顶/底对接端口 |
| （无） | 多储箱、RCS、气动、触地——仅 API / 代码配置 |

转换发生在消费方（如 `orbitx-cli`），不在 `orbitx-config` crate 内。

`fuel_level` 表达场景意图；是否写回 `Vessel.fuel_mass` / tanks 由应用层负责。

---

## 预制航天器参考

预设目录：[`crates/orbitx-config/presets/`](../crates/orbitx-config/presets/)。

### TOML 火箭类

| 文件 | name | class | 级数 | 级名（底→顶） | 一级推力 / Isp | 直径 (2r) | TVC | 备注 |
|------|------|-------|------|---------------|----------------|-----------|-----|------|
| `falcon9.toml` | Falcon 9 | `Falcon9` | 3 | F9-S1, F9-S2, Payload | 7.61 MN / 282 s | 3.7 m | 有 | 与 vessel Rust 预设对齐 |
| `saturn_v.toml` | Saturn V | `SaturnV` | 4 | S-IC, S-II, S-IVB, CSM-LM | 34.5 MN / 263 s | 10 m | 有 | 与 vessel Rust 预设对齐 |
| `long_march_2f.toml` | 长征二号F | `LongMarch2F` | 7 | CZ2F-S1, S2, Shenzhou, 4×Booster | 芯 2.962 MN + 助推 4×0.740 MN | 芯 6.7 m | 有 | Y 型公开资料近似；侧挂 Dock；见下方数据来源 |
| `long_march_5.toml` | 长征五号 | `LongMarch5` | 3 | CZ5-S1, CZ5-S2, Payload | 12 MN / 300 s | 10 m | 有 | 助推折入一级 |
| `long_march_7.toml` | 长征七号 | `LongMarch7` | 3 | CZ7-S1, CZ7-S2, Tianzhou | 7.2 MN / 310 s | 6.7 m | 有 | 载荷=天舟 |
| `long_march_9.toml` | 长征九号 | `LongMarch9` | 4 | CZ9-S1, CZ9-S2, CZ9-S3, Payload | 30 MN / 315 s | 10 m | 有 | 估算/规划参数 |

### 场景预制

| 文件 | 飞船实例 | class | status | 说明 |
|------|----------|-------|--------|------|
| `launch_scenario.toml` | `Falcon-9` | `Falcon9` | landed（赤道） | `fuel_level = [1.0, 1.0, 0.0]` |

### Rust 运行时预设（非 TOML）

见 [`crates/orbitx-vessel/src/presets.rs`](../crates/orbitx-vessel/src/presets.rs)：

| API | 对应 class | 说明 |
|-----|------------|------|
| `presets::falcon9()` | `Falcon9` | 数值与 `falcon9.toml` 基本一致；单元测试主用 |
| `presets::saturn_v()` | `SaturnV` | 同上 |
| `configure_default_aero` | — | 为 Vessel 填阻力等；**无 TOML 字段** |

长征系列**没有** `presets::long_march_*()`，仅通过 TOML / CLI 加载。

改 TOML **不会**自动更新 `orbitx-vessel/presets.rs`；维护 Falcon 9 / Saturn V 时需两边对照。

### CLI / Runtime 内置别名

`cargo run -p orbitx-cli -- <alias>` 与 `cargo run -p orbitx-runtime -- --rocket <alias>` 共用 [`orbitx-config`](../crates/orbitx-config/src/rocket/builtin/mod.rs) 别名表：

| 别名 | 嵌入的 TOML |
|------|-------------|
| `falcon9` | `falcon9.toml` |
| `saturnv` | `saturn_v.toml` |
| `lm5` | `long_march_5.toml` |
| `lm2f` | `long_march_2f.toml` |
| `lm7` | `long_march_7.toml` |
| `lm9` | `long_march_9.toml` |

复现测试覆盖 Falcon 9 / Saturn V 的 TOML → 轨迹全链路。

### 长征二号F 数据来源与控制分层

`long_march_2f.toml` 取 **CZ-2F 载人 Y 型**公开汇总（起飞质量约 479.8 t、起飞推力约 5923 kN），非遥测级。主要出处：

1. [国家航天局 · 长征二号F](https://www.cnsa.gov.cn/n6758824/n6759008/n6759011/c6794041/content.html)（总体质量、尺寸）
2. [中国航天科技集团公开介绍](https://m.spacechina.com/n146/n238/n12985/c3961203/content.html)
3. [中文维基 · 长征二号F运载火箭](https://zh.wikipedia.org/zh-hans/长征二号F运载火箭)（分级干/总重、YF-20 / YF-24B）

逃逸塔/整流罩未单独成 vessel，与官方总重差额并入载荷干重。Orbiter 树内无 CZ-2F 配置。

**油门组合与分离顺序**由上层控制决定（物理层仅单船 `set_throttle` / `undock`）。`orbitx-cli` 过渡策略：`SyncPrimary` 按对接图同步 lit（`active` ∪ 侧挂有推叶；同轴非 lit 有推油门为 0，分离后随 `active` 切换）；侧挂叶优先分离。`active` 仍为单主控指针，侧挂点火显示为 `FIRING` 而非第二个 `ACTIVE`。

规划中的 **`orbitx-controller`**（经 **Runtime** 调度）承接此类逻辑；产品四档（手飞 / 目标导向 / TargetWorkFlow / SuperWorkFlow）见 [`ARCHITECTURE.md`](ARCHITECTURE.md)。cli `control` 仅为过渡实现。

### 建模注意

- 侧挂助推：用 `docks` + `dock_links` 建独立 vessel（CZ-2F）；长征五号等仍可暂时折进一级。
- 末级载荷：`thrusters = []`，只保留质量与外形。
- 双数据源：`Falcon9` / `SaturnV` 同时存在于 TOML 与 Rust `presets`。

---

## 完整示例

### `falcon9.toml`（节选）

```toml
name = "Falcon 9"
class = "Falcon9"

[[stages]]
name = "F9-S1"
dry_mass = 25600.0
fuel_mass = 411000.0
length = 47.0
radius = 1.85
separation_impulse = 3.0
# 9×Merlin：见 presets/falcon9.toml 中完整 thrusters 列表
[[stages.thrusters]]
pos = [0.0, -23.5, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 914000.0
isp = 311.0
thrust_sl = 845000.0
isp_sl = 282.0
max_gimbal = 0.122
max_gimbal_rate = 0.35

[[stages]]
name = "F9-S2"
dry_mass = 4000.0
fuel_mass = 107500.0
length = 14.0
radius = 1.85
separation_impulse = 2.0
[[stages.thrusters]]
pos = [0.0, -7.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 934000.0
isp = 348.0
max_gimbal = 0.087
max_gimbal_rate = 0.17

[[stages]]
name = "Payload"
dry_mass = 22800.0
fuel_mass = 0.0
length = 5.0
radius = 1.85
separation_impulse = 1.0
thrusters = []
```

### `launch_scenario.toml`（节选）

```toml
[environment]
system = "Sol"
mjd = 52345.5

[focus]
ship = "Falcon-9"

[camera]
target = "Earth"
mode = "external"
distance = 300.0
azimuth = 0.0
elevation = 0.3
fov = 45.0

[hud]
mode = "surface"

[[ships]]
name = "Falcon-9"
class = "Falcon9"
status = "landed"
body = "Earth"
longitude = 0.0
latitude = 0.0
heading = 90.0
altitude = 2.5
fuel_level = [1.0, 1.0, 0.0]
```

---

## 控制工作流（Control Workflow）

`orbitx-controller`（P4.1）落地两档工作流 TOML：`TargetWorkFlow`（模式 c，简易目标导向）与
`SuperWorkFlow`（模式 d，复杂自动控制）。权威类型与解析见
[`crates/orbitx-controller/src/workflow/mod.rs`](../crates/orbitx-controller/src/workflow/mod.rs)；
四档控制分层与产品闭环见 [`ARCHITECTURE.md`](ARCHITECTURE.md) 与 [`CONTROLLER.md`](CONTROLLER.md)。

### `TargetWorkFlow`（`kind = "target"`）

按 `[[phases]]` 序列化目标 + 过渡条件；满足过渡即进入下一阶段。末段可不写 `transition`（永驻）。

```toml
kind = "target"
name = "falcon9-ascent"

[[phases]]
mode = "vertical_hold"
throttle = 1.0
transition = { altitude_gt = 10000.0 }

[[phases]]
mode = "gravity_turn"
throttle = 1.0
kick_pitch = 0.0
kick_yaw = 0.087
kick_rate = 0.05
transition = { altitude_gt = 80000.0 }

[[phases]]
mode = "prograde_hold"
throttle = 1.0
```

`mode` 取值与字段：

| mode | 字段 | 说明 |
|------|------|------|
| `vertical_hold` | `throttle` | 保竖直（pitch/yaw 目标 = 0） |
| `pitch_to` | `pitch`, `yaw`, `throttle` | 朝指定俯仰/偏航角 [rad] |
| `prograde_hold` | `throttle` | 沿速度方向 |
| `retrograde_hold` | `throttle` | 反速度方向 |
| `gravity_turn` | `throttle`, `kick_pitch`, `kick_yaw`, `kick_rate`, `min_alt`, `min_speed` | 标准重力转向。过 `min_alt`（默认 500 m）和 `min_speed`（默认 50 m/s）后，姿态指令沿 `(kick_pitch, kick_yaw)` 按 `kick_rate`（默认 0.05 rad/s）爬升，播种段用原来的 TVC 跟随该指令。地速在播种方向上的倾角为正（水平投影对垂直地速的 `atan2`）且机头到位后，仍用原来的 TVC 对齐地面速度，直到箭体接近水平。默认 `kick_pitch` 0、`kick_yaw` 约 +5°，在默认赤道台上朝当地向东。 |

`transition` **恰好一个**条件字段（解析时校验）：

| 字段 | 单位 | 满足条件 |
|------|------|----------|
| `altitude_gt` | m | 高度（距地表）> 值 |
| `speed_gt` | m/s | 速率 > 值 |
| `apoapsis_gt` | m | 远地点距中心体 > 值（需 Runtime 传 `mu`） |
| `periapsis_gt` | m | 近地点距中心体 > 值（需 Runtime 传 `mu`） |
| `fuel_pct_lt` | % | 燃料百分比 < 值 |
| `time_gt` | s | 当前阶段累计时长 > 值 |

### `SuperWorkFlow`（`kind = "super"`）

按 `[[steps]]` 序列命令执行器；即时命令（`throttle` / `tvc` / `rcs` / `separate`）执行一次后下一
tick 推进，`wait` 持续 `duration` 后推进。**P4.1 骨架**：完整的「子控制器舰队 + 连续姿态保持 +
分离派生子控制器 + 入轨自动驾驶」留待 P4.2+（见 [`ROADMAP.md`](ROADMAP.md)）。

```toml
kind = "super"
name = "falcon9-full"

[[steps]]
action = "throttle"
group = "Core"
level = 1.0

[[steps]]
action = "tvc"
group = "Core-tvc"
pitch = 0.0
yaw = 0.0

[[steps]]
action = "wait"
duration = 5.0

[[steps]]
action = "separate"
point = "Booster-sep-0"
```

`action` 取值与字段：

| action | 字段 | 说明 |
|--------|------|------|
| `throttle` | `group`, `level` | 设指定 throttle group（单船）油门；`level ∈ [0,1]` |
| `tvc` | `group`, `pitch`, `yaw` | 设指定 tvc 组目标角 [rad] |
| `rcs` | `group`, `axis`, `level` | 设指定 rcs 组；`axis ∈ {pitch, yaw, bank}` |
| `separate` | `point` | 执行分离点；分离后工作流自动重建主 caps |
| `wait` | `duration` | 等待 `duration` 秒后推进（`duration ≥ 0`） |

`group` / `point` 的 id 由 `ControlCapability` 从 `Assembly` 自动派生（`{vessel}` / `{vessel}-tvc` /
`{vessel}-{group_type}` / `{vessel}-sep-{port}` 等），见 [`CONTROLLER.md`](CONTROLLER.md)。

---

## 航天器三文件（`sc_<class>/`）

由 sim-rocket 侧 `AssemblyExporter` 从设计场景的火箭树投影写出（见 [`sim-rocket/docs/assembly_model.md`](../../sim-rocket/docs/assembly_model.md)）。三文件头部都写 `schema = 1`、`name`、`class`，必须一致。版本加载规则见上「配置文件版本通用规则」。

`design.toml` 是唯一可手改、可回读的源；`scene.toml` 与 `sim.toml` 在确认保存时由同一棵火箭树投影，不单独编辑。

### 分层

`orbitx-cli` 今天的角色 = 未来 Godot 侧 `sim-rocket/rust` 的角色：会话控制、装配投影、读写用户 TOML。Godot（GDScript）只做 UI 与场景节点；三文件投影都在 Rust 层完成。`sim.toml` 字段形状与 `RocketConfig` 对齐，由 sim-rocket 侧序列化写出，供以后独立进程的 orbitx 读取；不在 GDScript 里手写级投影或 TOML 拼装。

### `design.toml`

设计工坊回读用。每条实例对应一个 `PartController`，主键是 `instance_id`。

- 位姿：`origin` + `basis`（3×3），根缩放不存（恒为 1）。
- 缩放：`PartController.get_scale_params()` 全套（`scale`、`scale_mode`、三轴系数、`top_radius_factor` / `bottom_radius_factor` / `bottom_socket_layout`）。直接读控制器，不只存 `Visual.scale`，否则回读可变截面和底口布局会丢。
- `attach`：`mode` / `parent_instance_id` / `target_id` / `child_port` / `theta` / `height`。
- `symmetry_groups`：现有组表（成员、主件、count、lattice）。
- `components`：六个能力的 `to_dict()` 全量，供 Inspector 回读。`throttle_opening`、`gimbal_angle`、`level` 是运行时控制量，写入时固定回默认（1 / 0 / 0），不把飞行状态冻进设计文件。
- `rocket_root_instance_id`。

回读：清空 `PartsRoot`，按 catalog 实例化，`assign_instance_id` 用存档 UUID，套用缩放、位姿、attach、对称组、组件字段。

### 级的切分（scene 与 sim 共用）

只走火箭树，不走场景父子节点。

- `separator_stack` 且 `marks_stage_boundary`：边界下方（含这只分离器）为下一级，上方为上一级。没有级间分离器的多段箭体并成一级。
- `booster_*` 各自成级。径向分离器（连接芯级与助推的那只件）的质量归芯级（父级）；该分离器承载的 `separation_impulse` 投影到它分离出去的助推级上——分离时给子级一个推开脉冲，避免与父级碰撞。
- 发动机、翼面、整流罩、载荷、伞不单独成级，并入所挂箭体的那一级。
- 级坐标原点取该级满载质心（`dry_mass×dry_center + fuel_mass×fuel_center`）。orbitx 单级没有质心字段，一份质量被视为集中在体坐标原点。`dry_mass`、`fuel_mass`、推力 `pos`、对接 `pos` 都写入现有字段；`dry_center` / `fuel_center` 不另开字段，只用来定这个原点。推力口、对接口和 `scene.toml` 里的位姿都相对它。燃料烧完后质心向干质心漂，orbitx 不算，保存时也不写。
- 级名按底→顶 `S1`、`S2`…，助推按方位角 `B1`、`B2`…。
- `separation_impulse` 是**分离器部件的能力字段**。被抛离的那一级携带它（与 `Assembly::separate_stage` 读底级脉冲一致：代码读 `bottom` 级的 `separation_impulse`）。最上级没有父级时脉冲为 0；径向分离器的脉冲赋给它分离出去的助推级，分离器自身质量留在芯级。
- **所有级都显式写 `docks`**，无对接需求的级写 `[]`。绝不靠 `StageConfig.docks` 缺省——缺省会按 `length` 在 ±length/2 自动生成顶/底口，而级原点在质心而非几何中心，自动生成的位置会错位。

### `scene.toml`

飞行沙盒按级挂模型用。**以级为单位**，不是逐件：同一级所有部件合成为一个大部件，包含该级能力的聚合（结构上类似 `sim.toml` 的级聚合）。目的：减少渲染精灵数量；支持级分离后作为独立整体渲染。不含吸附参数、对称组、逐件位姿（这些只在 `design.toml`）。

- `[[bodies]]`：`id` 与 sim 级名相同，`stage_index` 对齐 `sim.toml` 的 `stages` 下标。
- 级聚合外形：合并后的级局部 `origin` / `basis` / `scale`（来自该级各件缩放后外包），级模型/网格引用，级能力聚合（质量、外形尺寸、气动外形等渲染所需）。
- 本轮生成级聚合体并保证能被设计侧解析；沙盒里真正实例化留到飞行场景接入。

### `sim.toml`

字段与现有 `RocketConfig` 对齐，保存后 `RocketConfig::from_file` 能读。**文件保持可扩展**：当前引擎会读的字段必须写对；为后续 orbitx 预留的扩展段/数组可以先写出，serde 忽略未知字段，不会把现有加载读坏。设计里有、仿真配置还不读的能力不写进 `sim.toml`，只留在 `design.toml` 的 `components` 与 [`sim-rocket/docs/parts/orbitx_pending.md`](../../sim-rocket/docs/parts/orbitx_pending.md)（缺口建档，可追溯，与文件可扩展是两件事）。

已有字段，由部件投影（外形相关量先按缩放生效，再合成）：

- `dry_mass` / `fuel_mass`：级内 `Mass` / `Tank` 求和。质量按体积缩放（× `vmul`）。
- `[[stages.thrusters]]`：每台 `Engine` 一条；允许空数组。`pos`/`dir` 来自装配姿态（火箭类推力沿零件 +Y，喷口在 -Y）。`thrust`/`isp`/海平面双点/万向节/`throttle_rate` 来自该件 `Engine`。推力按面积缩放（× `sx·sz`），比冲不变，燃料质量流率随之按面积缩放。
- `length` / `radius`：该级各件缩放后的 `Mass.length_m` / `radius_m` 取外包。可变截面用缩放后的上下半径外包。必填，不参与惯量公式本身。
- `inertia`：缩放后的各件 `Mass.inertia_kg_m2` 合成，绕级质心、对齐级坐标轴。orbitx 加载时再除以总质量得到 PMI。
- `tidaldamp`：该级各件 `Mass.tidaldamp` 按总质量加权平均。默认 0。
- `docks` + 顶层 `dock_links`：由跨级 attach 口变到级坐标。栈式顶口↔底口，助推走侧口。
- `cd_mach`：缩放后的 `Aero.reference_area_m2` 加权合成整级表；阻力作用在级原点。

### 缩放对能力的生效规则

能力字段存的是静止基准（catalog 默认，无用户单独配置）；`design.toml` 另存 `scale` 全套。投影 `scene` / `sim` 时，所有量按物理量纲随该件自己的 `(sx, sy, sz)` 缩放生效后再合成（火箭类 Y 纵轴，X/Z 径向，`vmul = sx·sy·sz`）：

| 量 | 缩放 | 均匀 k |
|---|---|---|
| `length_m` | × `sy` | × k |
| `radius_m` | × `(sx+sz)/2` | × k |
| 米制位置偏移（`dry_center`/`fuel_center`/压心） | 各分量 × 对应轴 | × k |
| `reference_area_m2`、控制面面积 | × 面内两轴积（箭体截面 `sx·sz`） | × k² |
| `dry_mass` / `fuel_mass` | × `vmul` | × k³ |
| `inertia_kg_m2` `Iyy`（纵轴滚转） | × `vmul·sr²`，`sr=(sx+sz)/2` | × k⁵ |
| `inertia_kg_m2` `Ixx` | × `vmul·(sy²+sz²)/2` | × k⁵ |
| `inertia_kg_m2` `Izz` | × `vmul·(sx²+sy²)/2` | × k⁵ |
| 翼面升力/阻力（力） | × 面积比 | × k² |
| 引擎推力 `thrust` | × `sx·sz`（面积档，喷管喉部） | × k² |
| 引擎比冲 `isp` | 不变 | × 1 |
| 燃料质量流率 `ṁ` | × `sx·sz`（随推力） | × k² |
| `cd`/`cl` 无量纲系数 | 不变 | × 1 |
| `tidaldamp`/`throttle_rate`/万向节角与速率 | 不变 | × 1 |

推力 × k²、燃料量 × k³，燃烧时间 ∝ k³/k² = k，随放大线性变长。质量 × k³、推力 × k²，T/W ∝ 1/k，随放大下降——几何相似缩放的物理。

合成进 `stages.inertia`（满载，绕级质心，已缩放）：每件用缩放后的惯量，参考点是该件缩放后的满载质心，变到级坐标；对角惯量转到级坐标，平行轴挪到级原点，相加。orbitx 只收对角线；只写 `[Ixx, Iyy, Izz]`，惯量积丢掉。零件相对级轴有滚转时是近似，记在已知缺陷里。

### 已知缺陷（本轮不改步进）

- 燃料消耗只减 `fuel_mass`。质心不从满载位置漂向干质心，惯量也不变。保存进去的是加满时的惯量；orbitx 加载时用当时总质量归一化成 PMI，之后燃料减少 PMI 仍停在满载比值。
- 惯量合成只保留对角线，零件相对级轴滚转时丢掉惯量积。

### 缺口建档（与文件可扩展无关）

设计有而仿真尚未接、或两边零件不对齐的项，单独建档在 [`sim-rocket/docs/parts/orbitx_pending.md`](../../sim-rocket/docs/parts/orbitx_pending.md)，避免以后忘掉。它不是「禁止往 TOML 写扩展键」的理由；文件扩展性见上。本轮这些项仍留在部件能力与 `design.toml` 的 `components`（Inspector 回读要用）；`sim.toml` / `scene.toml` 是否预写对应扩展段，按该项是否已有明确落点决定——没有约定 schema 的先只记文档，有约定的可以先写出供以后消费。

