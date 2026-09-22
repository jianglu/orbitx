# orbitx TOML 配置结构

orbitx 使用自有 TOML 描述火箭类、场景、天体与太阳系。本文档说明**航天器相关**的 `rocket.toml` 与 `scenario.toml` 字段规则，并列出仓库内预制航天器。

权威类型定义见：

- [`crates/orbitx-config/src/rocket/mod.rs`](../crates/orbitx-config/src/rocket/mod.rs)
- [`crates/orbitx-config/src/scenario.rs`](../crates/orbitx-config/src/scenario.rs)
- [`crates/orbitx-config/src/body.rs`](../crates/orbitx-config/src/body.rs)（大气 `model`）
- 预设文件：[`crates/orbitx-config/presets/`](../crates/orbitx-config/presets/)

本格式为 orbitx 原生 TOML，与 Orbiter 的 `.cfg` / `.scn` **不兼容**。

## 四类配置一览

| 文件角色 | Rust 类型 | 说明 |
|----------|-----------|------|
| `rocket.toml` | `RocketConfig` | 火箭类：级结构、质量、推力、TVC |
| `scenario.toml` | `ScenarioConfig` | 场景：时间、焦点、相机、飞船实例 |
| `body.toml` | `BodyConfig` | 天体物理参数（见 `orbitx-config/src/body.rs`，本文不展开） |
| `system.toml` | `SystemConfig` | 太阳系树（见 `orbitx-config/src/system.rs`，本文不展开） |

航天器组织 = **火箭类定义**（`RocketConfig`）+ **场景中的飞船实例**（`ShipConfig`，通过 `class` 引用火箭类）。

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

### CLI 内置别名

`cargo run -p orbitx-cli -- <alias>`：

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
