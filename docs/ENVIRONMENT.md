# orbitx 环境（`orbitx-environment`）

权威设计：行星系统的**状态与步进**。算法（引力、Pines、积分、刚体、自转公式、大气模型）留在 `orbitx-dynamics`。产品帧序见 [`RUNTIME.md`](RUNTIME.md)。

## 目标与边界

环境只加载 `scenario_xxx.toml`。天体表决定有哪些星、谁自转、谁随历表运动、谁进入力学。`GravBody` 只含 `dynamics = true` 的天体；画面画同一张表上的全部天体。

对照 Orbiter `Psys`：保留「统一 tick 下场进入积分器」。渲染不焊在天体对象上，由宿主读 `bodies()`。

**不做**（本阶段不改这些行为）：

- 任务文件 `task_xxx.toml`、发射台 `lp_xxx.toml`、航天器目录 `sc_xxx/`。
- 天气、风场、时变大气、磁场、辐射、光压。
- 天体速度改正、木星/土星卫星质心偏移。`Vsop87B.ear` 已是地心，不做质心平移。
- 改 tick 序；改 `StepEnv` 签名。大气、恒星周期、行星半径仍是 `Assembly` 字段，每 tick 从 `primary_surface` 刷新。
- FlightRecorder `environment` 落盘 schema（P6）。
- 台架钉死/松绑（Runtime 会话策略）。坠毁 / 地表注入（G4 仍光滑球；P5.1 见 [`SURFACE_RUNTIME.md`](SURFACE_RUNTIME.md)）。
- Godot 加载实现（P4.5）；scenario / zenoh 下发地表路径（P5.1）。
- **不**在 environment 内根据火箭位置做地表预取（调度权威在 Runtime）。

依赖单向：

```text
orbitx-surface     → （独立读库；无 dynamics；P5.1 由 Runtime 并列持有并调度）
orbitx-environment → orbitx-dynamics / orbitx-ephemeris / orbitx-config / orbitx-math
                     （G4 过渡仍可依赖 surface 做 attach_surface；P5.1 目标不再由 env 驱动缓存）
orbitx-runtime     → orbitx-environment（World）+ orbitx-surface（地表工作集）
orbitx-app         → orbitx-environment
orbitx-vessel      → 只消费 StepEnv / GravBody / Runtime 注入的地表查询（不依赖 surface）
```

## 公开类型

运行类型仍叫 `PlanetarySystem`（从 dynamics 迁出）。配置类型在 `orbitx-config`：`PlanetaryScenario`。旧 `orbitx_config::Environment`（`system` + `mjd`）属于退役的混合场景文件，不作为运行环境。

| 类型 | 位置 | 职责 |
|------|------|------|
| `PlanetaryScenario` | `orbitx-config` | `scenario_xxx.toml` |
| `PlanetarySystem` | 本 crate | 运行中的天体表、MJD、步进 |
| `CelestialBody` | 本 crate | 一颗星：位置、自转、历表、力学开关、大气配置 |
| `GravityModel` / `EphemerisModel` | 本 crate | 已加载的重力与历表 |
| `PrimarySurface` | 本 crate | 主天体半径、自转周期、大气；G4 可选 `elev_radius_m` / `contact_height` / `ground_radius` |
| `SurfaceBinding` | 本 crate | `attach_surface` 打开的 `orbitx-surface` 包句柄 |
| `GravBody` | `orbitx-dynamics` | 积分器消费的引力体 |

## 生命周期

```rust
impl PlanetarySystem {
    pub fn load(scenario: &PlanetaryScenario, data_root: &Path) -> Result<Self, String>;
    pub fn update(&mut self);
    pub fn advance(&mut self, dt_days: f64);
    pub fn grav_bodies(&self) -> Vec<GravBody>;
    pub fn primary_surface(&self) -> PrimarySurface;
    pub fn attach_surface(&mut self, root: impl AsRef<Path>) -> Result<(), String>;
    pub fn bodies(&self) -> &[CelestialBody];
}
```

`load` 按 toml 建树。`dynamics = false` 的天体仍在 `bodies` 里并更新位置，但不进入 `grav_bodies`，也不提供大气。`rotation` 缺省或 `enabled = false`：转角冻在初值，地面风速为 0（`sid_rot_period = 0`）。没有 `ephemeris` 时位置锁在 `fixed_pos`（有父体则相对父体，否则绝对）。`advance` 只推进 MJD。

### G4 地表包门面（`attach_surface`，过渡）

G4 为可测门面：宿主可对 `PlanetarySystem` 调用 `attach_surface`。**P5.1 权威调度**见 [`SURFACE_RUNTIME.md`](SURFACE_RUNTIME.md)（Runtime 并列持有 surface、分频预取/注入）；env **不**根据火箭位置预测缓存。

- `|R_elev_m − primary.size| ≤ 0.5 m`，否则 `Err`（防 10 m 硬编码错位；datum 协作仍需要）。
- 绑定后 `PrimarySurface::elev_radius_m = R_elev_m`，`ocean_surface_m` 只读包常量；`contact_height` / `ground_radius` **转发** `orbitx-surface`（不在 env 内重写插值）。
- 未绑定：`elev_radius_m == radius == body.size`，`contact_height ≡ 0`，`ground_radius == size`。
- G4：**不**改 `crash.rs` / 触点入环。P5.1 由 Runtime 按三态注入后收口。
- API 坐标为**度**；契约见 [`SURFACE_DATASET.md`](SURFACE_DATASET.md)。

历表数据在 orbitx 自带的 `assets/orbitx-data`。内部仍按 `Src/Celbody/...` 找 `.dat`。`resolve_ephemeris_data` 供 **CLI** 在缺省时展开成具体路径再传给 runtime。`orbitx-runtime` 必须收到 `--ephemeris-data`，不用环境变量或自动探测代替。

## 场景字段

| 字段 | 含义 |
|------|------|
| 出现在 `[[bodies]]` | 环境里有这颗星，渲染画它 |
| `dynamics` | 默认 `true`。`false`：不进 `GravBody`、不提供大气 |
| `rotation` | 有且 `enabled = true` 才推进转角 |
| `ephemeris` | 有则位置随 MJD 运动 |
| `fixed_pos` | 无历表时的位置 |
| `primary` | 积分原点与大气所属天体 |
| `mjd` | 起始时刻 |

别名 `earth` 解析到 `presets/scenario_earth.toml`：地球参与力学并自转，太阳只显示、固定在 1 AU。谁在缺省时使用它：只有 `orbitx-cli`。直接跑 `orbitx-runtime` 必须显式传入全部参数。

## 帧序

与 RUNTIME 一致：Control @ T0 → `update` → `grav_bodies`（主天体在原点）→ 刷新 Assembly 大气 / 周期 / 半径 → `Assembly::step` → `advance`。

地心系运动方程在 `Assembly::step`：

```text
a = gacc(r_ship) − gacc(r_primary)
```

`gacc` 只累加 `dynamics = true` 的天体。主天体在原点时，减项是其他力学天体在原点的引力。假太阳不在其中，减项为 0。

## 模块地图

```text
crates/orbitx-environment/src/
  body/        CelestialBody、历表与重力加载
  system/      PlanetarySystem::load / update / grav_bodies / attach_surface
  frame/       平移到 primary；PrimarySurface（含 G4 采样转发）
  surface.rs   SurfaceBinding（orbitx-surface 句柄）
  ephem_path/  resolve_ephemeris_data
```

测试与源文件同目录的 `tests.rs`。
