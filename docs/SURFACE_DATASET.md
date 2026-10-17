# 共用地表数据集（Godot ↔ orbitx）

离线**预处理一次**、两边只读的统一地表包：地形高程、陆/水分类、水面高度。  
Godot 用于视觉（置换、水面 shading）；**`orbitx-runtime`**（P5）用于接触/坠毁等采样权威。  
运行时**只读本包**，不读 NASA/GSW 等原始图（原料仅供 G0b 离线烘焙）。  
Feature / 根场景见 [`sim-rocket/docs/GALAXY.md`](../../sim-rocket/docs/GALAXY.md)；地球视觉细则见 [`sim-rocket/docs/EARTH.md`](../../sim-rocket/docs/EARTH.md)；排期见 [`sim-rocket/docs/ROADMAP_GALAXY.md`](../../sim-rocket/docs/ROADMAP_GALAXY.md)（**G0a 选源下载 → G0b 预处理验收 → G4 运行时接入**）。

原料须满足：免费商用或一次性付费可商用；多源协议互相兼容；Low/Med/High 成套完整（详见 ROADMAP G0a/G0b）。

## 1. 原则

- **同一文件树 + 同一 datum + 同一采样语义**；禁止两边各算各的海陆启发式（例如单靠 `elev < 0`）。
- 大包外挂，不进 git；会话/配置只传 `dataset_id` 与根路径。
- Godot **不**为采样而链接 orbitx 动力学；orbitx **不**依赖 Godot 进程。
- **`orbitx-runtime` 消费路径 CPU-only**：读盘 + CPU 双线性/最近邻；**禁止**为采样/接触引入 wgpu、GPU 纹理采样或 Compute。产品契约**不依赖 GPU**（稀疏探针，非全图 GPU 扫）。
- Godot（及可选 `orbitx-app` 对照）可将视觉切块上传 GPU；与 runtime 读法可不同，**语义与黄金测点一致**即可。不把 GPU bake 写进 runtime 契约；离线预处理默认 CPU CLI。

## 2. Datum 与单位

| 项 | 约定（改则升 `format_version`） |
|----|--------------------------------|
| 水平 | WGS84 地理坐标，`lat_rad` / `lon_rad`（lon ∈ (-π, π]） |
| 垂直 | **MSL 海拔米**（与 SRTM / HydroLAKES 同源）：**`h = 0` = 平均海平面**；正向外 |
| `R_elev` | `h=0` 所附着的参考球半径；与 Orbiter/`Earth.cfg` / `BodyConfig::earth().size` 一致：**6 371 010 m**（地心距 ≈ `R_elev + h`） |
| 本包存「高」 | 不存地心距；用户可见海拔与 DEM/地图一致（海平面为 0） |

改 `R_elev`、海平面常量或垂直框架 → 升 `format_version`，整包重烘焙，禁止静默混用。

## 3. 图层（逻辑）

| 层 | 类型 | 含义 |
|----|------|------|
| `terrain_m` | float | **固体地形**高（陆地地表；水下可为海底/湖底 bathymetry，或无数据时填 NaN 并由 mask 处理） |
| `water_class` | u8 枚举 | `0=Land`，`1=Ocean`，`2=InlandWater`（湖/水库等），`3=River`（可选，多见于高层级瓦片） |
| `water_surface_m` | float | **自由水面**高（同椭球高）。仅 `water_class ≠ Land` 时有效；Land 上为 NaN/忽略 |

### 3.1 有效表面（接触用）

消费方统一：

```text
if water_class == Land:
    h_contact = terrain_m
else:
    h_contact = water_surface_m   # 浮于水面；不落到水底除非专门「潜水」模式（本期不做）
```

视觉：Godot 网格置换用 `terrain_m`（陆）或沿法线到 `water_surface_m` 的海面片（由 Mask 选中）；细节见 EARTH §5.5。

### 3.2 海洋水面高（MSL 零点，禁止运行时动态算）

**现行约定（产品）：以 DEM/MSL 为垂直标准。**

```text
ocean_water_surface_m = 0
```

- `terrain_m`、湖面高 = 源 DEM / HydroLAKES 的 **MSL 米原样入库**（不整体加减常数）。
- `Ocean` 的 `water_surface_m`（及同像素 `terrain`）填 **0**。
- 用户可见海拔、海岸接触：近岸陆 ≈ 0、海面 = 0，与地图一致。
- 地心距仍用 `R_elev + h`；`R_elev` 只表示「`h=0` 贴在哪个球上」，**不再**用 `R_msl − R_elev` 去改海面高。

**排期：** G0b 写入 manifest 并烤进包；Godot / `orbitx-runtime` **只读**。  
**orbitx 运行时禁止**再对 DEM 加减「海平面差值」。

内陆湖：G0b 写入该湖表面高（HydroLAKES 等，同一 MSL 框架），**不得**用海洋常值顶替。

#### 历史方案（已废止，勿再用于烤包）

曾用 WGS84 体积平均半径与 `R_elev` 之差填海：

```text
R_msl ≈ 6_371_000.790009 m
ocean_water_surface_m = R_msl - R_elev ≈ -9.209991 m   # SUPERSEDED
```

该做法使海面相对球零点为 −9.21，而陆地 DEM 仍为 MSL≈0，海岸出现假台阶，且与用户「海拔 0 = 海平面」不一致。说明见 [`OCEAN_MSL.md`](../../simrocket_earth/notices/OCEAN_MSL.md)（archived）。

#### G0b 垂直框架 — 必须遵守

| # | 约束 | 说明 |
|---|------|------|
| 1 | **MSL 零点** | 包内 `h=0` = 平均海平面；与 SRTM/地图海拔同语义。 |
| 2 | **海面常量 = 0** | `ocean_surface_m = 0`；Ocean 层填 0；禁止再填 −9.21。 |
| 3 | **源高程不平移** | 陆地/湖面按 MSL 原样入库；禁止对全图加 `R_msl−R_elev`。 |
| 4 | **出处入档** | datum=`msl_on_elev_sphere`（或等价）、`R_elev_m`、`ocean_surface_m=0` 写入 manifest + 本文件修订记录。 |
| 5 | **湖≠海** | 内陆水只用 HydroLAKES（等）湖面高；禁止用海洋常量填湖。 |
| 6 | **运行时只读** | Godot / `orbitx-runtime` 禁止帧内重算 geoid / 差值 / 动态求海平面。 |

### 3.3 渲染与物理一致

Godot 视觉与 orbitx 接触/高度校准须：

- 读同一 `dataset_id` 包与同一 `ocean_surface_m` / `R_elev_m` 常量（G0b 产出）；  
- 遵守 §3.1 接触公式与 §4 采样语义；  
- **禁止**两侧各用启发式（如 `elev < 0` = 海）或运行时重算海平面。  

物理引擎（大气海平面、高度表、接触）以包内 `h`（海面为 0）为准，不在帧循环中重算（P5/G4 接入时消费）。

## 4. 采样 API（语义契约）

两边实现须对齐（语言可不同，测夹具同点同结果）：

```text
sample_terrain(lat, lon) -> f64          # 双线性；越界/无瓦片 → 约定错误或父级
sample_water_class(lat, lon) -> WaterClass
sample_water_surface(lat, lon) -> Option<f64>  # Land → None
sample_contact_height(lat, lon) -> f64   # 见 §3.1
```

插值：`terrain_m` / `water_surface_m` 双线性；`water_class` **最近邻**（禁止把海陆插成 0.5）。

## 5. 预处理管线（离线）

输入（示例，按许可证选用）→ 输出统一瓦片包：

| 输入 | 用途 |
|------|------|
| NASA / SRTM 等 DEM | `terrain_m`（60°N–56°S 主覆盖） |
| NOAA GLOBE 30″ | SRTM 未写入的陆地（极区与内部空洞）；**不**写入与 Ocean 四邻接的海岸陆像素 |
| 海陆 / [GSW](https://global-surface-water.appspot.com/) 等 | `water_class`（内陆 occurrence ≥50，不做季节性剔除；时期口径见下） |
| G0b 已钉 `ocean_surface_m` | `Ocean` 的 `water_surface_m`（烤包拷贝常量，不运行时重算） |
| HydroLAKES 等湖面高 | `InlandWater` 的 `water_surface_m` |
| （可选）河网矢量 | 高层级 `River` |

输出目录约定（与 EARTH 切块可同树或子目录）：

```text
<entry_root>/
  manifest.toml          # format_version, dataset_id, datum, levels, layer 文件格式
  terrain/...
  water_class/...
  water_surface/...
```

`manifest.toml` 至少含：`format_version`、`dataset_id`、`datum`（现行 `msl_on_elev_sphere`）、`R_elev_m`、`ocean_surface_m`（预计算常量，现行 0）、瓦片 level 范围、nodata 约定。

**水系时期（G0b）**：视觉层为 Blue Marble NG 2004-07。GSW occurrence 为 1984–2015 长期观测；内陆 **≥50** 标 `InlandWater`（近海贴 Ocean 且 ≥50 仍标 `Ocean`）。这是与该视觉时期一致的常年水体代理，**不**再按季节性把 50–75 剔除，也不做逐年对齐。

**极区高程**：SRTM 不到的陆地用 NOAA GLOBE 30″ fallback；60°N / 56°S 各约 ±1 输出像素与 SRTM 互权重过渡。海岸陆像素不采 GLOBE，保持 `ocean_surface_m`。

## 6. 版本与测试

- 改枚举、datum、插值 → 升 `format_version`；Godot 与 orbitx 拒绝不兼容主版本。
- 现行包为 **`format_version = 2`**（MSL 零点 + GLOBE fallback）。`orbitx-surface` 加载时主版本 ≠ 2 则报错并要求重烘。Godot 夹具不读 manifest。
- 黄金测点表（固定 lat/lon → class / terrain / water_surface / contact）两边 CI 或手测对照，误差阈值写入 manifest（建议高程 &lt; 1 m 量级相对同源瓦片）。Low / Med / High 共用同一张表。

## 7. 与 P5

orbitx P5.1「共用高程」扩展为**本文件定义的地表包**（高程 + 水分类 + 水面高）。会话下发同一 `dataset_id` / 路径。物理高度零点相对 `ocean_surface_m` 校准（只读常量）。

## 8. 修订记录

| 日期 | 变更 |
|------|------|
| 2026-10-16 | 废除 Ocean=`0`；`ocean_water_surface_m = R_msl − R_elev`（`R_elev=6 371 010`）；禁止运行时动态算；**数值计算与烤包属 G0b**（G0a 仅钉公式） |
| 2026-10-16 | G0b：钉 `R_msl=6 371 000.790009`、`ocean_water_surface_m=−9.209991`（WGS84 体积平均半径；`g0b_ocean_msl.py` / `OCEAN_MSL.md`） |
| 2026-10-16 | 增补 §3.2「G0b 海平面校正 — 必须遵守」六条（球面等效 / DEM 零点核对 / 禁止先验 0 / 出处 / 湖≠海 / 只读） |
| 2026-10-16 | G0b terrain：三档包改为 ESA SRTM90 米制（`g0b_bake_terrain_srtm.py`）；废止 HYP 代理；`surface_golden_points.toml` 增海岸陆/海成对测点 |
| 2026-10-17 | **垂直基准改订为 MSL**：`ocean_water_surface_m = 0`；DEM/湖面原样；废止用 `R_msl−R_elev≈−9.21` 填海；orbitx 只读、不算差值 |
| 2026-10-17 | **`format_version = 2`**：datum 变更升版；`dataset_id` 为 `earth_g0b_{low,medium,high}_v2`。`orbitx-surface` 拒绝其它主版本 |
| 2026-10-17 | terrain：ESA SRTM90 为主；未覆盖陆地用 NOAA GLOBE 30″ fallback（海岸不采；60°N/56°S ±1 px 过渡）。水系维持 GSW ≥50，对齐 BMNG 2004-07，不做季节性剔除 |
