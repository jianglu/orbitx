# 地表运行时调度（P5.1 权威设计）

共用地表**数据包契约**见 [`SURFACE_DATASET.md`](SURFACE_DATASET.md)。  
本文约定 **orbitx-runtime 如何持有 / 预取 / 向 vessel 注入**地表查询；**不**描述 Godot 四叉树视觉流式（两侧同包同采样语义，启停与缓存规则独立）。

G4 已交付：`orbitx-surface` CPU 读库 +（过渡）`PlanetarySystem::attach_surface` 门面 + Godot 视觉接入。  
**P5.1（orbitx 闭环）已落地**：Runtime `SurfaceSession` + clap；surface 有界 LRU / prefetch；vessel `GroundContact` 入触点与坠毁。**Godot/Zenoh 协商（§2.1）随 P4.5 已落地**（启动 argv + `SetSurfaceWorkingMaxLevel` 切档热更新）。

---

## 1. 分层（谁驱动谁）

```text
配置（场景 / 发射场 / 会话 argv 等）
        │
        ▼
orbitx-runtime          唯一同时看见火箭状态与地表包的编排方
   ├─ PlanetarySystem   天体 / 历表 / 大气（environment）
   └─ SurfacePackage    读库 + 有界瓦缓存（surface；Runtime 并列持有）
        │
        │ 每 N_prefetch：按 pos/vel 预取 / 启停工作集
        │ 每 N_inject：注入 GroundQuery（或本帧不可用）
        ▼
orbitx-vessel           消费者：着陆触点 / 碰撞 / 坠毁
                        ✗ 不依赖 orbitx-surface，不读盘，不调度瓦
```

| 角色 | 职责 |
|------|------|
| **Runtime** | 构建时按配置打开 env + surface；分频预取与注入；把查询（或「不可用」）带入本步 vessel |
| **orbitx-surface** | 打开包、解码瓦、**多级/有界缓存**、`sample_*` / `prefetch`；**不知道火箭** |
| **orbitx-environment** | 天体表与步进；**不**根据火箭位置调度地表缓存。G4 的 `attach_surface` 为过渡门面；P5.1 目标为 Runtime 并列持有 surface，env 仅保留与 `body.size` / `R_elev` 一致性校验所需的 datum 协作 |
| **orbitx-vessel** | 只消费 Runtime 注入的地表查询结果做触点与坠毁；无表时不跑 DEM 接触 |

**禁止**：`orbitx-vessel → orbitx-surface` 直接依赖；在 `PlanetarySystem` 内塞入 `Assembly` 做预测。

与 Orbiter 对照（仅参考）：Orbiter 由 `Planet::ElevMgr` 持有数据、Vessel 带小 `etilecache`、查询路径内同步加载、高度自适应 LOD、远距 `alt_max` 关闭。本产品进程边界不同，采用 **Runtime 显式预取 + 分频注入**，不照搬 Vessel 持 cache。

---

## 2. 工作集与 LOD

- **目标深度 `working_max_level`**：物理预取/采样优先使用的四叉树 level。须与 Godot 当前质量档渲染地形 **最大深度对齐**（例：High 视觉/terrain 到 z9 → 物理也协商为 9；切 Low/Med 同理）。**不上限写死**；以会话协商值为准，并 **钳在包 manifest 该层 `max_level` 以内**（`min(协商值, package.max_level)`）。
- **空间范围**：火箭周围约 **1～2 km²** 命中的目标深度瓦（半径/瓦数可配置；后续可按设备性能动态收放）。
- **速度外推**：`|v|` 大时沿水平速度方向 **多预取 1～2 瓦**。
- **Fallback**：缺瓦 / 读失败 / 解码失败 → 祖先回退 → 仍失败则本帧按「已开但未就绪」处理（见 §4 光球占位）。

不要求与 Godot 同帧驻留同一瓦集合；只要求 **同一 `SURFACE_DATASET` 包与同一采样算法**。

### 2.1 Godot ↔ Runtime：协商与热更新

Godot 拉起 / 接入 `orbitx-runtime` 时必须协商地表工作深度（及包路径等）；**运行中可改**（切质量档、降载等）。

| 时机 | 机制（**P4.5 ✅ 已落地**：argv + `SessionCmd`；P5.1 会话协议同通道） | 内容 |
|------|----------------------------------|------|
| **启动** | spawn argv 与/或首包会话配置（与 scenario、dataset 根路径一并） | 至少：`dataset` 根、`working_max_level`（及可选工作集半径等） |
| **运行时** | 本机 Zenoh / 会话命令（与 Pause/SetWarp 同类控制面） | `SetSurfaceWorkingMaxLevel(u32)`（名称实现时可定）；可选同步工作集半径 |

约定：

1. **Godot 为视觉档权威**：切 Low/Med/High（或用户改地形最大 level）后，须把新的 `working_max_level` 下发给 Runtime，使物理工作集与渲染最大深度同级。
2. Runtime 收到后：更新目标 level；**钳制**到已打开包的 `layers.terrain.max_level`（及 water_* 各自上限按层取 min）；触发工作集按新深度重算（可保留祖先瓦作过渡）。
3. 降档时可主动驱逐深于新上限的瓦，控内存；升档则纳入预取队列，未就绪期间走 §4 光球占位。
4. 未协商或非法值：回退为包 `max_level`，并记日志；不静默用错误深度。

---

## 3. 分频：`N_prefetch` 与 `N_inject`

| 参数 | 含义 | 备注 |
|------|------|------|
| **N_prefetch** | 每多少个固定 `sim_dt` 评估一次工作集（启停、预取、驱逐） | 可稀于物理步；例：数步～数十步 |
| **N_inject** | 每多少步把当前地表查询状态交给 vessel | 近地接触需要真实高度时通常 **每步注入**；远距调度关闭时可不注入 |

二者**分开配置**，以便预取省 IO 判断、注入跟物理需求走。默认值与上限在实现时钉进会话/Runtime 配置，并允许后续按 IO/设备智能匹配。

---

## 4. 启停滞后与三态注入

**海拔阈值（可配置，默认建议）**

| 参数 | 默认建议 | 含义 |
|------|----------|------|
| `pause_alt_m` | **30 000** | 超过且径向速度向外 → **关闭**地表调度 |
| `resume_alt_m` | **20 000** | 低于或径向接近 → **打开**调度（滞后防抖） |

关闭条件：`alt > pause_alt_m` **且** `v · r̂ > 0`（远离主天体）。  
打开条件：`alt < resume_alt_m` **或** `v · r̂ ≤ 0`（接近 / 低空）。  
`pause_alt_m > resume_alt_m`，禁止对称单阈值来回抖。

从暂停高度降到打开高度之间的仿真时间应足以完成有界预取（与 `sim_dt`、每步最大加载瓦数、磁盘性能相关）；具体数值做成配置，而非写死设备假设。

**向 vessel 注入的三态**

| 状态 | 向 vessel 提供 | 触点 / 坠毁 |
|------|----------------|-------------|
| **调度关闭** | 不供表 | **不计算** DEM 触点与基于起伏的坠毁（框架约定：此阶段无地面交互；渲染与物理分离，无需为「穿模」保留光球） |
| **调度已开，工作集未就绪**（预取中 / miss / IO 失败回退中） | **光球高程**（`R_elev` / `body.size`，接触高 ≡ 0） | 按光球占位计算，直到目标瓦就绪 |
| **调度已开，工作集命中** | 真实 `ground_radius`（§3.1 接触高） | 完整起伏触点；P5.1 收回光滑球 `alt≤0` 为权威路径 |

要点：「关调度」与「开着但没数据」必须区分；后者用光球**占位**，不是视觉安全网。

---

## 5. surface 内部缓存

`orbitx-surface` 须支持**多级、有界**缓存，避免同瓦反复 IO：

1. **瓦解码缓存**（必做）：整瓦（f32 格网或平瓦常数）+ 有界 LRU / 工作集上限；`prefetch` 只写入本层。  
2. **热点采样微缓存**（可选后续）：极少 `(lat,lon)→h` 点，供同帧多触点。

G4 现状为无界 `HashMap` memo，P5.1 须改为有界并暴露大致 `cache_stats`（瓦数 / 字节）便于调参。

采样仍为 **CPU-only**；禁止为接触引入 GPU。

---

## 6. 与 G4 / 入环边界

| 项 | G4（已落地） | P5.1（orbitx 闭环） |
|----|--------------|---------------------|
| 读库 / 黄金语义 | ✅ | ✅ 保持 |
| 持有方 | env `attach_surface` 过渡 | ✅ **Runtime `SurfaceSession`**（`--surface-package`） |
| 预取 / 启停 | 无 | ✅ `on_tick` + 30/20 km 滞后 |
| vessel 入环 | 无（crash 仍光滑球） | ✅ `GroundContact` → step 触点 / crash |
| surface 缓存 | 无界 HashMap | ✅ 有界 LRU + `prefetch_around` |
| Godot / Zenoh | 视觉流式独立 | ✅ §2.1 协商（P4.5：`OrbitxSessionBridge`） |

**CLI（验收）**：`orbitx-runtime --surface-package <DIR> [--surface-working-max-level N] …`；省略包路径时 tick 仍注入光滑球（兼容无地表会话测例）。

**orbitx-cli**：透传同名旗；`--scenario earth` 且未指定包时，若仓库内 `simrocket_earth/packages/physical` 存在则自动打开（共享物理包；视觉分档由 Godot 加载）。坠毁：**只看接触力/穿透**（曾离地后：法向力超 `≈5×重量` 或穿透 >0.5 m；水面触水即毁，不要求离地）；起飞未离地时的刮地接触力不判毁。**已删除** `v_imp` 速度门限，不作兜底。空 `touchdown_points` 时步进合成默认底环触点以保证有力。触点速度与 HUD/气动同口径：扣地球共转 `ω×r`（`PlanetOrientation.omega`，与引力 `rot_matrix` 同源），共转贴地不产生对抗整段线速度的假摩擦。高程采样与 HUD 经纬为**行星体固系**（`Rᵀ · r` 后取 lat/lon；`R = PlanetOrientation.rot`），与共转速度同口径，避免共转箭在惯性地形图上东移刮丘。触地终局：陆地且无动力、**体固相对**位姿稳定连续 **5 s**（对齐 Orbiter `CheckLanded` / `prel`）→ `InitLanded`（保留朝向、体固 `land_rot` 共转）+ 着陆成功；水面触水即坠毁（无海上平台，暂不支持）；点火 10 s 台架未释放（有效推力口径）=发射失败；从未离地慢速回落台面不判终局、台架重新接管。着陆成功/发射失败由 runtime 上报、CLI 显示字样并暂停步进；高度与触地判定均以底部净间隙为口径（Slice `Alt` 仍为底部海拔，勿与间隙混用）。

---

## 7. 发射场高程覆盖与一致性规范（lp）

发射场由 `lp_xxx.toml` 描述（字段表见 [`CONFIG_TOML.md`](CONFIG_TOML.md)），orbitx 与 Godot 共用同一文件。启动：`orbitx-runtime --launchpad <lp> [--pad <台位>] [--heading-deg <度>]`，其中 `--launchpad` 对 runtime **必填**（不内置默认发射场，未传报错）；`orbitx-cli` 同名透传，未传参时默认单源（`sim-rocket/launchpads/lp_default.toml`，地表包默认 medium 档、缺失回退 low）。带 `elevation_override` 的 lp 必须同时开地表包；覆盖非法在开局报错，绝不飞行中降级。

**天体绑定**：lp 经纬度归属 `body` 指明的天体（按 scenario 天体表解析，未知天体启动报错）；地表包同样按天体绑定——`SurfaceSessionConfig.body_name`（有 lp 时为 lp.body，否则主天体），加载期与会话打开时均以 `|R_elev − 该天体 size| ≤ 0.5 m` 校验。架构支持多天体，当前实例只有地球：本轮仅主天体可摆放/飞行，lp 落在非主天体时加载与包校验可通过、构建 bundle 明确报错（非主天体相对系进 `GroundContact` 与 tick 编排为下一轮）。

### 7.1 高程合成：硬切换（无状态纯函数）

`contact_height(lat, lon)` 判定顺序：

1. 查询点换算到发射场当地 ENU 偏移 `(east, north)`（§7.2 公式）。
2. 有覆盖且在足迹内（`|east| ≤ (W−1)·cell/2` 且 `|north| ≤ (H−1)·cell/2`，边界算内）→ 覆盖网格双线性值，权威替换，不查全球包；陆/水分类恒为陆地。
3. 否则 → 全球包 `sample_contact_height`（分类走全球 water_class）。

**底部高程截平**：lp 声明发射场底部高程 `base_elevation_m`（场坪权威高程）。足迹内采样再做一步对齐：地形高截平为 `min(高程, base_elevation_m)`——只截高、不填低（场坪是推平的高地）；水面高与水陆分类不处理（地表包独立水面层负责，海上平台另行处理）。无覆盖网格时无足迹、不做面状对齐。足迹边界外地形高于场坪时边界成陡坎，靠边缘收敛契约与 >5 m 告警兜底，不做羽化。加载时各台位点全球高程与 `base_elevation_m` 比对，超差 >5 m 仅告警（lp 数据可能填错）。

**场坪填平域（`[pad_area]`，2026-10-25 增补，最终裁决层）**：lp 可声明场级圆域填平，在截平与全球采样之后**最终裁决**（与 Godot 侧 shader 同式，双端一字不差）：

```text
dist = ‖(east, north) − center_enu_m‖      // 相对场原点切平面投影（§7.2 公式）
t    = clamp((dist − radius_m) / blend_m, 0, 1)
h    = lerp(base_elevation_m, h_底层采样, t)
```

- 域内（t = 0）：**强制 `h = base_elevation_m`（填平，非只截高）**；地面法向 = ENU up；陆/水分类恒陆地（填平含水面，如沿海场坪）；`level_used = None`。
- 过渡带（0 < t < 1）：高度向底层采样（覆盖网格或全球包）线性过渡；法向/分类维持底层采样。
- 域外（t ≥ 1）：底层采样原样；t = 1 边界按构造连续。
- `blend_m = 0` 为硬边界（dist ≤ radius 即域内）。
- 填平域依赖 `base_elevation_m`：lp 有 `[pad_area]` 而 runtime 侧无场坪高时 `SurfaceSession::open` 报错拒绝（语义不完整，静默回退会埋穿模）。
- P4.5 参数：文昌 lp `radius_m = 60`、`blend_m = 20`，与 Godot 场坪圆盘同参。

同一经纬度永远同源，无滞回/防抖；跨界一步切换是预期行为。边缘向全球高程收敛是**生成阶段数据契约**；runtime 打开覆盖时采样最外圈节点与全球值比对，超差 >5 m 仅日志告警、不拦截。覆盖网格独立于瓦片层级（格距 1–5 m 量级），不受 `working_max_level` 影响，开局全量入内存；步进路径只做内存采样、零 IO。

采样顺带产出**地面法向**（覆盖/全球包同一邻域差分口径，先转 ENU 分量再换到查询点当地基）与**实际命中层级**（全球包回退后；覆盖/光滑球为 None），经 `GroundContact::sample_at_world` 暴露；坡度判毁后续再加。

职责：`orbitx-config` 只解析/校验 lp；`orbitx-surface::LocalElevationGrid` 管投影/足迹/双线性；`SurfaceSession` 的组合采样器承担硬切换（先问覆盖、None 落全球包）；`orbitx-vessel` 只消费 `GroundContact`。

### 7.2 一致性规范（Godot 与 orbitx 共同遵守）

- 常数：高程球半径取地表包 manifest 的 `R_elev_m`。
- **体固系约定**（地理经纬 / ENU / 地表包 lat/lon）：纬度 = +Y 北、经度从 +X 向 +Z（`lon = atan2(z, x)`）。**仅**定义在行星体固系。
- **惯性 ↔ 体固**：`R = PlanetOrientation.rot`（=`RotationState.rot_matrix`，含黄赤交角 + 恒星自转）；`r_inertial = R · r_body`，`r_body = Rᵀ · r_inertial`。共转速度 `v = ω × r`（`ω = PlanetOrientation.omega`，与引力同源）。
- **体固采样 / HUD**：惯性世界点先 `Rᵀ` 再取经纬查包；HUD 经纬同口径。摆放在体固系算场坪/ENU 后再乘 `R(t0)`。
- 经纬度 → 体固单位向量：`up = (cosφ·cosλ, sinφ, cosφ·sinλ)`；当地基 `east = (−sinλ, 0, cosλ)`，`north = (−sinφ·cosλ, cosφ, −sinφ·sinλ)`。
- 覆盖投影：查询点先落到高程球 `P = R_elev·up`，相对原点地心点 `O` 做基投影 `(east, north) = ((P−O)·east₀, (P−O)·north₀)`；反之切平面点 `O + east·e + north·n` 取方向得经纬度。
- 网格存储：行主序 f32 小端，row 0 = 北缘、col 0 = 西缘，节点 `east = (col − (W−1)/2)·cell`、`north = ((H−1)/2 − row)·cell`；值为相对高程球的绝对地面高（m）。
- 黄金夹具：[`crates/orbitx-surface/fixtures/launchpad_golden.toml`](../crates/orbitx-surface/fixtures/launchpad_golden.toml)（原点地心向量 + 采样点 ENU/经纬度/高度/法向），orbitx 单测已断言；Godot 联调时用同一夹具校验本侧换算与采样。

Godot 视觉侧直读同一 lp + 网格 + 地表包，按本规范在本侧坐标系预渲染同一抽象地形，不经 orbitx 拿高程（跨进程性能取舍，靠规范与夹具保证不损精度）。

### 7.3 摆放

`build_sim_bundle` 先开 `SurfaceSession`（含覆盖），再按选定台位摆放。台位支持面（火箭底部落点）**不走地形采样**，按发射场坐标系换算：支持面地心半径 = `R_elev + base_elevation_m + axis_offset_m[2]`（`axis_offset_m[2]` = 台架高度，相对发射场底部平面；地形低于场坪时台架等效架空、高于场坪时采样侧截平，两侧自洽）。`place` 只按质心 + 半装配高近似落点，装配完成后会话沿底部径向平移全栈做**底部归零**，使主组合体底部精确落在支持面（不再依赖装配原点偏移约定，多级组合同样适用）。台架**位置与姿态一次钉死在体固系**：`pad_pos_body` / `pad_r_body` 取归零后惯性量经 `Rᵀ(t0)`（姿态含 lp `heading_deg` / `axis_dir`）；每拍只做 `R(t)·(·)` + `ω×r`，不再每拍重算 `launch_attitude`。任务 toml 落地前默认放置位姿以发射场 lp 为准。

触地判定一律用**底部净间隙**口径（底点相对地面采样半径的高度；离地解锁为底部相对**地形**净间隙曾 >1 m；体固系采样后共转不再造成「地图东移」假刮丘，仍不用支持面径向高度做离地判定）。已释放但从未离地、且确在下沉时慢速回落台面不判终局，台架重新接管（清点火窗口，可再次点火）；仍在上升时视为正在爬升脱离，不接管；未离地刮地的接触力也不判坠毁。曾离地后着陆须无动力且**体固相对**位姿稳定满 5 s（位移 `Rᵀ·pos`、姿态 `Rᵀ·r`，对齐 Orbiter `prel`；共转贴地不计惯性东漂）→ `InitLanded`：**保留落地朝向**写入体固 `LandedState`（`pos_body` / `r_body`，对齐 Orbiter `land_rot`），底部贴地 + 共转速度；之后每拍 `R(t)·r_body` + `ω×r`（与台架钉死同源），不再 `launch_attitude` 硬掰竖直、也不冻惯性系姿态；坠毁改接触力/穿透门限（无 `v_imp`）。发射失败只由"点火 10 s 有效推力未超重量 1.05 倍未释放"产生，其 T/W 与台架释放门槛均为大气有效推力口径（diagnostics，与遥测一致）。注意 Slice 遥测 `Alt` 显示的是**底部海拔**（相对高程球，台上待命 ≈ 场坪海拔 + 台架高），不是净间隙——海拔是位置量、净间隙是判定量，两者勿混。

---

## 8. 修订记录

| 日期 | 变更 |
|------|------|
| 2026-10-21 | 初版：Runtime 驱动；`N_prefetch`/`N_inject`；30/20 km 滞后；三态注入；surface 有界多级缓存；与 Orbiter ElevMgr 对照说明 |
| 2026-10-21 | §2.1：Godot 启动协商 `working_max_level`；运行时热更新（切档同步）；钳制到包 max_level |
| 2026-10-21 | **P5.1 实现勾选**：SurfaceSession / GroundContact / LRU+prefetch |
| 2026-10-25 | §7.1 增补 `[pad_area]` 场坪填平域（最终裁决层）：统一公式与 Godot shader 同式，域内强制填平（含水面）；lp schema 2 增量段、`PadAreaSpec` 经 CLI 注入 `SurfaceSessionConfig`；缺 base 报错 |
| 2026-10-24 | **§2.1 落地（P4.5）**：Godot 启动 argv 协商 + `SetSurfaceWorkingMaxLevel` 切档热更新（`OrbitxSessionBridge`） |
| 2026-10-22 | CLI 默认 earth→low 包；软着陆门限 3 m/s 地表相对径向；HUD 高度对齐 GroundContact |
| 2026-10-22 | §7 发射场 lp：高程覆盖硬切换合成、一致性规范与黄金夹具、台位摆放 |
| 2026-10-22 | §7 天体绑定：lp.body 按天体表解析、地表包按所属天体校验；非主天体飞行留待下一轮 |
| 2026-10-22 | 默认发射场海南文昌（CLI 兜底、runtime `--launchpad` 必填）；CLI 地表包默认 medium 档 |
| 2026-10-22 | lp schema 2 高度模型：`base_elevation_m` 底部高程 + 足迹内截平（水面不碰）；`axis_offset_m[2]` 定为台架高度、支持面按坐标系换算；摆放底部归零；高度/触地判定改底部净间隙；台架释放与发射失败 T/W 改大气有效推力口径；回落台面重新接管 |
| 2026-10-23 | 着陆/坠毁对齐 Orbiter：无动力+位姿稳定 5 s 才 Landed（`InitLanded` 贴地）；坠毁仅接触力/穿透；合成默认底环触点；删除 `v_imp` 速度门限 |
| 2026-10-23 | 触点力扣地球共转 `ω×r`（与 HUD/气动同口径），修复共转被当成滑移摩擦导致贴地误毁 |
| 2026-10-23 | 高程采样/HUD 经纬改 +Y 体固系（`spin_y`），共转箭不再在惯性地形图上东移刮丘 |
| 2026-10-23 | 近地体固系对齐 `RotationState`：`PlanetOrientation`（`rot`+`omega`）经 `PrimarySurface` 注入；废除产品路径 `spin_y` / +Y 简化 |
| 2026-10-23 | 台架钉点改存体固系（`pad_pos_body`），每拍 `R(t)·r_body`，怠速不再相对发射场东漂 |
| 2026-10-23 | 台架姿态体固钉死一次（`pad_r_body`，含 lp heading）；步进不再 `launch_attitude(径向)` |
| 2026-10-23 | `InitLanded` 对齐 Orbiter 动态触地：保留朝向 → 体固 `land_rot`；LANDED 每拍 `R·r_body` 共转 |
| 2026-10-23 | 着陆稳定窗口改体固相对位移/姿态判动静（对齐 Orbiter `prel`；共转可贴地满 5 s） |
