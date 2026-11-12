# FlightRecorder 记录格式

黑匣子**磁盘格式**的唯一权威。实现架构（多级缓存、IO tokio、背压、与 Runtime 生命周期）见 [`RUNTIME.md`](RUNTIME.md)。  
**P6.1 已实现**：`orbitx-recorder`（格式读写库）、`orbitx-runtime` 写路径、`orbitx-flightplayer`（zenoh 查询服务）；Recorder MCP / 回放 UI 属 **P6.2**（非本期）。

## 目录布局

录制根目录（`--recorder-dir`；Godot 端传 `user://orbitx_records` 对应的平台路径）下，
一次会话一个目录；**`session_id = <火箭ID>_<hex4>_<UTC 年-月-日_时分秒>`**
（例 `Test_1_a3f2_2026-11-03_191345`；火箭名净化为目录安全字符，同秒冲突由
`SessionWriter::create` 追加 `-2`…`-100` 后缀，空目录可复用）。**同进程 Reset = 封当前段
→ `finish` 旧会话 → 开新 session 目录**（新 `session_start` 帧）。

平台缺省根目录（`orbitx-flightplayer` 缺省；Godot 端 `user://orbitx_records` 同位）：

| 平台 | 路径 |
|------|------|
| macOS | `~/Library/Application Support/WLCY/SimRocket/orbitx_records` |
| Windows | `%APPDATA%\WLCY\SimRocket\orbitx_records` |
| Linux | `$XDG_DATA_HOME`（缺 `~/.local/share`）`/WLCY/SimRocket/orbitx_records` |

```text
<session>/
  manifest.json      # 入口：元数据、压缩参数、段列表、索引
  seg_000000.orec    # 段文件（可多个）
  seg_000001.orec
  …
```

运行时**不**写全量 JSONL。分析时由工具/MCP 解压 CBOR 帧再导出 JSON。

### 竞赛会话子树（会话上下文标签，C3 落地）

`Configure.session_context.recorder_subtree`（通用标签，runtime 不解其义，见
[`RUNTIME.md`](RUNTIME.md)）指定会话目录的**相对子树**；竞赛飞行由客户端填
`competition/<pack>/<grade>/<task>`，会话目录落在：

```text
<recorder_root>/
├── <自由飞行会话>…                                  # 无标签 = 现状根，行为不变
└── competition/<pack>/<grade>/<task>/
    └── <attempt>-<session_id>/                      # 按（包,组别,任务,尝试）寻址
```

`orbitx-flightplayer` 的 `sessions` 查询天然覆盖子树（递归列目录）；停机事件 `Halted`
回传的会话引用即精确目录（sim-rocket `COMPETITION.md` §10/§11）。

## `manifest.json`

| 字段 | 类型 | 说明 |
|------|------|------|
| `schema_version` | u32 | 本文格式版本；当前 **`1`** |
| `sim_dt` | u64 | 固定物理步长 [ms] |
| `drive` | string | `"client-step"` \| `"self-paced"` |
| `created_unix_ms` | u64 | 会话创建墙钟（仅元数据，不参与可复现） |
| `codec` | string | 段压缩：产品默认 **`"zstd"`** |
| `zstd_level` | i32 | 默认 **`3`**（`codec=zstd` 时） |
| `segments` | array | 见下 |
| `index` | array | 稀疏索引：`{ "sim_t", "segment", "frame_ordinal" }` |

`segments[]` 元素：

| 字段 | 说明 |
|------|------|
| `name` | 如 `seg_000000.orec` |
| `first_sim_t` / `last_sim_t` | 段内帧仿真时间范围 [ms] |
| `frame_count` | 未压缩帧流中的帧数 |
| `uncompressed_len` | 解压后 payload 字节数 |
| `compressed_len` | zstd 比特流字节数 |

## `*.orec` 段文件

### 字节布局

```text
[ magick 8 bytes ][ header ][ payload ]
```

- **魔数**：ASCII `OXREC001`（8 字节）。
- **header**（小端定长，schema 1）：

| 偏移 | 类型 | 字段 |
|------|------|------|
| 0 | u32 | `header_version` = 1 |
| 4 | u32 | `schema_version` = 1 |
| 8 | u16 | `codec`：`1` = zstd，`0` = none（**仅开发**；产品默认 zstd） |
| 10 | i16 | `zstd_level`（codec=zstd；否则 0） |
| 12 | u64 | `uncompressed_len` |
| 20 | u64 | `compressed_len` |
| 28 | u32 | `frame_count` |
| 32 | u32 | reserved = 0 |

- **payload**：若 `codec=zstd`，为 zstd 压缩比特流；解压后得到**帧流**。若 `codec=none`，payload 即帧流。

### 压缩规范（冻结）

| 项 | 约定 |
|----|------|
| 算法 | **Zstandard（zstd）** |
| 实现库 | Rust **[`zstd`](https://crates.io/crates/zstd)**（官方 C 库绑定）；禁止自研压缩 |
| 作用域 | **按段**压缩：段内多帧先拼成帧流，再对整段 payload zstd；**不对单帧各自压** |
| 默认级 | **level = 3**（写入 manifest / 段头；可配置） |
| 执行位置 | **仅** Recorder IO 路径；禁止 Runtime 步进线程压缩 |

编解码必须同一算法与封装；更换不兼容压缩须升 `schema_version`。

### 帧流（解压后）

连续 **length-delimited CBOR**：

```text
[ u32 LE length ][ CBOR bytes ] …
```

每帧一个 CBOR map（逻辑字段如下；Rust 侧用 `serde` + **`ciborium`**）。

## 帧类型与必含字段

### `kind = "session_start"`（会话初，通常每会话一帧）

| 字段 | 说明 |
|------|------|
| `kind` | `"session_start"` |
| `schema_version` | u32 |
| `sim_t` | u64（通常 0）[ms] |
| `sim_dt` | u64 [ms] |
| `environment` | 环境完整初始快照（天体/世界态，可视化可还原） |
| `vessels` | **全部**航天器（含日后可分离体）初始状态 |
| `control` | 控制方式 / 配置摘要 |

### `kind = "step"`（每完整物理步一帧）

| 字段 | 说明 |
|------|------|
| `kind` | `"step"` |
| `sim_t` | 本步结束后仿真时刻 [ms] |
| `step_index` | u64 |
| `inputs` | 本步飞行 Input / 生效会话命令摘要（`Shutdown`/`Reset` 不入帧；会话边界由 `session_start` 表达） |
| `environment` | **本步环境状态**（与船同步，可视化精度） |
| `separated` | 本步发生分离的船序号（u32[]；无则空） |
| `terminal` | `{ crash_msg, landed_msg, launch_failed_msg }`（Option<string>；触发时非空） |
| `bodies` | **主组合体 + 全部分离体** 位姿/速度/姿态/燃料/`thrust_level`/诊断（13 项环境类：`a_grav`…`load_factor`，权威直录自 `FlightDiagnostics`） |

**完备性**：字段集 ≥ 可视化回放所需；**不得**因对外 `Slice` 瘦身而省略分离体或环境。  
FlightRecorder 是完整权威轨迹；Comms `Slice` 可另做带宽裁剪。

## 读取端（P6.1）

- **Rust Reader**：`orbitx-recorder::reader`——`list_sessions(root)`（含 `manifest.json` 的目录，名序）、
  `SessionReader::open(dir)`（manifest 校验 schema 1）、`frames()`（段序全量）、
  `frames_between(first, last)`（段级 `last_sim_t` 剪枝 + 帧内 `sim_t` 过滤）。
- **zenoh 查询服务** `orbitx-flightplayer`（独立进程；key `orbitx/recorder/query`，JSON）：
  - 请求 `{"op":"sessions"}` → `{"ok":true,"sessions":[{dir,drive,sim_dt_ms,created_unix_ms,segment_count,frame_count,first_sim_t,last_sim_t}]}`
  - 请求 `{"op":"frames","session":"<目录名>","from":ms?,"to":ms?}` → `{"ok":true,"session","count","frames":[...]}`
  - 错误 → `{"ok":false,"error":"..."}`；`session` 按目录名白名单匹配（不参与路径拼接）。
  - 运行：`cargo run -p orbitx-flightplayer [-- --zenoh-endpoint local --root <dir>]`；
    缺省 root 见上文平台表，缺省 listen `tcp/127.0.0.1:17448`。

### 查询演进（P6.2；竞赛评分消费，C3/C4）

现 `frames` 为全字段全段拉取（2 万帧峰值内存 ~0.6–0.8GB，见 sim-rocket `AGENT.md` 已知
局限）。竞赛评分数据链路（sim-rocket `COMPETITION.md` §10）按需演进：

| 演进 | 说明 |
|------|------|
| 帧窗落地 | `from` / `to` 参数已预留 → 实现段级剪枝 + 帧内过滤（Reader `frames_between` 已有基础） |
| 字段投影 | 请求携带字段集 → 只解包/返回投影字段（减 wire 体积与客户端解析） |
| 扫描定位 | 评分「预测拉取」支撑：按比例预测近似位置 → 向前/向后扫描锁定目标帧组（避免全量解压；依赖稀疏索引 + 段级剪枝） |

版本兜底：wire 投影携带 `WIRE_SCHEMA_VERSION`（与 recorder `schema_version` 对拍门禁已有），
消费方（评分读取 API）校验不匹配即明确报错。

## 与 AI / MCP

- 入口：读 `manifest.json`（或 `sessions` 查询）。
- 取帧：`frames` 查询（范围过滤）；或本地 Reader 解压 CBOR → JSON。
- Recorder MCP（`recorder_open` / `recorder_frame(sim_t)` / `recorder_range`）与回放 UI 属 **P6.2**（非本期）。

## 版本

当前 **`schema_version = 1`**。不兼容变更必须递增，并保留旧版阅读说明或迁移工具。
