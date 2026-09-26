# FlightRecorder 记录格式

黑匣子**磁盘格式**的唯一权威。实现架构（多级缓存、IO tokio、背压、与 Runtime 生命周期）见 [`RUNTIME.md`](RUNTIME.md)。  
**产品落盘 / FlightPlayer / Recorder MCP** 排期 **P6**（外围完善）；二者按本文解码。

## 目录布局

一次会话一个目录（由 `--recorder-dir/<session_id>/` 等决定）：

```text
<session>/
  manifest.json      # 入口：元数据、压缩参数、段列表、索引
  seg_000000.orec    # 段文件（可多个）
  seg_000001.orec
  …
```

运行时**不**写全量 JSONL。分析时由工具/MCP 解压 CBOR 帧再导出 JSON。

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
| `vessels` | **全部**航空器（含日后可分离体）初始状态 |
| `control` | 控制方式 / 配置摘要 |

### `kind = "step"`（每完整物理步一帧）

| 字段 | 说明 |
|------|------|
| `kind` | `"step"` |
| `sim_t` | 本步结束后仿真时刻 [ms] |
| `step_index` | u64 |
| `inputs` | 本步飞行 Input / 生效会话命令摘要 |
| `environment` | **本步环境状态**（与船同步，可视化精度） |
| `bodies` | **主组合体 + 全部分离体** 位姿/速度/姿态/燃料等 |

**完备性**：字段集 ≥ 可视化回放所需；**不得**因对外 `Slice` 瘦身而省略分离体或环境。  
FlightRecorder 是完整权威轨迹；Comms `Slice` 可另做带宽裁剪。

## 与 AI / MCP

- 入口：读 `manifest.json`。
- 取帧：按索引打开段 → 按规范解压 → 拆 CBOR → JSON。
- 后续 Recorder MCP（非本期）示例能力：`recorder_open` / `recorder_frame(sim_t)` / `recorder_range`。

## 版本

当前 **`schema_version = 1`**。不兼容变更必须递增，并保留旧版阅读说明或迁移工具。
