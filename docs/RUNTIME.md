# orbitx-runtime 设计文档

物理步进总运行时与产品主程序的权威设计。crate 落地后模块地图见 `crates/orbitx-runtime/README.md`；产品闭环顶层见 [`ARCHITECTURE.md`](ARCHITECTURE.md)；Controller 见 [`CONTROLLER.md`](CONTROLLER.md)；**FlightRecorder 磁盘格式**见 [`FLIGHT_RECORDER.md`](FLIGHT_RECORDER.md)；排期见 [`ROADMAP.md`](ROADMAP.md) P4.2–P4.5（Recorder 落盘 → P6）。

## 目标与边界

**是**：时钟；每步编排；双服务进程拓扑（Runtime 线程 + Comms）；**Comms / IO 双 tokio**；进程内 channel；生命周期与优雅退出；Log / FlightRecorder 实现架构；飞行 input / 会话命令 / 切片契约；**启动参数**；**台架/坠毁会话策略（P4.3 过渡）**；**本机 Zenoh+SHM + protobuf Comms（P4.3）**。

**不是**：力模型 / 积分公式 / GNC；GUI；跨设备 Zenoh；FlightRecorder **逐字段 schema**（→ [`FLIGHT_RECORDER.md`](FLIGHT_RECORDER.md)）；长期环境状态宿主（→ P4.4）；FlightPlayer / Recorder MCP（后续）；自动分离编排（→ 客户端 / WorkFlow）。

依赖单向：

```text
orbitx-runtime → orbitx-controller → orbitx-vessel
              → orbitx-dynamics（过渡：PlanetarySystem / GravBody）
              → orbitx-config / orbitx-math
              → orbitx-protocol（protobuf 线类型；encode 仅 Comms）
```

Comms / IO 使用 tokio；**不**把 zenoh 或 IO 写盘链进物理步进热路径。

不依赖 `orbitx-app` / render。`orbitx-cli` 为同机 Zenoh 客户端（spawn 本进程）；Godot 同形态 → P4.5。

## 进程形态：双服务 + 双 tokio + channel（冻结）

```text
main（clap 启动参数）
  ├── 创建 flume channel + ShutdownFlag
  ├── 拉起 RuntimeService     → std::thread
  ├── 拉起 Comms tokio        → CommsService（本机 Zenoh+SHM + protobuf）
  ├── 拉起 IO tokio           → Log（tracing）+ FlightRecorder
  └── block_on(Comms + ctrl_c) → 有序停机 → join → 退出
```

```text
                    ┌──────────────────────────────────────────┐
                    │            orbitx-runtime 进程             │
  客户端 ──本机 SHM──►  │  CommsService（Comms tokio）               │
  (cli/Godot 同机)       │    Zenoh 本机 peer + SHM（P4.3）            │
                    │         │ flume（Arc / 小枚举）              │
                    │         ▼                                  │
                    │  RuntimeService（std::thread）              │
                    │    Clock / Control / 环境 / Assembly         │
                    │         │ 非阻塞入队                        │
                    │         ▼                                  │
                    │  IO tokio：tracing appender + Recorder L2/L3 │
                    └──────────────────────────────────────────┘
```

| 部件 | 环境 | 职责 | 禁止 |
|------|------|------|------|
| **RuntimeService** | `std::thread` | 固定 `sim_dt` 步进、Control、环境、`Assembly::step`、组 Slice；向 Log/Recorder **非阻塞入队** | 线程内跑 zenoh；热路径同步写盘 / `block_on` |
| **CommsService** | **Comms tokio** | **本机 Zenoh + SHM**；protobuf 入站→channel；Slice→外发 | `Assembly::step`；持仿真权威；跨设备会话；跑 Recorder/Log 写盘 |
| **Log + FlightRecorder** | **IO tokio**（与 Comms **分离**） | 消费队列、落盘；现成库 | 与 Zenoh 共 worker 同步堵盘 |
| **main** | 主线程 | clap、装配、信号、停机 join | 重步进 |

### Zenoh 角色（冻结：仅本机 SHM）

- Comms 内 Zenoh = **本机会话宿主 peer**（产品语义上的「服务端」）；`orbitx-cli` / Godot = **同机客户端 peer**。
- **当前版本仅支持本机 shared memory（SHM）IPC**；载荷走 SHM 零拷贝。
- **不支持跨设备 / 远程会话**（禁止非本机 `tcp`/`udp` endpoint；跨主机另排，非本期）。
- 启动参数 `--zenoh-endpoint` 默认为 `local`（本机 SHM 会话）；禁跨设备；载荷为 **protobuf**（`orbitx-protocol`）。

### 双 tokio 与背压

- Comms tokio 与 IO tokio **隔离**：编码/`write` 不得占用 Zenoh worker。
- Runtime / Comms → IO：**有界队列 + `try_send`**；禁止在步进线程或 Zenoh 回调里同步等磁盘。
- **Log**：满则**丢弃并计数**。
- **Recorder**：满则 L2 放大 / 对**会话节奏**反压（少接 `Step` / SelfPaced 略缓）；**不丢帧**；不通过占住 Comms worker 制造背压。
- IO 内 zstd / 大块 `write`：IO runtime 任务或 `spawn_blocking`（属 IO 池）。

### Channel 与零拷贝

| 方向 | 载荷 | 背压 |
|------|------|------|
| Comms → Runtime | `InputCmd` / 会话命令 | 有界；满则反压 Comms，**不丢**关键会话命令 |
| Runtime → Comms | `Arc<Slice>` | 有界；满则**丢旧留新** |

- channel：**`flume` 有界**（跨 `std::thread` ↔ tokio）。
- Slice：`Arc<Slice>`；P4.3 Comms 侧再编码为 `Bytes` / Zenoh，**不**在 Runtime 线程序列化。

P4.2：Comms = stub；P4.3 换 **本机 Zenoh + SHM**，**不改编排与 channel 契约**；跨设备不在本期。

### 生命周期与优雅退出

`ShutdownFlag` = `Arc<AtomicBool>`（Runtime **不**依赖 `CancellationToken`）。

**停机顺序**：

1. 终止信号 / 致命错误 → 置位。  
2. **停 Comms**（停收；P4.3 flush Zenoh）。  
3. **停 Runtime**：结束当前**完整**步；退出线程循环。  
4. **IO tokio drain** Log/Recorder（黑匣子尾巴落盘）→ 关 IO runtime。  
5. `join` Runtime → drop Comms runtime / World → 退出。

原则：信号处理器只置位；显式 join；World/Control 仅 Runtime 线程拥有。

## 启动参数（冻结）

便于 cli / Godot / 脚本 `spawn`。解析库：**`clap`**。

| 参数 | 含义 | 默认 |
|------|------|------|
| `--rocket <alias\|path>` | 火箭类（同 cli：别名或 rocket.toml） | `falcon9` |
| `--scenario <path>` | 可选 scenario.toml | 无 |
| `--control [base\|target]` | 控制模式 a/b（与 `--workflow` 互斥；无值=target） | 未指定时等价 `target` |
| `--workflow <path>` | 工作流 TOML（与 `--control` 互斥；`kind` 由文件决定 target/super） | 无 |
| `--drive <client-step\|self-paced>` | 步进节奏 | `self-paced` |
| `--sim-dt <ms>` | 固定物理步长（整数毫秒） | `20` |
| `--log-dir <path>` | 日志目录 | `./logs` |
| `--recorder-dir <path>` | 黑匣子目录 | `./flight_records` |
| `--zenoh-endpoint <id>` | 本机 Zenoh/SHM 会话标识 | `local`（禁止跨设备 endpoint） |
| `--ephemeris-data <path>` | 历表数据根（须含 `Src/Celbody/...`；覆盖 `ORBITX_EPHEMERIS_DATA` 与自动探测） | `assets/orbiter-data` |
| `-h` / `--help` | 用法 | — |

`--rocket` 别名与 cli 同源（`orbitx-config`：`falcon9` / `saturnv` / `lm5` / `lm2f` / `lm7` / `lm9`）。

`--control` 与 `--workflow` **互斥**：都未指定 → 默认 `control=target`；同时指定 → 启动失败。`--workflow` 的 Target/Super 由 TOML `kind` 决定，不另设 CLI 枚举。

场景细节走 `--scenario`；火箭类走 `--rocket`（别名或路径）。权威启动面是 **argv**（env 可补充）。

## Log（实现架构）

| 项 | 约定 |
|----|------|
| 方案 | **`tracing` + `tracing-subscriber` + `tracing-appender`（`non_blocking`）+ `tracing-log`** |
| 桥接 | **必须** `LogTracer::init`，依赖树 `log` 宏进同一 subscriber |
| 级别 | debug 构建到 `debug`；release：`tracing` feature **`release_max_level_info`** |
| 渠道 | debug：console + 文件；release：**仅文件** |
| 写端 | non_blocking 挂 **IO 侧**；与 Zenoh worker 隔离 |
| 背压 | 满则丢日志；不反压 Runtime / Zenoh |

禁止自研 logger / 以 `flexi_logger` 为权威。

## FlightRecorder（实现架构）

格式权威：[`FLIGHT_RECORDER.md`](FLIGHT_RECORDER.md)。  
**产品落盘实现（CBOR + zstd 段文件）排期 P6**；P4.2 仅 L1 入队 + IO 计数 stub。

| 项 | 约定 |
|----|------|
| 完整性 | **不丢帧**（落盘落地后）；全分离体 + 环境 + 步入参；≥ 可视化精度 |
| 多级缓存 | L1 Runtime 非阻塞入队 → L2 IO tokio 批处理/CBOR → L3 受控写盘（仿真期持续） |
| 库 | `ciborium` + `zstd`；调度在 **IO tokio**（P6） |
| 写盘 | **可以且应当**在仿真中写；约束是**不进步进 / Zenoh 热路径** |
| 极端耗尽 | 暂停积步并显式失败；禁止静默丢帧 |
| Player / MCP | → P6 |

与 Log 分离：Log 可丢；Recorder 不丢（落盘落地后）。

## 环境归属（演进）

| 阶段 | 环境状态宿主 |
|------|----------------|
| **P4.2** | `orbitx-dynamics::PlanetarySystem`（过渡） |
| **P4.4** | **`orbitx-environment`** |
| vessel | 永不 own 太阳系 |

## 时钟（`Clock`）与客户端同步（冻结）

### 硬性需求：可复现（含跨倍率）

操作按 **`sim_t` / 步号** 对齐时：任意 warp 下积到同一 `sim_t` 结果一致。倍率只改变墙钟耗时。

| 允许 | 禁止 |
|------|------|
| 每步 `dt ≡ sim_dt`（默认 **20 ms**；物理边界换算为秒） | `dt = sim_dt * warp` |
| warp = 调度更多固定步 | 变长 `Instant` 当物理 `dt` |

### 物理步长

- 默认 **`sim_dt = 20` ms**（`u64`）；权威 `sim_t` 亦为整数毫秒累加；运行中与 warp 均不改步长。
- 物理（`Assembly::step` 等）在边界换算：`dt_s = sim_dt_ms as f64 / 1000.0`。
- `Instant` 仅 SelfPaced 节拍；`SystemTime` 不参与步进。
- cli 旧 `FIXED_DT = 0.05` s 为过渡，迁 Runtime 后以 **20 ms** 为准。

### `DriveMode`

| 模式 | 触发 | 用途 |
|------|------|------|
| **`ClientStep`** | 客户端 `Step(N)`（N 次固定 `sim_dt`） | Godot 默认 |
| **`SelfPaced`** | Runtime accumulator | cli / 无头 |

仅整步后写 `Slice`。Godot：`_physics_process` → Step；`_process` 插值。禁止渲染帧驱动步进；禁止 `dt *= warp`。

## 每次物理步进（冻结权威序）

```text
获取用户输入（可选） → Control 阶段 → 环境 → 航空器 → 切片
```

| 阶段 | 含义 |
|------|------|
| **用户输入（可选）** | 仅 `Control::Controller` 帧初 drain 飞行输入；WorkFlow 不消费 |
| **Control** | T0、船积分前；只写执行器 |
| **环境** | `GravBody`（P4.2 `PlanetarySystem`） |
| **航空器** | `Assembly::step` |
| **切片** | telemetry → `Slice`；另入队 Recorder（完整帧） |

**否定**：`环境 → Control → 航空器`。

会话命令（Pause / SetWarp / Step / Shutdown）→ Clock；与 Control 变体无关。`SetWarp` 不改 `sim_dt`。

### 环境与航空器时刻（P4.2）

1. Control @ T0。  
2. `PlanetarySystem::update_positions`；`GravBody[]` 平移为**地心系**（地球在原点，便于大气 / `planet_radius`）。  
3. `Assembly::step`。  
4. `PlanetarySystem::advance(dt/86400)` → T1；填 Slice / Recorder。

星历加载对齐 app：`SystemConfig::sol()` + `--ephemeris-data` / `ORBITX_EPHEMERIS_DATA` / 工作区 `assets/orbiter-data`（**不**回落 `../orbiter`）；失败则 strip 星历 fallback。

## `ControlDrive`

复用 `orbitx-controller`（`build_manual_control` / `build_control`）。详见 [`CONTROLLER.md`](CONTROLLER.md)。

## 进程内 Input / Slice

- 飞行 `InputCmd`：SetThrottle / SetAttitudeAxes / Separate / …
- 会话：Pause / SetWarp / Step(N) / Shutdown / …
- `Slice`：对外可瘦身；Recorder 帧按 [`FLIGHT_RECORDER.md`](FLIGHT_RECORDER.md) 完整记录。

## 与 Orbiter / cli 对照

| 对照 | orbitx Runtime |
|------|----------------|
| 变长 SimDT | 不采用；固定 `sim_dt` |
| 单体混 IO | Runtime 线程 + Comms/IO 双 tokio |
| 帧末 UserInput | 帧初飞行 drain |

## 不做（相对 P4.2 骨架冻结项；P4.3 已补齐 Comms）

- ~~真 Zenoh 本机 SHM / protobuf~~ → **P4.3 ✅**；**跨设备 Zenoh 另排**
- `orbitx-environment`（P4.4）
- Godot bridge（P4.5）；高程 / 触点入环（P5）；废除 `UserVessel`
- FlightRecorder CBOR 落盘 / FlightPlayer / Recorder MCP（→ **P6**）
- Runtime 线程内嵌 tokio 跑物理；cli 持有/步进 Assembly；自动分离进 Controller

## 阶段节奏

- **文档**：本文件 + [`FLIGHT_RECORDER.md`](FLIGHT_RECORDER.md)。
- **阶段 A**：crate 骨架——clap、双 tokio、Runtime 线程、Comms stub、Log/Recorder stub、占位步进。
- **阶段 B（已实现）**：真步进——`Assembly` + `orbitx-controller` + `PlanetarySystem`（星历；地心系 Grav）+ Slice 摘要；**Recorder 入队 stub 即可（CBOR 段文件 → P6）**。
- **P4.3 ✅**：Comms → 本机 Zenoh + SHM + protobuf；cli spawn/终止 runtime；台架/坠毁迁 Runtime；富 Slice；标准 GravityTurn。