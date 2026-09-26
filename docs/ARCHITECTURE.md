# orbitx 产品架构（Runtime / Controller / 接触）

积木层（`math` / `dynamics` / `ephemeris` / `vessel`）按 Orbiter **技术参考**移植；本文件描述 **SimRocket 产品闭环** 的目标分层。完成度与排期见 [`ROADMAP.md`](ROADMAP.md)；crate 约束见 [`../AGENTS.md`](../AGENTS.md)；工作区 IPC 边界见根 [`AGENTS.md`](../../AGENTS.md)。

原 ROADMAP 的 P0–P2 是数值移植进度表；**不完整覆盖**本文件所述 Runtime / Godot 集成。二者互补，不以「P1 标 ✅」代替本架构已落地。

---

## 目标数据流

```text
Godot (sim-rocket)          orbitx 进程
  展示 / 会话控制              orbitx-runtime（主程序：通信 + Runtime，无 GUI）
       │                              │
       │         zenoh                │
       │   (+ protobuf / SHM)         ▼
       └──────────────────────►   Runtime 编排
  orbitx-cli ─────────────────►       │
    (同样经 zenoh)                     ├─► Controller（四档之一）
                                      ├─► Environment（P4.4；P4.2 暂 PlanetarySystem）
                                      └─► Assembly（积木）
```

- **CLI 与 Godot 同为同机 Zenoh 客户端**：`cli | Godot → 本机 zenoh/SHM → orbitx-runtime`。不直连 Controller；飞行 input 经 Runtime 转交（仅手动 `Control::Controller`）。**当前版本不支持跨设备。**
- **切片**：`orbitx-runtime → （zenoh）→ cli / Godot`。P4.2 先冻结进程内 `Slice`；传输在 P4.3+。
- **P4 编号即实施顺序**（见 ROADMAP）：
  - **P4.1** `orbitx-controller`（不改 cli）✅
  - **P4.2** `orbitx-runtime`（不改 cli；**无** Zenoh；环境暂用 `dynamics::PlanetarySystem`）
  - **P4.3** 本机 Zenoh + SHM + cli 客户端（禁跨设备）
  - **P4.4** `orbitx-environment`（环境状态从 dynamics 迁出）
  - **P4.5** Godot 会话 / bridge（原 P4.4）
- **遗留**：`orbitx-flight` / `orbitx-launch`（kiss3d）暂搁。
- **`demo-*`**：不强制本阶段改走 Zenoh。`demo-landing` 触点仍外挂（P5）。
- **`orbitx-app` / `UserVessel`**：本地 GUI 旁路，废除另排；`orbitx-app` **不是**产品仿真主进程。

根 AGENTS：`sim-rocket/rust ↔ zenoh ↔ orbitx-runtime`；CLI 同通道形态。

---

## Runtime（物理步进总运行时）

**权威设计文档**：[`RUNTIME.md`](RUNTIME.md)（双服务 / 双 tokio / 时钟 / tick / Log·Recorder 架构）。黑匣子**格式**：[`FLIGHT_RECORDER.md`](FLIGHT_RECORDER.md)。

| 职责 | 说明 |
|------|------|
| 时钟 | **固定 `sim_dt`（不随 warp 变）**；跨倍率可复现；`Instant` 仅 SelfPaced 节拍 |
| 编排 | 可选飞行 Input → Control → 环境 → `Assembly::step` → 切片 |
| I/O | **P4.2** flume + Comms **stub**；**P4.3** Comms=**本机 Zenoh + SHM**（专用 Comms tokio；**不支持跨设备**） |
| 节奏 | Godot默认 ClientStep；cli/无头 SelfPaced。**Runtime=`std::thread`**；**Comms tokio ⊥ IO tokio**（Log/Recorder），见 [`RUNTIME.md`](RUNTIME.md) |
| 观测 | tracing（non_blocking）+ FlightRecorder（不丢帧；格式见专文） |
| 启动 | `clap` argv（`--rocket` / `--scenario` / `--control` / `--workflow` / `--drive` / `--sim-dt` / `--log-dir` / `--recorder-dir` / `--zenoh-endpoint` / `--ephemeris-data`） |

**不是**：力模型/积分公式；GNC；长期环境状态宿主（P4.4 起为 `orbitx-environment`）。

**进程形态**：产品主程序 **`orbitx-runtime`**（无 GUI）。cli/Godot 以 argv/`spawn` 拉起后经 Zenoh **客户端**接入。**`orbitx-app`** 保留作本地 wgpu/egui 可视化。

---

## Controller（`orbitx-controller`）

在 Runtime 进程内、环境与船积分 **前**（T0）写执行器。不管时钟、不对客户端发切片、不管会话暂停/倍速。P4.1 已完成；经 Runtime 调用。

**权威设计文档**：[`CONTROLLER.md`](CONTROLLER.md)。完整帧序以 [`RUNTIME.md`](RUNTIME.md) 为准。

要点速览：
- **BaseController** 是 vessel 唯一全功能门面；`TargetController` / `WorkFlow` 必经它。
- 四档 a–d；顶层 `Control::Controller` | `Control::WorkFlow`。
- 飞行 UserInput 仅手动模式；WorkFlow 自拟飞控。
- 显示遥测由 Runtime 直读 telemetry，不经 Controller。

---

## 环境（演进）

| 阶段 | 宿主 |
|------|------|
| P4.2 | `orbitx-dynamics::PlanetarySystem`（过渡；dynamics 本为算法库） |
| P4.4 | **`orbitx-environment`**：世界状态与步进；dynamics 纯算法 |
| vessel | 不 own 环境；消费 `GravBody` / 大气 |

---

## 接触与碰撞

| 类型 | 权威 | 说明 |
|------|------|------|
| 星球地表 | **Orbitx** | 与 Godot **共用高程数据集**；稀疏触点探针；力进本步积分（P5） |
| 级间 / 多船简单体 | **Orbitx** | 代理几何；宽相后再窄相（P5） |
| 羽流撞击 | **待实现，本期不做** | |
| 复杂场景道具 | Godot 可选 | 经 zenoh 作 input 补充 |

现有 `touchdown` / `demo-landing` 外挂施力：**非**联机权威闭环。

---

## 会话生命周期（概念）

1. 父进程（cli / Godot）**spawn `orbitx-runtime`**（`--rocket` / `--zenoh-endpoint` 等）→ 再以**同机** Zenoh 客户端接入；完整会话配置协议 **P4.5**。  
2. 启停 / 暂停 / 倍率（会话命令 → Clock）。  
3. **Godot**：`_physics_process` → Step（+ 可选飞行 input）→ 收 Slice；`_process` 插值。**不是**渲染帧驱动步进。  
4. 循环：可选飞行 input → Control → 环境 → 固定 `sim_dt` step → 切片；Recorder 在 IO tokio 受控落盘。  
5. 终止信号 → 停 Comms → Runtime 完整步结束 → IO drain → 进程退出（见 [`RUNTIME.md`](RUNTIME.md)）。

---

## 工程约定（传输）

- 端到端延迟目标约 **一个物理步进**。  
- protobuf + zenoh：**仅本机 SHM**（P4.3 / P4.5）；跨设备另排。  
- **时钟主从**：Godot 会话默认 ClientStep；勿与 SelfPaced 双主。  
- **可复现硬需求**：操作按 `sim_t` 对齐时，不同 warp 积到同一 `sim_t` 结果一致；禁止 `dt *= warp`。详见 [`RUNTIME.md`](RUNTIME.md)。

---

## 与 Orbiter 单体的关系

保留：「统一 tick、环境场进入同一积分器」。  
不保留：渲染焊在物理对象上、进程内插件 Pre/Post 作为产品默认。  
Godot / CLI 是前端与会话客户端，不是第二套飞行动力学引擎。
