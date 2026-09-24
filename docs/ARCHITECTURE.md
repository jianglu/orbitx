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
                                      └─► PlanetarySystem + Assembly（积木）
```

- **CLI 与 Godot 同为 Zenoh 客户端**：`cli | Godot → zenoh → orbitx-runtime`。不直连 Controller；input 经 Runtime 转交。
- **切片**：`orbitx-runtime → （zenoh）→ cli / Godot`（遥测、姿态、按需环境数据等）。schema 待定。
- **P4 编号即实施顺序**（见 ROADMAP）：
  - **P4.1** 只建 `orbitx-controller`（不改 cli）
  - **P4.2** 只建 `orbitx-runtime`（不改 cli；含 Zenoh 服务端）
  - **P4.3** 改 cli 为 Zenoh 客户端
  - **P4.4** Godot 会话 / bridge
- **遗留**：`orbitx-flight` / `orbitx-launch`（kiss3d）暂搁，不纳入 P4.1–P4.3。
- **`demo-*`**：不强制本阶段改走 Zenoh。`demo-aero` 已用 `Assembly::step`；`demo-landing` 触点仍外挂（P5）；`demo-orrery` 无船步进。
- **`orbitx-app` / `UserVessel`**：本地 GUI 旁路，废除另排，不在 P4.1–P4.3。
- **`orbitx-app`**：现有 **wgpu/egui 本地可视化**，保留原名；**不是**产品仿真主进程。

根 AGENTS：`sim-rocket/rust ↔ zenoh ↔ orbitx-runtime`；CLI 同通道形态。

---

## Runtime（物理步进总运行时）

| 职责 | 说明 |
|------|------|
| 时钟 | 时间倍率、暂停、单步、`dt` 策略 |
| 编排 | 推进天体 → 引力源 → 各 `Assembly::step`（及接触） |
| I/O | Zenoh：收 input / 步进信号；发物理步进切片 |
| 节奏 | **可自驱**，或 **等待客户端步进信号**（启动参数/配置选择；帧同步细则另定） |

**不是**：力模型/积分公式本身；也不是 GNC 策略。

**进程形态**：产品主程序为 **`orbitx-runtime`**（通信 + Runtime 编排，**无 GUI**）。现有 **`orbitx-app`** 继续作本地 wgpu/egui 可视化，名称保留，避免与主进程冲突。

---

## Controller（`orbitx-controller`）

在 Runtime 进程内、tick **前**根据转来的 input + 船态写执行器。不管时钟、不对客户端发切片。P4.1 建本 crate（不改 cli）；P4.3 切 Zenoh 后由 Runtime 侧调用。

**权威设计文档**：[`CONTROLLER.md`](CONTROLLER.md)（分层 / BaseController 门面 / 四档 / 类层次 / ControlCapability / 舰队模式 / 遥测上行 / tick 顺序 / 热路径性能）。crate 模块地图见 [`../crates/orbitx-controller/README.md`](../crates/orbitx-controller/README.md)。

要点速览：
- **BaseController** 是 vessel 唯一全功能门面（读遥测 + 写执行器）；`TargetController` / `WorkFlow` 必经它，不直接碰 `Assembly`。
- 四档：a 实时轴/开关 → BaseController；b 飞行目标 → TargetController；c 简易工作流 → TargetWorkFlow → TargetController；d 复杂工作流 → SuperWorkFlow（入轨自动驾驶）。
- `ControlCapability`（`for_primary` / `for_detached` 按需构建）+ `BodyRef`；控制器不 own caps，由 Runtime（手动模式）或 WorkFlow（WorkFlow 模式）拥有。
- Runtime 持顶层 `Control` 枚举二选一：`Controller(Map<BodyRef, Box<dyn Controller>>)`（手动舰队 + 焦点）/ `WorkFlow(Box<dyn WorkFlow>)`（单一编排）。叶控制器与 workflow 仍用 trait 对象，枚举只在顶层闭集分派。
- 显示遥测由 Runtime 直读 `orbitx_vessel::telemetry`，不经 Controller；渲染用步进后数据。

---

## 接触与碰撞

| 类型 | 权威 | 说明 |
|------|------|------|
| 星球地表 | **Orbitx** | 与 Godot **共用高程数据集**（会话加载时配置 dataset id/路径等，勿每帧推整图）；稀疏触点探针 + 质心法向；力进本步积分（对齐 Orbiter `Elevation` + `TOUCHDOWN_VTX` 思路） |
| 级间 / 多船简单体 | **Orbitx** | 代理几何（capsule/OBB/凸包）；宽相：质心距 \(d \le L_a/2 + L_b/2\)（\(L\) = 包围盒最长边）才窄相；分离后短时 ignore |
| 分离冲量 | 可选 | 仅作 **clearance / 防撞**，不表示点火吹飞；与点火常有时间差 |
| 羽流撞击 | **待实现，本期不做** | Orbiter 核心亦无对邻船施力的羽流物理；吹飞观感暂接受上级自飞、下级相对落后 |
| 复杂场景道具 | Godot 可选 | 经 zenoh 作为 input 补充；非级间/地表默认路径 |

现有 `touchdown` 原语与 `demo-landing` 外挂施力：**非**联机权威闭环。入环 + 高程共用见 ROADMAP P5。

积分权威始终在 Orbitx：即使存在 Godot 接触补充，也以力/冲量/事件输入步进，避免双边改写质心。

---

## 会话生命周期（概念）

1. 客户端（Godot / 日后统一由会话配置）拉起 **`orbitx-runtime`** → 下发环境 / 时间 / 火箭 / 控制方式 / 高程 dataset 等  
2. 客户端决定开始 / 暂停 / 停止  
3. Runtime 与客户端物理帧对齐（细则另议；cli 与 Godot 均可发步进信号）  
4. 循环：input（含可选非高度场碰撞）→ Controller → step → 切片回传  

---

## 工程约定（传输）

- 目标端到端延迟控制在约 **一个物理步进** 内（架构必然时延，靠工程压）。  
- protobuf 提高有效密度；zenoh 优先共享内存 / 零拷贝。  
- 时钟主从由配置决定，避免双主。

---

## 与 Orbiter 单体的关系

保留：「统一 tick、环境场进入同一积分器」。  
不保留：渲染焊在物理对象上、进程内插件 Pre/Post 作为产品默认、双缓冲仅为渲染服务。  
Godot / CLI 是前端与会话客户端，不是第二套飞行动力学引擎。
