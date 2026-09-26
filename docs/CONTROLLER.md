# orbitx-controller 设计文档

航天器控制策略 crate 的权威设计文档。crate 模块地图见 [`../crates/orbitx-controller/README.md`](../crates/orbitx-controller/README.md)；产品闭环顶层架构见 [`ARCHITECTURE.md`](ARCHITECTURE.md)；排期见 [`ROADMAP.md`](ROADMAP.md) P4.1。

## 目标与边界

落地四档控制（a/b/c/d）与类层次（Base / Target / WorkFlow）。Controller 只负责控制逻辑：写执行器、解析工作流、驱动目标导向算法。真正响应控制在 `orbitx-vessel`；controller 不重写物理、不重写查询、不存遥测副本。

依赖单向：`orbitx-controller → orbitx-vessel`（+ `orbitx-math`、`serde`、`toml`），不反向依赖 Runtime。cli 不动（`orbitx-cli` 暂留旧 `control.rs`，P4.3 切 Zenoh 时退役）。

## 分层

```text
BaseController（vessel 唯一全功能门面：读遥测 + 写执行器，持 &mut Assembly + &caps）
  ▲
  │ 只经 base 读 + 写
  ├── TargetController（模式 b，控制算法，不 own caps，只 own 算法状态）
  │     ▲
  │     │ 按阶段条件设目标配置
  │     ├── TargetWorkFlow（模式 c，own 1 TargetController + 主 caps）
  │
  └── SuperWorkFlow（模式 d，own 子控制器舰队 + 各 body caps，编排 + 分离派生，入轨自动驾驶）

模式 a：客户端实时轴/开关 → BaseController 直接调方法（不经 Controller trait）
```

`TargetController` / `WorkFlow` 都不直接碰 `Assembly`，必经 `BaseController`。

## BaseController：vessel 唯一全功能门面

唯一直接操作 `Assembly` 的单元——**既读遥测也写执行器**。持有 `&mut Assembly + &caps`，读方法取 `&self`、写方法取 `&mut self`，分离的方法调用无借用冲突。

```rust
pub struct BaseController<'a> { asm: &'a mut Assembly, caps: &'a ControlCapability }

impl<'a> BaseController<'a> {
    // 读遥测（委托 Assembly 的 telemetry trait impl；Base 是控制器侧唯一出口）
    fn attitude_errors(&self) -> (f64, f64);
    fn pitch_yaw_angles(&self) -> (f64, f64);
    fn tip_angle(&self) -> f64;
    fn roll_angle(&self) -> f64;
    fn omega(&self) -> Vec3;
    fn velocity(&self) -> Vec3;
    fn position(&self) -> Vec3;
    fn speed(&self) -> f64;
    fn total_mass(&self) -> f64;
    fn fuel_mass(&self) -> f64;
    fn fuel_percent(&self) -> f64;
    fn lit_thrusting_indices(&self) -> impl Iterator<Item = usize> + '_;
    fn primary_thrust_sum(&self) -> f64;

    // 写执行器
    fn set_throttle(&mut self, policy: ThrottlePolicy, level: f64);
    fn apply_tvc(&mut self, group_id: &str, pitch_target: f64, yaw_target: f64, dt: f64);
    fn set_rcs(&mut self, group_id: &str, axis: RotAxis, level: f64);
    fn separate(&mut self, point_id: &str) -> Vec<usize>;  // 透传 undock 的 detached 下标
    fn should_auto_separate(&self) -> bool;

    fn caps(&self) -> &ControlCapability;
}
```

模式 a：客户端实时轴/开关 → 直接调 `BaseController` 方法（不经 `Controller` trait）。

## 四档

| 模式 | 客户端下发 | Orbitx 侧 |
|------|------------|-----------|
| **a** | 实时轴/开关 | **BaseController** → 控制 API（直接调方法） |
| **b** | 飞行目标（方向、节流目标、分离等） | **TargetController**（控制算法；经 BaseController 读+写；类现 cli） |
| **c** | 简易目标导向工作流 | **TargetWorkFlow** → TargetController（按阶段条件设目标配置） |
| **d** | 复杂自动控制工作流 | **SuperWorkFlow** → BaseController / 子控制器舰队（入轨自动驾驶） |

## 类层次与 trait

```rust
// 叶控制器统一 tick 入口（TargetController 等）。BaseController 不 impl。
pub trait Controller {
    fn tick(&mut self, base: &mut BaseController, dt: f64);
}

// 顶层编排，tick 取 &mut Assembly 以便为多 body 构造 BaseController。
pub trait WorkFlow {
    fn tick(&mut self, asm: &mut Assembly, dt: f64);
    fn is_done(&self) -> bool;
}
```

- **BaseController**：vessel 唯一全功能门面，不 impl `Controller`（模式 a 由外部直接调方法）。
- **TargetController**：目标导向控制算法，只经 `BaseController` 改执行器。**不 own caps**，只 own 算法状态（`TargetMode`、重力转向进度等）。
- **TargetWorkFlow**：own 1 个 `TargetController`（主 body）+ 主 body caps；tick 按阶段条件设目标配置，构造主 body `BaseController` 调 `target.tick`。
- **SuperWorkFlow**：own 子控制器舰队（各 body caps + 子 `TargetController` / 直接 `BaseController`）+ toml 编排状态；tick 按编排为每 body 构造 `BaseController`、经 base 读遥测/姿态、精确命令各部件；分离时按编排重建 caps + 派生子控制器。

## TargetController（模式 b）

目标导向控制器——用户期望航天器朝哪飞、推力多大（即现 cli 控制能力）。内部根据目标 + 当前姿态/状态 + 部分遥测，经 `BaseController` 控制本步怎么飞。`TargetMode` 枚举：

- `VerticalHold { throttle }` → apply_tvc(0, 0)
- `PitchTo { pitch, yaw, throttle }` → apply_tvc(pitch, yaw)
- `ProgradeHold { throttle }` → 由速度方向反解 pitch/yaw 目标
- `RetrogradeHold { throttle }` → 反向
- `GravityTurn { throttle, pitch_rate }` → 渐进俯仰，保低迎角

`TargetController` 不 own caps（caps 由 Runtime/WorkFlow 拥有，避免 tick 时 `&mut self` 与 `base.caps()` 借用冲突），只 own 算法状态。caps 变化由拥有者换，`TargetController` 经 `base` 观察新 caps，算法状态保留。

## 能力契约 ControlCapability

controller 能控制什么，由航天器抽象决定，不靠 introspect vessel 字段。契约类型在 controller crate（消费者拥有契约），数据由 `for_primary` / `for_detached` 从 `Assembly` 投影（vessel 是物理真相源），按需构建。controller 按稳定 group id 命令执行器，不硬编码索引。

```rust
pub enum BodyRef { Primary, Detached(usize) }  // 可控体标识

pub struct ControlCapability {
    pub body: BodyRef,
    pub throttle_groups: Vec<ThrottleGroup>,
    pub tvc_groups: Vec<TvcGroup>,
    pub dock_ports: Vec<DockPortRef>,
    pub rcs_groups: Vec<RcsGroupRef>,
    pub separation_points: Vec<SeparationPoint>,
}

impl ControlCapability {
    pub fn for_primary(asm: &Assembly) -> Self;       // 主组合体执行器
    pub fn for_detached(asm: &Assembly, idx: usize) -> Self;  // 某分离船执行器
}
```

- **生命周期**：按需（事件驱动）重建——会话开始建主 caps，分离/对接事件点重建。稳态 tick 不重建、不指纹轮询。
- **归属**：手动模式 → Runtime own；WorkFlow 模式 → WorkFlow own。**控制器不 own caps**；caps 由拥有者换，控制器经 `base` 观察新 caps，算法状态保留。无 `update_caps` 方法。
- **BodyRef**：`Primary` / `Detached(usize)`，标识控制器绑定的可控体；分离出的独立体可按场景派生自己的控制器。

## Runtime 顶层 Control enum（P4.2 接线）

Runtime 持单一顶层枚举约定控制器类型，二选一闭集：

```rust
pub enum Control {
    Controller(Map<BodyRef, Box<dyn Controller>>),  // 手动模式：多控制器按 BodyRef 索引 + 焦点
    WorkFlow(Box<dyn WorkFlow>),                    // WorkFlow 模式：单一编排
}
```

- **手动模式（`Controller` 变体）**：Runtime 持 `Control::Controller(map)` + 焦点 `BodyRef`；输入路由到焦点控制器。caps 由 Runtime 并行持有（`Map<BodyRef, ControlCapability>`，事件驱动重建、不每 tick）；tick 时为焦点 body 构造 `BaseController { asm, caps }` 调 `controller.tick`。分离时 Runtime 重建主 caps（`for_primary`）+ 按场景 `ControllerAssignment` 为新 detached vessel 派生子控制器（`for_detached`）并入 map。
- **WorkFlow 模式（`WorkFlow` 变体）**：Runtime 持 `Control::WorkFlow(box)` 一个；WorkFlow 内部 own 子控制器 + 各 body caps，分离时 WorkFlow 自己重建 caps + 派生子控制器。Runtime 不直接持子控制器。
- 叶控制器（`Box<dyn Controller>`）与 workflow（`Box<dyn WorkFlow>`）仍用 trait 对象；`Control` 枚举只在顶层把两模式收敛为闭集，穷尽 match、无顶层 vtable 间接。
- **归属**：`Control` 定义在 controller crate（紧邻 `factory`，`build_control` 返回 `Control`）；Runtime 持有并驱动。

## 遥测上行（不经 Controller）

显示遥测/姿态由 Runtime **直接经 `orbitx_vessel::telemetry` 读 `Assembly`** → 切片 → zenoh → 客户端。Controller/WorkFlow 不在这条路径上。

`telemetry` trait（在 `orbitx-vessel::telemetry`，纯读、无 mutation、无 copy）两个消费者：
- `BaseController`（控制环读，经 base 方法暴露给控制器）
- Runtime（显示切片读，直读 Assembly）

两者都直读 `Assembly`，不经彼此。

## tick 顺序（Runtime 编排；摘要）

完整冻结帧序见 [`RUNTIME.md`](RUNTIME.md)。摘要：

1. 可选飞行 UserInput（仅 `Control::Controller`；帧初）
2. Controller / WorkFlow tick（读 pre-step、写执行器；@ T0）
3. 环境（求 `GravBody` + 推进天体；P4.2 经 `PlanetarySystem`）
4. `Assembly::step`
5. Runtime 读 **post-step** telemetry → 切片

渲染显示步进后数据。会话暂停/倍速等**不**经 Controller/WorkFlow。

## 热路径性能

稳态每 tick 零堆分配：
- `ControlCapability` 不每 tick 重建（仅分离/对接事件点）。
- 遥测 index-set 方法返回惰性迭代器（借 Assembly 现有数据，无 mutation、无 copy）。
- 姿态/运动学/质量方法返回 `f64` / `(f64,f64)` / `Vec3`（Copy）。
- TVC / set_throttle 循环纯算术。
- group id lookup 在 ≤5 个 group 上线性扫描。

## WorkFlow TOML（模式 c/d，Godot 编辑）

TargetWorkFlow（模式 c）按阶段（`[[phases]]`）序列化目标 + transition；SuperWorkFlow（模式 d）按步骤（`[[steps]]`）编排 throttle/tvc/wait/separate 等。schema 与示例见 [`CONFIG_TOML.md`](CONFIG_TOML.md) 控制段（P4.1 阶段 B 落地）。

## 不做（P4.1 范围外）

- 不改 `orbitx-cli` 接线（P4.3 切 Zenoh 时退役）
- 不建 `orbitx-runtime` / 舰队驱动循环（P4.2）
- 不接 Zenoh / protobuf / SHM（P4.3 / P4.5）
- 不做 config 侧 `ControlCapabilityDesc` 命名/覆写（P4.1 用自动派生 id）
- 不动 demo-* / flight / launch / UserVessel
- 不做高程 / 接触入环（P5）
- 不建 `orbitx-environment`（P4.4）
