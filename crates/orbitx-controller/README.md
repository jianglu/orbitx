# orbitx-controller

航天器控制策略（P4.1）。落地四档控制（a/b/c/d）与类层次（Base / Target / WorkFlow）。

**状态：阶段 A 骨架**（模块结构 + trait/类型签名 + `todo!()` 占位）。阶段 B 实现见 [`../../docs/ROADMAP.md`](../../docs/ROADMAP.md) P4.1。

**设计权威**：[`../../docs/CONTROLLER.md`](../../docs/CONTROLLER.md)（分层 / BaseController 门面 / 四档 / 类层次 / ControlCapability / 舰队模式 / 遥测上行 / tick 顺序 / 热路径性能）。本 README 仅给 crate 模块地图与边界速览。

## 边界

- **Controller 只负责控制逻辑**：写执行器、解析工作流、驱动目标导向算法。真正响应控制在 `orbitx-vessel`；controller 不重写物理、不重写查询、不存遥测副本。
- **依赖单向**：`orbitx-controller → orbitx-vessel`（+ `orbitx-math`、`serde`、`toml`），不反向依赖 Runtime。
- **cli 不动**：`orbitx-cli` 暂留旧 `control.rs`，P4.3 切 Zenoh 时退役。

## 分层速览

```text
BaseController（vessel 唯一全功能门面：读遥测 + 写执行器，持 &mut Assembly + &caps）
  ├── TargetController（模式 b，控制算法，不 own caps）
  │     └── TargetWorkFlow（模式 c，own 1 TargetController + 主 caps）
  └── SuperWorkFlow（模式 d，own 子控制器舰队 + 各 body caps，编排 + 分离派生，入轨自动驾驶）

模式 a：客户端实时轴/开关 → BaseController 直接调方法（不经 Controller trait）
```

`TargetController` / `WorkFlow` 都不直接碰 `Assembly`，必经 `BaseController`。详见 [`../../docs/CONTROLLER.md`](../../docs/CONTROLLER.md)。

## 模块

| 模块 | 职责 |
|------|------|
| [`base`](src/base/mod.rs) | `BaseController`（vessel 唯一全功能门面）+ `Controller` trait（叶控制器 tick 入口） |
| [`target`](src/target/mod.rs) | `TargetController`（模式 b，目标导向算法）+ `TargetMode` 枚举 |
| [`workflow`](src/workflow/mod.rs) | `WorkFlow` trait + `WorkFlowDesc`（TOML 数据模型）+ `from_toml_str`；[`target_wf`] / [`super_wf`] 子模块 |
| [`workflow/target_wf`](src/workflow/target_wf.rs) | `TargetWorkFlow`（模式 c，own 1 TargetController + 主 caps） |
| [`workflow/super_wf`](src/workflow/super_wf.rs) | `SuperWorkFlow`（模式 d，own 子控制器舰队 + 分离派生） |
| [`capability`](src/capability/mod.rs) | `ControlCapability` 契约 + `BodyRef` + `for_primary` / `for_detached` 投影 |
| [`throttle`](src/throttle/mod.rs) | `ThrottlePolicy`（ActiveOnly / SyncPrimary）+ `apply_throttle` |
| [`tvc`](src/tvc/mod.rs) | TVC 双轴 PD（`TVC_KP` / `TVC_KD` + `apply_tvc`） |
| [`separation`](src/separation/mod.rs) | `should_auto_separate` + `perform_separate`（返回 detached 下标） |
| [`rcs`](src/rcs/mod.rs) | `set_rcs`（按 group id 委托 `vessel::set_attitude_rot/lin`） |
| [`factory`](src/factory.rs) | `build_controller` / `build_workflow` / `build_control` + `Control` 顶层枚举 + `ControllerAssignment` |

## 不做（P4.1 范围外）

详见 [`../../docs/CONTROLLER.md`](../../docs/CONTROLLER.md)「不做」节。
