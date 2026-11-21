# WorkFlow 图编排排期（ROADMAP_WORKFLOW）

产品图编排（Godot GraphEdit ↔ orbitx WorkFlow）的分阶段排期。  
权威设计见 [`WORKFLOW.md`](WORKFLOW.md)。阶段号用 **W\***，避免与主 [`ROADMAP.md`](ROADMAP.md) 的 P0–P6、[`ROADMAP_AERO.md`](ROADMAP_AERO.md) 的 A\* 撞号。

## 与 P4 的关系

| | |
|--|--|
| P4.1 | 线性 Target/Super TOML + 运行时 ✅（骨架） |
| 本主题 | 图模型 / IR / Godot 编排 / 调度语义对齐 [`WORKFLOW.md`](WORKFLOW.md) |
| P4.5 | Godot bridge / 会话；**W4 可与 P4.5 部分并行**（W1–W3 偏 orbitx） |

**实施次序**：文档确认 → W1 → W2 → W3 → W4。确认前不写生产代码（单测草稿除外，可选）。

实施时的改动面 / 禁令 / 模块落点见工作区计划「WorkFlow 图编排 — 实施计划（整理版）」；**拓扑与 IR 仍以本文链到的 WORKFLOW.md 为准**，避免两处细则分叉。

## 当前状态 / 目标

| | |
|--|--|
| 现状 | 线性 `wf_*.toml`；无游戏内图编排；无主节点三态图调度 |
| 目标 | 图 IR + ACTIVE 调度；Target/Super 分栏；Godot `features/workflow` 浮窗存盘 |
| 本轮不做 | 功能口真实驱动、对接、栅格翼 deploy、通用蓝图 |

## 阶段表

| 阶段 | 内容 | 验收（测什么 / 看到什么） |
|------|------|--------------------------|
| **W0** | [`WORKFLOW.md`](WORKFLOW.md) + 本文件；主文档指针 | 文档互链；无代码行为变化 |
| **W1** | 图 IR + `ignite()` + 条件/延时（`count=0` 无限）+ Target 飞控 + 多 ACTIVE | **单测 DoD**：round-trip、校验拒边、ignite 一次、ACTIVE only、条件双口、delay 有限/无限、TargetMode、多 ACTIVE；**线性旧测回归** |
| **W2** | Super 飞控 throttle/tvc/rcs；kind 分流 | **单测**：三 action 下发；target 文件错载 Super action 失败；多条件；W1+线性回归 |
| **W3** | 分离/激活一次有效；级名交棒 | **单测**：separate/activate 各一次；invalidate 开/关；拒无 sep 口挂激活；级名 ACTIVE；回归 |
| **W4** | `features/workflow` 浮窗；级色；存盘；会话 ignite | gdext IR/校验单测；手工验收清单；`cargo test -p orbitx-controller` 绿 |

**单测总则**：W1–W3 以 `orbitx-controller` 为准，合并前全绿；测夹具/样例 TOML 固定落盘；禁止只靠手工 cli。细则用例表见实施计划整理版。

## 进度

- [x] W0 文档初稿（待产品确认后勾「已确认」）
- [x] W0 **产品确认**（拓扑 / 颜色 / 存盘）→ 再动 W1 代码
- [x] W1 图 IR + Target 闭环
- [x] W2 Super 飞控单元
- [x] W3 分离 / 激活 / 级名
- [x] W4 Godot feature + 会话

## DoD（主题完成）

- Target 图可驱动上升类飞行（经 Runtime tick）；**W1–W3 单测 DoD 已落地且 `cargo test -p orbitx-controller` 绿**。
- Super 可下发 Base 能力，并完成分离器 + 激活交棒。
- Godot 可按级着色编排、校验标红、写入 `wf_target.toml` / `wf_super.toml`。

> 注：现行产品路径（竞赛 DLC）写入的是图 IR（`[[nodes]]` / `[[edges]]`，`<kind>:<layer>:<task_id>.toml`
> 通式名），由 `GraphWorkFlow` 执行；上面这条 DoD 描述的是 W4 当时的线性档口径。

## 待办 / 架构债

### TD-1 图与线性双实现未收敛（P0，见 `REQUIREMENTS_RELEASE_1215.md` §2.13）

**现状**：`GraphWorkFlow`（本目录 `graph/`，`e90c0c4` 2026-10-12 随 W1–W4 落地）是**平行重写**
的第二套执行器，未复用 `TargetWorkFlow` / `SuperWorkFlow` 的任何逻辑。WORKFLOW.md 开篇所述
「线性 TOML 可视为图的退化编译产物」的收敛意图从未落地。

两条加载路径互不相干（`orbitx-runtime/src/cli/mod.rs::load_workflow` 按是否含 `[[nodes]]` 分流）：

| 路径 | TOML 形态 | 实现 | 产品路径 |
|------|-----------|------|----------|
| `SessionControl::Graph` | `[[nodes]]` + `[[edges]]` | `GraphWorkFlow` | **竞赛唯一路径** |
| `SessionControl::WorkFlow` | `[[phases]]` / `[[steps]]` | `TargetWorkFlow` / `SuperWorkFlow` | 非产品路径（迁移期旧档） |

**后果**：同一概念在两处各写一遍，已实际造成分歧：

| 概念 | `TargetWorkFlow` | `GraphWorkFlow` | `SuperWorkFlow` |
|------|------------------|-----------------|-----------------|
| `throttle_cmd()` | 读 target（正确） | 已修（`last_throttle` 锁存，2026-11-21） | **未实现**（trait 默认 0.0） |
| 能力域 | `[[phases]].mode` | `DriveSpec::Target` + `DriveSpec::Action` | `[[steps]].action` |
| 停机清零 | — | `apply_shutdown_controls` | `reset_halt_state`（不含回显） |

2026-11-21 真机缺陷即由此暴露：竞赛 super 组别只许 `action = "throttle"` 直驱，直驱只写执行器
不写指令记录 → 切片 `throttle_cmd` 恒 0 → HUD 节流阀手柄不动、读数 0%，而引擎实际满推（TWR 1.93）。

**待解**：

1. `SuperWorkFlow` 的 `throttle_cmd()` 缺口补齐；
2. `DriveSpec` 与 `[[phases]]` / `[[steps]]` 的能力域镜像关系是否形式化（对拍表 / 共享类型）；
3. 是否以 `GraphWorkFlow` 包装两个线性执行器取代重写（收敛路径）。

**验收**（同 `REQUIREMENTS_RELEASE_1215.md` §2.13）：三实现 `throttle_cmd()` 契约测试全绿；
控制面语义单一真相源可指认；若豁免收敛须说明防漂移手段。

## 文档确认清单（写代码前）

- [x] [`WORKFLOW.md`](WORKFLOW.md) §2–§5 拓扑与调度无异议
- [x] Target / Super 差异（§6）无异议
- [x] 颜色与级色板（§7）可落地
- [x] 存盘 / feature 边界（§8）与设计工坊规则一致
- [x] 图 IR（[`WORKFLOW.md`](WORKFLOW.md) §9）字段与样例无异议后开 W1
