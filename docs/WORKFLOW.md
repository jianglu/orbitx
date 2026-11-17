# WorkFlow 图编排（权威设计）

游戏内（Godot GraphEdit）编排 **TargetWorkFlow / SuperWorkFlow** 的产品模型与连接规则。  
控制契约仍是 [`CONTROLLER.md`](CONTROLLER.md) 的 `ControlCapability` / `BaseController` / `TargetController`；本文件不发明 caps 之外的执行器。

| 相关文档 | 职责 |
|----------|------|
| 本文 | 图拓扑、三态、口/控制器/驱动、Target vs Super、颜色、存盘与 Godot 边界 |
| [`ROADMAP_WORKFLOW.md`](ROADMAP_WORKFLOW.md) | W1–W4 排期与 DoD |
| [`CONFIG_TOML.md`](CONFIG_TOML.md) | 现行线性 `wf_*.toml` 骨架；**图 IR 定稿后以本文为准演进 schema** |
| [`CONTROLLER.md`](CONTROLLER.md) | Base / Target / 四档 a–d |

**现状**：P4.1 已有线性 `[[phases]]` / `[[steps]]` 运行时。目标是图 IR →（校验）→ 运行时 WorkFlow；线性 TOML 可视为图的退化编译产物或过渡格式，细节在排期中收敛。

---

## 1. 目标与边界

- Godot：会话控制 + 图编辑；**不**在展示进程内嵌轨道步进。
- orbitx：`WorkFlow::tick` 快返回，只改控制面；物理在 Runtime。
- 不做：通用蓝图、对接流程、栅格翼 `deploy_target`（见 AERO 排期）、单喷管下标命令。

---

## 2. 节点类型三层

**主节点 = 级**（侧视/拓扑点选绑定级名）。

```text
点火                    （发射命令；不进入下面两层）
主节点(级)
  └─ 控制器：条件 / 延时（可选，可互嵌，深度不限）
        └─ 驱动（叶）：
              ├─ 激活
              └─ Base / Target 对应的能力驱动器
```

| 层 | 类型 | 说明 |
|----|------|------|
| 点火 | 发射命令 | **恰好连接一个**主节点；由客户端一次触发（见 §5.0）；触发后自身失效，目标主节点 → ACTIVE |
| 主节点(级) | 级 | 状态 `IDLE` / `ACTIVE` / `INVALID`；按本级能力露口 |
| 控制器 | 条件、延时 | 门控；不算驱动 |
| 驱动（叶） | 激活 + 能力驱动 | 无出边到控制器；激活特有「→ 下一主节点」 |
| 停机 | **Halt 指令**（§5.5） | 图末端停机原语；**三开关参数化**（缺省全真 = 真停机）；语义 =「本段终止」；与竞赛解耦（「任务完成」是客户端赋予的语义） |

---

## 3. 主节点能力口

按「本级是否具备该类能力」露口，**与器件个数无关**（有两个分离器仍只有一个分离口）。

定稿结构：**飞控 / 分离 / \<功能\>**

| 口 | 露口条件 | 子树驱动域 | v1 |
|----|----------|------------|-----|
| **飞控口** | 恒露出（v13 两 kind 统一：seed 前缀 = wf_target 快照，mode 驱动可挂无执行器级，投影不因 kind 收窄） | `TargetMode` 引导与 `throttle` / `tvc` / `rcs` 直驱**均合法**（v13 口域统一，与 kind 无关；油门在 TargetMode 参数内或由直驱写） | 有 |
| **分离口** | ≥1 分离器 | 仅分离器驱动 + **激活**驱动 | 有 |
| **功能口** | ≥1 功能件（伞/腿/板等） | 功能驱动 | 口位预留；caps 就绪后启用 |

规则：

- **驱动域只由祖先主节点口决定**，与中间控制器类型无关。
- 能力口可挂控制器和/或**直连驱动**。
- 口随装配/caps 有无增减；不随器件数变成多口。多执行器用多个驱动叶区分。

---

## 4. 连接规则（闭集）

| 从 → 到 | 允许 |
|---------|------|
| 点火 → 主节点 | 是（恰好 1） |
| 主节点.口 → 条件 / 延时 | 是 |
| 主节点.口 → 驱动 | 是（直连） |
| 主节点.口 → 停机（Halt） | 是 |
| 条件 ↔ 延时 | 是（互嵌） |
| 条件 / 延时 → 驱动 | 是（域随祖先口） |
| 条件 / 延时 → 停机（Halt） | 是 |
| 条件 / 延时 → 主节点 | 否 |
| 能力驱动 → 任意 | 否（叶） |
| 激活 → 下一主节点 | 是（必接；无交棒则不要放激活） |
| 激活 → 控制器 / 其他驱动 | 否（叶） |
| 激活挂在非分离口 | 否 |
| 跨口域驱动（如飞控口下挂分离器） | 否 |
| 停机（Halt） → 主节点 | 仅**前缀 halt** 允许恰好 1 条出边 → 增量段入口（§5.5 / §9.1）——单文件 C3b 整图 = 图内真实 halt→main 边；DLC 叠加形态 = `entry_link` 合成产物；`target` 图 halt 出边必须 = 0（§5.5） |
| 驱动 / 激活 → 停机（Halt） | 否 |

条件控制有 **true / false** 两出口：本拍谓词为真只开 true 子树，为假只开 false 子树；未接线侧无动作。谓词集 v1 **先全量**（高度/速度/远近地点/燃料%/时间/姿态相关等可读集），再按问题减法。与/或/非组合。

---

## 5. 调度与语义

### 5.0 谁触发点火（接口保留）

点火**不是**每拍自动发生，而是客户端一次命令，类似 cli 里 `w` 键通过接口改油门的模式：场景 UI（按钮等）或键位绑定 → 会话/`InputCmd`（名称实现时定，如 `WorkflowIgnite`）→ Runtime → `WorkFlow` 消费。

- **orbitx**：`WorkFlow` 保留 `ignite()`（或等价一次入口）；未触发前点火节点与下游主节点保持未启动；触发后点火失效、所连主节点 → ACTIVE。
- **Godot**：场景留 UI/输入绑定位，不在文档规定具体控件样式。
- **cli**：可按同样接口加键位（不必占用 `w`；`w` 仍是油门切换的参考交互形态）。

每拍（`sim_dt`）：

1. `WorkFlow::tick` 快返回（若本拍刚收到 ignite，先完成 §5.0 再调度）。
2. 枚举所有 **ACTIVE** 主节点（允许多个；教学可抢执行器，引擎不强制同组合体单 ACTIVE）。
3. 对各已露出口：直连驱动 + 控制器子树。
4. 飞控/功能能力驱动可每拍 hold；**分离器驱动与激活为一次有效**（使能执行后用尽，本主节点本次 ACTIVE 周期内不再执行）。
5. `IDLE` / `INVALID` 主节点整棵子树不算。

### 5.1 延时控制

- 配置：`count`、`interval_s`（≥0，仿真秒）。
- **`count`**：`≥1` 为有限次数；**`0` 表示无限**（祖先生效期间按间隔一直脉冲，直到主节点离开 ACTIVE）。
- 祖先生效时才累加 `sim_dt`；满间隔脉冲打开子树一次；`interval_s = 0` 表示祖先生效时每拍脉冲（有限时仍受 `count`；无限则每拍）。
- 主节点进入 ACTIVE 时重置该主下全部延时累加器与已触发次数。
- 不堵 Runtime（假 wait）。

### 5.2 激活驱动

- 仅分离口域；**叶**；必指下一主节点。
- **一次有效**；可选是否将**当前**主节点置 `INVALID`（可只激下一主、当前仍 ACTIVE）。
- 无自带谓词；何时使能由上游条件/延时或分离口直连决定。
- 物理 `separate` 只由**分离器驱动叶**完成，与激活并列、互不为父子。
- 同拍多驱动顺序**不保证、不重要**（只改后续控制面）。

### 5.3 交棒与级名

- 无分离口的级不自挂激活；交棒只能由仍有分离口的主节点上的激活指入（符合发射程序）。
- 用户只认**级名**；分离体沿用级名，激活的下一主节点按级名关联。引擎 `BodyRef` 对用户不可见。

### 5.4 校验与标红（编辑期 / 加载期）

流程上某节点无法驱动（口消失、caps id 失效、激活未接下一主等）→ **标红提醒**，勿静默错飞。

### 5.5 Halt 指令（停机原语，参数化）

图末端**停机指令节点**：**三开关参数化**（C3 定稿，替代初版「纯标记无参数」）、**不知道
任何客户端业务**（「竞赛任务完成」是 DLC 侧赋予 Halt 的语义，见 sim-rocket `COMPETITION.md`
§8；自由工坊可直接使用 = 「程序到此停机」的中性功能）。

**节点参数**（图 IR `type = "halt"` 三个 bool；serde 缺省全 `true` = 真停机，旧文件不含
字段 → 解析为缺省，向后兼容，schema 维持 1）：

| 参数 | 缺省 | 语义 |
|------|------|------|
| `shutdown_controls` | `true` | 停全部控制输出——关节流阀、停 RCS/TVC 等一切力学输出能力（航天器停机）。**锁存后每拍持续断言清零**（幂等）：`BaseController` 命令写入 vessel 后持续生效、无自动衰减，仅激活当拍清零会在后续拍保持推力 |
| `stop_stepping` | `true` | 停止仿真步进（切片停发）；runtime 进停机终态（`Shutdown` / `Reset` 仍响应，见 RUNTIME.md） |
| `save_recorder` | `true` | 黑匣子 flush 收尾 + 上行停机事件 `Halted`（未启用录制则仅发事件、目录为空）；`false` = **不发** `Halted` |

**呈现标识 `title`（可选 String，C3 补全轮）**：halt 节点可携带的**能力字段**——客户端
语义键（呈现标识），serde 缺省 `None`、序列化 `None` 跳过；引擎解析支持但**执行路径
零消费**（visit / validate / dispatch 均不读）；业务含义 = 客户端赋予（引擎不理解
竞赛，见 sim-rocket `COMPETITION.md` §8）。字段登记见 §9.2。

**执行规则：Halt 节点角色由出边数判定（C5 裁定 2026-11-16；修订 C3「一律按参数执行」
口径）**——C3 定稿建立在单文件整图前提上；C5 起**角色一律看合成图出边数**（单文件
super 图自身即合成图；DLC 叠加文件经 `entry_link` 拼接，§8.1 / §9.1）：出边 = 0 的
Halt 与出边 = 1 的前缀边界 Halt 在文件内都可能是出边 = 0 的节点，前缀角色只能由合成图
上的**出边数**表达（与本节末校验规则同口径；sim-rocket 侧 C3b prefix/suffix 分槽仅在
DLC 场景简化——prefix 槽作废，suffix 槽仅用于 DLC 叠加文件末段 halt；本体工坊现状
保留）：

| 出边数 | 角色 | 执行语义 |
|--------|------|----------|
| `0`（末端） | 停机点 | **按节点三参数执行**：缺省全真 = 真停机（停步进 + flush + `Halted` 上行）；`shutdown_controls=true` 其余关 = 仅停控制（每拍断言清零，步进/录制继续）；全关 = 零指令静默 |
| `1`（仅前缀边界 Halt 合法） | **边界锚点** | **零停机动作**——不锁存三参数、不清零、不停步进、不存黑匣子、不发 `Halted`；纯图调度：激活出边目标主节点（增量段入口）+ **前缀侧现役 ACTIVE 主节点全部失效**（激活器 `invalidate_current=true` 语义的多级推广——防前段引导/驱动残留与后段同拍抢执行器）。业务语义 =「前段程序全体松手：控制输出保持最后值（滑行态），后段接管」；**控制模式交接**：Target 基础层引导段终止 → Super 叠加层（BaseController 直驱）接管（§8.1 / §9.1，DLC 形态） |

锁存语义（仅出边 = 0 的 Halt）：激活即锁存三参数为本次停机动作（`WorkFlow::halt_action()`
上报 runtime，见 RUNTIME.md 停机收尾）。**多 halt 分时锁存（C3b 修订）**：锁存器每次激活
时刷新——叠加图中途「仅停控制」点与末段停机点先后激活时以后者取值覆盖（前缀边界锚点
出边 = 1，**不参与锁存**；C5 前旧形态「前缀 halt 三关先锁存、末段覆盖刷新」随双文件
方案作废，controller 用例随 C5 P1 改写）。
已知组合注记：`save_recorder=true` 且 `stop_stepping=false` = 先 flush 收尾、后续帧不落盘
（当前语义，需要再演进）。

图 IR 校验（编辑期 / 加载期，双处同规则；拓扑规则不变）：

- `kind=target` 图：Halt 节点**出边数必须 = 0**（末端）；
- `kind=super` 图（统一按**合成图**校验——单文件整图自身即合成图；DLC 叠加文件经
  `entry_link` 拼接，§9.1）：恰好一个 Halt 出边 = 0（末段终点）；其余 Halt 中作为前缀
  边界者出边恰好 = 1（增量段入口；单文件形态 = 图内真实 halt→main 边，DLC 形态 =
  `entry_link` 合成产物），未被引用者出边必须 = 0（前缀 Halt 获得出边的唯一途径：
  单文件形态 = 图内边，DLC 形态 = `entry_link`——基础文件本身永远出边 = 0，零改动）；
- 激活/驱动不得连接 Halt（Halt 只接受主节点口 / 控制器 / 延时来源边，同 §4 闭集原则）；
- 参数为纯 bool，无额外校验（角色由出边数判定，见上表）；
- 叠加文件自身校验（加载期，合成前）：`base` 引用存在且为 `kind=target` 图；
  `entry_link.from` 指向基础图中的 Halt 节点、`to` 指向本文件主节点；**基础图 Halt 恰
  恰 1**（即 `entry_link.from` 所指——引擎不拦多 Halt（单文件多停机点合法），DLC 形态
  收紧：多余 Halt = 真停机残留会提前终局 + `from` 引用歧义）；**增量段 `edges` 不得
  引用基础图节点 id**（跨文件边仅 `entry_link` 合成产生，否则合成图上成为真实边绕过
  闭集）；增量段自身拓扑合法（闭集 + 口域，`to` 主节点的唯一跨文件入边由 `entry_link`
  合成后自然满足）；
- 客户端可附加业务级校验（如「每程序恰好一个 Halt + 身份匹配」），由 DLC 编排场景经注入
  API 携带（本引擎不强制；自由程序无 Halt 照常运行）。

---

## 6. Target vs Super

| | Target（`kind=target`） | Super（`kind=super`） |
|--|------------------------|----------------------|
| 飞控口驱动 | `TargetMode` 与 `throttle` / `tvc` / `rcs` 均合法（v13 口域统一） | 同左 |
| 分离口 | 分离器 + 激活 | 同左 |
| 功能口 | 预留 | 预留 |
| 飞控口条件个数 | 不限（2026-11-15 修订：撤销 v13「≤1」静态校验——条件每拍只走一支，子树累加计数把嵌套分段引导误判为并发；同拍多引导按下方写序语义执行并运行时告警） | 同左 |
| 姿控计算 | `TargetController` 内置 | 用户拼 Base 单元；v1 开环跑通 |

`TargetMode`：`vertical_hold` / `pitch_to` / `nose_to`（罗盘机头指向：`azimuth` 方位 0=北顺时针、`elevation` 仰角；参数同现行 CONTROLLER / CONFIG_TOML）/ `prograde_hold` / `retrograde_hold` / `gravity_turn`。  
多 TargetMode 叶同时激活：**不拦截**——`set_mode` 互覆，同拍仅**遍历序最后者生效**（先 visit 叶当拍被覆盖，为死代码）；运行时对同拍多次引导发令打 WARN。同类型节点同色（见下）。

**同拍混用写序注记（v13；2026-11-15 去规则化修订）**：TargetMode 引导（`target.tick` 内嵌油门写入）与 `throttle` 直驱同拍作用于同一执行器组时，**后执行者胜出**（图遍历序决定，行为确定但不作为编排契约）。同拍多引导发令由运行时打 WARN（人话提示「仅最后一个生效」）；编辑器侧软提示属交互层职责，不做静态拒绝。TVC/RCS 直驱与 TargetMode 姿态回路同拍叠加属开环偏置，语义同 Super 档现行行为。

---

## 7. 颜色

同类型同色。主节点色 = **火箭拓扑该级色**（侧视/装配拓扑与 GraphEdit 共用级色板）。

| 类型 | Hex（参考） |
|------|-------------|
| 点火 | `#E8A317` |
| 主节点(级) | 级色板（见下） |
| 条件 | `#7B6BC7` |
| 延时 | `#2A9D8F` |
| 飞控驱动 | `#2E8B57` |
| 分离器驱动 | `#D35400` |
| 激活 | `#C2185B` |
| 功能驱动 | `#607D4E` |
| 停机（Halt） | `#546E7A` |
| 异常标红描边 | `#E53935` |

级色板参考（按级稳定序循环；权威一处共享）：

| 序 | 例 | Hex |
|----|----|-----|
| 0 | S1 | `#4A90D9` |
| 1 | S2 | `#E07A3D` |
| 2 | S3 | `#6BCB77` |
| 3 | 助推 A | `#9B59B6` |
| 4 | 助推 B | `#F4D35E` |
| 5 | 上面级 | `#E76F51` |

---

## 8. Godot 与存盘（产品边界）

- 独立 feature：`sim-rocket/features/workflow/`（**不**并入 `features/design/`）。
- 渠道：`channels/*/manifest.json` 的 `enabled_features`；精简渠道默认关。
- 设计工坊：feature 启用才显示入口；浮窗叠在设计场景上，不切走工坊路由。
- 目录：正式 `spacecraft/sc_<uuid>/`；未保存用 `sc_cache/`（全局一份）。文件名**双形态分流**（C5 裁定 2026-11-16）：本体设计工坊沿用现状单文件固定名 `wf_target.toml` / `wf_super.toml`（两者都在时默认打开 Target）；DLC 竞赛任务采用多文件三段通式（§8.1）。

### 8.1 DLC 多文件形态（C5 裁定 2026-11-16）

**双形态分流**：本体设计工坊沿用现状单文件（`wf_target.toml` / `wf_super.toml`，
整图自足，零改动零迁移）；**DLC 竞赛任务**采用基础文件 + 叠加文件多份存盘——
叠加文件不含基础图任何节点，替代单文件「前缀快照 + 增量段」整图形态。

**文件名三段通式**（DLC 形态专用）：

```
wf_<target|super>_<base|addon>_<DLC任务id>.toml
     └控制模式─┘ └─层角色─┘  └─任务标识─┘
```

| 段 | 语义 | 依据 |
|----|------|------|
| 第一段 | **WorkFlow 控制模式**：`target` = TargetWorkFlow → TargetController（目标导向引导）；`super` = SuperWorkFlow → BaseController（编排 + 直驱）——文件名即控制模式声明 | CONTROLLER.md 模式 c/d |
| 第二段 | **层角色（创建时定性）**：`base` = 被引用的基础程序；`addon` = 带引用头的叠加文件。被谁引用是运行时事实，不改名 | — |
| 第三段 | **DLC 任务 id**（如 `t_orbit` / `t_circulate`），任务 → 文件按 id 直查 | 消费键显式给完整文件名，加载器零猜测 |

当前业务实例：

| 文件 | 模式 | 内容 | 写入方 |
|------|------|------|--------|
| `wf_target_base_t_orbit.toml` | TargetWorkFlow（引导） | 任务一完整发射程序，作基础层（**零改动**——叠加文件不含其任何内容） | DLC 设计阶段① |
| `wf_super_addon_t_circulate.toml` | SuperWorkFlow（直驱） | **引用头 + 增量段**：`base = "wf_target_base_t_orbit.toml"` + `entry_link`（§9.1）+ 增量段 `nodes`/`edges` | DLC 设计阶段④ 增量编排 |

要点：

- **形态区分 = IR 结构性判据**：`kind=super` 文件有无 `base` 键——无 = 单文件整图
  （本体现状路径照旧）；有 = 叠加文件（读引用头拼接合成，§9.1）；
- **基础文件结构性只读**：叠加文件不含前缀内容，任务二编排对任务一程序无从写起
  （DLC 场景替代 C3b「seed 快照 + E6 四面比对」保护——该链仅 DLC 场景拆除）；
- 前缀只读的交互锁在编辑器（禁编辑 / 禁拖拽 / 禁删——叠加会话中基础图节点画布位同样
  不可改，因无处落盘）；
- 会话加载：消费键显式给完整文件名——任务一 → `wf_target_base_<task1>.toml`；任务二
  → `wf_super_addon_<task2>.toml`，orbitx 加载层读 `base` 引用同目录拼接合成单图
  （§9.1）后走既有校验与执行器——**执行器与校验器消费的永远是合成图**；
- **边界切换 = 控制模式交接**（§5.5）：基础层 TargetController 引导段终止 → 叠加层
  SuperWorkFlow 接管（BaseController 直驱）；
- 后续任务（任务三…）按通式直接加文件（`wf_super_addon_<task3>.toml`），其 `base`
  头可指向既有叠加文件 → 链式叠加；
- 存量兼容（2026-11-16 裁定修订）：DLC 竞赛池识别旧固定名 `wf_target.toml` → **直接
  重命名**为 `wf_target_base_<task>.toml`（内容零变化，仅任务一产物存量——任务二编排
  是 C5 新功能无叠加存量，无拆分问题）；改名 IO 失败 → 兜底回落旧名直载（target 图
  新旧名等价，引擎零感知）。本体工坊固定名不受影响；resolve 收敛在 Godot 侧单点。
  写副作用注记（加固③，2026-11-17）：该改名发生在 `resolve_wf_path` **首次命中
  旧名时**（幂等，之后不再触发）；调用点含发射前置复校 / 会话装配 / DLC 校验器
  target_path 解析等多处，首次触发点不定——读路径语义的调用方须容忍该一次性改名。
- 关闭：非正式会话静默写 `sc_cache`；正式火箭未保存才三按钮（覆盖 / 另存留原目录 / 不保存）。细则实现时与设计存盘对齐，并同步 [`CONFIG_TOML.md`](CONFIG_TOML.md) 的 `sc_<uuid>` 规则。

---

## 9. 图 IR（定稿设计）

存盘与运行共用 **TOML 图 IR**（文件名双形态：本体工坊固定名 `wf_target.toml` /
`wf_super.toml`；DLC 竞赛三段通式见 §8.1）。Godot 只读写文件；orbitx 校验后按图调度。
`x`/`y` 仅编辑器布局，运行时忽略。

文件头：`schema` / `kind` / `name`（与航天器三文件版本规则一致，见 CONFIG_TOML）。
`kind` = WorkFlow 控制模式（`target` = TargetWorkFlow → TargetController；`super` =
SuperWorkFlow → BaseController，CONTROLLER.md 模式 c/d）——文件名第一段与 `kind` 一致。

### 9.1 总结构与叠加文件合成

**基础图（`kind=target`）**——单文件完整图（本体工坊 `wf_target.toml`；DLC 形态
`wf_target_base_<task>.toml`），总结构：

```toml
schema = 1
kind = "target"          # "target" | "super"
name = "demo-ascent"

[[nodes]]
id = "ign0"
type = "ignite"
x = 0.0
y = 40.0

[[nodes]]
id = "m_core"
type = "main"
stage = "Core"           # 级名；着色查级色板
x = 180.0
y = 40.0

[[nodes]]
id = "c_alt"
type = "condition"
combine = "and"          # "and" | "or"（无 not；取反靠 true/false 口叠加）
terms = [
  { kind = "altitude", op = "gt", value = 10000.0 },
]
x = 360.0
y = 0.0

[[nodes]]
id = "d_hold"
type = "drive"
# 飞控口下 mode（TargetMode 引导）与 action（throttle/tvc/rcs 直驱）均合法
# （v13 口域统一，见 9.3）
mode = "vertical_hold"
throttle = 1.0
x = 540.0
y = 0.0

[[nodes]]
id = "sep_booster"
type = "drive"
action = "separate"      # Super / 分离域；Target 分离叶同样用 action="separate"
point = "Booster-sep-0"
x = 540.0
y = 120.0

[[nodes]]
id = "act_next"
type = "activate"
invalidate_current = true
x = 540.0
y = 200.0

[[nodes]]
id = "m_upper"
type = "main"
stage = "Upper"
x = 720.0
y = 200.0

[[edges]]
from = "ign0"
to = "m_core"

[[edges]]
from = "m_core"
from_port = "flight"     # 主节点口：flight | sep | function
to = "c_alt"

[[edges]]
from = "c_alt"
from_port = "true"       # 条件口：true | false
to = "d_hold"

[[edges]]
from = "m_core"
from_port = "sep"
to = "sep_booster"

[[edges]]
from = "m_core"
from_port = "sep"
to = "act_next"

[[edges]]
from = "act_next"
from_port = "next"       # 激活唯一出边
to = "m_upper"
```

**叠加文件（`kind=super` + `base` 键，DLC 形态，§8.1）**——增量段 + 引用头，不含基础图
任何节点：

```toml
schema = 1
kind = "super"
name = "circulate"
base = "wf_target_base_t_orbit.toml"   # 基础图引用（同目录 sibling；必须 kind=target）
                                        # ↑ `base` 键存在性 = 叠加文件形态判据（无 = 单文件整图）
entry_link = { halt = "h1", to = "m_burn" }   # 前缀边界锚点接线（合成期展开为一条边）
                                  # halt = 基础图中 Halt 节点 id；to = 本文件主节点 id

[[nodes]]                          # —— 以下全部为增量段（基础图零内容）——
id = "m_burn"
type = "main"
stage = "Upper"
x = 0.0
y = 40.0

# …增量段 nodes / edges（闭集与口域规则同上；末段 Halt 出边 = 0）…
```

合成规则（orbitx 加载层，执行器/校验器只见合成图）：读 `base` 图 → 读叠加文件 →
把 `entry_link` 展开为基础图 `halt` 节点的一条出边（`→ to` 主节点）→ 两图节点/边合并
（节点 id 两文件各自命名空间，合成后冲突即拒载）→ 合成图走 §5.5 校验（唯一前缀 halt
出边 = 1 恰为 entry_link 产物）→ 调度执行。合成图节点携带来源形态（target 基础层 /
super 叠加层）——**边界切换 = 控制模式交接**：前缀边界锚点激活增量段入口即
TargetController 引导段终止、SuperWorkFlow（BaseController 直驱）接管（§5.5）。
`x`/`y` 布局：编辑器叠加会话中基础图节点画布位只读（§8.1），叠加文件不存基础节点位。

### 9.2 节点类型字段

| `type` | 必填字段 | 可选 | 出边端口 |
|--------|----------|------|----------|
| `ignite` | `id` | `x`,`y` | （默认）→ 恰好 1 个 `main` |
| `main` | `id`, `stage` | `x`,`y` | `flight` / `sep` / `function`（无能力则禁止使用该 port） |
| `condition` | `id`, `combine`, `terms` | `x`,`y` | `true` / `false` |
| `delay` | `id`, `count`（`≥1` 有限；**`0`=无限**）, `interval_s` (≥0) | `x`,`y` | （默认）→ 控制器或驱动 |
| `drive` | `id` + 见 9.3 | `x`,`y` | 无（叶） |
| `activate` | `id`, `invalidate_current` | `x`,`y` | `next` → 恰好 1 个 `main` |
| `halt` | `id` | `x`,`y` + 三开关 `shutdown_controls` / `stop_stepping` / `save_recorder`（bool，缺省全 `true` = 真停机）+ `title`（可选 String，呈现标识能力字段，缺省省略；见 §5.5） | 无（文件内恒末端；出边数 = 角色判据：出边 0 = 按参数停机，出边 1 = 边界锚点——仅合成图中经叠加文件 `entry_link` 产生，见 §5.5 / §9.1） |

`terms[]` 元素：`{ kind, op, value }`（与线性 `transition` 单条同形）。`kind` 仅为被测量；`op` ∈ `gt`\|`lt`\|`gte`\|`lte`\|`eq`；`terms` 至少 1 条。

| kind | value 单位 | 切片字段 |
|------|------------|----------|
| `altitude` | m（距地表） | `focus.altitude` |
| `speed` | m/s（对地速率） | `focus.speed` |
| `v_vert` | m/s | `focus.v_vert` |
| `v_horiz` | m/s | `focus.v_horiz` |
| `apoapsis` | m（距地表） | `orbit.ap_alt` |
| `periapsis` | m（距地表） | `orbit.pe_alt` |
| `fuel_pct` | %（0..100，整箭） | `focus.fuel_pct` |
| `stage_fuel_pct` | %（0..100，所属主节点的级） | `stages` 中 `name` 与该级相同的 `fuel_pct`；没有该级则为假 |
| `dynamic_pressure` | Pa | `focus.env.dynamic_pressure` |
| `static_pressure` | Pa | `focus.env.pressure` |
| `density` | kg/m³ | `focus.env.density` |
| `mach` | — | `focus.env.mach` |
| `twr` | — | `focus.twr` |
| `time` | s（该条件节点本次生效累计，非墙钟） | 不在切片上 |
| `tip_angle` | rad | `focus.tip` |
| `pitch` / `yaw` / `roll` | rad | `focus.pitch` / `yaw` / `roll` |
| `aoa` | rad（无符号迎角） | `focus.aoa` |
| `pitch_rate` / `yaw_rate` / `roll_rate` | rad/s | `focus.omega` 的 x / z / y |
| `tvc_pitch` / `tvc_yaw` | rad | `focus.gimbal_pitch` / `gimbal_yaw` |
| `throttle` | 0..1 | `focus.throttle` |

条件在控制拍读取**上一步**切片。第一步切片未就绪时，除 `time` 外这些条件为假。编辑器里角度与角速率按度填写，节流按百分数填写，写入文件时换回上表单位。

`combine = "and"`：全部 term 真为真；`"or"`：任一真为真。不做单卡「非」：对立动作接 `false` 口；更复杂否定再串一级条件控制器。

### 9.3 `drive` 载荷（随祖先口域校验；v13 与 `kind` 解耦）

**域不写进文件**：由边回溯到所属 `main` 的 `from_port` 决定；校验时驱动类型必须属于该域。v13 口域统一后驱动合法性**只由端口域决定**（`check_drive_domain` 无 kind 参数）。

| 文件 `kind` | 口域 | `drive` 字段 |
|-------------|------|----------------|
| `target` / `super` | `flight` | `mode` = TargetMode 名 + 该模式参数（同现行 phases）**或** `action` ∈ `throttle`\|`tvc`\|`rcs` + 对应字段（同现行 steps）；两类不可混写在同一节点 |
| `target` / `super` | `sep` | `action = "separate"`, `point` = 分离点 id |
| 任一 | `function` | v1 禁止出现；口预留 |

数值校验与 kind 无关：`throttle` 的 `level` 须在 [0,1]；`tvc` 的 pitch/yaw 限幅同执行器。

Godot 侧镜像校验（`simrocket_native` `WorkflowCanvas.validate_toml`，编辑器保存/加载链）已对齐本节口径（2026-11-17 修复⑥）：`check_drive` 同样只按端口域判定（kind 参数退役），`E_TARGET_ACTION` / `E_SUPER_MODE` / `E_TARGET_CONDITION`（≤1，随 orbitx 3dc6527 撤销）退役；悬空驱动（mode/action 皆空）报 `E_DRIVE`。能力**供给**收窄（级别 → Controller 固定映射）仍归编辑器，与合法性校验分属两层。

飞控直驱示例（target / super 图通用）：

```toml
[[nodes]]
id = "thr"
type = "drive"
action = "throttle"
group = "Core"
level = 1.0
```

### 9.4 边

```toml
[[edges]]
from = "<node_id>"
from_port = "<port>"   # 可省略：非 main/condition/activate 时视为 "out"
to = "<node_id>"
to_port = "in"         # 可省略；预留
```

合法 `(from.type, from_port) → to.type` 即本文 §4 闭集。校验失败 → 编辑器标红 + 加载拒绝运行。

### 9.5 运行时状态（不进 TOML）

调度器内存态：每 `main` 的 `IDLE|ACTIVE|INVALID`；每 `delay` 的累加器与已触发次数；每 `drive(separate)` / `activate` 的「本 ACTIVE 周期是否已用尽」。进入 ACTIVE 时重置该主子树内延时与一次有效标记。

### 9.6 与线性格式

- **新文件**：只写图 IR（本节）。
- **旧 `[[phases]]` / `[[steps]]`**：继续可被现行解析器加载，直至 W1+ 迁移完成；不与图 IR 混在同一文件。
- 不要求 Godot 再导出线性格式；若需兼容 cli 旧路径，可由工具图→线性单向编译（非产品主路径）。

字段表权威在本文；[`CONFIG_TOML.md`](CONFIG_TOML.md) 控制工作流节保留现行线性说明并链到本节。实现时 `orbitx-controller` 增加 `graph` 模块解析本 IR。

---

## 10. 与现行代码的关系

| 现行 | 目标 |
|------|------|
| `TargetWorkFlow` 线性 phases + transition | 图：主节点飞控口上的条件 + TargetMode 驱动；transition 语义并入条件控制器 |
| `SuperWorkFlow` 线性 steps + wait | 图：飞控/分离驱动 + 延时控制器；`wait` → 延时 |
| 单指针推进 | 多 ACTIVE 主节点 + 一次有效分离/激活 |

P4.1 行为测在迁移期保持绿；新图 IR 落地后以本文 + 单测为准。
