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
| 停机 | **Halt 指令**（§5.5） | 图末端停机原语；语义 =「本段终止」；与竞赛解耦（「任务完成」是客户端赋予的语义） |

---

## 3. 主节点能力口

按「本级是否具备该类能力」露口，**与器件个数无关**（有两个分离器仍只有一个分离口）。

定稿结构：**飞控 / 分离 / \<功能\>**

| 口 | 露口条件 | 子树驱动域 | v1 |
|----|----------|------------|-----|
| **飞控口** | ≥1 油门组 / TVC / RCS，或 Target 图 | Super：`throttle` / `tvc` / `rcs`；Target：`TargetMode`（油门在模式参数内） | 有 |
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
| 停机（Halt） → 主节点 | 仅 `super` 图**前缀** halt（恰好 1 条 → 增量段入口）；`target` 图 halt 出边必须 = 0（§5.5） |
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

### 5.5 Halt 指令（停机原语）

图末端**停机指令节点**：纯标记、无参数、**不知道任何客户端业务**（「竞赛任务完成」是
DLC 侧赋予 Halt 的语义，见 sim-rocket `COMPETITION.md` §8；自由工坊可直接使用 =
「程序到此停机」的中性功能）。

**核心语义：Halt = 「本段终止」**，执行上下文决定其含义：

| 执行上下文 | Halt 的含义 |
|-----------|------------|
| 单程序（`kind=target` 图独立执行 / 自由程序） | **真停机**：停止仿真步进 → 黑匣子 flush 收尾 → 上行停机事件（`Halted`，见 RUNTIME.md） |
| `kind=super` 组合执行、Halt 位于**前缀段**（快照自 target 图） | **不停机**——控制权移交边界：前缀段终止，增量段接管（Super 衔接语义） |
| `kind=super` 组合执行、Halt 位于**末段** | 真停机（同单程序） |

图 IR 校验（编辑期 / 加载期，双处同规则）：

- `kind=target` 图：Halt 节点**出边数必须 = 0**（末端）；
- `kind=super` 图：**恰好一个** Halt 出边 = 0（末段终点）；其余 Halt（前缀，通常 1 个）出边**恰好 = 1**（增量段入口）；
- 激活/驱动不得连接 Halt（Halt 只接受主节点口 / 控制器 / 延时来源边，同 §4 闭集原则）；
- 客户端可附加业务级校验（如「每程序恰好一个 Halt + 身份匹配」），由 DLC 编排场景经注入
  API 携带（本引擎不强制；自由程序无 Halt 照常运行）。

---

## 6. Target vs Super

| | Target（`kind=target`） | Super（`kind=super`） |
|--|------------------------|----------------------|
| 飞控口驱动 | 仅 `TargetMode` | `throttle` / `tvc` / `rcs` |
| 分离口 | 分离器 + 激活 | 同左 |
| 功能口 | 预留 | 预留 |
| 飞控口条件个数 | ≤1 | 可多 |
| 姿控计算 | `TargetController` 内置 | 用户拼 Base 单元；v1 开环跑通 |

`TargetMode`：`vertical_hold` / `pitch_to` / `nose_to`（罗盘机头指向：`azimuth` 方位 0=北顺时针、`elevation` 仰角；参数同现行 CONTROLLER / CONFIG_TOML）/ `prograde_hold` / `retrograde_hold` / `gravity_turn`。  
多 TargetMode 叶同时生效：**不规避**，抢执行器；同类型节点同色（见下）。

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
- 目录：正式 `spacecraft/sc_<uuid>/`；未保存用 `sc_cache/`（全局一份）。各固定一份 Target / Super：`wf_target.toml`、`wf_super.toml`。两者都在时默认打开 Target。
- 关闭：非正式会话静默写 `sc_cache`；正式火箭未保存才三按钮（覆盖 / 另存留原目录 / 不保存）。细则实现时与设计存盘对齐，并同步 [`CONFIG_TOML.md`](CONFIG_TOML.md) 的 `sc_<uuid>` 规则。

---

## 9. 图 IR（定稿设计）

存盘与运行共用一份 **TOML 图 IR**（文件名固定为 `wf_target.toml` / `wf_super.toml`）。Godot 只读写该文件；orbitx 校验后按图调度。`x`/`y` 仅编辑器布局，运行时忽略。

文件头：`schema` / `kind` / `name`（与航天器三文件版本规则一致，见 CONFIG_TOML）。

### 9.1 总结构

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
# Target：mode + 模式字段；Super：action + 对应字段（见 9.3）
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

### 9.2 节点类型字段

| `type` | 必填字段 | 可选 | 出边端口 |
|--------|----------|------|----------|
| `ignite` | `id` | `x`,`y` | （默认）→ 恰好 1 个 `main` |
| `main` | `id`, `stage` | `x`,`y` | `flight` / `sep` / `function`（无能力则禁止使用该 port） |
| `condition` | `id`, `combine`, `terms` | `x`,`y` | `true` / `false` |
| `delay` | `id`, `count`（`≥1` 有限；**`0`=无限**）, `interval_s` (≥0) | `x`,`y` | （默认）→ 控制器或驱动 |
| `drive` | `id` + 见 9.3 | `x`,`y` | 无（叶） |
| `activate` | `id`, `invalidate_current` | `x`,`y` | `next` → 恰好 1 个 `main` |
| `halt` | `id` | `x`,`y` | 无（末端；`super` 图中前缀 halt 恰好 1 条出边 → 增量段入口；出边规则见 §5.5） |

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

### 9.3 `drive` 载荷（随 `kind` 文件 + 祖先口域校验）

**域不写进文件**：由边回溯到所属 `main` 的 `from_port` 决定；校验时驱动类型必须属于该域。

| 文件 `kind` | 口域 | `drive` 字段 |
|-------------|------|----------------|
| `target` | `flight` | `mode` = TargetMode 名 + 该模式参数（同现行 phases） |
| `target` / `super` | `sep` | `action = "separate"`, `point` = 分离点 id |
| `super` | `flight` | `action` ∈ `throttle`\|`tvc`\|`rcs` + 对应字段（同现行 steps） |
| 任一 | `function` | v1 禁止出现；口预留 |

Super 飞控示例：

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
