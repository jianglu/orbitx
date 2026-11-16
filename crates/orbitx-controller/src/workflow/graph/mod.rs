//! 图 IR（[`WORKFLOW.md`](../../../../docs/WORKFLOW.md) §9）：解析、校验、ACTIVE 调度。
//!
//! 与线性 `[[phases]]` / `[[steps]]` 分文件、分模块，不混解析路径。

mod runtime;

pub use runtime::GraphWorkFlow;

use std::collections::{HashMap, HashSet};
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::workflow::{SchemaError, TargetModeDesc, WorkFlowKind};

pub use crate::workflow::pred::{PredOp, PredTerm};

/// 主节点三态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MainState {
    Idle,
    Active,
    Invalid,
}

/// 能力口。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortDomain {
    Flight,
    Sep,
    Function,
}

/// 图 IR。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GraphIr {
    pub schema: u32,
    pub kind: WorkFlowKind,
    pub name: String,
    /// DLC 叠加形态（WORKFLOW.md §8.1 / §9.1）：引用的基础文件名（同目录 sibling）。
    /// 无 = 单文件整图直载（本体工坊形态）。仅 [`load_composed`] 消费，合成产物剥离；
    /// 直解析（[`from_toml_str`]）读到即拒。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub base: Option<String>,
    /// DLC 叠加形态边界锚点引用：基础图 `halt` → 叠加文件入口主节点 `to`。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry_link: Option<EntryLink>,
    #[serde(default)]
    pub nodes: Vec<GraphNode>,
    #[serde(default)]
    pub edges: Vec<GraphEdge>,
}

/// 叠加文件引用头（WORKFLOW.md §9.1）：基础图停机节点 → 叠加段入口主节点。
/// 经 [`compose`] 展开为合成图上的真实边界边（`from_port = "out"`）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntryLink {
    pub halt: String,
    pub to: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GraphNode {
    pub id: String,
    #[serde(flatten)]
    pub body: NodeBody,
    #[serde(default)]
    pub x: f64,
    #[serde(default)]
    pub y: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum NodeBody {
    Ignite,
    Main {
        stage: String,
    },
    Condition {
        combine: Combine,
        terms: Vec<PredTerm>,
    },
    Delay {
        count: u32,
        interval_s: f64,
    },
    Drive {
        #[serde(flatten)]
        spec: DriveSpec,
    },
    Activate {
        invalidate_current: bool,
    },
    /// 停机指令（WORKFLOW.md §5.5）：三开关参数化，缺省全 `true` = 真停机
    /// （旧文件不含字段 → 解析为缺省，向后兼容，schema 维持 1）。
    /// `title` = 呈现标识能力字段（客户端语义键，serde 缺省 `None`、序列化
    /// `None` 跳过；执行路径零消费——visit / validate / dispatch 均不读）。
    Halt {
        #[serde(default = "default_true")]
        shutdown_controls: bool,
        #[serde(default = "default_true")]
        stop_stepping: bool,
        #[serde(default = "default_true")]
        save_recorder: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        title: Option<String>,
    },
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Combine {
    And,
    Or,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DriveSpec {
    Action(ActionDrive),
    Target(TargetModeDesc),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum ActionDrive {
    Throttle {
        group: String,
        level: f64,
    },
    Tvc {
        group: String,
        pitch: f64,
        yaw: f64,
    },
    Rcs {
        group: String,
        axis: String,
        level: f64,
    },
    Separate {
        point: String,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GraphEdge {
    pub from: String,
    pub to: String,
    /// 省略时为 `out`（点火 / 延时）。主节点为 `flight|sep|function`；条件为 `true|false`；激活为 `next`。
    #[serde(default = "default_port")]
    pub from_port: String,
}

fn default_port() -> String {
    "out".into()
}

/// 某级是否露出能力口（由装配投影得出；校验用）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StagePorts {
    pub flight: bool,
    pub sep: bool,
    pub function: bool,
}

impl GraphIr {
    pub fn to_toml(&self) -> Result<String, SchemaError> {
        toml::to_string_pretty(self).map_err(|e| SchemaError::Validate(e.to_string()))
    }
}

/// 解析图 IR TOML 并做结构校验。`stage_ports` 为 `None` 时不检查级能力是否露出（仅闭集与域）。
///
/// 叠加文件（含 `base` / `entry_link` 键）**不能**经此加载——引擎读不到同目录基础文件，
/// 必须走 [`load_composed`] 同目录拼接合成（校验器与执行器消费的永远是合成图，
/// WORKFLOW.md §8.1）。
pub fn from_toml_str(
    s: &str,
    stage_ports: Option<&HashMap<String, StagePorts>>,
) -> Result<GraphIr, SchemaError> {
    let ir: GraphIr = toml::from_str(s)?;
    validate(&ir, stage_ports)?;
    Ok(ir)
}

pub fn validate(
    ir: &GraphIr,
    stage_ports: Option<&HashMap<String, StagePorts>>,
) -> Result<(), SchemaError> {
    if ir.base.is_some() || ir.entry_link.is_some() {
        return Err(SchemaError::Validate(
            "叠加文件（含 base/entry_link）须经 load_composed 同目录拼接加载，不能直接解析"
                .into(),
        ));
    }
    if ir.schema != 1 {
        return Err(SchemaError::Validate(format!(
            "图 IR schema 须为 1，实际 {}",
            ir.schema
        )));
    }
    if ir.name.trim().is_empty() {
        return Err(SchemaError::Validate("name 不能为空".into()));
    }
    if ir.nodes.is_empty() {
        return Err(SchemaError::Validate("nodes 不能为空".into()));
    }

    let mut by_id: HashMap<&str, &GraphNode> = HashMap::new();
    for n in &ir.nodes {
        if n.id.trim().is_empty() {
            return Err(SchemaError::Validate("节点 id 不能为空".into()));
        }
        if by_id.insert(n.id.as_str(), n).is_some() {
            return Err(SchemaError::Validate(format!("重复节点 id {}", n.id)));
        }
        if let NodeBody::Delay { interval_s, .. } = &n.body {
            if *interval_s < 0.0 {
                return Err(SchemaError::Validate(format!(
                    "延时 {} interval_s 不能为负",
                    n.id
                )));
            }
        }
        if let NodeBody::Condition { terms, .. } = &n.body {
            if terms.is_empty() {
                return Err(SchemaError::Validate(format!(
                    "条件 {} terms 不能为空",
                    n.id
                )));
            }
            for (i, t) in terms.iter().enumerate() {
                t.validate(&format!("条件 {} terms[{i}]", n.id))
                    .map_err(SchemaError::Validate)?;
            }
        }
        if let NodeBody::Drive { spec } = &n.body {
            if let DriveSpec::Action(ActionDrive::Throttle { level, .. }) = spec {
                if !(0.0..=1.0).contains(level) {
                    return Err(SchemaError::Validate(format!(
                        "驱动 {} throttle level 须在 [0,1]",
                        n.id
                    )));
                }
            }
        }
    }

    let ignites: Vec<_> = ir
        .nodes
        .iter()
        .filter(|n| matches!(n.body, NodeBody::Ignite))
        .collect();
    if ignites.len() != 1 {
        return Err(SchemaError::Validate(format!(
            "须恰好一个点火节点，实际 {}",
            ignites.len()
        )));
    }

    let mut outs: HashMap<(&str, &str), Vec<&str>> = HashMap::new();
    for e in &ir.edges {
        let from = by_id
            .get(e.from.as_str())
            .ok_or_else(|| SchemaError::Validate(format!("边 from 未知 {}", e.from)))?;
        let to = by_id
            .get(e.to.as_str())
            .ok_or_else(|| SchemaError::Validate(format!("边 to 未知 {}", e.to)))?;
        let port = if e.from_port.is_empty() {
            "out"
        } else {
            e.from_port.as_str()
        };
        check_port_name(&from.body, port, &from.id)?;
        outs.entry((from.id.as_str(), port))
            .or_default()
            .push(to.id.as_str());
    }

    let ignite_id = ignites[0].id.as_str();
    let ignite_tos = outs.get(&(ignite_id, "out")).map(|v| v.len()).unwrap_or(0);
    if ignite_tos != 1 {
        return Err(SchemaError::Validate(format!(
            "点火须恰好连接一个主节点，实际 {ignite_tos}"
        )));
    }
    let ignite_to = outs[&(ignite_id, "out")][0];
    if !matches!(by_id[ignite_to].body, NodeBody::Main { .. }) {
        return Err(SchemaError::Validate("点火只能连接主节点".into()));
    }

    // 每个节点的祖先能力域（从主节点口流入）。
    let mut domain: HashMap<&str, PortDomain> = HashMap::new();

    let mains: Vec<&GraphNode> = ir
        .nodes
        .iter()
        .filter(|n| matches!(n.body, NodeBody::Main { .. }))
        .collect();

    for main in &mains {
        let stage = match &main.body {
            NodeBody::Main { stage } => stage.as_str(),
            _ => unreachable!(),
        };
        let ports = stage_ports.and_then(|m| m.get(stage));
        if stage_ports.is_some() && ports.is_none() {
            return Err(SchemaError::Validate(format!("未知级 {stage}")));
        }
        for (port_name, dom) in [
            ("flight", PortDomain::Flight),
            ("sep", PortDomain::Sep),
            ("function", PortDomain::Function),
        ] {
            let Some(children) = outs.get(&(main.id.as_str(), port_name)) else {
                continue;
            };
            if let Some(p) = ports {
                let exposed = match dom {
                    PortDomain::Flight => p.flight,
                    PortDomain::Sep => p.sep,
                    PortDomain::Function => p.function,
                };
                if !exposed {
                    return Err(SchemaError::Validate(format!(
                        "级 {stage} 未露出 {port_name} 口"
                    )));
                }
            }
            if dom == PortDomain::Function {
                return Err(SchemaError::Validate("功能口 v1 预留，不可接线".into()));
            }
            for child in children {
                assign_domain(&mut domain, child, dom, &by_id)?;
                walk_domain(child, dom, &by_id, &outs, &mut domain)?;
            }
        }
    }

    // 激活必须有 next；驱动无出边。
    for n in &ir.nodes {
        match &n.body {
            NodeBody::Activate { .. } => {
                let next = outs
                    .get(&(n.id.as_str(), "next"))
                    .map(|v| v.as_slice())
                    .unwrap_or(&[]);
                if next.len() != 1 || !matches!(by_id[next[0]].body, NodeBody::Main { .. }) {
                    return Err(SchemaError::Validate(format!(
                        "激活 {} 须恰好连接一个下一主节点",
                        n.id
                    )));
                }
                let dom = domain.get(n.id.as_str()).copied();
                if dom != Some(PortDomain::Sep) {
                    return Err(SchemaError::Validate(format!(
                        "激活 {} 只能挂在分离口下",
                        n.id
                    )));
                }
            }
            NodeBody::Drive { spec } => {
                if outs.keys().any(|(id, _)| *id == n.id.as_str()) {
                    return Err(SchemaError::Validate(format!("驱动 {} 不能有出边", n.id)));
                }
                let dom = domain.get(n.id.as_str()).copied().ok_or_else(|| {
                    SchemaError::Validate(format!("驱动 {} 未挂到主节点口下", n.id))
                })?;
                check_drive_domain(dom, spec, &n.id)?;
            }
            NodeBody::Ignite | NodeBody::Main { .. } | NodeBody::Halt { .. } => {}
            NodeBody::Condition { .. } | NodeBody::Delay { .. } => {
                if !domain.contains_key(n.id.as_str())
                    && !reachable_from_main(&n.id, &ir.edges, &by_id)
                {
                    // 孤立控制器允许存在但不报错？要求挂上。未挂到主节点则标错。
                    let inbound = ir.edges.iter().any(|e| e.to == n.id);
                    if !inbound {
                        return Err(SchemaError::Validate(format!(
                            "节点 {} 未连接到主节点",
                            n.id
                        )));
                    }
                }
            }
        }
    }

    validate_halt_topology(ir, &by_id, &outs)?;

    Ok(())
}

/// Halt 拓扑规则（WORKFLOW.md §5.5）：
/// - `kind=target`：Halt 出边数必须 = 0（末端）；
/// - `kind=super`：无 halt 不拒（自由程序照常运行）；存在 halt 时——恰好一个
///   出边 = 0（末段终点），其余（前缀）出边恰好 = 1 且指向主节点（增量段入口）；
/// - 入边闭集：Halt 只接受主节点口 / 条件 true·false / 延时 out；激活、驱动拒连；
/// - 参数为纯 bool，无额外校验（组合语义注记见 §5.5）。
fn validate_halt_topology(
    ir: &GraphIr,
    by_id: &HashMap<&str, &GraphNode>,
    outs: &HashMap<(&str, &str), Vec<&str>>,
) -> Result<(), SchemaError> {
    let halt_ids: Vec<&str> = ir
        .nodes
        .iter()
        .filter(|n| matches!(n.body, NodeBody::Halt { .. }))
        .map(|n| n.id.as_str())
        .collect();
    if halt_ids.is_empty() {
        return Ok(());
    }

    // 入边闭集。
    for e in &ir.edges {
        let Some(to) = by_id.get(e.to.as_str()) else {
            continue;
        };
        if !matches!(to.body, NodeBody::Halt { .. }) {
            continue;
        }
        let port = if e.from_port.is_empty() {
            "out"
        } else {
            e.from_port.as_str()
        };
        let from = by_id[e.from.as_str()];
        let ok = match &from.body {
            NodeBody::Main { .. } => true,
            NodeBody::Condition { .. } => matches!(port, "true" | "false"),
            NodeBody::Delay { .. } => port == "out",
            _ => false,
        };
        if !ok {
            return Err(SchemaError::Validate(format!(
                "停机 {} 不能由 {}（{} 口）驱动——只接受主节点口 / 条件 / 延时来源",
                e.to, e.from, port
            )));
        }
    }

    // 出边规则。
    let out_counts: Vec<(&str, usize)> = halt_ids
        .iter()
        .map(|h| (*h, outs.get(&(*h, "out")).map(|v| v.len()).unwrap_or(0)))
        .collect();
    match ir.kind {
        WorkFlowKind::Target => {
            for (h, n) in &out_counts {
                if *n != 0 {
                    return Err(SchemaError::Validate(format!(
                        "Target 图停机 {h} 出边数须为 0（末端），实际 {n}"
                    )));
                }
            }
        }
        WorkFlowKind::Super => {
            let end: Vec<_> = out_counts.iter().filter(|(_, n)| *n == 0).collect();
            let prefix: Vec<_> = out_counts.iter().filter(|(_, n)| *n != 0).collect();
            if end.len() != 1 {
                return Err(SchemaError::Validate(format!(
                    "Super 图须恰好一个末段停机（出边 = 0），实际 {}",
                    end.len()
                )));
            }
            for (h, n) in &prefix {
                if *n != 1 {
                    return Err(SchemaError::Validate(format!(
                        "Super 图前缀停机 {h} 出边须恰好 1（增量段入口），实际 {n}"
                    )));
                }
                for to in &outs[&(*h, "out")] {
                    if !matches!(by_id[to].body, NodeBody::Main { .. }) {
                        return Err(SchemaError::Validate(format!(
                            "Super 图前缀停机 {h} 出边须指向主节点（增量段入口），实际指向 {to}"
                        )));
                    }
                }
            }
        }
    }
    Ok(())
}

/// DLC 叠加合成（WORKFLOW.md §8.1 / §9.1）：基础图 + 叠加段 → 单合成图。
///
/// 校验（合成专属，口域 / 闭集 / halt 拓扑交由合成后整体 [`validate`]）：
/// ① 基础 `kind=target`（**链式一层限定**——指向叠加文件即拒，链式叠加留待后续任务）、
///    叠加 `kind=super`；② `entry_link.halt` ∈ 基础图且为 Halt、`to` ∈ 叠加段且为
///    Main；③ 基础图 Halt 恰恰 1（即 `entry_link.halt` 所指——多余 Halt = 真停机
///    残留会提前终局 + `from` 引用歧义）；④ 节点 id 跨文件冲突拒载；⑤ 叠加段 `edges`
///    不得引用基础图节点（跨文件边仅由 `entry_link` 合成产生，否则绕过闭集）。
/// 产物：`kind=super`（沿用叠加文件），节点与边合并，`entry_link` 展开为
/// `from_port="out"` 真实边界边，`base` / `entry_link` 剥离。
pub fn compose(base: GraphIr, addon: GraphIr, link: &EntryLink) -> Result<GraphIr, SchemaError> {
    if base.kind != WorkFlowKind::Target {
        return Err(SchemaError::Validate(format!(
            "叠加基础文件 kind 须为 target（链式叠加不支持，基础文件不得为叠加文件），实际 {:?}",
            base.kind
        )));
    }
    if addon.kind != WorkFlowKind::Super {
        return Err(SchemaError::Validate(format!(
            "叠加文件 kind 须为 super，实际 {:?}",
            addon.kind
        )));
    }

    let base_halt_count = base
        .nodes
        .iter()
        .filter(|n| matches!(n.body, NodeBody::Halt { .. }))
        .count();
    if base_halt_count != 1 {
        return Err(SchemaError::Validate(format!(
            "叠加基础图须恰好 1 个停机节点（entry_link.from 所指），实际 {base_halt_count}"
        )));
    }

    let halt_node = base
        .nodes
        .iter()
        .find(|n| n.id == link.halt)
        .ok_or_else(|| {
            SchemaError::Validate(format!("entry_link.halt `{}` 不在基础图中", link.halt))
        })?;
    if !matches!(halt_node.body, NodeBody::Halt { .. }) {
        return Err(SchemaError::Validate(format!(
            "entry_link.halt `{}` 须为停机节点",
            link.halt
        )));
    }
    let entry_node = addon
        .nodes
        .iter()
        .find(|n| n.id == link.to)
        .ok_or_else(|| {
            SchemaError::Validate(format!("entry_link.to `{}` 不在叠加文件中", link.to))
        })?;
    if !matches!(entry_node.body, NodeBody::Main { .. }) {
        return Err(SchemaError::Validate(format!(
            "entry_link.to `{}` 须为主节点（增量段入口）",
            link.to
        )));
    }

    let base_ids: HashSet<&str> = base.nodes.iter().map(|n| n.id.as_str()).collect();
    for n in &addon.nodes {
        if base_ids.contains(n.id.as_str()) {
            return Err(SchemaError::Validate(format!(
                "叠加节点 id `{}` 与基础图冲突",
                n.id
            )));
        }
    }
    for e in &addon.edges {
        if base_ids.contains(e.from.as_str()) || base_ids.contains(e.to.as_str()) {
            return Err(SchemaError::Validate(format!(
                "叠加段边 {}→{} 引用了基础图节点——跨文件边仅由 entry_link 合成产生",
                e.from, e.to
            )));
        }
    }

    let mut nodes = base.nodes;
    nodes.extend(addon.nodes);
    let mut edges = base.edges;
    edges.push(GraphEdge {
        from: link.halt.clone(),
        to: link.to.clone(),
        from_port: "out".into(),
    });
    edges.extend(addon.edges);
    Ok(GraphIr {
        schema: 1,
        kind: WorkFlowKind::Super,
        name: addon.name,
        base: None,
        entry_link: None,
        nodes,
        edges,
    })
}

/// 加载图工作流（单文件整图 / DLC 叠加文件统一入口，WORKFLOW.md §8.1）：
/// 按文件内 `base` 键分形态——无 = 单文件整图直载（`from_toml_str` 语义）；
/// 有 = 同目录读基础文件（sibling，防路径逃逸），基础图先单独按 target 口径校验，
/// 再 [`compose`] 合成并整体校验。错误信息携带具体文件名。
pub fn load_composed(
    path: &Path,
    stage_ports: Option<&HashMap<String, StagePorts>>,
) -> Result<GraphIr, SchemaError> {
    let text = std::fs::read_to_string(path).map_err(|e| {
        SchemaError::Validate(format!("读取图工作流 `{}` 失败：{e}", path.display()))
    })?;
    let ir: GraphIr = toml::from_str(&text).map_err(|e| {
        SchemaError::Validate(format!("解析图工作流 `{}` 失败：{e}", path.display()))
    })?;

    let Some(base_name) = ir.base.clone() else {
        validate(&ir, stage_ports)?;
        return Ok(ir);
    };

    let Some(link) = ir.entry_link.clone() else {
        return Err(SchemaError::Validate(format!(
            "叠加文件 `{}` 有 base 引用但缺 entry_link（引用头须成对出现）",
            path.display()
        )));
    };
    if base_name.trim().is_empty() {
        return Err(SchemaError::Validate(format!(
            "叠加文件 `{}` base 引用不能为空",
            path.display()
        )));
    }

    let base_path = path
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(&base_name);
    let base_text = std::fs::read_to_string(&base_path).map_err(|e| {
        SchemaError::Validate(format!(
            "叠加文件 `{}` 引用的基础文件 `{}` 读取失败：{e}",
            path.display(),
            base_path.display()
        ))
    })?;
    let base_ir: GraphIr = toml::from_str(&base_text).map_err(|e| {
        SchemaError::Validate(format!(
            "解析基础文件 `{}` 失败：{e}",
            base_path.display()
        ))
    })?;
    validate(&base_ir, stage_ports).map_err(|e| {
        SchemaError::Validate(format!("基础文件 `{}` 校验失败：{e}", base_path.display()))
    })?;

    let composed = compose(base_ir, ir, &link)?;
    validate(&composed, stage_ports)?;
    Ok(composed)
}

fn reachable_from_main(id: &str, edges: &[GraphEdge], by_id: &HashMap<&str, &GraphNode>) -> bool {
    let mut stack = vec![id.to_string()];
    let mut seen = HashSet::new();
    while let Some(cur) = stack.pop() {
        if !seen.insert(cur.clone()) {
            continue;
        }
        for e in edges.iter().filter(|e| e.to == cur) {
            if let Some(n) = by_id.get(e.from.as_str()) {
                if matches!(n.body, NodeBody::Main { .. }) {
                    return true;
                }
                stack.push(n.id.clone());
            }
        }
    }
    false
}

fn check_port_name(body: &NodeBody, port: &str, id: &str) -> Result<(), SchemaError> {
    let ok = match body {
        NodeBody::Ignite | NodeBody::Delay { .. } | NodeBody::Halt { .. } => port == "out",
        NodeBody::Main { .. } => matches!(port, "flight" | "sep" | "function"),
        NodeBody::Condition { .. } => matches!(port, "true" | "false"),
        NodeBody::Activate { .. } => port == "next",
        NodeBody::Drive { .. } => false,
    };
    if ok {
        Ok(())
    } else {
        Err(SchemaError::Validate(format!(
            "节点 {id} 不允许端口 {port}"
        )))
    }
}

fn assign_domain<'a>(
    domain: &mut HashMap<&'a str, PortDomain>,
    id: &'a str,
    dom: PortDomain,
    by_id: &HashMap<&str, &GraphNode>,
) -> Result<(), SchemaError> {
    if matches!(by_id[id].body, NodeBody::Main { .. }) {
        return Err(SchemaError::Validate(
            "条件/延时/驱动不能把主节点当子节点（交棒仅激活 next）".into(),
        ));
    }
    if let Some(prev) = domain.insert(id, dom) {
        if prev != dom {
            return Err(SchemaError::Validate(format!(
                "节点 {id} 不能同时属于两个能力口"
            )));
        }
    }
    Ok(())
}

/// 后序遍历能力口子树：登记域 + 校验驱动口域。（引导条件并发**不限**——
/// 条件每拍只走一支，嵌套分段引导运行时无互抢；同拍并挂多引导由执行器
/// WARN 提示「仅最后一个生效」，2026-11-15 撤销 v13「≤1」静态校验。）
fn walk_domain<'a>(
    id: &'a str,
    dom: PortDomain,
    by_id: &HashMap<&'a str, &'a GraphNode>,
    outs: &HashMap<(&str, &str), Vec<&'a str>>,
    domain: &mut HashMap<&'a str, PortDomain>,
) -> Result<(), SchemaError> {
    let node = by_id[id];
    match &node.body {
        NodeBody::Condition { .. } => {
            for port in ["true", "false"] {
                if let Some(children) = outs.get(&(id, port)) {
                    for c in children {
                        assign_domain(domain, c, dom, by_id)?;
                        walk_domain(c, dom, by_id, outs, domain)?;
                    }
                }
            }
        }
        NodeBody::Delay { .. } => {
            if let Some(children) = outs.get(&(id, "out")) {
                for c in children {
                    assign_domain(domain, c, dom, by_id)?;
                    walk_domain(c, dom, by_id, outs, domain)?;
                }
            }
        }
        NodeBody::Drive { spec } => {
            check_drive_domain(dom, spec, id)?;
        }
        NodeBody::Activate { .. } => {
            if dom != PortDomain::Sep {
                return Err(SchemaError::Validate(format!("激活 {id} 只能挂在分离口下")));
            }
        }
        // 停机是能力口树上的合法叶子（无出边出域；super 前缀出边另在
        // validate_halt_topology 校验）。
        NodeBody::Halt { .. } => {}
        NodeBody::Ignite | NodeBody::Main { .. } => {
            return Err(SchemaError::Validate(format!(
                "节点 {id} 不能作为能力口子节点"
            )));
        }
    }
    Ok(())
}

/// 口域统一（v13）：驱动合法性只由**端口域**决定，与工作流 kind 解耦——
/// 飞控口下 TargetMode 引导与 Base 直驱（throttle/tvc/rcs）均合法（图执行器
/// `exec_drive` 本就双语义统一）；分离器仍限 sep 口；功能口 v1 不接驱动。
/// 同拍混用两类驱动的写序注记见 WORKFLOW.md §图 IR。
fn check_drive_domain(dom: PortDomain, spec: &DriveSpec, id: &str) -> Result<(), SchemaError> {
    let bad = |msg: String| Err(SchemaError::Validate(msg));
    match dom {
        PortDomain::Function => bad(format!("功能口 v1 不接驱动 {id}")),
        PortDomain::Sep => match spec {
            DriveSpec::Action(ActionDrive::Separate { .. }) => Ok(()),
            _ => bad(format!("分离口只能挂分离器驱动 {id}")),
        },
        PortDomain::Flight => match spec {
            DriveSpec::Action(ActionDrive::Separate { .. }) => {
                bad(format!("飞控口不能挂分离器驱动 {id}"))
            }
            _ => Ok(()),
        },
    }
}

/// 由装配投影各级能力口。flight 口两 kind 恒露出（v13 口域统一：seed 前缀
/// = wf_target 快照原样复制，mode 驱动可能挂在无执行器级——投影不得因 kind
/// 收窄，否则 seeded wf_super 发射时误报「未露出 flight 口」）。
pub fn stage_ports_from_assembly(asm: &orbitx_vessel::Assembly) -> HashMap<String, StagePorts> {
    use crate::capability::ControlCapability;
    let caps = ControlCapability::for_primary(asm);
    let mut map = HashMap::new();
    for v in &asm.vessels {
        let sep = caps
            .separation_points
            .iter()
            .any(|p| p.id == "stage-sep" || p.id.starts_with(&format!("{}-sep", v.name)));
        map.insert(
            v.name.clone(),
            StagePorts {
                flight: true,
                sep,
                function: false,
            },
        );
    }
    map
}

#[cfg(test)]
mod tests;
