//! 图 IR（[`WORKFLOW.md`](../../../../docs/WORKFLOW.md) §9）：解析、校验、ACTIVE 调度。
//!
//! 与线性 `[[phases]]` / `[[steps]]` 分文件、分模块，不混解析路径。

mod runtime;

pub use runtime::GraphWorkFlow;

use std::collections::{HashMap, HashSet};

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
    #[serde(default)]
    pub nodes: Vec<GraphNode>,
    #[serde(default)]
    pub edges: Vec<GraphEdge>,
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
    let mut flight_conditions: HashMap<&str, usize> = HashMap::new();

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
                walk_domain(
                    child,
                    dom,
                    main.id.as_str(),
                    &by_id,
                    &outs,
                    &mut domain,
                    ir.kind,
                    &mut flight_conditions,
                )?;
            }
        }
    }

    if ir.kind == WorkFlowKind::Target {
        for (main_id, n) in &flight_conditions {
            if *n > 1 {
                return Err(SchemaError::Validate(format!(
                    "Target 飞控口条件控制须 ≤1（主节点 {main_id} 有 {n}）"
                )));
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
                check_drive_domain(ir.kind, dom, spec, &n.id)?;
            }
            NodeBody::Ignite | NodeBody::Main { .. } => {}
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

    Ok(())
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
        NodeBody::Ignite | NodeBody::Delay { .. } => port == "out",
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

fn walk_domain<'a>(
    id: &'a str,
    dom: PortDomain,
    main_id: &'a str,
    by_id: &HashMap<&'a str, &'a GraphNode>,
    outs: &HashMap<(&str, &str), Vec<&'a str>>,
    domain: &mut HashMap<&'a str, PortDomain>,
    kind: WorkFlowKind,
    flight_conditions: &mut HashMap<&'a str, usize>,
) -> Result<(), SchemaError> {
    let node = by_id[id];
    match &node.body {
        NodeBody::Condition { .. } => {
            if dom == PortDomain::Flight {
                *flight_conditions.entry(main_id).or_insert(0) += 1;
            }
            for port in ["true", "false"] {
                if let Some(children) = outs.get(&(id, port)) {
                    for c in children {
                        assign_domain(domain, c, dom, by_id)?;
                        walk_domain(
                            c,
                            dom,
                            main_id,
                            by_id,
                            outs,
                            domain,
                            kind,
                            flight_conditions,
                        )?;
                    }
                }
            }
        }
        NodeBody::Delay { .. } => {
            if let Some(children) = outs.get(&(id, "out")) {
                for c in children {
                    assign_domain(domain, c, dom, by_id)?;
                    walk_domain(
                        c,
                        dom,
                        main_id,
                        by_id,
                        outs,
                        domain,
                        kind,
                        flight_conditions,
                    )?;
                }
            }
        }
        NodeBody::Drive { spec } => {
            check_drive_domain(kind, dom, spec, id)?;
        }
        NodeBody::Activate { .. } => {
            if dom != PortDomain::Sep {
                return Err(SchemaError::Validate(format!("激活 {id} 只能挂在分离口下")));
            }
        }
        NodeBody::Ignite | NodeBody::Main { .. } => {
            return Err(SchemaError::Validate(format!(
                "节点 {id} 不能作为能力口子节点"
            )));
        }
    }
    Ok(())
}

fn check_drive_domain(
    kind: WorkFlowKind,
    dom: PortDomain,
    spec: &DriveSpec,
    id: &str,
) -> Result<(), SchemaError> {
    let bad = |msg: String| Err(SchemaError::Validate(msg));
    match dom {
        PortDomain::Function => bad(format!("功能口 v1 不接驱动 {id}")),
        PortDomain::Sep => match spec {
            DriveSpec::Action(ActionDrive::Separate { .. }) => Ok(()),
            _ => bad(format!("分离口只能挂分离器驱动 {id}")),
        },
        PortDomain::Flight => match (kind, spec) {
            (WorkFlowKind::Target, DriveSpec::Target(_)) => Ok(()),
            (WorkFlowKind::Target, DriveSpec::Action(_)) => {
                bad(format!("Target 飞控口不能挂 Base 动作驱动 {id}"))
            }
            (WorkFlowKind::Super, DriveSpec::Action(ActionDrive::Separate { .. })) => {
                bad(format!("飞控口不能挂分离器驱动 {id}"))
            }
            (WorkFlowKind::Super, DriveSpec::Action(_)) => Ok(()),
            (WorkFlowKind::Super, DriveSpec::Target(_)) => {
                bad(format!("Super 飞控口不能挂 TargetMode {id}"))
            }
        },
    }
}

/// 由装配投影各级能力口（Target 图飞控口始终露出）。
pub fn stage_ports_from_assembly(
    asm: &orbitx_vessel::Assembly,
    kind: WorkFlowKind,
) -> HashMap<String, StagePorts> {
    use crate::capability::ControlCapability;
    let caps = ControlCapability::for_primary(asm);
    let mut map = HashMap::new();
    for v in &asm.vessels {
        let flight = kind == WorkFlowKind::Target
            || caps.throttle_groups.iter().any(|g| g.id == v.name)
            || caps
                .tvc_groups
                .iter()
                .any(|g| g.id.starts_with(&format!("{}-tvc", v.name)))
            || caps
                .rcs_groups
                .iter()
                .any(|g| g.id.starts_with(&format!("{}-", v.name)));
        let sep = caps
            .separation_points
            .iter()
            .any(|p| p.id == "stage-sep" || p.id.starts_with(&format!("{}-sep", v.name)));
        map.insert(
            v.name.clone(),
            StagePorts {
                flight,
                sep,
                function: false,
            },
        );
    }
    map
}

#[cfg(test)]
mod tests;
