//! 渲染图 — 多 pass 调度，移植自 Orbiter D3D9Client `Scene::RenderMainScene`。
//!
//! 渲染 Pass 按依赖拓扑排序执行，每个 Pass 产出中间纹理供后续 Pass 消费。

use std::collections::HashMap;

/// Pass 标识符。
pub type PassId = u32;

/// 标准渲染 Pass 枚举（对应 Orbiter D3D9Client Scene::RenderMainScene 顺序）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum StandardPass {
    /// 1. 天球背景（恒星、星座线、网格）
    CelestialSphere = 0,
    /// 2. 行星表面（四叉树瓦片 + LOD）
    PlanetSurface = 1,
    /// 3. 云层（独立旋转的云瓦片）
    CloudLayer = 2,
    /// 4. 大气散射（Rayleigh/Mie 后处理）
    Atmosphere = 3,
    /// 5. 行星环系（Saturn 等）
    RingSystem = 4,
    /// 6. 航天器网格
    VesselMesh = 5,
    /// 7. 粒子效果（尾焰、再入火焰）
    Particles = 6,
    /// 8. 轨道线/轨迹
    OrbitLines = 7,
    /// 9. 标签/标记
    Labels = 8,
    /// 10. HUD/MFD 叠加（egui render pass）
    HudOverlay = 9,
}

impl StandardPass {
    /// 所有标准 Pass 按渲染顺序排列。
    pub fn all() -> &'static [StandardPass] {
        &[
            StandardPass::CelestialSphere,
            StandardPass::PlanetSurface,
            StandardPass::CloudLayer,
            StandardPass::Atmosphere,
            StandardPass::RingSystem,
            StandardPass::VesselMesh,
            StandardPass::Particles,
            StandardPass::OrbitLines,
            StandardPass::Labels,
            StandardPass::HudOverlay,
        ]
    }

    /// Pass 名称。
    pub fn name(&self) -> &'static str {
        match self {
            StandardPass::CelestialSphere => "celestial_sphere",
            StandardPass::PlanetSurface => "planet_surface",
            StandardPass::CloudLayer => "cloud_layer",
            StandardPass::Atmosphere => "atmosphere",
            StandardPass::RingSystem => "ring_system",
            StandardPass::VesselMesh => "vessel_mesh",
            StandardPass::Particles => "particles",
            StandardPass::OrbitLines => "orbit_lines",
            StandardPass::Labels => "labels",
            StandardPass::HudOverlay => "hud_overlay",
        }
    }

    /// 转为 PassId。
    pub fn id(&self) -> PassId {
        *self as PassId
    }
}

/// 渲染 Pass 描述。
pub struct RenderPass {
    pub id: PassId,
    pub name: String,
    /// 此 Pass 需要的前置 Pass（依赖）。
    pub dependencies: Vec<PassId>,
    /// 此 Pass 是否启用。
    pub enabled: bool,
}

/// 渲染图 — 管理 Pass 集合和执行顺序。
pub struct RenderGraph {
    passes: HashMap<PassId, RenderPass>,
    /// 拓扑排序后的执行顺序。
    execution_order: Vec<PassId>,
    /// 是否需要重新排序。
    dirty: bool,
}

impl RenderGraph {
    /// 创建包含所有标准 Pass 的渲染图。
    pub fn new() -> Self {
        let mut graph = Self {
            passes: HashMap::new(),
            execution_order: Vec::new(),
            dirty: false,
        };

        // 添加标准 Pass（按顺序，大气散射依赖行星表面+云层）
        for pass in StandardPass::all() {
            let deps = match pass {
                StandardPass::Atmosphere => vec![
                    StandardPass::PlanetSurface.id(),
                    StandardPass::CloudLayer.id(),
                ],
                StandardPass::HudOverlay => vec![
                    StandardPass::PlanetSurface.id(),
                    StandardPass::VesselMesh.id(),
                ],
                _ => Vec::new(),
            };
            graph.add_pass(RenderPass {
                id: pass.id(),
                name: pass.name().to_string(),
                dependencies: deps,
                enabled: true,
            });
        }

        graph.rebuild_order();
        graph
    }

    /// 添加一个 Pass。
    pub fn add_pass(&mut self, pass: RenderPass) {
        self.passes.insert(pass.id, pass);
        self.dirty = true;
    }

    /// 启用/禁用 Pass。
    pub fn set_enabled(&mut self, id: PassId, enabled: bool) {
        if let Some(pass) = self.passes.get_mut(&id) {
            pass.enabled = enabled;
        }
    }

    /// 获取执行顺序（已排序的 PassId 列表）。
    pub fn execution_order(&self) -> &[PassId] {
        &self.execution_order
    }

    /// 获取 Pass 信息。
    pub fn get_pass(&self, id: PassId) -> Option<&RenderPass> {
        self.passes.get(&id)
    }

    /// 重新计算拓扑排序。
    fn rebuild_order(&mut self) {
        // 简化：标准 Pass 已按渲染顺序定义，直接按 id 排序。
        // 对于自定义 Pass，需要真正的拓扑排序。
        let mut order: Vec<PassId> = self.passes.keys().copied().collect();
        order.sort();
        self.execution_order = order;
        self.dirty = false;
    }

    /// 获取启用的 Pass 数量。
    pub fn enabled_count(&self) -> usize {
        self.passes.values().filter(|p| p.enabled).count()
    }
}

impl Default for RenderGraph {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_pass_count() {
        assert_eq!(StandardPass::all().len(), 10);
    }

    #[test]
    fn render_graph_default_has_all_passes() {
        let graph = RenderGraph::new();
        assert_eq!(graph.enabled_count(), 10);
    }

    #[test]
    fn execution_order_matches_standard() {
        let graph = RenderGraph::new();
        let order = graph.execution_order();
        // Should be 0, 1, 2, ..., 9
        assert_eq!(order.len(), 10);
        for (i, &id) in order.iter().enumerate() {
            assert_eq!(id, i as PassId);
        }
    }

    #[test]
    fn disable_pass() {
        let mut graph = RenderGraph::new();
        graph.set_enabled(StandardPass::Atmosphere.id(), false);
        assert_eq!(graph.enabled_count(), 9);
    }

    #[test]
    fn atmosphere_depends_on_planet_and_cloud() {
        let graph = RenderGraph::new();
        let atmo = graph.get_pass(StandardPass::Atmosphere.id()).unwrap();
        assert!(atmo.dependencies.contains(&StandardPass::PlanetSurface.id()));
        assert!(atmo.dependencies.contains(&StandardPass::CloudLayer.id()));
    }
}
