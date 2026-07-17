//! 场景节点 — 可见对象树，移植自 Orbiter `VObject.cpp`。
//!
//! 每个节点持有 f64 仿真坐标（Transform64），每帧由 CoordinateBridge
//! 转换为 f32 渲染数据（RenderData），供 GPU 使用。

use orbitx_math::vec3::Vec3;
use orbitx_math::mat3::Matrix3;

/// 场景节点标识符。
pub type NodeId = u64;

/// f64 变换（仿真坐标系，左手黄道 J2000）。
#[derive(Clone, Debug)]
pub struct Transform64 {
    /// 位置（米）。
    pub position: Vec3,
    /// 旋转矩阵。
    pub rotation: Matrix3,
    /// 缩放因子。
    pub scale: f64,
}

impl Default for Transform64 {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            rotation: Matrix3::IDENTITY,
            scale: 1.0,
        }
    }
}

/// GPU 端渲染数据（f32，相机相对坐标系）。
///
/// 每帧由 `CoordinateBridge::to_render()` 从 `Transform64` 更新。
#[derive(Clone, Debug)]
pub struct RenderData {
    /// 渲染空间位置（f32，相机相对）。
    pub position: glam::Vec3,
    /// 渲染空间旋转（f32，3×3）。
    pub rotation: glam::Mat3,
    /// 渲染空间缩放（f32）。
    pub scale: f32,
    /// 到相机距离（米，用于 LOD 选择和排序）。
    pub dist_to_cam: f64,
    /// 屏幕投影大小（像素，用于远处 fallback）。
    pub screen_size: f32,
}

impl Default for RenderData {
    fn default() -> Self {
        Self {
            position: glam::Vec3::ZERO,
            rotation: glam::Mat3::IDENTITY,
            scale: 1.0,
            dist_to_cam: 0.0,
            screen_size: 0.0,
        }
    }
}

/// 行星渲染状态。
#[derive(Clone, Debug)]
pub struct PlanetRenderState {
    /// 物理半径（米）。
    pub radius: f64,
    /// 最小渲染半径（渲染单位，保证远处可见）。
    pub min_render_radius: f32,
    /// 颜色 RGBA。
    pub color: [f32; 4],
    /// 是否有大气层。
    pub has_atmosphere: bool,
    /// 是否有环系。
    pub has_rings: bool,
    /// 表面纹理键（天体名，对应内置等距柱状贴图）；None 则用纯色。
    pub texture: Option<String>,
    /// 大气辉光颜色 RGB（有大气时）；None 则不渲染大气壳。
    pub atmosphere_color: Option<[f32; 3]>,
    /// 是否渲染云层壳（对应内置 `<name>_clouds` 贴图，目前仅地球）。
    pub clouds: bool,
}

/// 航天器渲染状态。
#[derive(Clone, Debug)]
pub struct VesselRenderState {
    /// 网格名称（用于查找已加载网格）。
    pub mesh_name: String,
    /// 颜色 RGBA。
    pub color: [f32; 4],
    /// 当前油门（0..1）— 驱动尾焰渲染大小/亮度；0 时不绘制尾焰。
    pub throttle: f32,
}

/// 场景节点类型。
#[derive(Clone, Debug)]
pub enum NodeType {
    /// 恒星（点精灵/billboard）。
    Star,
    /// 行星（含 LOD 瓦片管理器）。
    Planet(PlanetRenderState),
    /// 航天器（含网格）。
    Vessel(VesselRenderState),
    /// 表面基地。
    Base,
    /// 轨道线。
    OrbitLine,
    /// 粒子系统。
    ParticleSystem,
}

/// 场景节点。
pub struct SceneNode {
    /// 节点标识符。
    pub id: NodeId,
    /// 节点类型。
    pub node_type: NodeType,
    /// f64 仿真坐标变换。
    pub transform: Transform64,
    /// 是否可见。
    pub visible: bool,
    /// 子节点 ID 列表。
    pub children: Vec<NodeId>,
    /// 父节点 ID。
    pub parent: Option<NodeId>,
    /// GPU 端渲染数据（每帧更新）。
    pub render_data: RenderData,
}

impl SceneNode {
    /// 创建新节点。
    pub fn new(id: NodeId, node_type: NodeType) -> Self {
        Self {
            id,
            node_type,
            transform: Transform64::default(),
            visible: true,
            children: Vec::new(),
            parent: None,
            render_data: RenderData::default(),
        }
    }

    /// 创建行星节点。
    pub fn new_planet(id: NodeId, radius: f64, min_render_radius: f32, color: [f32; 4]) -> Self {
        let mut node = Self::new(id, NodeType::Planet(PlanetRenderState {
            radius,
            min_render_radius,
            color,
            has_atmosphere: false,
            has_rings: false,
            texture: None,
            atmosphere_color: None,
            clouds: false,
        }));
        node.transform.scale = radius;
        node
    }

    /// 创建恒星节点。
    pub fn new_star(id: NodeId, radius: f64, min_render_radius: f32, color: [f32; 4]) -> Self {
        let mut node = Self::new(id, NodeType::Star);
        node.transform.scale = radius;
        node.render_data.screen_size = min_render_radius;
        node
    }

    /// 创建航天器节点。
    ///
    /// `scale` 为特征长度（米），驱动 SceneManager 的屏幕投影计算；
    /// 远距时以 billboard 渲染，近距时以简单几何渲染。
    pub fn new_vessel(id: NodeId, scale: f64, mesh_name: impl Into<String>, color: [f32; 4]) -> Self {
        let mut node = Self::new(id, NodeType::Vessel(VesselRenderState {
            mesh_name: mesh_name.into(),
            color,
            throttle: 0.0,
        }));
        node.transform.scale = scale;
        node
    }

    /// 更新渲染数据（每帧调用）。
    ///
    /// `coord` — 坐标桥，用于 f64→f32 转换。
    /// `cam_pos` — 相机仿真坐标（用于计算距离）。
    pub fn update_render_data(
        &mut self,
        coord: &crate::coord::CoordinateBridge,
        cam_pos: &Vec3,
    ) {
        self.render_data.position = coord.to_render(&self.transform.position);
        self.render_data.rotation = coord.to_render_mat3(&self.transform.rotation);
        self.render_data.scale = coord.to_render_radius(self.transform.scale);

        // 计算到相机距离
        let diff = self.transform.position - *cam_pos;
        self.render_data.dist_to_cam = diff.length();

        // 计算屏幕投影大小（简化：基于半径/距离比）
        if self.render_data.dist_to_cam > 0.0 {
            // 角直径（弧度）≈ 2 * radius / distance
            let angular_size = 2.0 * self.transform.scale / self.render_data.dist_to_cam;
            // 假设 1000 像素高度对应 60° 视场
            self.render_data.screen_size = (angular_size * 1000.0 / std::f64::consts::FRAC_PI_3) as f32;
        }
    }

    /// 判断是否需要远处 fallback 渲染。
    ///
    /// 当屏幕投影太小（< min_render_radius 像素）时，
    /// 使用 RenderAsPixel/Disc/Spot 代替完整网格。
    pub fn needs_distant_fallback(&self) -> bool {
        match &self.node_type {
            NodeType::Planet(state) => {
                self.render_data.screen_size < state.min_render_radius
            }
            NodeType::Vessel(_) => {
                self.render_data.screen_size < 2.0  // 2 像素阈值
            }
            _ => false,
        }
    }
}

/// 场景管理器 — 管理所有场景节点。
pub struct SceneManager {
    nodes: Vec<SceneNode>,
    next_id: NodeId,
}

impl SceneManager {
    /// 创建空场景。
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            next_id: 0,
        }
    }

    /// 添加节点，返回其 ID。
    pub fn add_node(&mut self, mut node: SceneNode) -> NodeId {
        let id = self.next_id;
        node.id = id;
        self.next_id += 1;
        self.nodes.push(node);
        id
    }

    /// 获取所有节点。
    pub fn nodes(&self) -> &[SceneNode] {
        &self.nodes
    }

    /// 获取所有节点（可变）。
    pub fn nodes_mut(&mut self) -> &mut Vec<SceneNode> {
        &mut self.nodes
    }

    /// 按类型查找节点。
    pub fn find_by_type(&self, target_type: &NodeType) -> Vec<&SceneNode> {
        // 简化匹配：按类型名称
        self.nodes.iter().filter(|n| {
            std::mem::discriminant(&n.node_type) == std::mem::discriminant(target_type)
        }).collect()
    }

    /// 更新所有节点的渲染数据。
    pub fn update_all(
        &mut self,
        coord: &crate::coord::CoordinateBridge,
        cam_pos: &Vec3,
    ) {
        for node in &mut self.nodes {
            if node.visible {
                node.update_render_data(coord, cam_pos);
            }
        }
    }

    /// 节点数量。
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }
}

impl Default for SceneManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform64_default() {
        let t = Transform64::default();
        assert_eq!(t.position, Vec3::ZERO);
        assert_eq!(t.rotation, Matrix3::IDENTITY);
        assert_eq!(t.scale, 1.0);
    }

    #[test]
    fn scene_manager_add_nodes() {
        let mut mgr = SceneManager::new();
        let id0 = mgr.add_node(SceneNode::new(0, NodeType::Star));
        let id1 = mgr.add_node(SceneNode::new(1, NodeType::Planet(PlanetRenderState {
            radius: 6.371e6, min_render_radius: 5.0, color: [0.0, 0.0, 1.0, 1.0],
            has_atmosphere: true, has_rings: false, texture: None, atmosphere_color: None, clouds: false,
        })));
        assert_eq!(id0, 0);
        assert_eq!(id1, 1);
        assert_eq!(mgr.len(), 2);
    }

    #[test]
    fn planet_needs_distant_fallback_when_small() {
        let mut node = SceneNode::new_planet(0, 6.371e6, 5.0, [0.0, 0.0, 1.0, 1.0]);
        // When screen_size < min_render_radius, needs fallback
        node.render_data.screen_size = 2.0;
        assert!(node.needs_distant_fallback());
        node.render_data.screen_size = 10.0;
        assert!(!node.needs_distant_fallback());
    }

    #[test]
    fn update_render_data_basic() {
        use crate::coord::CoordinateBridge;
        let mut node = SceneNode::new_planet(0, 6.371e6, 5.0, [0.0, 0.0, 1.0, 1.0]);
        node.transform.position = Vec3::new(1.0e11, 0.0, 0.0);
        let coord = CoordinateBridge::new_solar_system(20.0);
        let cam_pos = Vec3::ZERO;
        node.update_render_data(&coord, &cam_pos);
        // Position should be non-zero (1 AU away)
        assert!(node.render_data.position.x > 0.0);
        // Distance should be ~1 AU
        assert!(node.render_data.dist_to_cam > 1.0e10);
    }
}
