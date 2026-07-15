//! 场景管理：创建天体的 kiss3d 节点，每帧更新位置和轨道线。

use kiss3d::light::Light;
use kiss3d::prelude::*;

use crate::bridge::CameraFrame;
use crate::sim::{BodyState, Simulation};

/// 渲染场景：管理 kiss3d SceneNode3d 和天体节点。
pub struct SpaceScene {
    pub scene: SceneNode3d,
    /// 天体节点列表（与 Simulation 的 body_states 对应）。
    body_nodes: Vec<SceneNode3d>,
    /// 轨道线（预计算的多段线）。
    orbit_polylines: Vec<Polyline3d>,
}

impl SpaceScene {
    /// 创建场景，设置光照。
    pub fn new(sim: &Simulation, frame: &CameraFrame) -> Self {
        let mut scene = SceneNode3d::empty();

        // 方向光源（模拟太阳光，无衰减，均匀照亮所有行星）。
        scene.add_light(
            Light::directional(Vec3::new(-0.3, -0.5, -0.8)).with_intensity(3.0),
        );

        // 为每个天体创建一个球体节点。
        let states = sim.body_states();
        let mut body_nodes = Vec::with_capacity(states.len());

        for state in &states {
            let color = Color::new(
                state.color[0],
                state.color[1],
                state.color[2],
                state.color[3],
            );
            // 渲染半径：使用最小显示半径（行星实际太小看不见）。
            let r = state.min_render_radius;
            let mut node = scene.add_sphere(r).set_color(color);
            // 太阳发光体不接收阴影。
            if state.name == "Sun" {
                node.set_surface_rendering_activation(true);
            }
            body_nodes.push(node);
        }

        // 为行星（不含太阳）生成轨道线。每条用对应行星的高亮颜色。
        let orbit_colors = [
            Color::new(1.0, 0.9, 0.5, 1.0), // Mercury - 亮黄
            Color::new(1.0, 0.8, 0.3, 1.0), // Venus - 橙黄
            Color::new(0.3, 0.8, 1.0, 1.0), // Earth - 亮青
            Color::new(1.0, 0.5, 0.2, 1.0), // Mars - 亮橙
            Color::new(1.0, 0.85, 0.4, 1.0), // Jupiter - 金黄
            Color::new(0.9, 0.9, 0.5, 1.0), // Saturn - 浅金
            Color::new(0.4, 1.0, 1.0, 1.0), // Uranus - 亮青绿
            Color::new(0.4, 0.6, 1.0, 1.0), // Neptune - 亮蓝
        ];

        let mut orbit_polylines = Vec::new();
        for i in 1..sim.num_bodies() {
            let points = sim.sample_orbit(i, 128);
            if points.len() < 2 {
                continue;
            }
            let vertices: Vec<Vec3> = points.iter().map(|p| frame.to_render(*p)).collect();
            let color = orbit_colors.get(i - 1).copied().unwrap_or(Color::new(1.0, 1.0, 1.0, 1.0));
            let mut poly = Polyline3d::new(vertices)
                .with_color(color)
                .with_width(1.0);
            poly.perspective = false; // 恒定线宽，不随距离变细
            orbit_polylines.push(poly);
        }

        SpaceScene {
            scene,
            body_nodes,
            orbit_polylines,
        }
    }

    /// 每帧更新：设置天体位置 + 绘制轨道线。
    pub fn update(&mut self, window: &mut Window, states: &[BodyState], frame: &CameraFrame) {
        // 更新天体位置。
        for (i, state) in states.iter().enumerate() {
            if i < self.body_nodes.len() {
                let pos = frame.to_render(state.pos);
                self.body_nodes[i].set_position(pos);
            }
        }

        // 绘制轨道线（每帧重绘，因为浮点原点可能变化）。
        for poly in &self.orbit_polylines {
            window.draw_polyline(poly);
        }
    }
}
