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

        // 太阳作为光源（位于原点）。
        scene.add_light(Light::point(1e9)).set_position(Vec3::ZERO);

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
            let node = scene.add_sphere(r).set_color(color);
            body_nodes.push(node);
        }

        // 为行星（不含太阳）生成轨道线。
        let mut orbit_polylines = Vec::new();
        for i in 1..sim.num_bodies() {
            let points = sim.sample_orbit(i, 128);
            if points.len() < 2 {
                continue;
            }
            let vertices: Vec<Vec3> = points.iter().map(|p| frame.to_render(*p)).collect();
            let mut poly = Polyline3d::new(vertices)
                .with_color(Color::new(0.3, 0.4, 0.6, 0.5))
                .with_width(1.0);
            poly.perspective = true;
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
