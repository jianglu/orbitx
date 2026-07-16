//! 渲染抽象层 — 浮点原点坐标桥、相机系统、渲染图、场景节点。
//!
//! 移植自 Orbiter `Camera.cpp`/`Scene.cpp`/`VObject.cpp`，
//! 但使用 wgpu + glam 替代 D3D7，f64→f32 浮点原点替代 D3D 视觉原点。

pub mod camera;
pub mod coord;
pub mod render_graph;
pub mod scene;

pub use camera::{CameraSystem, ExternalCamMode, InternalCamMode, LogDepthConfig};
pub use coord::CoordinateBridge;
pub use render_graph::{RenderGraph, RenderPass, PassId, StandardPass};
pub use scene::{SceneNode, NodeType, Transform64, RenderData, NodeId, PlanetRenderState, VesselRenderState, SceneManager};
