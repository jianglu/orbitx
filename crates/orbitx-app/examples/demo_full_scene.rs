//! Demo 7: full scene-graph integration test.
//!
//! Replicates orbitx-app's rendering pipeline WITHOUT the HUD/MFD, using the
//! REAL ephemeris (PlanetarySystem via ephem_bridge) driving a REAL
//! SceneManager, plus the ecliptic grid + orbit rings and interactive camera.
//!
//! This is an adaptation of demo_camera_interaction.rs (Demo 8): same dual
//! sphere+billboard pipelines, line pipeline for ecliptic grid + orbit rings,
//! interactive camera controls, per-frame uniform writes, and a transparent
//! egui overlay. The only difference: the scene is driven by real ephemeris
//! rather than a hardcoded `const BODIES`.
//!
//! Run with:
//!   ORBITER_SRC=/path/to/orbiter cargo run -p orbitx-app --example demo_full_scene

use std::num::{NonZeroU32, NonZeroU64};
use std::sync::Arc;

use egui_wgpu::CallbackTrait;
use wgpu::util::DeviceExt;
use winit::{
    application::ApplicationHandler,
    event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::ActiveEventLoop,
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowAttributes},
};

use orbitx_app::ephem_bridge;
use orbitx_app::sphere::{self, Vertex};
use orbitx_dynamics::PlanetarySystem;
use orbitx_math::vec3::Vec3;
use orbitx_render::{CameraSystem, CoordinateBridge, ExternalCamMode, NodeType, SceneManager};

// ---------------------------------------------------------------------------
// Shaders
// ---------------------------------------------------------------------------

const PLANET_WGSL: &str = include_str!("planet_basic.wgsl");
const BILLBOARD_WGSL: &str = include_str!("../src/shader/billboard.wgsl");

/// One astronomical unit in meters.
const AU_M: f64 = 1.49597870700e11;

/// Generous upper bound on line vertices (ecliptic grid + orbit rings). The
/// circle segment counts are fixed, so the actual count is stable once the
/// scene is built; this just guarantees the buffer never overflows.
const LINE_VERTEX_CAPACITY: usize = 16384;

/// Line shader: log-depth colored 3D lines (ecliptic grid + orbit rings).
const LINE_WGSL: &str = concat!(
"struct LineUniforms {\n",
"    view_proj: mat4x4<f32>,\n",
"    log_depth: vec4<f32>,\n",
"};\n",
"@group(0) @binding(0) var<uniform> u: LineUniforms;\n",
"struct VsIn { @location(0) pos: vec3<f32>, @location(1) color: vec4<f32> };\n",
"struct VsOut { @builtin(position) clip: vec4<f32>, @location(0) color: vec4<f32> };\n",
"@vertex fn vs_main(in: VsIn) -> VsOut {\n",
"    var out: VsOut;\n",
"    let clip = u.view_proj * vec4<f32>(in.pos, 1.0);\n",
"    let c = u.log_depth.x;\n",
"    let inv_log_far = u.log_depth.z;\n",
"    let log_z = log2(c * clip.w + 1.0) * inv_log_far;\n",
"    out.clip = vec4<f32>(clip.x, clip.y, log_z * clip.w, clip.w);\n",
"    out.color = in.color;\n",
"    return out;\n",
"}\n",
"@fragment fn fs_main(in: VsOut) -> @location(0) vec4<f32> { return in.color; }\n",
);

// ---------------------------------------------------------------------------
// Uniform structs - must match WGSL exactly
// ---------------------------------------------------------------------------

/// Sphere uniforms (176 bytes) - matches planet.wgsl Uniforms.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    mvp: [[f32; 4]; 4],
    model: [[f32; 4]; 4],
    base_color: [f32; 4],
    light_dir: [f32; 4],
    log_depth: [f32; 4], // [C=1.0, far, inv_log_far, 0.0]
}

/// Billboard uniforms (128 bytes) - matches billboard.wgsl Uniforms.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct BillboardUniforms {
    center: [f32; 4],      // xyz = world position, w = unused (1.0)
    color: [f32; 4],       // rgba
    screen_size: [f32; 4], // x = pixel radius, y = vp_width, z = vp_height, w = unused
    vp_row0: [f32; 4],
    vp_row1: [f32; 4],
    vp_row2: [f32; 4],
    vp_row3: [f32; 4],
}

/// Line vertex: position + rgba color.
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct LineVertex {
    pos: [f32; 3],
    color: [f32; 4],
}

impl LineVertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4];

    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<LineVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

/// Line uniforms (80 bytes) - matches LINE_WGSL LineUniforms.
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct LineUniforms {
    view_proj: [[f32; 4]; 4],
    log_depth: [f32; 4],
}

// ---------------------------------------------------------------------------
// Line geometry (built each frame from live scene positions)
// ---------------------------------------------------------------------------

/// Returns line-list vertices in SIM space (pairs of points = segments):
/// an ecliptic grid centered at the Sun plus one orbit ring per planet node
/// at its current heliocentric distance.
fn build_line_geometry(scene: &SceneManager) -> Vec<(Vec3, [f32; 4])> {
    let mut v: Vec<(Vec3, [f32; 4])> = Vec::new();
    let seg = 128usize;

    // Ecliptic grid: concentric circles in the y=0 plane, centered at origin.
    let grid_color = [0.3f32, 0.35, 0.5, 0.45];
    let mut r = 0.5f64;
    while r <= 3.0001 {
        let radius = r * AU_M;
        for i in 0..seg {
            let a0 = (i as f64) / (seg as f64) * std::f64::consts::TAU;
            let a1 = ((i + 1) as f64) / (seg as f64) * std::f64::consts::TAU;
            v.push((Vec3::new(radius * a0.cos(), 0.0, radius * a0.sin()), grid_color));
            v.push((Vec3::new(radius * a1.cos(), 0.0, radius * a1.sin()), grid_color));
        }
        r += 0.5;
    }

    // Radial spokes every 30 degrees out to 3 AU.
    let rmax = 3.0 * AU_M;
    for k in 0..12 {
        let a = (k as f64) / 12.0 * std::f64::consts::TAU;
        v.push((Vec3::new(0.0, 0.0, 0.0), grid_color));
        v.push((Vec3::new(rmax * a.cos(), 0.0, rmax * a.sin()), grid_color));
    }

    // Orbit rings: one circle per planet node at its heliocentric distance
    // (Sun sits near the origin), in the ecliptic plane.
    for node in scene.nodes() {
        if let NodeType::Planet(ps) = &node.node_type {
            let radius = node.transform.position.length();
            let color = [ps.color[0], ps.color[1], ps.color[2], 0.8];
            for j in 0..seg {
                let a0 = (j as f64) / (seg as f64) * std::f64::consts::TAU;
                let a1 = ((j + 1) as f64) / (seg as f64) * std::f64::consts::TAU;
                v.push((Vec3::new(radius * a0.cos(), 0.0, radius * a0.sin()), color));
                v.push((Vec3::new(radius * a1.cos(), 0.0, radius * a1.sin()), color));
            }

            // Drop line: vertical segment from the body down to its projection
            // onto the ecliptic plane, showing its height above/below the plane.
            let p = node.transform.position;
            let foot = Vec3::new(p.x, 0.0, p.z);
            let drop_color = [ps.color[0], ps.color[1], ps.color[2], 0.55];
            v.push((p, drop_color));
            v.push((foot, drop_color));
        }
    }
    v
}

// ---------------------------------------------------------------------------
// Per-node render resources
// ---------------------------------------------------------------------------

/// Whether a node is rendered as a sphere or a billboard this frame.
#[derive(Clone, Copy, PartialEq)]
enum RenderMode {
    Sphere,
    Billboard,
}

struct BodyRender {
    // Sphere resources (always allocated, used when mode == Sphere)
    sphere_uniform_buffer: wgpu::Buffer,
    sphere_bind_group: wgpu::BindGroup,
    // Billboard resources (always allocated, used when mode == Billboard)
    billboard_uniform_buffer: wgpu::Buffer,
    billboard_bind_group: wgpu::BindGroup,
    mode: RenderMode,
}

// ---------------------------------------------------------------------------
// Callback - draws all renderable nodes in one paint call
// ---------------------------------------------------------------------------

struct SceneCallback {
    sphere_pipeline: wgpu::RenderPipeline,
    billboard_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    body_renders: Vec<BodyRender>,
    line_pipeline: wgpu::RenderPipeline,
    line_vertex_buffer: wgpu::Buffer,
    line_uniform_buffer: wgpu::Buffer,
    line_bind_group: wgpu::BindGroup,
    line_vertex_count: u32,
}

impl CallbackTrait for SceneCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        // Draw lines first (ecliptic grid + orbit rings), they write depth.
        if self.line_vertex_count > 0 {
            render_pass.set_pipeline(&self.line_pipeline);
            render_pass.set_bind_group(0, &self.line_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.line_vertex_buffer.slice(..));
            render_pass.draw(0..self.line_vertex_count, 0..1);
        }
        // Draw spheres (they write depth).
        for br in &self.body_renders {
            if br.mode == RenderMode::Sphere {
                render_pass.set_pipeline(&self.sphere_pipeline);
                render_pass.set_bind_group(0, &br.sphere_bind_group, &[]);
                render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
                render_pass.set_index_buffer(
                    self.index_buffer.slice(..),
                    wgpu::IndexFormat::Uint16,
                );
                render_pass.draw_indexed(0..self.index_count, 0, 0..1);
            }
        }
        // Draw billboards (no depth write, always visible).
        for br in &self.body_renders {
            if br.mode == RenderMode::Billboard {
                render_pass.set_pipeline(&self.billboard_pipeline);
                render_pass.set_bind_group(0, &br.billboard_bind_group, &[]);
                render_pass.draw(0..6, 0..1);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Scene state: ephemeris + SceneManager + camera + coord bridge
// ---------------------------------------------------------------------------

struct SceneState {
    planetary: PlanetarySystem,
    scene: SceneManager,
    has_ephemeris: bool,
    camera: CameraSystem,
    coord_bridge: CoordinateBridge,
    sim_time: f64,
    time_warp: f64,
    /// Indices into `scene.nodes()` of renderable nodes (Star + Planet).
    renderable_indices: Vec<usize>,
    /// Current render mode for each renderable node (parallel to above).
    body_modes: Vec<RenderMode>,
}

impl SceneState {
    fn new() -> Self {
        let orbiter_src = ephem_bridge::resolve_orbiter_src();
        let mut planetary = ephem_bridge::create_planetary_system(&orbiter_src);
        let has_ephemeris = planetary.bodies.iter().any(|b| b.ephemeris.is_some());
        let mut scene = ephem_bridge::create_scene_from_psys(&planetary);

        // Prime positions once so line rings / camera start sensibly.
        if has_ephemeris {
            planetary.mjd = ephem_bridge::sim_time_to_mjd(0.0);
            planetary.update_positions();
        }
        ephem_bridge::sync_positions(&planetary, &mut scene);

        // Renderable nodes are Stars and Planets only.
        let renderable_indices: Vec<usize> = scene
            .nodes()
            .iter()
            .enumerate()
            .filter(|(_, n)| matches!(n.node_type, NodeType::Star | NodeType::Planet(_)))
            .map(|(i, _)| i)
            .collect();
        let body_modes = vec![RenderMode::Billboard; renderable_indices.len()];

        let mut camera = CameraSystem::new();
        camera.target = 0; // Sun
        camera.set_ext_mode(ExternalCamMode::TargetRelative {
            dist: 6.0e11,
            phi: std::f64::consts::PI,
            theta: 0.3,
        });

        let coord_bridge = CoordinateBridge::new_solar_system(20.0);

        // Initial camera update.
        let body_positions: Vec<Vec3> =
            scene.nodes().iter().map(|n| n.transform.position).collect();
        camera.update(&body_positions, 0.016);

        Self {
            planetary,
            scene,
            has_ephemeris,
            camera,
            coord_bridge,
            sim_time: 0.0,
            time_warp: 1.0,
            renderable_indices,
            body_modes,
        }
    }

    /// Read the current camera distance from the external camera mode.
    fn current_dist(&self) -> f64 {
        match self.camera.ext_mode {
            ExternalCamMode::TargetRelative { dist, .. } => dist,
            _ => 0.0,
        }
    }

    /// Sun render-space direction for lighting; fallback if no Sun / at origin.
    fn light_dir(&self) -> glam::Vec3 {
        for node in self.scene.nodes() {
            if matches!(node.node_type, NodeType::Star) {
                let p = node.render_data.position;
                if p.length() > 1e-6 {
                    return p.normalize();
                }
                break;
            }
        }
        glam::Vec3::new(0.3, 1.0, 0.5).normalize()
    }

    /// Log-depth far value for the current render scale.
    fn log_depth(&self) -> [f32; 4] {
        let c = 1.0f32;
        let far = (1.0e14 * self.coord_bridge.scale()) as f32;
        let inv_log_far = 1.0 / (c * far + 1.0).log2();
        [c, far, inv_log_far, 0.0]
    }

    /// Advance simulation, sync positions, update camera and render data.
    fn update(&mut self, vp_width: f32, vp_height: f32) {
        // Advance time.
        self.sim_time += 0.016 * self.time_warp;
        if self.has_ephemeris {
            self.planetary.mjd = ephem_bridge::sim_time_to_mjd(self.sim_time);
            self.planetary.update_positions();
        }
        ephem_bridge::sync_positions(&self.planetary, &mut self.scene);

        // Camera.
        let body_positions: Vec<Vec3> =
            self.scene.nodes().iter().map(|n| n.transform.position).collect();
        self.camera.update(&body_positions, 0.016);
        self.coord_bridge.set_origin(self.camera.cam_pos_sim());
        self.camera.set_render_scale(self.coord_bridge.scale());
        self.camera.set_aspect(vp_width as f64 / vp_height as f64);
        let cam_pos = self.camera.cam_pos_sim();
        self.scene.update_all(&self.coord_bridge, &cam_pos);

        // Decide render mode per renderable node.
        for (slot, &node_idx) in self.renderable_indices.iter().enumerate() {
            let node = &self.scene.nodes()[node_idx];
            let pos: [f32; 3] = node.render_data.position.into();
            let scale = node.render_data.scale;
            let render_dist = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
            let fov_y = std::f32::consts::FRAC_PI_3;
            let screen_px = if render_dist > 1e-6 {
                (scale / render_dist) * vp_height / fov_y
            } else {
                0.0
            };
            let is_star = matches!(node.node_type, NodeType::Star);
            let min_visible_px = if is_star { 6.0 } else { 3.0 };
            self.body_modes[slot] = if screen_px >= min_visible_px {
                RenderMode::Sphere
            } else {
                RenderMode::Billboard
            };
        }
    }

    /// Base color for a renderable node.
    fn node_color(&self, node_idx: usize) -> [f32; 4] {
        let node = &self.scene.nodes()[node_idx];
        match &node.node_type {
            NodeType::Star => [1.0, 0.95, 0.4, 1.0],
            NodeType::Planet(ps) => ps.color,
            _ => [1.0, 1.0, 1.0, 1.0],
        }
    }

    /// Build sphere uniforms for a renderable node.
    fn sphere_uniforms(&self, node_idx: usize) -> Uniforms {
        let node = &self.scene.nodes()[node_idx];
        let render_pos = node.render_data.position;
        let render_radius = node.render_data.scale;

        let view = self.camera.view_matrix();
        let proj = self.camera.projection_matrix();
        let view_proj = proj * view;

        let model = glam::Mat4::from_scale_rotation_translation(
            glam::Vec3::splat(render_radius),
            glam::Quat::IDENTITY,
            render_pos,
        );
        let mvp = view_proj * model;
        let light = self.light_dir();

        Uniforms {
            mvp: mvp.to_cols_array_2d(),
            model: model.to_cols_array_2d(),
            base_color: self.node_color(node_idx),
            light_dir: [light.x, light.y, light.z, 0.0],
            log_depth: self.log_depth(),
        }
    }

    /// Build billboard uniforms for a renderable node.
    fn billboard_uniforms(&self, node_idx: usize, vp_width: f32, vp_height: f32) -> BillboardUniforms {
        let node = &self.scene.nodes()[node_idx];
        let render_pos = node.render_data.position;
        let scale = node.render_data.scale;

        let view = self.camera.view_matrix();
        let proj = self.camera.projection_matrix();
        let view_proj = proj * view;

        let render_dist = render_pos.length();
        let fov_y = std::f32::consts::FRAC_PI_3;
        let screen_px = if render_dist > 1e-6 {
            (scale / render_dist) * vp_height / fov_y
        } else {
            0.0
        };
        let is_star = matches!(node.node_type, NodeType::Star);
        let min_visible_px = if is_star { 6.0 } else { 3.0 };
        let pixel_radius = screen_px.max(min_visible_px);

        let vp_cols = view_proj.to_cols_array_2d();
        BillboardUniforms {
            center: [render_pos.x, render_pos.y, render_pos.z, 1.0],
            color: self.node_color(node_idx),
            screen_size: [pixel_radius, vp_width, vp_height, 0.0],
            vp_row0: vp_cols[0],
            vp_row1: vp_cols[1],
            vp_row2: vp_cols[2],
            vp_row3: vp_cols[3],
        }
    }

    /// Build line uniforms (view-projection + log-depth) for this frame.
    fn line_uniforms(&self) -> LineUniforms {
        let view = self.camera.view_matrix();
        let proj = self.camera.projection_matrix();
        let view_proj = proj * view;
        LineUniforms {
            view_proj: view_proj.to_cols_array_2d(),
            log_depth: self.log_depth(),
        }
    }

    /// Build render-space line vertices from live scene positions this frame.
    fn line_vertices(&self) -> Vec<LineVertex> {
        build_line_geometry(&self.scene)
            .into_iter()
            .map(|(p, c)| {
                let r = self.coord_bridge.to_render(&p);
                LineVertex {
                    pos: [r.x, r.y, r.z],
                    color: c,
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Application
// ---------------------------------------------------------------------------

struct App {
    window: Option<Arc<Window>>,
    egui_ctx: egui::Context,
    painter: Option<egui_wgpu::winit::Painter>,
    egui_state: Option<egui_winit::State>,
    scene_callback: Option<SceneCallback>,
    scene_state: SceneState,
    running: bool,
    /// Whether the left mouse button is currently held (orbit drag).
    dragging: bool,
    /// Last observed cursor position (physical pixels).
    last_cursor: (f64, f64),
}

impl App {
    fn new() -> Self {
        Self {
            window: None,
            egui_ctx: egui::Context::default(),
            painter: None,
            egui_state: None,
            scene_callback: None,
            scene_state: SceneState::new(),
            running: true,
            dragging: false,
            last_cursor: (0.0, 0.0),
        }
    }

    fn render(&mut self) {
        let painter = match &mut self.painter {
            Some(p) => p,
            None => return,
        };
        let egui_state = match &mut self.egui_state {
            Some(s) => s,
            None => return,
        };
        let window = match &self.window {
            Some(w) => w,
            None => return,
        };
        let callback = match &self.scene_callback {
            Some(c) => c,
            None => return,
        };

        let vp_size = window.inner_size();
        let vp_w = vp_size.width as f32;
        let vp_h = vp_size.height as f32;

        // Update scene state each frame.
        self.scene_state.update(vp_w, vp_h);

        // Update uniform buffers for each renderable node.
        if let Some(rs) = painter.render_state() {
            for (slot, br) in callback.body_renders.iter().enumerate() {
                let node_idx = self.scene_state.renderable_indices[slot];
                match self.scene_state.body_modes[slot] {
                    RenderMode::Sphere => {
                        let uniforms = self.scene_state.sphere_uniforms(node_idx);
                        rs.queue.write_buffer(
                            &br.sphere_uniform_buffer,
                            0,
                            bytemuck::cast_slice(&[uniforms]),
                        );
                    }
                    RenderMode::Billboard => {
                        let uniforms = self.scene_state.billboard_uniforms(node_idx, vp_w, vp_h);
                        rs.queue.write_buffer(
                            &br.billboard_uniform_buffer,
                            0,
                            bytemuck::cast_slice(&[uniforms]),
                        );
                    }
                }
            }

            // Update line uniforms + vertices each frame (planets move).
            let lu = self.scene_state.line_uniforms();
            rs.queue
                .write_buffer(&callback.line_uniform_buffer, 0, bytemuck::cast_slice(&[lu]));
            let lv = self.scene_state.line_vertices();
            let write_count = lv.len().min(LINE_VERTEX_CAPACITY);
            rs.queue.write_buffer(
                &callback.line_vertex_buffer,
                0,
                bytemuck::cast_slice(&lv[..write_count]),
            );
        }

        // Snapshot data for UI + callback.
        let body_modes: Vec<RenderMode> = self.scene_state.body_modes.clone();
        let live_line_count = build_line_geometry(&self.scene_state.scene)
            .len()
            .min(LINE_VERTEX_CAPACITY) as u32;
        let target = self.scene_state.camera.target;
        let dist = self.scene_state.current_dist();
        let sim_time = self.scene_state.sim_time;
        let time_warp = self.scene_state.time_warp;
        let has_ephemeris = self.scene_state.has_ephemeris;
        let body_count = self.scene_state.scene.len();

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            let rect = ui.max_rect();

            // Build body renders for the callback with current modes.
            let body_renders: Vec<BodyRender> = callback
                .body_renders
                .iter()
                .enumerate()
                .map(|(i, br)| BodyRender {
                    mode: body_modes[i],
                    sphere_uniform_buffer: br.sphere_uniform_buffer.clone(),
                    sphere_bind_group: br.sphere_bind_group.clone(),
                    billboard_uniform_buffer: br.billboard_uniform_buffer.clone(),
                    billboard_bind_group: br.billboard_bind_group.clone(),
                })
                .collect();

            let cb = egui_wgpu::Callback::new_paint_callback(
                rect,
                SceneCallback {
                    sphere_pipeline: callback.sphere_pipeline.clone(),
                    billboard_pipeline: callback.billboard_pipeline.clone(),
                    vertex_buffer: callback.vertex_buffer.clone(),
                    index_buffer: callback.index_buffer.clone(),
                    index_count: callback.index_count,
                    body_renders,
                    line_pipeline: callback.line_pipeline.clone(),
                    line_vertex_buffer: callback.line_vertex_buffer.clone(),
                    line_uniform_buffer: callback.line_uniform_buffer.clone(),
                    line_bind_group: callback.line_bind_group.clone(),
                    line_vertex_count: live_line_count,
                },
            );
            ui.painter().add(cb);

            egui::CentralPanel::default()
                .frame(
                    egui::Frame::new()
                        .inner_margin(8)
                        .fill(egui::Color32::TRANSPARENT),
                )
                .show(ui, |ui| {
                    ui.label("Demo 7: Full Scene Integration (ephemeris + SceneManager)");
                    ui.label(if has_ephemeris {
                        "Ephemeris: LIVE"
                    } else {
                        "Ephemeris: NONE"
                    });
                    ui.label(format!("Bodies: {}", body_count));
                    ui.label(format!("Camera target: [{}]", target));
                    ui.label(format!("Camera distance: {:.3e} m", dist));
                    ui.label(format!("sim_time: {:.1} s", sim_time));
                    ui.label(format!("time_warp: {:.3}x", time_warp));
                    ui.separator();
                    ui.label("Drag: orbit | Scroll: zoom | Tab/]/[: focus | Esc: quit");
                    ui.label("Ecliptic grid + orbit rings shown");
                });
        });

        egui_state.handle_platform_output(window, full_output.platform_output);
        let clipped_primitives = self
            .egui_ctx
            .tessellate(full_output.shapes, window.scale_factor() as f32);
        painter.paint_and_update_textures(
            egui::viewport::ViewportId::ROOT,
            window.scale_factor() as f32,
            [0.0, 0.0, 0.02, 1.0],
            &clipped_primitives,
            &full_output.textures_delta,
            vec![],
            window,
        );
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let window_attrs = WindowAttributes::default()
            .with_title("demo_full_scene")
            .with_inner_size(winit::dpi::LogicalSize::new(1024, 768));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(),
            egui::viewport::ViewportId::ROOT,
            &window,
            None,
            None,
            None,
        );

        // Painter with depth texture enabled (all pipelines use Depth32Float).
        let mut painter = pollster::block_on(egui_wgpu::winit::Painter::new(
            self.egui_ctx.clone(),
            egui_wgpu::WgpuConfiguration::default(),
            false,
            egui_wgpu::RendererOptions {
                depth_stencil_format: Some(wgpu::TextureFormat::Depth32Float),
                ..Default::default()
            },
        ));

        pollster::block_on(painter.set_window(
            egui::viewport::ViewportId::ROOT,
            Some(window.clone()),
        ))
        .expect("Failed to set window on painter");

        let renderable_count = self.scene_state.renderable_indices.len();

        let scene_callback = if let Some(rs) = painter.render_state() {
            let device = &rs.device;
            let target_format = rs.target_format;

            // ----------------------------------------------------------------
            // Sphere pipeline
            // ----------------------------------------------------------------

            let sphere_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("sphere-bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: Some(
                            NonZeroU64::new(std::mem::size_of::<Uniforms>() as u64).unwrap(),
                        ),
                    },
                    count: None,
                }],
            });

            let sphere_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("sphere-layout"),
                bind_group_layouts: &[Some(&sphere_bgl)],
                immediate_size: 0,
            });

            let sphere_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("planet-shader"),
                source: wgpu::ShaderSource::Wgsl(PLANET_WGSL.into()),
            });

            let sphere_pipeline =
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("sphere-pipeline"),
                    layout: Some(&sphere_layout),
                    vertex: wgpu::VertexState {
                        module: &sphere_shader,
                        entry_point: Some("vs_main"),
                        buffers: &[Vertex::desc().clone()],
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &sphere_shader,
                        entry_point: Some("fs_main"),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: target_format,
                            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                    }),
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        strip_index_format: None,
                        front_face: wgpu::FrontFace::Ccw,
                        cull_mode: Some(wgpu::Face::Back),
                        polygon_mode: wgpu::PolygonMode::Fill,
                        unclipped_depth: false,
                        conservative: false,
                    },
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: wgpu::TextureFormat::Depth32Float,
                        depth_write_enabled: Some(true),
                        depth_compare: Some(wgpu::CompareFunction::Less),
                        stencil: wgpu::StencilState::default(),
                        bias: wgpu::DepthBiasState::default(),
                    }),
                    multisample: wgpu::MultisampleState {
                        count: 1,
                        mask: !0,
                        alpha_to_coverage_enabled: false,
                    },
                    multiview_mask: None,
                    cache: None,
                });

            // Generate sphere geometry.
            let (vertices, indices) = sphere::generate_uv_sphere(24, 16);
            let index_count = indices.len() as u32;

            let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("sphere-vertices"),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("sphere-indices"),
                contents: bytemuck::cast_slice(&indices),
                usage: wgpu::BufferUsages::INDEX,
            });

            // ----------------------------------------------------------------
            // Billboard pipeline
            // ----------------------------------------------------------------

            let bb_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("bb-bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

            let bb_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("bb-layout"),
                bind_group_layouts: &[Some(&bb_bgl)],
                immediate_size: 0,
            });

            let bb_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("billboard-shader"),
                source: wgpu::ShaderSource::Wgsl(BILLBOARD_WGSL.into()),
            });

            let billboard_pipeline =
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("bb-pipeline"),
                    layout: Some(&bb_layout),
                    vertex: wgpu::VertexState {
                        module: &bb_shader,
                        entry_point: Some("vs_main"),
                        buffers: &[],
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &bb_shader,
                        entry_point: Some("fs_main"),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: target_format,
                            blend: Some(wgpu::BlendState {
                                color: wgpu::BlendComponent {
                                    src_factor: wgpu::BlendFactor::SrcAlpha,
                                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                                    operation: wgpu::BlendOperation::Add,
                                },
                                alpha: wgpu::BlendComponent {
                                    src_factor: wgpu::BlendFactor::One,
                                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                                    operation: wgpu::BlendOperation::Add,
                                },
                            }),
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                    }),
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        strip_index_format: None,
                        front_face: wgpu::FrontFace::Ccw,
                        cull_mode: None,
                        polygon_mode: wgpu::PolygonMode::Fill,
                        unclipped_depth: false,
                        conservative: false,
                    },
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: wgpu::TextureFormat::Depth32Float,
                        depth_write_enabled: Some(false),
                        depth_compare: Some(wgpu::CompareFunction::Less),
                        stencil: wgpu::StencilState::default(),
                        bias: wgpu::DepthBiasState::default(),
                    }),
                    multisample: wgpu::MultisampleState {
                        count: 1,
                        mask: !0,
                        alpha_to_coverage_enabled: false,
                    },
                    multiview_mask: None,
                    cache: None,
                });

            // ----------------------------------------------------------------
            // Per-node resources: create BOTH sphere and billboard buffers so
            // we can switch mode each frame without reallocating. One pair per
            // renderable node (fixed count at startup).
            // ----------------------------------------------------------------

            let mut body_renders = Vec::with_capacity(renderable_count);
            for slot in 0..renderable_count {
                let node_idx = self.scene_state.renderable_indices[slot];
                let color = self.scene_state.node_color(node_idx);

                let sphere_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(&format!("sphere-ub-{slot}")),
                    contents: bytemuck::cast_slice(&[Uniforms {
                        mvp: [[0.0; 4]; 4],
                        model: [[0.0; 4]; 4],
                        base_color: color,
                        light_dir: [0.0; 4],
                        log_depth: [1.0, 1.0, 1.0, 0.0],
                    }]),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let sphere_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(&format!("sphere-bg-{slot}")),
                    layout: &sphere_bgl,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: sphere_buf.as_entire_binding(),
                    }],
                });

                let bb_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(&format!("bb-ub-{slot}")),
                    contents: bytemuck::cast_slice(&[BillboardUniforms {
                        center: [0.0; 4],
                        color,
                        screen_size: [0.0; 4],
                        vp_row0: [0.0; 4],
                        vp_row1: [0.0; 4],
                        vp_row2: [0.0; 4],
                        vp_row3: [0.0; 4],
                    }]),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let bb_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(&format!("bb-bg-{slot}")),
                    layout: &bb_bgl,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: bb_buf.as_entire_binding(),
                    }],
                });

                body_renders.push(BodyRender {
                    sphere_uniform_buffer: sphere_buf,
                    sphere_bind_group: sphere_bg,
                    billboard_uniform_buffer: bb_buf,
                    billboard_bind_group: bb_bg,
                    mode: RenderMode::Billboard, // updated each frame
                });
            }

            // ----------------------------------------------------------------
            // Line pipeline (ecliptic grid + orbit rings)
            // ----------------------------------------------------------------

            let line_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("line-bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: Some(
                            NonZeroU64::new(std::mem::size_of::<LineUniforms>() as u64).unwrap(),
                        ),
                    },
                    count: None,
                }],
            });

            let line_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("line-ub"),
                size: std::mem::size_of::<LineUniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });

            let line_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("line-bg"),
                layout: &line_bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: line_uniform_buffer.as_entire_binding(),
                }],
            });

            let line_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("line-layout"),
                bind_group_layouts: &[Some(&line_bgl)],
                immediate_size: 0,
            });

            let line_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("line-shader"),
                source: wgpu::ShaderSource::Wgsl(LINE_WGSL.into()),
            });

            let line_pipeline =
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("line-pipeline"),
                    layout: Some(&line_layout),
                    vertex: wgpu::VertexState {
                        module: &line_shader,
                        entry_point: Some("vs_main"),
                        buffers: &[LineVertex::desc()],
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &line_shader,
                        entry_point: Some("fs_main"),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: target_format,
                            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                    }),
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::LineList,
                        strip_index_format: None,
                        front_face: wgpu::FrontFace::Ccw,
                        cull_mode: None,
                        polygon_mode: wgpu::PolygonMode::Fill,
                        unclipped_depth: false,
                        conservative: false,
                    },
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: wgpu::TextureFormat::Depth32Float,
                        depth_write_enabled: Some(true),
                        depth_compare: Some(wgpu::CompareFunction::Less),
                        stencil: wgpu::StencilState::default(),
                        bias: wgpu::DepthBiasState::default(),
                    }),
                    multisample: wgpu::MultisampleState {
                        count: 1,
                        mask: !0,
                        alpha_to_coverage_enabled: false,
                    },
                    multiview_mask: None,
                    cache: None,
                });

            let line_vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("line-vertices"),
                size: (LINE_VERTEX_CAPACITY * std::mem::size_of::<LineVertex>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });

            Some(SceneCallback {
                sphere_pipeline,
                billboard_pipeline,
                vertex_buffer,
                index_buffer,
                index_count,
                body_renders,
                line_pipeline,
                line_vertex_buffer,
                line_uniform_buffer,
                line_bind_group,
                line_vertex_count: 0,
            })
        } else {
            None
        };

        self.window = Some(window);
        self.egui_state = Some(egui_state);
        self.painter = Some(painter);
        self.scene_callback = scene_callback;
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        // Always forward events to egui first.
        if let (Some(egui_state), Some(window)) = (&mut self.egui_state, &self.window) {
            let _ = egui_state.on_window_event(window, &event);
        }

        match event {
            WindowEvent::CloseRequested => {
                self.running = false;
                event_loop.exit();
            }
            WindowEvent::Resized(physical_size) => {
                if physical_size.width == 0 || physical_size.height == 0 {
                    return;
                }
                if let Some(painter) = &mut self.painter {
                    if let (Some(w), Some(h)) = (
                        NonZeroU32::new(physical_size.width),
                        NonZeroU32::new(physical_size.height),
                    ) {
                        painter.on_window_resized(egui::viewport::ViewportId::ROOT, w, h);
                    }
                }
            }
            // Left mouse button toggles orbit-drag state.
            WindowEvent::MouseInput { button: MouseButton::Left, state, .. } => {
                self.dragging = state == ElementState::Pressed;
            }
            // Cursor movement: while dragging, orbit the camera.
            WindowEvent::CursorMoved { position, .. } => {
                let (x, y) = (position.x, position.y);
                if self.dragging {
                    let dx = x - self.last_cursor.0;
                    let dy = y - self.last_cursor.1;
                    // mouse_drag applies a 0.005 sensitivity factor internally,
                    // so pass raw pixel deltas here.
                    self.scene_state.camera.mouse_drag(dx, dy);
                }
                self.last_cursor = (x, y);
            }
            // Scroll wheel zooms in/out.
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(p) => p.y / 100.0,
                };
                self.scene_state.camera.mouse_scroll(scroll);
            }
            WindowEvent::KeyboardInput {
                event: KeyEvent {
                    physical_key, state, ..
                },
                ..
            } => {
                if state == ElementState::Pressed {
                    if let PhysicalKey::Code(key_code) = physical_key {
                        let count = self.scene_state.scene.len().max(1);
                        match key_code {
                            KeyCode::Escape => {
                                self.running = false;
                                event_loop.exit();
                            }
                            KeyCode::Tab | KeyCode::BracketRight => {
                                self.scene_state.camera.target =
                                    (self.scene_state.camera.target + 1) % count;
                            }
                            KeyCode::BracketLeft => {
                                self.scene_state.camera.target =
                                    (self.scene_state.camera.target + count - 1) % count;
                            }
                            _ => {}
                        }
                    }
                }
            }
            WindowEvent::RedrawRequested => {
                self.render();
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            _ => {}
        }
    }
}

fn main() {
    let event_loop = winit::event_loop::EventLoop::new().unwrap();
    event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    let mut app = App::new();
    eprintln!(
        "demo_full_scene: has_ephemeris={} bodies={} renderable={}",
        app.scene_state.has_ephemeris,
        app.scene_state.scene.len(),
        app.scene_state.renderable_indices.len(),
    );
    event_loop.run_app(&mut app).unwrap();
}
