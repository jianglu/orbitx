//! Demo 8: verifies interactive camera controls (mouse orbit, scroll zoom,
//! focus/target switching) render correctly.
//!
//! Reuses the full coordinate transform chain from Demo 4: hardcoded sim
//! positions -> CameraSystem update -> CoordinateBridge conversion ->
//! sphere/billboard decision -> rendering. On top of that, this demo wires up
//! interactive input: dragging the mouse orbits the camera around its target,
//! scrolling zooms in/out, and Tab / ] / [ cycle which body the camera looks at.
//!
//! This is an adaptation of demo_coord_camera.rs (Demo 4): same multi-body
//! scene, dual sphere+billboard pipelines, per-frame uniform updates, and a
//! transparent egui overlay, plus interactive camera controls.
//!
//! Run with:  cargo run -p orbitx-app --example demo_camera_interaction

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

use orbitx_app::sphere::{self, Vertex};
use orbitx_math::vec3::Vec3;
use orbitx_render::{CameraSystem, CoordinateBridge, ExternalCamMode};

// ---------------------------------------------------------------------------
// Shaders
// ---------------------------------------------------------------------------

const PLANET_WGSL: &str = include_str!("../src/shader/planet.wgsl");
const BILLBOARD_WGSL: &str = include_str!("../src/shader/billboard.wgsl");

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

// ---------------------------------------------------------------------------
// Body data
// ---------------------------------------------------------------------------

struct Body {
    name: &'static str,
    sim_pos: Vec3,
    radius_m: f64,
    color: [f32; 4],
    min_render_px: f32,
}

const BODIES: &[Body] = &[
    Body {
        name: "Sun",
        sim_pos: Vec3::ZERO,
        radius_m: 6.96e8,
        color: [1.0, 0.95, 0.4, 1.0],
        min_render_px: 12.0,
    },
    Body {
        name: "Earth",
        sim_pos: Vec3::new(1.0e11, 2.0e10, -3.0e10),
        radius_m: 6.371e6,
        color: [0.3, 0.6, 1.0, 1.0],
        min_render_px: 4.0,
    },
    Body {
        name: "Mars",
        sim_pos: Vec3::new(2.0e11, -1.0e10, 5.0e10),
        radius_m: 3.39e6,
        color: [1.0, 0.4, 0.2, 1.0],
        min_render_px: 3.0,
    },
];

// ---------------------------------------------------------------------------
// Per-body render resources
// ---------------------------------------------------------------------------

/// Whether a body is rendered as a sphere or a billboard.
#[derive(Clone, Copy, PartialEq)]
enum RenderMode {
    Sphere,
    Billboard,
}

struct BodyRender {
    name: &'static str,
    mode: RenderMode,
    // Sphere resources (always allocated, used when mode == Sphere)
    sphere_uniform_buffer: wgpu::Buffer,
    sphere_bind_group: wgpu::BindGroup,
    // Billboard resources (always allocated, used when mode == Billboard)
    billboard_uniform_buffer: wgpu::Buffer,
    billboard_bind_group: wgpu::BindGroup,
}

// ---------------------------------------------------------------------------
// Callback - draws all bodies in one paint call
// ---------------------------------------------------------------------------

struct SceneCallback {
    sphere_pipeline: wgpu::RenderPipeline,
    billboard_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    body_renders: Vec<BodyRender>,
}

impl CallbackTrait for SceneCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        // Draw spheres first (they write depth)
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
        // Draw billboards second (no depth write, always visible)
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
// Coordinate transform + camera state
// ---------------------------------------------------------------------------

struct CoordState {
    camera: CameraSystem,
    coord_bridge: CoordinateBridge,
    /// Current render mode for each body (updated each frame).
    body_modes: Vec<RenderMode>,
    /// Camera position in sim meters (for display).
    cam_pos_sim: Vec3,
    /// Render scale (for display).
    render_scale: f64,
}

impl CoordState {
    fn new() -> Self {
        let mut camera = CameraSystem::new();
        camera.target = 0; // Sun
        camera.set_ext_mode(ExternalCamMode::TargetRelative {
            dist: 4.0e11,
            phi: std::f64::consts::PI,
            theta: 0.2,
        });

        let coord_bridge = CoordinateBridge::new_solar_system(20.0);
        let render_scale = coord_bridge.scale();

        // Initial camera update
        let body_positions: Vec<Vec3> = BODIES.iter().map(|b| b.sim_pos).collect();
        camera.update(&body_positions, 0.016);

        let cam_pos_sim = camera.cam_pos_sim();

        Self {
            camera,
            coord_bridge,
            body_modes: vec![RenderMode::Billboard; BODIES.len()],
            cam_pos_sim,
            render_scale,
        }
    }

    /// Read the current camera distance from the external camera mode.
    fn current_dist(&self) -> f64 {
        match self.camera.ext_mode {
            ExternalCamMode::TargetRelative { dist, .. } => dist,
            _ => 0.0,
        }
    }

    /// Update coordinate bridge origin and compute render data for each body.
    fn update(&mut self, vp_width: f32, vp_height: f32) {
        // Update camera (uses the interactively-modified ext_mode + target)
        let body_positions: Vec<Vec3> = BODIES.iter().map(|b| b.sim_pos).collect();
        self.camera.update(&body_positions, 0.016);

        // Set floating-point origin to camera position
        self.cam_pos_sim = self.camera.cam_pos_sim();
        self.coord_bridge.set_origin(self.cam_pos_sim);
        self.render_scale = self.coord_bridge.scale();

        // Compute render scale for the camera
        self.camera.set_render_scale(self.render_scale);
        self.camera.set_aspect(vp_width as f64 / vp_height as f64);

        // Get view-projection matrix for screen-size estimation
        let view = self.camera.view_matrix();
        let proj = self.camera.projection_matrix();
        let view_proj = proj * view;

        // Decide sphere vs billboard for each body
        for (i, body) in BODIES.iter().enumerate() {
            let render_pos = self.coord_bridge.to_render(&body.sim_pos);
            let render_radius = self.coord_bridge.to_render_radius(body.radius_m);

            // Estimate screen size in pixels
            let screen_px =
                estimate_screen_pixels(&view_proj, render_pos, render_radius, vp_width, vp_height);

            self.body_modes[i] = if screen_px < body.min_render_px {
                RenderMode::Billboard
            } else {
                RenderMode::Sphere
            };
        }
    }

    /// Build sphere uniforms for a body.
    fn sphere_uniforms(&self, body_idx: usize) -> Uniforms {
        let body = &BODIES[body_idx];
        let render_pos = self.coord_bridge.to_render(&body.sim_pos);
        let render_radius = self.coord_bridge.to_render_radius(body.radius_m);

        let view = self.camera.view_matrix();
        let proj = self.camera.projection_matrix();
        let view_proj = proj * view;

        let model = glam::Mat4::from_scale_rotation_translation(
            glam::Vec3::splat(render_radius),
            glam::Quat::IDENTITY,
            render_pos,
        );
        let mvp = view_proj * model;

        // Light direction from sun render position
        let sun_render_pos = self.coord_bridge.to_render(&BODIES[0].sim_pos);
        let light_dir = if sun_render_pos.length() > 0.0 {
            sun_render_pos.normalize()
        } else {
            glam::Vec3::new(0.0, 1.0, 0.0)
        };

        // Log depth values
        let log_depth_c = 1.0f32;
        let log_depth_far = (1.0e14 * self.render_scale) as f32;
        let inv_log_far = 1.0 / (log_depth_c * log_depth_far + 1.0).log2();

        Uniforms {
            mvp: mvp.to_cols_array_2d(),
            model: model.to_cols_array_2d(),
            base_color: body.color,
            light_dir: [light_dir.x, light_dir.y, light_dir.z, 0.0],
            log_depth: [log_depth_c, log_depth_far, inv_log_far, 0.0],
        }
    }

    /// Build billboard uniforms for a body.
    fn billboard_uniforms(&self, body_idx: usize, vp_width: f32, vp_height: f32) -> BillboardUniforms {
        let body = &BODIES[body_idx];
        let render_pos = self.coord_bridge.to_render(&body.sim_pos);
        let render_radius = self.coord_bridge.to_render_radius(body.radius_m);

        let view = self.camera.view_matrix();
        let proj = self.camera.projection_matrix();
        let view_proj = proj * view;

        // Compute pixel radius for billboard
        let screen_px =
            estimate_screen_pixels(&view_proj, render_pos, render_radius, vp_width, vp_height);
        let pixel_radius = screen_px.max(body.min_render_px);

        let vp_cols = view_proj.to_cols_array_2d();

        BillboardUniforms {
            center: [render_pos.x, render_pos.y, render_pos.z, 1.0],
            color: body.color,
            screen_size: [pixel_radius, vp_width, vp_height, 0.0],
            vp_row0: vp_cols[0],
            vp_row1: vp_cols[1],
            vp_row2: vp_cols[2],
            vp_row3: vp_cols[3],
        }
    }
}

/// Estimate the screen pixel size of a sphere at the given render position and radius.
fn estimate_screen_pixels(
    _view_proj: &glam::Mat4,
    render_pos: glam::Vec3,
    render_radius: f32,
    _vp_width: f32,
    vp_height: f32,
) -> f32 {
    // Projected radius in pixels. The camera is the floating-point origin in
    // render space, so distance-to-camera = |render_pos|. Both the radius and
    // this distance are in render units, so the ratio is unit-consistent.
    // This matches scene_renderer.rs::FrameScene::from_scene.
    let fov_y = std::f32::consts::FRAC_PI_3; // 60 deg (camera default)
    let render_dist = render_pos.length();
    if render_dist <= 1e-6 {
        return 0.0;
    }
    (render_radius / render_dist) * vp_height / fov_y
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
    coord_state: CoordState,
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
            coord_state: CoordState::new(),
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

        // Update coordinate state each frame
        self.coord_state.update(vp_w, vp_h);

        // Update uniform buffers for each body
        if let Some(rs) = painter.render_state() {
            for (i, br) in callback.body_renders.iter().enumerate() {
                match self.coord_state.body_modes[i] {
                    RenderMode::Sphere => {
                        let uniforms = self.coord_state.sphere_uniforms(i);
                        rs.queue.write_buffer(
                            &br.sphere_uniform_buffer,
                            0,
                            bytemuck::cast_slice(&[uniforms]),
                        );
                    }
                    RenderMode::Billboard => {
                        let uniforms = self.coord_state.billboard_uniforms(i, vp_w, vp_h);
                        rs.queue.write_buffer(
                            &br.billboard_uniform_buffer,
                            0,
                            bytemuck::cast_slice(&[uniforms]),
                        );
                    }
                }
            }
        }

        // Snapshot body modes and camera info for UI and callback
        let body_modes: Vec<RenderMode> = self.coord_state.body_modes.clone();
        let target = self.coord_state.camera.target;
        let target_name = BODIES[target].name;
        let dist = self.coord_state.current_dist();

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            let rect = ui.max_rect();

            // Build body renders for the callback with current modes
            let body_renders: Vec<BodyRender> = callback
                .body_renders
                .iter()
                .enumerate()
                .map(|(i, br)| BodyRender {
                    name: br.name,
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
                    ui.label("Demo 8: Camera Interaction");
                    ui.label("Drag: orbit | Scroll: zoom | Tab/]/[: switch focus");
                    ui.label(format!("Focus target: [{}] {}", target, target_name));
                    ui.label(format!("Camera distance: {:.3e} m", dist));
                    ui.separator();
                    for (i, body) in BODIES.iter().enumerate() {
                        let mode_str = match body_modes[i] {
                            RenderMode::Sphere => "SPHERE",
                            RenderMode::Billboard => "BILLBOARD",
                        };
                        ui.label(format!("{}: {}", body.name, mode_str));
                    }
                    ui.separator();
                    ui.label("Press Esc to quit.");
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
            .with_title("demo_camera_interaction")
            .with_inner_size(winit::dpi::LogicalSize::new(800, 600));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(),
            egui::viewport::ViewportId::ROOT,
            &window,
            None,
            None,
            None,
        );

        // Painter with depth texture enabled (both pipelines use Depth32Float).
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

            // Generate sphere geometry
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
            // Per-body resources: create BOTH sphere and billboard buffers
            // so we can switch mode each frame without reallocating.
            // ----------------------------------------------------------------

            let mut body_renders = Vec::with_capacity(BODIES.len());
            for body in BODIES {
                // Sphere uniform buffer + bind group
                let sphere_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(&format!("sphere-ub-{}", body.name)),
                    contents: bytemuck::cast_slice(&[Uniforms {
                        mvp: [[0.0; 4]; 4],
                        model: [[0.0; 4]; 4],
                        base_color: body.color,
                        light_dir: [0.0; 4],
                        log_depth: [1.0, 1.0, 1.0, 0.0],
                    }]),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let sphere_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(&format!("sphere-bg-{}", body.name)),
                    layout: &sphere_bgl,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: sphere_buf.as_entire_binding(),
                    }],
                });

                // Billboard uniform buffer + bind group
                let bb_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(&format!("bb-ub-{}", body.name)),
                    contents: bytemuck::cast_slice(&[BillboardUniforms {
                        center: [0.0; 4],
                        color: body.color,
                        screen_size: [0.0; 4],
                        vp_row0: [0.0; 4],
                        vp_row1: [0.0; 4],
                        vp_row2: [0.0; 4],
                        vp_row3: [0.0; 4],
                    }]),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let bb_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(&format!("bb-bg-{}", body.name)),
                    layout: &bb_bgl,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: bb_buf.as_entire_binding(),
                    }],
                });

                body_renders.push(BodyRender {
                    name: body.name,
                    mode: RenderMode::Billboard, // updated each frame
                    sphere_uniform_buffer: sphere_buf,
                    sphere_bind_group: sphere_bg,
                    billboard_uniform_buffer: bb_buf,
                    billboard_bind_group: bb_bg,
                });
            }

            Some(SceneCallback {
                sphere_pipeline,
                billboard_pipeline,
                vertex_buffer,
                index_buffer,
                index_count,
                body_renders,
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
        // Always forward events to egui first (like the template does).
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
                    // mouse_drag already applies a 0.005 sensitivity factor
                    // internally, so pass raw pixel deltas here.
                    self.coord_state.camera.mouse_drag(dx, dy);
                }
                self.last_cursor = (x, y);
            }
            // Scroll wheel zooms in/out.
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(p) => p.y / 100.0,
                };
                self.coord_state.camera.mouse_scroll(scroll);
            }
            WindowEvent::KeyboardInput {
                event: KeyEvent {
                    physical_key, state, ..
                },
                ..
            } => {
                if state == ElementState::Pressed {
                    if let PhysicalKey::Code(key_code) = physical_key {
                        match key_code {
                            KeyCode::Escape => {
                                self.running = false;
                                event_loop.exit();
                            }
                            // Cycle to next body.
                            KeyCode::Tab | KeyCode::BracketRight => {
                                self.coord_state.camera.target =
                                    (self.coord_state.camera.target + 1) % BODIES.len();
                            }
                            // Cycle to previous body.
                            KeyCode::BracketLeft => {
                                self.coord_state.camera.target =
                                    (self.coord_state.camera.target + BODIES.len() - 1)
                                        % BODIES.len();
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
    event_loop.run_app(&mut app).unwrap();
}
