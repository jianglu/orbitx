//! Demo 3: verifies the sphere shader + logarithmic depth buffer.
//!
//! Renders three colored 3D spheres using the production planet.wgsl shader
//! and sphere geometry from the same crate. The camera is hardcoded (no
//! CoordinateBridge), and each sphere has its own uniform buffer and bind
//! group so that all uniforms are baked at creation time (static scene).
//!
//! Run with:  cargo run -p orbitx-app --example demo_callback_sphere

use std::num::NonZeroU32;
use std::sync::Arc;

use egui_wgpu::CallbackTrait;
use wgpu::util::DeviceExt;
use winit::{
    application::ApplicationHandler,
    event::{ElementState, KeyEvent, WindowEvent},
    event_loop::ActiveEventLoop,
    window::{Window, WindowAttributes},
};

use orbitx_app::sphere::{self, Vertex};

// ---------------------------------------------------------------------------
// Planet WGSL - reuse the production shader via include_str!
// ---------------------------------------------------------------------------

const PLANET_WGSL: &str = include_str!("../src/shader/planet.wgsl");

// ---------------------------------------------------------------------------
// Uniform struct - must match the WGSL Uniforms struct exactly (176 bytes).
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    mvp: [[f32; 4]; 4],      // 64 bytes
    model: [[f32; 4]; 4],     // 64 bytes
    base_color: [f32; 4],     // 16 bytes
    light_dir: [f32; 4],      // 16 bytes
    log_depth: [f32; 4],      // 16 bytes: [C=1.0, far, inv_log_far, 0.0]
}

// ---------------------------------------------------------------------------
// Sphere descriptions - hardcoded positions, scales, and colors for testing.
// ---------------------------------------------------------------------------

struct SphereDesc {
    position: [f32; 3],
    scale: f32,
    color: [f32; 4],
    label: &'static str,
}

const SPHERES: &[SphereDesc] = &[
    SphereDesc {
        position: [0.0, 0.0, 0.0],
        scale: 0.5,
        color: [0.2, 0.4, 1.0, 1.0],
        label: "Blue (center)",
    },
    SphereDesc {
        position: [-1.5, 0.0, -2.0],
        scale: 0.3,
        color: [1.0, 0.2, 0.15, 1.0],
        label: "Red (left)",
    },
    SphereDesc {
        position: [1.0, -0.5, -1.0],
        scale: 0.4,
        color: [0.2, 0.9, 0.3, 1.0],
        label: "Green (right)",
    },
];

// ---------------------------------------------------------------------------
// Camera constants
// ---------------------------------------------------------------------------

const LIGHT_DIR: [f32; 4] = [0.3, 1.0, 0.5, 0.0]; // normalized in shader

const LOG_DEPTH_C: f32 = 1.0;
const LOG_DEPTH_FAR: f32 = 1000.0;

fn compute_view_proj() -> glam::Mat4 {
    let view = glam::Mat4::look_at_rh(
        glam::Vec3::new(0.0, 0.0, 5.0),
        glam::Vec3::ZERO,
        glam::Vec3::Y,
    );
    let proj = glam::Mat4::perspective_rh(
        std::f32::consts::FRAC_PI_3,
        4.0 / 3.0,
        0.1,
        1000.0,
    );
    proj * view
}

// ---------------------------------------------------------------------------
// Callback - draws all spheres in one paint call.
// ---------------------------------------------------------------------------

struct SphereCallback {
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    uniform_buffers: Vec<wgpu::Buffer>,
    bind_groups: Vec<wgpu::BindGroup>,
}

impl CallbackTrait for SphereCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        for i in 0..SPHERES.len() {
            render_pass.set_pipeline(&self.pipeline);
            render_pass.set_bind_group(0, &self.bind_groups[i], &[]);
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
            render_pass.draw_indexed(0..self.index_count, 0, 0..1);
        }
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
    sphere_callback: Option<SphereCallback>,
    running: bool,
}

impl App {
    fn new() -> Self {
        Self {
            window: None,
            egui_ctx: egui::Context::default(),
            painter: None,
            egui_state: None,
            sphere_callback: None,
            running: true,
        }
    }

    fn render(&mut self) {
        let painter = match &mut self.painter { Some(p) => p, None => return };
        let egui_state = match &mut self.egui_state { Some(s) => s, None => return };
        let window = match &self.window { Some(w) => w, None => return };
        let callback = match &self.sphere_callback { Some(c) => c, None => return };

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            let rect = ui.max_rect();
            let cb = egui_wgpu::Callback::new_paint_callback(
                rect,
                SphereCallback {
                    pipeline: callback.pipeline.clone(),
                    vertex_buffer: callback.vertex_buffer.clone(),
                    index_buffer: callback.index_buffer.clone(),
                    index_count: callback.index_count,
                    uniform_buffers: callback.uniform_buffers.clone(),
                    bind_groups: callback.bind_groups.clone(),
                },
            );
            ui.painter().add(cb);

            egui::CentralPanel::default()
                .frame(egui::Frame::new().inner_margin(8).fill(egui::Color32::TRANSPARENT))
                .show(ui, |ui| {
                    ui.label("Sphere + Log Depth Demo");
                    ui.label("3 colored spheres via planet.wgsl with logarithmic depth.");
                    ui.label("Press Esc to quit.");
                });
        });

        egui_state.handle_platform_output(window, full_output.platform_output);
        let clipped_primitives = self.egui_ctx.tessellate(full_output.shapes, window.scale_factor() as f32);
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
        if self.window.is_some() { return; }

        let window_attrs = WindowAttributes::default()
            .with_title("demo_callback_sphere")
            .with_inner_size(winit::dpi::LogicalSize::new(800, 600));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(),
            egui::viewport::ViewportId::ROOT,
            &window,
            None, None, None,
        );

        // Painter with depth texture enabled (sphere pipeline uses Depth32Float).
        let mut painter = pollster::block_on(egui_wgpu::winit::Painter::new(
            self.egui_ctx.clone(),
            egui_wgpu::WgpuConfiguration::default(),
            false,
            egui_wgpu::RendererOptions {
                depth_stencil_format: Some(wgpu::TextureFormat::Depth32Float),
                ..Default::default()
            },
        ));

        pollster::block_on(painter.set_window(egui::viewport::ViewportId::ROOT, Some(window.clone())))
            .expect("Failed to set window on painter");

        let sphere_callback = if let Some(rs) = painter.render_state() {
            let device = &rs.device;
            let target_format = rs.target_format;

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

            // Bind group layout: single uniform buffer at binding 0.
            let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("sphere-bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: Some(std::num::NonZeroU64::new(std::mem::size_of::<Uniforms>() as u64).unwrap()),
                    },
                    count: None,
                }],
            });

            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("sphere-layout"),
                bind_group_layouts: &[Some(&bgl)],
                immediate_size: 0,
            });

            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("planet-shader"),
                source: wgpu::ShaderSource::Wgsl(PLANET_WGSL.into()),
            });

            // Pipeline matches scene_renderer.rs sphere pipeline exactly.
            let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("sphere-pipeline"),
                layout: Some(&layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    buffers: &[Vertex::desc().clone()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
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

            // Create one uniform buffer + bind group per sphere.
            let view_proj = compute_view_proj();
            let inv_log_far = 1.0 / (LOG_DEPTH_C * LOG_DEPTH_FAR + 1.0).log2();

            let mut uniform_buffers = Vec::with_capacity(SPHERES.len());
            let mut bind_groups = Vec::with_capacity(SPHERES.len());
            for desc in SPHERES {
                let model = glam::Mat4::from_scale_rotation_translation(
                    glam::Vec3::splat(desc.scale),
                    glam::Quat::IDENTITY,
                    glam::Vec3::from(desc.position),
                );
                let mvp = view_proj * model;
                let uniforms = Uniforms {
                    mvp: mvp.to_cols_array_2d(),
                    model: model.to_cols_array_2d(),
                    base_color: desc.color,
                    light_dir: LIGHT_DIR,
                    log_depth: [LOG_DEPTH_C, LOG_DEPTH_FAR, inv_log_far, 0.0],
                };
                let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(desc.label),
                    contents: bytemuck::cast_slice(&[uniforms]),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(desc.label),
                    layout: &bgl,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: buffer.as_entire_binding(),
                    }],
                });
                uniform_buffers.push(buffer);
                bind_groups.push(bg);
            }

            Some(SphereCallback {
                pipeline,
                vertex_buffer,
                index_buffer,
                index_count,
                uniform_buffers,
                bind_groups,
            })
        } else { None };

        self.window = Some(window);
        self.egui_state = Some(egui_state);
        self.painter = Some(painter);
        self.sphere_callback = sphere_callback;
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        if let (Some(egui_state), Some(window)) = (&mut self.egui_state, &self.window) {
            let _ = egui_state.on_window_event(window, &event);
        }

        match event {
            WindowEvent::CloseRequested => {
                self.running = false;
                event_loop.exit();
            }
            WindowEvent::Resized(physical_size) => {
                if physical_size.width == 0 || physical_size.height == 0 { return; }
                if let Some(painter) = &mut self.painter {
                    if let (Some(w), Some(h)) = (
                        NonZeroU32::new(physical_size.width),
                        NonZeroU32::new(physical_size.height),
                    ) {
                        painter.on_window_resized(egui::viewport::ViewportId::ROOT, w, h);
                    }
                }
            }
            WindowEvent::KeyboardInput {
                event: KeyEvent { physical_key, state, .. }, ..
            } => {
                if state == ElementState::Pressed {
                    if let winit::keyboard::PhysicalKey::Code(key_code) = physical_key {
                        if key_code == winit::keyboard::KeyCode::Escape {
                            self.running = false;
                            event_loop.exit();
                        }
                    }
                }
            }
            WindowEvent::RedrawRequested => {
                self.render();
                if let Some(window) = &self.window { window.request_redraw(); }
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
