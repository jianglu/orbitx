//! Demo 2: verifies the billboard shader and render pipeline.
//!
//! Renders five colored discs at fixed NDC positions using the real
//! billboard.wgsl shader. No camera, no ephemeris, no coordinate transforms -
//! just an identity VP matrix so that center in world space maps 1:1 to clip
//! space. This confirms the billboard uniform layout, bind-group wiring, and
//! disc/glow fragment shader all work correctly.
//!
//! Run with:  cargo run -p orbitx-app --example demo_callback_billboard

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

// ---------------------------------------------------------------------------
// Billboard WGSL - reuse the production shader via include_str!
// ---------------------------------------------------------------------------

const BILLBOARD_WGSL: &str = include_str!("../src/shader/billboard.wgsl");

// ---------------------------------------------------------------------------
// Uniform struct - must match the WGSL Uniforms struct exactly.
// ---------------------------------------------------------------------------

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
// Billboard descriptions - hardcoded positions for testing.
// ---------------------------------------------------------------------------

struct BillboardDesc {
    center: [f32; 3],
    pixel_radius: f32,
    color: [f32; 4],
    label: &'static str,
}

const BILLBOARDS: &[BillboardDesc] = &[
    BillboardDesc { center: [0.0, 0.0, 0.5], pixel_radius: 30.0, color: [1.0, 0.95, 0.4, 1.0], label: "Sun (center)" },
    BillboardDesc { center: [-0.5, 0.3, 0.5], pixel_radius: 15.0, color: [0.3, 0.6, 1.0, 1.0], label: "Earth (upper-left)" },
    BillboardDesc { center: [0.5, -0.3, 0.5], pixel_radius: 10.0, color: [1.0, 0.4, 0.2, 1.0], label: "Mars (lower-right)" },
    BillboardDesc { center: [-0.7, -0.5, 0.5], pixel_radius: 8.0, color: [0.3, 1.0, 0.5, 1.0], label: "Green dot (lower-left)" },
    BillboardDesc { center: [0.6, 0.6, 0.5], pixel_radius: 12.0, color: [0.7, 0.3, 1.0, 1.0], label: "Purple (upper-right)" },
];

// ---------------------------------------------------------------------------
// Callback - draws all billboards in one paint call.
// ---------------------------------------------------------------------------

struct BillboardCallback {
    pipeline: wgpu::RenderPipeline,
    uniform_buffers: Vec<wgpu::Buffer>,
    bind_groups: Vec<wgpu::BindGroup>,
}

impl CallbackTrait for BillboardCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        for i in 0..self.uniform_buffers.len() {
            render_pass.set_pipeline(&self.pipeline);
            render_pass.set_bind_group(0, &self.bind_groups[i], &[]);
            render_pass.draw(0..6, 0..1);
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
    bb_callback: Option<BillboardCallback>,
    running: bool,
}

impl App {
    fn new() -> Self {
        Self {
            window: None,
            egui_ctx: egui::Context::default(),
            painter: None,
            egui_state: None,
            bb_callback: None,
            running: true,
        }
    }

    fn render(&mut self) {
        let painter = match &mut self.painter { Some(p) => p, None => return };
        let egui_state = match &mut self.egui_state { Some(s) => s, None => return };
        let window = match &self.window { Some(w) => w, None => return };
        let callback = match &self.bb_callback { Some(c) => c, None => return };

        // Update uniforms each frame so viewport size stays current.
        if let Some(rs) = painter.render_state() {
            let vp_size = window.inner_size();
            let vp_w = vp_size.width as f32;
            let vp_h = vp_size.height as f32;
            let identity: [[f32; 4]; 4] = [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ];
            for (i, desc) in BILLBOARDS.iter().enumerate() {
                let uniforms = BillboardUniforms {
                    center: [desc.center[0], desc.center[1], desc.center[2], 1.0],
                    color: desc.color,
                    screen_size: [desc.pixel_radius, vp_w, vp_h, 0.0],
                    vp_row0: identity[0], vp_row1: identity[1],
                    vp_row2: identity[2], vp_row3: identity[3],
                };
                rs.queue.write_buffer(&callback.uniform_buffers[i], 0, bytemuck::cast_slice(&[uniforms]));
            }
        }

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            let rect = ui.max_rect();
            let cb = egui_wgpu::Callback::new_paint_callback(
                rect,
                BillboardCallback {
                    pipeline: callback.pipeline.clone(),
                    uniform_buffers: callback.uniform_buffers.clone(),
                    bind_groups: callback.bind_groups.clone(),
                },
            );
            ui.painter().add(cb);

            egui::CentralPanel::default()
                .frame(egui::Frame::new().inner_margin(8).fill(egui::Color32::TRANSPARENT))
                .show(ui, |ui| {
                    ui.label("Billboard demo - 5 colored discs via billboard.wgsl");
                    ui.label("Identity VP matrix, hardcoded NDC positions, no camera.");
                    ui.label("Press Esc to quit.");
                });
        });

        egui_state.handle_platform_output(window, full_output.platform_output);
        let clipped_primitives = self.egui_ctx.tessellate(full_output.shapes, window.scale_factor() as f32);
        painter.paint_and_update_textures(
            egui::viewport::ViewportId::ROOT, window.scale_factor() as f32,
            [0.0, 0.0, 0.02, 1.0], &clipped_primitives, &full_output.textures_delta, vec![], window,
        );
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() { return; }

        let window_attrs = WindowAttributes::default()
            .with_title("demo_callback_billboard")
            .with_inner_size(winit::dpi::LogicalSize::new(800, 600));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(), egui::viewport::ViewportId::ROOT, &window, None, None, None,
        );

        // Painter with depth texture enabled (billboard pipeline uses Depth32Float).
        let mut painter = pollster::block_on(egui_wgpu::winit::Painter::new(
            self.egui_ctx.clone(), egui_wgpu::WgpuConfiguration::default(), false,
            egui_wgpu::RendererOptions {
                depth_stencil_format: Some(wgpu::TextureFormat::Depth32Float),
                ..Default::default()
            },
        ));

        pollster::block_on(painter.set_window(egui::viewport::ViewportId::ROOT, Some(window.clone())))
            .expect("Failed to set window on painter");

        let bb_callback = if let Some(rs) = painter.render_state() {
            let device = &rs.device;
            let target_format = rs.target_format;

            let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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

            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("bb-layout"),
                bind_group_layouts: &[Some(&bgl)],
                immediate_size: 0,
            });

            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("billboard-shader"),
                source: wgpu::ShaderSource::Wgsl(BILLBOARD_WGSL.into()),
            });

            // Pipeline matches scene_renderer.rs billboard pipeline exactly.
            let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("bb-pipeline"),
                layout: Some(&layout),
                vertex: wgpu::VertexState {
                    module: &shader, entry_point: Some("vs_main"), buffers: &[],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader, entry_point: Some("fs_main"),
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
                    strip_index_format: None, front_face: wgpu::FrontFace::Ccw,
                    cull_mode: None, polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false, conservative: false,
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: Some(false),
                    depth_compare: Some(wgpu::CompareFunction::Less),
                    stencil: wgpu::StencilState::default(),
                    bias: wgpu::DepthBiasState::default(),
                }),
                multisample: wgpu::MultisampleState {
                    count: 1, mask: !0, alpha_to_coverage_enabled: false,
                },
                multiview_mask: None, cache: None,
            });

            // Create one uniform buffer + bind group per billboard.
            let vp_size = window.inner_size();
            let vp_w = vp_size.width as f32;
            let vp_h = vp_size.height as f32;
            let identity: [[f32; 4]; 4] = [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ];
            let mut uniform_buffers = Vec::with_capacity(BILLBOARDS.len());
            let mut bind_groups = Vec::with_capacity(BILLBOARDS.len());
            for desc in BILLBOARDS {
                let uniforms = BillboardUniforms {
                    center: [desc.center[0], desc.center[1], desc.center[2], 1.0],
                    color: desc.color,
                    screen_size: [desc.pixel_radius, vp_w, vp_h, 0.0],
                    vp_row0: identity[0], vp_row1: identity[1],
                    vp_row2: identity[2], vp_row3: identity[3],
                };
                let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(desc.label),
                    contents: bytemuck::cast_slice(&[uniforms]),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(desc.label), layout: &bgl,
                    entries: &[wgpu::BindGroupEntry { binding: 0, resource: buffer.as_entire_binding() }],
                });
                uniform_buffers.push(buffer);
                bind_groups.push(bg);
            }
            Some(BillboardCallback { pipeline, uniform_buffers, bind_groups })
        } else { None };

        self.window = Some(window);
        self.egui_state = Some(egui_state);
        self.painter = Some(painter);
        self.bb_callback = bb_callback;
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
