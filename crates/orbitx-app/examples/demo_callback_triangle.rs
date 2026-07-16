//! Minimal demo: renders a red triangle via egui_wgpu::CallbackTrait.
//!
//! Run with:  cargo run -p orbitx-app --example demo_callback_triangle

use std::num::NonZeroU32;
use std::sync::Arc;

use egui_wgpu::CallbackTrait;
use winit::{
    application::ApplicationHandler,
    event::{ElementState, KeyEvent, WindowEvent},
    event_loop::ActiveEventLoop,
    window::{Window, WindowAttributes},
};

// ---------------------------------------------------------------------------
// WGSL shader (uses concat! to avoid raw-string-literal edge cases)
// ---------------------------------------------------------------------------

const TRIANGLE_WGSL: &str = concat!(
    "@vertex\n",
    "fn vs_main(@builtin(vertex_index) vi: u32) -> @builtin(position) vec4f {\n",
    "    var pos = array<vec2f, 3>(\n",
    "        vec2f(-0.5, -0.5),\n",
    "        vec2f( 0.5, -0.5),\n",
    "        vec2f( 0.0,  0.5),\n",
    "    );\n",
    "    return vec4f(pos[vi], 0.0, 1.0);\n",
    "}\n",
    "\n",
    "@fragment\n",
    "fn fs_main() -> @location(0) vec4f {\n",
    "    return vec4f(1.0, 0.0, 0.0, 1.0);\n",
    "}\n",
);

// ---------------------------------------------------------------------------
// Callback struct - pipeline is stored directly, NOT in callback_resources
// ---------------------------------------------------------------------------

struct TriangleCallback {
    pipeline: wgpu::RenderPipeline,
}

impl CallbackTrait for TriangleCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        render_pass.set_pipeline(&self.pipeline);
        render_pass.draw(0..3, 0..1);
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
    triangle_pipeline: Option<wgpu::RenderPipeline>,
    running: bool,
}

impl App {
    fn new() -> Self {
        Self {
            window: None,
            egui_ctx: egui::Context::default(),
            painter: None,
            egui_state: None,
            triangle_pipeline: None,
            running: true,
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
        let pipeline = match &self.triangle_pipeline {
            Some(p) => p.clone(),
            None => return,
        };

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            // 1) Add the callback FIRST so it paints behind everything else.
            let rect = ui.max_rect();
            let callback = egui_wgpu::Callback::new_paint_callback(
                rect,
                TriangleCallback { pipeline: pipeline.clone() },
            );
            ui.painter().add(callback);

            // 2) CentralPanel with transparent fill so the triangle shows through.
            egui::CentralPanel::default()
                .frame(
                    egui::Frame::new()
                        .inner_margin(8)
                        .fill(egui::Color32::TRANSPARENT),
                )
                .show(ui, |ui| {
                    ui.label("Red triangle rendered via egui_wgpu::CallbackTrait");
                });
        });

        egui_state.handle_platform_output(window, full_output.platform_output);

        let clipped_primitives = self.egui_ctx.tessellate(
            full_output.shapes,
            window.scale_factor() as f32,
        );

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
            .with_title("demo_callback_triangle")
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

        // Painter with NO depth_stencil_format
        let mut painter = pollster::block_on(egui_wgpu::winit::Painter::new(
            self.egui_ctx.clone(),
            egui_wgpu::WgpuConfiguration::default(),
            false,
            egui_wgpu::RendererOptions {
                depth_stencil_format: None,
                ..Default::default()
            },
        ));

        pollster::block_on(painter.set_window(
            egui::viewport::ViewportId::ROOT,
            Some(window.clone()),
        ))
        .expect("Failed to set window on painter");

        // Create a simple render pipeline for the red triangle.
        let triangle_pipeline = if let Some(rs) = painter.render_state() {
            let device = &rs.device;
            let target_format = rs.target_format;

            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("triangle-shader"),
                source: wgpu::ShaderSource::Wgsl(TRIANGLE_WGSL.into()),
            });

            let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("triangle-layout"),
                bind_group_layouts: &[],
                immediate_size: 0,
            });

            Some(device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("triangle-pipeline"),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    buffers: &[],
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
                    cull_mode: None,
                    polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false,
                    conservative: false,
                },
                depth_stencil: None,
                multisample: wgpu::MultisampleState {
                    count: 1,
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                multiview_mask: None,
                cache: None,
            }))
        } else {
            None
        };

        self.window = Some(window);
        self.egui_state = Some(egui_state);
        self.painter = Some(painter);
        self.triangle_pipeline = triangle_pipeline;
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
            WindowEvent::KeyboardInput {
                event: KeyEvent {
                    physical_key,
                    state,
                    ..
                },
                ..
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
