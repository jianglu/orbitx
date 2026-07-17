//! Demo: Atmosphere Shell (Earth) - P3B-2 verification.
//!
//! Verifies the atmosphere-shell limb-glow shader on a textured Earth. It
//! renders the textured Earth sphere (same as demo_textured_planet) PLUS a
//! slightly larger atmosphere shell (radius * 1.03) that produces a blue limb
//! glow (halo) hugging the lit limb and fading on the night side.
//!
//! Surface uses `src/shader/planet.wgsl` (group 0 = uniforms, group 1 =
//! texture + sampler). The shell uses `src/shader/atmosphere.wgsl` (a single
//! uniform buffer at group 0 binding 0) rendered with premultiplied-alpha
//! blending over the planet + background.
//!
//! Run with:  cargo run -p orbitx-app --example demo_atmosphere

use std::num::NonZeroU32;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

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
// WGSL - reuse the production shaders via include_str!
// ---------------------------------------------------------------------------

const PLANET_WGSL: &str = include_str!("../src/shader/planet.wgsl");
const ATMO_WGSL: &str = include_str!("../src/shader/atmosphere.wgsl");

// ---------------------------------------------------------------------------
// Uniform structs - must match the WGSL structs exactly (176 bytes each).
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    mvp: [[f32; 4]; 4],   // 64 bytes
    model: [[f32; 4]; 4], // 64 bytes
    base_color: [f32; 4], // 16 bytes
    light_dir: [f32; 4],  // 16 bytes
    log_depth: [f32; 4],  // 16 bytes: [C=1.0, far, inv_log_far, use_texture=1.0]
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AtmoUniforms {
    mvp: [[f32; 4]; 4],   // 64 bytes
    model: [[f32; 4]; 4], // 64 bytes
    atmo_color: [f32; 4], // 16 bytes: rgb tint, a = intensity
    light_dir: [f32; 4],  // 16 bytes
    log_depth: [f32; 4],  // 16 bytes: [C, far, inv_log_far, unused]
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const LIGHT_DIR: [f32; 4] = [0.4, 0.6, 0.7, 0.0]; // normalized in shader

const LOG_DEPTH_C: f32 = 1.0;
const LOG_DEPTH_FAR: f32 = 1000.0;

/// Atmosphere shell radius factor relative to the unit planet (3% larger).
const ATMO_SCALE: f32 = 1.03;

/// Sky-blue tint, intensity 1.0.
const ATMO_COLOR: [f32; 4] = [0.3, 0.55, 1.0, 1.0];

/// Resolve the Earth texture path. Compile-time workspace path first
/// (cwd-independent), then cwd-relative fallback. Mirrors
/// scene_renderer.rs::resolve_texture_dir().
fn resolve_earth_texture() -> Option<PathBuf> {
    let bundled = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("assets")
        .join("textures")
        .join("planets")
        .join("Earth.jpg");
    if bundled.is_file() {
        return Some(bundled);
    }
    let cwd = PathBuf::from("assets/textures/planets/Earth.jpg");
    if cwd.is_file() {
        return Some(cwd);
    }
    None
}

/// Texture bind group layout (group 1): equirectangular map + sampler.
/// Copied from scene_renderer.rs::make_texture_bgl.
fn make_texture_bgl(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("planet-tex-bgl"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
    })
}

/// Upload RGBA8 pixels as a 2D texture and return its view.
/// Copied from scene_renderer.rs::upload_texture.
fn upload_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    label: &str,
    width: u32,
    height: u32,
    rgba: &[u8],
) -> wgpu::TextureView {
    let size = wgpu::Extent3d { width, height, depth_or_array_layers: 1 };
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &tex,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        rgba,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(4 * width),
            rows_per_image: Some(height),
        },
        size,
    );
    tex.create_view(&wgpu::TextureViewDescriptor::default())
}

/// Camera orbiting the sphere; angle drives a slow rotation over time.
fn compute_view_proj(angle: f32, aspect: f32) -> glam::Mat4 {
    let eye = glam::Vec3::new(3.0 * angle.cos(), 0.6, 3.0 * angle.sin());
    let view = glam::Mat4::look_at_rh(eye, glam::Vec3::ZERO, glam::Vec3::Y);
    let proj = glam::Mat4::perspective_rh(std::f32::consts::FRAC_PI_3, aspect, 0.01, 1000.0);
    proj * view
}

// ---------------------------------------------------------------------------
// Callback - draws the textured Earth sphere PLUS the atmosphere shell.
// ---------------------------------------------------------------------------

struct SphereCallback {
    // Shared sphere geometry (reused by both draws).
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,

    // Planet surface pipeline (textured Earth).
    pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
    texture_bind_group: wgpu::BindGroup,

    // Atmosphere shell pipeline (blue limb glow).
    atmo_pipeline: wgpu::RenderPipeline,
    atmo_uniform_buffer: wgpu::Buffer,
    atmo_uniform_bind_group: wgpu::BindGroup,
}

impl CallbackTrait for SphereCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        // Draw the textured planet first (writes depth).
        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, &self.uniform_bind_group, &[]);
        render_pass.set_bind_group(1, &self.texture_bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        render_pass.draw_indexed(0..self.index_count, 0, 0..1);

        // Then draw the atmosphere shell (blends over planet + background).
        render_pass.set_pipeline(&self.atmo_pipeline);
        render_pass.set_bind_group(0, &self.atmo_uniform_bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        render_pass.draw_indexed(0..self.index_count, 0, 0..1);
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
    start: Instant,
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
            start: Instant::now(),
            running: true,
        }
    }

    fn render(&mut self) {
        let painter = match &mut self.painter { Some(p) => p, None => return };
        let egui_state = match &mut self.egui_state { Some(s) => s, None => return };
        let window = match &self.window { Some(w) => w, None => return };
        let callback = match &self.sphere_callback { Some(c) => c, None => return };

        // Update both uniform buffers each frame on the main thread: the camera
        // orbits over time so mvp changes every frame. The texture bind group
        // is static (built once).
        if let Some(rs) = painter.render_state() {
            let size = window.inner_size();
            let aspect = if size.height > 0 {
                size.width as f32 / size.height as f32
            } else {
                4.0 / 3.0
            };
            let t = self.start.elapsed().as_secs_f32();
            let angle = t * 0.3;
            let view_proj = compute_view_proj(angle, aspect);
            let inv_log_far = 1.0 / (LOG_DEPTH_C * LOG_DEPTH_FAR + 1.0).log2();

            // Planet: unit sphere at origin (radius 1.0), identity model.
            let model = glam::Mat4::IDENTITY;
            let mvp = view_proj * model;
            let uniforms = Uniforms {
                mvp: mvp.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                base_color: [1.0, 1.0, 1.0, 1.0],
                light_dir: LIGHT_DIR,
                // Last component (use_texture) = 1.0 so the shader samples the texture.
                log_depth: [LOG_DEPTH_C, LOG_DEPTH_FAR, inv_log_far, 1.0],
            };
            rs.queue
                .write_buffer(&callback.uniform_buffer, 0, bytemuck::cast_slice(&[uniforms]));

            // Atmosphere shell: same planet model scaled 3% larger.
            let atmo_model = glam::Mat4::from_scale(glam::Vec3::splat(ATMO_SCALE));
            let atmo_mvp = view_proj * atmo_model;
            let atmo_uniforms = AtmoUniforms {
                mvp: atmo_mvp.to_cols_array_2d(),
                model: atmo_model.to_cols_array_2d(),
                atmo_color: ATMO_COLOR,
                light_dir: LIGHT_DIR,
                log_depth: [LOG_DEPTH_C, LOG_DEPTH_FAR, inv_log_far, 0.0],
            };
            rs.queue.write_buffer(
                &callback.atmo_uniform_buffer,
                0,
                bytemuck::cast_slice(&[atmo_uniforms]),
            );
        }

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            let rect = ui.max_rect();
            let cb = egui_wgpu::Callback::new_paint_callback(
                rect,
                SphereCallback {
                    vertex_buffer: callback.vertex_buffer.clone(),
                    index_buffer: callback.index_buffer.clone(),
                    index_count: callback.index_count,
                    pipeline: callback.pipeline.clone(),
                    uniform_buffer: callback.uniform_buffer.clone(),
                    uniform_bind_group: callback.uniform_bind_group.clone(),
                    texture_bind_group: callback.texture_bind_group.clone(),
                    atmo_pipeline: callback.atmo_pipeline.clone(),
                    atmo_uniform_buffer: callback.atmo_uniform_buffer.clone(),
                    atmo_uniform_bind_group: callback.atmo_uniform_bind_group.clone(),
                },
            );
            ui.painter().add(cb);

            egui::CentralPanel::default()
                .frame(egui::Frame::new().inner_margin(8).fill(egui::Color32::TRANSPARENT))
                .show(ui, |ui| {
                    ui.label("Demo: Atmosphere Shell (Earth)");
                    ui.label("Blue limb glow via Fresnel rim + day-side fade");
                    ui.label("The halo should hug the lit limb and fade on the night side");
                    ui.label("Press Esc to quit.");
                });
        });

        egui_state.handle_platform_output(window, full_output.platform_output);
        let clipped_primitives =
            self.egui_ctx.tessellate(full_output.shapes, window.scale_factor() as f32);
        painter.paint_and_update_textures(
            egui::viewport::ViewportId::ROOT,
            window.scale_factor() as f32,
            [0.02, 0.02, 0.05, 1.0],
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
            .with_title("demo_atmosphere")
            .with_inner_size(winit::dpi::LogicalSize::new(800, 600));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(),
            egui::viewport::ViewportId::ROOT,
            &window,
            None, None, None,
        );

        // Painter with depth texture enabled (pipelines use Depth32Float).
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
            let queue = &rs.queue;
            let target_format = rs.target_format;

            // Generate sphere geometry (position + normal + uv), shared by both draws.
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

            // -----------------------------------------------------------------
            // Planet surface pipeline (textured Earth).
            // -----------------------------------------------------------------

            // Group 0: single uniform buffer at binding 0 (VERTEX | FRAGMENT).
            let uniform_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("sphere-bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: Some(
                            std::num::NonZeroU64::new(std::mem::size_of::<Uniforms>() as u64).unwrap(),
                        ),
                    },
                    count: None,
                }],
            });

            // Group 1: texture + sampler (FRAGMENT).
            let tex_bgl = make_texture_bgl(device);

            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("sphere-layout"),
                bind_group_layouts: &[Some(&uniform_bgl), Some(&tex_bgl)],
                immediate_size: 0,
            });

            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("planet-shader"),
                source: wgpu::ShaderSource::Wgsl(PLANET_WGSL.into()),
            });

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

            // Uniform buffer + group-0 bind group (updated per frame).
            let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("earth-uniform"),
                size: std::mem::size_of::<Uniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("earth-uniform-bg"),
                layout: &uniform_bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform_buffer.as_entire_binding(),
                }],
            });

            // Load the Earth equirectangular texture.
            let tex_path = resolve_earth_texture()
                .expect("Earth.jpg not found in assets/textures/planets");
            println!("Loaded Earth texture from: {}", tex_path.display());
            let bytes = std::fs::read(&tex_path).expect("failed to read Earth.jpg");
            let img = image::load_from_memory(&bytes)
                .expect("failed to decode Earth.jpg")
                .to_rgba8();
            let (w, h) = img.dimensions();
            let view = upload_texture(device, queue, "Earth", w, h, &img);

            // Sampler: Repeat u (longitude wraps), ClampToEdge v (poles), Linear.
            let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                label: Some("earth-sampler"),
                address_mode_u: wgpu::AddressMode::Repeat,
                address_mode_v: wgpu::AddressMode::ClampToEdge,
                address_mode_w: wgpu::AddressMode::ClampToEdge,
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                mipmap_filter: wgpu::MipmapFilterMode::Nearest,
                ..Default::default()
            });

            let texture_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("earth-tex-bg"),
                layout: &tex_bgl,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Sampler(&sampler),
                    },
                ],
            });

            // -----------------------------------------------------------------
            // Atmosphere shell pipeline (blue limb glow).
            // -----------------------------------------------------------------

            // Group 0: single uniform buffer at binding 0 (VERTEX | FRAGMENT).
            let atmo_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("atmo-bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: Some(
                            std::num::NonZeroU64::new(std::mem::size_of::<AtmoUniforms>() as u64)
                                .unwrap(),
                        ),
                    },
                    count: None,
                }],
            });

            let atmo_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("atmo-layout"),
                bind_group_layouts: &[Some(&atmo_bgl)],
                immediate_size: 0,
            });

            let atmo_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("atmosphere-shader"),
                source: wgpu::ShaderSource::Wgsl(ATMO_WGSL.into()),
            });

            let atmo_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("atmo-pipeline"),
                layout: Some(&atmo_layout),
                vertex: wgpu::VertexState {
                    module: &atmo_shader,
                    entry_point: Some("vs_main"),
                    buffers: &[Vertex::desc().clone()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &atmo_shader,
                    entry_point: Some("fs_main"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: target_format,
                        // Premultiplied alpha: the shader outputs rgb * alpha.
                        blend: Some(wgpu::BlendState {
                            color: wgpu::BlendComponent {
                                src_factor: wgpu::BlendFactor::One,
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
                    cull_mode: Some(wgpu::Face::Back),
                    polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false,
                    conservative: false,
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    // Test against planet depth but do not write, so the halo
                    // blends over the planet and background.
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

            let atmo_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("atmo-uniform"),
                size: std::mem::size_of::<AtmoUniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let atmo_uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("atmo-uniform-bg"),
                layout: &atmo_bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: atmo_uniform_buffer.as_entire_binding(),
                }],
            });

            Some(SphereCallback {
                vertex_buffer,
                index_buffer,
                index_count,
                pipeline,
                uniform_buffer,
                uniform_bind_group,
                texture_bind_group,
                atmo_pipeline,
                atmo_uniform_buffer,
                atmo_uniform_bind_group,
            })
        } else {
            None
        };

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
