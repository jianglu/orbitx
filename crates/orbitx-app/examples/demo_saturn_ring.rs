//! Demo: Saturn Ring - P3B-3 verification.
//!
//! Verifies planetary ring rendering: a textured Saturn sphere plus a flat
//! ring annulus that samples a radial ring-profile texture (alpha encodes
//! gaps). Both the sphere and the ring share a fixed ~26 degree tilt so the
//! ring sits in the planet's equatorial plane and is not edge-on. The camera
//! auto-orbits far enough out that both the planet and ring stay in frame.
//!
//! Two pipelines:
//!   - Planet: `src/shader/planet.wgsl`, group 0 = uniforms, group 1 = surface
//!     texture + sampler. Writes depth.
//!   - Ring: `src/shader/ring.wgsl`, group 0 = uniforms, group 1 = ring texture
//!     + sampler. Double-sided, alpha-blended, depth-tested but no depth write.
//!
//! Run with:  cargo run -p orbitx-app --example demo_saturn_ring

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
// WGSL - reuse the production textured planet + ring shaders via include_str!
// ---------------------------------------------------------------------------

const PLANET_WGSL: &str = include_str!("../src/shader/planet.wgsl");
const RING_WGSL: &str = include_str!("../src/shader/ring.wgsl");

// ---------------------------------------------------------------------------
// Uniform structs - must match the WGSL structs exactly.
// ---------------------------------------------------------------------------

/// Planet uniforms - matches planet.wgsl Uniforms (176 bytes).
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    mvp: [[f32; 4]; 4],   // 64 bytes
    model: [[f32; 4]; 4], // 64 bytes
    base_color: [f32; 4], // 16 bytes
    light_dir: [f32; 4],  // 16 bytes
    log_depth: [f32; 4],  // 16 bytes: [C=1.0, far, inv_log_far, use_texture=1.0]
}

/// Ring uniforms - matches ring.wgsl RingUniforms (144 bytes).
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct RingUniforms {
    mvp: [[f32; 4]; 4],   // 64 bytes
    model: [[f32; 4]; 4], // 64 bytes
    light_dir: [f32; 4],  // 16 bytes
    log_depth: [f32; 4],  // 16 bytes: [C=1.0, far, inv_log_far, unused]
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const LIGHT_DIR: [f32; 4] = [0.5, 0.4, 0.6, 0.0]; // normalized in shader

const LOG_DEPTH_C: f32 = 1.0;
const LOG_DEPTH_FAR: f32 = 1000.0;

/// Saturn's axial tilt (radians) applied about the Z axis.
const TILT_RADIANS: f32 = 26.0 * std::f32::consts::PI / 180.0;

/// Resolve a planet texture path by file name. Compile-time workspace path
/// first (cwd-independent), then cwd-relative fallback. Mirrors
/// scene_renderer.rs::resolve_texture_dir().
fn resolve_planet_texture(file_name: &str) -> Option<PathBuf> {
    let bundled = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("assets")
        .join("textures")
        .join("planets")
        .join(file_name);
    if bundled.is_file() {
        return Some(bundled);
    }
    let cwd = PathBuf::from(format!("assets/textures/planets/{file_name}"));
    if cwd.is_file() {
        return Some(cwd);
    }
    None
}

/// Texture bind group layout (group 1): 2D texture + sampler.
/// Copied from scene_renderer.rs::make_texture_bgl. Shared by both pipelines.
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

/// Camera orbiting the planet; angle drives a slow rotation over time. The
/// orbit distance (~5.0) keeps both the planet and its ring in frame.
fn compute_view_proj(angle: f32, aspect: f32) -> glam::Mat4 {
    let eye = glam::Vec3::new(5.0 * angle.cos(), 1.5, 5.0 * angle.sin());
    let view = glam::Mat4::look_at_rh(eye, glam::Vec3::ZERO, glam::Vec3::Y);
    let proj = glam::Mat4::perspective_rh(std::f32::consts::FRAC_PI_3, aspect, 0.01, 1000.0);
    proj * view
}

// ---------------------------------------------------------------------------
// Callback - draws the textured Saturn sphere, then the alpha-blended ring.
// ---------------------------------------------------------------------------

struct SaturnCallback {
    // Planet
    planet_pipeline: wgpu::RenderPipeline,
    planet_vertex_buffer: wgpu::Buffer,
    planet_index_buffer: wgpu::Buffer,
    planet_index_count: u32,
    planet_uniform_buffer: wgpu::Buffer,
    planet_uniform_bind_group: wgpu::BindGroup,
    planet_texture_bind_group: wgpu::BindGroup,
    // Ring
    ring_pipeline: wgpu::RenderPipeline,
    ring_vertex_buffer: wgpu::Buffer,
    ring_index_buffer: wgpu::Buffer,
    ring_index_count: u32,
    ring_uniform_buffer: wgpu::Buffer,
    ring_uniform_bind_group: wgpu::BindGroup,
    ring_texture_bind_group: wgpu::BindGroup,
}

impl CallbackTrait for SaturnCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        // Planet first - writes depth.
        render_pass.set_pipeline(&self.planet_pipeline);
        render_pass.set_bind_group(0, &self.planet_uniform_bind_group, &[]);
        render_pass.set_bind_group(1, &self.planet_texture_bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.planet_vertex_buffer.slice(..));
        render_pass
            .set_index_buffer(self.planet_index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        render_pass.draw_indexed(0..self.planet_index_count, 0, 0..1);

        // Ring second - alpha-blended, depth-tested, no depth write.
        render_pass.set_pipeline(&self.ring_pipeline);
        render_pass.set_bind_group(0, &self.ring_uniform_bind_group, &[]);
        render_pass.set_bind_group(1, &self.ring_texture_bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.ring_vertex_buffer.slice(..));
        render_pass
            .set_index_buffer(self.ring_index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        render_pass.draw_indexed(0..self.ring_index_count, 0, 0..1);
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
    saturn_callback: Option<SaturnCallback>,
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
            saturn_callback: None,
            start: Instant::now(),
            running: true,
        }
    }

    fn render(&mut self) {
        let painter = match &mut self.painter { Some(p) => p, None => return };
        let egui_state = match &mut self.egui_state { Some(s) => s, None => return };
        let window = match &self.window { Some(w) => w, None => return };
        let callback = match &self.saturn_callback { Some(c) => c, None => return };

        // Update both uniform buffers each frame on the main thread: the camera
        // orbits over time so mvp changes every frame. Texture bind groups are
        // static (built once).
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

            // Fixed tilt about Z, shared by planet and ring so the ring sits in
            // the planet's equatorial plane.
            let tilt = glam::Mat4::from_rotation_z(TILT_RADIANS);
            let model_planet = tilt * glam::Mat4::from_scale(glam::Vec3::splat(1.0));
            let model_ring = tilt;
            let mvp_planet = view_proj * model_planet;
            let mvp_ring = view_proj * model_ring;

            let inv_log_far = 1.0 / (LOG_DEPTH_C * LOG_DEPTH_FAR + 1.0).log2();

            let planet_uniforms = Uniforms {
                mvp: mvp_planet.to_cols_array_2d(),
                model: model_planet.to_cols_array_2d(),
                base_color: [1.0, 1.0, 1.0, 1.0],
                light_dir: LIGHT_DIR,
                // Last component (use_texture) = 1.0 so the shader samples the texture.
                log_depth: [LOG_DEPTH_C, LOG_DEPTH_FAR, inv_log_far, 1.0],
            };
            rs.queue.write_buffer(
                &callback.planet_uniform_buffer,
                0,
                bytemuck::cast_slice(&[planet_uniforms]),
            );

            let ring_uniforms = RingUniforms {
                mvp: mvp_ring.to_cols_array_2d(),
                model: model_ring.to_cols_array_2d(),
                light_dir: LIGHT_DIR,
                log_depth: [LOG_DEPTH_C, LOG_DEPTH_FAR, inv_log_far, 0.0],
            };
            rs.queue.write_buffer(
                &callback.ring_uniform_buffer,
                0,
                bytemuck::cast_slice(&[ring_uniforms]),
            );
        }

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            let rect = ui.max_rect();
            let cb = egui_wgpu::Callback::new_paint_callback(
                rect,
                SaturnCallback {
                    planet_pipeline: callback.planet_pipeline.clone(),
                    planet_vertex_buffer: callback.planet_vertex_buffer.clone(),
                    planet_index_buffer: callback.planet_index_buffer.clone(),
                    planet_index_count: callback.planet_index_count,
                    planet_uniform_buffer: callback.planet_uniform_buffer.clone(),
                    planet_uniform_bind_group: callback.planet_uniform_bind_group.clone(),
                    planet_texture_bind_group: callback.planet_texture_bind_group.clone(),
                    ring_pipeline: callback.ring_pipeline.clone(),
                    ring_vertex_buffer: callback.ring_vertex_buffer.clone(),
                    ring_index_buffer: callback.ring_index_buffer.clone(),
                    ring_index_count: callback.ring_index_count,
                    ring_uniform_buffer: callback.ring_uniform_buffer.clone(),
                    ring_uniform_bind_group: callback.ring_uniform_bind_group.clone(),
                    ring_texture_bind_group: callback.ring_texture_bind_group.clone(),
                },
            );
            ui.painter().add(cb);

            egui::CentralPanel::default()
                .frame(egui::Frame::new().inner_margin(8).fill(egui::Color32::TRANSPARENT))
                .show(ui, |ui| {
                    ui.label("Demo: Saturn Ring");
                    ui.label("Textured planet + radial ring texture (alpha gaps)");
                    ui.label("Ring tilted ~26deg, double-sided");
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
            .with_title("demo_saturn_ring")
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

        let saturn_callback = if let Some(rs) = painter.render_state() {
            let device = &rs.device;
            let queue = &rs.queue;
            let target_format = rs.target_format;

            // --- Shared layouts -------------------------------------------------

            // Group 1: texture + sampler (FRAGMENT). Shared by both pipelines.
            let tex_bgl = make_texture_bgl(device);

            // Shared sampler: Repeat u (longitude wraps / radial), ClampToEdge v.
            let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                label: Some("saturn-sampler"),
                address_mode_u: wgpu::AddressMode::Repeat,
                address_mode_v: wgpu::AddressMode::ClampToEdge,
                address_mode_w: wgpu::AddressMode::ClampToEdge,
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                mipmap_filter: wgpu::MipmapFilterMode::Nearest,
                ..Default::default()
            });

            // --- Planet pipeline ------------------------------------------------

            let (vertices, indices) = sphere::generate_uv_sphere(24, 16);
            let planet_index_count = indices.len() as u32;

            let planet_vertex_buffer =
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("saturn-vertices"),
                    contents: bytemuck::cast_slice(&vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });
            let planet_index_buffer =
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("saturn-indices"),
                    contents: bytemuck::cast_slice(&indices),
                    usage: wgpu::BufferUsages::INDEX,
                });

            // Group 0: single uniform buffer at binding 0 (VERTEX | FRAGMENT).
            let planet_uniform_bgl =
                device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("saturn-uniform-bgl"),
                    entries: &[wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: Some(
                                std::num::NonZeroU64::new(std::mem::size_of::<Uniforms>() as u64)
                                    .unwrap(),
                            ),
                        },
                        count: None,
                    }],
                });

            let planet_layout =
                device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("saturn-planet-layout"),
                    bind_group_layouts: &[Some(&planet_uniform_bgl), Some(&tex_bgl)],
                    immediate_size: 0,
                });

            let planet_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("planet-shader"),
                source: wgpu::ShaderSource::Wgsl(PLANET_WGSL.into()),
            });

            let planet_pipeline =
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("saturn-planet-pipeline"),
                    layout: Some(&planet_layout),
                    vertex: wgpu::VertexState {
                        module: &planet_shader,
                        entry_point: Some("vs_main"),
                        buffers: &[Vertex::desc().clone()],
                        compilation_options: wgpu::PipelineCompilationOptions::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &planet_shader,
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

            let planet_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("saturn-uniform"),
                size: std::mem::size_of::<Uniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let planet_uniform_bind_group =
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("saturn-uniform-bg"),
                    layout: &planet_uniform_bgl,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: planet_uniform_buffer.as_entire_binding(),
                    }],
                });

            // Load the Saturn equirectangular surface texture.
            let saturn_path = resolve_planet_texture("Saturn.jpg")
                .expect("Saturn.jpg not found in assets/textures/planets");
            println!("Loaded Saturn texture from: {}", saturn_path.display());
            let bytes = std::fs::read(&saturn_path).expect("failed to read Saturn.jpg");
            let img = image::load_from_memory(&bytes)
                .expect("failed to decode Saturn.jpg")
                .to_rgba8();
            let (w, h) = img.dimensions();
            let planet_view = upload_texture(device, queue, "Saturn", w, h, &img);

            let planet_texture_bind_group =
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("saturn-tex-bg"),
                    layout: &tex_bgl,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: wgpu::BindingResource::TextureView(&planet_view),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: wgpu::BindingResource::Sampler(&sampler),
                        },
                    ],
                });

            // --- Ring pipeline --------------------------------------------------

            // Ring geometry spans 1.2..2.3 body radii (unit sphere has radius 1.0).
            let (ring_vertices, ring_indices) = sphere::generate_ring(1.2, 2.3, 128);
            let ring_index_count = ring_indices.len() as u32;

            let ring_vertex_buffer =
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("ring-vertices"),
                    contents: bytemuck::cast_slice(&ring_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });
            let ring_index_buffer =
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("ring-indices"),
                    contents: bytemuck::cast_slice(&ring_indices),
                    usage: wgpu::BufferUsages::INDEX,
                });

            // Group 0: ring uniform buffer at binding 0 (VERTEX | FRAGMENT).
            let ring_uniform_bgl =
                device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("ring-uniform-bgl"),
                    entries: &[wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: Some(
                                std::num::NonZeroU64::new(
                                    std::mem::size_of::<RingUniforms>() as u64,
                                )
                                .unwrap(),
                            ),
                        },
                        count: None,
                    }],
                });

            let ring_layout =
                device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("ring-layout"),
                    bind_group_layouts: &[Some(&ring_uniform_bgl), Some(&tex_bgl)],
                    immediate_size: 0,
                });

            let ring_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("ring-shader"),
                source: wgpu::ShaderSource::Wgsl(RING_WGSL.into()),
            });

            let ring_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("ring-pipeline"),
                layout: Some(&ring_layout),
                vertex: wgpu::VertexState {
                    module: &ring_shader,
                    entry_point: Some("vs_main"),
                    buffers: &[Vertex::desc().clone()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &ring_shader,
                    entry_point: Some("fs_main"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: target_format,
                        // Standard straight (non-premultiplied) alpha blending.
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
                    // Double-sided: visible from above and below the ring plane.
                    cull_mode: None,
                    polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false,
                    conservative: false,
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    // Test against the planet depth, but do not write depth.
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

            let ring_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("ring-uniform"),
                size: std::mem::size_of::<RingUniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let ring_uniform_bind_group =
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("ring-uniform-bg"),
                    layout: &ring_uniform_bgl,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: ring_uniform_buffer.as_entire_binding(),
                    }],
                });

            // Load the Saturn ring radial-profile texture (RGBA, alpha = gaps).
            let ring_path = resolve_planet_texture("Saturn_ring.png")
                .expect("Saturn_ring.png not found in assets/textures/planets");
            println!("Loaded Saturn ring texture from: {}", ring_path.display());
            let ring_bytes = std::fs::read(&ring_path).expect("failed to read Saturn_ring.png");
            let ring_img = image::load_from_memory(&ring_bytes)
                .expect("failed to decode Saturn_ring.png")
                .to_rgba8();
            let (rw, rh) = ring_img.dimensions();
            let ring_view = upload_texture(device, queue, "Saturn_ring", rw, rh, &ring_img);

            let ring_texture_bind_group =
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("ring-tex-bg"),
                    layout: &tex_bgl,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: wgpu::BindingResource::TextureView(&ring_view),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: wgpu::BindingResource::Sampler(&sampler),
                        },
                    ],
                });

            Some(SaturnCallback {
                planet_pipeline,
                planet_vertex_buffer,
                planet_index_buffer,
                planet_index_count,
                planet_uniform_buffer,
                planet_uniform_bind_group,
                planet_texture_bind_group,
                ring_pipeline,
                ring_vertex_buffer,
                ring_index_buffer,
                ring_index_count,
                ring_uniform_buffer,
                ring_uniform_bind_group,
                ring_texture_bind_group,
            })
        } else {
            None
        };

        self.window = Some(window);
        self.egui_state = Some(egui_state);
        self.painter = Some(painter);
        self.saturn_callback = saturn_callback;
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
