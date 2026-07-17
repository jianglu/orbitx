//! Demo: Realistic Sun - phase-1 verification.
//!
//! Animated photosphere (granulation + limb darkening + blackbody + sunspots)
//! via `src/shader/sun.wgsl`, plus two additive corona glow shells via
//! `src/shader/corona.wgsl`. The camera auto-orbits.
//!
//! Run with:  cargo run -p orbitx-app --example demo_sun

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

const SUN_WGSL: &str = include_str!("../src/shader/sun.wgsl");
const CORONA_WGSL: &str = include_str!("../src/shader/corona.wgsl");

const LOG_DEPTH_C: f32 = 1.0;
const LOG_DEPTH_FAR: f32 = 1000.0;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct SunUniforms {
    mvp: [[f32; 4]; 4],
    model: [[f32; 4]; 4],
    params: [f32; 4],
    log_depth: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct CoronaUniforms {
    vp: [[f32; 4]; 4],
    center: [f32; 4],     // xyz world center, w = quad half-size
    color: [f32; 4],
    cam_right: [f32; 4],
    cam_up: [f32; 4],
    params: [f32; 4],     // time, inner_frac, falloff, unused
    log_depth: [f32; 4],
}

fn resolve_sun_texture() -> Option<PathBuf> {
    let bundled = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..").join("..").join("assets").join("textures").join("planets").join("Sun.jpg");
    if bundled.is_file() {
        return Some(bundled);
    }
    let cwd = PathBuf::from("assets/textures/planets/Sun.jpg");
    if cwd.is_file() {
        return Some(cwd);
    }
    None
}

fn make_texture_bgl(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("sun-tex-bgl"),
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

fn upload_texture(
    device: &wgpu::Device, queue: &wgpu::Queue, label: &str,
    width: u32, height: u32, rgba: &[u8],
) -> wgpu::TextureView {
    let size = wgpu::Extent3d { width, height, depth_or_array_layers: 1 };
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label), size, mip_level_count: 1, sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &tex, mip_level: 0, origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        rgba,
        wgpu::TexelCopyBufferLayout {
            offset: 0, bytes_per_row: Some(4 * width), rows_per_image: Some(height),
        },
        size,
    );
    tex.create_view(&wgpu::TextureViewDescriptor::default())
}

fn uniform_bgl(device: &wgpu::Device, label: &str, min_size: u64) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: Some(std::num::NonZeroU64::new(min_size).unwrap()),
            },
            count: None,
        }],
    })
}

fn compute_camera(angle: f32, aspect: f32) -> (glam::Mat4, glam::Vec3, glam::Vec3) {
    let eye = glam::Vec3::new(4.0 * angle.cos(), 0.8, 4.0 * angle.sin());
    let view = glam::Mat4::look_at_rh(eye, glam::Vec3::ZERO, glam::Vec3::Y);
    let proj = glam::Mat4::perspective_rh(std::f32::consts::FRAC_PI_3, aspect, 0.01, 1000.0);
    let forward = (-eye).normalize();
    let right = forward.cross(glam::Vec3::Y).normalize();
    let up = right.cross(forward);
    (proj * view, right, up)
}

struct SunCallback {
    sun_pipeline: wgpu::RenderPipeline,
    corona_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    sun_uniform_buffer: wgpu::Buffer,
    sun_uniform_bg: wgpu::BindGroup,
    sun_texture_bg: wgpu::BindGroup,
    corona_buffer: wgpu::Buffer,
    corona_bg: wgpu::BindGroup,
}

impl CallbackTrait for SunCallback {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        rp: &mut wgpu::RenderPass<'static>,
        _res: &egui_wgpu::CallbackResources,
    ) {
        // Sun photosphere (opaque, depth write).
        rp.set_pipeline(&self.sun_pipeline);
        rp.set_bind_group(0, &self.sun_uniform_bg, &[]);
        rp.set_bind_group(1, &self.sun_texture_bg, &[]);
        rp.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        rp.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        rp.draw_indexed(0..self.index_count, 0, 0..1);

        // Corona billboard (additive, camera-facing, no vertex buffer).
        rp.set_pipeline(&self.corona_pipeline);
        rp.set_bind_group(0, &self.corona_bg, &[]);
        rp.draw(0..6, 0..1);
    }
}

struct App {
    window: Option<Arc<Window>>,
    egui_ctx: egui::Context,
    painter: Option<egui_wgpu::winit::Painter>,
    egui_state: Option<egui_winit::State>,
    cb: Option<SunCallback>,
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
            cb: None,
            start: Instant::now(),
            running: true,
        }
    }

    fn render(&mut self) {
        let painter = match &mut self.painter { Some(p) => p, None => return };
        let egui_state = match &mut self.egui_state { Some(s) => s, None => return };
        let window = match &self.window { Some(w) => w, None => return };
        let cb = match &self.cb { Some(c) => c, None => return };

        if let Some(rs) = painter.render_state() {
            let size = window.inner_size();
            let aspect = if size.height > 0 { size.width as f32 / size.height as f32 } else { 4.0 / 3.0 };
            let t = self.start.elapsed().as_secs_f32();
            let (view_proj, cam_right, cam_up) = compute_camera(t * 0.25, aspect);
            let inv_log_far = 1.0 / (LOG_DEPTH_C * LOG_DEPTH_FAR + 1.0).log2();
            let ld = [LOG_DEPTH_C, LOG_DEPTH_FAR, inv_log_far, 0.0];

            // Sun sphere (unit).
            let model = glam::Mat4::IDENTITY;
            let mvp = view_proj * model;
            let su = SunUniforms {
                mvp: mvp.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                params: [t, 1.0, 0.0, 0.0],
                log_depth: ld,
            };
            rs.queue.write_buffer(&cb.sun_uniform_buffer, 0, bytemuck::cast_slice(&[su]));

            // Corona billboard: camera-facing quad, half-size 3.5x sun radius.
            // inner_frac = sun_radius / quad_half = 1.0 / 3.5 ~ 0.286 (rim).
            let cu = CoronaUniforms {
                vp: view_proj.to_cols_array_2d(),
                center: [0.0, 0.0, 0.0, 3.5],
                color: [1.0, 0.75, 0.45, 1.1],
                cam_right: [cam_right.x, cam_right.y, cam_right.z, 0.0],
                cam_up: [cam_up.x, cam_up.y, cam_up.z, 0.0],
                params: [t, 0.286, 2.2, 0.0],
                log_depth: ld,
            };
            rs.queue.write_buffer(&cb.corona_buffer, 0, bytemuck::cast_slice(&[cu]));
        }

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            let rect = ui.max_rect();
            let cbc = SunCallback {
                sun_pipeline: cb.sun_pipeline.clone(),
                corona_pipeline: cb.corona_pipeline.clone(),
                vertex_buffer: cb.vertex_buffer.clone(),
                index_buffer: cb.index_buffer.clone(),
                index_count: cb.index_count,
                sun_uniform_buffer: cb.sun_uniform_buffer.clone(),
                sun_uniform_bg: cb.sun_uniform_bg.clone(),
                sun_texture_bg: cb.sun_texture_bg.clone(),
                corona_buffer: cb.corona_buffer.clone(),
                corona_bg: cb.corona_bg.clone(),
            };
            ui.painter().add(egui_wgpu::Callback::new_paint_callback(rect, cbc));

            egui::CentralPanel::default()
                .frame(egui::Frame::new().inner_margin(8).fill(egui::Color32::TRANSPARENT))
                .show(ui, |ui| {
                    ui.label("Demo: Realistic Sun");
                    ui.label("Granulation + limb darkening + sunspots + corona");
                    ui.label("Surface shimmers; corona glows at the limb");
                    ui.label("Press Esc to quit.");
                });
        });

        egui_state.handle_platform_output(window, full_output.platform_output);
        let clipped = self.egui_ctx.tessellate(full_output.shapes, window.scale_factor() as f32);
        painter.paint_and_update_textures(
            egui::viewport::ViewportId::ROOT,
            window.scale_factor() as f32,
            [0.0, 0.0, 0.02, 1.0],
            &clipped,
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
            .with_title("demo_sun")
            .with_inner_size(winit::dpi::LogicalSize::new(800, 600));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        let egui_state = egui_winit::State::new(
            self.egui_ctx.clone(), egui::viewport::ViewportId::ROOT, &window, None, None, None,
        );

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
            .expect("set_window");

        let cb = if let Some(rs) = painter.render_state() {
            let device = &rs.device;
            let queue = &rs.queue;
            let fmt = rs.target_format;

            let (vertices, indices) = sphere::generate_uv_sphere(48, 32);
            let index_count = indices.len() as u32;
            let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("sphere-v"), contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("sphere-i"), contents: bytemuck::cast_slice(&indices),
                usage: wgpu::BufferUsages::INDEX,
            });

            // --- Sun pipeline (group0 uniform + group1 texture) ---
            let sun_ubgl = uniform_bgl(device, "sun-ubgl", std::mem::size_of::<SunUniforms>() as u64);
            let tex_bgl = make_texture_bgl(device);
            let sun_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("sun-layout"),
                bind_group_layouts: &[Some(&sun_ubgl), Some(&tex_bgl)],
                immediate_size: 0,
            });
            let sun_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("sun-shader"),
                source: wgpu::ShaderSource::Wgsl(SUN_WGSL.into()),
            });
            let sun_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("sun-pipeline"),
                layout: Some(&sun_layout),
                vertex: wgpu::VertexState {
                    module: &sun_shader, entry_point: Some("vs_main"),
                    buffers: &[Vertex::desc().clone()],
                    compilation_options: Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &sun_shader, entry_point: Some("fs_main"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: fmt, blend: None, write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    strip_index_format: None, front_face: wgpu::FrontFace::Ccw,
                    cull_mode: Some(wgpu::Face::Back), polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false, conservative: false,
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::Less),
                    stencil: Default::default(), bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState { count: 1, mask: !0, alpha_to_coverage_enabled: false },
                multiview_mask: None, cache: None,
            });

            // --- Corona pipeline (group0 uniform, additive, no depth write) ---
            let corona_ubgl = uniform_bgl(device, "corona-ubgl", std::mem::size_of::<CoronaUniforms>() as u64);
            let corona_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("corona-layout"),
                bind_group_layouts: &[Some(&corona_ubgl)],
                immediate_size: 0,
            });
            let corona_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("corona-shader"),
                source: wgpu::ShaderSource::Wgsl(CORONA_WGSL.into()),
            });
            let additive = wgpu::BlendState {
                color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
                alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
            };
            let corona_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("corona-pipeline"),
                layout: Some(&corona_layout),
                vertex: wgpu::VertexState {
                    module: &corona_shader, entry_point: Some("vs_main"),
                    buffers: &[],
                    compilation_options: Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &corona_shader, entry_point: Some("fs_main"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: fmt, blend: Some(additive), write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    strip_index_format: None, front_face: wgpu::FrontFace::Ccw,
                    cull_mode: Some(wgpu::Face::Back), polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false, conservative: false,
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: Some(false),
                    depth_compare: Some(wgpu::CompareFunction::Less),
                    stencil: Default::default(), bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState { count: 1, mask: !0, alpha_to_coverage_enabled: false },
                multiview_mask: None, cache: None,
            });

            // Sun uniform buffer + bind group.
            let sun_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("sun-ub"), size: std::mem::size_of::<SunUniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let sun_uniform_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("sun-ubg"), layout: &sun_ubgl,
                entries: &[wgpu::BindGroupEntry { binding: 0, resource: sun_uniform_buffer.as_entire_binding() }],
            });

            // Sun texture.
            let tex_path = resolve_sun_texture().expect("Sun.jpg not found");
            println!("Loaded Sun texture from: {}", tex_path.display());
            let bytes = std::fs::read(&tex_path).expect("read Sun.jpg");
            let img = image::load_from_memory(&bytes).expect("decode Sun.jpg").to_rgba8();
            let (w, h) = img.dimensions();
            let view = upload_texture(device, queue, "Sun", w, h, &img);
            let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                label: Some("sun-sampler"),
                address_mode_u: wgpu::AddressMode::Repeat,
                address_mode_v: wgpu::AddressMode::ClampToEdge,
                address_mode_w: wgpu::AddressMode::ClampToEdge,
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                mipmap_filter: wgpu::MipmapFilterMode::Nearest,
                ..Default::default()
            });
            let sun_texture_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("sun-tex-bg"), layout: &tex_bgl,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) },
                    wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&sampler) },
                ],
            });

            // Corona uniform buffers + bind groups.
            let mk_corona = |label: &str| {
                let buf = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some(label), size: std::mem::size_of::<CoronaUniforms>() as u64,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(label), layout: &corona_ubgl,
                    entries: &[wgpu::BindGroupEntry { binding: 0, resource: buf.as_entire_binding() }],
                });
                (buf, bg)
            };
            let (corona_buffer, corona_bg) = mk_corona("corona");

            Some(SunCallback {
                sun_pipeline, corona_pipeline,
                vertex_buffer, index_buffer, index_count,
                sun_uniform_buffer, sun_uniform_bg, sun_texture_bg,
                corona_buffer, corona_bg,
            })
        } else {
            None
        };

        self.window = Some(window);
        self.egui_state = Some(egui_state);
        self.painter = Some(painter);
        self.cb = cb;
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: winit::window::WindowId, event: WindowEvent) {
        if let (Some(egui_state), Some(window)) = (&mut self.egui_state, &self.window) {
            let _ = egui_state.on_window_event(window, &event);
        }
        match event {
            WindowEvent::CloseRequested => { self.running = false; event_loop.exit(); }
            WindowEvent::Resized(sz) => {
                if sz.width == 0 || sz.height == 0 { return; }
                if let Some(painter) = &mut self.painter {
                    if let (Some(w), Some(h)) = (NonZeroU32::new(sz.width), NonZeroU32::new(sz.height)) {
                        painter.on_window_resized(egui::viewport::ViewportId::ROOT, w, h);
                    }
                }
            }
            WindowEvent::KeyboardInput { event: KeyEvent { physical_key, state, .. }, .. } => {
                if state == ElementState::Pressed {
                    if let winit::keyboard::PhysicalKey::Code(k) = physical_key {
                        if k == winit::keyboard::KeyCode::Escape { self.running = false; event_loop.exit(); }
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
