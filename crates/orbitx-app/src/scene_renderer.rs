//! Scene renderer - wgpu pipeline + egui CallbackTrait for 3D planet/star spheres
//! and billboard fallback for distant bodies.

use egui::PaintCallbackInfo;
use egui_wgpu::{CallbackResources, CallbackTrait};
use glam::Mat4;
use orbitx_math::vec3::Vec3;
use orbitx_render::{CameraSystem, CoordinateBridge, NodeType, SceneManager};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use wgpu::util::DeviceExt;

use crate::sphere::{self, Vertex};

/// One astronomical unit in meters (sim space).
const AU_M: f64 = 1.49597870700e11;

/// Upper bound on line-list vertices (ecliptic grid + orbit rings + drop lines).
/// The circle segment counts are fixed, so this guarantees the buffer never
/// overflows; excess vertices are truncated.
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

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    mvp: [[f32; 4]; 4],
    model: [[f32; 4]; 4],
    base_color: [f32; 4],
    light_dir: [f32; 4],
    log_depth: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct BillboardUniforms {
    center: [f32; 4],
    color: [f32; 4],
    screen_size: [f32; 4],
    vp_row0: [f32; 4],
    vp_row1: [f32; 4],
    vp_row2: [f32; 4],
    vp_row3: [f32; 4],
}

/// Line vertex: position + rgba color (repr C, matches LINE_WGSL VsIn).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LineVertex {
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

#[derive(Clone)]
pub enum BodyDraw {
    Sphere { position: [f32; 3], scale: f32, color: [f32; 4], texture: Option<String> },
    Billboard { position: [f32; 3], pixel_radius: f32, color: [f32; 4] },
}

#[derive(Clone)]
pub struct FrameScene {
    pub view_proj: Mat4,
    pub draws: Vec<BodyDraw>,
    pub light_dir: [f32; 3],
    pub log_depth_c: f32,
    pub log_depth_far: f32,
    pub viewport_size: [f32; 2],
    pub line_vertices: Vec<LineVertex>,
}

impl FrameScene {
    pub fn from_scene(camera: &CameraSystem, scene: &SceneManager, viewport_size: [f32; 2]) -> Self {
        let view = camera.view_matrix();
        let proj = camera.projection_matrix();
        let view_proj = proj * view;
        let mut draws = Vec::new();
        for node in scene.nodes() {
            if !node.visible { continue; }
            let (color, _cfg_min, texture) = match &node.node_type {
                NodeType::Star => ([1.0, 0.95, 0.4, 1.0], 8.0f32, None),
                NodeType::Planet(ps) => (ps.color, ps.min_render_radius, ps.texture.clone()),
                _ => continue,
            };
            let pos: [f32; 3] = node.render_data.position.into();
            let scale = node.render_data.scale;
            let is_star = matches!(&node.node_type, NodeType::Star);

            // Projected radius in pixels. Camera is the floating-point origin
            // in render space, so distance-to-camera = |render_pos|. Both the
            // radius (`scale`) and this distance are in render units, so the
            // angular size is unit-consistent.
            let render_dist = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
            let fov_y = camera.fov_y() as f32;
            let screen_px = if render_dist > 1e-6 {
                (scale / render_dist) * viewport_size[1] / fov_y
            } else {
                0.0
            };

            // Minimum visible pixel radius so distant bodies never vanish.
            let min_visible_px = if is_star { 6.0 } else { 3.0 };

            let draw = if screen_px < min_visible_px {
                BodyDraw::Billboard {
                    position: pos,
                    pixel_radius: screen_px.max(min_visible_px),
                    color,
                }
            } else {
                BodyDraw::Sphere { position: pos, scale, color, texture }
            };
            draws.push(draw);
        }
        let sun_pos = scene.nodes().iter()
            .find(|n| matches!(&n.node_type, NodeType::Star))
            .map(|n| n.render_data.position);
        let light_dir = match sun_pos {
            Some(sp) => { let d = sp.normalize(); [d.x, d.y, d.z] }
            None => [0.3, 1.0, 0.5],
        };
        Self {
            view_proj, draws, light_dir,
            log_depth_c: camera.log_depth_constant_render(),
            log_depth_far: camera.log_depth_far_render(),
            viewport_size,
            line_vertices: Vec::new(),
        }
    }
}

#[derive(Clone)]
pub struct SceneRenderer {
    device: wgpu::Device,
    pipeline: wgpu::RenderPipeline,
    bb_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    // Bind group layouts kept so more per-draw slots can be allocated later.
    s_bgl: wgpu::BindGroupLayout,
    b_bgl: wgpu::BindGroupLayout,
    // Per-draw uniform pools: each draw gets its own buffer + bind group so
    // queued writes never clobber each other before the command buffer runs.
    sphere_slots: Vec<(wgpu::Buffer, wgpu::BindGroup)>,
    bb_slots: Vec<(wgpu::Buffer, wgpu::BindGroup)>,
    // Planet surface textures (group 1): one bind group per bundled body map,
    // plus a white 1x1 fallback for untextured bodies.
    texture_bind_groups: HashMap<String, wgpu::BindGroup>,
    white_bind_group: wgpu::BindGroup,
    // Line rendering (ecliptic grid + orbit rings + drop lines).
    line_pipeline: wgpu::RenderPipeline,
    line_vertex_buffer: wgpu::Buffer,
    line_uniform_buffer: wgpu::Buffer,
    line_bind_group: wgpu::BindGroup,
    frame_scene: Option<FrameScene>,
}

fn create_bgl_bg(device: &wgpu::Device, label: &str, buf: &wgpu::Buffer, min_size: u64)
    -> (wgpu::BindGroupLayout, wgpu::BindGroup)
{
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
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
    });
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout: &bgl,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: buf.as_entire_binding(),
        }],
    });
    (bgl, bg)
}

/// Allocate one uniform buffer + bind group of `size` bytes bound to `layout`.
/// Used to grow the per-draw pools so each draw has its own buffer.
fn make_uniform_slot(device: &wgpu::Device, layout: &wgpu::BindGroupLayout, label: &str, size: u64)
    -> (wgpu::Buffer, wgpu::BindGroup)
{
    let buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: buf.as_entire_binding(),
        }],
    });
    (buf, bg)
}

fn make_depth_stencil(write: bool) -> wgpu::DepthStencilState {
    wgpu::DepthStencilState {
        format: wgpu::TextureFormat::Depth32Float,
        depth_write_enabled: Some(write),
        depth_compare: Some(wgpu::CompareFunction::Less),
        stencil: wgpu::StencilState::default(),
        bias: wgpu::DepthBiasState::default(),
    }
}

/// Resolve the bundled planet-texture directory (`assets/textures/planets`).
/// Compile-time workspace path first (cwd-independent), then cwd-relative.
fn resolve_texture_dir() -> Option<PathBuf> {
    let bundled = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..").join("..").join("assets").join("textures").join("planets");
    if bundled.is_dir() {
        return Some(bundled);
    }
    let cwd = PathBuf::from("assets/textures/planets");
    if cwd.is_dir() {
        return Some(cwd);
    }
    None
}

/// Texture bind group layout (group 1): equirectangular map + sampler.
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
fn upload_texture(
    device: &wgpu::Device, queue: &wgpu::Queue,
    label: &str, width: u32, height: u32, rgba: &[u8],
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

/// Load all bundled planet PNGs into texture bind groups keyed by body name.
fn load_planet_textures(
    device: &wgpu::Device, queue: &wgpu::Queue,
    bgl: &wgpu::BindGroupLayout, sampler: &wgpu::Sampler,
) -> HashMap<String, wgpu::BindGroup> {
    let mut out = HashMap::new();
    let Some(dir) = resolve_texture_dir() else {
        eprintln!("Note: planet texture dir not found, using flat colors");
        return out;
    };
    let entries = match std::fs::read_dir(&dir) {
        Ok(e) => e,
        Err(_) => return out,
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let ext = path.extension().and_then(|s| s.to_str());
        if !matches!(ext, Some("png") | Some("jpg") | Some("jpeg")) {
            continue;
        }
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else { continue };
        // Prefer a higher-res map if the same body has multiple files: skip if
        // this body already loaded (dir iteration order is unspecified, so we
        // keep the first; hi-res .jpg and low-res .png never coexist per body).
        if out.contains_key(stem) {
            continue;
        }
        let bytes = match std::fs::read(&path) {
            Ok(b) => b,
            Err(_) => continue,
        };
        let img = match image::load_from_memory(&bytes) {
            Ok(i) => i.to_rgba8(),
            Err(e) => { eprintln!("Note: failed to decode {}: {e}", path.display()); continue; }
        };
        let (w, h) = img.dimensions();
        let view = upload_texture(device, queue, stem, w, h, &img);
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(stem),
            layout: bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(sampler) },
            ],
        });
        out.insert(stem.to_string(), bg);
    }
    out
}

impl SceneRenderer {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, target_format: wgpu::TextureFormat) -> Self {
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

        // Sphere pipeline
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scene-uniform"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let (s_bgl, s_bg) = create_bgl_bg(device, "sphere-bgl", &uniform_buffer,
            std::mem::size_of::<Uniforms>() as u64);

        // Texture group (group 1): equirectangular map + sampler, shared sampler.
        let tex_bgl = make_texture_bgl(device);
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("planet-sampler"),
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Nearest,
            ..Default::default()
        });
        let texture_bind_groups = load_planet_textures(device, queue, &tex_bgl, &sampler);
        // 1x1 white fallback for untextured bodies.
        let white_view = upload_texture(device, queue, "white", 1, 1, &[255, 255, 255, 255]);
        let white_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("white-tex"),
            layout: &tex_bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&white_view) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&sampler) },
            ],
        });

        let s_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("sphere-layout"),
            bind_group_layouts: &[Some(&s_bgl), Some(&tex_bgl)],
            immediate_size: 0,
        });
        let s_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("planet-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader/planet.wgsl").into()),
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("sphere-pipeline"),
            layout: Some(&s_layout),
            vertex: wgpu::VertexState {
                module: &s_shader, entry_point: Some("vs_main"),
                buffers: &[Vertex::desc().clone()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &s_shader, entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None, front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back), polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false, conservative: false,
            },
            depth_stencil: Some(make_depth_stencil(true)),
            multisample: wgpu::MultisampleState { count: 1, mask: !0, alpha_to_coverage_enabled: false },
            multiview_mask: None, cache: None,
        });

        // Billboard pipeline
        let bb_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("bb-uniform"),
            size: std::mem::size_of::<BillboardUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let (b_bgl, b_bg) = create_bgl_bg(device, "bb-bgl", &bb_uniform_buffer,
            std::mem::size_of::<BillboardUniforms>() as u64);
        let b_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("bb-layout"),
            bind_group_layouts: &[Some(&b_bgl)],
            immediate_size: 0,
        });
        let b_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("billboard-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader/billboard.wgsl").into()),
        });
        let bb_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("bb-pipeline"),
            layout: Some(&b_layout),
            vertex: wgpu::VertexState {
                module: &b_shader, entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &b_shader, entry_point: Some("fs_main"),
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
            depth_stencil: Some(make_depth_stencil(false)),
            multisample: wgpu::MultisampleState { count: 1, mask: !0, alpha_to_coverage_enabled: false },
            multiview_mask: None, cache: None,
        });

        // Line pipeline (ecliptic grid + orbit rings + drop lines)
        let line_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("line-bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: Some(
                        std::num::NonZeroU64::new(std::mem::size_of::<LineUniforms>() as u64).unwrap(),
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
        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("line-pipeline"),
            layout: Some(&line_layout),
            vertex: wgpu::VertexState {
                module: &line_shader, entry_point: Some("vs_main"),
                buffers: &[LineVertex::desc()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &line_shader, entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None, front_face: wgpu::FrontFace::Ccw,
                cull_mode: None, polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false, conservative: false,
            },
            depth_stencil: Some(make_depth_stencil(true)),
            multisample: wgpu::MultisampleState { count: 1, mask: !0, alpha_to_coverage_enabled: false },
            multiview_mask: None, cache: None,
        });
        let line_vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("line-vertices"),
            size: (LINE_VERTEX_CAPACITY * std::mem::size_of::<LineVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            device: device.clone(),
            pipeline, bb_pipeline,
            vertex_buffer, index_buffer, index_count,
            s_bgl, b_bgl,
            sphere_slots: vec![(uniform_buffer, s_bg)],
            bb_slots: vec![(bb_uniform_buffer, b_bg)],
            texture_bind_groups,
            white_bind_group,
            line_pipeline, line_vertex_buffer, line_uniform_buffer, line_bind_group,
            frame_scene: None,
        }
    }

    pub fn set_frame(&mut self, frame: FrameScene) {
        // Grow the per-draw uniform pools so each draw this frame owns a
        // distinct buffer + bind group. This is the fix for the single-buffer
        // bug: queued write_buffer calls all flush before the command buffer
        // runs, so sharing one buffer made every draw read the last write.
        let need = frame.draws.len();
        while self.sphere_slots.len() < need {
            let i = self.sphere_slots.len();
            self.sphere_slots.push(make_uniform_slot(
                &self.device, &self.s_bgl, &format!("sphere-ub-{i}"),
                std::mem::size_of::<Uniforms>() as u64,
            ));
        }
        while self.bb_slots.len() < need {
            let i = self.bb_slots.len();
            self.bb_slots.push(make_uniform_slot(
                &self.device, &self.b_bgl, &format!("bb-ub-{i}"),
                std::mem::size_of::<BillboardUniforms>() as u64,
            ));
        }
        self.frame_scene = Some(frame);
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_sphere(&self, pass: &mut wgpu::RenderPass<'_>, queue: &wgpu::Queue,
        slot: usize, view_proj: &Mat4, position: &[f32; 3], scale: f32, color: &[f32; 4],
        light_dir: &[f32; 3], log_depth_c: f32, log_depth_far: f32, texture: &Option<String>)
    {
        let model = Mat4::from_scale_rotation_translation(
            glam::Vec3::splat(scale), glam::Quat::IDENTITY, glam::Vec3::from(*position),
        );
        let mvp = *view_proj * model;
        let inv_log_far = 1.0 / (log_depth_c * log_depth_far + 1.0).log2();
        // Pick texture bind group; log_depth.w carries the use_texture flag.
        let (tex_bg, use_texture) = match texture.as_ref().and_then(|k| self.texture_bind_groups.get(k)) {
            Some(bg) => (bg, 1.0f32),
            None => (&self.white_bind_group, 0.0f32),
        };
        let uniforms = Uniforms {
            mvp: mvp.to_cols_array_2d(), model: model.to_cols_array_2d(),
            base_color: *color, light_dir: [light_dir[0], light_dir[1], light_dir[2], 0.0],
            log_depth: [log_depth_c, log_depth_far, inv_log_far, use_texture],
        };
        let (buf, bg) = &self.sphere_slots[slot];
        queue.write_buffer(buf, 0, bytemuck::cast_slice(&[uniforms]));
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, bg, &[]);
        pass.set_bind_group(1, tex_bg, &[]);
        pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }

    fn draw_billboard(&self, pass: &mut wgpu::RenderPass<'_>, queue: &wgpu::Queue,
        slot: usize, view_proj: &Mat4, position: &[f32; 3], pixel_radius: f32,
        color: &[f32; 4], viewport_size: &[f32; 2])
    {
        let vp = view_proj.to_cols_array_2d();
        let uniforms = BillboardUniforms {
            center: [position[0], position[1], position[2], 1.0],
            color: *color,
            screen_size: [pixel_radius, viewport_size[0], viewport_size[1], 0.0],
            vp_row0: vp[0], vp_row1: vp[1], vp_row2: vp[2], vp_row3: vp[3],
        };
        let (buf, bg) = &self.bb_slots[slot];
        queue.write_buffer(buf, 0, bytemuck::cast_slice(&[uniforms]));
        pass.set_pipeline(&self.bb_pipeline);
        pass.set_bind_group(0, bg, &[]);
        pass.draw(0..6, 0..1);
    }
}

pub struct SceneCallback;

impl CallbackTrait for SceneCallback {
    fn paint(&self, _info: PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &CallbackResources)
    {
        let Some(renderer) = callback_resources.get::<SceneRenderer>() else { return };
        let Some(queue) = callback_resources.get::<wgpu::Queue>() else { return };
        let Some(frame) = &renderer.frame_scene else { return };

        // Draw lines first (ecliptic grid + orbit rings + drop lines); they
        // write depth so spheres behind them occlude correctly.
        let line_count = frame.line_vertices.len().min(LINE_VERTEX_CAPACITY);
        if line_count > 0 {
            let inv_log_far = 1.0 / (frame.log_depth_c * frame.log_depth_far + 1.0).log2();
            let lu = LineUniforms {
                view_proj: frame.view_proj.to_cols_array_2d(),
                log_depth: [frame.log_depth_c, frame.log_depth_far, inv_log_far, 0.0],
            };
            queue.write_buffer(&renderer.line_uniform_buffer, 0, bytemuck::cast_slice(&[lu]));
            queue.write_buffer(&renderer.line_vertex_buffer, 0,
                bytemuck::cast_slice(&frame.line_vertices[..line_count]));
            render_pass.set_pipeline(&renderer.line_pipeline);
            render_pass.set_bind_group(0, &renderer.line_bind_group, &[]);
            render_pass.set_vertex_buffer(0, renderer.line_vertex_buffer.slice(..));
            render_pass.draw(0..line_count as u32, 0..1);
        }

        // Each draw uses its own uniform slot so queued writes never collide.
        let mut si = 0usize;
        let mut bi = 0usize;
        for draw in &frame.draws {
            match draw {
                BodyDraw::Sphere { position, scale, color, texture } => {
                    renderer.draw_sphere(render_pass, queue, si, &frame.view_proj,
                        position, *scale, color, &frame.light_dir,
                        frame.log_depth_c, frame.log_depth_far, texture);
                    si += 1;
                }
                BodyDraw::Billboard { position, pixel_radius, color } => {
                    renderer.draw_billboard(render_pass, queue, bi, &frame.view_proj,
                        position, *pixel_radius, color, &frame.viewport_size);
                    bi += 1;
                }
            }
        }
    }
}

/// Build render-space line vertices for the ecliptic grid, per-planet orbit
/// rings, and drop lines. Geometry is generated in sim space (meters) and each
/// point is converted to render space via `coord.to_render`. Emitted as
/// LineList pairs; truncated to `LINE_VERTEX_CAPACITY`.
pub fn build_scene_lines(scene: &SceneManager, coord: &CoordinateBridge) -> Vec<LineVertex> {
    let mut sim: Vec<(Vec3, [f32; 4])> = Vec::new();
    let seg = 128usize;

    // Ecliptic grid: concentric circles in the y=0 plane, centered at origin.
    let grid_color = [0.3f32, 0.35, 0.5, 0.45];
    let mut r = 0.5f64;
    while r <= 3.0001 {
        let radius = r * AU_M;
        for i in 0..seg {
            let a0 = (i as f64) / (seg as f64) * std::f64::consts::TAU;
            let a1 = ((i + 1) as f64) / (seg as f64) * std::f64::consts::TAU;
            sim.push((Vec3::new(radius * a0.cos(), 0.0, radius * a0.sin()), grid_color));
            sim.push((Vec3::new(radius * a1.cos(), 0.0, radius * a1.sin()), grid_color));
        }
        r += 0.5;
    }

    // Radial spokes every 30 degrees out to 3 AU.
    let rmax = 3.0 * AU_M;
    for k in 0..12 {
        let a = (k as f64) / 12.0 * std::f64::consts::TAU;
        sim.push((Vec3::new(0.0, 0.0, 0.0), grid_color));
        sim.push((Vec3::new(rmax * a.cos(), 0.0, rmax * a.sin()), grid_color));
    }

    // Orbit rings + drop lines: one per planet node at its heliocentric distance.
    for node in scene.nodes() {
        if let NodeType::Planet(ps) = &node.node_type {
            let radius = node.transform.position.length();
            let color = [ps.color[0], ps.color[1], ps.color[2], 0.8];
            for j in 0..seg {
                let a0 = (j as f64) / (seg as f64) * std::f64::consts::TAU;
                let a1 = ((j + 1) as f64) / (seg as f64) * std::f64::consts::TAU;
                sim.push((Vec3::new(radius * a0.cos(), 0.0, radius * a0.sin()), color));
                sim.push((Vec3::new(radius * a1.cos(), 0.0, radius * a1.sin()), color));
            }

            // Drop line: from the body to its ecliptic-plane projection.
            let p = node.transform.position;
            let foot = Vec3::new(p.x, 0.0, p.z);
            let drop_color = [ps.color[0], ps.color[1], ps.color[2], 0.55];
            sim.push((p, drop_color));
            sim.push((foot, drop_color));
        }
    }

    sim.truncate(LINE_VERTEX_CAPACITY);
    sim.into_iter()
        .map(|(p, c)| {
            let render = coord.to_render(&p);
            LineVertex { pos: [render.x, render.y, render.z], color: c }
        })
        .collect()
}
