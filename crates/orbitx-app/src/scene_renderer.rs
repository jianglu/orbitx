//! Scene renderer - wgpu pipeline + egui CallbackTrait for 3D planet/star spheres
//! and billboard fallback for distant bodies.

use egui::PaintCallbackInfo;
use egui_wgpu::{CallbackResources, CallbackTrait};
use glam::Mat4;
use orbitx_render::{CameraSystem, NodeType, SceneManager};
use wgpu::util::DeviceExt;

use crate::sphere::{self, Vertex};

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

#[derive(Clone)]
pub enum BodyDraw {
    Sphere { position: [f32; 3], scale: f32, color: [f32; 4] },
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
}

impl FrameScene {
    pub fn from_scene(camera: &CameraSystem, scene: &SceneManager, viewport_size: [f32; 2]) -> Self {
        let view = camera.view_matrix();
        let proj = camera.projection_matrix();
        let view_proj = proj * view;
        let mut draws = Vec::new();
        for node in scene.nodes() {
            if !node.visible { continue; }
            let (color, min_render_px) = match &node.node_type {
                NodeType::Star => ([1.0, 0.95, 0.4, 1.0], 8.0f32),
                NodeType::Planet(ps) => (ps.color, ps.min_render_radius),
                _ => continue,
            };
            let pos: [f32; 3] = node.render_data.position.into();
            let scale = node.render_data.scale;
            let cam_dist = node.render_data.dist_to_cam as f32;
            let is_star = matches!(&node.node_type, NodeType::Star);
            let screen_px = if cam_dist > 1.0 {
                let fov_y = camera.fov_y() as f32;
                let angular_r = scale / cam_dist;
                angular_r / fov_y * viewport_size[1]
            } else {
                min_render_px
            };
            let draw = if is_star || screen_px < min_render_px {
                let px = if is_star {
                    screen_px.max(min_render_px)
                } else {
                    min_render_px.max(screen_px).max(2.0)
                };
                BodyDraw::Billboard { position: pos, pixel_radius: px, color }
            } else {
                BodyDraw::Sphere { position: pos, scale, color }
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
        }
    }
}

#[derive(Clone)]
pub struct SceneRenderer {
    pipeline: wgpu::RenderPipeline,
    bb_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    uniform_buffer: wgpu::
Buffer,
    bind_group: wgpu::BindGroup,
    bb_uniform_buffer: wgpu::Buffer,
    bb_bind_group: wgpu::BindGroup,
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

fn make_depth_stencil(write: bool) -> wgpu::DepthStencilState {
    wgpu::DepthStencilState {
        format: wgpu::TextureFormat::Depth32Float,
        depth_write_enabled: Some(write),
        depth_compare: Some(wgpu::CompareFunction::Less),
        stencil: wgpu::StencilState::default(),
        bias: wgpu::DepthBiasState::default(),
    }
}

impl SceneRenderer {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
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
        let s_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("sphere-layout"),
            bind_group_layouts: &[Some(&s_bgl)],
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

        Self {
            pipeline, bb_pipeline,
            vertex_buffer, index_buffer, index_count,
            uniform_buffer, bind_group: s_bg,
            bb_uniform_buffer, bb_bind_group: b_bg,
            frame_scene: None,
        }
    }

    pub fn set_frame(&mut self, frame: FrameScene) {
        self.frame_scene = Some(frame);
    }

    fn draw_sphere(&self, pass: &mut wgpu::RenderPass<'_>, queue: &wgpu::Queue,
        view_proj: &Mat4, position: &[f32; 3], scale: f32, color: &[f32; 4],
        light_dir: &[f32; 3], log_depth_c: f32, log_depth_far: f32)
    {
        let model = Mat4::from_scale_rotation_translation(
            glam::Vec3::splat(scale), glam::Quat::IDENTITY, glam::Vec3::from(*position),
        );
        let mvp = *view_proj * model;
        let inv_log_far = 1.0 / (log_depth_c * log_depth_far + 1.0).log2();
        let uniforms = Uniforms {
            mvp: mvp.to_cols_array_2d(), model: model.to_cols_array_2d(),
            base_color: *color, light_dir: [light_dir[0], light_dir[1], light_dir[2], 0.0],
            log_depth: [log_depth_c, log_depth_far, inv_log_far, 0.0],
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::cast_slice(&[uniforms]));
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }

    fn draw_billboard(&self, pass: &mut wgpu::RenderPass<'_>, queue: &wgpu::Queue,
        view_proj: &Mat4, position: &[f32; 3], pixel_radius: f32,
        color: &[f32; 4], viewport_size: &[f32; 2])
    {
        let vp = view_proj.to_cols_array_2d();
        let uniforms = BillboardUniforms {
            center: [position[0], position[1], position[2], 1.0],
            color: *color,
            screen_size: [pixel_radius, viewport_size[0], viewport_size[1], 0.0],
            vp_row0: vp[0], vp_row1: vp[1], vp_row2: vp[2], vp_row3: vp[3],
        };
        queue.write_buffer(&self.bb_uniform_buffer, 0, bytemuck::cast_slice(&[uniforms]));
        pass.set_pipeline(&self.bb_pipeline);
        pass.set_bind_group(0, &self.bb_bind_group, &[]);
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
        for draw in &frame.draws {
            match draw {
                BodyDraw::Sphere { position, scale, color } => {
                    renderer.draw_sphere(render_pass, queue, &frame.view_proj,
                        position, *scale, color, &frame.light_dir,
                        frame.log_depth_c, frame.log_depth_far);
                }
                BodyDraw::Billboard { position, pixel_radius, color } => {
                    renderer.draw_billboard(render_pass, queue, &frame.view_proj,
                        position, *pixel_radius, color, &frame.viewport_size);
                }
            }
        }
    }
}
