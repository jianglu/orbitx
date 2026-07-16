//! Scene renderer - wgpu pipeline + egui CallbackTrait for 3D planet/star spheres.

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

#[derive(Clone)]
pub struct FrameScene {
    pub view_proj: Mat4,
    pub bodies: Vec<([f32; 3], f32, [f32; 4])>,
    pub light_dir: [f32; 3],
    pub log_depth_c: f32,
    pub log_depth_far: f32,
}

impl FrameScene {
    pub fn from_scene(camera: &CameraSystem, scene: &SceneManager) -> Self {
        let view = camera.view_matrix();
        let proj = camera.projection_matrix();
        let view_proj = proj * view;
        let mut bodies = Vec::new();
        for node in scene.nodes() {
            if !node.visible { continue; }
            let color = match &node.node_type {
                NodeType::Star => [1.0, 0.95, 0.4, 1.0],
                NodeType::Planet(ps) => ps.color,
                _ => continue,
            };
            bodies.push((node.render_data.position.into(), node.render_data.scale, color));
        }
        Self {
            view_proj, bodies,
            light_dir: [0.3, 1.0, 0.5],
            log_depth_c: camera.log_depth.constant(),
            log_depth_far: camera.log_depth.far_f32(),
        }
    }
}

#[derive(Clone)]
pub struct SceneRenderer {
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    frame_scene: Option<FrameScene>,
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

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scene-uniform"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("scene-bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: Some(std::num::NonZeroU64::new(
                        std::mem::size_of::<Uniforms>() as u64
                    ).unwrap()),
                },
                count: None,
            }],
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scene-bg"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene-pipeline-layout"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("planet-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader/planet.wgsl").into()),
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene-pipeline"),
            layout: Some(&pipeline_layout),
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

        Self {
            pipeline,
            vertex_buffer,
            index_buffer,
            index_count,
            uniform_buffer,
            bind_group,
            frame_scene: None,
        }
    }

    pub fn set_frame(&mut self, frame: FrameScene) {
        self.frame_scene = Some(frame);
    }

    fn draw_body(
        &self,
        pass: &mut wgpu::RenderPass<'_>,
        queue: &wgpu::Queue,
        view_proj: &Mat4,
        position: &[f32; 3],
        scale: f32,
        color: &[f32; 4],
        light_dir: &[f32; 3],
        log_depth_c: f32,
        log_depth_far: f32,
    ) {
        let model = Mat4::from_scale_rotation_translation(
            glam::Vec3::splat(scale),
            glam::Quat::IDENTITY,
            glam::Vec3::new(position[0], position[1], position[2]),
        );
        let mvp = *view_proj * model;
        let inv_log_far = 1.0 / (log_depth_c * log_depth_far + 1.0).log2();
        let uniforms = Uniforms {
            mvp: mvp.to_cols_array_2d(),
            model: model.to_cols_array_2d(),
            base_color: *color,
            light_dir: [light_dir[0], light_dir[1], light_dir[2], 0.0],
            log_depth: [log_depth_c, log_depth_far, inv_log_far, 0.0],
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::cast_slice(&[uniforms]));
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }
}

pub struct SceneCallback;

impl CallbackTrait for SceneCallback {
    fn paint(
        &self,
        _info: PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &CallbackResources,
    ) {
        let Some(renderer) = callback_resources.get::<SceneRenderer>() else { return };
        let Some(queue) = callback_resources.get::<wgpu::Queue>() else { return };
        let Some(frame) = &renderer.frame_scene else { return };
        for (position, scale, color) in &frame.bodies {
            renderer.draw_body(
                render_pass, queue, &frame.view_proj,
                position, *scale, color, &frame.light_dir,
                frame.log_depth_c, frame.log_depth_far,
            );
        }
    }
}
