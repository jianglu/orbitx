// Planet sphere shader with logarithmic depth buffer + optional surface texture.
//
// Vertex shader: transforms UV sphere vertices with MVP, outputs log-depth + uv.
// Fragment shader: samples equirectangular texture (or base color) + diffuse lighting.

struct Uniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    base_color: vec4<f32>,
    light_dir: vec4<f32>,   // xyz = direction (normalized), w = unused
    log_depth: vec4<f32>,   // x = C, y = far, z = 1/log2(C*far+1), w = use_texture flag
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

// Surface texture (group 1): equirectangular map + sampler.
@group(1) @binding(0) var surf_tex: texture_2d<f32>;
@group(1) @binding(1) var surf_smp: sampler;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    // Model-space normal (uniform scale, so no inverse-transpose needed)
    out.world_normal = (uniforms.model * vec4<f32>(in.normal, 0.0)).xyz;

    // MVP transform
    let clip = uniforms.mvp * vec4<f32>(in.position, 1.0);

    // Logarithmic depth buffer
    // z_ndc = log2(C * w + 1) / log2(C * far + 1)
    let c = uniforms.log_depth.x;
    let inv_log_far = uniforms.log_depth.z;
    let log_z = log2(c * clip.w + 1.0) * inv_log_far;

    out.clip_pos = vec4<f32>(clip.x, clip.y, log_z * clip.w, clip.w);
    out.uv = in.uv;

    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Normalize interpolated normal
    let n = normalize(in.world_normal);
    let light_dir = normalize(uniforms.light_dir.xyz);

    // Half-lambert diffuse for softer shading
    let ndotl = dot(n, light_dir);
    let diffuse = ndotl * 0.5 + 0.5;

    let ambient = 0.15;
    let lighting = ambient + diffuse * 0.85;

    // Surface color: sampled texture when use_texture > 0.5, else base color.
    let use_texture = uniforms.log_depth.w;
    let tex_rgb = textureSample(surf_tex, surf_smp, in.uv).rgb;
    let surface = mix(uniforms.base_color.rgb, tex_rgb, step(0.5, use_texture));

    return vec4<f32>(surface * lighting, uniforms.base_color.a);
}
