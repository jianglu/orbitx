// Planet sphere shader with logarithmic depth buffer.
//
// Vertex shader: transforms UV sphere vertices with MVP, outputs log-depth.
// Fragment shader: simple diffuse lighting + base color.

struct Uniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    base_color: vec4<f32>,
    light_dir: vec4<f32>,   // xyz = direction (normalized), w = unused
    log_depth: vec4<f32>,   // x = C constant, y = far, z = 1/log2(C*far+1), w = unused
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
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

    return vec4<f32>(uniforms.base_color.rgb * lighting, uniforms.base_color.a);
}
