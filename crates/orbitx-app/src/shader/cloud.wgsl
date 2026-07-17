// Cloud shell shader: a thin sphere just above the planet surface, sampling an
// equirectangular cloud map whose luminance is used as opacity (white = dense
// cloud, black = clear). Lit by the day side, alpha-blended over the surface.

struct CloudUniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    light_dir: vec4<f32>,   // xyz world-space light direction
    log_depth: vec4<f32>,   // x = C, y = far, z = 1/log2(C*far+1), w = opacity scale
};

@group(0) @binding(0) var<uniform> u: CloudUniforms;
@group(1) @binding(0) var cloud_tex: texture_2d<f32>;
@group(1) @binding(1) var cloud_smp: sampler;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) wnormal: vec3<f32>,
    @location(1) uv: vec2<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    out.wnormal = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
    let clip = u.mvp * vec4<f32>(in.pos, 1.0);
    let c = u.log_depth.x;
    let inv_log_far = u.log_depth.z;
    let log_z = log2(c * clip.w + 1.0) * inv_log_far;
    out.clip = vec4<f32>(clip.x, clip.y, log_z * clip.w, clip.w);
    out.uv = in.uv;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let texel = textureSample(cloud_tex, cloud_smp, in.uv).rgb;
    // Luminance as cloud opacity.
    let lum = dot(texel, vec3<f32>(0.299, 0.587, 0.114));
    let opacity = clamp(lum * u.log_depth.w, 0.0, 1.0);
    if (opacity < 0.02) {
        discard;
    }
    // Day-side lighting (soft terminator).
    let n = normalize(in.wnormal);
    let ld = normalize(u.light_dir.xyz);
    let day = clamp(dot(n, ld) * 0.5 + 0.5, 0.0, 1.0);
    let lit = 0.2 + 0.8 * day;
    return vec4<f32>(vec3<f32>(lit), opacity);
}
