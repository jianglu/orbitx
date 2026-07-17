// Planetary ring shader: flat annulus in the planet's equatorial plane,
// sampling a radial ring-profile texture (u = inner..outer, alpha encodes gaps).
// Rendered double-sided, alpha-blended.

struct RingUniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    light_dir: vec4<f32>,   // xyz world-space light direction
    log_depth: vec4<f32>,   // x = C, y = far, z = 1/log2(C*far+1), w = unused
};

@group(0) @binding(0) var<uniform> u: RingUniforms;
@group(1) @binding(0) var ring_tex: texture_2d<f32>;
@group(1) @binding(1) var ring_smp: sampler;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
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
    let texel = textureSample(ring_tex, ring_smp, in.uv);
    // texel.a encodes ring density / gaps; discard nearly-empty regions.
    if (texel.a < 0.02) {
        discard;
    }
    // Straight (non-premultiplied) alpha blending.
    return vec4<f32>(texel.rgb, texel.a);
}
