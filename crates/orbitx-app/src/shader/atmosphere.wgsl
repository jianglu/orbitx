// Atmosphere shell shader: a thin sphere slightly larger than the planet,
// rendered with a view-dependent limb glow (Fresnel-like rim) modulated by the
// day side. Composited over the planet + background with premultiplied alpha.

struct AtmoUniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    atmo_color: vec4<f32>,  // rgb tint, a = intensity
    light_dir: vec4<f32>,   // xyz world-space light direction (normalized)
    log_depth: vec4<f32>,   // x = C, y = far, z = 1/log2(C*far+1), w = unused
};

@group(0) @binding(0) var<uniform> u: AtmoUniforms;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) wnormal: vec3<f32>,
    @location(1) wpos: vec3<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    let world = u.model * vec4<f32>(in.pos, 1.0);
    out.wpos = world.xyz;
    out.wnormal = (u.model * vec4<f32>(in.normal, 0.0)).xyz;

    let clip = u.mvp * vec4<f32>(in.pos, 1.0);
    let c = u.log_depth.x;
    let inv_log_far = u.log_depth.z;
    let log_z = log2(c * clip.w + 1.0) * inv_log_far;
    out.clip = vec4<f32>(clip.x, clip.y, log_z * clip.w, clip.w);
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.wnormal);
    // Camera is the floating-point origin in render space, so view direction
    // toward the camera is -normalize(fragment world position).
    let view_dir = normalize(-in.wpos);

    // Fresnel-like rim: 0 at the center of the disc, 1 at the limb.
    let rim = 1.0 - max(dot(n, view_dir), 0.0);
    let glow = pow(rim, 3.0);

    // Fade toward the night side so the glow concentrates on the lit limb.
    let ld = normalize(u.light_dir.xyz);
    let day = clamp(dot(n, ld) * 0.5 + 0.5, 0.0, 1.0);

    let alpha = glow * u.atmo_color.a * day;
    // Premultiplied output (rgb already multiplied by alpha).
    return vec4<f32>(u.atmo_color.rgb * alpha, alpha);
}
