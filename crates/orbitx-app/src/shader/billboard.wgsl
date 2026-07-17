// Billboard shader for distant-body fallback rendering.
// Renders a camera-facing disc/glow for bodies that are sub-pixel at true scale.
//
// Depth: emits the same logarithmic depth as planet.wgsl so billboards sort
// correctly against sphere/mesh draws. Previously the linear NDC depth here
// clashed with log-depth on planets: at any large clip.w the billboard's
// ndc.z was ~1.0 while a planet's log ndc.z stayed near 0.5, so the billboard
// always lost the depth test and was clipped by e.g. Earth.
//
// Two currently-unused uniform slots carry log-depth parameters:
//   center.w      = inv_log_far  = 1 / log2(C * far + 1)
//   screen_size.w = log_depth_c  = C

struct Uniforms {
    center: vec4f,       // xyz = world position, w = inv_log_far
    color: vec4f,        // rgba
    screen_size: vec4f,  // x = pixel radius, y = vp_w, z = vp_h, w = log_depth_c
    vp_row0: vec4f,
    vp_row1: vec4f,
    vp_row2: vec4f,
    vp_row3: vec4f,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) pos: vec4f,
    @location(0) uv: vec2f,
};

@vertex
fn vs_main(@builtin(vertex_index) vid: u32) -> VsOut {
    var positions = array<vec2f, 6>(
        vec2f(-1.0, -1.0),
        vec2f( 1.0, -1.0),
        vec2f(-1.0,  1.0),
        vec2f(-1.0,  1.0),
        vec2f( 1.0, -1.0),
        vec2f( 1.0,  1.0),
    );

    let quad_pos = positions[vid];
    let uv = quad_pos * 0.5 + 0.5;

    let vp = mat4x4f(u.vp_row0, u.vp_row1, u.vp_row2, u.vp_row3);
    let center_clip = vp * vec4f(u.center.xyz, 1.0);
    let w = center_clip.w;

    let pixel_r = u.screen_size.x;
    let offset_x = quad_pos.x * pixel_r * 2.0 / u.screen_size.y;
    let offset_y = quad_pos.y * pixel_r * 2.0 / u.screen_size.z;

    // Log-depth (matches planet.wgsl): z_ndc = log2(C * w + 1) * inv_log_far.
    let c = u.screen_size.w;
    let inv_log_far = u.center.w;
    let log_z = log2(c * w + 1.0) * inv_log_far;

    var out: VsOut;
    // Build (x, y, log_z, 1) then multiply by w so after auto perspective-divide
    // we get (ndc.xy, log_z). Note: this preserves the depth ordering used by
    // spheres and the vessel mesh.
    out.pos = vec4f(
        center_clip.x / w + offset_x,
        center_clip.y / w + offset_y,
        log_z,
        1.0,
    );
    out.pos = out.pos * w;
    out.uv = uv;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4f {
    let d = distance(in.uv, vec2f(0.5, 0.5)) * 2.0;
    let alpha = 1.0 - smoothstep(0.7, 1.0, d);
    let glow = exp(-d * d * 3.0) * 0.3;
    let a = alpha + glow;
    if (a < 0.01) {
        discard;
    }
    return vec4f(u.color.rgb, u.color.a * a);
}
