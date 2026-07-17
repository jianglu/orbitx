// Corona shader: a single camera-facing billboard quad centered on the Sun,
// with a smooth radial glow that peaks at the Sun's rim and fades outward.
// Additive; occluded by the Sun sphere (log-depth, depth-test Less, no write),
// so it appears as a soft continuous halo around the disc - no hard shells.

struct CoronaUniforms {
    vp: mat4x4<f32>,
    center: vec4<f32>,     // xyz world center, w = quad half-size (world units)
    color: vec4<f32>,      // rgb tint, a = intensity
    cam_right: vec4<f32>,  // world-space camera right axis
    cam_up: vec4<f32>,     // world-space camera up axis
    params: vec4<f32>,     // x = time, y = inner_frac (rim), z = falloff, w = unused
    log_depth: vec4<f32>,  // x = C, y = far, z = 1/log2(C*far+1), w = unused
};

@group(0) @binding(0) var<uniform> u: CoronaUniforms;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vid: u32) -> VsOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 1.0, -1.0),
        vec2<f32>(-1.0,  1.0),
        vec2<f32>(-1.0,  1.0),
        vec2<f32>( 1.0, -1.0),
        vec2<f32>( 1.0,  1.0),
    );
    let c = corners[vid];
    let world = u.center.xyz
        + u.cam_right.xyz * (c.x * u.center.w)
        + u.cam_up.xyz * (c.y * u.center.w);
    let clip = u.vp * vec4<f32>(world, 1.0);

    // Log-depth so it matches the Sun sphere (which occludes the disc region).
    let cc = u.log_depth.x;
    let inv_log_far = u.log_depth.z;
    let log_z = log2(cc * clip.w + 1.0) * inv_log_far;

    var out: VsOut;
    out.clip = vec4<f32>(clip.x, clip.y, log_z * clip.w, clip.w);
    out.uv = c;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let r = length(in.uv);          // 0 at center, 1 at quad half-extent edge
    if (r > 1.0) {
        discard;
    }
    let time = u.params.x;
    let inner = u.params.y;         // fraction where the Sun disc ends
    let falloff = u.params.z;

    // Smooth radial glow: peaks at the rim (r = inner), fades to 0 at r = 1.
    // Continuous power curve -> no hard concentric edges.
    let t = clamp((1.0 - r) / max(1.0 - inner, 1e-3), 0.0, 1.0);
    var glow = pow(t, falloff);

    // Faint angular streamers (K-corona filaments), slowly rotating.
    let ang = atan2(in.uv.y, in.uv.x);
    let streak = 0.8 + 0.2 * sin(ang * 14.0 + time * 0.2);
    glow = glow * streak;

    // Subtle activity pulse.
    let pulse = 1.0 + 0.06 * sin(time * 0.5);
    let a = clamp(glow * u.color.a * pulse, 0.0, 1.0);

    // Premultiplied additive output.
    return vec4<f32>(u.color.rgb * a, a);
}
