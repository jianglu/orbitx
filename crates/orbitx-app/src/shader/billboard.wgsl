// Billboard shader for distant-body fallback rendering.
// Renders a camera-facing disc/glow for bodies that are sub-pixel at true scale.

struct Uniforms {
    center: vec4f,       // xyz = world position, w = unused
    color: vec4f,        // rgba
    screen_size: vec4f,  // x = pixel radius, y = viewport_width, z = viewport_height, w = unused
    vp_row0: vec4f,      // view-projection matrix row 0
    vp_row1: vec4f,      // view-projection matrix row 1
    vp_row2: vec4f,      // view-projection matrix row 2
    vp_row3: vec4f,      // view-projection matrix row 3
};

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) pos: vec4f,
    @location(0) uv: vec2f,
};

@vertex
fn vs_main(@builtin(vertex_index) vid: u32) -> VsOut {
    // Quad vertices: two triangles covering [-1,1] x [-1,1]
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

    // Project center to clip space
    let vp = mat4x4f(u.vp_row0, u.vp_row1, u.vp_row2, u.vp_row3);
    let center_clip = vp * u.center;

    // Offset in NDC by pixel radius
    let pixel_r = u.screen_size.x;
    let w = center_clip.w;
    let offset_x = quad_pos.x * pixel_r * 2.0 / u.screen_size.y;
    let offset_y = quad_pos.y * pixel_r * 2.0 / u.screen_size.z;

    var out: VsOut;
    out.pos = vec4f(
        center_clip.x / w + offset_x,
        center_clip.y / w + offset_y,
        center_clip.z / w,
        1.0,
    );
    // Reconstruct w for perspective
    out.pos = out.pos * w;
    out.uv = uv;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4f {
    // Distance from center in UV space [0,1]
    let d = distance(in.uv, vec2f(0.5, 0.5)) * 2.0;

    // Disc with soft edge
    let alpha = 1.0 - smoothstep(0.7, 1.0, d);

    // Glow falloff outside the disc
    let glow = exp(-d * d * 3.0) * 0.3;

    let a = alpha + glow;
    if (a < 0.01) {
        discard;
    }

    return vec4f(u.color.rgb, u.color.a * a);
}
