// Sun photosphere shader: animated granulation (FBM value noise) + limb
// darkening + blackbody color ramp + sunspots. Fully emissive (unlit).
// The camera is the render-space origin, so the view direction toward the
// camera at a fragment is -normalize(worldPos).

struct SunUniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    params: vec4<f32>,     // x = time, y = activity, z = unused, w = unused
    log_depth: vec4<f32>,  // x = C, y = far, z = 1/log2(C*far+1), w = unused
};

@group(0) @binding(0) var<uniform> u: SunUniforms;
@group(1) @binding(0) var sun_tex: texture_2d<f32>;
@group(1) @binding(1) var sun_smp: sampler;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) wnormal: vec3<f32>,
    @location(1) wpos: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) lpos: vec3<f32>,  // local (unit-sphere) position for stable noise
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    out.wnormal = (u.model * vec4<f32>(in.normal, 0.0)).xyz;
    out.wpos = (u.model * vec4<f32>(in.pos, 1.0)).xyz;
    out.uv = in.uv;
    out.lpos = in.pos;
    let clip = u.mvp * vec4<f32>(in.pos, 1.0);
    let c = u.log_depth.x;
    let inv_log_far = u.log_depth.z;
    let log_z = log2(c * clip.w + 1.0) * inv_log_far;
    out.clip = vec4<f32>(clip.x, clip.y, log_z * clip.w, clip.w);
    return out;
}

// --- 3D value noise + FBM ---
fn hash13(p: vec3<f32>) -> f32 {
    var p3 = fract(p * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn vnoise(x: vec3<f32>) -> f32 {
    let i = floor(x);
    let f = fract(x);
    let u = f * f * (3.0 - 2.0 * f);
    let n000 = hash13(i + vec3<f32>(0.0, 0.0, 0.0));
    let n100 = hash13(i + vec3<f32>(1.0, 0.0, 0.0));
    let n010 = hash13(i + vec3<f32>(0.0, 1.0, 0.0));
    let n110 = hash13(i + vec3<f32>(1.0, 1.0, 0.0));
    let n001 = hash13(i + vec3<f32>(0.0, 0.0, 1.0));
    let n101 = hash13(i + vec3<f32>(1.0, 0.0, 1.0));
    let n011 = hash13(i + vec3<f32>(0.0, 1.0, 1.0));
    let n111 = hash13(i + vec3<f32>(1.0, 1.0, 1.0));
    let nx00 = mix(n000, n100, u.x);
    let nx10 = mix(n010, n110, u.x);
    let nx01 = mix(n001, n101, u.x);
    let nx11 = mix(n011, n111, u.x);
    let nxy0 = mix(nx00, nx10, u.y);
    let nxy1 = mix(nx01, nx11, u.y);
    return mix(nxy0, nxy1, u.z);
}

fn fbm(p: vec3<f32>) -> f32 {
    var v = 0.0;
    var amp = 0.5;
    var q = p;
    for (var i = 0; i < 5; i = i + 1) {
        v = v + amp * vnoise(q);
        q = q * 2.02;
        amp = amp * 0.5;
    }
    return v;
}

// Blackbody-ish ramp: t in [0,1], cool->hot. Kept bright (photosphere).
fn sun_color(t: f32) -> vec3<f32> {
    let cool = vec3<f32>(1.0, 0.55, 0.20);   // bright orange (cooler regions)
    let mid  = vec3<f32>(1.0, 0.82, 0.45);   // orange-yellow
    let hot  = vec3<f32>(1.0, 0.98, 0.90);   // yellow-white (hot core)
    let a = smoothstep(0.0, 0.55, t);
    let b = smoothstep(0.45, 1.0, t);
    return mix(mix(cool, mid, a), hot, b);
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let time = u.params.x;
    let n = normalize(in.wnormal);
    let view_dir = normalize(-in.wpos);

    // Granulation: animated FBM over the unit-sphere position. Flow the noise
    // slowly over time so the convection cells shimmer.
    let base = normalize(in.lpos);
    let flow = time * 0.03;
    let gran = fbm(base * 14.0 + vec3<f32>(0.0, 0.0, flow));
    let fine = fbm(base * 34.0 - vec3<f32>(flow, 0.0, 0.0));
    let cell = gran * 0.7 + fine * 0.3;

    // Large-scale texture base (Sun.jpg) blended in for global structure.
    let tex = textureSample(sun_tex, sun_smp, in.uv).rgb;
    let tex_lum = dot(tex, vec3<f32>(0.299, 0.587, 0.114));

    // Brightness field combines granulation + texture luminance.
    // Brightness field: keep the photosphere bright overall (it is the
    // brightest thing in the scene) so its lit limb blends into the corona.
    var bright = clamp(cell * 0.5 + tex_lum * 0.4 + 0.45, 0.0, 1.0);

    // Sunspots: low-frequency dark regions.
    let spot_field = fbm(base * 3.5 + vec3<f32>(11.0, 5.0, flow * 0.5));
    let spot = smoothstep(0.62, 0.48, spot_field); // 1 inside spot
    bright = bright * (1.0 - spot * 0.8);

    // Gentle limb darkening: the edge stays bright (no dark ring at the disc
    // boundary against the corona).
    let mu = max(dot(n, view_dir), 0.0);
    let limb = 0.6 + 0.4 * pow(mu, 0.35);

    var color = sun_color(bright);
    // Sunspot umbra tint.
    color = mix(color, vec3<f32>(0.35, 0.13, 0.04), spot * 0.65);
    color = color * limb;

    // Strong emissive gain (dazzling photosphere) with a subtle activity pulse.
    let pulse = 1.0 + 0.05 * sin(time * 0.6);
    color = color * (2.3 * pulse);

    return vec4<f32>(color, 1.0);
}
