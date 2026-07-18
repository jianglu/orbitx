// Atmosphere shell shader — physically-inspired single-scattering (P3B-4c partial).
//
// Upgrades the previous Fresnel-rim glow to a Rayleigh + Mie single-scattering
// model that naturally produces:
//   - Blue day-side sky (Rayleigh scattering, wavelength-dependent β)
//   - Warm terminator / sunset colors (sun-path extinction thickens blue path)
//   - Bright forward-scatter halo when looking toward the sun (Mie phase)
//   - Thick limb at grazing view (long path through atmosphere)
//   - Dark night side
//
// The shader stays "thin shell" (no ray-marching): it treats each fragment as
// a single scattering event at the top of the atmosphere. Effective path length
// is estimated as H / cos(view_angle) and sun path as H / cos(sun_angle).
// Physically-motivated β_R / β_M / phase functions give the correct hue and
// intensity trends; a per-planet `atmo_color` biases the base Rayleigh
// coefficients so Mars looks reddish, Venus yellow, Titan orange, etc.

struct AtmoUniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    atmo_color: vec4<f32>,  // rgb = per-planet hue bias, a = intensity multiplier
    light_dir: vec4<f32>,   // xyz = direction from planet to sun (normalized)
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

const PI: f32 = 3.14159265;

// Physical scale heights + scattering coefficients (Earth-like reference).
const H_R: f32 = 8000.0;         // Rayleigh scale height (m)
const BETA_R: vec3<f32> = vec3<f32>(5.802e-6, 13.558e-6, 33.100e-6); // per meter
const BETA_M: f32 = 21.0e-6;     // Mie scattering, wavelength-independent
const G_MIE: f32 = 0.76;         // Mie asymmetry (forward-scatter)
const SUN_INTENSITY: f32 = 20.0; // ad-hoc gain so single-scatter is visible

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.wnormal);
    // Camera is at render-space origin, so view direction from fragment toward
    // the camera is -normalize(wpos).
    let view_dir = normalize(-in.wpos);
    // `light_dir` is the direction from the planet toward the sun. We treat
    // it as the sun direction at the fragment (planet ≪ 1 AU from the sun).
    let sun_dir = normalize(u.light_dir.xyz);

    // Angles.
    let mu_v = max(dot(n, view_dir), 0.02);   // 0 at limb, 1 at zenith
    let mu_s = dot(n, sun_dir);                // >0 day, <0 night
    // Scattering angle cosine: light propagates from sun (→ -sun_dir) into
    // atmosphere and exits toward camera (→ view_dir). Scattering angle θ is
    // between incoming (-sun_dir) and outgoing (view_dir) directions.
    // cos θ = dot(-sun_dir, view_dir) = -dot(sun_dir, view_dir).
    let cos_t = -dot(sun_dir, view_dir);
    let cos2 = cos_t * cos_t;

    // Rayleigh phase (symmetric in cos θ).
    let phase_r = (3.0 / (16.0 * PI)) * (1.0 + cos2);
    // Henyey–Greenstein Mie phase (forward-heavy at g=0.76).
    let g2 = G_MIE * G_MIE;
    let phase_m_num = 3.0 * (1.0 - g2) * (1.0 + cos2);
    let phase_m_den = 8.0 * PI * (2.0 + g2) * pow(1.0 + g2 - 2.0 * G_MIE * cos_t, 1.5);
    let phase_m = phase_m_num / max(phase_m_den, 1e-4);

    // Per-planet biased Rayleigh coefficient. The user-supplied atmo_color
    // scales the natural blue-heavy spectrum: Earth ~ (0.30, 0.55, 1.00)
    // leaves blue dominant; Mars ~ (0.85, 0.55, 0.40) inverts toward red-orange.
    let beta_r = BETA_R * (u.atmo_color.rgb * 3.0);

    // Sun-path extinction: sunlight traversing the atmosphere before scattering
    // at the fragment gets Rayleigh-attenuated. Path length ≈ H_R / mu_s;
    // clamp mu_s so terminator has finite (large) path.
    let mu_s_clamped = max(mu_s + 0.1, 0.05);
    let sun_ext = exp(-beta_r * H_R / mu_s_clamped);

    // View-path length through atmosphere ≈ H_R / mu_v (Chapman approximation).
    let view_path = H_R / mu_v;

    // Single-scattering approximation:
    //   L_scatter ≈ [β_R · P_R + β_M · P_M] · (view path density) · T_sun
    let in_scatter = (beta_r * phase_r + vec3<f32>(BETA_M) * phase_m)
                   * view_path * sun_ext;

    // Day-side fade (smooth terminator crossover).
    let day = smoothstep(-0.1, 0.15, mu_s);

    // Overall intensity + user-provided alpha modulation.
    let intensity = SUN_INTENSITY * day * u.atmo_color.a;

    let color = in_scatter * intensity;

    // Alpha = perceived luminance clamped to [0, 1]. Ensures the atmosphere
    // is transparent in the middle of the disc (mu_v→1) and opaque at the limb
    // (mu_v→0). Also fades cleanly to zero on the night side.
    let luminance = dot(color, vec3<f32>(0.2126, 0.7152, 0.0722));
    let alpha = clamp(luminance, 0.0, 1.0);

    // Premultiplied output (rgb already multiplied by alpha for correct blending).
    return vec4<f32>(color * alpha, alpha);
}
