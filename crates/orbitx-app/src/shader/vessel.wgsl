// Vessel PBR-ish shader (P3C-4a).
//
// Uses the same Uniforms layout as planet.wgsl so bind groups, buffers,
// and vertex layout are all shared. Two uniform fields are repurposed:
//   - light_dir.w  = metallic  (0 = pure dielectric, 1 = pure metal)
//   - log_depth.w  = roughness (0 = mirror, 1 = fully rough)
//
// Fragment shading is a simplified analytical PBR (Cook-Torrance-ish):
//   Lambert diffuse × (1 - metallic)
// + GGX specular × Schlick Fresnel × constant geometric
// + Fresnel rim highlight
// + small ambient
// The camera is the floating-point origin in render space (see coord.rs),
// so view_dir = -normalize(world_pos).

struct Uniforms {
    mvp: mat4x4<f32>,
    model: mat4x4<f32>,
    base_color: vec4<f32>,
    light_dir: vec4<f32>,   // xyz = direction, w = metallic
    log_depth: vec4<f32>,   // x = C, y = far, z = 1/log2(C*far+1), w = roughness
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

// Group 1 exists to keep pipeline layout compatible with the planet pipeline;
// vessel shader does not sample from it (no vessel textures yet).
@group(1) @binding(0) var _tex: texture_2d<f32>;
@group(1) @binding(1) var _smp: sampler;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) world_pos: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    // World-space normal (uniform scale, no inverse-transpose needed)
    out.world_normal = (uniforms.model * vec4<f32>(in.normal, 0.0)).xyz;
    // World-space position (camera is at origin in render space)
    out.world_pos = (uniforms.model * vec4<f32>(in.position, 1.0)).xyz;

    let clip = uniforms.mvp * vec4<f32>(in.position, 1.0);
    let c = uniforms.log_depth.x;
    let inv_log_far = uniforms.log_depth.z;
    let log_z = log2(c * clip.w + 1.0) * inv_log_far;
    out.clip_pos = vec4<f32>(clip.x, clip.y, log_z * clip.w, clip.w);

    return out;
}

const PI: f32 = 3.14159265;

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let l = normalize(uniforms.light_dir.xyz);
    // Camera at origin → view direction from fragment to camera:
    let v = normalize(-in.world_pos);
    let h = normalize(l + v);

    let ndl = max(dot(n, l), 0.0);
    let ndv = max(dot(n, v), 0.0);
    let ndh = max(dot(n, h), 0.0);
    let vdh = max(dot(v, h), 0.0);

    let metallic = clamp(uniforms.light_dir.w, 0.0, 1.0);
    // Roughness floor 0.05 keeps highlight non-degenerate near mirror.
    let roughness = clamp(uniforms.log_depth.w, 0.05, 1.0);
    let base = uniforms.base_color.rgb;

    // Lambert diffuse (dielectric only — metals absorb diffuse energy).
    let diffuse = base * (1.0 - metallic) * ndl;

    // GGX / Trowbridge-Reitz distribution.
    let alpha = roughness * roughness;
    let a2 = alpha * alpha;
    let denom = ndh * ndh * (a2 - 1.0) + 1.0;
    let ndf = a2 / max(PI * denom * denom, 1e-6);

    // Schlick Fresnel: F0 = 0.04 dielectric, base_color for metals.
    let f0 = mix(vec3<f32>(0.04), base, metallic);
    let fresnel = f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - vdh, 5.0);

    // Simplified geometric attenuation (Smith-Schlick style, roughness-dependent).
    // Full Smith-GGX would use k = (roughness+1)^2/8; this is a cheap approximation.
    let k = (roughness + 1.0) * (roughness + 1.0) * 0.125;
    let g_v = ndv / (ndv * (1.0 - k) + k);
    let g_l = ndl / (ndl * (1.0 - k) + k);
    let geom = g_v * g_l;

    let spec_denom = 4.0 * ndl * ndv + 1e-4;
    let specular = (ndf * fresnel * geom) / spec_denom;

    // Rim edge (subtle atmospheric halo, especially on metal).
    let rim = pow(1.0 - ndv, 3.0) * mix(0.05, 0.25, metallic);

    // Ambient + emissive floor.
    // Space has no natural fill light, so a matte vessel would fade to pure
    // black on the shadow side and be near-invisible against the starfield.
    // We add a base-color ambient (0.30) plus a small self-luminous floor
    // (0.15) so the vessel stays legible from every angle. Total shadow-side
    // brightness ≈ base * 0.45 → clearly visible against black space.
    let ambient = base * 0.30;
    let emissive_floor = base * 0.15;

    let color = ambient + emissive_floor + diffuse + specular * ndl + vec3<f32>(rim);
    return vec4<f32>(color, uniforms.base_color.a);
}
