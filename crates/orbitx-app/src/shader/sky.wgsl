// Skybox shader: a large sphere centered on the camera, textured with an
// equirectangular star map. Unlit; depth is forced to the far plane so it
// never occludes scene geometry (it is drawn first as the background).

struct SkyUniforms {
    view_proj: mat4x4<f32>,  // camera rotation only (translation ignored / camera at origin)
    tint: vec4<f32>,         // rgb multiplier, a = unused
};

@group(0) @binding(0) var<uniform> u: SkyUniforms;
@group(1) @binding(0) var sky_tex: texture_2d<f32>;
@group(1) @binding(1) var sky_smp: sampler;

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
    let clip = u.view_proj * vec4<f32>(in.pos, 1.0);
    // Force depth to the far plane (z = w -> ndc.z = 1) so the sky is always
    // behind everything else.
    out.clip = vec4<f32>(clip.x, clip.y, clip.w, clip.w);
    out.uv = in.uv;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let c = textureSample(sky_tex, sky_smp, in.uv).rgb * u.tint.rgb;
    return vec4<f32>(c, 1.0);
}
