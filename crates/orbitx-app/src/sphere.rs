//! UV sphere geometry generation for planet/star rendering.
//!
//! Generates a unit sphere (radius 1.0) with per-vertex normals,
//! suitable for rendering with a model matrix that applies position and scale.

use bytemuck::{Pod, Zeroable};

/// Vertex format: position + normal + uv, 32 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
}

impl Vertex {
    const DESC: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![
            0 => Float32x3,
            1 => Float32x3,
            2 => Float32x2,
        ],
    };

    pub fn desc() -> &'static wgpu::VertexBufferLayout<'static> {
        &Self::DESC
    }
}

/// Generate a UV sphere with the given segment and ring count.
pub fn generate_uv_sphere(segments: u32, rings: u32) -> (Vec<Vertex>, Vec<u16>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for ring in 0..=rings {
        let theta = std::f32::consts::PI * ring as f32 / rings as f32;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        for segment in 0..=segments {
            let phi = 2.0 * std::f32::consts::PI * segment as f32 / segments as f32;
            let x = sin_theta * phi.cos();
            let y = cos_theta;
            let z = sin_theta * phi.sin();

            // Equirectangular UV: u = longitude/2pi, v = colatitude/pi
            // (v=0 at the north pole = top of the map image).
            let u = segment as f32 / segments as f32;
            let v = ring as f32 / rings as f32;

            vertices.push(Vertex {
                position: [x, y, z],
                normal: [x, y, z],
                uv: [u, v],
            });
        }
    }

    for ring in 0..rings {
        let row_start = ring * (segments + 1);
        let next_row_start = (ring + 1) * (segments + 1);

        for segment in 0..segments {
            let curr = row_start + segment;
            let next = row_start + segment + 1;
            let curr_below = next_row_start + segment;
            let next_below = next_row_start + segment + 1;

            indices.push(curr as u16);
            indices.push(curr_below as u16);
            indices.push(next as u16);

            indices.push(next as u16);
            indices.push(curr_below as u16);
            indices.push(next_below as u16);
        }
    }

    (vertices, indices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_has_correct_vertex_count() {
        let (vertices, _) = generate_uv_sphere(24, 16);
        assert_eq!(vertices.len(), 425);
    }

    #[test]
    fn sphere_has_correct_index_count() {
        let (_, indices) = generate_uv_sphere(24, 16);
        assert_eq!(indices.len(), 2304);
    }

    #[test]
    fn sphere_vertices_are_unit_length() {
        let (vertices, _) = generate_uv_sphere(12, 8);
        for v in &vertices {
            let len = (v.position[0].powi(2) + v.position[1].powi(2) + v.position[2].powi(2)).sqrt();
            assert!((len - 1.0).abs() < 0.01, "len={}", len);
        }
    }

    #[test]
    fn sphere_top_pole() {
        let (vertices, _) = generate_uv_sphere(8, 4);
        assert!((vertices[0].position[1] - 1.0).abs() < 0.01);
    }

    #[test]
    fn sphere_uv_range() {
        let (vertices, _) = generate_uv_sphere(24, 16);
        for v in &vertices {
            assert!((0.0..=1.0).contains(&v.uv[0]), "u out of range: {}", v.uv[0]);
            assert!((0.0..=1.0).contains(&v.uv[1]), "v out of range: {}", v.uv[1]);
        }
        // North pole (first vertex) at v=0 (top of map), south pole at v=1.
        assert_eq!(vertices[0].uv[1], 0.0);
        assert_eq!(vertices[vertices.len() - 1].uv[1], 1.0);
    }

    #[test]
    fn all_indices_in_bounds() {
        let (vertices, indices) = generate_uv_sphere(24, 16);
        let max_idx = vertices.len() as u16;
        for &idx in &indices {
            assert!(idx < max_idx, "index {} >= {}", idx, max_idx);
        }
    }
}
