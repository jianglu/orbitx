//! 程序化火箭 mesh 生成 — 无资产依赖的最小可行 vessel 几何。
//!
//! 局部坐标：+Y 为鼻锥方向（"上"），+Z 为发动机方向（"下"）。
//! 半径 1.0，总长 3.0（近似 3:1 长径比火箭，含尾翼），单位无关；
//! 由渲染时的 model 矩阵按 vessel.scale 米数缩放。
//!
//! 结构：
//! - 圆柱身（radius 1.0，长度 1.6，从 y=-0.8 到 y=+0.8，radial=16 段）
//! - 上锥（鼻锥，从 y=+0.8 到 y=+1.6 收尖）
//! - 下锥（发动机喷管收敛，从 y=-0.8 到 y=-1.4）
//! - 4 片十字尾翼（y=-0.9..-0.5，从 body radius 1.0 外扩到 1.7）
//!
//! 顶点复用 sphere::Vertex（position + normal + uv）以共用 planet.wgsl 管线。

use crate::sphere::Vertex;

/// 生成火箭 mesh，返回顶点数组和 u16 索引数组（与 sphere 管线一致）。
///
/// `radial_segments` 控制圆柱和锥的绕周细分（默认建议 16）。顶点总数远低于 u16 上限。
pub fn generate_rocket(radial_segments: u32) -> (Vec<Vertex>, Vec<u16>) {
    let seg = radial_segments.max(6);
    let mut verts: Vec<Vertex> = Vec::new();
    let mut idx: Vec<u16> = Vec::new();

    // 参数
    let r_body: f32 = 1.0;
    let y_top: f32 = 0.8;     // 身与鼻锥交界
    let y_bot: f32 = -0.8;    // 身与发动机锥交界
    let y_nose: f32 = 1.6;    // 鼻锥顶点
    let y_engine: f32 = -1.4; // 发动机锥底
    let r_engine: f32 = 0.45; // 发动机口半径

    // ---- Cylinder body（radial_segments × 1 ring）----
    // 环 0：y=y_bot，环 1：y=y_top
    let cyl_start = verts.len() as u16;
    for ring in 0..=1 {
        let y = if ring == 0 { y_bot } else { y_top };
        for i in 0..=seg {
            let phi = 2.0 * std::f32::consts::PI * (i as f32) / (seg as f32);
            let (sp, cp) = phi.sin_cos();
            verts.push(Vertex {
                position: [r_body * cp, y, r_body * sp],
                normal: [cp, 0.0, sp],   // 侧向法线
                uv: [i as f32 / seg as f32, ring as f32],
            });
        }
    }
    // 索引：连接环 0 和环 1
    let stride = (seg + 1) as u16;
    for i in 0..(seg as u16) {
        let a = cyl_start + i;
        let b = cyl_start + i + 1;
        let c = cyl_start + stride + i;
        let d = cyl_start + stride + i + 1;
        // CCW from outside: a→b→d, a→d→c
        idx.extend_from_slice(&[a, b, d, a, d, c]);
    }

    // ---- Nose cone（从 y_top 圆环到 y_nose 顶点）----
    let nose_ring_start = verts.len() as u16;
    for i in 0..=seg {
        let phi = 2.0 * std::f32::consts::PI * (i as f32) / (seg as f32);
        let (sp, cp) = phi.sin_cos();
        let slope = (y_nose - y_top) / r_body;
        let nlen = (1.0 + slope * slope).sqrt();
        verts.push(Vertex {
            position: [r_body * cp, y_top, r_body * sp],
            normal: [cp / nlen, slope / nlen, sp / nlen],
            uv: [i as f32 / seg as f32, 0.0],
        });
    }
    let nose_tip = verts.len() as u16;
    verts.push(Vertex {
        position: [0.0, y_nose, 0.0],
        normal: [0.0, 1.0, 0.0],
        uv: [0.5, 1.0],
    });
    for i in 0..(seg as u16) {
        let a = nose_ring_start + i;
        let b = nose_ring_start + i + 1;
        idx.extend_from_slice(&[a, b, nose_tip]);
    }

    // ---- Engine cone（从 y_bot 圆环收敛到 y_engine 处的小圆环）----
    let eng_top = verts.len() as u16;
    for i in 0..=seg {
        let phi = 2.0 * std::f32::consts::PI * (i as f32) / (seg as f32);
        let (sp, cp) = phi.sin_cos();
        let slope = (y_bot - y_engine) / (r_body - r_engine);
        let nlen = (1.0 + slope * slope).sqrt();
        verts.push(Vertex {
            position: [r_body * cp, y_bot, r_body * sp],
            normal: [cp / nlen, -slope / nlen, sp / nlen],
            uv: [i as f32 / seg as f32, 0.0],
        });
    }
    let eng_bot = verts.len() as u16;
    for i in 0..=seg {
        let phi = 2.0 * std::f32::consts::PI * (i as f32) / (seg as f32);
        let (sp, cp) = phi.sin_cos();
        let slope = (y_bot - y_engine) / (r_body - r_engine);
        let nlen = (1.0 + slope * slope).sqrt();
        verts.push(Vertex {
            position: [r_engine * cp, y_engine, r_engine * sp],
            normal: [cp / nlen, -slope / nlen, sp / nlen],
            uv: [i as f32 / seg as f32, 1.0],
        });
    }
    for i in 0..(seg as u16) {
        let a = eng_top + i;
        let b = eng_top + i + 1;
        let c = eng_bot + i;
        let d = eng_bot + i + 1;
        idx.extend_from_slice(&[a, d, b, a, c, d]);
    }

    // ---- Engine bottom disk（封底）----
    let disk_ring = eng_bot;
    let disk_center = verts.len() as u16;
    verts.push(Vertex {
        position: [0.0, y_engine, 0.0],
        normal: [0.0, -1.0, 0.0],
        uv: [0.5, 0.5],
    });
    for i in 0..(seg as u16) {
        let a = disk_ring + i;
        let b = disk_ring + i + 1;
        idx.extend_from_slice(&[a, disk_center, b]);
    }

    // ---- 4 fins（十字排列）----
    let fin_thickness: f32 = 0.06;
    let fin_r_out: f32 = 1.7;
    let fin_y0: f32 = -0.9;
    let fin_y1: f32 = -0.5;
    let fin_r_in: f32 = r_body;
    for k in 0..4 {
        let angle = k as f32 * std::f32::consts::FRAC_PI_2;
        let (sa, ca) = angle.sin_cos();
        let hx = fin_thickness * 0.5;
        let make = |r: f32, y: f32, t: f32, ny: f32| {
            let px = r * ca + t * (-sa);
            let pz = r * sa + t * ca;
            let nx = -sa * ny.signum();
            let nz = ca * ny.signum();
            let n = if ny.abs() < 0.1 { [nx, 0.0, nz] } else { [0.0, ny, 0.0] };
            Vertex { position: [px, y, pz], normal: n, uv: [0.0, 0.0] }
        };
        let base = verts.len() as u16;
        verts.push(make(fin_r_in, fin_y0, hx, 1.0));
        verts.push(make(fin_r_out, fin_y0, hx, 1.0));
        verts.push(make(fin_r_out, fin_y1, hx, 1.0));
        verts.push(make(fin_r_in, fin_y1, hx, 1.0));
        verts.push(make(fin_r_in, fin_y0, -hx, -1.0));
        verts.push(make(fin_r_out, fin_y0, -hx, -1.0));
        verts.push(make(fin_r_out, fin_y1, -hx, -1.0));
        verts.push(make(fin_r_in, fin_y1, -hx, -1.0));
        idx.extend_from_slice(&[
            base + 0, base + 1, base + 2, base + 0, base + 2, base + 3,
            base + 4, base + 6, base + 5, base + 4, base + 7, base + 6,
        ]);
    }

    (verts, idx)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rocket_generates_nonempty_mesh() {
        let (v, i) = generate_rocket(16);
        assert!(v.len() > 20, "should have body + nose + engine + fin verts");
        assert!(i.len() % 3 == 0, "indices should form triangles");
    }

    #[test]
    fn rocket_indices_reference_valid_verts() {
        let (v, i) = generate_rocket(16);
        let n = v.len() as u16;
        for &idx in &i {
            assert!(idx < n, "index {} out of bounds (n={})", idx, n);
        }
    }

    #[test]
    fn rocket_extents_reasonable() {
        let (v, _) = generate_rocket(16);
        let (mut ymin, mut ymax) = (f32::INFINITY, f32::NEG_INFINITY);
        for vtx in &v {
            ymin = ymin.min(vtx.position[1]);
            ymax = ymax.max(vtx.position[1]);
        }
        assert!(ymin < -1.3 && ymin > -1.5, "engine bottom near y=-1.4");
        assert!(ymax > 1.5 && ymax < 1.7, "nose tip near y=+1.6");
    }

    #[test]
    fn rocket_radial_segments_scale() {
        let (v6, _) = generate_rocket(6);
        let (v16, _) = generate_rocket(16);
        let (v32, _) = generate_rocket(32);
        assert!(v6.len() < v16.len() && v16.len() < v32.len());
    }
}
