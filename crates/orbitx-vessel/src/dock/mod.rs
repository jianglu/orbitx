//! 对接端口：连接多级火箭与侧挂助推的接口（Orbiter `PortSpec`）。

use orbitx_math::Vec3;

/// 对接端口，用于连接多级火箭 / 侧挂助推。
#[derive(Clone, Debug)]
pub struct DockPort {
    /// 体坐标系下的位置（Orbiter `ref`）[m]。
    pub pos: Vec3,
    /// 对接接近方向（单位向量，指向外；Orbiter `dir`）。
    pub dir: Vec3,
    /// 滚转对齐参考方向（单位向量，须与 `dir` 近似正交；Orbiter `rot`）。
    pub rot: Vec3,
    /// 已连接的 Vessel ID + 对方端口索引。
    pub connected_to: Option<(u64, usize)>,
}

impl DockPort {
    /// 创建对接端口；`rot` 缺省为与 `dir` 正交的 `(0,0,1)`（若与 `dir` 平行则用 `(1,0,0)`）。
    pub fn new(pos: Vec3, dir: Vec3) -> Self {
        let rot = default_rot_for_dir(dir);
        Self {
            pos,
            dir,
            rot,
            connected_to: None,
        }
    }

    /// 创建带显式 `rot` 的对接端口。
    pub fn with_rot(pos: Vec3, dir: Vec3, rot: Vec3) -> Self {
        Self {
            pos,
            dir,
            rot,
            connected_to: None,
        }
    }
}

/// 为接近方向选一个默认纵向对齐向量。
fn default_rot_for_dir(dir: Vec3) -> Vec3 {
    let d = dir.unit();
    let candidate = Vec3::new(0.0, 0.0, 1.0);
    if orbitx_math::dot(d, candidate).abs() > 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        candidate
    }
}

#[cfg(test)]
mod tests;
