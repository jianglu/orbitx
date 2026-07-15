//! 对接端口：连接多级火箭的接口。

use orbitx_math::Vec3;

/// 对接端口，用于连接多级火箭。
#[derive(Clone, Debug)]
pub struct DockPort {
    /// 体坐标系下的位置。
    pub pos: Vec3,
    /// 对接方向（单位向量，指向外）。
    pub dir: Vec3,
    /// 已连接的 Vessel ID + 对方端口索引。
    pub connected_to: Option<(u64, usize)>,
}

impl DockPort {
    /// 创建新对接端口。
    pub fn new(pos: Vec3, dir: Vec3) -> Self {
        Self {
            pos,
            dir,
            connected_to: None,
        }
    }
}
