//! 火箭级定义：干质量、燃料、推力、比冲等静态参数。

use crate::dock::DockPort;
use crate::thruster::Thruster;
use orbitx_math::Vec3;

/// 火箭级的静态参数，用于初始化 Vessel。
///
/// stages[0] = 底层级（第一级），最后一级 = 有效载荷。
#[derive(Clone, Debug)]
pub struct StageSpec {
    /// 级名称。
    pub name: &'static str,
    /// 空重（不含燃料）[kg]。
    pub dry_mass: f64,
    /// 燃料质量 [kg]。
    pub fuel_mass: f64,
    /// 发动机总推力 [N]。
    pub thrust: f64,
    /// 比冲 [s]。
    pub isp: f64,
    /// 推力方向（体坐标系，单位向量）。
    pub engine_dir: Vec3,
    /// 发动机位置（体坐标系，用于力矩计算）。
    pub engine_pos: Vec3,
    /// 级长度 [m]（用于对接端口定位和渲染）。
    pub length: f64,
    /// 级半径 [m]。
    pub radius: f64,
    /// 分离时施加的脉冲速度 [m/s]（沿轴向）。
    pub separation_impulse: f64,
}

impl StageSpec {
    /// 生成该级的推进器列表。
    pub fn make_thrusters(&self) -> Vec<Thruster> {
        if self.thrust > 0.0 {
            vec![Thruster::new(
                self.engine_pos,
                self.engine_dir,
                self.thrust,
                self.isp,
            )]
        } else {
            Vec::new()
        }
    }

    /// 生成该级的对接端口。
    /// 底端口在 -Y/2（连接下级），顶端口在 +Y/2（连接上级）。
    pub fn make_docks(&self) -> Vec<DockPort> {
        let half = self.length / 2.0;
        vec![
            DockPort::new(Vec3::new(0.0, -half, 0.0), Vec3::new(0.0, -1.0, 0.0)), // 底端口
            DockPort::new(Vec3::new(0.0, half, 0.0), Vec3::new(0.0, 1.0, 0.0)),   // 顶端口
        ]
    }

    /// 该级总质量（干质量+燃料）。
    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.fuel_mass
    }
}
