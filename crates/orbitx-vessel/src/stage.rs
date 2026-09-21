//! 火箭级定义：干质量、燃料、多推进器、惯量、TVC 等静态参数。

use crate::dock::DockPort;
use crate::thruster::{pfac_from_sl_points, Thruster};
use orbitx_math::Vec3;

/// PMI"未定义"哨兵值。
pub const PMI_UNDEF: Vec3 = Vec3::new(-1.0, -1.0, -1.0);

/// 默认火箭轴向阻力 Cd(M) 表（教学用估算）。
pub fn default_rocket_cd_mach() -> Vec<(f64, f64)> {
    vec![
        (0.0, 0.30),
        (0.6, 0.32),
        (0.9, 0.55),
        (1.1, 0.95),
        (1.5, 0.70),
        (2.5, 0.45),
        (5.0, 0.35),
    ]
}

/// 单台推进器静态参数（真空额定 + 可选海平面双点）。
#[derive(Clone, Debug)]
pub struct ThrusterSpec {
    pub pos: Vec3,
    pub dir: Vec3,
    /// 真空最大推力 [N]。
    pub thrust: f64,
    /// 真空比冲 [s]。
    pub isp: f64,
    /// 海平面推力 [N]（可选，与 isp_sl 用于推导 pfac）。
    pub thrust_sl: Option<f64>,
    /// 海平面比冲 [s]。
    pub isp_sl: Option<f64>,
    pub max_gimbal: f64,
    pub max_gimbal_rate: f64,
    pub gimbal_axis: Vec3,
    /// 节流斜坡最大速率 [1/s]。0 = 瞬时。
    pub throttle_rate: f64,
}

impl Default for ThrusterSpec {
    fn default() -> Self {
        Self {
            pos: Vec3::ZERO,
            dir: Vec3::new(0.0, 1.0, 0.0),
            thrust: 0.0,
            isp: 0.0,
            thrust_sl: None,
            isp_sl: None,
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            throttle_rate: 0.0,
        }
    }
}

impl ThrusterSpec {
    pub fn to_thruster(&self) -> Thruster {
        let pfac = pfac_from_sl_points(self.isp, self.isp_sl, self.thrust, self.thrust_sl);
        Thruster::new(self.pos, self.dir, self.thrust, self.isp)
            .with_tvc(self.max_gimbal, self.max_gimbal_rate, self.gimbal_axis)
            .with_throttle_rate(self.throttle_rate)
            .with_pfac(pfac)
    }
}

/// 默认 PMI（归一化，单位 m²）。
pub fn default_pmi(radius: f64, length: f64, mass: f64) -> Vec3 {
    if mass > 0.0 {
        let i_axial = 0.5 * radius * radius;
        let i_trans = (3.0 * radius * radius + length * length) / 12.0;
        Vec3::new(i_trans, i_axial, i_trans)
    } else {
        Vec3::new(2.0 * radius, radius, 2.0 * radius)
    }
}

/// 火箭级的静态参数，用于初始化 Vessel。
#[derive(Clone, Debug, Default)]
pub struct StageSpec {
    pub name: &'static str,
    pub dry_mass: f64,
    pub fuel_mass: f64,
    /// 推进器列表（有动力级非空；载荷为空）。
    pub thrusters: Vec<ThrusterSpec>,
    pub length: f64,
    pub radius: f64,
    pub separation_impulse: f64,
    pub pmi: Vec3,
    /// 重力梯度阻尼（Orbiter tidaldamp）。
    pub tidaldamp: f64,
    /// 轴向阻力 Cd(M) 表；空则用 [`default_rocket_cd_mach`]。
    pub cd_mach: Vec<(f64, f64)>,
    pub docks: Option<Vec<DockPort>>,
}

impl StageSpec {
    /// 便捷：单机推进级（测试/演示用）。
    pub fn with_single_thruster(
        name: &'static str,
        dry_mass: f64,
        fuel_mass: f64,
        thrust: f64,
        isp: f64,
        engine_pos: Vec3,
        engine_dir: Vec3,
        length: f64,
        radius: f64,
        separation_impulse: f64,
    ) -> Self {
        Self {
            name,
            dry_mass,
            fuel_mass,
            thrusters: if thrust > 0.0 {
                vec![ThrusterSpec {
                    pos: engine_pos,
                    dir: engine_dir,
                    thrust,
                    isp,
                    ..Default::default()
                }]
            } else {
                vec![]
            },
            length,
            radius,
            separation_impulse,
            ..Default::default()
        }
    }

    /// 由推进器规格生成运行时推进器。
    pub fn make_thrusters(&self) -> Vec<Thruster> {
        self.thrusters.iter().map(|s| s.to_thruster()).collect()
    }

    /// 真空总推力 [N]（配置汇总）。
    pub fn vacuum_thrust_sum(&self) -> f64 {
        self.thrusters.iter().map(|t| t.thrust).sum()
    }

    pub fn make_docks(&self) -> Vec<DockPort> {
        if let Some(ref docks) = self.docks {
            return docks
                .iter()
                .map(|d| DockPort::with_rot(d.pos, d.dir, d.rot))
                .collect();
        }
        let half = self.length / 2.0;
        let rot = Vec3::new(0.0, 0.0, 1.0);
        vec![
            DockPort::with_rot(
                Vec3::new(0.0, -half, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                rot,
            ),
            DockPort::with_rot(
                Vec3::new(0.0, half, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                rot,
            ),
        ]
    }

    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.fuel_mass
    }

    pub fn effective_pmi(&self) -> Vec3 {
        let m = self.total_mass();
        if self.pmi.x > 0.0 && self.pmi.y > 0.0 && self.pmi.z > 0.0 && m > 0.0 {
            Vec3::new(self.pmi.x / m, self.pmi.y / m, self.pmi.z / m)
        } else {
            default_pmi(self.radius, self.length, m)
        }
    }

    /// 用于气动的 Cd(M) 表。
    pub fn cd_mach_table(&self) -> Vec<(f64, f64)> {
        if self.cd_mach.is_empty() {
            default_rocket_cd_mach()
        } else {
            self.cd_mach.clone()
        }
    }
}
