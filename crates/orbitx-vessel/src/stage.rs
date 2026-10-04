//! 火箭级定义：干质量、多储箱、多推进器、干/燃惯量、TVC 等静态参数。

use crate::dock::DockPort;
use crate::fuel::PropellantTank;
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

/// 均匀圆柱主惯量对角线 [kg·m²]（Y 纵轴）：`Iyy=½mr²`，`Ixx=Izz=m/12(3r²+h²)`。
pub fn cylinder_inertia(m_kg: f64, r_m: f64, h_m: f64) -> Vec3 {
    if m_kg <= 0.0 {
        return Vec3::ZERO;
    }
    let i_axial = 0.5 * m_kg * r_m * r_m;
    let i_trans = m_kg / 12.0 * (3.0 * r_m * r_m + h_m * h_m);
    Vec3::new(i_trans, i_axial, i_trans)
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

/// 储箱静态参数。
#[derive(Clone, Debug)]
pub struct TankSpec {
    pub id: u32,
    pub max_mass: f64,
    pub mass: f64,
    pub pos: Vec3,
    pub inertia: Vec3,
    pub efficiency: f64,
}

impl TankSpec {
    pub fn to_tank(&self) -> PropellantTank {
        PropellantTank::with_mass(
            self.id,
            self.max_mass,
            self.mass,
            self.pos,
            self.inertia,
            self.efficiency,
        )
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
    /// 干质心 [m]（体坐标，原点 = 满载质心）。
    pub dry_center: Vec3,
    /// 干惯量对角线 [kg·m²]，绕 `dry_center`。
    pub dry_inertia: Vec3,
    pub tanks: Vec<TankSpec>,
    /// 推进器列表（有动力级非空；载荷为空）。
    pub thrusters: Vec<ThrusterSpec>,
    pub length: f64,
    pub radius: f64,
    pub separation_impulse: f64,
    /// 重力梯度阻尼（Orbiter tidaldamp）。
    pub tidaldamp: f64,
    /// 轴向阻力 Cd(M) 表；空则用 [`default_rocket_cd_mach`]。
    pub cd_mach: Vec<(f64, f64)>,
    pub docks: Option<Vec<DockPort>>,
}

impl StageSpec {
    /// 由干重 + 燃料质量估算 `dry_center` / `dry_inertia` / 单罐（满载 COM≈原点）。
    pub fn estimated_mass_props(
        dry_mass: f64,
        fuel_mass: f64,
        length: f64,
        radius: f64,
    ) -> (Vec3, Vec3, Vec<TankSpec>) {
        let dry_inertia = cylinder_inertia(dry_mass, radius, length);
        if fuel_mass > 0.0 && dry_mass + fuel_mass > 0.0 {
            let y_dry = -2.0 * fuel_mass / (dry_mass + fuel_mass);
            let y_fuel = 2.0 * dry_mass / (dry_mass + fuel_mass);
            (
                Vec3::new(0.0, y_dry, 0.0),
                dry_inertia,
                vec![TankSpec {
                    id: 0,
                    max_mass: fuel_mass,
                    mass: fuel_mass,
                    pos: Vec3::new(0.0, y_fuel, 0.0),
                    inertia: cylinder_inertia(fuel_mass, radius, length * 0.7),
                    efficiency: 1.0,
                }],
            )
        } else {
            (Vec3::ZERO, dry_inertia, vec![])
        }
    }

    /// 填充干/燃几何（覆盖 `dry_center`/`dry_inertia`/`tanks`）。
    pub fn with_fuel(mut self, fuel_mass: f64) -> Self {
        let (dc, di, tanks) =
            Self::estimated_mass_props(self.dry_mass, fuel_mass, self.length, self.radius);
        self.dry_center = dc;
        self.dry_inertia = di;
        self.tanks = tanks;
        self
    }

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
        .with_fuel(fuel_mass)
    }

    /// 由推进器规格生成运行时推进器。
    pub fn make_thrusters(&self) -> Vec<Thruster> {
        self.thrusters.iter().map(|s| s.to_thruster()).collect()
    }

    pub fn make_tanks(&self) -> Vec<PropellantTank> {
        self.tanks.iter().map(|t| t.to_tank()).collect()
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
            DockPort::with_rot(Vec3::new(0.0, -half, 0.0), Vec3::new(0.0, -1.0, 0.0), rot),
            DockPort::with_rot(Vec3::new(0.0, half, 0.0), Vec3::new(0.0, 1.0, 0.0), rot),
        ]
    }

    pub fn fuel_mass(&self) -> f64 {
        self.tanks.iter().map(|t| t.mass).sum()
    }

    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.fuel_mass()
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
