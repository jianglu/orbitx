//! 火箭级定义：干质量、燃料、推力、比冲、惯量、TVC 等静态参数。

use crate::dock::DockPort;
use crate::thruster::Thruster;
use orbitx_math::Vec3;

/// PMI"未定义"哨兵值（任意分量 <0），对应 Orbiter `Rigidbody.cpp` 中
/// `pmi.Set(-1,-1,-1)`。实际 PMI 由 [`Vessel::from_spec`] 用
/// [`default_pmi`] 推断。
pub const PMI_UNDEF: Vec3 = Vec3::new(-1.0, -1.0, -1.0);

/// 默认 PMI（Orbiter `Vessel.cpp:733-735`：`pmi<0` 时按 `size` 推断）。
///
/// Orbiter 的 PMI 是**归一化主惯量**（真实惯量 / 质量），单位 m²。这样
/// `EulerInv(τ/mass, ω, pmi)` 的量纲为 `(N·m/kg) / m² = 1/s²`。
/// 这里把火箭视为细长圆柱体（轴向 Y），用圆柱体公式除以质量：
/// 横向 `I/m = (3r² + h²)/12`，轴向 `I/m = r²/2`。
pub fn default_pmi(radius: f64, length: f64, mass: f64) -> Vec3 {
    if mass > 0.0 {
        // 归一化惯量（除以质量），单位 m²。
        let i_axial = 0.5 * radius * radius;
        let i_trans = (3.0 * radius * radius + length * length) / 12.0;
        Vec3::new(i_trans, i_axial, i_trans)
    } else {
        Vec3::new(2.0 * radius, radius, 2.0 * radius)
    }
}

/// 火箭级的静态参数，用于初始化 Vessel。
///
/// stages[0] = 底层级（第一级），最后一位 = 有效载荷。
///
/// `Default` 提供安全的零值（无燃料/推力/TVC，PMI 未定义），便于字面量
/// 用 `..StageSpec::default()` 省略不关心的字段。
#[derive(Clone, Debug, Default)]
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
    /// 主惯量张量（体坐标系对角线）[kg·m²]。`PMI_UNDEF` 表示由
    /// [`default_pmi`] 用圆柱体公式推断。
    pub pmi: Vec3,
    /// TVC 最大偏转角 [rad]。0 = 无矢量控制。
    pub max_gimbal: f64,
    /// TVC 最大偏转角速率 [rad/s]。0 = 无限制。
    pub max_gimbal_rate: f64,
    /// TVC 偏转轴（体坐标系）。默认 X 轴（俯仰方向）。
    pub gimbal_axis: Vec3,
}

impl StageSpec {
    /// 生成该级的推进器列表（带 TVC 参数）。
    pub fn make_thrusters(&self) -> Vec<Thruster> {
        if self.thrust > 0.0 {
            let t = Thruster::new(self.engine_pos, self.engine_dir, self.thrust, self.isp)
                .with_tvc(self.max_gimbal, self.max_gimbal_rate, self.gimbal_axis);
            vec![t]
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

    /// 该级的有效归一化 PMI（单位 m²）。
    ///
    /// 若 [`pmi`](Self::pmi) 已定义（来自配置的 `inertia` 字段，视为**真实
    /// 惯量** kg·m²），则除以质量归一化；否则用 [`default_pmi`]（已归一化）
    /// 推断。归一化后与 Orbiter 的 PMI 约定一致，可直接用于 `euler_inv`。
    pub fn effective_pmi(&self) -> Vec3 {
        let m = self.total_mass();
        if self.pmi.x > 0.0 && self.pmi.y > 0.0 && self.pmi.z > 0.0 && m > 0.0 {
            // 配置填的是真实惯量（kg·m²），归一化为 m²。
            Vec3::new(self.pmi.x / m, self.pmi.y / m, self.pmi.z / m)
        } else {
            default_pmi(self.radius, self.length, m)
        }
    }
}

