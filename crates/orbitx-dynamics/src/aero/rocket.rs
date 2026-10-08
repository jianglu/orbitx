//! 产品火箭气动：体轴筒体 + 逐翼局部迎角升阻。
//!
//! 算法权威见 `orbitx/docs/AERO.md`。不改 [`super::compute_aero_forces`] 语义。
//! Godot / 级体坐标：**+Y 纵轴**。

use orbitx_math::{cross, piecewise_linear, Vec3};

use super::AeroForces;

/// 背风未伸出轮廓时的动压因子（教学级）。
pub const LEEWARD_Q_FACTOR: f64 = 0.5;

/// 固定翼默认失速角 [rad]（亚音速）。
pub const DEFAULT_ALPHA_STALL_FIN: f64 = 18.0_f64.to_radians();
/// 栅格翼默认失速角 [rad]。
pub const DEFAULT_ALPHA_STALL_GRID: f64 = 28.0_f64.to_radians();

/// 三轴投影面积 [m²]：沿体轴 X/Y/Z **看过去**的外轮廓面积（Y=纵轴迎风圆盘）。
#[derive(Clone, Copy, Debug, Default)]
pub struct TriaxialAreas {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 火箭筒体系数（教学级）。
#[derive(Clone, Debug)]
pub struct RocketBodyAero {
    /// 轴向 Cd(M) 表；空则用 `cd0`。
    pub cd_mach: Vec<(f64, f64)>,
    /// 无表时的轴向 Cd。
    pub cd0: f64,
    /// 法向力斜率 CN/α [1/rad]（细长体教学默认）。
    pub cn_alpha: f64,
    /// 俯仰角速度阻尼，对应旧 `rdrag.x`。
    pub pitch_damp: f64,
    /// 偏航角速度阻尼，对应旧 `rdrag.z`。
    pub yaw_damp: f64,
    /// 滚转角速度阻尼，对应旧 `rdrag.y`。
    pub roll_damp: f64,
}

impl Default for RocketBodyAero {
    fn default() -> Self {
        Self {
            cd_mach: Vec::new(),
            cd0: 0.3,
            cn_alpha: 2.0,
            pitch_damp: 1.0,
            yaw_damp: 1.0,
            roll_damp: 0.1,
        }
    }
}

impl RocketBodyAero {
    pub fn cd_at(&self, mach: f64) -> f64 {
        if self.cd_mach.is_empty() {
            self.cd0
        } else {
            piecewise_linear(&self.cd_mach, mach, self.cd0)
        }
    }
}

/// 翼面种类。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FinKind {
    Fixed,
    Grid,
}

/// 一条升力面（已在簇体轴下）。
#[derive(Clone, Debug)]
pub struct LiftingSurface {
    /// 压心（簇体坐标）[m]。
    pub ref_pos: Vec3,
    /// 翼面法向（指向「上」表面，簇体坐标）。
    pub normal: Vec3,
    /// 弦向（前缘→后缘），簇体坐标。
    pub chord_dir: Vec3,
    /// 参考面积 [m²]。
    pub area: f64,
    /// 展弦比。
    pub aspect_ratio: f64,
    /// CL/α [1/rad]。
    pub cl_alpha: f64,
    /// 零升阻力系数（剖面；零升力改由 `edge_area`）。
    pub cd0: f64,
    /// 零升迎风窄缝 [m²] = 厚度 × 展长。
    pub edge_area: f64,
    /// 亚音速失速角 [rad]。
    pub alpha_stall0: f64,
    pub kind: FinKind,
    /// 展收 0..1（栅格）；固定翼恒 1。
    pub deploy: f64,
    /// 是否完全在筒体背风且未伸出包络（由 vessel 几何判定后传入）。
    pub leeward_sheltered: bool,
}

impl LiftingSurface {
    pub fn effective_area(&self) -> f64 {
        self.area * self.deploy.clamp(0.0, 1.0)
    }
}

/// `compute_rocket_aero` 输入（几何已在簇轴）。
#[derive(Clone, Debug)]
pub struct RocketAeroInput<'a> {
    /// 质心体轴空速（船相对大气，前飞 `vy>0`）[m/s]。
    pub airvel_body: Vec3,
    /// 体轴角速度 [rad/s]；翼当地空速用 `airvel + ω×(r−cg)`。
    pub omega_body: Vec3,
    pub rho: f64,
    pub sound_speed: f64,
    pub areas: TriaxialAreas,
    /// 筒体作用点（包络几何中部）[m]。
    pub body_cop: Vec3,
    /// 簇质心 [m]。
    pub cg: Vec3,
    pub body: &'a RocketBodyAero,
    pub surfaces: &'a [LiftingSurface],
}

/// Orbiter `oapiGetWaveDrag` 逐符号。
pub fn wave_drag(mach: f64, m1: f64, m2: f64, m3: f64, cmax: f64) -> f64 {
    if mach < m1 {
        return 0.0;
    }
    if mach < m2 {
        return cmax * (mach - m1) / (m2 - m1);
    }
    if mach < m3 {
        return cmax;
    }
    cmax * ((m3 * m3 - 1.0) / (mach * mach - 1.0)).sqrt()
}

/// 诱导阻力（Orbiter `oapiGetInducedDrag`）。
pub fn induced_drag(cl: f64, aspect_ratio: f64, oswald: f64) -> f64 {
    if aspect_ratio <= 1e-9 || oswald <= 1e-9 {
        return 0.0;
    }
    (cl * cl) / (std::f64::consts::PI * aspect_ratio * oswald)
}

/// 失速角随马赫：亚音速满额，跨音速提前，超音速再降。
pub fn alpha_stall_mach(alpha_stall0: f64, mach: f64) -> f64 {
    if mach < 0.8 {
        alpha_stall0
    } else if mach < 1.2 {
        let t = (mach - 0.8) / 0.4;
        alpha_stall0 * (1.0 - 0.35 * t)
    } else {
        alpha_stall0 * 0.65
    }
}

/// 栅格效率 η(M)：跨音速下降。
pub fn grid_eta(mach: f64) -> f64 {
    if mach < 0.8 {
        1.0
    } else if mach < 1.5 {
        1.0 - 0.4 * (mach - 0.8) / 0.7
    } else {
        0.6
    }
}

/// 展收限速（与 `slew_throttle` 同形）：`deploy` → `deploy_target`，速率 [1/s]。
pub fn slew_deploy(deploy: f64, deploy_target: f64, deploy_rate: f64, dt: f64) -> f64 {
    let cmd = deploy_target.clamp(0.0, 1.0);
    if deploy_rate <= 0.0 {
        return cmd;
    }
    let max_step = deploy_rate * dt;
    (deploy + (cmd - deploy).clamp(-max_step, max_step)).clamp(0.0, 1.0)
}

/// 横向（垂直纵轴）有效侧面积：正/侧两档按 |vx|/|vz| 占比。
pub fn side_area(areas: TriaxialAreas, airvel: Vec3) -> f64 {
    let lat = (airvel.x * airvel.x + airvel.z * airvel.z).sqrt();
    if lat < 1e-12 {
        return 0.5 * (areas.x + areas.z);
    }
    (airvel.x.abs() * areas.x + airvel.z.abs() * areas.z) / lat
}

fn cl_of_alpha(alpha: f64, cl_alpha: f64, alpha_stall: f64) -> f64 {
    let a = alpha.abs();
    let sign = if alpha >= 0.0 { 1.0 } else { -1.0 };
    if a <= alpha_stall {
        return cl_alpha * alpha;
    }
    // 过失速：从峰值线性掉到 ±90° 为 0
    let cl_peak = cl_alpha * alpha_stall;
    let denom = (std::f64::consts::FRAC_PI_2 - alpha_stall).max(1e-6);
    let over = ((a - alpha_stall) / denom).clamp(0.0, 1.0);
    sign * cl_peak * (1.0 - over).max(0.0)
}

/// 局部迎角：弦向 LE→TE；零迎角时 `airvel ≈ −chord`（前飞 `+Y` 配弦向 `−Y`）。
pub fn fin_local_alpha(airvel: Vec3, normal: Vec3, chord_dir: Vec3) -> f64 {
    let n = {
        let l = normal.length();
        if l < 1e-12 {
            return 0.0;
        }
        normal * (1.0 / l)
    };
    let c = {
        let l = chord_dir.length();
        if l < 1e-12 {
            return 0.0;
        }
        chord_dir * (1.0 / l)
    };
    let v_c = orbitx_math::dot(airvel, c);
    let v_n = orbitx_math::dot(airvel, n);
    // α = atan2(v_n, −v_c)：airvel≈−c 时 −v_c>0
    v_n.atan2(-v_c)
}

/// 火箭气动力矩：`τ = (r − cg) × F`（推力、RCS、气动同一叉乘）。
pub fn moment_about_cg(force: Vec3, point: Vec3, cg: Vec3) -> Vec3 {
    cross(point - cg, force)
}

fn axis_sign(v: f64) -> f64 {
    if v > 0.0 {
        1.0
    } else if v < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// 势流法向用 `sin(2α)`；横流系数（Jorgensen 圆截面约 1.2，筒体取 1.1）。
const BODY_CROSSFLOW: f64 = 1.1;
/// 翼面分离法向力系数。
const FIN_SEPARATED: f64 = 1.2;
/// 背风：空速低于此不遮蔽。
const LEEWARD_MIN_SPEED: f64 = 5.0;
/// 背风：侧滑角低于此不遮蔽。
const LEEWARD_MIN_BETA: f64 = 10.0 * std::f64::consts::PI / 180.0;

/// 仅筒体（A1）：体轴轴向 + 法向，作用在 `body_cop`，力矩对 `cg`。
/// 另加三轴角速度阻尼（无翼也生效），不通过压心的 `ω×r`。
pub fn compute_body_aero(
    airvel_body: Vec3,
    omega_body: Vec3,
    rho: f64,
    sound_speed: f64,
    areas: TriaxialAreas,
    body_cop: Vec3,
    cg: Vec3,
    body: &RocketBodyAero,
) -> AeroForces {
    let mut out = AeroForces::default();
    if rho < 1e-15 {
        return out;
    }
    let speed = airvel_body.length();
    if speed < 1e-6 {
        return out;
    }
    let q = 0.5 * rho * speed * speed;
    out.dynamic_pressure = q;
    let a = sound_speed.max(1.0);
    let mach = speed / a;
    out.mach = mach;

    // 轴向：Cd(M)·½ρ·vy²·Sy（Allen / Jorgensen，全迎角同一式）。
    let cd = body.cd_at(mach);
    let vy = airvel_body.y;
    let f_axial_mag = cd * 0.5 * rho * vy * vy * areas.y;
    let f_y = -axis_sign(vy) * f_axial_mag;

    // α = atan2(|v_lat|, |vy|)。势流 (cn_alpha/2)·q·Sy·sin(2α)；横流 1.1·½ρ·|v_lat|²·S_lat。
    let v_lat = Vec3::new(airvel_body.x, 0.0, airvel_body.z);
    let v_lat_mag = v_lat.length();
    let alpha = v_lat_mag.atan2(vy.abs().max(1e-12));
    let f_pot = (body.cn_alpha * 0.5) * q * areas.y * (2.0 * alpha).sin();
    let s_lat = side_area(areas, airvel_body);
    let f_cross = BODY_CROSSFLOW * 0.5 * rho * v_lat_mag * v_lat_mag * s_lat;
    let f_n_mag = f_pot + f_cross;
    let f_lat = if v_lat_mag > 1e-12 {
        v_lat * (-f_n_mag / v_lat_mag)
    } else {
        Vec3::ZERO
    };

    let force = Vec3::new(f_lat.x, f_y, f_lat.z);
    out.force = force;
    out.drag_force = f_axial_mag;
    out.lift_force = f_n_mag;
    out.cd_eff = cd;
    out.torque = moment_about_cg(force, body_cop, cg);
    // 俯仰 ω_x → YZ 投影 Sx；偏航 ω_z → XY 投影 Sz；滚转 → 迎风 Sy。
    out.torque.x -= q * areas.x * body.pitch_damp * omega_body.x;
    out.torque.z -= q * areas.z * body.yaw_damp * omega_body.z;
    out.torque.y -= q * areas.y * body.roll_damp * omega_body.y;
    out
}


/// 背风遮蔽：`|V|≥5 m/s` 且 `β≥10°` 时，下风侧且压心未伸出 `R_body` → 动压 ×1/2。
///
/// - `body_radius_at_y(y)`：该站位筒体截面外半径；伸出则满算。
/// - 不用翼法向当可见度。
pub fn update_leeward_sheltered(
    surfaces: &mut [LiftingSurface],
    airvel_body: Vec3,
    body_radius_at_y: impl Fn(f64) -> f64,
) {
    let v_lat = Vec3::new(airvel_body.x, 0.0, airvel_body.z);
    let v_lat_mag = v_lat.length();
    let speed = airvel_body.length();
    let beta = v_lat_mag.atan2(airvel_body.y.abs());
    if speed < LEEWARD_MIN_SPEED || beta < LEEWARD_MIN_BETA || v_lat_mag < 1e-12 {
        for s in surfaces.iter_mut() {
            s.leeward_sheltered = false;
        }
        return;
    }
    let down = v_lat * (1.0 / v_lat_mag);
    for s in surfaces.iter_mut() {
        let r_xz = Vec3::new(s.ref_pos.x, 0.0, s.ref_pos.z);
        let rad = (r_xz.x * r_xz.x + r_xz.z * r_xz.z).sqrt();
        let lee = orbitx_math::dot(r_xz, down) > 0.0;
        let r_body = body_radius_at_y(s.ref_pos.y);
        s.leeward_sheltered = lee && rad <= r_body + 1e-9;
    }
}
struct FinForceSplit {
    force: Vec3,
    lift: f64,
    drag: f64,
}

fn fin_force_split(surf: &LiftingSurface, airvel: Vec3, q: f64, mach: f64) -> FinForceSplit {
    let zero = FinForceSplit {
        force: Vec3::ZERO,
        lift: 0.0,
        drag: 0.0,
    };
    let deploy = surf.deploy.clamp(0.0, 1.0);
    let area = surf.area * deploy;
    let edge = surf.edge_area * deploy;
    if (area < 1e-12 && edge < 1e-12) || q < 1e-18 {
        return zero;
    }
    let q_eff = if surf.leeward_sheltered {
        q * LEEWARD_Q_FACTOR
    } else {
        q
    };

    let alpha = fin_local_alpha(airvel, surf.normal, surf.chord_dir);
    let mut a_stall = alpha_stall_mach(surf.alpha_stall0, mach);
    let mut cl_a = surf.cl_alpha;
    if surf.kind == FinKind::Grid {
        let eta = grid_eta(mach);
        cl_a *= eta;
        a_stall = alpha_stall_mach(surf.alpha_stall0.max(DEFAULT_ALPHA_STALL_GRID), mach);
    }
    let cl = cl_of_alpha(alpha, cl_a, a_stall);
    let cd_plan = induced_drag(cl, surf.aspect_ratio.max(0.1), 0.7)
        + wave_drag(mach, 0.75, 1.0, 1.1, 0.04);

    let n = {
        let l = surf.normal.length();
        if l < 1e-12 {
            return zero;
        }
        surf.normal * (1.0 / l)
    };
    let speed = airvel.length();
    let vhat = if speed > 1e-12 {
        airvel * (1.0 / speed)
    } else {
        Vec3::ZERO
    };
    // 低压侧：n_lee = −sign(空速·法向)·法向。附着升力取其中垂直于当地空速的分量。
    let vn = orbitx_math::dot(airvel, n);
    let n_lee = n * (-axis_sign(vn));
    let n_lift = n_lee - vhat * orbitx_math::dot(n_lee, vhat);
    let attached = cl.abs() * q_eff * area;
    let separated = FIN_SEPARATED * alpha.sin().powi(2) * q_eff * area;
    let lift_force = n_lift * attached + n_lee * separated;
    let ddir = vhat * -1.0;
    let drag = q_eff * edge * alpha.cos().abs() + cd_plan * q_eff * area;
    let drag_force = ddir * drag;
    let force = lift_force + drag_force;
    FinForceSplit {
        force,
        lift: attached + separated,
        drag,
    }
}

fn surface_force(
    surf: &LiftingSurface,
    airvel: Vec3,
    q: f64,
    mach: f64,
) -> (Vec3, f64, f64) {
    let s = fin_force_split(surf, airvel, q, mach);
    (s.force, s.lift, s.drag)
}

/// 完整火箭气动（筒体 + 翼）。
pub fn compute_rocket_aero(input: &RocketAeroInput<'_>) -> AeroForces {
    let mut out = compute_body_aero(
        input.airvel_body,
        input.omega_body,
        input.rho,
        input.sound_speed,
        input.areas,
        input.body_cop,
        input.cg,
        input.body,
    );
    if input.rho < 1e-15 {
        return out;
    }
    let speed = input.airvel_body.length();
    if speed < 1e-6 {
        return out;
    }
    let q = out.dynamic_pressure;
    let mach = out.mach;

    let mut lift_sum = out.lift_force;
    let mut drag_sum = out.drag_force;
    for s in input.surfaces {
        let v_pt = input.airvel_body + cross(input.omega_body, s.ref_pos - input.cg);
        let (f, lift, drag) = surface_force(s, v_pt, q, mach);
        out.force += f;
        out.torque += moment_about_cg(f, s.ref_pos, input.cg);
        lift_sum += lift;
        drag_sum += drag;
    }
    out.lift_force = lift_sum;
    out.drag_force = drag_sum;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // 下列 A1–A3 测用 chord=+Y 且 airvel.y<0（与产品弦向相反的成对约定，公式内部自洽）。
    // 产品步进是 airvel.y>0、chord=−Y，见 a12_*。

    fn body_default() -> RocketBodyAero {
        RocketBodyAero {
            cd_mach: vec![(0.0, 0.3), (1.0, 0.45), (5.0, 0.35)],
            cd0: 0.3,
            cn_alpha: 2.0,
            ..RocketBodyAero::default()
        }
    }

    fn areas_slender() -> TriaxialAreas {
        // L=20, R=1 → frontal π, side ≈ 40
        TriaxialAreas {
            x: 40.0,
            y: std::f64::consts::PI,
            z: 40.0,
        }
    }

    #[test]
    fn a1_zero_alpha_axial_only() {
        // 前进：空气相对船 −Y
        let airvel = Vec3::new(0.0, -100.0, 0.0);
        let cop = Vec3::new(0.0, 5.0, 0.0);
        let cg = Vec3::new(0.0, 2.0, 0.0);
        let body = body_default();
        let r = compute_body_aero(airvel, Vec3::ZERO, 1.225, 340.0, areas_slender(), cop, cg, &body);
        assert!(r.force.x.abs() < 1e-6, "no side force {:?}", r.force);
        assert!(r.force.z.abs() < 1e-6, "no side force {:?}", r.force);
        assert!(r.force.y > 0.0, "axial drag should push aft (+Y if flow −Y) got {}", r.force.y);
        // 力矩臂 (cop-cg)= (0,3,0)；F=(0,Fy,0) → F×r = 0
        assert!(r.torque.length() < 1e-6, "collinear arm {:?}", r.torque);
    }

    #[test]
    fn a1_alpha_grows_normal() {
        let body = body_default();
        let areas = areas_slender();
        let cop = Vec3::ZERO;
        let cg = Vec3::ZERO;
        let r0 = compute_body_aero(Vec3::new(0.0, -100.0, 0.0), Vec3::ZERO, 1.225, 340.0, areas, cop, cg, &body);
        let r1 = compute_body_aero(Vec3::new(30.0, -100.0, 0.0), Vec3::ZERO, 1.225, 340.0, areas, cop, cg, &body);
        assert!(r1.lift_force > r0.lift_force + 1.0, "{} vs {}", r1.lift_force, r0.lift_force);
        assert!(r1.force.x.abs() > 1.0);
    }

    #[test]
    fn a1_small_alpha_potential_and_ninety_crossflow() {
        let body = body_default();
        let areas = areas_slender();
        let rho = 1.225;
        let speed = 100.0;
        let alpha = 0.02_f64;
        let airvel = Vec3::new(speed * alpha.sin(), -speed * alpha.cos(), 0.0);
        let r = compute_body_aero(
            airvel,
            Vec3::ZERO,
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        let q = 0.5 * rho * speed * speed;
        let a = airvel.x.abs().atan2(airvel.y.abs());
        let f_pot = (body.cn_alpha * 0.5) * q * areas.y * (2.0 * a).sin();
        let v_lat = airvel.x.abs();
        let f_cross = 1.1 * 0.5 * rho * v_lat * v_lat * areas.x;
        let expect = f_pot + f_cross;
        assert!(
            (r.lift_force - expect).abs() / expect < 1e-6,
            "{} vs {}",
            r.lift_force,
            expect
        );
        let slope = 2.0 * a * q * areas.y;
        assert!((f_pot - slope).abs() / slope < 0.02, "pot {f_pot} slope {slope}");

        let side = Vec3::new(0.0, 0.0, speed);
        let r90 = compute_body_aero(
            side,
            Vec3::ZERO,
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        let expect90 = 1.1 * 0.5 * rho * speed * speed * areas.z;
        assert!(r90.force.y.abs() < 1e-4, "no axial at 90 {:?}", r90.force);
        assert!(
            (r90.force.z + expect90).abs() / expect90 < 1e-6,
            "{:?} vs {}",
            r90.force,
            expect90
        );
    }

    #[test]
    fn a1_aft_cp_weathervanes_into_flow() {
        let body = body_default();
        let airvel = Vec3::new(0.0, 100.0, 20.0);
        let r = compute_body_aero(
            airvel,
            Vec3::ZERO,
            1.225,
            340.0,
            areas_slender(),
            Vec3::new(0.0, -4.0, 0.0),
            Vec3::ZERO,
            &body,
        );
        assert!(
            r.torque.x * airvel.z > 0.0,
            "aft CP should turn nose toward +Z {:?}",
            r.torque
        );
    }

    #[test]
    fn a1_moment_arm_envelope_minus_cg() {
        let body = body_default();
        let airvel = Vec3::new(20.0, -80.0, 0.0);
        let cop = Vec3::new(0.0, 10.0, 0.0);
        let cg = Vec3::new(0.0, 0.0, 0.0);
        let r = compute_body_aero(airvel, Vec3::ZERO, 1.225, 340.0, areas_slender(), cop, cg, &body);
        // r×F：r=(0,10,0) → torque.z = −Fx·10
        let expect_z = -r.force.x * 10.0;
        assert!((r.torque.z - expect_z).abs() < 1e-6, "{:?} vs {}", r.torque, expect_z);
    }

    #[test]
    fn a1_tandem_frontal_is_one_disk() {
        // 包络面积已是「最前一圈」；加权不把两级圆盘相加——由调用方保证 Sy 为包络
        let a = TriaxialAreas {
            x: 40.0,
            y: std::f64::consts::PI,
            z: 40.0,
        };
        let double = TriaxialAreas {
            x: 40.0,
            y: 2.0 * std::f64::consts::PI,
            z: 40.0,
        };
        let body = body_default();
        let airvel = Vec3::new(0.0, -100.0, 0.0);
        let r1 = compute_body_aero(airvel, Vec3::ZERO, 1.225, 340.0, a, Vec3::ZERO, Vec3::ZERO, &body);
        let r2 = compute_body_aero(airvel, Vec3::ZERO, 1.225, 340.0, double, Vec3::ZERO, Vec3::ZERO, &body);
        assert!(r2.drag_force > r1.drag_force * 1.5);
    }

    #[test]
    fn wave_drag_matches_orbiter_shape() {
        assert_eq!(wave_drag(0.5, 0.75, 1.0, 1.1, 0.04), 0.0);
        let mid = wave_drag(0.875, 0.75, 1.0, 1.1, 0.04);
        assert!((mid - 0.02).abs() < 1e-9);
        assert!((wave_drag(1.05, 0.75, 1.0, 1.1, 0.04) - 0.04).abs() < 1e-9);
    }

    #[test]
    fn a2_fin_lift_at_small_alpha() {
        let body = body_default();
        // 翼在 XZ：法向 +Z，弦向 +Y → 侧风产生迎角
        let surf = LiftingSurface {
            ref_pos: Vec3::new(1.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            chord_dir: Vec3::new(0.0, 1.0, 0.0),
            area: 2.0,
            aspect_ratio: 2.0,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
            kind: FinKind::Fixed,
            deploy: 1.0,
            leeward_sheltered: false,
        };
        let airvel = Vec3::new(0.0, -100.0, 20.0);
        let input = RocketAeroInput {
            airvel_body: airvel,
            omega_body: Vec3::ZERO,
            rho: 1.225,
            sound_speed: 340.0,
            areas: areas_slender(),
            body_cop: Vec3::ZERO,
            cg: Vec3::ZERO,
            body: &body,
            surfaces: &[surf.clone()],
        };
        let with = compute_rocket_aero(&input);
        let without = compute_body_aero(
            airvel,
            Vec3::ZERO,
            1.225,
            340.0,
            areas_slender(),
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        assert!(
            (with.force - without.force).length() > 10.0,
            "fin should add force {:?} vs {:?}",
            with.force,
            without.force
        );
    }

    #[test]
    fn a2_zero_alpha_drag_is_edge_and_ninety_is_separated() {
        let mut surf = fin_prod(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0));
        surf.area = 2.22;
        surf.edge_area = 0.084;
        let q = 4000.0;
        let (f0, lift0, drag0) = surface_force(&surf, Vec3::new(0.0, 100.0, 0.0), q, 0.2);
        assert!(lift0 < 1e-6, "zero alpha lift {lift0}");
        assert!((drag0 - q * surf.edge_area).abs() < 1e-6, "drag {drag0}");
        assert!(f0.y < 0.0, "drag opposes +Y {:?}", f0);

        let (f90, lift90, _) = surface_force(&surf, Vec3::new(0.0, 0.0, 100.0), q, 0.2);
        let expect = 1.2 * q * surf.area;
        assert!((lift90 - expect).abs() / expect < 1e-6, "sep {lift90}");
        assert!(
            (f90.z + expect).abs() / expect < 1e-3,
            "90° force anti-flow {:?}",
            f90
        );
    }

    #[test]
    fn a2_small_sideslip_toward_low_pressure() {
        let surf = fin_prod(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0));
        let (f, _, _) = surface_force(&surf, Vec3::new(0.0, 100.0, 5.0), 5000.0, 0.2);
        assert!(f.z < 0.0, "sideslip +Z, force toward −Z {:?}", f);
    }

    #[test]
    fn a2_stall_drops_attached_before_ninety() {
        let surf = LiftingSurface {
            ref_pos: Vec3::ZERO,
            normal: Vec3::new(0.0, 0.0, 1.0),
            chord_dir: Vec3::new(0.0, 1.0, 0.0),
            area: 2.0,
            aspect_ratio: 2.0,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: 10.0_f64.to_radians(),
            kind: FinKind::Fixed,
            deploy: 1.0,
            leeward_sheltered: false,
        };
        let (f_lo, lift_lo, _) =
            surface_force(&surf, Vec3::new(0.0, -100.0, 5.0), 5000.0, 0.3);
        // ~87° 迎角，过失速应明显低于小迎角峰值
        let (f_hi, lift_hi, _) =
            surface_force(&surf, Vec3::new(0.0, -10.0, 200.0), 5000.0, 0.3);
        let a_lo = fin_local_alpha(Vec3::new(0.0, -100.0, 5.0), surf.normal, surf.chord_dir);
        let a_hi = fin_local_alpha(Vec3::new(0.0, -10.0, 200.0), surf.normal, surf.chord_dir);
        let cl_lo = cl_of_alpha(a_lo, surf.cl_alpha, surf.alpha_stall0).abs();
        let cl_hi = cl_of_alpha(a_hi, surf.cl_alpha, surf.alpha_stall0).abs();
        assert!(lift_lo > 1.0, "{lift_lo}");
        assert!(cl_hi < cl_lo * 0.5, "attached stall {cl_hi} vs {cl_lo}");
        assert!(lift_hi > lift_lo, "separated normal grows {lift_hi} vs {lift_lo}");
        let _ = (f_lo, f_hi);
    }

    #[test]
    fn a3_leeward_halves_q() {
        let mut surf = LiftingSurface {
            ref_pos: Vec3::ZERO,
            normal: Vec3::new(0.0, 0.0, 1.0),
            chord_dir: Vec3::new(0.0, 1.0, 0.0),
            area: 2.0,
            aspect_ratio: 2.0,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
            kind: FinKind::Fixed,
            deploy: 1.0,
            leeward_sheltered: false,
        };
        let airvel = Vec3::new(0.0, -100.0, 20.0);
        let (_, lift_full, drag_full) = surface_force(&surf, airvel, 5000.0, 0.3);
        surf.leeward_sheltered = true;
        let (_, lift_lee, drag_lee) = surface_force(&surf, airvel, 5000.0, 0.3);
        assert!((lift_lee * 2.0 - lift_full).abs() / lift_full < 1e-9);
        assert!((drag_lee * 2.0 - drag_full).abs() / drag_full.max(1e-9) < 1e-9);
    }

    #[test]
    fn a3_slew_deploy_rate() {
        let d = slew_deploy(0.0, 1.0, 0.5, 0.1);
        assert!((d - 0.05).abs() < 1e-12);
        let d = slew_deploy(0.0, 1.0, 0.0, 0.1);
        assert_eq!(d, 1.0);
        let mut x = 0.0;
        for _ in 0..20 {
            x = slew_deploy(x, 1.0, 1.0, 0.1);
        }
        assert!((x - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a3_deploy_zero_kills_fin_force() {
        let surf = LiftingSurface {
            ref_pos: Vec3::ZERO,
            normal: Vec3::new(0.0, 0.0, 1.0),
            chord_dir: Vec3::new(0.0, 1.0, 0.0),
            area: 2.0,
            aspect_ratio: 2.0,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
            kind: FinKind::Grid,
            deploy: 0.0,
            leeward_sheltered: false,
        };
        let (f, _, _) = surface_force(&surf, Vec3::new(0.0, -100.0, 20.0), 5000.0, 0.3);
        assert!(f.length() < 1e-9);
    }

    fn fin_prod(ref_pos: Vec3, normal: Vec3) -> LiftingSurface {
        LiftingSurface {
            ref_pos,
            normal,
            chord_dir: Vec3::new(0.0, -1.0, 0.0),
            area: 1.0,
            aspect_ratio: 2.0,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
            kind: FinKind::Fixed,
            deploy: 1.0,
            leeward_sheltered: false,
        }
    }

    fn cruciform_four() -> [LiftingSurface; 4] {
        let s = 2.0;
        [
            fin_prod(Vec3::new(s, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
            fin_prod(Vec3::new(-s, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0)),
            fin_prod(Vec3::new(0.0, 0.0, s), Vec3::new(1.0, 0.0, 0.0)),
            fin_prod(Vec3::new(0.0, 0.0, -s), Vec3::new(-1.0, 0.0, 0.0)),
        ]
    }

    #[test]
    fn a12_prod_chord_small_alpha() {
        let a0 = fin_local_alpha(
            Vec3::new(0.0, 100.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, -1.0, 0.0),
        );
        assert!(a0.abs() < 1e-12, "{a0}");
        let a = fin_local_alpha(
            Vec3::new(0.0, 100.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, -1.0, 0.0),
        );
        assert!(a.abs() < 0.1, "expected small α, got {a}");
        assert!((a - 5.0_f64.atan2(100.0)).abs() < 1e-12);
    }

    #[test]
    fn a12_cruciform_zero_omega_no_roll_torque() {
        let body = body_default();
        let surfaces = cruciform_four();
        let one = [surfaces[0].clone()];
        let airvel = Vec3::new(0.0, 100.0, 8.0);
        let input_four = RocketAeroInput {
            airvel_body: airvel,
            omega_body: Vec3::ZERO,
            rho: 1.225,
            sound_speed: 340.0,
            areas: areas_slender(),
            body_cop: Vec3::ZERO,
            cg: Vec3::ZERO,
            body: &body,
            surfaces: &surfaces,
        };
        let four = compute_rocket_aero(&input_four);
        let one_aero = compute_rocket_aero(&RocketAeroInput {
            surfaces: &one,
            ..input_four
        });
        let one_ty = one_aero.torque.y.abs().max(one_aero.torque.length());
        assert!(
            four.torque.y.abs() < 1e-6 + 1e-3 * one_ty.max(1.0),
            "net τy {} vs single-fin scale {}",
            four.torque.y,
            one_ty
        );
    }

    #[test]
    fn a12_cruciform_roll_rate_damps() {
        let body = body_default();
        let surfaces = cruciform_four();
        let airvel = Vec3::new(0.0, 100.0, 0.0);
        let base = RocketAeroInput {
            airvel_body: airvel,
            omega_body: Vec3::ZERO,
            rho: 1.225,
            sound_speed: 340.0,
            areas: areas_slender(),
            body_cop: Vec3::ZERO,
            cg: Vec3::ZERO,
            body: &body,
            surfaces: &surfaces,
        };
        let t0 = compute_rocket_aero(&base);
        let spinning = compute_rocket_aero(&RocketAeroInput {
            omega_body: Vec3::new(0.0, 1.0, 0.0),
            ..base
        });
        assert!(
            spinning.torque.y * 1.0 < -1.0,
            "τy should oppose +ωy, got {}",
            spinning.torque.y
        );
        assert!(
            spinning.torque.y.abs() > t0.torque.y.abs() + 10.0,
            "damping |τy| {} vs zero-ω {}",
            spinning.torque.y.abs(),
            t0.torque.y.abs()
        );
    }

    #[test]
    fn body_sideslip_force_opposes_and_dissipates() {
        let airvel = Vec3::new(0.0, 200.0, 40.0);
        let r = compute_body_aero(
            airvel,
            Vec3::ZERO,
            1.225,
            340.0,
            areas_slender(),
            Vec3::ZERO,
            Vec3::ZERO,
            &body_default(),
        );
        let power = orbitx_math::dot(r.force, airvel);
        assert!(power <= 1e-6, "aero power should dissipate, got {power}");
        assert!(
            r.force.z * airvel.z < 0.0,
            "Fz should oppose vz, Fz={} vz={}",
            r.force.z,
            airvel.z
        );
    }

    #[test]
    fn fin_force_dissipates_toward_low_pressure() {
        let surf = LiftingSurface {
            ref_pos: Vec3::ZERO,
            normal: Vec3::new(0.0, 0.0, 1.0),
            chord_dir: Vec3::new(0.0, -1.0, 0.0),
            area: 2.0,
            aspect_ratio: 2.0,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
            kind: FinKind::Fixed,
            deploy: 1.0,
            leeward_sheltered: false,
        };
        let airvel = Vec3::new(0.0, 100.0, 8.0);
        let (f, _, _) = surface_force(&surf, airvel, 5000.0, 0.3);
        let power = orbitx_math::dot(f, airvel);
        assert!(power < 0.0, "net fin force should dissipate, F·v={power}");
        assert!(f.z < -1.0, "low-pressure side force {:?}", f);
    }

    #[test]
    fn body_rate_damping_opposes_each_axis() {
        let areas = areas_slender();
        let body = body_default();
        let airvel = Vec3::new(0.0, 100.0, 0.0);
        let rho = 1.225;
        let q = 0.5 * rho * 100.0 * 100.0;
        let cop = Vec3::new(0.0, 5.0, 0.0);
        let cg = Vec3::new(0.0, 2.0, 0.0);
        let still = compute_body_aero(airvel, Vec3::ZERO, rho, 340.0, areas, cop, cg, &body);
        assert!(
            still.torque.length() < 1e-6,
            "zero ω torque {:?}",
            still.torque
        );

        let wx = 0.4;
        let pitch = compute_body_aero(
            airvel,
            Vec3::new(wx, 0.0, 0.0),
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        let expect_x = -q * areas.x * body.pitch_damp * wx;
        assert!(
            (pitch.torque.x - expect_x).abs() < 1e-6,
            "τx {} vs {}",
            pitch.torque.x,
            expect_x
        );
        assert!(pitch.torque.y.abs() < 1e-8 && pitch.torque.z.abs() < 1e-8);

        let wz = 0.25;
        let yaw = compute_body_aero(
            airvel,
            Vec3::new(0.0, 0.0, wz),
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        let expect_z = -q * areas.z * body.yaw_damp * wz;
        assert!(
            (yaw.torque.z - expect_z).abs() < 1e-6,
            "τz {} vs {}",
            yaw.torque.z,
            expect_z
        );
        assert!(yaw.torque.x.abs() < 1e-8 && yaw.torque.y.abs() < 1e-8);

        let wy = 1.5;
        let roll = compute_body_aero(
            airvel,
            Vec3::new(0.0, wy, 0.0),
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        let expect_y = -q * areas.y * body.roll_damp * wy;
        assert!(
            (roll.torque.y - expect_y).abs() < 1e-6,
            "τy {} vs {}",
            roll.torque.y,
            expect_y
        );
        assert!(roll.torque.x.abs() < 1e-8 && roll.torque.z.abs() < 1e-8);

        let twice_rate = compute_body_aero(
            airvel,
            Vec3::new(wx * 2.0, 0.0, 0.0),
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        assert!((twice_rate.torque.x - 2.0 * pitch.torque.x).abs() < 1e-6);

        let twice_q = compute_body_aero(
            airvel,
            Vec3::new(wx, 0.0, 0.0),
            rho * 2.0,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        assert!((twice_q.torque.x - 2.0 * pitch.torque.x).abs() < 1e-4);

        let vacuum = compute_body_aero(
            airvel,
            Vec3::new(wx, wy, wz),
            0.0,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        assert_eq!(vacuum.torque, Vec3::ZERO);
    }

    #[test]
    fn body_rate_damping_uses_facing_area_when_sx_ne_sz() {
        let areas = TriaxialAreas {
            x: 10.0,
            y: std::f64::consts::PI,
            z: 40.0,
        };
        let body = body_default();
        let airvel = Vec3::new(0.0, 100.0, 0.0);
        let rho = 1.225;
        let q = 0.5 * rho * 100.0 * 100.0;
        let wx = 0.5;
        let pitch = compute_body_aero(
            airvel,
            Vec3::new(wx, 0.0, 0.0),
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        // 俯仰应乘 Sx，不是 Sz
        assert!((pitch.torque.x - (-q * areas.x * body.pitch_damp * wx)).abs() < 1e-6);
        assert!((pitch.torque.x - (-q * areas.z * body.pitch_damp * wx)).abs() > 1.0);

        let wz = 0.5;
        let yaw = compute_body_aero(
            airvel,
            Vec3::new(0.0, 0.0, wz),
            rho,
            340.0,
            areas,
            Vec3::ZERO,
            Vec3::ZERO,
            &body,
        );
        assert!((yaw.torque.z - (-q * areas.z * body.yaw_damp * wz)).abs() < 1e-6);
        assert!((yaw.torque.z - (-q * areas.x * body.yaw_damp * wz)).abs() > 1.0);
    }

    #[test]
    fn leeward_runtime_downwind_sheltered_upwind_and_extended_not() {
        let mut surfs = [
            LiftingSurface {
                ref_pos: Vec3::new(0.5, 0.0, 0.0), // 下风、未伸出 R=1
                normal: Vec3::new(0.0, 0.0, 1.0),
                chord_dir: Vec3::new(0.0, -1.0, 0.0),
                area: 1.0,
                aspect_ratio: 2.0,
                cl_alpha: 3.5,
                cd0: 0.02,
                edge_area: 0.0,
                alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
                kind: FinKind::Fixed,
                deploy: 1.0,
                leeward_sheltered: false,
            },
            LiftingSurface {
                ref_pos: Vec3::new(-0.5, 0.0, 0.0), // 上风
                normal: Vec3::new(0.0, 0.0, 1.0),
                chord_dir: Vec3::new(0.0, -1.0, 0.0),
                area: 1.0,
                aspect_ratio: 2.0,
                cl_alpha: 3.5,
                cd0: 0.02,
                edge_area: 0.0,
                alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
                kind: FinKind::Fixed,
                deploy: 1.0,
                leeward_sheltered: false,
            },
            LiftingSurface {
                ref_pos: Vec3::new(2.0, 0.0, 0.0), // 下风但伸出
                normal: Vec3::new(0.0, 0.0, 1.0),
                chord_dir: Vec3::new(0.0, -1.0, 0.0),
                area: 1.0,
                aspect_ratio: 2.0,
                cl_alpha: 3.5,
                cd0: 0.02,
                edge_area: 0.0,
                alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
                kind: FinKind::Fixed,
                deploy: 1.0,
                leeward_sheltered: false,
            },
        ];
        let airvel = Vec3::new(20.0, 100.0, 0.0);
        update_leeward_sheltered(&mut surfs, airvel, |_| 1.0);
        assert!(surfs[0].leeward_sheltered);
        assert!(!surfs[1].leeward_sheltered);
        assert!(!surfs[2].leeward_sheltered);

        update_leeward_sheltered(&mut surfs, Vec3::new(0.0, 100.0, 0.0), |_| 1.0);
        assert!(!surfs[0].leeward_sheltered);
        assert!(!surfs[1].leeward_sheltered);
        assert!(!surfs[2].leeward_sheltered);
    }

    #[test]
    fn leeward_small_beta_and_low_speed_unsheltered() {
        let mut surfs = [LiftingSurface {
            ref_pos: Vec3::new(0.5, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            chord_dir: Vec3::new(0.0, -1.0, 0.0),
            area: 1.0,
            aspect_ratio: 2.0,
            cl_alpha: 3.5,
            cd0: 0.02,
            edge_area: 0.0,
            alpha_stall0: DEFAULT_ALPHA_STALL_FIN,
            kind: FinKind::Fixed,
            deploy: 1.0,
            leeward_sheltered: true,
        }];
        update_leeward_sheltered(&mut surfs, Vec3::new(1.75, 100.0, 0.0), |_| 1.0);
        assert!(!surfs[0].leeward_sheltered, "≈1° must not shelter");
        update_leeward_sheltered(&mut surfs, Vec3::new(2.828, 2.828, 0.0), |_| 1.0);
        assert!(!surfs[0].leeward_sheltered, "|V|<5 must not shelter");
    }
}
