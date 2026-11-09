//! 姿态运动学：由 `StateVectors` 导出姿态角（俯仰/偏航/滚转/tip）。
//!
//! 移自 vessel::attitude；属状态级运动学，归 dynamics。输入为纯数学类型
//! `StateVectors`，不依赖任何 vessel 类型。

use orbitx_math::{cross, dot, mul, tmul, Matrix3, StateVectors, Vec3};

/// 有符号俯仰/偏航角 [rad]（体轴相对当地径向）。
pub fn pitch_yaw_angles(state: &StateVectors) -> (f64, f64) {
    let (sp, sy) = attitude_errors(state);
    (sp.clamp(-1.0, 1.0).asin(), sy.clamp(-1.0, 1.0).asin())
}

/// 俯仰/偏航误差（径向在体坐标的分量，未 asin）。
pub fn attitude_errors(state: &StateVectors) -> (f64, f64) {
    let r_mag = state.pos.length();
    if r_mag < 1e-3 {
        return (0.0, 0.0);
    }
    let radial = state.pos * (1.0 / r_mag);
    let radial_body = tmul(state.r, radial);
    (radial_body.z, -radial_body.x)
}

/// 体 +Y 与径向夹角 [rad]。
pub fn tip_angle(state: &StateVectors) -> f64 {
    let r_mag = state.pos.length();
    if r_mag < 1e-3 {
        return 0.0;
    }
    let radial = state.pos * (1.0 / r_mag);
    let body_y = mul(state.r, Vec3::new(0.0, 1.0, 0.0));
    dot(body_y, radial).clamp(-1.0, 1.0).acos()
}

/// 绕体 +Y 滚转角 [rad]（当地东向参考）。
pub fn roll_angle(state: &StateVectors) -> f64 {
    let r_mag = state.pos.length();
    if r_mag < 1e-3 {
        return 0.0;
    }
    let pos = state.pos;
    let east = Vec3::new(-pos.z, 0.0, pos.x);
    if east.length() < 1e-9 {
        let radial = pos * (1.0 / r_mag);
        let east = cross(Vec3::new(0.0, 1.0, 0.0), radial);
        if east.length() < 1e-9 {
            return 0.0;
        }
        return roll_about_body_y(state.r, east.unit());
    }
    roll_about_body_y(state.r, east.unit())
}

fn roll_about_body_y(r: Matrix3, east: Vec3) -> f64 {
    let body_x = mul(r, Vec3::new(1.0, 0.0, 0.0));
    let body_y = mul(r, Vec3::new(0.0, 1.0, 0.0));
    let body_z = mul(r, Vec3::new(0.0, 0.0, 1.0));
    let mut refr = east - body_y * dot(east, body_y);
    if refr.length() < 1e-9 {
        return 0.0;
    }
    refr = refr.unit();
    dot(body_z, refr).atan2(dot(body_x, refr))
}

/// 罗盘方位/俯仰 → 当地惯性系机头方向 d̂（单位向量）。
///
/// 方位角 `azimuth` [rad]：罗盘语义 0=北、顺时针增（东=+90°）；`elevation`
/// [rad]：机头相对当地水平面的仰角（+90°=天顶、−90°=天底）。罗盘架由当地
/// 几何派生（与 [`roll_angle`] 同源）：east = unit(−z, 0, x)（星球自转东向
/// 切向）、up = radial、north = cross(east, up)。构造式
/// d̂ = north·cos(el)·cos(az) + east·cos(el)·sin(az) + up·sin(el)。
///
/// 精确 ±90°（|sin(el)|≈1）时方位不适定，塌缩为就近极竖直（d̂ = ±up）；
/// radial 退化为极轴（east 零向量）时同样按 el 符号取 ±up（防御分支，该
/// 构型下方位无定义，展示端亦不消费）。
pub fn nose_dir_from_az_el(radial: Vec3, azimuth: f64, elevation: f64) -> Vec3 {
    let up = radial.unit();
    let sin_el = elevation.sin();
    if (sin_el.abs() - 1.0).abs() < 1e-9 {
        return up * sin_el.signum();
    }
    let east = Vec3::new(-up.z, 0.0, up.x);
    if east.length2() < 1e-18 {
        return if sin_el >= 0.0 { up } else { -up };
    }
    let east = east.unit();
    let north = cross(east, up);
    let cos_el = elevation.cos();
    north * (cos_el * azimuth.cos()) + east * (cos_el * azimuth.sin()) + up * sin_el
}

/// 反解目标机头方向 d̂ 对应的 (pitch, yaw) TVC 目标 [rad]。
///
/// **固定罗盘架参考**（与 [`nose_dir_from_az_el`] 同源 east/north/up，与
/// Godot HUD 的 roll=0 目标显示约定一致）：目标读数只依赖 d̂ 与当地架、
/// 不依赖当前体姿态——PD 追固定读数可收敛（若用当前 bx 做参考，目标随
/// 姿态漂移，大角度下实测发散）。执行链定标（TVC 探针）：+pitch 读数把
/// 机头沿 −bz 水平投影倾、+yaw 读数绕 bz 负向转，逆解取
/// - `p = atan2(−d̂·north, d̂·up)`（+pitch 朝 −north = 罗盘北倾）
/// - `y = atan2(+d̂·east, d̂·up)`
///
/// 全域良定：极点 radial 时 east/north 零向量 → 点积 0 → (p,y)=(0,0)
/// 自然塌缩竖直（与 d̂ 极点塌缩衔接）；d̂ 水平时 atan2(·,0)=±π/2 良定。
/// **参考架注记**：本函数是固定罗盘架读数（与体姿态解耦），供 HUD 目标
/// 显示与罗盘几何测试使用；**控制路径不要直接与 [`pitch_yaw_angles`] 的
/// 体架读数相减做 PD**——滚转 ψ 会直接漏进稳态误差（2026-11-09 黑匣子
/// rocket_8562 实测：滚转 ~104° 时机头锁死在罗盘方位差 −70°/俯仰差 −53°
/// 的伪平衡点）。控制路径用 [`nose_target_angles_body`]。
pub fn nose_target_angles(radial: Vec3, desired: Vec3) -> (f64, f64) {
    let up = radial.unit();
    let d = desired.unit();
    let east = Vec3::new(-up.z, 0.0, up.x);
    let north = cross(east, up);
    let p = (-dot(d, north)).atan2(dot(d, up));
    let y = dot(d, east).atan2(dot(d, up));
    (p, y)
}

/// 反解目标机头方向 d̂ 对应的 (pitch, yaw) TVC 目标 [rad]——**体架相位对齐**
/// 变体（NoseTo 控制路径专用）。
///
/// 目标读数按当前体架相位构造目标体架：机头 `by* = d̂`、`bz*` 取当前体
/// +Z 的垂直 d̂ 分量（保持滚转相位；`bz*` 退化时改用体 +X）、`bx* = by*×bz*`
/// 保持体架手性（`bz = bx×by` 同一分量约定），再按 [`pitch_yaw_angles`]
/// 的同一读数定义取值。由此 PD 误差（当前读数 − 目标读数）满足
/// **「机头 = d̂ ⇔ 误差 = 0」对任意滚转相位**——TVC 无滚转轴时消除滚转
/// 演化向稳态方位/仰角的串扰；与 `ProgradeHold` 的 `prograde_target_angles`
/// （体 +X 相位构造）同一族。
///
/// 拓扑限制（与 pitch_to / retrograde 同类）：d̂ ≈ −by（180° 反指）时相位
/// 保持构造只能表达出部分误差，闭环大翻转不收敛；正交体架输入下 `bz*`、
/// `bx*` 双重退化不可达，第二层分支仅为数值防御。
pub fn nose_target_angles_body(radial: Vec3, desired: Vec3, bx: Vec3, bz: Vec3) -> (f64, f64) {
    let up = radial.unit();
    let d = desired.unit();
    let mut bz_t = bz - d * dot(bz, d);
    if bz_t.length2() < 1e-12 {
        bz_t = bx - d * dot(bx, d);
    }
    if bz_t.length2() < 1e-12 {
        return (0.0, 0.0);
    }
    let bz_t = bz_t.unit();
    let bx_t = cross(d, bz_t);
    let p = dot(bz_t, up).clamp(-1.0, 1.0).asin();
    let y = (-dot(bx_t, up)).clamp(-1.0, 1.0).asin();
    (p, y)
}

#[cfg(test)]
mod tests;
