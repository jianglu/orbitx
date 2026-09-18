//! SuperVessel 几何与刚体合成辅助（移植自 Orbiter `SuperVessel` / `RelDockingPos`）。
//!
//! 组合体坐标系约定（对齐 `SuperVessel.h` 注释）：
//! - 原点与姿态取 root 成员体坐标（root 的 `rpos = 0`、`rrot = I`）
//! - 子船点：`ps = rrot_i * pv + rpos_i`
//! - 世界点：`pg = R_sv * (ps - cg) + gpos`（`gpos` 为组合体 CG）

use crate::dock::DockPort;
use orbitx_math::{cross, mul, tmul, Matrix3, Quat, StateVectors, Vec3};

/// 子船在组合体（root）坐标系中的相对位姿。
#[derive(Clone, Debug)]
pub struct SubVesselData {
    /// 在 `Assembly::vessels` 中的下标。
    pub vessel_index: usize,
    /// 子船原点相对 root 的位置 [m]。
    pub rpos: Vec3,
    /// 子船体坐标 → 组合体坐标的旋转。
    pub rrot: Matrix3,
    /// 与 `rrot` 对应的四元数。
    pub rq: Quat,
}

/// 计算 `target` 相对 `mine` 的位姿，使双方指定端口对齐对接。
///
/// 移植 `Vessel::RelDockingPos`（`Vessel.cpp:2884-2928`）：
/// - 本口 `dir` ↔ 对方 `-dir`
/// - 本口 `rot` ↔ 对方 `rot`
/// - 返回的 `rpos` / `rrot` 把 **target 体坐标** 映到 **mine 体坐标**
pub fn rel_docking_pos(mine: &DockPort, target: &DockPort) -> (Vec3, Matrix3) {
    let as_ = target.dir;
    let bs = target.rot;
    let cs = cross(as_, bs);
    let at = -mine.dir;
    let bt = mine.rot;
    let ct = cross(at, bt);

    let den = cs.x * (as_.y * bs.z - as_.z * bs.y)
        + cs.y * (as_.z * bs.x - as_.x * bs.z)
        + cs.z * (as_.x * bs.y - as_.y * bs.x);

    let r = if den.abs() < 1e-18 {
        Matrix3::IDENTITY
    } else {
        Matrix3::new(
            (ct.x * (as_.y * bs.z - as_.z * bs.y)
                + bt.x * (as_.z * cs.y - as_.y * cs.z)
                + at.x * (bs.y * cs.z - bs.z * cs.y))
                / den,
            (ct.x * (as_.z * bs.x - as_.x * bs.z)
                + bt.x * (as_.x * cs.z - as_.z * cs.x)
                + at.x * (bs.z * cs.x - bs.x * cs.z))
                / den,
            (ct.x * (as_.x * bs.y - as_.y * bs.x)
                + bt.x * (as_.y * cs.x - as_.x * cs.y)
                + at.x * (bs.x * cs.y - bs.y * cs.x))
                / den,
            (ct.y * (as_.y * bs.z - as_.z * bs.y)
                + bt.y * (as_.z * cs.y - as_.y * cs.z)
                + at.y * (bs.y * cs.z - bs.z * cs.y))
                / den,
            (ct.y * (as_.z * bs.x - as_.x * bs.z)
                + bt.y * (as_.x * cs.z - as_.z * cs.x)
                + at.y * (bs.z * cs.x - bs.x * cs.z))
                / den,
            (ct.y * (as_.x * bs.y - as_.y * bs.x)
                + bt.y * (as_.y * cs.x - as_.x * cs.y)
                + at.y * (bs.x * cs.y - bs.y * cs.x))
                / den,
            (ct.z * (as_.y * bs.z - as_.z * bs.y)
                + bt.z * (as_.z * cs.y - as_.y * cs.z)
                + at.z * (bs.y * cs.z - bs.z * cs.y))
                / den,
            (ct.z * (as_.z * bs.x - as_.x * bs.z)
                + bt.z * (as_.x * cs.z - as_.z * cs.x)
                + at.z * (bs.z * cs.x - bs.x * cs.z))
                / den,
            (ct.z * (as_.x * bs.y - as_.y * bs.x)
                + bt.z * (as_.y * cs.x - as_.x * cs.y)
                + at.z * (bs.x * cs.y - bs.y * cs.x))
                / den,
        )
    };

    let p = mine.pos - mul(r, target.pos);
    (p, r)
}

/// 组合体质心（组合体 / root 坐标系）。
pub fn center_of_mass(components: &[SubVesselData], masses: &[f64]) -> Vec3 {
    let mut m_sum = 0.0;
    let mut cg = Vec3::ZERO;
    for (c, &m) in components.iter().zip(masses.iter()) {
        m_sum += m;
        cg += c.rpos * m;
    }
    if m_sum < 1e-3 {
        Vec3::ZERO
    } else {
        cg * (1.0 / m_sum)
    }
}

/// 合成组合体归一化 PMI（m²），移植 `SuperVessel::CalcPMI`（含相对旋转）。
pub fn composite_pmi(
    components: &[SubVesselData],
    masses: &[f64],
    vessel_pmis: &[Vec3],
    cg: Vec3,
) -> Vec3 {
    let total_mass: f64 = masses.iter().sum();
    if total_mass < 1e-3 || components.is_empty() {
        return Vec3::new(1.0, 1.0, 1.0);
    }

    let mut pmi = Vec3::ZERO;
    for (i, c) in components.iter().enumerate() {
        let vpmi = vessel_pmis[i];
        let vmass = masses[i] / 6.0;
        let mut r0 = [Vec3::ZERO; 6];
        r0[0].x = (1.5 * (-vpmi.x + vpmi.y + vpmi.z).abs()).sqrt();
        r0[1].x = -r0[0].x;
        r0[2].y = (1.5 * (vpmi.x - vpmi.y + vpmi.z).abs()).sqrt();
        r0[3].y = -r0[2].y;
        r0[4].z = (1.5 * (vpmi.x + vpmi.y - vpmi.z).abs()).sqrt();
        r0[5].z = -r0[4].z;

        let mut vpmix = 0.0;
        let mut vpmiy = 0.0;
        let mut vpmiz = 0.0;
        for rj in &r0 {
            let rt = mul(c.rrot, *rj) + c.rpos - cg;
            let rtx2 = rt.x * rt.x;
            let rty2 = rt.y * rt.y;
            let rtz2 = rt.z * rt.z;
            vpmix += rty2 + rtz2;
            vpmiy += rtx2 + rtz2;
            vpmiz += rtx2 + rty2;
        }
        pmi.x += vmass * vpmix;
        pmi.y += vmass * vpmiy;
        pmi.z += vmass * vpmiz;
    }

    Vec3::new(pmi.x / total_mass, pmi.y / total_mass, pmi.z / total_mass)
}

/// 将子船体坐标力/力矩累加到组合体坐标（`AddComponentForceAndMoment`）。
pub fn add_component_force_and_moment(
    f_sv: &mut Vec3,
    m_sv: &mut Vec3,
    f_comp: Vec3,
    m_comp: Vec3,
    comp: &SubVesselData,
    cg: Vec3,
) {
    let f_trans = mul(comp.rrot, f_comp);
    *f_sv += f_trans;
    *m_sv += mul(comp.rrot, m_comp) + cross(f_trans, comp.rpos - cg);
}

/// 由组合体状态写回子船状态（`ComponentStateVectors`）。
pub fn component_state_vectors(
    sv: &StateVectors,
    comp: &SubVesselData,
    cg: Vec3,
) -> StateVectors {
    let mut out = *sv;
    out.vel = sv.vel + mul(sv.r, cross(comp.rpos - cg, sv.omega));
    out.pos = sv.pos + mul(sv.r, comp.rpos - cg);
    out.omega = tmul(comp.rrot, sv.omega);
    out.q = sv.q.hamilton(comp.rq);
    out.r = Matrix3::from_quat(out.q);
    out
}

/// 由 root 子船状态与 CG 得到组合体状态（root 须 `rpos≈0`、`rrot≈I`）。
pub fn supervessel_state_from_root(root_state: &StateVectors, cg: Vec3) -> StateVectors {
    let mut s = *root_state;
    // CG 世界位置 = root 原点世界位置 + R * cg（root rpos = 0）
    s.pos = root_state.pos + mul(root_state.r, cg);
    // 角速度已在 root/组合体同一姿态下
    s
}

#[cfg(test)]
mod tests;
