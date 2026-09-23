//! Rigid-body angular dynamics: Euler's equation and gravity-gradient torque.
//!
//! Mirrors Orbiter's `Rigidbody.cpp`. The integrators in [`crate::integrator`]
//! operate on a `StateVectors` bundle that already carries the angular state
//! (`omega`, `q`, `r`). They expect the force closure to return angular
//! *acceleration*, so converting a torque into `dω/dt` — i.e. solving Euler's
//! equation — is the caller's responsibility (see the note in
//! `integrator.rs:14-17`). The free functions here perform that conversion,
//! exactly as Orbiter's `RigidBody::EulerInv_*` member methods do.
//!
//! ## Principal moments of inertia (PMI)
//!
//! Orbiter stores the inertia tensor as a diagonal `Vector pmi` in the vessel
//! body frame (`Rigidbody.h:224`). For a rocket the longitudinal axis is **Y**
//! (`+Y` toward the nose, `-Y` toward the engines), so `pmi.y` is the *axial*
//! moment (small) and `pmi.x = pmi.z` are the *transverse* moments (large).
//!
//! ## Mass-normalised torque convention
//!
//! Orbiter's `Vessel::GetIntermediateMoments` (`Vessel.cpp:910-922`) divides
//! the accumulated torque by mass before returning it:
//!
//! ```text
//! tau += M / mass;
//! ```
//!
//! i.e. the `tau` handed to `EulerInv_*` is a *specific torque* [N·m / kg]. The
//! expressions below are used verbatim; they do **not** divide by mass again.
//!
//! ## Left-handed system
//!
//! The cross-axis coupling signs follow Orbiter's left-handed ecliptic J2000
//! frame (see `orbitx-math/src/lib.rs:8-14`). The formulae are copied
//! symbol-for-symbol from `Rigidbody.cpp:458-511`; do not "correct" the signs.

use orbitx_math::{cross, Vec3};
use orbitx_math::consts::GGRAV;

/// Solve Euler's equation for angular acceleration — full coupled form
/// (`EulerInv_full`, Rigidbody.cpp:468-481).
///
/// Solves the left-handed Euler equation
///
/// ```text
/// I·dω/dt + (Iω) × ω = τ
/// ```
///
/// for `dω/dt`, given specific torque `tau`, angular velocity `omega`, and the
/// diagonal inertia tensor `pmi`.
pub fn euler_inv_full(tau: Vec3, omega: Vec3, pmi: Vec3) -> Vec3 {
    Vec3::new(
        (tau.x - (pmi.y - pmi.z) * omega.y * omega.z) / pmi.x,
        (tau.y - (pmi.z - pmi.x) * omega.z * omega.x) / pmi.y,
        (tau.z - (pmi.x - pmi.y) * omega.x * omega.y) / pmi.z,
    )
}

/// Solve Euler's equation — simplified decoupled form
/// (`EulerInv_simple`, Rigidbody.cpp:485-497).
///
/// Drops the `(Iω) × ω` cross-axis coupling and simply returns `τ / I`. Orbiter
/// uses this at high time-acceleration to avoid the coupling-driven
/// instabilities of the full equation.
pub fn euler_inv_simple(tau: Vec3, pmi: Vec3) -> Vec3 {
    Vec3::new(tau.x / pmi.x, tau.y / pmi.y, tau.z / pmi.z)
}

/// Trivial angular acceleration — returns zero
/// (`EulerInv_zero`, Rigidbody.cpp:501-511).
///
/// Suppresses both the coupling terms and the torque, solving `I·dω/dt = 0`.
/// Used by Orbiter to disable attitude dynamics entirely.
pub fn euler_inv_zero() -> Vec3 {
    Vec3::ZERO
}

/// Forward Euler equation — returns specific torque from angular acceleration
/// (`Euler_full`, Rigidbody.cpp:458-464).
///
/// The inverse of [`euler_inv_full`]: given `omegadot` and `omega`, returns the
/// specific torque `τ` that would produce that angular acceleration. Mainly
/// useful for tests.
pub fn euler_full(omegadot: Vec3, omega: Vec3, pmi: Vec3) -> Vec3 {
    Vec3::new(
        omegadot.x * pmi.x + (pmi.y - pmi.z) * omega.y * omega.z,
        omegadot.y * pmi.y + (pmi.z - pmi.x) * omega.z * omega.x,
        omegadot.z * pmi.z + (pmi.x - pmi.y) * omega.x * omega.y,
    )
}

/// Gravity-gradient torque (mass-normalised) with optional tidal damping
/// (`RigidBody::GetIntermediateMoments` angular part, Rigidbody.cpp:345-363;
/// also `RigidBody::GetTorque`, Rigidbody.cpp:424-447).
///
/// Computes the specific torque exerted on a rigid body by the gravity gradient
/// of a central body of mass `cbody_mass` whose position relative to the vessel
/// is `rel_pos` (in the **global** frame). `pmi` is the body-frame diagonal
/// inertia tensor; `rot` maps body→world (`mul(rot, v)`); `omega` is the
/// current angular velocity (body frame); `tidaldamp` is the dimensionless
/// damping factor (Orbiter's `GravityGradientDamping` config value); `dt` is
/// the current step size used to cap the damping.
///
/// Returns the specific torque [N·m / kg] in the **body** frame, ready to be
/// passed to [`euler_inv_full`]. Returns zero when the gravity-gradient effect
/// is to be suppressed (`b_ignore` set) or `rel_pos` is degenerate.
/// Gravity-gradient torque (mass-normalised) with optional tidal damping
/// (`RigidBody::GetIntermediateMoments` angular part, Rigidbody.cpp:345-363;
/// also `RigidBody::GetTorque`, Rigidbody.cpp:424-447).
///
/// Computes the specific torque exerted on a rigid body by the gravity gradient
/// of a central body of mass `cbody_mass` whose position relative to the vessel
/// is `rel_pos` (in the **global** frame). `pmi` is the body-frame diagonal
/// inertia tensor; `rot` maps body→world (`mul(rot, v)`); `omega` is the
/// current angular velocity (body frame); `tidaldamp` is the dimensionless
/// damping factor (Orbiter's `GravityGradientDamping` config value); `dt` is
/// the current step size used to cap the damping.
///
/// Returns the specific torque [N·m / kg] in the **body** frame, ready to be
/// passed to [`euler_inv_full`]. Returns zero when the gravity-gradient effect
/// is to be suppressed (`b_ignore` set) or `rel_pos` is degenerate.
#[allow(clippy::too_many_arguments)] // 忠实移植 Orbiter 力矩计算的完整输入集
pub fn gravity_gradient_torque(
    rel_pos: Vec3,
    cbody_mass: f64,
    pmi: Vec3,
    rot: orbitx_math::Matrix3,
    omega: Vec3,
    tidaldamp: f64,
    dt: f64,
    b_ignore: bool,
) -> Vec3 {
    if b_ignore {
        return Vec3::ZERO;
    }
    let r0 = rel_pos.length();
    if r0 < 1e-3 {
        return Vec3::ZERO;
    }
    // Map the central body direction into the vessel frame.
    // Rigidbody.cpp:349: R0 = tmul(state.Q, cbody_pos - state.pos)
    let r0_body = orbitx_math::tmul(rot, rel_pos);
    let re = r0_body * (1.0 / r0);
    let mag = 3.0 * GGRAV * cbody_mass / (r0 * r0 * r0);
    let mut tau = cross(pmi * re, re) * mag;

    // Damping of angular velocity (Rigidbody.cpp:356-362).
    if tidaldamp != 0.0 {
        let damp = tidaldamp * mag;
        let scale = damp.min(dt * 0.1);
        if omega.x != 0.0 {
            tau.x -= scale * pmi.x * omega.x;
        }
        if omega.y != 0.0 {
            tau.y -= scale * pmi.y * omega.y;
        }
        if omega.z != 0.0 {
            tau.z -= scale * pmi.z * omega.z;
        }
    }
    tau
}

// ── 组合体刚体合成（移自 vessel::supervessel，移植 Orbiter SuperVessel） ──────
//
// 组合体坐标系约定（对齐 `SuperVessel.h` 注释）：
// - 原点与姿态取 root 成员体坐标（root 的 `rpos = 0`、`rrot = I`）
// - 子船点：`ps = rrot_i * pv + rpos_i`
// - 世界点：`pg = R_sv * (ps - cg) + gpos`（`gpos` 为组合体 CG）

use orbitx_math::{Matrix3 as M3, Quat as Q4, StateVectors as SV};

/// 子船在组合体（root）坐标系中的相对位姿。
///
/// `vessel_index` 仅为调用方关联 `Assembly::vessels` 下标而保留的标签，
/// dynamics 不依赖其语义。
#[derive(Clone, Debug)]
pub struct SubVesselData {
    /// 在 `Assembly::vessels` 中的下标（标签）。
    pub vessel_index: usize,
    /// 子船原点相对 root 的位置 [m]。
    pub rpos: Vec3,
    /// 子船体坐标 → 组合体坐标的旋转。
    pub rrot: M3,
    /// 与 `rrot` 对应的四元数。
    pub rq: Q4,
}

/// 对接口几何（仅 pos/dir/rot，不含对接图状态）；供 `rel_docking_pos` 入参。
#[derive(Clone, Copy, Debug)]
pub struct DockGeometry {
    pub pos: Vec3,
    pub dir: Vec3,
    pub rot: Vec3,
}

/// 计算 `target` 相对 `mine` 的位姿，使双方指定端口对齐对接。
///
/// 移植 `Vessel::RelDockingPos`（`Vessel.cpp:2884-2928`）：
/// - 本口 `dir` ↔ 对方 `-dir`
/// - 本口 `rot` ↔ 对方 `rot`
/// - 返回的 `rpos` / `rrot` 把 **target 体坐标** 映到 **mine 体坐标**
pub fn rel_docking_pos(mine: &DockGeometry, target: &DockGeometry) -> (Vec3, M3) {
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
        M3::IDENTITY
    } else {
        M3::new(
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

    let p = mine.pos - orbitx_math::mul(r, target.pos);
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
            let rt = orbitx_math::mul(c.rrot, *rj) + c.rpos - cg;
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
    let f_trans = orbitx_math::mul(comp.rrot, f_comp);
    *f_sv += f_trans;
    *m_sv += orbitx_math::mul(comp.rrot, m_comp) + cross(f_trans, comp.rpos - cg);
}

/// 由组合体状态写回子船状态（`ComponentStateVectors`）。
pub fn component_state_vectors(sv: &SV, comp: &SubVesselData, cg: Vec3) -> SV {
    let mut out = *sv;
    out.vel = sv.vel + orbitx_math::mul(sv.r, cross(comp.rpos - cg, sv.omega));
    out.pos = sv.pos + orbitx_math::mul(sv.r, comp.rpos - cg);
    out.omega = orbitx_math::tmul(comp.rrot, sv.omega);
    out.q = sv.q.hamilton(comp.rq);
    out.r = M3::from_quat(out.q);
    out
}

/// 由 root 子船状态与 CG 得到组合体状态（root 须 `rpos≈0`、`rrot≈I`）。
pub fn supervessel_state_from_root(root_state: &SV, cg: Vec3) -> SV {
    let mut s = *root_state;
    // CG 世界位置 = root 原点世界位置 + R * cg（root rpos = 0）
    s.pos = root_state.pos + orbitx_math::mul(root_state.r, cg);
    // 角速度已在 root/组合体同一姿态下
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::{Matrix3, Vec3};

    /// `EulerInv_full` must invert `Euler_full` exactly.
    #[test]
    fn euler_inv_inverts_full() {
        let pmi = Vec3::new(1e5, 3e4, 1e5);
        let omega = Vec3::new(0.02, -0.01, 0.05);
        let omegadot = Vec3::new(1e-3, 2e-3, -1e-3);
        let tau = euler_full(omegadot, omega, pmi);
        let recovered = euler_inv_full(tau, omega, pmi);
        for (a, b) in [
            (recovered.x, omegadot.x),
            (recovered.y, omegadot.y),
            (recovered.z, omegadot.z),
        ] {
            assert!((a - b).abs() < 1e-9, "{} vs {}", a, b);
        }
    }

    /// No coupling when two PMI components are equal (axisymmetric body): a pure
    /// spin about the symmetry axis must yield zero torque for zero external τ.
    #[test]
    fn axisymmetric_spin_no_coupling() {
        let pmi = Vec3::new(1e5, 3e4, 1e5); // x == z → axisymmetric about Y
        let omega = Vec3::new(0.0, 0.1, 0.0); // pure axial spin
        let tau = Vec3::ZERO;
        let arot = euler_inv_full(tau, omega, pmi);
        assert!(arot.length() < 1e-9, "non-zero α for free axial spin");
    }

    /// `EulerInv_simple` must equal `EulerInv_full` when ω = 0 (no coupling).
    #[test]
    fn simple_equals_full_at_rest() {
        let pmi = Vec3::new(1e5, 3e4, 1e5);
        let tau = Vec3::new(10.0, -5.0, 8.0);
        let full = euler_inv_full(tau, Vec3::ZERO, pmi);
        let simple = euler_inv_simple(tau, pmi);
        assert!((full - simple).length() < 1e-9);
    }

    /// `EulerInv_zero` always returns the zero vector.
    #[test]
    fn zero_is_zero() {
        assert_eq!(euler_inv_zero(), Vec3::ZERO);
    }

    /// Gravity-gradient torque vanishes for a spherically symmetric body
    /// (pmi.x == pmi.y == pmi.z): `cross(pmi*Re, Re) = cross(c*Re, Re) = 0`.
    #[test]
    fn grav_gradient_zero_for_isotropic() {
        let pmi = Vec3::new(1e5, 1e5, 1e5);
        let rel_pos = Vec3::new(6.4e6, 0.0, 0.0);
        let tau = gravity_gradient_torque(
            rel_pos,
            5.972e24,
            pmi,
            Matrix3::IDENTITY,
            Vec3::ZERO,
            0.0,
            1.0,
            false,
        );
        assert!(tau.length() < 1e-6, "isotropic body should have no ggd torque");
    }

    /// A non-isotropic body whose long axis is *not* aligned with the radial
    /// direction experiences a gradient torque that tends to restore alignment
    /// (gravity-gradient stabilisation). With the body tipped 45° between X and
    /// Y, `cross(pmi*Re, Re)` has a non-zero Z component because `pmi.x != pmi.y`.
    #[test]
    fn grav_gradient_nonzero_for_slender_body() {
        let pmi = Vec3::new(1e5, 1e3, 1e5); // slender about Y
        // Central body lies in the XY plane at 45°; with identity rotation the
        // body frame coincides with the global frame, so Re is not along a
        // principal axis and the PMI-weighted vector is no longer parallel to Re.
        let rel_pos = Vec3::new(4.5e6, 4.5e6, 0.0);
        let tau = gravity_gradient_torque(
            rel_pos,
            5.972e24,
            pmi,
            Matrix3::IDENTITY,
            Vec3::ZERO,
            0.0,
            1.0,
            false,
        );
        // Torque should be small but distinctly non-zero, along the body Z axis.
        assert!(tau.length() > 1e-9, "expected non-zero gradient torque, got {:?}", tau);
        // The restoring torque points along ±Z (out of the XY plane).
        assert!(tau.x.abs() < 1e-12 && tau.y.abs() < 1e-12,
            "torque should be along Z, got {:?}", tau);
    }

    // ── 组合体刚体合成测试（移自 vessel::supervessel/tests.rs） ──

    fn dock(pos: Vec3, dir: Vec3, rot: Vec3) -> DockGeometry {
        DockGeometry { pos, dir, rot }
    }

    #[test]
    fn rel_docking_pos_coaxial_stack() {
        // 底级顶口 ↔ 上级底口（纵轴 +Y）。
        let lower_top = dock(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        let upper_bottom = dock(Vec3::new(0.0, -3.0, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        let (p, r) = rel_docking_pos(&lower_top, &upper_bottom);
        // 上级原点应在下级上方 (5+3)=8 m。
        assert!(
            (p - Vec3::new(0.0, 8.0, 0.0)).length() < 1e-6,
            "rpos={p:?}"
        );
        // 相对旋转应接近单位阵。
        let id = Matrix3::IDENTITY;
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (r.get(i, j) - id.get(i, j)).abs() < 1e-6,
                    "rrot[{i},{j}]={}",
                    r.get(i, j)
                );
            }
        }
    }

    #[test]
    fn rel_docking_pos_lateral() {
        // 芯级右侧口 dir=+X，助推左侧口 dir=-X（仿 Atlantis ET/SRB）。
        let core_right = dock(Vec3::new(3.35, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        let booster_left = dock(Vec3::new(-1.125, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        let (p, r) = rel_docking_pos(&core_right, &booster_left);
        assert!(
            p.x > 3.0,
            "助推原点应在芯级 +X 侧: rpos={p:?}"
        );
        assert!(p.y.abs() < 1e-6 && p.z.abs() < 1e-6, "侧挂应无 Y/Z 偏移: {p:?}");
        // 助推与芯级纵轴平行 → rrot ≈ I
        assert!((r.get(0, 0) - 1.0).abs() < 1e-5);
        assert!((r.get(1, 1) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn composite_pmi_lateral_increases_transverse() {
        let core = SubVesselData {
            vessel_index: 0,
            rpos: Vec3::ZERO,
            rrot: Matrix3::IDENTITY,
            rq: Q4::IDENTITY,
        };
        let booster = SubVesselData {
            vessel_index: 1,
            rpos: Vec3::new(5.0, 0.0, 0.0),
            rrot: Matrix3::IDENTITY,
            rq: Q4::IDENTITY,
        };
        let masses = [1000.0, 500.0];
        let pmis = [
            Vec3::new(10.0, 2.0, 10.0),
            Vec3::new(4.0, 1.0, 4.0),
        ];
        let cg = center_of_mass(&[core.clone(), booster.clone()], &masses);
        let pmi = composite_pmi(&[core, booster], &masses, &pmis, cg);
        // 侧挂后绕 Y（纵轴）的惯量应因平行轴而增大。
        assert!(pmi.y > 2.0, "侧挂应增大轴向附近惯量分量: {pmi:?}");
    }

    #[test]
    fn component_state_vectors_round_trip_root() {
        // root rpos=0、rrot=I → component_state_vectors 应近似返回 sv（仅 pos/vel 偏移 cg）。
        let root = SubVesselData {
            vessel_index: 0,
            rpos: Vec3::ZERO,
            rrot: Matrix3::IDENTITY,
            rq: Q4::IDENTITY,
        };
        let sv = SV {
            pos: Vec3::new(6.4e6, 0.0, 0.0),
            vel: Vec3::new(0.0, 7800.0, 0.0),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Q4::IDENTITY,
        };
        let out = component_state_vectors(&sv, &root, Vec3::ZERO);
        assert!((out.pos - sv.pos).length() < 1e-9);
        assert!((out.vel - sv.vel).length() < 1e-9);
    }
}
