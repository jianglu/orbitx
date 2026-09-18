use super::*;
use crate::dock::DockPort;

#[test]
fn rel_docking_pos_coaxial_stack() {
    // 底级顶口 ↔ 上级底口（纵轴 +Y）。
    let lower_top = DockPort::with_rot(
        Vec3::new(0.0, 5.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    let upper_bottom = DockPort::with_rot(
        Vec3::new(0.0, -3.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
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
    let core_right = DockPort::with_rot(
        Vec3::new(3.35, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    let booster_left = DockPort::with_rot(
        Vec3::new(-1.125, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
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
        rq: Quat::IDENTITY,
    };
    let booster = SubVesselData {
        vessel_index: 1,
        rpos: Vec3::new(5.0, 0.0, 0.0),
        rrot: Matrix3::IDENTITY,
        rq: Quat::IDENTITY,
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
