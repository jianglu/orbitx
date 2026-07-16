//! 预设火箭配置：基于真实参数。
//!
//! 每个预设返回 `Vec<StageSpec>`，调用方可进一步配置气动力
//! （`Vessel::dragels`、`Vessel::cross_section`、`Vessel::rdrag`）。

use crate::stage::StageSpec;
use orbitx_math::Vec3;

/// 为 Falcon 9 / Saturn V 的各级配置默认气动力参数。
///
/// 包括阻力元件、截面积和气动阻尼系数。
pub fn configure_default_aero(vessels: &mut [crate::vessel::Vessel]) {
    for v in vessels.iter_mut() {
        // 简单阻力：Cd=0.3, S=π*r²（截面积）。
        let area = std::f64::consts::PI * v.radius * v.radius;
        v.dragels.push(crate::aero::DragElement {
            ref_pos: Vec3::ZERO,
            cd: 0.3,
            area,
        });
        v.cross_section = Vec3::new(area, area * 2.0, area);
        v.rdrag = Vec3::new(1.0, 0.1, 1.0);
    }
}

/// Falcon 9 两级 + 有效载荷。
///
/// 参数来源：公开技术规格（SpaceX 官网 / Wikipedia）。
/// stages[0] = 第一级，stages[1] = 第二级，stages[2] = 有效载荷。
pub fn falcon9() -> Vec<StageSpec> {
    vec![
        // 第一级：9 × Merlin 1D 海平面。
        StageSpec {
            name: "F9-S1",
            dry_mass: 25_600.0,
            fuel_mass: 411_000.0,
            thrust: 7_607_000.0, // 9 × 845 kN
            isp: 282.0,          // 海平面
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -23.5, 0.0), // 底部
            length: 47.0,
            radius: 1.85,
            separation_impulse: 3.0,
            // TVC：Merlin 1D 可双向摆动 ±5°~7°，作动速率约 20°/s。
            max_gimbal: 0.122, // 7°
            max_gimbal_rate: 0.35, // 20°/s
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        // 第二级：1 × Merlin Vacuum。
        StageSpec {
            name: "F9-S2",
            dry_mass: 4_000.0,
            fuel_mass: 107_500.0,
            thrust: 934_000.0, // 934 kN
            isp: 348.0,        // 真空
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -7.0, 0.0),
            length: 14.0,
            radius: 1.85,
            separation_impulse: 2.0,
            // MVac 也有 TVC，但真空段姿态控制需求小。
            max_gimbal: 0.087, // 5°
            max_gimbal_rate: 0.17, // 10°/s
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        // 有效载荷。
        StageSpec {
            name: "Payload",
            dry_mass: 22_800.0, // ~Starlink batch
            fuel_mass: 0.0,
            thrust: 0.0,
            isp: 0.0,
            engine_dir: Vec3::ZERO,
            engine_pos: Vec3::ZERO,
            length: 5.0,
            radius: 1.85,
            separation_impulse: 1.0,
            ..Default::default()
        },
    ]
}

/// Saturn V 三级 + 阿波罗载荷。
///
/// 参数来源：NASA Apollo News Reference。
pub fn saturn_v() -> Vec<StageSpec> {
    vec![
        // S-IC 第一级：5 × F-1。
        StageSpec {
            name: "S-IC",
            dry_mass: 130_000.0,
            fuel_mass: 2_150_000.0, // RP-1 + LOX
            thrust: 34_500_000.0,   // 5 × 6.9 MN 海平面
            isp: 263.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -21.0, 0.0),
            length: 42.0,
            radius: 5.0,
            separation_impulse: 4.0,
            // F-1 四机中心固定+周边摆动，TVC 约 ±6°。
            max_gimbal: 0.105, // 6°
            max_gimbal_rate: 0.26, // 15°/s
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        // S-II 第二级：5 × J-2。
        StageSpec {
            name: "S-II",
            dry_mass: 36_000.0,
            fuel_mass: 440_000.0, // LH2 + LOX
            thrust: 5_000_000.0,  // 5 × 1.0 MN 真空
            isp: 421.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -12.0, 0.0),
            length: 24.8,
            radius: 5.0,
            separation_impulse: 3.0,
            max_gimbal: 0.087, // 5°
            max_gimbal_rate: 0.17, // 10°/s
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        // S-IVB 第三级：1 × J-2。
        StageSpec {
            name: "S-IVB",
            dry_mass: 10_000.0,
            fuel_mass: 110_000.0, // LH2 + LOX
            thrust: 1_000_000.0,  // 1 × 1.0 MN 真空
            isp: 421.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -8.5, 0.0),
            length: 17.8,
            radius: 3.3,
            separation_impulse: 2.0,
            max_gimbal: 0.087, // 5°
            max_gimbal_rate: 0.17, // 10°/s
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        // 阿波罗指令服务舱 + 登月舱。
        StageSpec {
            name: "CSM-LM",
            dry_mass: 45_000.0,
            fuel_mass: 0.0,
            thrust: 0.0,
            isp: 0.0,
            engine_dir: Vec3::ZERO,
            engine_pos: Vec3::ZERO,
            length: 10.0,
            radius: 3.3,
            separation_impulse: 1.5,
            ..Default::default()
        },
    ]
}
