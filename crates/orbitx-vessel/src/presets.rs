//! 预设火箭配置：基于真实参数。

use crate::stage::{LiftingSurfaceSpec, StageSpec, ThrusterSpec};
use orbitx_dynamics::{FinKind, DEFAULT_ALPHA_STALL_FIN, DEFAULT_ALPHA_STALL_GRID};
use orbitx_math::Vec3;

/// 径向均布翼面（`n` 片，压心半径 `r`、轴向位置 `y`）。
fn radial_fins(
    n: usize,
    r: f64,
    y: f64,
    area: f64,
    aspect_ratio: f64,
    kind: FinKind,
    alpha_stall0: f64,
    cd0: f64,
    deploy_rate: f64,
) -> Vec<LiftingSurfaceSpec> {
    (0..n)
        .map(|i| {
            let a = (i as f64) * std::f64::consts::TAU / (n as f64);
            let normal = Vec3::new(a.cos(), 0.0, a.sin());
            LiftingSurfaceSpec {
                ref_pos: Vec3::new(r * a.cos(), y, r * a.sin()),
                normal,
                chord_dir: Vec3::new(0.0, -1.0, 0.0),
                area,
                aspect_ratio,
                cl_alpha: 3.5,
                cd0,
                edge_area: 0.0,
                alpha_stall0,
                kind,
                deploy: 1.0,
                deploy_target: 1.0,
                deploy_rate,
            }
        })
        .collect()
}

fn cd_mach_rocket() -> Vec<(f64, f64)> {
    crate::stage::default_rocket_cd_mach()
}

fn thruster(
    pos: Vec3,
    thrust_vac: f64,
    isp_vac: f64,
    thrust_sl: Option<f64>,
    isp_sl: Option<f64>,
    max_gimbal: f64,
    max_gimbal_rate: f64,
    throttle_rate: f64,
) -> ThrusterSpec {
    ThrusterSpec {
        pos,
        dir: Vec3::new(0.0, 1.0, 0.0),
        thrust: thrust_vac,
        isp: isp_vac,
        thrust_sl,
        isp_sl,
        max_gimbal,
        max_gimbal_rate,
        gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        throttle_rate,
    }
}

/// 为各级补默认气动（兼容旧调用）。
///
/// 已走火箭路径（`rocket_body`）时不钉原点 `DragElement`。
pub fn configure_default_aero(vessels: &mut [crate::vessel::Vessel]) {
    for v in vessels.iter_mut() {
        if v.rocket_body.is_some() {
            continue;
        }
        if v.dragels.is_empty() {
            let area = std::f64::consts::PI * v.radius * v.radius;
            v.dragels.push(
                crate::aero::DragElement::constant(Vec3::ZERO, 0.3, area)
                    .with_cd_mach(cd_mach_rocket()),
            );
            v.cross_section = Vec3::new(area, area * 2.0, area);
            v.rdrag = Vec3::new(1.0, 0.1, 1.0);
        }
    }
}

/// Falcon 9 两级 + 有效载荷。
pub fn falcon9() -> Vec<StageSpec> {
    // Merlin 1D：海平面 ~845 kN / Isp~282；真空 ~914 kN / ~311 s
    // throttle_rate：Merlin 无公开数据，用 NASA CECE ~0.8/s 作可节流代理。
    let merlin = |pos: Vec3| {
        thruster(
            pos,
            914_000.0,
            311.0,
            Some(845_000.0),
            Some(282.0),
            0.122,
            0.35,
            0.8,
        )
    };
    // 九机：中心 + 八角环近似
    let r = 1.2;
    let mut s1_thrusters = vec![merlin(Vec3::new(0.0, -23.5, 0.0))];
    for i in 0..8 {
        let a = (i as f64) * std::f64::consts::TAU / 8.0;
        s1_thrusters.push(merlin(Vec3::new(r * a.cos(), -23.5, r * a.sin())));
    }

    vec![
        StageSpec {
            name: "F9-S1",
            dry_mass: 25_600.0,
            thrusters: s1_thrusters,
            length: 47.0,
            radius: 1.85,
            separation_impulse: 3.0,
            tidaldamp: 0.0,
            cd_mach: cd_mach_rocket(),
            lifting_surfaces: radial_fins(
                4,
                2.1,
                18.0,
                2.5,
                1.2,
                FinKind::Grid,
                DEFAULT_ALPHA_STALL_GRID,
                0.05,
                1.0,
            ),
            ..Default::default()
        }
        .with_fuel(411_000.0),
        StageSpec {
            name: "F9-S2",
            dry_mass: 4_000.0,
            thrusters: vec![thruster(
                Vec3::new(0.0, -7.0, 0.0),
                934_000.0,
                348.0,
                Some(900_000.0),
                Some(330.0),
                0.087,
                0.17,
                0.8,
            )],
            length: 14.0,
            radius: 1.85,
            separation_impulse: 2.0,
            cd_mach: cd_mach_rocket(),
            ..Default::default()
        }
        .with_fuel(107_500.0),
        StageSpec {
            name: "Payload",
            dry_mass: 22_800.0,
            thrusters: vec![],
            length: 5.0,
            radius: 1.85,
            separation_impulse: 1.0,
            cd_mach: cd_mach_rocket(),
            ..Default::default()
        }
        .with_fuel(0.0),
    ]
}

/// Saturn V 三级 + 阿波罗载荷。
pub fn saturn_v() -> Vec<StageSpec> {
    // F-1：海平面 ~6.77 MN / Isp~263；真空 ~7.77 MN / ~304 s
    let f1 = |pos: Vec3| {
        thruster(
            pos,
            7_770_000.0,
            304.0,
            Some(6_770_000.0),
            Some(263.0),
            0.105,
            0.26,
            0.8,
        )
    };
    let mut sic = vec![f1(Vec3::new(0.0, -21.0, 0.0))];
    for i in 0..4 {
        let a = (i as f64) * std::f64::consts::FRAC_PI_2;
        sic.push(f1(Vec3::new(2.5 * a.cos(), -21.0, 2.5 * a.sin())));
    }
    let j2 =
        |pos: Vec3, gimbal: f64| thruster(pos, 1_000_000.0, 421.0, None, None, gimbal, 0.17, 0.8);

    vec![
        StageSpec {
            name: "S-IC",
            dry_mass: 130_000.0,
            thrusters: sic,
            length: 42.0,
            radius: 5.0,
            separation_impulse: 4.0,
            cd_mach: cd_mach_rocket(),
            lifting_surfaces: radial_fins(
                4,
                5.5,
                -18.0,
                8.0,
                1.5,
                FinKind::Fixed,
                DEFAULT_ALPHA_STALL_FIN,
                0.02,
                0.0,
            ),
            ..Default::default()
        }
        .with_fuel(2_150_000.0),
        StageSpec {
            name: "S-II",
            dry_mass: 36_000.0,
            thrusters: {
                let mut t = vec![j2(Vec3::new(0.0, -12.0, 0.0), 0.0)];
                for i in 0..4 {
                    let a = (i as f64) * std::f64::consts::FRAC_PI_2;
                    t.push(j2(Vec3::new(2.0 * a.cos(), -12.0, 2.0 * a.sin()), 0.087));
                }
                t
            },
            length: 24.8,
            radius: 5.0,
            separation_impulse: 3.0,
            cd_mach: cd_mach_rocket(),
            ..Default::default()
        }
        .with_fuel(440_000.0),
        StageSpec {
            name: "S-IVB",
            dry_mass: 10_000.0,
            thrusters: vec![j2(Vec3::new(0.0, -8.5, 0.0), 0.087)],
            length: 17.8,
            radius: 3.3,
            separation_impulse: 2.0,
            cd_mach: cd_mach_rocket(),
            ..Default::default()
        }
        .with_fuel(110_000.0),
        StageSpec {
            name: "CSM-LM",
            dry_mass: 45_000.0,
            thrusters: vec![],
            length: 10.0,
            radius: 3.3,
            separation_impulse: 1.5,
            cd_mach: cd_mach_rocket(),
            ..Default::default()
        }
        .with_fuel(0.0),
    ]
}
