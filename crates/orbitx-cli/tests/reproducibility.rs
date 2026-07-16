//! 可复现性集成测试：从 TOML 配置解析到物理积分的完整链路。
//!
//! 验证相同输入（rocket.toml + 固定步长）始终产生逐位一致的轨迹。
//! 这些测试覆盖单元测试无法触及的 config → StageSpec 转换链路。

use orbitx_config::RocketConfig;
use orbitx_dynamics::GravBody;
use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::{StageSpec, Assembly};

/// 复刻 main.rs 的 rocket_to_stages（私有函数，这里内联以测试转换链路）。
fn rocket_to_stages(config: &RocketConfig) -> Vec<StageSpec> {
    config.stages.iter().map(|s| {
        let pmi = s.inertia
            .map(|i| Vec3::new(i[0], i[1], i[2]))
            .unwrap_or(orbitx_vessel::stage::PMI_UNDEF);
        StageSpec {
            name: Box::leak(s.name.clone().into_boxed_str()),
            dry_mass: s.dry_mass,
            fuel_mass: s.fuel_mass,
            thrust: s.thrust,
            isp: s.isp,
            engine_dir: Vec3::new(s.engine_dir[0], s.engine_dir[1], s.engine_dir[2]),
            engine_pos: Vec3::new(s.engine_pos[0], s.engine_pos[1], s.engine_pos[2]),
            length: s.length,
            radius: s.radius,
            separation_impulse: s.separation_impulse,
            pmi,
            max_gimbal: s.max_gimbal,
            max_gimbal_rate: s.max_gimbal_rate,
            gimbal_axis: Vec3::new(s.gimbal_axis[0], s.gimbal_axis[1], s.gimbal_axis[2]),
        }
    }).collect()
}

const FALCON9_TOML: &str = include_str!("../../orbitx-config/presets/falcon9.toml");
const SATURNV_TOML: &str = include_str!("../../orbitx-config/presets/saturn_v.toml");

fn earth() -> GravBody {
    GravBody {
        pos: Vec3::ZERO,
        mass: 5.972e24,
        size: 6_371_000.0,
        jcoeff: vec![], rotation: None, pines: None,
    }
}

/// 断言两个活动级状态 13 个分量逐位相等。
fn assert_bit_identical(a: &Assembly, b: &Assembly, ctx: &str) {
    let s1 = a.vessels[a.active].state;
    let s2 = b.vessels[b.active].state;
    for (name, v1, v2) in [
        ("pos.x", s1.pos.x, s2.pos.x),
        ("pos.y", s1.pos.y, s2.pos.y),
        ("pos.z", s1.pos.z, s2.pos.z),
        ("vel.x", s1.vel.x, s2.vel.x),
        ("vel.y", s1.vel.y, s2.vel.y),
        ("vel.z", s1.vel.z, s2.vel.z),
        ("omega.x", s1.omega.x, s2.omega.x),
        ("omega.y", s1.omega.y, s2.omega.y),
        ("omega.z", s1.omega.z, s2.omega.z),
        ("q.vx", s1.q.vx, s2.q.vx),
        ("q.vy", s1.q.vy, s2.q.vy),
        ("q.vz", s1.q.vz, s2.q.vz),
        ("q.s", s1.q.s, s2.q.s),
    ] {
        assert_eq!(v1.to_bits(), v2.to_bits(), "{ctx}: {name} 不一致");
    }
}

/// 从 TOML 构建可运行的 Assembly（全油门）。
fn build_from_toml(toml: &str) -> Assembly {
    let config = RocketConfig::from_toml_str(toml).expect("解析 TOML");
    let stages = rocket_to_stages(&config);
    let mut asm = Assembly::new(&stages, StateVectors::default());
    asm.set_throttle(1.0);
    asm
}

#[test]
fn falcon9_toml_trajectory_is_deterministic() {
    let dt = 0.05;
    let earth = earth();
    let mut a1 = build_from_toml(FALCON9_TOML);
    let mut a2 = build_from_toml(FALCON9_TOML);
    for _ in 0..150 {
        a1.step(dt, &[earth.clone()]);
        a2.step(dt, &[earth.clone()]);
    }
    assert_bit_identical(&a1, &a2, "Falcon9 TOML 完整链路");
}

#[test]
fn saturnv_toml_trajectory_is_deterministic() {
    let dt = 0.05;
    let earth = earth();
    let mut a1 = build_from_toml(SATURNV_TOML);
    let mut a2 = build_from_toml(SATURNV_TOML);
    for _ in 0..150 {
        a1.step(dt, &[earth.clone()]);
        a2.step(dt, &[earth.clone()]);
    }
    assert_bit_identical(&a1, &a2, "SaturnV TOML 完整链路");
}

/// 不同的步长拆分（如 2×0.025 vs 1×0.05）不应影响可复现性——
/// 两次运行用各自固定的步长序列，结果各自可复现。
#[test]
fn different_step_subdivision_each_deterministic() {
    let earth = earth();
    let dt = 0.05;

    // 序列 A：每个 tick 用 dt。
    let mut a1a = build_from_toml(FALCON9_TOML);
    let mut a1b = build_from_toml(FALCON9_TOML);
    for _ in 0..100 {
        a1a.step(dt, &[earth.clone()]);
        a1b.step(dt, &[earth.clone()]);
    }
    assert_bit_identical(&a1a, &a1b, "序列A（dt=0.05）");

    // 序列 B：每个 tick 用 2×dt/2（总时间相同，子步更多）。
    let mut a2a = build_from_toml(FALCON9_TOML);
    let mut a2b = build_from_toml(FALCON9_TOML);
    for _ in 0..100 {
        a2a.step(dt * 0.5, &[earth.clone()]);
        a2a.step(dt * 0.5, &[earth.clone()]);
        a2b.step(dt * 0.5, &[earth.clone()]);
        a2b.step(dt * 0.5, &[earth.clone()]);
    }
    assert_bit_identical(&a2a, &a2b, "序列B（2×dt/2）");
    // 注意：A 和 B 的轨迹本身不同（步长影响数值积分精度），
    // 但 A vs A、B vs B 必须各自逐位一致。
}

/// 长时间运行（1000+ 步）的确定性——排除浮点误差累积导致的发散。
#[test]
fn long_run_is_deterministic() {
    let dt = 0.05;
    let earth = earth();
    let mut a1 = build_from_toml(FALCON9_TOML);
    let mut a2 = build_from_toml(FALCON9_TOML);
    for _ in 0..1000 {
        a1.step(dt, &[earth.clone()]);
        a2.step(dt, &[earth.clone()]);
    }
    assert_bit_identical(&a1, &a2, "长时间运行（1000步）");
}
