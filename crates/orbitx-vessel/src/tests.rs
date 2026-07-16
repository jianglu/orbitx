#[cfg(test)]
mod tests {
    use crate::*;
    use orbitx_math::StateVectors;

    fn falcon9() -> Vec<StageSpec> {
        presets::falcon9()
    }

    /// 断言两个 Assembly 的活动级状态在所有分量上逐位（bit）相等。
    ///
    /// 这是可复现性的严格标准：确定性意味着完全相同的浮点位模式，而非仅
    /// 容差内接近。比较 pos/vel/omega（各 3 分量）+ q（4 分量）= 13 个 f64。
    fn assert_states_identical(a: &Assembly, b: &Assembly, ctx: &str) {
        let s1 = a.vessels[a.active].state;
        let s2 = b.vessels[b.active].state;
        let pairs = [
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
        ];
        for (name, v1, v2) in pairs {
            assert_eq!(
                v1.to_bits(),
                v2.to_bits(),
                "{ctx}: {name} 不一致: {v1} vs {v2}"
            );
        }
    }

    /// 断言两个 Assembly 的燃料质量逐位相等（所有级）。
    fn assert_fuel_identical(a: &Assembly, b: &Assembly, ctx: &str) {
        assert_eq!(
            a.vessels.len(),
            b.vessels.len(),
            "{ctx}: 级数不一致"
        );
        for (i, (va, vb)) in a.vessels.iter().zip(b.vessels.iter()).enumerate() {
            assert_eq!(
                va.fuel_mass.to_bits(),
                vb.fuel_mass.to_bits(),
                "{ctx}: 第{i}级燃料不一致: {} vs {}",
                va.fuel_mass,
                vb.fuel_mass
            );
        }
    }

    #[test]
    fn total_mass() {
        let asm = Assembly::new(&falcon9(), StateVectors::default());
        // S1: 25600 + 411000 = 436600
        // S2: 4000 + 107500 = 111500
        // Payload: 22800
        let expected = 436_600.0 + 111_500.0 + 22_800.0;
        assert!(
            (asm.total_mass() - expected).abs() < 0.1,
            "总质量: {} vs 期望 {}",
            asm.total_mass(),
            expected
        );
    }

    #[test]
    fn stage_count() {
        let asm = Assembly::new(&falcon9(), StateVectors::default());
        assert_eq!(asm.stage_count(), 3);
    }

    #[test]
    fn separation_reduces_stages() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        assert_eq!(asm.stage_count(), 3);
        asm.separate_stage();
        assert_eq!(asm.stage_count(), 2);
        assert_eq!(asm.active_name(), "F9-S2");
    }

    #[test]
    fn double_separation() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.separate_stage();
        asm.separate_stage();
        assert_eq!(asm.stage_count(), 1);
        assert_eq!(asm.active_name(), "Payload");
    }

    #[test]
    fn no_separation_when_one_stage() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.separate_stage();
        asm.separate_stage();
        let active = asm.active;
        asm.separate_stage(); // 不应崩溃
        assert_eq!(asm.active, active);
        assert_eq!(asm.stage_count(), 1);
    }

    #[test]
    fn fuel_consumption() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.set_throttle(1.0);
        let fuel_before = asm.total_fuel();
        // 1 秒，无引力。
        asm.step(1.0, &[]);
        let fuel_after = asm.total_fuel();
        assert!(
            fuel_after < fuel_before,
            "燃料应减少: {} -> {}",
            fuel_before,
            fuel_after
        );
        // 消耗率 = thrust / (isp * g0) = 7607000 / (282 * 9.80665) ≈ 2753.8 kg/s
        let expected_rate = 7_607_000.0 / (282.0 * thruster::G0);
        let actual_rate = fuel_before - fuel_after;
        let rel_err = (actual_rate - expected_rate).abs() / expected_rate;
        // RK4 积分 + 燃料在步末一次性扣除，与连续消耗有轻微差异。
        assert!(
            rel_err < 0.15,
            "消耗率误差 {rel_err}: {actual_rate} vs {expected_rate}"
        );
    }

    #[test]
    fn dock_connections() {
        let asm = Assembly::new(&falcon9(), StateVectors::default());
        // S1 顶端口应连接到 S2 底端口。
        let s1 = &asm.vessels[0];
        let s2 = &asm.vessels[1];
        assert!(s1.docks[1].connected_to.is_some(), "S1 顶端口应已连接");
        assert!(s2.docks[0].connected_to.is_some(), "S2 底端口应已连接");
        // 验证连接目标是正确的 ID。
        let (s1_conn_id, _) = s1.docks[1].connected_to.unwrap();
        assert_eq!(s1_conn_id, s2.id);
    }

    #[test]
    fn separation_breaks_docks() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.separate_stage();
        // S1 应被标记为 detached。
        assert!(asm.vessels[0].detached);
        // S2 底端口应断开。
        let s2 = &asm.vessels[1];
        assert!(s2.docks[0].connected_to.is_none(), "S2 底端口应已断开");
    }

    #[test]
    fn throttle_clamps() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.set_throttle(1.5); // 超范围
        for v in &asm.vessels {
            if !v.detached {
                for t in &v.thrusters {
                    assert!(t.level <= 1.0);
                }
            }
        }
    }

    #[test]
    fn saturn_v_mass() {
        let specs = presets::saturn_v();
        let asm = Assembly::new(&specs, StateVectors::default());
        // S-IC: 130000 + 2150000 = 2280000
        // S-II: 36000 + 440000 = 476000
        // S-IVB: 10000 + 110000 = 120000
        // CSM-LM: 45000
        let expected = 2_280_000.0 + 476_000.0 + 120_000.0 + 45_000.0;
        assert!(
            (asm.total_mass() - expected).abs() < 0.1,
            "Saturn V 总质量: {} vs 期望 {}",
            asm.total_mass(),
            expected
        );
    }

    #[test]
    fn composite_pmi_is_axisymmetric() {
        // Falcon 9 各级都是圆截面（pmi.x == pmi.z），同轴堆叠后组合体也应
        // 保持 pmi.x == pmi.z（绕纵轴对称）。
        let asm = Assembly::new(&falcon9(), StateVectors::default());
        let pmi = asm.composite_pmi();
        assert!(
            (pmi.x - pmi.z).abs() < 1.0,
            "组合体应保持轴向对称: pmi.x={} pmi.z={}",
            pmi.x,
            pmi.z
        );
        // 轴向惯量（Y）应远小于横向（X/Z），因为火箭细长。
        assert!(
            pmi.y < pmi.x,
            "轴向惯量应小于横向: pmi.y={} pmi.x={}",
            pmi.y,
            pmi.x
        );
    }

    #[test]
    fn gimbal_deflection_produces_angular_acceleration() {
        // 端到端：施加 gimbal 偏转 → 推力偏心 → 产生力矩 → 角速度变化。
        // 单级火箭，全油门，gimbal 设为最大偏转，积分若干步后角速度应非零。
        use orbitx_math::{Matrix3, Quat, Vec3};
        let spec = StageSpec {
            name: "test",
            dry_mass: 1000.0,
            fuel_mass: 5000.0,
            thrust: 200_000.0, // 推力足以产生明显力矩
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -5.0, 0.0), // 发动机在尾部
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0), // 用默认推断
            max_gimbal: 0.2,
            max_gimbal_rate: 100.0, // 无速率限制，立即到位
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        let mut asm = Assembly::new(
            &[spec],
            StateVectors {
                pos: Vec3::new(0.0, 0.0, 6_371_000.0),
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: Matrix3::IDENTITY,
                q: Quat::IDENTITY,
            },
        );
        asm.set_throttle(1.0);
        // 设置 gimbal 到最大偏转。
        for v in &mut asm.vessels {
            for t in &mut v.thrusters {
                t.set_gimbal(0.2);
            }
        }
        let omega_before = asm.vessels[0].state.omega.x;
        // 积分 0.5 秒（无引力体，纯推力力矩）。
        asm.step(0.5, &[]);
        let omega_after = asm.vessels[0].state.omega.x;
        assert!(
            omega_after.abs() > omega_before.abs(),
            "gimbal 偏转应产生角加速度: ω_before={} ω_after={}",
            omega_before,
            omega_after
        );
        assert!(
            omega_after.abs() > 1e-6,
            "角速度应明显非零: {}",
            omega_after
        );
    }

    #[test]
    fn no_gimbal_no_angular_motion() {
        // 对照组：gimbal=0（推力对准质心）→ 无力矩 → 角速度保持零。
        use orbitx_math::{Matrix3, Quat, Vec3};
        let spec = StageSpec {
            name: "test",
            dry_mass: 1000.0,
            fuel_mass: 5000.0,
            thrust: 200_000.0,
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -5.0, 0.0),
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.2,
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        let mut asm = Assembly::new(
            &[spec],
            StateVectors {
                pos: Vec3::new(0.0, 0.0, 6_371_000.0),
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: Matrix3::IDENTITY,
                q: Quat::IDENTITY,
            },
        );
        asm.set_throttle(1.0);
        // gimbal 保持 0（默认）。
        asm.step(0.5, &[]);
        let omega = asm.vessels[0].state.omega;
        assert!(
            omega.length() < 1e-9,
            "无 gimbal 偏转时角速度应为零: {:?}",
            omega
        );
    }

    /// 端到端：发射台垂直姿态 + engine_dir=+Y + 推力 → 火箭应沿径向向上加速。
    /// 验证推力方向链路：base_dir(+Y) 经 launch_attitude 旋转 → 世界 +Z（径向）。
    #[test]
    fn vertical_thrust_accelerates_upward() {
        use orbitx_math::{cross, dot, mul, Matrix3, Quat, Vec3};
        let spec = StageSpec {
            name: "test",
            dry_mass: 1000.0,
            fuel_mass: 5000.0,
            thrust: 200_000.0,
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0), // 推力朝头部
            engine_pos: Vec3::new(0.0, -5.0, 0.0),
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.0, // 无 TVC，纯垂直
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        // 发射点在 +Z 轴：pos=(0,0,Re)，径向 up=+Z。
        let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
        let up = pos * (1.0 / pos.length());
        // 构造 launch_attitude：体 +Y 对齐 up（+Z）。
        let ref_axis = Vec3::new(0.0, 1.0, 0.0);
        let bx = cross(up, ref_axis).unit();
        let bz = cross(bx, up).unit();
        let by = up;
        let rot = Matrix3::new(
            bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z,
        );
        let q = Quat::from_matrix(rot);

        let mut asm = Assembly::new(
            &[spec],
            StateVectors {
                pos,
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: rot,
                q,
            },
        );
        asm.set_throttle(1.0);
        asm.step(1.0, &[]); // 无引力，纯推力

        let vel = asm.vessels[0].state.vel;
        let v_radial = dot(vel, up);
        // 推力 200kN / 质量 6000kg ≈ 33 m/s² × 1s → 径向速度应明显为正。
        assert!(
            v_radial > 10.0,
            "火箭应沿径向向上加速，v_radial={}",
            v_radial
        );
        // 切向分量应接近零（无水平漂移）。
        let v_tan = vel - up * v_radial;
        assert!(
            v_tan.length() < 1.0,
            "不应有显著水平速度: {:?}",
            v_tan
        );
        // 角速度应保持零（推力对准质心）。
        assert!(
            asm.vessels[0].state.omega.length() < 1e-6,
            "垂直推力不应产生角速度"
        );
        let _ = mul;
    }

    /// TVC 方向：正 gimbal 偏转应使火箭俯仰角增大（姿态前倾）。
    /// 验证控制闭环符号正确——若反了，重力转向会让火箭朝错误方向翻滚。
    #[test]
    fn tvc_positive_gimbal_increases_pitch() {
        use orbitx_math::{cross, dot, mul, Matrix3, Quat, Vec3};
        let spec = StageSpec {
            name: "test",
            dry_mass: 1000.0,
            fuel_mass: 5000.0,
            thrust: 200_000.0,
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -5.0, 0.0),
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.2,
            max_gimbal_rate: 100.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
        let up = pos * (1.0 / pos.length());
        let ref_axis = Vec3::new(0.0, 1.0, 0.0);
        let bx = cross(up, ref_axis).unit();
        let bz = cross(bx, up).unit();
        let by = up;
        let rot = Matrix3::new(
            bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z,
        );
        let q = Quat::from_matrix(rot);

        let mut asm = Assembly::new(
            &[spec],
            StateVectors {
                pos,
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: rot,
                q,
            },
        );
        asm.set_throttle(1.0);

        // 当前俯仰角（体 +Y 与径向的夹角）。
        let pitch_of = |a: &Assembly| -> f64 {
            let s = a.vessels[0].state;
            let body_y_world = mul(s.r, Vec3::new(0.0, 1.0, 0.0));
            let radial = s.pos * (1.0 / s.pos.length());
            dot(body_y_world, radial).clamp(-1.0, 1.0).acos()
        };

        let pitch0 = pitch_of(&asm);
        // 正 gimbal 偏转。
        for v in &mut asm.vessels {
            for t in &mut v.thrusters {
                t.set_gimbal(0.15);
            }
        }
        // 积分若干步让姿态演化。
        for _ in 0..10 {
            asm.step(0.05, &[]);
        }
        let pitch1 = pitch_of(&asm);
        assert!(
            pitch1 > pitch0,
            "正 gimbal 应增大俯仰角: {} -> {}",
            pitch0.to_degrees(),
            pitch1.to_degrees()
        );
        let _ = pitch0;
    }

    /// 可复现性：相同初始条件 + 固定步长，两次运行的最终状态必须逐位相等。
    /// 这依赖物理层是纯函数、单线程、无随机源——逐符号移植 Orbiter 的核心保证。
    #[test]
    fn fixed_dt_is_deterministic() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::Vec3;
        let mk = || {
            let mut a = Assembly::new(&falcon9(), StateVectors::default());
            a.set_throttle(1.0);
            a
        };
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![], rotation: None, pines: None,
        };
        let dt = 0.05_f64;

        let mut a1 = mk();
        let mut a2 = mk();
        for _ in 0..100 {
            a1.step(dt, &[earth.clone()]);
            a2.step(dt, &[earth.clone()]);
        }
        assert_states_identical(&a1, &a2, "固定步长垂直飞行");
        assert_fuel_identical(&a1, &a2, "固定步长垂直飞行");
    }

    /// TVC 活跃（gimbal 偏转驱动姿态演化）下仍需逐位可复现。
    /// 这是最严格的确定性场景——刚体姿态动力学路径必须完全一致。
    #[test]
    fn tvc_active_is_deterministic() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{cross, dot, mul, Matrix3, Quat, Vec3};
        let mk = || {
            let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
            let up = pos * (1.0 / pos.length());
            let bx = cross(up, Vec3::new(0.0, 1.0, 0.0)).unit();
            let bz = cross(bx, up).unit();
            let rot = Matrix3::new(bx.x, up.x, bz.x, bx.y, up.y, bz.y, bx.z, up.z, bz.z);
            let q = Quat::from_matrix(rot);
            let mut a = Assembly::new(
                &falcon9(),
                StateVectors {
                    pos,
                    vel: Vec3::ZERO,
                    omega: Vec3::ZERO,
                    r: rot,
                    q,
                },
            );
            a.set_throttle(1.0);
            // 固定 gimbal 偏转，驱动 TVC 力矩。
            for v in &mut a.vessels {
                for t in &mut v.thrusters {
                    t.set_gimbal(0.08);
                }
            }
            a
        };
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![], rotation: None, pines: None,
        };
        let dt = 0.05_f64;

        let mut a1 = mk();
        let mut a2 = mk();
        for _ in 0..60 {
            a1.step(dt, &[earth.clone()]);
            a2.step(dt, &[earth.clone()]);
        }
        assert_states_identical(&a1, &a2, "TVC 活跃姿态演化");
    }

    /// 变步长序列（模拟 time_scale 切换）的可复现性。
    /// 相同的 dt 序列 → 相同结果，即使步长在过程中变化。
    #[test]
    fn variable_dt_sequence_is_deterministic() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::Vec3;
        let mk = || {
            let mut a = Assembly::new(&falcon9(), StateVectors::default());
            a.set_throttle(1.0);
            a
        };
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![], rotation: None, pines: None,
        };
        // 变步长序列：模拟 1x → 2x → 0.5x 切换。
        let dt_seq = [0.05, 0.05, 0.1, 0.1, 0.025, 0.05, 0.1, 0.025, 0.05, 0.05];

        let mut a1 = mk();
        let mut a2 = mk();
        for &dt in &dt_seq {
            for _ in 0..10 {
                a1.step(dt, &[earth.clone()]);
                a2.step(dt, &[earth.clone()]);
            }
        }
        assert_states_identical(&a1, &a2, "变步长序列");
        assert_fuel_identical(&a1, &a2, "变步长序列");
    }

    /// 级分离后的确定性：separate_stage 不引入随机性，分离后轨迹仍可复现。
    #[test]
    fn staging_is_deterministic() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::Vec3;
        let mk = || {
            let mut a = Assembly::new(&falcon9(), StateVectors::default());
            a.set_throttle(1.0);
            a
        };
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![], rotation: None, pines: None,
        };
        let dt = 0.05_f64;

        let mut a1 = mk();
        let mut a2 = mk();
        for i in 0..200 {
            a1.step(dt, &[earth.clone()]);
            a2.step(dt, &[earth.clone()]);
            // 在相同时机分离。
            if i == 50 || i == 120 {
                a1.separate_stage();
                a2.separate_stage();
            }
        }
        assert_eq!(a1.stage_count(), a2.stage_count(), "分离后级数不一致");
        assert_eq!(a1.active, a2.active, "活动级索引不一致");
        assert_states_identical(&a1, &a2, "级分离后");
    }

    /// 所有内置预设火箭都需满足可复现性。
    #[test]
    fn all_presets_are_deterministic() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::Vec3;
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![], rotation: None, pines: None,
        };
        let dt = 0.05_f64;
        let presets: Vec<(&str, Vec<StageSpec>)> = vec![
            ("Falcon9", presets::falcon9()),
            ("SaturnV", presets::saturn_v()),
        ];
        for (name, spec) in presets {
            let mk = || {
                let mut a = Assembly::new(&spec, StateVectors::default());
                a.set_throttle(1.0);
                a
            };
            let mut a1 = mk();
            let mut a2 = mk();
            for _ in 0..80 {
                a1.step(dt, &[earth.clone()]);
                a2.step(dt, &[earth.clone()]);
            }
            assert_states_identical(&a1, &a2, name);
            assert_fuel_identical(&a1, &a2, name);
        }
    }

    /// 重置（重新创建 Assembly）后从相同初始条件出发，轨迹必须可复现。
    /// 验证 Assembly::new 本身不依赖任何全局/随机状态。
    #[test]
    fn rebuild_produces_identical_trajectory() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::Vec3;
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![], rotation: None, pines: None,
        };
        let dt = 0.05_f64;
        let spec = falcon9();

        // 第一次运行。
        let mut a1 = Assembly::new(&spec, StateVectors::default());
        a1.set_throttle(1.0);
        for _ in 0..50 {
            a1.step(dt, &[earth.clone()]);
        }
        let snapshot = a1.vessels[0].state;

        // 重新创建并运行（模拟 reset）。
        let mut a2 = Assembly::new(&spec, StateVectors::default());
        a2.set_throttle(1.0);
        for _ in 0..50 {
            a2.step(dt, &[earth.clone()]);
        }
        assert_states_identical(&a1, &a2, "重建后轨迹");
        let _ = snapshot;
    }

    // ── P1 集成/端到端测试 ──────────────────────────────────────────

    /// Falcon 9 完整上升（含气动），不崩溃。
    #[test]
    fn falcon9_full_ascent_with_aero() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{Vec3, Matrix3, Quat, cross, dot};
        use crate::aero::{DragElement, ExponentialAtmosphere};
        use crate::touchdown::TouchdownVertex;

        let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
        let up = pos * (1.0 / pos.length());
        let ref_axis = Vec3::new(0.0, 1.0, 0.0);
        let bx = cross(up, ref_axis).unit();
        let bz = cross(bx, up).unit();
        let rot = Matrix3::new(bx.x, up.x, bz.x, bx.y, up.y, bz.y, bx.z, up.z, bz.z);
        let q = Quat::from_matrix(rot);

        let mut asm = Assembly::new(&falcon9(), StateVectors {
            pos, vel: Vec3::ZERO, omega: Vec3::ZERO, r: rot, q,
        });
        // 配置气动：简单阻力元件。
        for v in &mut asm.vessels {
            v.dragels.push(DragElement { ref_pos: Vec3::ZERO, cd: 0.3, area: 10.0 });
            v.cross_section = Vec3::new(1.0, 10.0, 1.0);
            v.rdrag = Vec3::new(1.0, 0.1, 1.0);
        }
        asm.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
        asm.planet_radius = 6_371_000.0;

        asm.set_throttle(1.0);
        let earth = GravBody { pos: Vec3::ZERO, mass: 5.972e24, size: 6_371_000.0, jcoeff: vec![], rotation: None, pines: None };
        let dt = 0.05;

        // 运行 200 步（10 秒）——不应崩溃。
        for _ in 0..200 {
            asm.step(dt, &[earth.clone()]);
        }
        // 高度应增加（推力 > 重力 + 阻力）。
        let alt = asm.vessels[asm.active].state.pos.length() - 6_371_000.0;
        assert!(alt > 0.0, "F9 应已离开发射台: alt = {alt} m");
    }

    /// 再入时气动减速：有阻力 vs 无阻力对照。
    #[test]
    fn reentry_deceleration() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{Vec3, Matrix3, Quat};
        use crate::aero::{DragElement, ExponentialAtmosphere};

        // 单级，无推力，从 30 km 以 1000 m/s 水平速度。
        let spec = StageSpec {
            name: "reentry",
            dry_mass: 10000.0,
            fuel_mass: 0.0,
            thrust: 0.0,
            isp: 0.0,
            engine_dir: Vec3::ZERO,
            engine_pos: Vec3::ZERO,
            length: 10.0,
            radius: 2.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        let init_state = StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 30_000.0),
            vel: Vec3::new(1000.0, 0.0, -50.0),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };

        // 有阻力版本。
        let mut asm_aero = Assembly::new(&[spec.clone()], init_state);
        asm_aero.vessels[0].dragels.push(DragElement { ref_pos: Vec3::ZERO, cd: 0.5, area: 5.0 });
        asm_aero.vessels[0].cross_section = Vec3::new(1.0, 5.0, 1.0);
        asm_aero.vessels[0].rdrag = Vec3::new(1.0, 0.1, 1.0);
        asm_aero.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
        asm_aero.planet_radius = 6_371_000.0;

        // 无阻力版本。
        let mut asm_no_aero = Assembly::new(&[spec.clone()], init_state);
        // 不配置大气。

        let earth = GravBody { pos: Vec3::ZERO, mass: 5.972e24, size: 6_371_000.0, jcoeff: vec![], rotation: None, pines: None };
        let dt = 0.01;

        for _ in 0..3000 {
            asm_aero.step(dt, &[earth.clone()]);
            asm_no_aero.step(dt, &[earth.clone()]);
            let alt = asm_aero.vessels[0].state.pos.length() - 6_371_000.0;
            if alt < 0.0 || alt > 200_000.0 {
                break;
            }
        }
        // 有阻力版本速度应低于无阻力版本。
        let v_aero = asm_aero.vessels[0].state.vel.length();
        let v_no_aero = asm_no_aero.vessels[0].state.vel.length();
        assert!(
            v_aero < v_no_aero,
            "有阻力应比无阻力慢: v_aero={v_aero:.0} v_no_aero={v_no_aero:.0}"
        );
    }

    /// RCS 姿态控制产生角速度。
    #[test]
    fn rcs_attitude_hold() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{Vec3, Matrix3, Quat};
        use crate::rcs::{add_default_rcs, set_attitude_rot, RotAxis};

        let spec = StageSpec {
            name: "rcs-test",
            dry_mass: 5000.0,
            fuel_mass: 5000.0,
            thrust: 0.0,
            isp: 0.0,
            engine_dir: Vec3::ZERO,
            engine_pos: Vec3::ZERO,
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        let mut asm = Assembly::new(&[spec], StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0),
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        });
        add_default_rcs(&mut asm.vessels[0], 5.0, 10_000.0);
        set_attitude_rot(&mut asm.vessels[0], RotAxis::Pitch, 1.0);

        let earth = GravBody { pos: Vec3::ZERO, mass: 5.972e24, size: 6_371_000.0, jcoeff: vec![], rotation: None, pines: None };
        let dt = 0.05;
        for _ in 0..20 {
            asm.step(dt, &[earth.clone()]);
        }
        let omega = asm.vessels[0].state.omega;
        // RCS 俯仰应产生角速度。
        assert!(omega.x.abs() > 1e-6, "RCS 应产生角速度: omega = {:?}", omega);
    }

    /// 多储箱独立消耗。
    #[test]
    fn multi_tank_independent_consumption() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{Vec3, Matrix3, Quat};
        use crate::fuel::PropellantTank;

        let spec = StageSpec {
            name: "multi-tank",
            dry_mass: 5000.0,
            fuel_mass: 0.0, // 不用旧式 fuel_mass
            thrust: 100_000.0,
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -5.0, 0.0),
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        let mut asm = Assembly::new(&[spec], StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0),
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        });
        // 添加两个储箱，推进器从 tank 0 消耗。
        asm.vessels[0].tanks.push(PropellantTank::new(0, 500.0, 1.0));
        asm.vessels[0].tanks.push(PropellantTank::new(1, 300.0, 1.0));
        asm.vessels[0].thrusters[0].tank_id = Some(0);

        asm.set_throttle(1.0);
        let earth = GravBody { pos: Vec3::ZERO, mass: 5.972e24, size: 6_371_000.0, jcoeff: vec![], rotation: None, pines: None };
        asm.step(1.0, &[earth.clone()]);

        // Tank 0 应减少，tank 1 不变。
        assert!(asm.vessels[0].tanks[0].mass < 500.0, "tank 0 应消耗燃料");
        assert!((asm.vessels[0].tanks[1].mass - 300.0).abs() < 1e-6, "tank 1 不应消耗");
    }

    /// 着陆触点使下沉停止。
    #[test]
    fn landing_touchdown_stops_descent() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{Vec3, Matrix3, Quat};
        use crate::touchdown::{TouchdownVertex, compute_surface_forces};

        let spec = StageSpec {
            name: "lander",
            dry_mass: 2000.0,
            fuel_mass: 0.0,
            thrust: 0.0,
            isp: 0.0,
            engine_dir: Vec3::ZERO,
            engine_pos: Vec3::ZERO,
            length: 10.0,
            radius: 2.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
        };
        let mut asm = Assembly::new(&[spec], StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 5.0), // 5 m 高度
            vel: Vec3::new(0.0, 0.0, -2.0), // 2 m/s 下沉
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        });
        asm.planet_radius = 6_371_000.0;
        // 添加着陆架。
        asm.vessels[0].touchdown_points = crate::touchdown::make_landing_gear(2.0, -5.0, 5e5, 1e4, 0.5);

        let earth = GravBody { pos: Vec3::ZERO, mass: 5.972e24, size: 6_371_000.0, jcoeff: vec![], rotation: None, pines: None };
        let dt = 0.01;

        // 运行 500 步（5 秒）——着陆后应稳定。
        for _ in 0..500 {
            asm.step(dt, &[earth.clone()]);
            // 计算接触力并施加。
            let contact = compute_surface_forces(
                &asm.vessels[0].touchdown_points,
                &asm.vessels[0].state,
                6_371_000.0,
                dt,
                asm.total_mass(),
            );
            if contact.in_contact {
                let m = asm.total_mass();
                for v in &mut asm.vessels {
                    if !v.detached {
                        v.state.vel += contact.force * (dt / m);
                    }
                }
            }
        }
        // 最终下沉速度应很小（< 0.5 m/s）。
        let vz = asm.vessels[0].state.vel.z;
        assert!(vz > -0.5, "着陆后下沉速度应很小: vz = {vz:.3}");
    }
}
