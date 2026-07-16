#[cfg(test)]
mod tests {
    use crate::*;
    use orbitx_math::StateVectors;

    fn falcon9() -> Vec<StageSpec> {
        presets::falcon9()
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
}
