//! 发射台钉死 / 松绑（P4.3 过渡；终态为发射台插件）。

use orbitx_dynamics::{to_body, to_inertial};
use orbitx_math::{dot, Matrix3, Quat, Vec3};
use orbitx_vessel::{surface_inertial_velocity, Assembly, G0};

/// 首次点火后未释放台架的发射失败超时 [s]（仿真时间）。
pub const LAUNCH_FAIL_TIMEOUT_S: f64 = 10.0;

/// 会话台架状态。
///
/// 台位点与姿态钉在**行星体固系**（会话初始化一次，来自 lp 摆放）；
/// 每拍只用当前 `PlanetOrientation.rot` 转到惯性系，不再重算 `launch_attitude`。
#[derive(Debug, Clone)]
pub struct PadState {
    pub launched: bool,
    /// 台架钉点（行星体固系）[m]。
    pub pad_pos_body: Vec3,
    /// 台架姿态（体固系：vessel → 行星体固）。
    pub pad_r_body: Matrix3,
    /// 首次点火的仿真时刻 [s]；固定窗口起点，中途停推不重置（Reset 重建清除）。
    pub first_ignition_t: Option<f64>,
}

impl PadState {
    /// 会话起点：惯性位置/姿态 + 当时体固→惯性 `rot` → 存体固钉死量。
    pub fn new(pad_pos_inertial: Vec3, r_inertial: Matrix3, rot: Matrix3) -> Self {
        Self {
            launched: false,
            pad_pos_body: to_body(rot, pad_pos_inertial),
            pad_r_body: rot.transp().matmul(r_inertial),
            first_ignition_t: None,
        }
    }

    /// 当前惯性系台架钉点。
    pub fn pad_pos_inertial(&self, rot: Matrix3) -> Vec3 {
        to_inertial(rot, self.pad_pos_body)
    }

    /// 当前惯性系台架姿态（vessel → 惯性）。
    pub fn pad_r_inertial(&self, rot: Matrix3) -> Matrix3 {
        rot.matmul(self.pad_r_body)
    }

    /// 回落台面后重新接管：未离地慢速回落台架时调用，下一 tick 起重新钉回，
    /// 点火窗口清零，可再次点火（不产生终局）。
    pub fn reengage(&mut self) {
        self.launched = false;
        self.first_ignition_t = None;
    }
}

/// 记录首次点火时刻；返回 true 表示点火满 10 s 仍未释放（发射失败）。
///
/// 只记第一次点火，窗口固定；中途停推不重置。已释放返回 false。
pub fn note_ignition_and_check_timeout(
    pad: &mut PadState,
    thrusting_level: f64,
    sim_t_s: f64,
) -> bool {
    if pad.launched {
        return false;
    }
    if pad.first_ignition_t.is_none() && thrusting_level > 1e-6 {
        pad.first_ignition_t = Some(sim_t_s);
    }
    match pad.first_ignition_t {
        Some(t0) => sim_t_s - t0 >= LAUNCH_FAIL_TIMEOUT_S,
        None => false,
    }
}

/// 未分离体上有效推力器的最大油门水平（0..1）。WorkFlow 写 `thruster.level` 后靠此松绑。
pub fn max_thrusting_level(asm: &Assembly) -> f64 {
    asm.vessels
        .iter()
        .filter(|v| !v.detached)
        .flat_map(|v| v.thrusters.iter())
        .filter(|t| t.max_thrust > 0.0)
        .map(|t| t.level)
        .fold(0.0_f64, f64::max)
}

/// 未分离体本步大气修正后的有效推力之和 [N]（步进 diagnostics 口径，与遥测一致）。
pub fn effective_thrust(asm: &Assembly) -> f64 {
    asm.vessels
        .iter()
        .filter(|v| !v.detached)
        .map(|v| v.diagnostics.thrust)
        .sum()
}

/// 步后：检测松绑；若仍在台架则钉死（体固钉点 × 当前 `rot` → 惯性）。
///
/// 释放门槛用**大气有效推力**（`effective_thrust_n`，diagnostics 口径）：
/// 有效推力 > 重量×1.05 才释放；真空额定达标但大气修正后不足时不释放、
/// 继续钉台，由点火 10 s 窗口判发射失败。
pub fn after_step(
    asm: &mut Assembly,
    pad: &mut PadState,
    thrusting_level: f64,
    effective_thrust_n: f64,
) {
    if !pad.launched && thrusting_level > 1e-6 {
        let weight = asm.total_mass() * G0;
        if effective_thrust_n > weight * 1.05 {
            pad.launched = true;
        } else {
            let pos = asm.state.pos;
            let vel = asm.state.vel;
            let r_mag = pos.length();
            if r_mag > 1e-3 {
                let v_radial = dot(vel, pos * (1.0 / r_mag));
                if v_radial > 0.5 {
                    pad.launched = true;
                }
            }
        }
    }

    if pad.launched {
        return;
    }

    let rot = asm.planet_orientation.rot;
    let pad_pos = pad.pad_pos_inertial(rot);
    let pad_vel = surface_inertial_velocity(pad_pos, asm.planet_orientation.omega);
    let lock_r = pad.pad_r_inertial(rot);
    let lock_q = Quat::from_matrix(lock_r);
    // 整体刚体锁定：root 船锚在台位、各船保持装配 rest 布局；
    // 勿逐船改写位姿为 pad 位姿（会压扁 dock rest，组合体底沿虚高）。
    asm.lock_primary_rigid_on(pad_pos, pad_vel, lock_r, lock_q);
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_dynamics::PlanetOrientation;
    use orbitx_math::{Quat, StateVectors, PI2};
    use orbitx_vessel::{geocentric_lat_lon_deg, Assembly, StageSpec, ThrusterSpec};

    fn booster_asm() -> Assembly {
        let stage = StageSpec {
            name: "core",
            dry_mass: 10_000.0,
            thrusters: vec![ThrusterSpec {
                pos: Vec3::new(0.0, -10.0, 0.0),
                dir: Vec3::new(0.0, 1.0, 0.0),
                thrust: 500_000.0,
                isp: 300.0,
                max_gimbal: 0.1,
                max_gimbal_rate: 1.0,
                gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
                ..Default::default()
            }],
            length: 20.0,
            radius: 1.5,
            ..Default::default()
        }
        .with_fuel(20_000.0);
        let pad = Vec3::new(0.0, 0.0, 6_371_000.0 + 44.0);
        Assembly::new(
            &[stage],
            StateVectors {
                pos: pad,
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: Default::default(),
                q: Quat::IDENTITY,
            },
        )
    }

    #[test]
    fn max_thrusting_level_reads_thruster_level() {
        let mut asm = booster_asm();
        assert_eq!(max_thrusting_level(&asm), 0.0);
        asm.vessels[0].thrusters[0].level = 0.8;
        assert!((max_thrusting_level(&asm) - 0.8).abs() < 1e-12);
    }

    #[test]
    fn after_step_releases_when_actual_level_high() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 1.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos, Matrix3::IDENTITY, Matrix3::IDENTITY);
        let level = max_thrusting_level(&asm);
        after_step(&mut asm, &mut pad, level, 500_000.0);
        assert!(pad.launched, "有效推力大于重量时应松绑");
    }

    #[test]
    fn after_step_holds_when_effective_thrust_insufficient() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 1.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos, Matrix3::IDENTITY, Matrix3::IDENTITY);
        let weight = asm.total_mass() * orbitx_vessel::G0;
        after_step(&mut asm, &mut pad, 1.0, weight * 1.02);
        assert!(!pad.launched, "有效推力未超重量 1.05 倍不应释放");
    }

    #[test]
    fn reengage_allows_second_release() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 1.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos, Matrix3::IDENTITY, Matrix3::IDENTITY);
        after_step(&mut asm, &mut pad, 1.0, 500_000.0);
        assert!(pad.launched);
        pad.first_ignition_t = Some(42.0);
        pad.reengage();
        assert!(!pad.launched);
        assert_eq!(pad.first_ignition_t, None);
        after_step(&mut asm, &mut pad, 1.0, 500_000.0);
        assert!(pad.launched, "重新钉回后应可再次释放");
    }

    #[test]
    fn ignition_timeout_ten_seconds_fixed_window() {
        use super::note_ignition_and_check_timeout;
        let pad_pos = Vec3::new(0.0, 0.0, 6_371_000.0);
        let mut pad = PadState::new(pad_pos, Matrix3::IDENTITY, Matrix3::IDENTITY);
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.0, 100.0));
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.5, 100.0));
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.0, 105.0));
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.5, 109.9));
        assert!(note_ignition_and_check_timeout(&mut pad, 0.5, 110.0));
        pad.launched = true;
        assert!(!note_ignition_and_check_timeout(&mut pad, 0.5, 200.0));
    }

    #[test]
    fn after_step_stays_pinned_when_level_zero() {
        let mut asm = booster_asm();
        asm.planet_radius = 6_371_000.0;
        asm.vessels[0].thrusters[0].level = 0.0;
        let pad_pos = asm.state.pos;
        let mut pad = PadState::new(pad_pos, Matrix3::IDENTITY, Matrix3::IDENTITY);
        after_step(&mut asm, &mut pad, 0.0, 0.0);
        assert!(!pad.launched);
        assert_eq!(asm.state.vel, Vec3::ZERO);
    }

    #[test]
    fn after_step_preserves_dock_rest_layout() {
        // 钉死 = root 锚定 + 各船保持装配 rest 布局；勿把非 root 船压扁进
        // root 位姿（曾使侧挂助推器底沿假性抬高 ~4.5 m，UI 高度虚高）。
        let a = StageSpec {
            name: "core",
            dry_mass: 10_000.0,
            length: 20.0,
            radius: 1.5,
            ..Default::default()
        }
        .with_fuel(20_000.0);
        let b = StageSpec {
            name: "booster",
            dry_mass: 4_000.0,
            length: 10.0,
            radius: 1.5,
            ..Default::default()
        }
        .with_fuel(8_000.0);
        let init = StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 44.0),
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: Default::default(),
            q: Quat::IDENTITY,
        };
        let mut asm = Assembly::with_dock_links(&[a, b], init, &[(0, 1, 1, 0)]);
        asm.planet_radius = 6_371_000.0;
        assert!(
            asm.components.len() == 2,
            "both vessels should be docked into primary"
        );
        let root_pos = asm.vessels[0].state.pos;
        let rest_rel = asm.vessels[1].state.pos - root_pos;
        assert!(
            rest_rel.length() > 1.0,
            "booster should sit offset from root (rest), got {:?}",
            rest_rel
        );
        let mut pad = PadState::new(root_pos, Matrix3::IDENTITY, Matrix3::IDENTITY);
        after_step(&mut asm, &mut pad, 0.0, 0.0);
        assert!(!pad.launched);
        assert!(
            (asm.vessels[0].state.pos - root_pos).length() < 1e-6,
            "root must stay pinned at pad anchor"
        );
        let rel = asm.vessels[1].state.pos - asm.vessels[0].state.pos;
        assert!(
            (rel - rest_rel).length() < 1e-6,
            "rest layout must persist: {:?} vs {:?}",
            rel,
            rest_rel
        );
        assert_eq!(asm.vessels[1].state.vel, Vec3::ZERO);
    }

    #[test]
    fn pad_body_fixed_lon_stable_as_rot_advances() {
        let r = 6_371_000.0;
        let pos0 = Vec3::new(r, 0.0, 0.0);
        let mut pad = PadState::new(pos0, Matrix3::IDENTITY, Matrix3::IDENTITY);
        let (lat0, lon0) = geocentric_lat_lon_deg(pad.pad_pos_body);

        let angle = PI2 * 60.0 / 86_164.0;
        let c = angle.cos();
        let s = angle.sin();
        let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        let omega = Vec3::new(0.0, PI2 / 86_164.0, 0.0);

        let mut asm = booster_asm();
        asm.state.pos = pos0;
        asm.planet_orientation = PlanetOrientation { rot, omega };
        asm.planet_radius = r;
        after_step(&mut asm, &mut pad, 0.0, 0.0);

        let (lat_b, lon_b) = geocentric_lat_lon_deg(pad.pad_pos_body);
        assert!((lat_b - lat0).abs() < 1e-9);
        assert!((lon_b - lon0).abs() < 1e-9);
        assert!((asm.state.pos - pos0).length() > 100.0);
        let expect_v = surface_inertial_velocity(asm.state.pos, omega);
        assert!((asm.state.vel - expect_v).length() < 1e-6);
    }

    #[test]
    fn pad_attitude_preserves_body_heading_across_rot() {
        // 初始化带非平凡姿态；R 推进后体固姿态不变，惯性 = R·r_body。
        let r0 = Matrix3::new(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        let pos0 = Vec3::new(0.0, 0.0, 6_371_044.0);
        let mut pad = PadState::new(pos0, r0, Matrix3::IDENTITY);
        assert_eq!(pad.pad_r_body, r0);

        let angle = 0.3_f64;
        let (c, s) = (angle.cos(), angle.sin());
        let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        let mut asm = booster_asm();
        asm.planet_orientation = PlanetOrientation {
            rot,
            omega: Vec3::ZERO,
        };
        after_step(&mut asm, &mut pad, 0.0, 0.0);
        let expect_r = rot.matmul(r0);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (asm.state.r.get(i, j) - expect_r.get(i, j)).abs() < 1e-12,
                    "R[{i},{j}]"
                );
            }
        }
        assert_eq!(pad.pad_r_body, r0);
    }
}
