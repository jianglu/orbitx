//! RCS（反应控制系统）推进器组。
//!
//! 对应 Orbiter 的 `THGROUP_TYPE`（`OrbiterAPI.h:1666`）和
//! `CreateDefaultAttitudeSet`（`Vessel.cpp:2133`）。
//!
//! 推进器组将多个推进器绑定到一个控制通道，使得 `set_group_level`
//! 可以同时控制组内所有推进器。15 个标准组覆盖主发动机、反推、
//! 悬停、6 轴姿态控制（3 旋转 + 3 平移）。

use crate::thruster::Thruster;
use crate::vessel::Vessel;
use orbitx_math::Vec3;

/// 推进器组类型（对应 Orbiter `THGROUP_TYPE`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ThrusterGroupType {
    /// 主发动机。
    Main,
    /// 反推发动机。
    Retro,
    /// 悬停发动机。
    Hover,
    /// 俯仰向上。
    AttPitchUp,
    /// 俯仰向下。
    AttPitchDown,
    /// 偏航左。
    AttYawLeft,
    /// 偏航右。
    AttYawRight,
    /// 滚转左。
    AttBankLeft,
    /// 滚转右。
    AttBankRight,
    /// 平移右。
    AttRight,
    /// 平移左。
    AttLeft,
    /// 平移上。
    AttUp,
    /// 平移下。
    AttDown,
    /// 平移前。
    AttForward,
    /// 平移后。
    AttBack,
}

/// 推进器组（对应 Orbiter `ThrustGroupSpec`，`Vessel.h:95`）。
#[derive(Clone, Debug)]
pub struct ThrusterGroup {
    /// 组类型。
    pub group_type: ThrusterGroupType,
    /// 组内推进器在 Vessel.thrusters 中的索引。
    pub thruster_indices: Vec<usize>,
    /// 组内所有推进器的最大推力之和 [N]。
    pub max_thrust_sum: f64,
}

impl ThrusterGroup {
    /// 创建新的推进器组。
    pub fn new(group_type: ThrusterGroupType, thruster_indices: Vec<usize>, max_thrust_sum: f64) -> Self {
        Self { group_type, thruster_indices, max_thrust_sum }
    }
}

/// 为 Vessel 创建默认的 12 推进器 RCS 布局。
///
/// 移植自 Orbiter `CreateDefaultAttitudeSet`（`Vessel.cpp:2133`）。
///
/// 在体坐标系中放置 12 个小型推进器，构成 6 旋转 + 6 平移组：
/// - **旋转**（力臂 = ±size，方向垂直于力臂）：
///   - 俯仰上下（2 推进器，在 Z=+/-size 处产生 +/-Y 力）
///   - 偏航左右（2 推进器，在 Z=+/-size 处产生 +/-X 力）
///   - 滚转左右（2 推进器，在 X=+/-size 处产生 +/-Y 力）
/// - **平移**（6 推进器，过质心，产生纯力无力矩）：
///   - 上下左右前后（各 1 推进器，每推力 = max_thrust/6）
///
/// # 参数
/// - `vessel`: 要添加 RCS 的航天器
/// - `size`: 航天器特征尺寸 [m]（力臂长度）
/// - `max_thrust`: RCS 总推力 [N]（旋转组每推进器 = max_thrust/2，平移 = max_thrust/6）
pub fn add_default_rcs(vessel: &mut Vessel, size: f64, max_thrust: f64) {
    let base_idx = vessel.thrusters.len();
    let rot_thrust = max_thrust / 2.0;
    let lin_thrust = max_thrust / 6.0;
    let rcs_isp = 220.0;

    // ── 旋转组（6 推进器）──
    // 俯仰向上：在 z=+size 处产生 +Y 力。
    vessel.thrusters.push(Thruster::new(
        Vec3::new(0.0, 0.0, size),
        Vec3::new(0.0, 1.0, 0.0),
        rot_thrust, rcs_isp,
    ));
    // 俯仰向下：在 z=-size 处产生 -Y 力。
    vessel.thrusters.push(Thruster::new(
        Vec3::new(0.0, 0.0, -size),
        Vec3::new(0.0, -1.0, 0.0),
        rot_thrust, rcs_isp,
    ));
    // 偏航左：在 z=+size 处产生 -X 力。
    vessel.thrusters.push(Thruster::new(
        Vec3::new(0.0, 0.0, size),
        Vec3::new(-1.0, 0.0, 0.0),
        rot_thrust, rcs_isp,
    ));
    // 偏航右：在 z=-size 处产生 +X 力。
    vessel.thrusters.push(Thruster::new(
        Vec3::new(0.0, 0.0, -size),
        Vec3::new(1.0, 0.0, 0.0),
        rot_thrust, rcs_isp,
    ));
    // 滚转左：在 x=+size 处产生 -Y 力。
    vessel.thrusters.push(Thruster::new(
        Vec3::new(size, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        rot_thrust, rcs_isp,
    ));
    // 滚转右：在 x=-size 处产生 +Y 力。
    vessel.thrusters.push(Thruster::new(
        Vec3::new(-size, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        rot_thrust, rcs_isp,
    ));

    // ── 平移组（6 推进器，过质心 → 无力矩）──
    vessel.thrusters.push(Thruster::new(
        Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), lin_thrust, rcs_isp,
    ));
    vessel.thrusters.push(Thruster::new(
        Vec3::ZERO, Vec3::new(-1.0, 0.0, 0.0), lin_thrust, rcs_isp,
    ));
    vessel.thrusters.push(Thruster::new(
        Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), lin_thrust, rcs_isp,
    ));
    vessel.thrusters.push(Thruster::new(
        Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), lin_thrust, rcs_isp,
    ));
    vessel.thrusters.push(Thruster::new(
        Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0), lin_thrust, rcs_isp,
    ));
    vessel.thrusters.push(Thruster::new(
        Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), lin_thrust, rcs_isp,
    ));

    // ── 注册推进器组 ──
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttPitchUp, vec![base_idx + 0], rot_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttPitchDown, vec![base_idx + 1], rot_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttYawLeft, vec![base_idx + 2], rot_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttYawRight, vec![base_idx + 3], rot_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttBankLeft, vec![base_idx + 4], rot_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttBankRight, vec![base_idx + 5], rot_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttRight, vec![base_idx + 6], lin_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttLeft, vec![base_idx + 7], lin_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttUp, vec![base_idx + 8], lin_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttDown, vec![base_idx + 9], lin_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttForward, vec![base_idx + 10], lin_thrust,
    ));
    vessel.thruster_groups.push(ThrusterGroup::new(
        ThrusterGroupType::AttBack, vec![base_idx + 11], lin_thrust,
    ));
}

/// 设置指定推进器组内所有推进器的油门。
///
/// 对应 Orbiter `SetThrusterGroupLevel`（`Vessel.h:920`）。
/// 油门值被限幅到 [0, 1]。
pub fn set_group_level(vessel: &mut Vessel, group_type: ThrusterGroupType, level: f64) {
    let level = level.clamp(0.0, 1.0);
    if let Some(group) = vessel.thruster_groups.iter().find(|g| g.group_type == group_type) {
        for &idx in &group.thruster_indices {
            if let Some(thruster) = vessel.thrusters.get_mut(idx) {
                thruster.level = level;
            }
        }
    }
}

/// 获取指定推进器组的当前油门。
///
/// 返回组内第一个推进器的油门值（组内应一致）。
pub fn get_group_level(vessel: &Vessel, group_type: ThrusterGroupType) -> f64 {
    if let Some(group) = vessel.thruster_groups.iter().find(|g| g.group_type == group_type) {
        if let Some(&idx) = group.thruster_indices.first() {
            if let Some(thruster) = vessel.thrusters.get(idx) {
                return thruster.level;
            }
        }
    }
    0.0
}

/// 姿态旋转轴。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotAxis {
    Pitch,
    Yaw,
    Bank,
}

/// 平移轴。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinAxis {
    X,
    Y,
    Z,
}

/// 设置绕指定轴的姿态旋转率（对应 Orbiter `SetAttitudeRotX/Y/Z`）。
///
/// 正 `level` 激活正向组（如 PitchUp），负 `level` 激活反向组（如 PitchDown）。
/// 对立组被设为 0。
pub fn set_attitude_rot(vessel: &mut Vessel, axis: RotAxis, level: f64) {
    let (pos_group, neg_group) = match axis {
        RotAxis::Pitch => (ThrusterGroupType::AttPitchUp, ThrusterGroupType::AttPitchDown),
        RotAxis::Yaw => (ThrusterGroupType::AttYawRight, ThrusterGroupType::AttYawLeft),
        RotAxis::Bank => (ThrusterGroupType::AttBankRight, ThrusterGroupType::AttBankLeft),
    };
    if level >= 0.0 {
        set_group_level(vessel, pos_group, level);
        set_group_level(vessel, neg_group, 0.0);
    } else {
        set_group_level(vessel, pos_group, 0.0);
        set_group_level(vessel, neg_group, -level);
    }
}

/// 设置沿指定轴的平移推力（对应 Orbiter `SetAttitudeLinX/Y/Z`）。
pub fn set_attitude_lin(vessel: &mut Vessel, axis: LinAxis, level: f64) {
    let (pos_group, neg_group) = match axis {
        LinAxis::X => (ThrusterGroupType::AttRight, ThrusterGroupType::AttLeft),
        LinAxis::Y => (ThrusterGroupType::AttUp, ThrusterGroupType::AttDown),
        LinAxis::Z => (ThrusterGroupType::AttForward, ThrusterGroupType::AttBack),
    };
    if level >= 0.0 {
        set_group_level(vessel, pos_group, level);
        set_group_level(vessel, neg_group, 0.0);
    } else {
        set_group_level(vessel, pos_group, 0.0);
        set_group_level(vessel, neg_group, -level);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stage::StageSpec;
    use orbitx_math::{cross, Matrix3, Quat, StateVectors};

    fn make_vessel_with_rcs() -> Vessel {
        let mut v = Vessel::from_spec(
            0,
            &StageSpec {
                name: "test",
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
            },
            StateVectors {
                pos: Vec3::new(0.0, 0.0, 6_371_000.0),
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: Matrix3::IDENTITY,
                q: Quat::IDENTITY,
            },
        );
        add_default_rcs(&mut v, 5.0, 10_000.0);
        v
    }

    #[test]
    fn default_rcs_layout_12_thrusters() {
        let v = make_vessel_with_rcs();
        assert_eq!(v.thrusters.len(), 12, "should have 12 RCS thrusters");
        assert_eq!(v.thruster_groups.len(), 12, "should have 12 thruster groups");
    }

    #[test]
    fn rcs_pitch_up_produces_torque() {
        let v = make_vessel_with_rcs();
        let group = v.thruster_groups.iter()
            .find(|g| g.group_type == ThrusterGroupType::AttPitchUp).unwrap();
        let idx = group.thruster_indices[0];
        let t = &v.thrusters[idx];
        let f = t.base_dir * t.max_thrust;
        let tau = cross(f, t.pos);
        assert!(tau.x > 0.0, "pitch up should produce +X torque: {:?}", tau);
    }

    #[test]
    fn rcs_yaw_produces_torque() {
        let v = make_vessel_with_rcs();
        let group = v.thruster_groups.iter()
            .find(|g| g.group_type == ThrusterGroupType::AttYawLeft).unwrap();
        let idx = group.thruster_indices[0];
        let t = &v.thrusters[idx];
        let f = t.base_dir * t.max_thrust;
        let tau = cross(f, t.pos);
        assert!(tau.length() > 1e-3, "yaw thruster should produce torque: {:?}", tau);
    }

    #[test]
    fn rcs_translation_produces_no_torque() {
        let v = make_vessel_with_rcs();
        for gt in [
            ThrusterGroupType::AttRight,
            ThrusterGroupType::AttUp,
            ThrusterGroupType::AttForward,
        ] {
            let group = v.thruster_groups.iter().find(|g| g.group_type == gt).unwrap();
            let idx = group.thruster_indices[0];
            let t = &v.thrusters[idx];
            let f = t.base_dir * t.max_thrust;
            let tau = cross(f, t.pos);
            assert!(tau.length() < 1e-9, "translation group {:?} should not produce torque: {:?}", gt, tau);
        }
    }

    #[test]
    fn group_level_clamps_0_to_1() {
        let mut v = make_vessel_with_rcs();
        set_group_level(&mut v, ThrusterGroupType::AttPitchUp, 2.0);
        let level = get_group_level(&v, ThrusterGroupType::AttPitchUp);
        assert!(level <= 1.0, "level should clamp to 1.0: {level}");
        set_group_level(&mut v, ThrusterGroupType::AttPitchUp, -1.0);
        let level = get_group_level(&v, ThrusterGroupType::AttPitchUp);
        assert!(level >= 0.0, "level should clamp to 0.0: {level}");
    }

    #[test]
    fn set_attitude_rot_pitch() {
        let mut v = make_vessel_with_rcs();
        set_attitude_rot(&mut v, RotAxis::Pitch, 0.5);
        assert!((get_group_level(&v, ThrusterGroupType::AttPitchUp) - 0.5).abs() < 1e-10);
        assert!(get_group_level(&v, ThrusterGroupType::AttPitchDown).abs() < 1e-10);
        set_attitude_rot(&mut v, RotAxis::Pitch, -0.3);
        assert!(get_group_level(&v, ThrusterGroupType::AttPitchUp).abs() < 1e-10);
        assert!((get_group_level(&v, ThrusterGroupType::AttPitchDown) - 0.3).abs() < 1e-10);
    }

    #[test]
    fn set_attitude_lin_y() {
        let mut v = make_vessel_with_rcs();
        set_attitude_lin(&mut v, LinAxis::Y, 0.8);
        assert!((get_group_level(&v, ThrusterGroupType::AttUp) - 0.8).abs() < 1e-10);
        assert!(get_group_level(&v, ThrusterGroupType::AttDown).abs() < 1e-10);
    }
}
