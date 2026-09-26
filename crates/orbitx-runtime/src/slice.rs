//! 对外切片（进程内）；Comms 侧编为 protobuf。

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Slice {
    pub sim_t: u64,
    pub step_index: u64,
    pub paused: bool,
    pub warp: f64,
    pub rocket_name: String,
    pub active_name: String,
    pub launched: bool,
    pub crash_msg: String,
    pub pitch_target: f64,
    pub yaw_target: f64,
    pub roll_target: f64,
    pub throttle_cmd: f64,
    pub gravity_turn: bool,
    pub focus: FocusTelem,
    pub detached: Vec<FocusTelem>,
    pub launchpad: LaunchpadTelem,
    pub orbit: OrbitTelem,
    pub stages: Vec<StageTelem>,
    pub attitude: AttitudeTelem,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FocusTelem {
    pub is_primary: bool,
    pub display_name: String,
    pub vessel_index: u32,
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub altitude: f64,
    pub speed: f64,
    pub v_vert: f64,
    pub v_horiz: f64,
    pub mass: f64,
    pub fuel: f64,
    pub fuel_pct: f64,
    pub thrust: f64,
    pub twr: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub tip: f64,
    pub env: EnvTelem,
    pub omega: [f64; 3],
    pub gimbal_pitch: f64,
    pub gimbal_yaw: f64,
    pub throttle: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EnvTelem {
    pub a_grav: f64,
    pub g_multiple: f64,
    pub mach: f64,
    pub density: f64,
    pub dynamic_pressure: f64,
    pub pressure: f64,
    pub thrust_atm_scale: f64,
    pub isp_eff: f64,
    pub drag_force: f64,
    pub cd_eff: f64,
    pub temperature: f64,
    pub sound_speed: f64,
    pub load_factor: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LaunchpadTelem {
    pub lat_deg: f64,
    pub lng_deg: f64,
    pub alt_m: f64,
    pub launched: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OrbitTelem {
    pub escaping: bool,
    pub suborbital: bool,
    pub ap_alt: f64,
    pub pe_alt: f64,
    pub period_s: f64,
    pub energy_mj_kg: f64,
    /// 0=亚轨道提示；1=Kepler 要素；2=逃逸（旧 cli Orbit 面板分支）。
    pub hud_mode: u32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StageTelem {
    pub name: String,
    pub fuel_pct: f64,
    pub active: bool,
    pub detached: bool,
    pub crashed: bool,
    pub firing: bool,
    pub empty_fuel: bool,
    pub strap_on: bool,
    /// 剩余燃料质量 [kg]。
    pub fuel: f64,
    /// `Assembly.vessels` 下标。
    pub vessel_index: u32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AttitudeTelem {
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub pitch_target: f64,
    pub yaw_target: f64,
    pub roll_target: f64,
    pub omega: [f64; 3],
    pub throttle: f64,
    pub gimbal_pitch: f64,
    pub gimbal_yaw: f64,
}

fn focus_to_proto(f: &FocusTelem) -> orbitx_protocol::FocusTelem {
    orbitx_protocol::FocusTelem {
        is_primary: f.is_primary,
        display_name: f.display_name.clone(),
        vessel_index: f.vessel_index,
        pos_x: f.pos[0],
        pos_y: f.pos[1],
        pos_z: f.pos[2],
        vel_x: f.vel[0],
        vel_y: f.vel[1],
        vel_z: f.vel[2],
        altitude: f.altitude,
        speed: f.speed,
        v_vert: f.v_vert,
        v_horiz: f.v_horiz,
        mass: f.mass,
        fuel: f.fuel,
        fuel_pct: f.fuel_pct,
        thrust: f.thrust,
        twr: f.twr,
        pitch: f.pitch,
        yaw: f.yaw,
        roll: f.roll,
        tip: f.tip,
        env: Some(orbitx_protocol::EnvTelem {
            a_grav: f.env.a_grav,
            g_multiple: f.env.g_multiple,
            mach: f.env.mach,
            density: f.env.density,
            dynamic_pressure: f.env.dynamic_pressure,
            pressure: f.env.pressure,
            thrust_atm_scale: f.env.thrust_atm_scale,
            isp_eff: f.env.isp_eff,
            drag_force: f.env.drag_force,
            cd_eff: f.env.cd_eff,
            temperature: f.env.temperature,
            sound_speed: f.env.sound_speed,
            load_factor: f.env.load_factor,
        }),
        omega_x: f.omega[0],
        omega_y: f.omega[1],
        omega_z: f.omega[2],
        gimbal_pitch: f.gimbal_pitch,
        gimbal_yaw: f.gimbal_yaw,
        throttle: f.throttle,
    }
}

impl Slice {
    pub fn to_proto(&self) -> orbitx_protocol::Slice {
        orbitx_protocol::Slice {
            sim_t: self.sim_t,
            step_index: self.step_index,
            paused: self.paused,
            warp: self.warp,
            rocket_name: self.rocket_name.clone(),
            active_name: self.active_name.clone(),
            launched: self.launched,
            crash_msg: self.crash_msg.clone(),
            pitch_target: self.pitch_target,
            yaw_target: self.yaw_target,
            roll_target: self.roll_target,
            throttle_cmd: self.throttle_cmd,
            gravity_turn: self.gravity_turn,
            focus: Some(focus_to_proto(&self.focus)),
            detached: self.detached.iter().map(focus_to_proto).collect(),
            launchpad: Some(orbitx_protocol::LaunchpadTelem {
                lat_deg: self.launchpad.lat_deg,
                lng_deg: self.launchpad.lng_deg,
                alt_m: self.launchpad.alt_m,
                launched: self.launchpad.launched,
            }),
            orbit: Some(orbitx_protocol::OrbitTelem {
                escaping: self.orbit.escaping,
                suborbital: self.orbit.suborbital,
                ap_alt: self.orbit.ap_alt,
                pe_alt: self.orbit.pe_alt,
                period_s: self.orbit.period_s,
                energy_mj_kg: self.orbit.energy_mj_kg,
                hud_mode: self.orbit.hud_mode,
            }),
            stages: self
                .stages
                .iter()
                .map(|s| orbitx_protocol::StageTelem {
                    name: s.name.clone(),
                    fuel_pct: s.fuel_pct,
                    active: s.active,
                    detached: s.detached,
                    crashed: s.crashed,
                    firing: s.firing,
                    empty_fuel: s.empty_fuel,
                    strap_on: s.strap_on,
                    fuel: s.fuel,
                    vessel_index: s.vessel_index,
                })
                .collect(),
            attitude: Some(orbitx_protocol::AttitudeTelem {
                pitch: self.attitude.pitch,
                yaw: self.attitude.yaw,
                roll: self.attitude.roll,
                pitch_target: self.attitude.pitch_target,
                yaw_target: self.attitude.yaw_target,
                roll_target: self.attitude.roll_target,
                omega_x: self.attitude.omega[0],
                omega_y: self.attitude.omega[1],
                omega_z: self.attitude.omega[2],
                throttle: self.attitude.throttle,
                gimbal_pitch: self.attitude.gimbal_pitch,
                gimbal_yaw: self.attitude.gimbal_yaw,
            }),
        }
    }
}
