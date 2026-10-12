//! 图条件 `terms` 与线性 `transition` 共用的谓词测量 / 比较。
//!
//! 遥测量只读上一步步进切片。条件计时是节点自己的累计时间，不在切片上。

use orbitx_protocol::mem::Slice;
use serde::{Deserialize, Serialize};

/// 比较符（TOML 小写）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PredOp {
    Gt,
    Lt,
    Gte,
    Lte,
    Eq,
}

/// 本步可求值的被测量闭集。
pub const KNOWN_KINDS: &[&str] = &[
    "altitude",
    "speed",
    "v_vert",
    "v_horiz",
    "apoapsis",
    "periapsis",
    "fuel_pct",
    "stage_fuel_pct",
    "time",
    "dynamic_pressure",
    "static_pressure",
    "density",
    "mach",
    "twr",
    "tip_angle",
    "pitch",
    "yaw",
    "roll",
    "aoa",
    "pitch_rate",
    "yaw_rate",
    "roll_rate",
    "tvc_pitch",
    "tvc_yaw",
    "throttle",
];

/// 单条谓词（图 `terms[]` 元素；线性 `transition` 同形）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PredTerm {
    pub kind: String,
    pub op: PredOp,
    pub value: f64,
}

impl PredTerm {
    /// 校验 kind / op（op 由 serde 保证合法）。
    pub fn validate(&self, ctx: &str) -> Result<(), String> {
        if !KNOWN_KINDS.contains(&self.kind.as_str()) {
            return Err(format!("{ctx}: 未知谓词 kind `{}`", self.kind));
        }
        Ok(())
    }
}

/// 取被测量。切片未就绪时遥测为 `None`（条件为假）；`time` 始终用节点累计。
/// `stage` 仅用于 `stage_fuel_pct`：所属主节点的级名，空则这条为假。
pub fn measure(kind: &str, slice: &Slice, time: f64, stage: &str) -> Option<f64> {
    if kind == "time" {
        return Some(time);
    }
    if !slice.ready {
        return None;
    }
    let f = &slice.focus;
    match kind {
        "altitude" => Some(f.altitude),
        "speed" => Some(f.speed),
        "v_vert" => Some(f.v_vert),
        "v_horiz" => Some(f.v_horiz),
        "fuel_pct" => Some(f.fuel_pct),
        "twr" => Some(f.twr),
        "mach" => Some(f.env.mach),
        "density" => Some(f.env.density),
        "dynamic_pressure" => Some(f.env.dynamic_pressure),
        "static_pressure" => Some(f.env.pressure),
        "pitch" => Some(f.pitch),
        "yaw" => Some(f.yaw),
        "roll" => Some(f.roll),
        "tip_angle" => Some(f.tip),
        "aoa" => Some(f.aoa),
        "throttle" => Some(f.throttle),
        "pitch_rate" => Some(f.omega[0]),
        "yaw_rate" => Some(f.omega[2]),
        "roll_rate" => Some(f.omega[1]),
        "tvc_pitch" => Some(f.gimbal_pitch),
        "tvc_yaw" => Some(f.gimbal_yaw),
        "stage_fuel_pct" => {
            if stage.is_empty() {
                return None;
            }
            slice
                .stages
                .iter()
                .find(|s| s.name == stage)
                .map(|s| s.fuel_pct)
        }
        "apoapsis" => Some(slice.orbit.ap_alt),
        "periapsis" => Some(slice.orbit.pe_alt),
        _ => None,
    }
}

/// 比较；`eq` 用相对容差。
pub fn compare(op: PredOp, measured: f64, value: f64) -> bool {
    match op {
        PredOp::Gt => measured > value,
        PredOp::Lt => measured < value,
        PredOp::Gte => measured >= value,
        PredOp::Lte => measured <= value,
        PredOp::Eq => (measured - value).abs() <= 1e-6 * value.abs().max(1.0),
    }
}

/// 单条 term 是否满足。
pub fn eval_term(term: &PredTerm, slice: &Slice, time: f64, stage: &str) -> bool {
    match measure(&term.kind, slice, time, stage) {
        Some(m) => compare(term.op, m, term.value),
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_protocol::mem::{FocusTelem, OrbitTelem, Slice, StageTelem};

    #[test]
    fn unread_slice_rejects_telem_but_keeps_time() {
        let slice = Slice::default();
        assert!(measure("altitude", &slice, 3.0, "").is_none());
        assert_eq!(measure("time", &slice, 3.0, ""), Some(3.0));
    }

    #[test]
    fn reads_slice_fields_without_recompute() {
        let slice = Slice {
            ready: true,
            focus: FocusTelem {
                altitude: 1_200.0,
                speed: 80.0,
                v_vert: 10.0,
                v_horiz: 70.0,
                fuel_pct: 40.0,
                twr: 1.4,
                pitch: 0.2,
                yaw: -0.1,
                roll: 0.05,
                tip: 0.22,
                aoa: 0.03,
                throttle: 0.8,
                omega: [0.01, 0.02, 0.03],
                gimbal_pitch: 0.04,
                gimbal_yaw: -0.05,
                env: orbitx_protocol::mem::EnvTelem {
                    mach: 0.9,
                    density: 0.4,
                    dynamic_pressure: 12_000.0,
                    pressure: 50_000.0,
                    ..Default::default()
                },
                ..Default::default()
            },
            orbit: OrbitTelem {
                ap_alt: 200_000.0,
                pe_alt: -10_000.0,
                ..Default::default()
            },
            stages: vec![
                StageTelem {
                    name: "idle".into(),
                    fuel_pct: 90.0,
                    active: false,
                    ..Default::default()
                },
                StageTelem {
                    name: "core".into(),
                    fuel_pct: 55.0,
                    active: true,
                    ..Default::default()
                },
            ],
            ..Default::default()
        };
        assert_eq!(measure("altitude", &slice, 0.0, ""), Some(1_200.0));
        assert_eq!(measure("speed", &slice, 0.0, ""), Some(80.0));
        assert_eq!(measure("v_vert", &slice, 0.0, ""), Some(10.0));
        assert_eq!(measure("pitch_rate", &slice, 0.0, ""), Some(0.01));
        assert_eq!(measure("yaw_rate", &slice, 0.0, ""), Some(0.03));
        assert_eq!(measure("roll_rate", &slice, 0.0, ""), Some(0.02));
        assert_eq!(measure("tvc_pitch", &slice, 0.0, ""), Some(0.04));
        assert_eq!(measure("static_pressure", &slice, 0.0, ""), Some(50_000.0));
        assert_eq!(measure("stage_fuel_pct", &slice, 0.0, "core"), Some(55.0));
        assert_eq!(measure("stage_fuel_pct", &slice, 0.0, "idle"), Some(90.0));
        assert!(measure("stage_fuel_pct", &slice, 0.0, "").is_none());
        assert!(measure("stage_fuel_pct", &slice, 0.0, "missing").is_none());
        assert_eq!(measure("apoapsis", &slice, 0.0, ""), Some(200_000.0));
        assert_eq!(measure("aoa", &slice, 0.0, ""), Some(0.03));
    }
}
