//! clap 启动参数。

use std::path::{Path, PathBuf};

use clap::{Parser, ValueEnum};
use orbitx_config::{load_rocket_source, resolve_scenario_spec, PlanetaryScenario};
use orbitx_controller::workflow::graph::{self, GraphIr};
use orbitx_controller::workflow::{from_toml_str, WorkFlowDesc, WorkFlowKind};

/// `orbitx-runtime` 进程级基本配置。
#[derive(Debug, Clone, Parser)]
#[command(name = "orbitx-runtime", about = "Orbitx product simulation host")]
pub struct RuntimeArgs {
    /// 火箭类：内置别名或 rocket.toml 路径。
    #[arg(long)]
    pub rocket: String,

    /// 环境：别名 `earth` 或 `scenario_xxx.toml` 路径。
    #[arg(long)]
    pub scenario: String,

    /// 控制模式（与 `--workflow` 互斥）。
    #[arg(long, value_enum)]
    pub control: Option<ControlKindArg>,

    /// 工作流 TOML 路径（与 `--control` 互斥；kind 由文件决定）。
    #[arg(long, value_name = "PATH")]
    pub workflow: Option<PathBuf>,

    /// 步进节奏。
    #[arg(long, value_enum)]
    pub drive: DriveModeArg,

    /// 固定物理步长 [ms]。
    #[arg(long, value_name = "MS")]
    pub sim_dt: u64,

    /// 日志目录。
    #[arg(long)]
    pub log_dir: PathBuf,

    /// 是否启用黑匣子（FlightRecorder L1 入队 + IO stub；真落盘 → P6）。
    /// 兼容 flag 与显式值：`--recorder-enabled` ≡ `--recorder-enabled true`。
    #[arg(
        long = "recorder-enabled",
        action = clap::ArgAction::Set,
        num_args = 0..=1,
        default_value_t = false,
        default_missing_value = "true",
        value_name = "BOOL"
    )]
    pub recorder_enabled: bool,

    /// 黑匣子输出目录（仅 `--recorder-enabled` 时必填）。
    #[arg(long)]
    pub recorder_dir: Option<PathBuf>,

    /// 本机 Zenoh/SHM 会话标识（当前版本仅本机）。
    #[arg(long = "zenoh-endpoint")]
    pub zenoh_endpoint: String,

    /// 历表数据根目录（须含 `Src/Celbody/...`）。
    #[arg(long = "ephemeris-data")]
    pub ephemeris_data: PathBuf,

    /// 可选共用地表包根目录（含 `manifest.toml`）；省略则光滑球触点/坠毁。
    #[arg(long = "surface-package", value_name = "DIR")]
    pub surface_package: Option<PathBuf>,

    /// 地表工作集最大四叉树 level（钳在包 `max_level` 内；默认用包上限）。
    #[arg(long = "surface-working-max-level", default_value_t = u32::MAX)]
    pub surface_working_max_level: u32,

    /// 发射场 `lp_xxx.toml`（必填；runtime 不内置默认发射场，覆盖网格须同时给 `--surface-package`）。
    #[arg(long = "launchpad", value_name = "PATH")]
    pub launchpad: Option<PathBuf>,

    /// 发射场台位名（缺省 = lp 第一个台位）。
    #[arg(long = "pad", value_name = "NAME")]
    pub pad: Option<String>,

    /// 覆盖台位朝向（度；缺省用台位 `heading_deg`）。
    #[arg(long = "heading-deg", value_name = "DEG")]
    pub heading_deg: Option<f64>,

    /// 起始历元覆盖（MJD；覆盖 scenario 的 mjd——任务层选白天发射窗口用）。
    #[arg(long = "mjd", value_name = "MJD")]
    pub mjd: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum DriveModeArg {
    #[value(name = "client-step")]
    ClientStep,
    #[value(name = "self-paced")]
    SelfPaced,
}

/// `--control` 取值（模式 a / b）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ControlKindArg {
    Base,
    Target,
}

/// 解析后的会话控制模式（与 `--control` / `--workflow` 互斥闭集）。
#[derive(Debug, Clone)]
pub enum SessionControl {
    Control(ControlKindArg),
    WorkFlow {
        path: PathBuf,
        desc: WorkFlowDesc,
    },
    /// 图 IR（`[[nodes]]`），与线性 phases/steps 分文件。
    Graph {
        path: PathBuf,
        ir: GraphIr,
    },
}

impl SessionControl {
    pub fn kind_label(&self) -> &'static str {
        match self {
            Self::Control(ControlKindArg::Base) => "control:base",
            Self::Control(ControlKindArg::Target) => "control:target",
            Self::WorkFlow { desc, .. } => match desc.kind {
                WorkFlowKind::Target => "workflow:target",
                WorkFlowKind::Super => "workflow:super",
            },
            Self::Graph { ir, .. } => match ir.kind {
                WorkFlowKind::Target => "workflow:graph:target",
                WorkFlowKind::Super => "workflow:graph:super",
            },
        }
    }
}

/// 已解析的发射场选择（台位与朝向已定版）。
#[derive(Debug, Clone)]
pub struct LoadedLaunchpad {
    pub config: orbitx_config::LaunchpadConfig,
    /// 选定台位名。
    pub pad_name: String,
    /// 生效朝向（`--heading-deg` 覆盖台位值）。
    pub heading_deg: f64,
    /// 经纬度所属天体（按 scenario 天体表解析）。
    pub body_name: String,
    /// 所属天体半径（m，天体表快照）。
    pub body_size_m: f64,
    /// 所属天体恒星自转周期（s；无自转 = 0）。
    pub body_sid_period_s: f64,
    /// 是否为主天体（本轮仅主天体可摆放/飞行）。
    pub is_primary: bool,
}

impl LoadedLaunchpad {
    pub fn pad(&self) -> &orbitx_config::PadConfig {
        self.config
            .pad(Some(&self.pad_name))
            .expect("pad resolved at load")
    }

    /// 台位支持面（火箭底部落点）的地心半径：按发射场坐标系换算，
    /// `R_elev + base_elevation_m + 台架高度(axis_offset_m[2])`，不走地形采样。
    pub fn support_radius_m(&self, elev_radius_m: f64) -> f64 {
        elev_radius_m + self.config.base_elevation_m + self.pad().axis_offset_m[2]
    }
}

/// 启动时已解析的会话配置（骨架阶段供日志 / World；Assembly 接线 → 阶段 B）。
#[derive(Debug, Clone)]
pub struct LoadedSession {
    pub rocket: orbitx_config::RocketConfig,
    pub scenario: PlanetaryScenario,
    pub control: SessionControl,
    /// P5.1：可选地表包会话配置（由 `--surface-package`）。
    pub surface: Option<crate::surface_session::SurfaceSessionConfig>,
    /// 可选发射场（由 `--launchpad`）。
    pub launchpad: Option<LoadedLaunchpad>,
}

impl RuntimeArgs {
    pub fn validate(&self) -> Result<(), String> {
        if self.sim_dt == 0 {
            return Err(format!("--sim-dt must be > 0 ms, got {}", self.sim_dt));
        }
        if self.recorder_enabled && self.recorder_dir.is_none() {
            return Err("--recorder-enabled 时必须指定 --recorder-dir".into());
        }
        validate_zenoh_endpoint(&self.zenoh_endpoint)?;
        let _ = self.load_session()?;
        Ok(())
    }

    /// 解析 `--control` / `--workflow` 互斥规则。
    pub fn resolve_control(&self) -> Result<SessionControl, String> {
        match (&self.control, &self.workflow) {
            (Some(_), Some(_)) => Err("--control 与 --workflow 互斥，请只指定其一".into()),
            (Some(c), None) => Ok(SessionControl::Control(*c)),
            (None, Some(path)) => load_workflow(path),
            (None, None) => Err("必须指定 --control 或 --workflow".into()),
        }
    }

    /// 解析 `--rocket` / `--scenario` / 控制模式。
    pub fn load_session(&self) -> Result<LoadedSession, String> {
        let rocket = load_rocket_source(&self.rocket)?;
        let path = resolve_scenario_spec(&self.scenario)?;
        if !path.exists() {
            return Err(format!("--scenario 文件不存在：{}", path.display()));
        }
        let mut scenario = PlanetaryScenario::from_file(&path)?;
        // P4.5：`--mjd` 历元覆盖（任务层选白天发射窗口等）。
        if let Some(m) = self.mjd {
            scenario.mjd = m;
        }
        if !self.ephemeris_data.is_dir() {
            return Err(format!(
                "--ephemeris-data 不是目录：{}",
                self.ephemeris_data.display()
            ));
        }
        let control = self.resolve_control()?;
        let mut surface = if let Some(ref root) = self.surface_package {
            if !root.is_dir() {
                return Err(format!(
                    "--surface-package 不是目录：{}",
                    root.display()
                ));
            }
            if !root.join("manifest.toml").is_file() {
                return Err(format!(
                    "--surface-package 缺少 manifest.toml：{}",
                    root.display()
                ));
            }
            Some(crate::surface_session::SurfaceSessionConfig {
                body_name: scenario.primary.clone(),
                package_root: root.clone(),
                working_max_level: self.surface_working_max_level,
                ..Default::default()
            })
        } else {
            None
        };
        let launchpad = self.load_launchpad(&scenario, &mut surface)?;
        // 地表包按绑定天体校验（R_elev 对所属天体 size），启动期暴露不符。
        if let Some(ref cfg) = surface {
            let bound = if cfg.body_name.is_empty() {
                scenario.primary.as_str()
            } else {
                cfg.body_name.as_str()
            };
            let body_size = scenario
                .bodies
                .iter()
                .find(|b| b.name == bound)
                .map(|b| b.size)
                .ok_or_else(|| format!("地表包绑定天体 `{bound}` 不在环境天体表中"))?;
            let text = std::fs::read_to_string(cfg.package_root.join("manifest.toml"))
                .map_err(|e| format!("读取地表包 manifest 失败：{e}"))?;
            let manifest = orbitx_surface::Manifest::parse(&text)?;
            if (manifest.r_elev_m - body_size).abs() > 0.5 {
                return Err(format!(
                    "地表包 R_elev_m={} 与绑定天体 `{bound}` size={body_size} 不符（eps 0.5 m）",
                    manifest.r_elev_m
                ));
            }
        }
        Ok(LoadedSession {
            rocket,
            scenario,
            control,
            surface,
            launchpad,
        })
    }

    /// 解析 `--launchpad` / `--pad` / `--heading-deg`，并把覆盖网格挂到地表会话配置。
    fn load_launchpad(
        &self,
        scenario: &PlanetaryScenario,
        surface: &mut Option<crate::surface_session::SurfaceSessionConfig>,
    ) -> Result<Option<LoadedLaunchpad>, String> {
        let Some(path) = &self.launchpad else {
            if self.pad.is_some() || self.heading_deg.is_some() {
                return Err("--pad / --heading-deg 须与 --launchpad 同时指定".into());
            }
            return Err(
                "必须指定 --launchpad <lp_xxx.toml>（runtime 不内置默认发射场；orbitx-cli 未传参时默认海南文昌）"
                    .into(),
            );
        };
        if !path.is_file() {
            return Err(format!("--launchpad 文件不存在：{}", path.display()));
        }
        let config = orbitx_config::LaunchpadConfig::from_file(path)?;
        // 经纬度归属：按天体表解析所属天体（架构多天体、当前实例可为非主天体）。
        let body_cfg = scenario
            .bodies
            .iter()
            .find(|b| b.name == config.body)
            .ok_or_else(|| {
                let names: Vec<&str> = scenario.bodies.iter().map(|b| b.name.as_str()).collect();
                format!(
                    "发射场 body `{}` 不在环境天体表中（可用：{}）",
                    config.body,
                    names.join(", ")
                )
            })?;
        let body_name = body_cfg.name.clone();
        let body_size_m = body_cfg.size;
        let body_sid_period_s = match &body_cfg.rotation {
            Some(r) if r.enabled => r.sid_rot_period,
            _ => 0.0,
        };
        let is_primary = body_name == scenario.primary;
        let pad = config.pad(self.pad.as_deref())?;
        let pad_name = pad.name.clone();
        let heading_deg = self.heading_deg.unwrap_or(pad.heading_deg);
        if let Some(cfg) = surface.as_mut() {
            // 地表包绑定到发射场所属天体（无 lp 时在 load_session 绑主天体）。
            cfg.body_name = body_name.clone();
            // 发射场高度模型：足迹内截平上限 + 填平域 + 各台位点超差告警采样点。
            cfg.launchpad_base_elevation_m = Some(config.base_elevation_m);
            cfg.pad_area = config.pad_area.as_ref().map(|pa| {
                crate::surface_session::PadAreaSpec {
                    origin_lat_deg: config.latitude_deg,
                    origin_lon_deg: config.longitude_deg,
                    center_enu_m: pa.center_enu_m,
                    radius_m: pa.radius_m,
                    blend_m: pa.blend_m,
                }
            });
            cfg.launchpad_pad_points = config
                .pads
                .iter()
                .map(|p| pad_point_lat_lon(&config, p, body_size_m))
                .collect();
        }
        if let Some(data) = config.load_grid_data()? {
            let Some(cfg) = surface.as_mut() else {
                return Err(
                    "发射场带 elevation_override 时须同时指定 --surface-package（覆盖与全球包合成）"
                        .into(),
                );
            };
            cfg.elevation_override = Some(crate::surface_session::ElevationOverrideSpec {
                origin_lat_deg: config.latitude_deg,
                origin_lon_deg: config.longitude_deg,
                cell_m: config.grid_cell_m.unwrap_or(0.0),
                width: config.grid_width.unwrap_or(0),
                height: config.grid_height.unwrap_or(0),
                data,
            });
        }
        Ok(Some(LoadedLaunchpad {
            config,
            pad_name,
            heading_deg,
            body_name,
            body_size_m,
            body_sid_period_s,
            is_primary,
        }))
    }
}

/// 台位轴基点（场原点 ENU 偏移，东/北分量）→ 经纬度（切平面近似，供加载期告警采样）。
fn pad_point_lat_lon(
    config: &orbitx_config::LaunchpadConfig,
    pad: &orbitx_config::PadConfig,
    radius_m: f64,
) -> (f64, f64) {
    let (east, north, up) =
        orbitx_surface::enu_basis(config.latitude_deg, config.longitude_deg);
    let p = [
        up[0] * radius_m + east[0] * pad.axis_offset_m[0] + north[0] * pad.axis_offset_m[1],
        up[1] * radius_m + east[1] * pad.axis_offset_m[0] + north[1] * pad.axis_offset_m[1],
        up[2] * radius_m + east[2] * pad.axis_offset_m[0] + north[2] * pad.axis_offset_m[1],
    ];
    let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt().max(1e-9);
    let lat = (p[1] / r).clamp(-1.0, 1.0).asin().to_degrees();
    let lon = p[2].atan2(p[0]).to_degrees();
    (lat, lon)
}

fn load_workflow(path: &Path) -> Result<SessionControl, String> {
    if !path.exists() {
        return Err(format!("--workflow 文件不存在：{}", path.display()));
    }
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("读取工作流 `{}` 失败：{e}", path.display()))?;
    if text.contains("[[nodes]]") {
        let ir = graph::from_toml_str(&text, None)
            .map_err(|e| format!("解析图工作流 `{}` 失败：{e}", path.display()))?;
        return Ok(SessionControl::Graph {
            path: path.to_path_buf(),
            ir,
        });
    }
    let desc =
        from_toml_str(&text).map_err(|e| format!("解析工作流 `{}` 失败：{e}", path.display()))?;
    Ok(SessionControl::WorkFlow {
        path: path.to_path_buf(),
        desc,
    })
}

/// 当前版本：仅本机 SHM。允许 `local` 与 loopback locator；拒绝非本机 tcp/udp。
pub fn validate_zenoh_endpoint(endpoint: &str) -> Result<(), String> {
    let ep = endpoint.trim();
    if ep.is_empty() {
        return Err("--zenoh-endpoint must not be empty".into());
    }
    if ep.eq_ignore_ascii_case("local") {
        return Ok(());
    }

    let lower = ep.to_ascii_lowercase();
    if let Some(rest) = lower.strip_prefix("tcp/") {
        return validate_loopback_host(rest, ep);
    }
    if let Some(rest) = lower.strip_prefix("udp/") {
        return validate_loopback_host(rest, ep);
    }
    if lower.starts_with("unixpipe:")
        || lower.starts_with("unixpipe/")
        || lower.starts_with("unixsock:")
        || lower.starts_with("unixsock/")
        || lower.starts_with("unix:")
        || lower.starts_with("unix/")
    {
        return Ok(());
    }

    Err(format!(
        "unsupported --zenoh-endpoint `{ep}`: current build is local-SHM only (use `local`, loopback, or unix* locator); cross-device is not supported"
    ))
}

fn validate_loopback_host(host_port: &str, original: &str) -> Result<(), String> {
    let host = host_port
        .rsplit_once(':')
        .map(|(h, _)| h)
        .unwrap_or(host_port);
    let host = host.trim_matches(|c| c == '[' || c == ']');
    if host == "127.0.0.1" || host == "::1" || host.eq_ignore_ascii_case("localhost") {
        return Ok(());
    }
    Err(format!(
        "cross-device Zenoh is not supported; refused endpoint `{original}` (host `{host}` is not loopback)"
    ))
}

#[cfg(test)]
mod tests;

/// 测试用：补齐 runtime 必填参数。`earth` 别名 + 捆绑历表。
#[cfg(test)]
pub(crate) fn test_runtime_args(rocket: &str) -> RuntimeArgs {
    RuntimeArgs {
        rocket: rocket.into(),
        scenario: "earth".into(),
        control: Some(ControlKindArg::Target),
        workflow: None,
        drive: DriveModeArg::SelfPaced,
        sim_dt: 20,
        log_dir: PathBuf::from("./logs"),
        recorder_enabled: false,
        recorder_dir: None,
        mjd: None,
        zenoh_endpoint: "local".into(),
        ephemeris_data: orbitx_environment::resolve_ephemeris_data(None),
        surface_package: None,
        surface_working_max_level: u32::MAX,
        launchpad: Some(
            // 单源：SimRocket/sim-rocket/launchpads/lp_default.toml（workspace 布局，
            // 从 orbitx-runtime crate 上溯 3 级；orbitx 独立检出时由调用方显式传）
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../../sim-rocket/launchpads/lp_default.toml"),
        ),
        pad: None,
        heading_deg: None,
    }
}
