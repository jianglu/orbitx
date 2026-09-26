//! clap 启动参数。

use std::path::{Path, PathBuf};

use clap::{Parser, ValueEnum};
use orbitx_config::{load_rocket_source, ScenarioConfig};
use orbitx_controller::workflow::{from_toml_str, WorkFlowDesc, WorkFlowKind};

/// `orbitx-runtime` 进程级基本配置。
#[derive(Debug, Clone, Parser)]
#[command(name = "orbitx-runtime", about = "Orbitx product simulation host")]
pub struct RuntimeArgs {
    /// 火箭类：内置别名（同 cli）或 rocket.toml 路径。
    #[arg(long, default_value = "falcon9")]
    pub rocket: String,

    /// 可选 scenario.toml 路径。
    #[arg(long)]
    pub scenario: Option<PathBuf>,

    /// 控制模式（与 `--workflow` 互斥；缺省为 target）。
    #[arg(long, value_enum, num_args = 0..=1, default_missing_value = "target")]
    pub control: Option<ControlKindArg>,

    /// 工作流 TOML 路径（与 `--control` 互斥；kind 由文件决定）。
    #[arg(long, value_name = "PATH")]
    pub workflow: Option<PathBuf>,

    /// 步进节奏。
    #[arg(long, value_enum, default_value_t = DriveModeArg::SelfPaced)]
    pub drive: DriveModeArg,

    /// 固定物理步长 [ms]。
    #[arg(long, default_value_t = 20, value_name = "MS")]
    pub sim_dt: u64,

    /// 日志目录。
    #[arg(long, default_value = "./logs")]
    pub log_dir: PathBuf,

    /// 黑匣子输出目录。
    #[arg(long, default_value = "./flight_records")]
    pub recorder_dir: PathBuf,

    /// 本机 Zenoh/SHM 会话标识（当前版本仅本机；默认 `local`）。
    #[arg(long = "zenoh-endpoint", default_value = "local")]
    pub zenoh_endpoint: String,

    /// 历表数据根目录（须含 `Src/Celbody/...`；覆盖 `ORBITX_EPHEMERIS_DATA` 与自动探测）。
    #[arg(long = "ephemeris-data")]
    pub ephemeris_data: Option<PathBuf>,
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
        }
    }
}

/// 启动时已解析的会话配置（骨架阶段供日志 / World；Assembly 接线 → 阶段 B）。
#[derive(Debug, Clone)]
pub struct LoadedSession {
    pub rocket: orbitx_config::RocketConfig,
    pub scenario: Option<ScenarioConfig>,
    pub control: SessionControl,
}

impl RuntimeArgs {
    pub fn validate(&self) -> Result<(), String> {
        if self.sim_dt == 0 {
            return Err(format!("--sim-dt must be > 0 ms, got {}", self.sim_dt));
        }
        validate_zenoh_endpoint(&self.zenoh_endpoint)?;
        let _ = self.load_session()?;
        Ok(())
    }

    /// 解析 `--control` / `--workflow` 互斥规则。
    pub fn resolve_control(&self) -> Result<SessionControl, String> {
        match (&self.control, &self.workflow) {
            (Some(_), Some(_)) => Err(
                "--control 与 --workflow 互斥，请只指定其一".into(),
            ),
            (Some(c), None) => Ok(SessionControl::Control(*c)),
            (None, Some(path)) => load_workflow(path),
            (None, None) => Ok(SessionControl::Control(ControlKindArg::Target)),
        }
    }

    /// 解析 `--rocket` / `--scenario` / 控制模式。
    pub fn load_session(&self) -> Result<LoadedSession, String> {
        let rocket = load_rocket_source(&self.rocket)?;
        let scenario = match &self.scenario {
            None => None,
            Some(path) => {
                if !path.exists() {
                    return Err(format!("--scenario 文件不存在：{}", path.display()));
                }
                Some(
                    ScenarioConfig::from_file(path)
                        .map_err(|e| format!("解析场景 `{}` 失败：{e}", path.display()))?,
                )
            }
        };
        let control = self.resolve_control()?;
        Ok(LoadedSession {
            rocket,
            scenario,
            control,
        })
    }
}

fn load_workflow(path: &Path) -> Result<SessionControl, String> {
    if !path.exists() {
        return Err(format!("--workflow 文件不存在：{}", path.display()));
    }
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("读取工作流 `{}` 失败：{e}", path.display()))?;
    let desc = from_toml_str(&text)
        .map_err(|e| format!("解析工作流 `{}` 失败：{e}", path.display()))?;
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
