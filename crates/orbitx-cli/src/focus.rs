//! 观察焦点：主组合体 + 各独立分离体之间的 UI 切换。
//!
//! 仅影响显示与控制门控；物理步进不读本模块。

use orbitx_vessel::Assembly;

/// UI / 控制门控所指向的观察主体。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewSubject {
    /// 主组合体（含 `active` 的连通分量）。
    Primary,
    /// 已分离（或不在主 `components` 中）的单船。
    Detached(usize),
}

/// 当前观察焦点。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ViewFocus {
    pub subject: ViewSubject,
}

impl Default for ViewFocus {
    fn default() -> Self {
        Self {
            subject: ViewSubject::Primary,
        }
    }
}

impl ViewFocus {
    pub fn primary() -> Self {
        Self::default()
    }

    pub fn is_primary(self) -> bool {
        matches!(self.subject, ViewSubject::Primary)
    }

    /// 是否允许飞行控制键（仅主组合体焦点）。
    pub fn controls_enabled(self) -> bool {
        self.is_primary()
    }

    /// 可观察列表：Primary 为首，其后分离体按下标升序。
    pub fn subjects(asm: &Assembly) -> Vec<ViewSubject> {
        let primary: std::collections::HashSet<usize> =
            asm.components.iter().map(|c| c.vessel_index).collect();
        let mut list = vec![ViewSubject::Primary];
        for (i, v) in asm.vessels.iter().enumerate() {
            if v.detached || !primary.contains(&i) {
                list.push(ViewSubject::Detached(i));
            }
        }
        list
    }

    /// 循环到下一个可观察主体。
    pub fn cycle(&mut self, asm: &Assembly) {
        let list = Self::subjects(asm);
        if list.is_empty() {
            self.subject = ViewSubject::Primary;
            return;
        }
        let cur = list
            .iter()
            .position(|&s| s == self.subject)
            .unwrap_or(0);
        self.subject = list[(cur + 1) % list.len()];
    }

    /// 分离/重置后校正非法 `Detached` 下标。
    pub fn clamp(&mut self, asm: &Assembly) {
        let list = Self::subjects(asm);
        if !list.contains(&self.subject) {
            self.subject = ViewSubject::Primary;
        }
    }

    /// 标题 / 级表用短名。
    pub fn display_name(self, asm: &Assembly) -> String {
        match self.subject {
            ViewSubject::Primary => {
                format!("主组合体 ({})", asm.active_name())
            }
            ViewSubject::Detached(i) => asm
                .vessels
                .get(i)
                .map(|v| v.name.clone())
                .unwrap_or_else(|| format!("#{i}")),
        }
    }

    /// 姿态 / 状态读取用的 vessel 下标（Primary → `active`）。
    pub fn vessel_index(self, asm: &Assembly) -> usize {
        match self.subject {
            ViewSubject::Primary => asm.active,
            ViewSubject::Detached(i) => i.min(asm.vessels.len().saturating_sub(1)),
        }
    }
}

/// 按键是否属于飞行控制（需 Primary 焦点）。元键不在此列。
pub fn is_flight_control_key(code: &str) -> bool {
    matches!(
        code,
        "w" | "W" | "s" | "S" | "g" | "G" | "Up" | "Down" | "Left" | "Right"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::StateVectors;
    use orbitx_vessel::{presets, StageSpec};

    #[test]
    fn subjects_primary_only_when_intact() {
        let asm = Assembly::new(&presets::falcon9(), StateVectors::default());
        let list = ViewFocus::subjects(&asm);
        assert_eq!(list, vec![ViewSubject::Primary]);
    }

    #[test]
    fn cycle_after_separate() {
        let mut asm = Assembly::new(&presets::falcon9(), StateVectors::default());
        asm.separate_stage();
        let mut focus = ViewFocus::primary();
        assert!(focus.is_primary());
        focus.cycle(&asm);
        assert_eq!(focus.subject, ViewSubject::Detached(0));
        focus.cycle(&asm);
        assert!(focus.is_primary());
    }

    #[test]
    fn clamp_invalid_detached() {
        let asm = Assembly::new(&presets::falcon9(), StateVectors::default());
        let mut focus = ViewFocus {
            subject: ViewSubject::Detached(99),
        };
        focus.clamp(&asm);
        assert!(focus.is_primary());
    }

    #[test]
    fn controls_enabled_only_primary() {
        assert!(ViewFocus::primary().controls_enabled());
        let f = ViewFocus {
            subject: ViewSubject::Detached(0),
        };
        assert!(!f.controls_enabled());
    }

    #[test]
    fn two_detached_order_by_index() {
        let stages = vec![
            StageSpec::with_single_thruster(
                "A",
                100.0,
                0.0,
                0.0,
                300.0,
                orbitx_math::Vec3::ZERO,
                orbitx_math::Vec3::new(0.0, 1.0, 0.0),
                5.0,
                1.0,
                1.0,
            ),
            StageSpec::with_single_thruster(
                "B",
                100.0,
                0.0,
                0.0,
                300.0,
                orbitx_math::Vec3::ZERO,
                orbitx_math::Vec3::new(0.0, 1.0, 0.0),
                5.0,
                1.0,
                1.0,
            ),
            StageSpec::with_single_thruster(
                "C",
                50.0,
                0.0,
                0.0,
                300.0,
                orbitx_math::Vec3::ZERO,
                orbitx_math::Vec3::new(0.0, 1.0, 0.0),
                3.0,
                0.5,
                0.0,
            ),
        ];
        let mut asm = Assembly::new(&stages, StateVectors::default());
        asm.separate_stage();
        asm.separate_stage();
        let list = ViewFocus::subjects(&asm);
        assert_eq!(
            list,
            vec![
                ViewSubject::Primary,
                ViewSubject::Detached(0),
                ViewSubject::Detached(1),
            ]
        );
    }
}
