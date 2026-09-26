//! 据 Slice 遥测判断是否应自动分离（不碰 Assembly）。

use orbitx_protocol::Slice;

/// 侧挂叶空燃料优先，否则活动级空燃料 → 发 `Separate`。
pub fn should_auto_separate(slice: &Slice) -> bool {
    let attached: usize = slice
        .stages
        .iter()
        .filter(|s| !s.detached && !s.crashed)
        .count();
    if attached <= 1 {
        return false;
    }
    if slice
        .stages
        .iter()
        .any(|s| s.strap_on && !s.detached && !s.crashed && s.empty_fuel)
    {
        return true;
    }
    slice
        .stages
        .iter()
        .any(|s| s.active && !s.detached && !s.crashed && s.empty_fuel)
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_protocol::StageTelem;

    fn stage(name: &str, active: bool, strap_on: bool, empty: bool) -> StageTelem {
        StageTelem {
            name: name.into(),
            active,
            strap_on,
            empty_fuel: empty,
            detached: false,
            crashed: false,
            ..Default::default()
        }
    }

    #[test]
    fn strap_on_empty_triggers() {
        let slice = Slice {
            stages: vec![
                stage("booster", false, true, true),
                stage("core", true, false, false),
            ],
            ..Default::default()
        };
        assert!(should_auto_separate(&slice));
    }

    #[test]
    fn single_stage_never() {
        let slice = Slice {
            stages: vec![stage("core", true, false, true)],
            ..Default::default()
        };
        assert!(!should_auto_separate(&slice));
    }
}
