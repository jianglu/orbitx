//! 一维分段线性查表插值（移植自 vessel `aero::interpolate_cd_mach`）。
//!
//! 通用工具：`table` 须按 x 升序。用于 Cd(M) 查表等。支持单分量 (`f64`) 与
//! 多分量 (`[f64; N]`) 表——同一二分+线性插值核心，避免各处重写。

/// 分段线性插值的分量类型：在分数 `t` 处线性插值。
pub trait Lerp: Copy {
    fn lerp(self, other: Self, t: f64) -> Self;
}

impl Lerp for f64 {
    #[inline]
    fn lerp(self, other: f64, t: f64) -> f64 {
        self + t * (other - self)
    }
}

impl Lerp for [f64; 3] {
    #[inline]
    fn lerp(self, other: [f64; 3], t: f64) -> [f64; 3] {
        [
            self[0] + t * (other[0] - self[0]),
            self[1] + t * (other[1] - self[1]),
            self[2] + t * (other[2] - self[2]),
        ]
    }
}

/// 分段线性插值：在 `table`（按 x 升序）中定位 `x` 所在区间并线性插值。
/// - 表空 → `fallback`
/// - 单点 → 该点 y
/// - `x` 越界 → 端点 y（钳位）
///
/// 泛型 `T: Lerp`：`f64` 用于单分量表（如 Cd(M)），`[f64; 3]` 用于多分量
/// 表（如 (cl, cm, cd)(α)），共享同一二分核心。
pub fn piecewise_linear<T: Lerp>(table: &[(f64, T)], x: f64, fallback: T) -> T {
    if table.is_empty() {
        return fallback;
    }
    if table.len() == 1 {
        return table[0].1;
    }
    let mut lo = 0usize;
    let mut hi = table.len() - 1;
    if x <= table[lo].0 {
        return table[lo].1;
    }
    if x >= table[hi].0 {
        return table[hi].1;
    }
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if table[mid].0 <= x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (x0, y0) = table[lo];
    let (x1, y1) = table[hi];
    let t = (x - x0) / (x1 - x0);
    y0.lerp(y1, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_returns_fallback() {
        assert_eq!(piecewise_linear(&[], 1.0, 7.0), 7.0);
    }

    #[test]
    fn single_point_returns_that_y() {
        assert_eq!(piecewise_linear(&[(5.0, 9.0)], 1.0, 0.0), 9.0);
        assert_eq!(piecewise_linear(&[(5.0, 9.0)], 99.0, 0.0), 9.0);
    }

    #[test]
    fn clamps_at_endpoints() {
        let t = vec![(0.0, 0.3), (1.0, 0.3), (1.2, 1.0), (2.0, 0.5)];
        assert_eq!(piecewise_linear(&t, -1.0, 0.0), 0.3);
        assert_eq!(piecewise_linear(&t, 3.0, 0.0), 0.5);
    }

    #[test]
    fn interpolates_midpoint() {
        let t = vec![(0.0, 0.0), (2.0, 10.0)];
        assert!((piecewise_linear(&t, 1.0, 0.0) - 5.0).abs() < 1e-12);
        assert!((piecewise_linear(&t, 0.5, 0.0) - 2.5).abs() < 1e-12);
        assert!((piecewise_linear(&t, 1.5, 0.0) - 7.5).abs() < 1e-12);
    }

    #[test]
    fn hits_exact_knot() {
        let t = vec![(0.0, 0.3), (1.0, 0.3), (1.2, 1.0), (2.0, 0.5)];
        assert!((piecewise_linear(&t, 1.2, 0.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn multi_y_interpolates() {
        // (aoa, [cl, cm, cd]) — midpoints lerp per-component.
        let t: Vec<(f64, [f64; 3])> = vec![
            (0.0, [0.0, 0.0, 0.1]),
            (1.0, [1.0, 0.5, 0.2]),
        ];
        let r = piecewise_linear(&t, 0.5, [0.0; 3]);
        assert!((r[0] - 0.5).abs() < 1e-12);
        assert!((r[1] - 0.25).abs() < 1e-12);
        assert!((r[2] - 0.15).abs() < 1e-12);
    }

    #[test]
    fn multi_y_empty_fallback() {
        let r = piecewise_linear(&[] as &[(f64, [f64; 3])], 1.0, [7.0; 3]);
        assert_eq!(r, [7.0; 3]);
    }

    #[test]
    fn multi_y_clamps() {
        let t: Vec<(f64, [f64; 3])> = vec![(0.0, [0.0; 3]), (1.0, [1.0; 3])];
        assert_eq!(piecewise_linear(&t, -1.0, [9.0; 3]), [0.0; 3]);
        assert_eq!(piecewise_linear(&t, 2.0, [9.0; 3]), [1.0; 3]);
    }
}
