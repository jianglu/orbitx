//! 共享场景模块：历表加载 + 坐标转换桥接 + 字体加载。
//!
//! 被 `orbitx-orrery`（太阳系仪）、`orbitx-flight`（航天器飞行）、
//! `orbitx-launch`（发射测试器）共用。

pub mod bridge;
pub mod sim;

pub use bridge::{scale_radius, CameraFrame, AU_METERS};
pub use sim::{BodyState, Simulation, MJD2000};

use kiss3d::text::Font;
use std::sync::Arc;

/// 加载支持中文的字体。按优先级搜索系统字体目录，找不到则回退到 kiss3d 默认字体。
///
/// kiss3d 内置的 WorkSans-Regular 不含 CJK 字符，中文会显示为空白。
/// 此函数搜索 macOS/Linux/Windows 上的常见 CJK 字体。
pub fn load_cjk_font() -> Arc<Font> {
    let candidates = [
        // macOS
        "/Library/Fonts/Arial Unicode.ttf",
        "/System/Library/Fonts/STHeiti Medium.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        "/System/Library/Fonts/Hiragino Sans GB.ttc",
        // Linux
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/wqy-zenhei/wqy-zenhei.ttc",
        "/usr/share/fonts/wqy-microhei/wqy-microhei.ttc",
        // Windows
        "C:\\Windows\\Fonts\\msyh.ttc",
        "C:\\Windows\\Fonts\\simsun.ttc",
        "C:\\Windows\\Fonts\\simhei.ttf",
    ];
    for path in &candidates {
        if let Some(font) = Font::new(std::path::Path::new(path)) {
            eprintln!("使用字体: {path}");
            return Arc::new(font);
        }
    }
    eprintln!("未找到 CJK 字体，回退到默认字体（中文可能无法显示）");
    Font::default()
}
