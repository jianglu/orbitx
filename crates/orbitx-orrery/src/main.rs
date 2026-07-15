//! orbitx 太阳系仪：kiss3d 太阳系渲染。
//!
//! 用法：cargo run -p orbitx-orrery
//!
//! 操作：
//! - 鼠标拖拽：旋转视角
//! - 滚轮：缩放
//! - +/-：加速/减速时间
//! - Space：暂停/继续
//! - Escape：退出

mod scene;

use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::prelude::*;

use orbitx_scene::{CameraFrame, Simulation};
use scene::SpaceScene;

/// 1 AU 在渲染中对应多少个单位。
const AU_RENDER_UNITS: f64 = 8.0;

#[kiss3d::main]
async fn main() {
    eprintln!("orbitx 太阳系仪启动中...");
    eprintln!("加载历表数据（从 Orbiter 源码目录）...");

    let mut sim = Simulation::new();
    let frame = CameraFrame::new(AU_RENDER_UNITS);

    let mut window = Window::new("orbitx 太阳系仪").await;
    window.set_background_color(Color::new(0.0, 0.0, 0.0, 1.0));
    window.set_ambient(0.8);
    window.rebind_close_key(Some(Key::Escape));

    let eye = Vec3::new(
        0.0,
        AU_RENDER_UNITS as f32 * 3.0,
        AU_RENDER_UNITS as f32 * 8.0,
    );
    let mut camera = OrbitCamera3d::new(eye, Vec3::ZERO);

    eprintln!("创建场景...");
    let mut space_scene = SpaceScene::new(&sim, &frame);

    let mut paused = false;
    let mut last_instant = std::time::Instant::now();

    eprintln!("渲染循环开始。操作：拖拽旋转，滚轮缩放，+/- 调速，Space 暂停，Esc 退出。");

    while window.render_3d(&mut space_scene.scene, &mut camera).await {
        let now = std::time::Instant::now();
        let dt = now.duration_since(last_instant).as_secs_f64();
        last_instant = now;

        for mut event in window.events().iter() {
            if let WindowEvent::Key(key, Action::Press, _) = event.value {
                match key {
                    Key::Space => {
                        paused = !paused;
                        eprintln!("暂停: {paused}");
                        event.inhibited = true;
                    }
                    Key::Add | Key::Equals => {
                        sim.time_scale *= 2.0;
                        eprintln!("时间加速: {} 天/秒", sim.time_scale / 86400.0);
                        event.inhibited = true;
                    }
                    Key::Minus => {
                        sim.time_scale /= 2.0;
                        eprintln!("时间减速: {} 天/秒", sim.time_scale / 86400.0);
                        event.inhibited = true;
                    }
                    _ => {}
                }
            }
        }

        if !paused {
            sim.step(dt.min(0.1));
        }

        let states = sim.body_states();
        space_scene.update(&mut window, &states, &frame);
    }

    eprintln!("退出。");
}
