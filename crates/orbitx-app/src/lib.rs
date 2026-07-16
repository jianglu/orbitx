//! orbitx main app - winit window + wgpu rendering + egui HUD + simulation loop.
//!
//! Integrates orbitx-render (rendering abstraction), orbitx-gfx-hud (HUD/MFD),
//! orbitx-dynamics (physics), orbitx-config (configuration).

mod app;
mod ephem_bridge;
mod input;
mod scene_renderer;
mod sphere;

pub use app::App;

/// Entry function (blocking main loop).
pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let event_loop = winit::event_loop::EventLoop::builder().build()?;
    let mut app = App::new();
    event_loop.run_app(&mut app)?;
    Ok(())
}
