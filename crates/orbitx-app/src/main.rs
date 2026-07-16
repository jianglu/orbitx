//! orbitx 主程序入口。

fn main() {
    orbitx_app::run().unwrap_or_else(|e| eprintln!("Error: {e}"));
}
