use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let shim_dir = manifest_dir.join("cpp");

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .warnings(false)
        .include(&shim_dir)
        .file(shim_dir.join("shim.cpp"))
        .compile("orbitx_dyn_oracle");

    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("shim.cpp").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("oracle.h").display()
    );
}
