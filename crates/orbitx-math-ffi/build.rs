use std::fs;
use std::path::PathBuf;

fn main() {
    // Locate the Orbiter source tree relative to this crate.
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let orbitx_root = manifest_dir
        .ancestors()
        .nth(2)
        .expect("crate must be inside a workspace");
    let orbiter_src = orbitx_root
        .parent()
        .map(|p| p.join("orbiter").join("Src").join("Orbiter"))
        .expect("cannot locate sibling 'orbiter' directory");

    if !orbiter_src.exists() {
        panic!(
            "Orbiter source not found at {}. The FFI oracle requires the \
             Orbiter source tree as a sibling of orbitx.",
            orbiter_src.display()
        );
    }

    let shim_dir = manifest_dir.join("cpp");
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());

    // Copy Vecmat.h, Vecmat.cpp, Astro.h, Astro.cpp into OUT_DIR, patching the
    // friend-default-argument that clang rejects (`int *sing = 0` in a non-
    // defining friend declaration — an MSVC/GCC extension). We only need the
    // strip for Vecmat.h since that's where the friend declarations live.
    let strip_friend_default = |s: String| s.replace("int *sing = 0", "int *sing");

    for name in &["Vecmat.h", "Vecmat.cpp", "Astro.h", "Astro.cpp"] {
        let src = orbiter_src.join(name);
        let dst = out_dir.join(name);
        let content = fs::read_to_string(&src)
            .unwrap_or_else(|e| panic!("failed to read {}: {e}", src.display()));
        let content = strip_friend_default(content);
        fs::write(&dst, content)
            .unwrap_or_else(|e| panic!("failed to write {}: {e}", dst.display()));
    }

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .warnings(false)
        .include(&shim_dir) // first: shadows OrbiterAPI.h
        .include(&out_dir) // second: patched Vecmat.h/.cpp, Astro.h/.cpp
        .file(shim_dir.join("shim.cpp"))
        .file(out_dir.join("Vecmat.cpp"))
        .file(out_dir.join("Astro.cpp"))
        .compile("orbitx_oracle");

    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("shim.cpp").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("orbiter_stub.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("OrbiterAPI.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        orbiter_src.join("Vecmat.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        orbiter_src.join("Vecmat.cpp").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        orbiter_src.join("Astro.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        orbiter_src.join("Astro.cpp").display()
    );
}
