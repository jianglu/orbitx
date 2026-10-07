use std::path::PathBuf;
#[cfg(windows)]
use std::path::Path;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let shim_dir = manifest_dir.join("cpp");

    let mut b = cc::Build::new();
    b.cpp(true)
        .std("c++17")
        .warnings(false)
        .flag_if_supported("-ffp-contract=off")
        .include(&shim_dir)
        .file(shim_dir.join("shim.cpp"))
        .file(shim_dir.join("rocket_aero.cpp"));

    // 仅 Windows：shim.cpp 含 C99 复合字面量 `(T){...}`，MSVC C++17 报 C4576。
    // clang-cl 接受且与 x86_64-pc-windows-msvc 的 rustc 链接兼容。不改 oracle 源码。
    // 其它平台保持 cc 默认编译器（gcc/clang），不探测 clang-cl。
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        if let Some(clang_cl) = find_clang_cl() {
            b.compiler(clang_cl);
        }
    }

    b.compile("orbitx_dyn_oracle");

    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("shim.cpp").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("rocket_aero.cpp").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        shim_dir.join("oracle.h").display()
    );
}

#[cfg(windows)]
fn find_clang_cl() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("CLANG_CL") {
        let pb = PathBuf::from(p);
        if pb.is_file() {
            return Some(pb);
        }
    }
    if let Ok(out) = std::process::Command::new("where.exe")
        .arg("clang-cl.exe")
        .output()
    {
        if out.status.success() {
            if let Some(line) = std::str::from_utf8(&out.stdout)
                .ok()
                .and_then(|s| s.lines().next())
            {
                let pb = PathBuf::from(line.trim());
                if pb.is_file() {
                    return Some(pb);
                }
            }
        }
    }
    let fallback = Path::new(r"D:\llvm\bin\clang-cl.exe");
    if fallback.is_file() {
        return Some(fallback.to_path_buf());
    }
    None
}

#[cfg(not(windows))]
fn find_clang_cl() -> Option<PathBuf> {
    None
}
