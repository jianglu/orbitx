fn main() -> Result<(), Box<dyn std::error::Error>> {
    std::env::set_var("PROTOC", protoc_bin_vendored::protoc_bin_path()?);
    let mut config = prost_build::Config::new();
    config.btree_map(["."]);
    config.compile_protos(
        &[
            "proto/orbitx/v1/inbound.proto",
            "proto/orbitx/v1/slice.proto",
        ],
        &["proto"],
    )?;
    Ok(())
}
