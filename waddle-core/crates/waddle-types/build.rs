//! Compiles the waddle-protocol schemas at build time.
//!
//! Uses protox (a pure-Rust protobuf compiler) so no system `protoc` is ever
//! required. Messages only — gRPC service codegen lives in waddle-controlplane
//! behind its `tonic-transport` feature, with `extern_path` pointing back at
//! these types.

use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let proto_root = manifest_dir.join("../../../waddle-protocol/proto");
    let proto_root = proto_root
        .canonicalize()
        .expect("waddle-protocol/proto must exist as a sibling of waddle-core");

    let files: Vec<PathBuf> = [
        "descriptors.proto",
        "control.proto",
        "episode.proto",
        "sidecar.proto",
        "services.proto",
        "media.proto",
    ]
    .iter()
    .map(|f| proto_root.join("waddle/v0").join(f))
    .collect();

    for f in &files {
        println!("cargo:rerun-if-changed={}", f.display());
    }

    let fds = protox::compile(&files, [&proto_root]).expect("waddle-protocol schemas must compile");

    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let descriptor_path = out_dir.join("descriptor_set.bin");
    // compile_fds() does not write file_descriptor_set_path itself; the
    // embedded descriptor set (MCAP channel schemas) is serialized here.
    std::fs::write(&descriptor_path, prost::Message::encode_to_vec(&fds))
        .expect("descriptor set must be writable");

    prost_build::Config::new()
        .compile_fds(fds)
        .expect("prost codegen must succeed");
}
