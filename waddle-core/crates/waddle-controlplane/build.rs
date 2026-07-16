//! gRPC service codegen for the `tonic-transport` feature.
//!
//! Message codegen lives in waddle-types; this build script emits ONLY the
//! `ControlPlane` service client/server (tonic-prost-build over a
//! protox-compiled descriptor set — no system `protoc` is ever required).
//! `extern_path` maps every `.waddle.v0` message back to
//! `waddle_types::pb::v0`, so exactly one copy of the wire types exists.
//! Featureless builds compile none of this (the build-deps are optional).

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    #[cfg(feature = "tonic-transport")]
    grpc_codegen();
}

#[cfg(feature = "tonic-transport")]
fn grpc_codegen() {
    use std::path::PathBuf;

    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let proto_root = manifest_dir.join("../../../waddle-protocol/proto");
    let proto_root = proto_root
        .canonicalize()
        .expect("waddle-protocol/proto must exist as a sibling of waddle-core");

    // The full schema set: services.proto pulls in the rest transitively,
    // but protox wants explicit roots and the list matches waddle-types.
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

    tonic_prost_build::configure()
        .build_client(true)
        // The server is generated for the in-process test plane
        // (tests/grpc_transport.rs); the SDK is never a ControlPlane server.
        .build_server(true)
        .extern_path(".waddle.v0", "::waddle_types::pb::v0")
        .compile_fds(fds)
        .expect("tonic service codegen must succeed");
}
