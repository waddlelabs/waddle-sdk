fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    // Source/wheel builds need this even outside the repository Cargo config.
    // Build-script link arguments on transitive dependencies do not reach our
    // final cdylib, so select it on the shim's own media feature and target.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos")
        && std::env::var_os("CARGO_FEATURE_LIVEKIT").is_some()
    {
        println!("cargo:rustc-link-arg=-ObjC");
    }
}
