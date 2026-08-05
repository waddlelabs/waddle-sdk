//! Every golden wire fixture in `waddle-protocol/fixtures/wire/` parses
//! STRICTLY against the descriptor its own envelope names, and re-serializing
//! is semantically stable.
//!
//! Why this exists: the wire fixtures are the protocol's worked examples of
//! its own messages, and until now only one of them was ever read by any test
//! (the robot declarations a scenario's `setup.robot_fixture` points at). The
//! rest were prose with a `.json` extension — a fixture with a misspelled
//! field, a wrong enum name, or an int64 written as a number rather than a
//! decimal string would sit in the repo indefinitely, teaching the shape
//! wrong to every reader and every future implementer who copies it.
//!
//! Fixtures are enumerated from the directory at test time — never a
//! hand-kept list (append-only directories plus a hand-kept list is how
//! coverage silently rots; the same rule `waddle-sidecar`'s sidecar-fixture
//! suite follows).

use std::path::PathBuf;

use waddle_conformance::Codec;

fn wire_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../waddle-protocol/fixtures/wire")
}

fn fixture_paths() -> Vec<PathBuf> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(wire_dir())
        .expect("wire fixtures directory exists")
        .map(|e| e.expect("dir entry").path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "json"))
        .collect();
    paths.sort();
    paths
}

#[test]
fn every_wire_fixture_parses_strictly_and_round_trips() {
    let paths = fixture_paths();
    // 10 goldens exist today; the directory is append-only.
    assert!(
        paths.len() >= 10,
        "expected at least 10 wire fixtures, found {}",
        paths.len()
    );

    let codec = Codec::new().expect("descriptor pool");
    for path in &paths {
        let raw = std::fs::read_to_string(path).expect("fixture readable");
        let envelope: serde_json::Value =
            serde_json::from_str(&raw).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        assert_eq!(
            envelope["format"],
            "waddle.fixture/v0",
            "{}: bad envelope format",
            path.display()
        );
        let full_name = envelope["type"]
            .as_str()
            .unwrap_or_else(|| panic!("{}: envelope \"type\" must be a string", path.display()));
        assert!(
            full_name.starts_with("waddle.v0."),
            "{}: envelope type {full_name:?} is not a v0 message",
            path.display()
        );
        // Every fixture is a worked example someone will read; an unlabelled
        // one teaches the shape without saying what it is an example OF.
        assert!(
            envelope["description"]
                .as_str()
                .is_some_and(|d| !d.is_empty()),
            "{}: a wire fixture must carry a non-empty description",
            path.display()
        );

        // Strict: prost-reflect denies unknown fields, so a misspelled field
        // or a wrong enum name fails here rather than teaching a reader a
        // shape the wire does not have.
        let message = &envelope["message"];
        let parsed = codec
            .parse_dynamic(full_name, message)
            .unwrap_or_else(|e| panic!("{} does not parse: {e}", path.display()));

        // json → message → json must be semantically stable. Compared as
        // values through a second parse: key order, whitespace, and the
        // canonical form's own choices (default fields present, int64 as
        // strings) are not what a fixture pins.
        let once = codec.dynamic_to_value(&parsed).expect("serialize");
        let again = codec
            .parse_dynamic(full_name, &once)
            .unwrap_or_else(|e| panic!("{} does not re-parse: {e}", path.display()));
        assert_eq!(
            once,
            codec.dynamic_to_value(&again).expect("serialize"),
            "{}: round trip drifted",
            path.display()
        );
    }
}
