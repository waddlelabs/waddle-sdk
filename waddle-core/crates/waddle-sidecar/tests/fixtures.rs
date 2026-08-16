//! The golden sidecar fixtures pin the JSON dialect: every fixture in
//! `waddle-protocol/fixtures/sidecars/` must parse, and re-serializing
//! through `pb::Sidecar` must be semantically stable.
//!
//! Fixtures are enumerated from the directory at test time — never a
//! hand-kept list (append-only directories plus a hand-kept list is how
//! coverage silently rots; see the fixtures README).

use std::path::PathBuf;

use waddle_sidecar::{sidecar_from_json, sidecar_to_json};

fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../waddle-protocol/fixtures/sidecars")
}

fn fixture_paths() -> Vec<PathBuf> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(fixtures_dir())
        .expect("fixtures directory exists")
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "json"))
        .collect();
    paths.sort();
    paths
}

#[test]
fn all_golden_sidecars_parse_and_round_trip_semantically() {
    let paths = fixture_paths();
    // 7 goldens exist today; the directory is append-only.
    assert!(
        paths.len() >= 7,
        "expected at least 7 sidecar fixtures, found {}",
        paths.len()
    );

    for path in paths {
        let raw = std::fs::read_to_string(&path).unwrap();
        let envelope: serde_json::Value = serde_json::from_str(&raw).unwrap();
        assert!(
            matches!(
                envelope["format"].as_str(),
                Some("waddle_sdk.fixture/v0" | "waddle.fixture/v0")
            ),
            "{}: bad envelope format",
            path.display()
        );
        assert_eq!(
            envelope["type"],
            "waddle.v0.Sidecar",
            "{}: bad envelope type",
            path.display()
        );

        // Strip the envelope; parse the golden message.
        let message = serde_json::to_string(&envelope["message"]).unwrap();
        let pb1 = sidecar_from_json(&message)
            .unwrap_or_else(|e| panic!("{} does not parse: {e}", path.display()));
        assert_eq!(
            pb1.sidecar_version,
            1,
            "{}: sidecar_version",
            path.display()
        );
        assert!(!pb1.episode_id.is_empty(), "{}", path.display());

        // json → pb → json must be semantically stable: canonicalize both
        // sides through pb again and compare as values (key order,
        // whitespace, and numeric formatting are irrelevant; comparing
        // JSON text is non-conforming).
        let json1 = sidecar_to_json(&pb1).unwrap();
        let pb2 = sidecar_from_json(&json1).unwrap();
        assert_eq!(pb1, pb2, "{}: pb round trip drifted", path.display());
        let json2 = sidecar_to_json(&pb2).unwrap();
        let v1: serde_json::Value = serde_json::from_str(&json1).unwrap();
        let v2: serde_json::Value = serde_json::from_str(&json2).unwrap();
        assert_eq!(v1, v2, "{}: JSON round trip drifted", path.display());
    }
}

#[test]
fn nominal_fixture_fields_survive_the_round_trip() {
    // Spot-check one fixture's semantics end to end (not just stability).
    let raw = std::fs::read_to_string(fixtures_dir().join("sidecar_nominal_success.json")).unwrap();
    let envelope: serde_json::Value = serde_json::from_str(&raw).unwrap();
    let s = sidecar_from_json(&serde_json::to_string(&envelope["message"]).unwrap()).unwrap();

    assert_eq!(s.episode_id, "int-4f3a9b2c1d");
    assert_eq!(s.outcome(), waddle_types::pb::v0::TerminalOutcome::Success);
    assert_eq!(s.bounds.as_ref().unwrap().t_start_ns, 3_620_000_000_000);
    assert_eq!(s.t_start_unix_ns, 1_784_000_020_000_000_000);
    assert_eq!(
        s.clock_anchor.as_ref().unwrap().monotonic_ns,
        3_600_000_000_000
    );
    assert_eq!(s.leases.len(), 1);
    assert_eq!(s.events.len(), 4);
    assert_eq!(s.task_metadata["variant"], "bath-towel");
    assert!(s.audit.as_ref().unwrap().random_quota_sample);
}
