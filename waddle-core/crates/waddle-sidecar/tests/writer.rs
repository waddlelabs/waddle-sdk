//! Atomic sidecar writes and the append-only manifest.

use waddle_sidecar::{ManifestWriter, sidecar_from_json, write_sidecar};
use waddle_types::pb::v0 as pb;

fn sidecar(episode_id: &str, outcome: pb::TerminalOutcome) -> pb::Sidecar {
    pb::Sidecar {
        sidecar_version: 1,
        episode_id: episode_id.into(),
        project: "towel-folding-pilot".into(),
        session_id: "sess-7d41f0".into(),
        robot_id: "yam-01".into(),
        cell_id: "cell-a".into(),
        task: "fold_towel_half".into(),
        t_start_unix_ns: 1_784_000_020_000_000_000,
        outcome: outcome as i32,
        ..Default::default()
    }
}

#[test]
fn write_is_atomic_and_named_by_episode_id() {
    let dir = tempfile::tempdir().unwrap();
    let s = sidecar("int-w1", pb::TerminalOutcome::Success);
    let path = write_sidecar(dir.path(), &s).unwrap();

    assert_eq!(path, dir.path().join("int-w1.sidecar.json"));
    // No .tmp left behind.
    let names: Vec<String> = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    assert_eq!(names, ["int-w1.sidecar.json"]);

    // The file parses back to the same record.
    let raw = std::fs::read_to_string(&path).unwrap();
    let back = sidecar_from_json(&raw).unwrap();
    assert_eq!(back, s);

    // Rewriting the same episode replaces the file, still atomically.
    let mut s2 = s.clone();
    s2.outcome_detail = "amended".into();
    let path2 = write_sidecar(dir.path(), &s2).unwrap();
    assert_eq!(path, path2);
    let back = sidecar_from_json(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(back.outcome_detail, "amended");
}

#[test]
fn empty_episode_id_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    assert!(write_sidecar(dir.path(), &pb::Sidecar::default()).is_err());
}

#[test]
fn manifest_appends_one_line_per_episode() {
    let dir = tempfile::tempdir().unwrap();
    let mut manifest = ManifestWriter::open(dir.path()).unwrap();

    for (id, outcome) in [
        ("int-m1", pb::TerminalOutcome::Success),
        ("int-m2", pb::TerminalOutcome::AbortedRetake),
    ] {
        let s = sidecar(id, outcome);
        let path = write_sidecar(dir.path(), &s).unwrap();
        manifest.append(&s, &path).unwrap();
    }
    // Re-opening appends, never truncates.
    drop(manifest);
    let mut manifest = ManifestWriter::open(dir.path()).unwrap();
    let s = sidecar("int-m3", pb::TerminalOutcome::Failure);
    let path = write_sidecar(dir.path(), &s).unwrap();
    manifest.append(&s, &path).unwrap();

    let raw = std::fs::read_to_string(dir.path().join("manifest.jsonl")).unwrap();
    let lines: Vec<serde_json::Value> = raw
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(lines.len(), 3);
    assert_eq!(lines[0]["episodeId"], "int-m1");
    assert_eq!(lines[0]["outcome"], "TERMINAL_OUTCOME_SUCCESS");
    assert_eq!(lines[0]["task"], "fold_towel_half");
    assert_eq!(lines[0]["tStartUnixNs"], "1784000020000000000");
    assert_eq!(lines[0]["robotId"], "yam-01");
    assert_eq!(lines[0]["cellId"], "cell-a");
    assert_eq!(
        lines[0]["path"].as_str().unwrap(),
        dir.path().join("int-m1.sidecar.json").to_string_lossy()
    );
    assert_eq!(lines[1]["outcome"], "TERMINAL_OUTCOME_ABORTED_RETAKE");
    assert_eq!(lines[2]["episodeId"], "int-m3");
}
