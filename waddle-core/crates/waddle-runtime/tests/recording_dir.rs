//! What `SessionBuilder::recording_dir` promises: a session told to keep the
//! local archive keeps it, or says at build time why it cannot.
//!
//! The failure this pins out of existence: a directory that does not exist
//! yet. Every writer downstream (`ManifestWriter::open`, the per-episode MCAP,
//! the sidecar rename) opens files INSIDE that directory, so a missing one
//! made every one of them fail — each of them swallowed, one per episode — and
//! the session ran to completion, streamed for as long as it was asked to, and
//! left nothing on disk. The archive is the one thing a local recording
//! session exists for, so it is created here, and a path that cannot be made
//! into a writable directory fails the build instead of the disk.

use waddle_runtime::{RuntimeError, Session};
use waddle_types::TerminalOutcome;
use waddle_types::pb::v0 as pb;

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "recording-bot".into(),
        robot_id: "rec-01".into(),
        cell_id: "cell-rec".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                joints: (0..3)
                    .map(|i| pb::JointDescriptor {
                        name: format!("j{i}"),
                        ..Default::default()
                    })
                    .collect(),
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        }),
        ..Default::default()
    }
}

#[test]
fn a_recording_directory_that_does_not_exist_yet_is_created_and_written_into() {
    let tmp = tempfile::tempdir().unwrap();
    // What a customer program passes: a relative-looking name nobody made.
    let dir = tmp.path().join("recordings");
    assert!(!dir.exists());

    let session = Session::builder("recording-project")
        .robot(robot())
        .recording_dir(&dir)
        .build()
        .expect("a recording directory is created, not required to pre-exist");

    let mut ep = session.start_episode("keep this one").unwrap();
    let id = ep.id().clone();
    for _ in 0..5 {
        ep.gate(&[0.1, 0.2, 0.3], None, Some(&[0.9, 0.8, 0.7]));
    }
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    assert!(
        dir.join(format!("{id}.sidecar.json")).exists(),
        "the episode's sidecar must land in the created directory"
    );
    assert!(
        dir.join(format!("{id}.mcap")).exists(),
        "the episode's MCAP must land in the created directory"
    );
    assert!(
        dir.join("manifest.jsonl").exists(),
        "the manifest index must land in the created directory"
    );
}

#[test]
fn nested_recording_directories_are_created_whole() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().join("runs").join("2026-08-04").join("morning");

    let session = Session::builder("recording-project")
        .robot(robot())
        .recording_dir(&dir)
        .build()
        .unwrap();
    let ep = session.start_episode("nested").unwrap();
    let id = ep.id().clone();
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    assert!(dir.join(format!("{id}.sidecar.json")).exists());
}

#[test]
fn a_recording_dir_that_cannot_be_a_directory_fails_the_build() {
    let tmp = tempfile::tempdir().unwrap();
    // A path that is already a FILE: nothing here can make it a directory,
    // and the session would otherwise open clean and record nothing.
    let path = tmp.path().join("recordings");
    std::fs::write(&path, b"not a directory").unwrap();

    let err = Session::builder("recording-project")
        .robot(robot())
        .recording_dir(&path)
        .build()
        .expect_err("a recording directory that cannot exist is a build failure");

    assert!(
        matches!(err, RuntimeError::RecordingDirUnusable { .. }),
        "expected RecordingDirUnusable, got {err:?}"
    );
    let text = err.to_string();
    assert!(
        text.contains("recordings"),
        "the error must name the path it could not use: {text}"
    );
}
