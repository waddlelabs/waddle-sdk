//! Durable sidecar persistence: atomic per-episode JSON files plus the
//! append-only `manifest.jsonl` index.

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

use waddle_types::pb::v0 as pb;

use crate::error::SidecarError;
use crate::json::sidecar_to_json;

/// Atomically write `<episode_id>.sidecar.json` into `dir`:
/// serialize → write `<path>.tmp` → fsync → rename. A crash leaves either
/// the previous file or the complete new one, never a torn record.
pub fn write_sidecar(dir: &Path, sidecar: &pb::Sidecar) -> Result<PathBuf, SidecarError> {
    if sidecar.episode_id.is_empty() {
        return Err(SidecarError::Invalid(
            "sidecar has an empty episode_id".to_owned(),
        ));
    }
    let json = sidecar_to_json(sidecar)?;
    let path = dir.join(format!("{}.sidecar.json", sidecar.episode_id));
    let tmp = dir.join(format!("{}.sidecar.json.tmp", sidecar.episode_id));
    {
        let mut f = File::create(&tmp)?;
        f.write_all(json.as_bytes())?;
        f.write_all(b"\n")?;
        f.sync_all()?;
    }
    std::fs::rename(&tmp, &path)?;
    // Make the rename itself durable. Best-effort: not every filesystem
    // supports fsync on a directory handle.
    if let Ok(d) = File::open(dir) {
        let _ = d.sync_all();
    }
    Ok(path)
}

/// Append-only writer for `manifest.jsonl`: one compact JSON line per
/// episode, so a corpus indexer can scan a recording directory without
/// parsing every sidecar.
#[derive(Debug)]
pub struct ManifestWriter {
    file: File,
}

impl ManifestWriter {
    /// Open (creating if absent) `<dir>/manifest.jsonl` for appending.
    pub fn open(dir: &Path) -> Result<Self, SidecarError> {
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(dir.join("manifest.jsonl"))?;
        Ok(Self { file })
    }

    /// Append one line for `sidecar`, whose record was written at `path`.
    pub fn append(&mut self, sidecar: &pb::Sidecar, path: &Path) -> Result<(), SidecarError> {
        let outcome = pb::TerminalOutcome::try_from(sidecar.outcome)
            .unwrap_or(pb::TerminalOutcome::Unspecified)
            .as_str_name();
        let line = serde_json::json!({
            "episodeId": sidecar.episode_id,
            "outcome": outcome,
            "task": sidecar.task,
            // Same convention as the record itself: int64 as decimal string.
            "tStartUnixNs": sidecar.t_start_unix_ns.to_string(),
            "robotId": sidecar.robot_id,
            "cellId": sidecar.cell_id,
            "path": path.to_string_lossy(),
        });
        let mut buf = serde_json::to_string(&line)?;
        buf.push('\n');
        self.file.write_all(buf.as_bytes())?;
        self.file.sync_data()?;
        Ok(())
    }
}
