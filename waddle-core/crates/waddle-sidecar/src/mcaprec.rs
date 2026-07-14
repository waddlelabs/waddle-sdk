//! Local-mode MCAP episode recorder.
//!
//! Writes one MCAP file per episode: the clock anchor as a metadata record
//! (`"waddle/clock_anchor"`, values as decimal strings), plus protobuf
//! channels created lazily per topic. Channel schemas carry the full
//! `waddle-types` FileDescriptorSet, so any MCAP reader can decode the
//! messages without this repo checked out.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use prost::Message as _;
use waddle_types::pb::v0 as pb;
use waddle_types::time::ClockAnchor;

use crate::error::SidecarError;

/// Topic for the episode event stream.
pub const EVENTS_TOPIC: &str = "/waddle/events";
/// Topic for gated action chunks.
pub const ACTIONS_TOPIC: &str = "/waddle/actions";
/// Name of the clock-anchor metadata record.
pub const CLOCK_ANCHOR_METADATA: &str = "waddle/clock_anchor";

/// One episode's MCAP writer. Events and actions are the minimal channel
/// set; series/media channels arrive with `waddle-media`.
pub struct McapEpisodeWriter {
    writer: mcap::Writer<BufWriter<File>>,
    /// topic → (channel id, next sequence number).
    channels: HashMap<&'static str, (u16, u32)>,
}

impl std::fmt::Debug for McapEpisodeWriter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McapEpisodeWriter")
            .field("channels", &self.channels)
            .finish_non_exhaustive()
    }
}

impl McapEpisodeWriter {
    /// Create the file and record the session clock anchor as metadata.
    /// The anchor locates this file's monotonic timestamps on the wall
    /// clock at read time; it is captured at file open, per the two-clock
    /// discipline.
    pub fn create(path: &Path, anchor: ClockAnchor) -> Result<Self, SidecarError> {
        let file = BufWriter::new(File::create(path)?);
        let mut writer = mcap::WriteOptions::new()
            .library("waddle-sidecar")
            .create(file)?;
        writer.write_metadata(&mcap::records::Metadata {
            name: CLOCK_ANCHOR_METADATA.to_owned(),
            metadata: BTreeMap::from([
                ("monotonic_ns".to_owned(), anchor.monotonic_ns.0.to_string()),
                ("unix_ns".to_owned(), anchor.unix_ns.0.to_string()),
            ]),
        })?;
        Ok(Self {
            writer,
            channels: HashMap::new(),
        })
    }

    /// Write one episode event on [`EVENTS_TOPIC`]; `log_time` is the
    /// event's session-monotonic `t_ns`.
    pub fn write_event(&mut self, event: &pb::EpisodeEvent) -> Result<(), SidecarError> {
        let data = event.encode_to_vec();
        self.write_message(EVENTS_TOPIC, "waddle.v0.EpisodeEvent", event.t_ns, &data)
    }

    /// Write one action chunk on [`ACTIONS_TOPIC`]; `log_time` is the
    /// chunk's `t_emitted_ns`.
    pub fn write_action(&mut self, chunk: &pb::ActionChunk) -> Result<(), SidecarError> {
        let data = chunk.encode_to_vec();
        self.write_message(
            ACTIONS_TOPIC,
            "waddle.v0.ActionChunk",
            chunk.t_emitted_ns,
            &data,
        )
    }

    /// Finish the MCAP (summary section, footer) and flush.
    pub fn finish(mut self) -> Result<(), SidecarError> {
        self.writer.finish()?;
        Ok(())
    }

    fn write_message(
        &mut self,
        topic: &'static str,
        schema_name: &str,
        t_ns: i64,
        data: &[u8],
    ) -> Result<(), SidecarError> {
        let (channel_id, sequence) = self.channel(topic, schema_name)?;
        let log_time = u64::try_from(t_ns).unwrap_or(0);
        self.writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id,
                sequence,
                log_time,
                publish_time: log_time,
            },
            data,
        )?;
        Ok(())
    }

    /// Lazily create the channel (and its protobuf schema) for a topic;
    /// returns the channel id and the next per-channel sequence number.
    fn channel(
        &mut self,
        topic: &'static str,
        schema_name: &str,
    ) -> Result<(u16, u32), SidecarError> {
        if let Some(entry) = self.channels.get_mut(topic) {
            let seq = entry.1;
            entry.1 += 1;
            return Ok((entry.0, seq));
        }
        let schema_id =
            self.writer
                .add_schema(schema_name, "protobuf", waddle_types::FILE_DESCRIPTOR_SET)?;
        let channel_id = self
            .writer
            .add_channel(schema_id, topic, "protobuf", &BTreeMap::new())?;
        self.channels.insert(topic, (channel_id, 1));
        Ok((channel_id, 0))
    }
}
