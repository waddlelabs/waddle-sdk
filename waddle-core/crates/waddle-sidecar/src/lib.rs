//! waddle-sidecar — the per-episode semantic record.
//!
//! The sidecar record IS [`waddle_types::pb::v0::Sidecar`]: there is no
//! parallel serde model. JSON serialization goes through prost-reflect's
//! `DynamicMessage` with canonical proto3 JSON (lowerCamelCase field names,
//! int64-as-string, defaults omitted), so sidecar files on disk are
//! wire-exact with the golden fixtures in
//! `waddle-protocol/fixtures/sidecars/`.
//!
//! Pieces:
//! - [`json`] — canonical proto3 JSON to/from `pb::Sidecar`/`pb::EpisodeEvent`;
//! - [`builder`] — [`SidecarBuilder`]: accumulates the episode event stream
//!   and derives claim/lease/provenance/intervention spans incrementally;
//! - [`writer`] — atomic sidecar file writes + the append-only
//!   `manifest.jsonl`;
//! - [`mcaprec`] — the Local-mode MCAP episode recorder;
//! - [`events`] — the bounded in-memory event ring and incident persistence;
//! - [`reference`] — Reference-mode helpers ([`RefResolver`],
//!   [`StreamRefBuilder`]).
//!
//! Two-clock discipline: episode bounds' epoch twins are set from [`Stamp`]s
//! captured at open/close time — never derived from the [`ClockAnchor`] at
//! close (that derivation is the production postmortem this workspace is
//! structured around).
//!
//! [`Stamp`]: waddle_types::time::Stamp
//! [`ClockAnchor`]: waddle_types::time::ClockAnchor

pub mod builder;
pub mod error;
pub mod events;
pub mod json;
pub mod mcaprec;
pub mod reference;
pub mod writer;

pub use builder::SidecarBuilder;
pub use error::SidecarError;
pub use events::{EventRing, IncidentPersist, NullIncidentPersist};
pub use json::{event_to_json, sidecar_from_json, sidecar_to_json};
pub use mcaprec::McapEpisodeWriter;
pub use reference::{RefResolver, StreamRefBuilder};
pub use writer::{ManifestWriter, write_sidecar};
