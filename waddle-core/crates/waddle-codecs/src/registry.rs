//! The codec registry: registration, mandatory round-trip certification,
//! and version resolution with **no floating "latest"** (N15).
//!
//! The write path never resolves "whatever is newest": a codec that encodes
//! chunks onto a robot's wire is pinned like any other actuation-relevant
//! artifact. Lookup succeeds only when a `(dialect, VersionReq)` pair
//! resolves to exactly one *certified* codec — exact pins (`=x.y.z`) always
//! do; ranges succeed only when unambiguous, and are refused otherwise.

use std::sync::Arc;

use waddle_types::pb::v0 as pb;

use crate::traits::{Codec, ObsFrame};

/// Errors from registry operations.
#[derive(Debug, thiserror::Error)]
pub enum RegistryError {
    /// A codec with this (dialect, version) is already registered.
    #[error("codec {dialect} {version} is already registered")]
    Duplicate {
        dialect: String,
        version: semver::Version,
    },
    /// The codec handed to [`Registry::certify`] was never registered.
    #[error("codec {dialect} {version} is not registered")]
    NotRegistered {
        dialect: String,
        version: semver::Version,
    },
    /// No registered codec matches the request at all.
    #[error("no codec registered for dialect {dialect:?} matching {req}")]
    NoMatch { dialect: String, req: String },
    /// Codecs match, but none has passed certification. Uncertified codecs
    /// are never returned by lookup.
    #[error(
        "codec(s) for dialect {dialect:?} matching {req} exist but none is certified: {versions:?}"
    )]
    NotCertified {
        dialect: String,
        req: String,
        versions: Vec<semver::Version>,
    },
    /// More than one certified codec matches. Refused rather than resolved:
    /// picking the newest would be a floating "latest" in the write path
    /// (N15). Pin exactly (`=x.y.z`) or narrow the range.
    #[error(
        "ambiguous codec request: dialect {dialect:?} req {req} matches {versions:?}; \
         pin an exact version (no \"latest\" resolution, N15)"
    )]
    Ambiguous {
        dialect: String,
        req: String,
        versions: Vec<semver::Version>,
    },
}

/// The fixture corpus a codec must round-trip to be certified. Fixtures must
/// lie in the dialect's representable subset — a fixture the wire cannot
/// carry (e.g. a provenance-tagged chunk on a wire without provenance) is a
/// certification failure by design, because it documents exactly what the
/// dialect loses.
#[derive(Debug, Clone, Default)]
pub struct CertFixtures {
    pub obs: Vec<ObsFrame>,
    pub actions: Vec<pb::ActionChunk>,
}

/// One certification failure: which fixture, and why.
#[derive(Debug, Clone)]
pub struct CertFailure {
    /// `"obs[i]"` or `"actions[i]"`.
    pub fixture: String,
    pub reason: String,
}

/// The result of certifying one codec against a fixture corpus.
#[derive(Debug, Clone)]
pub struct CertReport {
    pub dialect: String,
    pub version: semver::Version,
    pub obs_checked: usize,
    pub actions_checked: usize,
    pub failures: Vec<CertFailure>,
}

impl CertReport {
    /// Certification passes only when at least one fixture ran and every
    /// fixture round-tripped. An empty corpus never certifies: "vacuously
    /// green" is how an unexercised codec reaches a robot.
    #[must_use]
    pub fn passed(&self) -> bool {
        self.failures.is_empty() && (self.obs_checked + self.actions_checked) > 0
    }
}

struct Entry {
    codec: Arc<dyn Codec>,
    certified: bool,
}

impl std::fmt::Debug for Entry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Entry")
            .field("descriptor", self.codec.descriptor())
            .field("certified", &self.certified)
            .finish()
    }
}

/// Codec registry. Stores codecs together with their certification state;
/// see the module docs for the resolution rules.
#[derive(Debug, Default)]
pub struct Registry {
    entries: Vec<Entry>,
}

impl Registry {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a codec. It starts **uncertified** and is invisible to
    /// lookup until [`Self::certify`] passes.
    pub fn register(&mut self, codec: Arc<dyn Codec>) -> Result<(), RegistryError> {
        let d = codec.descriptor();
        if self.find(&d.dialect, &d.version).is_some() {
            return Err(RegistryError::Duplicate {
                dialect: d.dialect.clone(),
                version: d.version.clone(),
            });
        }
        self.entries.push(Entry {
            codec,
            certified: false,
        });
        Ok(())
    }

    /// Mandatory round-trip certification: for every fixture `x`, assert
    /// `decode(encode(x)) == x` — obs frames through
    /// `encode_obs`/`decode_obs`, chunks through
    /// `encode_action`/`decode_action`. The result is stored: a codec whose
    /// last certification failed is never returned by lookup.
    pub fn certify(
        &mut self,
        codec: &Arc<dyn Codec>,
        fixtures: &CertFixtures,
    ) -> Result<CertReport, RegistryError> {
        let d = codec.descriptor().clone();
        let Some(idx) = self.find(&d.dialect, &d.version) else {
            return Err(RegistryError::NotRegistered {
                dialect: d.dialect,
                version: d.version,
            });
        };
        let codec = &self.entries[idx].codec;

        let mut failures = Vec::new();
        for (i, obs) in fixtures.obs.iter().enumerate() {
            if let Err(reason) = round_trip_obs(codec.as_ref(), obs) {
                failures.push(CertFailure {
                    fixture: format!("obs[{i}]"),
                    reason,
                });
            }
        }
        for (i, chunk) in fixtures.actions.iter().enumerate() {
            if let Err(reason) = round_trip_action(codec.as_ref(), chunk) {
                failures.push(CertFailure {
                    fixture: format!("actions[{i}]"),
                    reason,
                });
            }
        }

        let report = CertReport {
            dialect: d.dialect,
            version: d.version,
            obs_checked: fixtures.obs.len(),
            actions_checked: fixtures.actions.len(),
            failures,
        };
        self.entries[idx].certified = report.passed();
        Ok(report)
    }

    /// Resolve `(dialect, req)` to a certified codec. Exact pinning
    /// (`=x.y.z`) is the preferred form; a range succeeds only when exactly
    /// one certified codec matches. Multiple matches are refused — never
    /// resolved to the newest (N15).
    pub fn lookup(
        &self,
        dialect: &str,
        req: &semver::VersionReq,
    ) -> Result<Arc<dyn Codec>, RegistryError> {
        let matching: Vec<&Entry> = self
            .entries
            .iter()
            .filter(|e| {
                let d = e.codec.descriptor();
                d.dialect == dialect && req.matches(&d.version)
            })
            .collect();
        if matching.is_empty() {
            return Err(RegistryError::NoMatch {
                dialect: dialect.to_owned(),
                req: req.to_string(),
            });
        }
        let certified: Vec<&Entry> = matching.iter().copied().filter(|e| e.certified).collect();
        match certified.as_slice() {
            [] => Err(RegistryError::NotCertified {
                dialect: dialect.to_owned(),
                req: req.to_string(),
                versions: versions_of(&matching),
            }),
            [one] => Ok(Arc::clone(&one.codec)),
            _ => Err(RegistryError::Ambiguous {
                dialect: dialect.to_owned(),
                req: req.to_string(),
                versions: versions_of(&certified),
            }),
        }
    }

    /// Exact-pin lookup by `(dialect, version)`.
    pub fn lookup_exact(
        &self,
        dialect: &str,
        version: &semver::Version,
    ) -> Result<Arc<dyn Codec>, RegistryError> {
        self.lookup(dialect, &exact_req(version))
    }

    fn find(&self, dialect: &str, version: &semver::Version) -> Option<usize> {
        self.entries.iter().position(|e| {
            let d = e.codec.descriptor();
            d.dialect == dialect && &d.version == version
        })
    }
}

fn versions_of(entries: &[&Entry]) -> Vec<semver::Version> {
    entries
        .iter()
        .map(|e| e.codec.descriptor().version.clone())
        .collect()
}

/// The `=x.y.z` requirement for a version.
#[must_use]
pub fn exact_req(version: &semver::Version) -> semver::VersionReq {
    semver::VersionReq {
        comparators: vec![semver::Comparator {
            op: semver::Op::Exact,
            major: version.major,
            minor: Some(version.minor),
            patch: Some(version.patch),
            pre: version.pre.clone(),
        }],
    }
}

fn round_trip_obs(codec: &dyn Codec, obs: &ObsFrame) -> Result<(), String> {
    let wire = codec
        .encode_obs(obs)
        .map_err(|e| format!("encode_obs: {e}"))?;
    let back = codec
        .decode_obs(&wire)
        .map_err(|e| format!("decode_obs: {e}"))?;
    if &back == obs {
        Ok(())
    } else {
        Err("decode(encode(obs)) != obs".to_owned())
    }
}

fn round_trip_action(codec: &dyn Codec, chunk: &pb::ActionChunk) -> Result<(), String> {
    let wire = codec
        .encode_action(chunk)
        .map_err(|e| format!("encode_action: {e}"))?;
    let back = codec
        .decode_action(&wire)
        .map_err(|e| format!("decode_action: {e}"))?;
    if &back == chunk {
        Ok(())
    } else {
        Err("decode(encode(chunk)) != chunk".to_owned())
    }
}
