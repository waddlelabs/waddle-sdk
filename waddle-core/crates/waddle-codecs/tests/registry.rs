//! Registry behavior: mandatory certification, exact pinning, and the
//! no-floating-latest rule (N15).

use std::sync::Arc;

use waddle_codecs::registry::exact_req;
use waddle_codecs::{
    CertFixtures, Codec, CodecDescriptor, LerobotAsyncCodec, ObsFrame, OpenPiCodec, Registry,
    RegistryError,
};
use waddle_types::pb::v0 as pb;

fn fixtures() -> CertFixtures {
    CertFixtures {
        obs: vec![
            ObsFrame {
                t_ns: 1_000,
                state: vec![0.0, 0.5, -0.5],
                images: vec![("cam".into(), bytes::Bytes::from_static(b"\x00\xff"))],
                task: "fold_towel_half".into(),
            },
            ObsFrame::default(),
        ],
        actions: vec![chunk(
            vec![vec![0.1, 0.2], vec![0.3, 0.4]],
            100_000_000,
            900,
            3,
        )],
    }
}

fn chunk(rows: Vec<Vec<f64>>, horizon_ns: i64, t_obs_ns: i64, seq: u64) -> pb::ActionChunk {
    let step = horizon_ns / rows.len() as i64;
    pb::ActionChunk {
        actions: rows
            .into_iter()
            .enumerate()
            .map(|(i, values)| pb::Action {
                t_offset_ns: i as i64 * step,
                target: Some(pb::action::Target::JointPosition(pb::JointVector {
                    values,
                })),
                ..Default::default()
            })
            .collect(),
        horizon_ns,
        t_obs_ns,
        seq,
        ..Default::default()
    }
}

#[test]
fn certification_gates_lookup() {
    let mut registry = Registry::new();
    let codec: Arc<dyn Codec> = Arc::new(LerobotAsyncCodec::new());
    registry.register(Arc::clone(&codec)).unwrap();

    // Registered but uncertified: invisible to lookup.
    let req = exact_req(&semver::Version::new(0, 1, 0));
    assert!(matches!(
        registry.lookup("lerobot-async", &req),
        Err(RegistryError::NotCertified { .. })
    ));

    let report = registry.certify(&codec, &fixtures()).unwrap();
    assert!(report.passed(), "failures: {:?}", report.failures);
    assert_eq!(report.obs_checked, 2);
    assert_eq!(report.actions_checked, 1);

    let found = registry.lookup("lerobot-async", &req).unwrap();
    assert_eq!(found.descriptor().dialect, "lerobot-async");
}

#[test]
fn both_in_tree_dialects_certify_green() {
    let mut registry = Registry::new();
    for codec in [
        Arc::new(LerobotAsyncCodec::new()) as Arc<dyn Codec>,
        Arc::new(OpenPiCodec::new()) as Arc<dyn Codec>,
    ] {
        registry.register(Arc::clone(&codec)).unwrap();
        let report = registry.certify(&codec, &fixtures()).unwrap();
        assert!(
            report.passed(),
            "{} failed certification: {:?}",
            report.dialect,
            report.failures
        );
    }
    registry
        .lookup_exact("openpi", &semver::Version::new(0, 1, 0))
        .unwrap();
}

#[test]
fn non_representable_fixture_fails_certification_and_lookup_refuses() {
    let mut registry = Registry::new();
    let codec: Arc<dyn Codec> = Arc::new(OpenPiCodec::new());
    registry.register(Arc::clone(&codec)).unwrap();

    // A provenance-tagged chunk cannot survive the openpi wire; the round
    // trip must fail and the codec must stay uncertified.
    let mut bad = fixtures();
    bad.actions[0].provenance = Some(pb::ProvenanceTag {
        kind: pb::ProvenanceKind::Teleop as i32,
        ..Default::default()
    });
    let report = registry.certify(&codec, &bad).unwrap();
    assert!(!report.passed());
    assert_eq!(report.failures.len(), 1);
    assert!(report.failures[0].fixture.starts_with("actions"));

    assert!(matches!(
        registry.lookup_exact("openpi", &semver::Version::new(0, 1, 0)),
        Err(RegistryError::NotCertified { .. })
    ));

    // A later green certification restores it.
    let report = registry.certify(&codec, &fixtures()).unwrap();
    assert!(report.passed());
    registry
        .lookup_exact("openpi", &semver::Version::new(0, 1, 0))
        .unwrap();
}

#[test]
fn empty_fixture_corpus_never_certifies() {
    let mut registry = Registry::new();
    let codec: Arc<dyn Codec> = Arc::new(LerobotAsyncCodec::new());
    registry.register(Arc::clone(&codec)).unwrap();
    let report = registry.certify(&codec, &CertFixtures::default()).unwrap();
    assert!(!report.passed(), "vacuous certification must not pass");
}

/// A second lerobot-async codec at a different version, wrapping the first.
struct V2(LerobotAsyncCodec, CodecDescriptor);

impl V2 {
    fn new() -> Self {
        let inner = LerobotAsyncCodec::new();
        let mut d = inner.descriptor().clone();
        d.version = semver::Version::new(0, 2, 0);
        Self(inner, d)
    }
}

impl Codec for V2 {
    fn descriptor(&self) -> &CodecDescriptor {
        &self.1
    }
    fn caps(&self) -> waddle_codecs::CodecCaps {
        self.0.caps()
    }
    fn decode_obs(&self, wire: &[u8]) -> Result<ObsFrame, waddle_codecs::CodecError> {
        self.0.decode_obs(wire)
    }
    fn encode_obs(&self, obs: &ObsFrame) -> Result<Vec<u8>, waddle_codecs::CodecError> {
        self.0.encode_obs(obs)
    }
    fn decode_action(&self, wire: &[u8]) -> Result<pb::ActionChunk, waddle_codecs::CodecError> {
        self.0.decode_action(wire)
    }
    fn encode_action(&self, chunk: &pb::ActionChunk) -> Result<Vec<u8>, waddle_codecs::CodecError> {
        self.0.encode_action(chunk)
    }
}

#[test]
fn ranges_never_resolve_to_latest() {
    let mut registry = Registry::new();
    let v1: Arc<dyn Codec> = Arc::new(LerobotAsyncCodec::new());
    let v2: Arc<dyn Codec> = Arc::new(V2::new());
    registry.register(Arc::clone(&v1)).unwrap();
    registry.register(Arc::clone(&v2)).unwrap();
    assert!(registry.certify(&v1, &fixtures()).unwrap().passed());
    assert!(registry.certify(&v2, &fixtures()).unwrap().passed());

    // A range matching both is refused, not resolved to the newest (N15).
    let floating = semver::VersionReq::parse(">=0.1.0").unwrap();
    assert!(matches!(
        registry.lookup("lerobot-async", &floating),
        Err(RegistryError::Ambiguous { versions, .. }) if versions.len() == 2
    ));

    // Exact pins resolve deterministically.
    let got = registry
        .lookup_exact("lerobot-async", &semver::Version::new(0, 1, 0))
        .unwrap();
    assert_eq!(got.descriptor().version, semver::Version::new(0, 1, 0));
    let got = registry
        .lookup_exact("lerobot-async", &semver::Version::new(0, 2, 0))
        .unwrap();
    assert_eq!(got.descriptor().version, semver::Version::new(0, 2, 0));

    // Unknown dialects miss loudly.
    assert!(matches!(
        registry.lookup("groot", &floating),
        Err(RegistryError::NoMatch { .. })
    ));
}

#[test]
fn duplicate_registration_is_rejected() {
    let mut registry = Registry::new();
    registry
        .register(Arc::new(LerobotAsyncCodec::new()))
        .unwrap();
    assert!(matches!(
        registry.register(Arc::new(LerobotAsyncCodec::new())),
        Err(RegistryError::Duplicate { .. })
    ));
}
