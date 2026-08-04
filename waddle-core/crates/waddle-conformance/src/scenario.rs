//! Serde model of the `waddle.behavior/v0` scenario file plus the
//! wire-fixture envelope loader for `setup.robot_fixture`.

use std::path::Path;

use serde::Deserialize;
use serde_json::{Map, Value};
use waddle_types::{HandoffPolicy, LeaseEnforcement, ResetVerificationMode, pb::v0 as pb};

use crate::emissions::Codec;
use crate::{ConformanceError, scenario_err};

pub const FORMAT: &str = "waddle.behavior/v0";
/// Feature flags this runner implements; scenarios requiring anything else
/// are skipped, not failed (scenario-format.md).
pub const SUPPORTED_FEATURES: &[&str] = &[
    "waddle.v0.core",
    "waddle.v0.reset.phases",
    "waddle.v0.reset.remote",
    "waddle.v0.agent",
    "waddle.v0.parts",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetKind {
    Fsm,
    Gate,
}

#[derive(Debug)]
pub struct Setup {
    /// Validated robot declaration, when the scenario references one.
    pub robot: Option<waddle_types::RobotDescription>,
    pub enforcement: LeaseEnforcement,
    pub handoff: HandoffPolicy,
    /// The mode the first episode's reset runs under (and the default for
    /// `episode_open` injects that omit one).
    pub verification: ResetVerificationMode,
}

/// One scenario step (scenario-format.md "Steps").
#[derive(Debug)]
pub enum Step {
    Advance(i64),
    Inject(Map<String, Value>),
    ExpectState(Map<String, Value>),
    ExpectEmission { expected: Value, within_ns: i64 },
    ExpectNoEmission { expected: Value, within_ns: i64 },
    ExpectOutput(Value),
    ExpectSend { expected: Value, within_ns: i64 },
}

#[derive(Debug)]
pub struct Scenario {
    pub name: String,
    pub description: String,
    pub target: TargetKind,
    pub requires_features: Vec<String>,
    pub setup: Setup,
    pub steps: Vec<Step>,
}

impl Scenario {
    #[must_use]
    pub fn features_supported(&self) -> bool {
        self.requires_features
            .iter()
            .all(|f| SUPPORTED_FEATURES.contains(&f.as_str()))
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawScenario {
    format: String,
    name: String,
    #[serde(default)]
    description: String,
    target: String,
    #[serde(default)]
    requires_features: Vec<String>,
    setup: RawSetup,
    steps: Vec<Map<String, Value>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawSetup {
    #[serde(default)]
    robot_fixture: Option<String>,
    #[serde(default)]
    lease_enforcement: Option<String>,
    handoff: Value,
    #[serde(default)]
    verification_mode: Option<String>,
}

/// The wire-fixture envelope (`fixtures/wire/*.json`).
#[derive(Debug, Deserialize)]
struct WireFixture {
    format: String,
    #[serde(rename = "type")]
    message_type: String,
    #[serde(default)]
    #[allow(dead_code)]
    description: String,
    message: Value,
}

pub fn load_scenario(path: &Path, codec: &Codec) -> Result<Scenario, ConformanceError> {
    let text = std::fs::read_to_string(path)?;
    let raw: RawScenario = serde_json::from_str(&text)?;
    if raw.format != FORMAT {
        return Err(scenario_err(format!(
            "{}: unsupported format {:?}",
            path.display(),
            raw.format
        )));
    }
    let target = match raw.target.as_str() {
        "fsm" => TargetKind::Fsm,
        "gate" => TargetKind::Gate,
        other => return Err(scenario_err(format!("unknown target {other:?}"))),
    };

    let robot = raw
        .setup
        .robot_fixture
        .as_deref()
        .map(|rel| {
            let base = path.parent().unwrap_or_else(|| Path::new("."));
            load_robot_fixture(&base.join(rel), codec)
        })
        .transpose()?;

    let enforcement = match raw.setup.lease_enforcement.as_deref() {
        None => LeaseEnforcement::Enforced,
        Some(s) => {
            let value = pb::LeaseEnforcement::from_str_name(s)
                .ok_or_else(|| scenario_err(format!("unknown lease_enforcement {s:?}")))?;
            LeaseEnforcement::from_pb(value as i32)?
        }
    };

    let handoff_pb: pb::HandoffPolicy =
        codec.parse("waddle.v0.HandoffPolicy", &raw.setup.handoff)?;
    let handoff = HandoffPolicy::from_pb(&handoff_pb)?;

    let verification = match raw.setup.verification_mode.as_deref() {
        None => ResetVerificationMode::Blocking,
        Some(s) => parse_verification_mode(s)?,
    };

    let steps = raw
        .steps
        .iter()
        .enumerate()
        .map(|(i, s)| parse_step(s).map_err(|e| scenario_err(format!("step {i}: {e}"))))
        .collect::<Result<Vec<_>, _>>()?;

    Ok(Scenario {
        name: raw.name,
        description: raw.description,
        target,
        requires_features: raw.requires_features,
        setup: Setup {
            robot,
            enforcement,
            handoff,
            verification,
        },
        steps,
    })
}

pub fn parse_verification_mode(s: &str) -> Result<ResetVerificationMode, ConformanceError> {
    let value = pb::ResetVerificationMode::from_str_name(s)
        .ok_or_else(|| scenario_err(format!("unknown verification mode {s:?}")))?;
    Ok(ResetVerificationMode::from_pb(value as i32)?)
}

fn load_robot_fixture(
    path: &Path,
    codec: &Codec,
) -> Result<waddle_types::RobotDescription, ConformanceError> {
    let text = std::fs::read_to_string(path)?;
    let fixture: WireFixture = serde_json::from_str(&text)?;
    if fixture.format != "waddle.fixture/v0" {
        return Err(scenario_err(format!(
            "{}: unsupported wire-fixture format {:?}",
            path.display(),
            fixture.format
        )));
    }
    if fixture.message_type != "waddle.v0.RobotDescription" {
        return Err(scenario_err(format!(
            "{}: robot_fixture must carry a waddle.v0.RobotDescription, got {}",
            path.display(),
            fixture.message_type
        )));
    }
    let pb_robot: pb::RobotDescription =
        codec.parse("waddle.v0.RobotDescription", &fixture.message)?;
    Ok(waddle_types::RobotDescription::try_from(&pb_robot)?)
}

fn parse_step(raw: &Map<String, Value>) -> Result<Step, ConformanceError> {
    if raw.len() != 1 {
        return Err(scenario_err(format!(
            "a step must have exactly one key, got {:?}",
            raw.keys().collect::<Vec<_>>()
        )));
    }
    let (key, value) = raw.iter().next().expect("len checked");
    match key.as_str() {
        "advance_ns" => Ok(Step::Advance(parse_ns(value)?)),
        "inject" => {
            let map = value
                .as_object()
                .ok_or_else(|| scenario_err("inject payload must be an object"))?;
            if !map.contains_key("kind") {
                return Err(scenario_err("inject payload missing \"kind\""));
            }
            Ok(Step::Inject(map.clone()))
        }
        "expect_state" => {
            let map = value
                .as_object()
                .ok_or_else(|| scenario_err("expect_state must be an object"))?;
            Ok(Step::ExpectState(map.clone()))
        }
        "expect_emission" | "expect_no_emission" => {
            let (expected, within_ns) = split_expectation(value, &["event", "effect"])?;
            if key == "expect_emission" {
                Ok(Step::ExpectEmission {
                    expected,
                    within_ns,
                })
            } else {
                Ok(Step::ExpectNoEmission {
                    expected,
                    within_ns,
                })
            }
        }
        "expect_output" => Ok(Step::ExpectOutput(value.clone())),
        "expect_send" => {
            let (expected, within_ns) = split_expectation(value, &["provenance"])?;
            Ok(Step::ExpectSend {
                expected,
                within_ns,
            })
        }
        other => Err(scenario_err(format!("unknown step kind {other:?}"))),
    }
}

/// Split `{"event"/"effect"/…: {...}, "within_ns": "0"}` into the match
/// pattern (with `within_ns` removed) and the window.
fn split_expectation(value: &Value, allowed: &[&str]) -> Result<(Value, i64), ConformanceError> {
    let map = value
        .as_object()
        .ok_or_else(|| scenario_err("expectation must be an object"))?;
    let within_ns = match map.get("within_ns") {
        Some(v) => parse_ns(v)?,
        None => 0,
    };
    let mut pattern = Map::new();
    for (k, v) in map {
        if k == "within_ns" {
            continue;
        }
        if !allowed.contains(&k.as_str()) {
            return Err(scenario_err(format!("unexpected expectation key {k:?}")));
        }
        pattern.insert(k.clone(), v.clone());
    }
    if pattern.is_empty() {
        return Err(scenario_err(format!(
            "expectation must contain one of {allowed:?}"
        )));
    }
    Ok((Value::Object(pattern), within_ns))
}

/// Nanosecond values are canonically decimal strings but tolerate numbers.
pub fn parse_ns(value: &Value) -> Result<i64, ConformanceError> {
    match value {
        Value::String(s) => s
            .parse::<i64>()
            .map_err(|_| scenario_err(format!("bad ns value {s:?}"))),
        Value::Number(n) => n
            .as_i64()
            .ok_or_else(|| scenario_err(format!("bad ns value {n}"))),
        other => Err(scenario_err(format!("bad ns value {other}"))),
    }
}
