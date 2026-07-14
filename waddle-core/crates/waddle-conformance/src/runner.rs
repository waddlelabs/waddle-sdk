//! Scenario execution: steps run strictly in order; every expectation is
//! checked with cursor semantics over the emission log; on failure the
//! report carries the failing step index and an expected-vs-actual diff.

use std::path::{Path, PathBuf};

use serde_json::{Map, Value};

use crate::emissions::Codec;
use crate::matching::{MatchCtx, collect_lease_ids, matches};
use crate::scenario::{Scenario, Step, load_scenario};
use crate::target::Target;
use crate::{ConformanceError, scenario_err};

/// The runner report for one scenario (scenario-format.md "Runner report").
#[derive(Debug)]
pub struct Report {
    pub name: String,
    pub pass: bool,
    /// True when the scenario's `requires_features` are not implemented and
    /// the scenario was skipped (skipped counts as not-failed, not passed).
    pub skipped: bool,
    pub failing_step: Option<usize>,
    pub detail: Option<String>,
}

impl Report {
    fn passed(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            pass: true,
            skipped: false,
            failing_step: None,
            detail: None,
        }
    }
}

/// A step outcome: `Err` carries the human-readable diff.
type StepResult = Result<(), String>;

pub fn run_scenario_file(path: &Path) -> Result<Report, ConformanceError> {
    let codec = Codec::new()?;
    let scenario = load_scenario(path, &codec)?;
    run_scenario(&scenario, codec)
}

/// Run every `*.json` scenario in a directory (sorted by file name).
pub fn run_dir(dir: &Path) -> Result<Vec<Report>, ConformanceError> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)?
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .map(|entry| entry.path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "json"))
        .collect();
    paths.sort();
    if paths.is_empty() {
        return Err(scenario_err(format!(
            "no scenarios found in {}",
            dir.display()
        )));
    }
    paths.iter().map(|p| run_scenario_file(p)).collect()
}

pub fn run_scenario(scenario: &Scenario, codec: Codec) -> Result<Report, ConformanceError> {
    if !scenario.features_supported() {
        return Ok(Report {
            name: scenario.name.clone(),
            pass: true,
            skipped: true,
            failing_step: None,
            detail: Some(format!(
                "skipped: requires features {:?}",
                scenario.requires_features
            )),
        });
    }
    let mut target = Target::new(scenario, codec)?;
    // Cursor semantics: `within_ns: "0"` means "already emitted by now,
    // since the last expectation" — each match advances the cursor past the
    // matched emission.
    let mut emission_cursor = 0usize;
    let mut send_cursor = 0usize;

    for (idx, step) in scenario.steps.iter().enumerate() {
        let outcome: StepResult = match step {
            Step::Advance(ns) => {
                target.advance(*ns)?;
                Ok(())
            }
            Step::Inject(payload) => {
                target.inject(payload)?;
                Ok(())
            }
            Step::ExpectState(paths) => check_state(&target, paths),
            Step::ExpectEmission {
                expected,
                within_ns,
            } => check_emission(&mut target, &mut emission_cursor, expected, *within_ns)?,
            Step::ExpectNoEmission {
                expected,
                within_ns,
            } => check_no_emission(&mut target, emission_cursor, expected, *within_ns)?,
            Step::ExpectOutput(expected) => check_output(&target, expected)?,
            Step::ExpectSend {
                expected,
                within_ns,
            } => check_send(&mut target, &mut send_cursor, expected, *within_ns)?,
        };
        if let Err(diff) = outcome {
            return Ok(Report {
                name: scenario.name.clone(),
                pass: false,
                skipped: false,
                failing_step: Some(idx),
                detail: Some(format!("step {idx} at t={}ns: {diff}", target.now)),
            });
        }
    }
    Ok(Report::passed(&scenario.name))
}

// -- expect_state ------------------------------------------------------------

fn check_state(target: &Target, paths: &Map<String, Value>) -> StepResult {
    let snapshot = target.snapshot();
    let ctx = MatchCtx {
        active_claim: target.active_claim_id(),
        prior_lease_ids: &[],
    };
    for (path, expected) in paths {
        let Some(actual) = resolve_path(&snapshot, path) else {
            return Err(format!(
                "expect_state: no such path {path:?} in snapshot\n{}",
                pretty(&snapshot)
            ));
        };
        if !matches(expected, actual, &ctx) {
            return Err(format!(
                "expect_state mismatch at {path:?}\n  expected: {}\n  actual:   {}\n  snapshot: {}",
                pretty(expected),
                pretty(actual),
                pretty(&snapshot)
            ));
        }
    }
    Ok(())
}

/// Resolve a dotted snapshot path, with `grants[VERB_X]` /
/// `grants[VERB_X/SPACE_KIND_Y]` selector support.
fn resolve_path<'v>(doc: &'v Value, path: &str) -> Option<&'v Value> {
    let mut current = doc;
    for segment in path.split('.') {
        if let Some((name, selector)) = parse_selector(segment) {
            let entries = current.get(name)?.as_array()?;
            let (verb, interface) = match selector.split_once('/') {
                Some((v, i)) => (v, Some(i)),
                None => (selector, None),
            };
            current = entries.iter().find(|entry| {
                entry.get("verb").and_then(Value::as_str) == Some(verb)
                    && interface.is_none_or(|iface| {
                        entry
                            .get("send_interfaces")
                            .and_then(Value::as_array)
                            .is_some_and(|list| list.iter().any(|v| v.as_str() == Some(iface)))
                    })
            })?;
        } else {
            current = current.get(segment)?;
        }
    }
    Some(current)
}

fn parse_selector(segment: &str) -> Option<(&str, &str)> {
    let open = segment.find('[')?;
    let close = segment.rfind(']')?;
    if close != segment.len() - 1 || close <= open {
        return None;
    }
    Some((&segment[..open], &segment[open + 1..close]))
}

// -- expect_emission / expect_no_emission ------------------------------------

/// Find the first emission at or after `from` matching `expected`, resolving
/// `$fresh_lease` against the lease ids seen strictly before each candidate.
fn find_emission(target: &Target, from: usize, expected: &Value) -> Option<usize> {
    let mut prior: Vec<String> = Vec::new();
    for entry in &target.emissions[..from.min(target.emissions.len())] {
        collect_lease_ids(&entry.value, &mut prior);
    }
    for idx in from..target.emissions.len() {
        let ctx = MatchCtx {
            active_claim: target.active_claim_id(),
            prior_lease_ids: &prior,
        };
        if matches(expected, &target.emissions[idx].value, &ctx) {
            return Some(idx);
        }
        collect_lease_ids(&target.emissions[idx].value, &mut prior);
    }
    None
}

fn check_emission(
    target: &mut Target,
    cursor: &mut usize,
    expected: &Value,
    within_ns: i64,
) -> Result<StepResult, ConformanceError> {
    if let Some(idx) = find_emission(target, *cursor, expected) {
        *cursor = idx + 1;
        return Ok(Ok(()));
    }
    // Advance virtual time stepwise (firing timers / running the pump)
    // while checking, up to the window end.
    let deadline = target.now.saturating_add(within_ns);
    while target.now < deadline {
        target.step_once(deadline)?;
        if let Some(idx) = find_emission(target, *cursor, expected) {
            *cursor = idx + 1;
            return Ok(Ok(()));
        }
    }
    Ok(Err(format!(
        "expect_emission: no match within {within_ns}ns\n  expected: {}\n  emissions since cursor:\n{}",
        pretty(expected),
        emission_tail(target, *cursor)
    )))
}

fn check_no_emission(
    target: &mut Target,
    cursor: usize,
    expected: &Value,
    within_ns: i64,
) -> Result<StepResult, ConformanceError> {
    // The window is "since the last expectation" (the emission cursor)
    // through the end of the advance — mirroring expect_emission's
    // `within_ns: "0"` semantics. The match cursor is untouched.
    target.advance(within_ns)?;
    if let Some(idx) = find_emission(target, cursor, expected) {
        let entry = &target.emissions[idx];
        return Ok(Err(format!(
            "expect_no_emission violated at t={}ns\n  forbidden: {}\n  matched:   {}",
            entry.at_ns,
            pretty(expected),
            pretty(&entry.value)
        )));
    }
    Ok(Ok(()))
}

fn emission_tail(target: &Target, from: usize) -> String {
    const MAX: usize = 20;
    let entries = &target.emissions[from.min(target.emissions.len())..];
    let mut out = String::new();
    for entry in entries.iter().take(MAX) {
        out.push_str(&format!("    t={}ns {}\n", entry.at_ns, entry.value));
    }
    if entries.len() > MAX {
        out.push_str(&format!("    … {} more\n", entries.len() - MAX));
    }
    if entries.is_empty() {
        out.push_str("    (none)\n");
    }
    out
}

// -- expect_output / expect_send ----------------------------------------------

fn check_output(target: &Target, expected: &Value) -> Result<StepResult, ConformanceError> {
    let Some(actual) = target.last_output_value()? else {
        return Ok(Err(
            "expect_output: no gate_tick has run in this scenario".to_owned()
        ));
    };
    let ctx = MatchCtx {
        active_claim: target.active_claim_id(),
        prior_lease_ids: &[],
    };
    if matches(expected, &actual, &ctx) {
        Ok(Ok(()))
    } else {
        Ok(Err(format!(
            "expect_output mismatch\n  expected: {}\n  actual:   {}",
            pretty(expected),
            pretty(&actual)
        )))
    }
}

fn check_send(
    target: &mut Target,
    cursor: &mut usize,
    expected: &Value,
    within_ns: i64,
) -> Result<StepResult, ConformanceError> {
    let find = |target: &Target, from: usize| -> Option<usize> {
        let ctx = MatchCtx {
            active_claim: target.active_claim_id(),
            prior_lease_ids: &[],
        };
        (from..target.send_log.len())
            .find(|&idx| matches(expected, &target.send_log[idx].value, &ctx))
    };
    if let Some(idx) = find(target, *cursor) {
        *cursor = idx + 1;
        return Ok(Ok(()));
    }
    let deadline = target.now.saturating_add(within_ns);
    while target.now < deadline {
        target.step_once(deadline)?;
        if let Some(idx) = find(target, *cursor) {
            *cursor = idx + 1;
            return Ok(Ok(()));
        }
    }
    let log = target
        .send_log
        .iter()
        .map(|e| format!("    t={}ns {}", e.at_ns, e.value))
        .collect::<Vec<_>>()
        .join("\n");
    Ok(Err(format!(
        "expect_send: no matching send within {within_ns}ns\n  expected: {}\n  send log:\n{}",
        pretty(expected),
        if log.is_empty() {
            "    (none)".to_owned()
        } else {
            log
        }
    )))
}

fn pretty(v: &Value) -> String {
    serde_json::to_string(v).unwrap_or_else(|_| "<unprintable>".to_owned())
}
