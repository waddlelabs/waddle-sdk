//! Drive every behavioral scenario in `waddle-protocol/fixtures/behaviors/`
//! through the conformance runner. All must pass; skips (unimplemented
//! feature flags) are reported but do not fail.

use std::path::Path;

fn behaviors_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../waddle-protocol/fixtures/behaviors")
}

/// The scenarios this runner does not run yet, by name: each would require a
/// feature flag absent from `waddle_conformance::SUPPORTED_FEATURES`, so the
/// runner would skip it — and a skipped scenario asserts NOTHING. Naming them
/// (rather than only counting the directory) keeps that state honest in both
/// directions. **Empty**: every flag the fixtures use is implemented, so
/// every scenario in the directory runs, and a scenario that stops running
/// for any reason — a `requires_features` typo, a flag renamed in
/// `SUPPORTED_FEATURES`, a new fixture written ahead of its flag — fails here
/// instead of quietly going green with its behavior unchecked. Writing
/// scenarios first stays a commitment rather than a note: the list has to
/// come back to empty before the work is done.
/// Never add a name here to silence a failure: a scenario that runs and
/// fails is a defect, not a pending flag.
const PENDING_FLAG_IMPLEMENTATION: &[&str] = &[];

#[test]
fn all_behavior_scenarios_pass() {
    let reports =
        waddle_conformance::run_dir(&behaviors_dir()).expect("scenario loading must succeed");

    assert_eq!(
        reports.len(),
        42,
        "expected the 42 pinned behavioral scenarios, found {}",
        reports.len()
    );

    let mut skipped: Vec<&str> = reports
        .iter()
        .filter(|r| r.skipped)
        .map(|r| r.name.as_str())
        .collect();
    skipped.sort_unstable();
    assert_eq!(
        skipped, PENDING_FLAG_IMPLEMENTATION,
        "the skipped set must be exactly the scenarios whose feature flag is \
         unimplemented (see PENDING_FLAG_IMPLEMENTATION)"
    );

    let mut failures = Vec::new();
    for report in &reports {
        if report.skipped {
            eprintln!(
                "SKIP {}: {}",
                report.name,
                report.detail.as_deref().unwrap_or("")
            );
            continue;
        }
        if report.pass {
            eprintln!("PASS {}", report.name);
        } else {
            failures.push(format!(
                "FAIL {} (step {}):\n{}",
                report.name,
                report
                    .failing_step
                    .map_or_else(|| "?".to_owned(), |i| i.to_string()),
                report.detail.as_deref().unwrap_or("(no detail)")
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "{} scenario(s) failed:\n\n{}",
        failures.len(),
        failures.join("\n\n")
    );
}

/// Mutate a passing scenario and run it from a temp dir. Guards the runner
/// against vacuous passes: a wrong assertion MUST fail.
fn run_mutated(source: &str, mutate: impl FnOnce(&str) -> String) -> waddle_conformance::Report {
    let text = std::fs::read_to_string(behaviors_dir().join(source)).expect("fixture readable");
    let mutated = mutate(&text);
    assert_ne!(text, mutated, "mutation must change the scenario");
    let dir = tempfile::tempdir().expect("tempdir");
    let path = dir.path().join(source);
    std::fs::write(&path, mutated).expect("write mutated scenario");
    waddle_conformance::run_scenario_file(&path).expect("mutated scenario must still load")
}

#[test]
fn wrong_expected_state_fails() {
    let report = run_mutated("estop_revokes_all_leases.json", |text| {
        text.replace(
            "\"episode.outcome\": \"TERMINAL_OUTCOME_ABORT\"",
            "\"episode.outcome\": \"TERMINAL_OUTCOME_SUCCESS\"",
        )
    });
    assert!(!report.pass, "a wrong expected outcome must fail");
    assert!(report.failing_step.is_some());
}

#[test]
fn emission_order_is_enforced() {
    // The implementation emits state{→TERMINAL} before claim RELEASED; a
    // scenario pinning the reverse order must fail (cursor semantics).
    let report = run_mutated("estop_revokes_all_leases.json", |text| {
        text.replace(
            "{ \"expect_emission\": { \"event\": { \"state\": { \"to\": \"EPISODE_STATE_TERMINAL\", \"outcome\": \"TERMINAL_OUTCOME_ABORT\" } }, \"within_ns\": \"0\" } },",
            "{ \"expect_emission\": { \"event\": { \"claim\": { \"kind\": \"CLAIM_EVENT_KIND_RELEASED\" } }, \"within_ns\": \"0\" } },\n    { \"expect_emission\": { \"event\": { \"state\": { \"to\": \"EPISODE_STATE_TERMINAL\" } }, \"within_ns\": \"0\" } },",
        )
    });
    assert!(!report.pass, "out-of-order emission expectations must fail");
}

#[test]
fn expect_no_emission_catches_matches() {
    // Forbidding the fault event that estop definitely emits must fail.
    let report = run_mutated("estop_revokes_all_leases.json", |text| {
        text.replace(
            "{ \"inject\": { \"kind\": \"estop\", \"detail\": \"site estop chain pressed\" } },",
            "{ \"expect_no_emission\": { \"event\": { \"fault\": {} }, \"within_ns\": \"100000000\" } },\n    { \"inject\": { \"kind\": \"estop\", \"detail\": \"site estop chain pressed\" } },\n    { \"expect_no_emission\": { \"event\": { \"fault\": {} }, \"within_ns\": \"100000000\" } },",
        )
    });
    assert!(!report.pass, "a forbidden emission in the window must fail");
}
