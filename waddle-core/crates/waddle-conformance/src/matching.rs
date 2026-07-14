//! Deep partial matching of expected JSON against actual JSON, with the
//! scenario-format matchers (`$any`, `$nonempty`, `$active_claim`,
//! `$fresh_lease`), ordered-subsequence semantics for repeated fields, and
//! canonical-JSON number tolerance (int64-as-string; f64 within 1e-9).

use serde_json::Value;

/// Context the `$…` matchers resolve against.
#[derive(Debug, Clone, Default)]
pub struct MatchCtx<'a> {
    /// The claim id of the currently active claim, if any.
    pub active_claim: Option<&'a str>,
    /// Every lease id seen earlier in this scenario run (strictly before the
    /// candidate emission for `$fresh_lease`).
    pub prior_lease_ids: &'a [String],
}

/// Does `expected` (a partial pattern) match `actual`?
#[must_use]
pub fn matches(expected: &Value, actual: &Value, ctx: &MatchCtx<'_>) -> bool {
    match expected {
        Value::String(s) if s.starts_with('$') => match s.as_str() {
            "$any" => true,
            "$nonempty" => is_nonempty(actual),
            "$active_claim" => ctx
                .active_claim
                .is_some_and(|id| actual.as_str() == Some(id)),
            "$fresh_lease" => actual.as_str().is_some_and(|id| {
                !id.is_empty() && !ctx.prior_lease_ids.iter().any(|seen| seen == id)
            }),
            // Unknown matchers never match (the scenario is wrong, and a
            // loud diff beats a silent pass).
            _ => false,
        },
        Value::Object(exp) => {
            let Value::Object(act) = actual else {
                return false;
            };
            exp.iter()
                .all(|(k, v)| act.get(k).is_some_and(|a| matches(v, a, ctx)))
        }
        Value::Array(exp) => {
            let Value::Array(act) = actual else {
                return false;
            };
            // Repeated fields match as an ordered subsequence.
            let mut cursor = 0usize;
            'outer: for e in exp {
                while cursor < act.len() {
                    let candidate = &act[cursor];
                    cursor += 1;
                    if matches(e, candidate, ctx) {
                        continue 'outer;
                    }
                }
                return false;
            }
            true
        }
        _ => leaf_matches(expected, actual),
    }
}

fn leaf_matches(expected: &Value, actual: &Value) -> bool {
    if expected == actual {
        return true;
    }
    // Canonical proto3 JSON emits int64 as decimal strings; scenario authors
    // may write either spelling. Fall back to a numeric comparison whenever
    // both sides are number-like (f64 tolerance 1e-9).
    if let (Some(e), Some(a)) = (as_number(expected), as_number(actual)) {
        return (e - a).abs() <= 1e-9;
    }
    false
}

fn as_number(v: &Value) -> Option<f64> {
    match v {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.parse::<f64>().ok(),
        _ => None,
    }
}

fn is_nonempty(v: &Value) -> bool {
    match v {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64().is_some_and(|f| f != 0.0),
        Value::String(s) => {
            if s.is_empty() {
                return false;
            }
            // Numeric strings (canonical int64) are "empty" when zero.
            s.parse::<f64>() != Ok(0.0)
        }
        Value::Array(a) => !a.is_empty(),
        Value::Object(o) => !o.is_empty(),
    }
}

/// Collect every `leaseId` string that appears anywhere in `value` — used to
/// build the `$fresh_lease` "seen before" set from the emission log.
pub fn collect_lease_ids(value: &Value, out: &mut Vec<String>) {
    match value {
        Value::Object(map) => {
            for (k, v) in map {
                if k == "leaseId"
                    && let Some(id) = v.as_str()
                    && !id.is_empty()
                    && !out.iter().any(|seen| seen == id)
                {
                    out.push(id.to_owned());
                }
                collect_lease_ids(v, out);
            }
        }
        Value::Array(items) => {
            for v in items {
                collect_lease_ids(v, out);
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn ctx<'a>(claim: Option<&'a str>, leases: &'a [String]) -> MatchCtx<'a> {
        MatchCtx {
            active_claim: claim,
            prior_lease_ids: leases,
        }
    }

    #[test]
    fn partial_object_and_subsequence() {
        let actual = json!({"a": {"b": 1, "c": 2}, "list": [1, 2, 3, 4]});
        assert!(matches(
            &json!({"a": {"c": 2}, "list": [2, 4]}),
            &actual,
            &ctx(None, &[])
        ));
        assert!(!matches(&json!({"list": [4, 2]}), &actual, &ctx(None, &[])));
    }

    #[test]
    fn int64_string_tolerance() {
        assert!(matches(
            &json!("300000000"),
            &json!("300000000"),
            &ctx(None, &[])
        ));
        assert!(matches(
            &json!("300000000"),
            &json!(300000000_i64),
            &ctx(None, &[])
        ));
        assert!(matches(&json!(0.5), &json!(0.5000000001), &ctx(None, &[])));
        assert!(!matches(&json!(0.5), &json!(0.51), &ctx(None, &[])));
    }

    #[test]
    fn dollar_matchers() {
        assert!(matches(&json!("$any"), &json!(""), &ctx(None, &[])));
        assert!(!matches(&json!("$nonempty"), &json!(""), &ctx(None, &[])));
        assert!(matches(&json!("$nonempty"), &json!("x"), &ctx(None, &[])));
        assert!(!matches(&json!("$nonempty"), &json!("0"), &ctx(None, &[])));
        assert!(matches(
            &json!("$active_claim"),
            &json!("claim-1"),
            &ctx(Some("claim-1"), &[])
        ));
        let seen = vec!["lease-1".to_owned()];
        assert!(matches(
            &json!("$fresh_lease"),
            &json!("lease-2"),
            &ctx(None, &seen)
        ));
        assert!(!matches(
            &json!("$fresh_lease"),
            &json!("lease-1"),
            &ctx(None, &seen)
        ));
    }

    #[test]
    fn lease_id_collection() {
        let mut out = Vec::new();
        collect_lease_ids(
            &json!({"event": {"lease": {"lease": {"leaseId": "lease-1"}}}}),
            &mut out,
        );
        assert_eq!(out, vec!["lease-1".to_owned()]);
    }
}
