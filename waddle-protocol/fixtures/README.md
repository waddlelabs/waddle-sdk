# Fixtures

Golden fixtures pin the meaning of the schemas. They are **append-only**:
changing an existing golden IS a breaking change, by definition
(`../docs/VERSIONING.md` §6). If an implementation and a golden disagree, the
implementation is wrong.

## Layout

| Directory | Contents |
|---|---|
| `wire/` | schema-canonical single messages — one fixture per convention or edge worth pinning (per codec dialect × upstream version where relevant) |
| `sidecars/` | complete golden `waddle.v0.Sidecar` records, one per scenario (e.g. a retake pair, a `reset_unverified` episode, an audit-labeled episode, a Reference-mode record) |
| `behaviors/` | behavioral scenarios for conformance tier 2. These do **not** use the message envelope below — they follow `../conformance/scenario-format.md` |

## Envelope format

Every fixture file under `wire/` and `sidecars/` is a single JSON object:

```json
{
  "format": "waddle_sdk.fixture/v0",
  "type": "waddle.v0.ActionChunk",
  "description": "what this fixture pins and why it exists",
  "message": { }
}
```

- `format` — MUST be exactly `"waddle_sdk.fixture/v0"`.
- `type` — the fully-qualified message name; MUST name a message in package
  `waddle.v0`. Files in `sidecars/` always carry `"waddle.v0.Sidecar"`.
- `description` — required; states what the fixture pins. A deprecated golden
  (superseded per `../docs/VERSIONING.md` §6) says so here; the file itself
  is never edited otherwise, and never deleted.
- `message` — the golden message in canonical proto3 JSON.

## Canonical proto3 JSON (normative for fixture authoring)

- Field names in **lowerCamelCase**: `tStartNs`, never `t_start_ns`.
- `int64`/`uint64` values — all `_ns`/`_unix_ns`/`_client_ns` times,
  durations, `seq` — as **decimal strings**: `"1234567890"`.
- Enums as **full prefixed names**: `"TERMINAL_OUTCOME_SUCCESS"`, never bare
  `"SUCCESS"`, never integers.
- `oneof`: exactly the set arm appears — never more than one arm; an absent
  oneof means unset.
- Fields at their proto3 default are **omitted** (no emit-defaults).
- `bytes` as standard base64.
- Doubles in shortest round-trip decimal; authors MUST write values that
  parse back bit-exact.

## Comparison is semantic, never textual

A conforming check parses the fixture's `message` into a typed message via
the schema and compares **message to message**: field-by-field equality in
which JSON key order, whitespace, and numeric formatting are irrelevant.
Comparing JSON text or bytes is non-conforming. A fixture that fails to parse
against its declared `type` — including any unknown field name — fails the
suite outright.

## Naming

`snake_case`, `<message_or_scenario>_<variant>.json`. Examples:

- `wire/action_chunk_ee_delta.json` — pins wxyz quaternion order and delta
  composition; referenced normatively by the `Quat` comment in
  `../proto/waddle/v0/descriptors.proto`.
- `sidecars/sidecar_retake_pair.json` — an `ABORTED_RETAKE` predecessor and
  its born-claimed successor, linked by `RetakeLink`.
- `behaviors/lease_handoff_hold_first.json` — a scenario, per
  `../conformance/scenario-format.md`.

## Adding fixtures

New fixtures are non-breaking and welcome; new FSM or gate behavior REQUIRES
one, together with the normative text it pins in `../docs/FSM.md` — a
guard-table row when a transition or guard moves, the governing section's
prose when none does (intake validation, dispatch shape, and the
blend/gripper/part contracts of §4–§5 are prose, not rows). See
`../docs/VERSIONING.md` §6. A fixture the document never names pins a
behavior the standard does not claim.

Runners MUST enumerate these directories at test time rather than keep a
fixture list by hand: append-only directories plus a hand-kept list is how
coverage silently rots.
