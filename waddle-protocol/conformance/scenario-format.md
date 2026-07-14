# Behavioral scenario format — `waddle.behavior/v0`

**Normative.** Every file in `fixtures/behaviors/` follows this schema, and every
conformance runner implements exactly this schema. If a runner and this document
disagree, this document wins and the runner is wrong.

Scenarios are deterministic: time is virtual (`advance_ns` steps), lease tokens
and ids are injected, and no step may depend on wall clocks or randomness. A
conforming implementation must produce identical results on every run.

## File shape

```json
{
  "format": "waddle.behavior/v0",
  "name": "snake_case_scenario_name",
  "description": "one-line human description",
  "target": "fsm",
  "requires_features": ["waddle.v0.core"],
  "setup": {
    "robot_fixture": "../wire/robot_description_bimanual_composite.json",
    "lease_enforcement": "LEASE_ENFORCEMENT_ADVISORY",
    "handoff": { "holdFirst": {} },
    "verification_mode": "RESET_VERIFICATION_MODE_BLOCKING"
  },
  "steps": [ ... ]
}
```

- `target` — which harness level runs the scenario:
  - `"fsm"` — the pure session state machine (episode + claim + lease + grant
    health). Effects are observable emissions.
  - `"gate"` — the FSM composed with a gate and a scripted caller loop
    (which may stall) plus a scripted intervention stream.
- `requires_features` — feature flags (see `docs/VERSIONING.md`) the
  implementation must have negotiated for the scenario to apply. Runners skip
  (not fail) scenarios whose flags they do not implement.
- `setup.robot_fixture` — path (relative to the scenario file) to a wire
  fixture whose `message` is a `waddle.v0.RobotDescription`.
- `setup.handoff` — canonical proto3 JSON of a `waddle.v0.HandoffPolicy`.
- `setup.verification_mode` — the mode the first episode's reset runs under.

All protobuf values anywhere in a scenario use canonical proto3 JSON
(lowerCamelCase fields, int64 as decimal strings, enums as full prefixed
names).

## Steps

A step is exactly one of:

### `advance_ns`

```json
{ "advance_ns": "100000000" }
```

Advances virtual time. Timers whose deadlines are reached fire (in deadline
order) before the next step executes.

### `inject`

```json
{ "inject": { "kind": "<kind>", ...payload } }
```

Closed set of kinds (v0):

| kind | payload | targets | meaning |
|---|---|---|---|
| `episode_open` | `episode_id`, `verification_mode?`, `born_claimed?`, `parent_episode_id?` | fsm, gate | open an episode in RESETTING |
| `reset_result` | `result`: `waddle.v0.ResetResult` | fsm, gate | the reset pipeline reported |
| `verification_result` | `verification`: `waddle.v0.ResetVerification` | fsm, gate | a (possibly late/async) reset verification |
| `start` | — | fsm | READY → RUNNING without modeling gate ticks |
| `gate_tick` | `action?`: `waddle.v0.Action`, `obs_t_ns?` | gate | one caller-loop tick through `gate()` |
| `chunk_arrival` | `chunk`: `waddle.v0.ActionChunk` | gate | a policy chunk arrives |
| `teleop_action` | `packet`: `waddle.v0.TeleopStreamPacket` | gate | an intervention-stream action arrives |
| `claim_request` | fields of `waddle.v0.ClaimEpisodeRequest` | fsm, gate | an actor asks to claim |
| `claim_granted` | `claim`: `waddle.v0.Claim` | fsm, gate | the claim was granted |
| `claim_released` | `claim_id` | fsm, gate | the claim was released |
| `engage` | `claim_id`, `initiator`: `waddle.v0.ActorKind` | fsm, gate | intervention engage begins |
| `release` | `claim_id`, `initiator` | fsm, gate | intervenor requests release |
| `retake` | `claim_id`, `initiator`, `successor_episode_id` | fsm, gate | intervenor retakes |
| `clutch` | `engaged`: bool, `part?` | fsm, gate | local-source clutch edge (self-initiated claims) |
| `verb_result` | `result`: `waddle.v0.VerbResult` | fsm, gate | a declared verb completed |
| `estop` | `detail?` | fsm, gate | the stop path fired |
| `terminate` | `outcome`: `waddle.v0.TerminalOutcome`, `reason?` | fsm, gate | integrator/supervisor terminates |
| `judge_result` | `judgment`: `waddle.v0.Judgment` | fsm, gate | an episode judgment arrives |
| `mark` | `mark`: `waddle.v0.MarkEvent` | fsm, gate | a human mark arrives |
| `proxy_signals` | `signals`: `waddle.v0.ProxySignals` | fsm | heartbeat proxy signals sampled |
| `heartbeat_ack` | `ack`: `waddle.v0.HeartbeatAck` | fsm | control plane acked (may carry grant changes) |
| `partition_start` / `partition_end` | — | fsm, gate | control-plane connectivity flips |
| `proprio_sample` | `sample`: `waddle.v0.ProprioSample`, `t_ns` | gate | proprioception (dual-write detection feed) |

Runners MUST reject a scenario containing an unknown kind (this set only grows
by protocol revision).

### `expect_state`

```json
{ "expect_state": { "episode.state": "EPISODE_STATE_RUNNING", "gate.mode": "GATE_MODE_PASSTHROUGH" } }
```

Partial match against the **state snapshot document** (below), keyed by dotted
paths. Present keys must match; absent keys are unconstrained.

### `expect_emission` / `expect_no_emission`

```json
{ "expect_emission": { "event": { "gate": { "to": "GATE_MODE_BYPASS" } }, "within_ns": "0" } }
{ "expect_no_emission": { "event": { "fault": {} }, "within_ns": "500000000" } }
```

- `event` — deep partial match against a `waddle.v0.EpisodeEvent` emitted on
  the session event stream. Repeated fields match as an ordered subsequence.
- `effect` — partial match against an emitted FSM effect (below). Valid on
  both targets: the gate target embeds the session FSM, so its effects are
  observable there too.
- `within_ns` — virtual-time window starting at this step; `"0"` means
  "already emitted by now, since the last expectation". `expect_no_emission`
  advances virtual time by `within_ns` and fails if a matching emission
  occurred in the window.

### `expect_output` (gate target only)

```json
{ "expect_output": { "kind": "noop", "reason": "NOOP_REASON_BYPASS_ACTIVE" } }
```

Asserts on the return of the **most recent** `gate_tick`. `kind` is one of
`pass | substitute | blend | noop | hold`; `provenance` may be asserted as a
partial `waddle.v0.ProvenanceTag`.

### `expect_send` (gate target only)

```json
{ "expect_send": { "provenance": { "kind": "PROVENANCE_KIND_TELEOP" }, "within_ns": "100000000" } }
```

Asserts the implementation invoked the integrator's `send` verb directly
(bypass mode pump), with a partial match on the dispatched chunk's provenance.

## Match values

Strings beginning with `$` are matchers, not literals:

| matcher | meaning |
|---|---|
| `$any` | any value, including empty |
| `$nonempty` | any non-empty value |
| `$active_claim` | the claim_id of the currently active claim |
| `$fresh_lease` | a lease id not equal to any lease id previously seen in this scenario |

## The state snapshot document

`expect_state` paths resolve against this document (canonical proto3 JSON
enum spellings):

```json
{
  "episode": {
    "id": "...",
    "state": "EPISODE_STATE_*",
    "outcome": "TERMINAL_OUTCOME_*",
    "intervention_phase": "INTERVENTION_PHASE_*",
    "born_claimed": false,
    "reset_unverified": false,
    "parent_episode_id": ""
  },
  "gate":  { "mode": "GATE_MODE_*" },
  "lease": { "holder_client_id": "", "lease_id": "", "enforcement": "LEASE_ENFORCEMENT_*" },
  "claim": { "active_claim_id": "", "source_name": "", "self_initiated": false },
  "grants": [ { "verb": "VERB_*", "send_interface": "SPACE_KIND_*", "status": "GRANT_STATUS_*" } ],
  "plane": { "connected": true, "buffered_events": 0 }
}
```

`grants` is matched by `(verb, send_interface)` lookup, expressed as
`grants[VERB_HOLD].status` or `grants[VERB_SEND/SPACE_KIND_EE_POSE_DELTA].status`.

## Effects vocabulary

The pure state machine communicates with its runtime through effects; both
targets expose them as emissions matchable by `expect_emission.effect`:

| effect | fields |
|---|---|
| `set_gate_mode` | `mode` |
| `request_verb` | `verb`, `chunk?` |
| `arm_timer` / `cancel_timer` | `timer_id`, `deadline_ns` |
| `open_successor` | `predecessor_episode_id`, `claim_id`, `born_claimed`, `verification_mode` |
| `mint_lease_token` | `to_client_id` |
| `set_flag` | `flag` (e.g. `"reset_unverified"`) |

## Runner report

For each scenario the runner reports `pass` or, on failure, the failing step
index, the expectation, and a diff of expected-vs-actual (state snapshot or
emission list). A runner MUST run every scenario whose `requires_features` it
satisfies and MUST NOT reorder steps.
