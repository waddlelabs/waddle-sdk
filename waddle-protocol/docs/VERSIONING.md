# VERSIONING — evolution rules for `waddle.v0`

This document is **normative**. It defines what may change inside protocol
package `waddle.v0`, what constitutes a breaking change, and how feature
flags — not release numbers — version the protocol (design rationale §2.7).
Vocabulary per `GLOSSARY.md`.

## 1. Package versioning

The protocol package is **`waddle.v0`**: the protobuf `package` statement and
the directory path `proto/waddle/v0/`. Rules:

- A **breaking change** (§2) MUST ship as a new package, `waddle.v1`, in a new
  directory. `waddle.v0` files are **never edited in place** except by the
  non-breaking additions of §2.
- There are no minor or patch numbers on the wire. Within a package, all
  evolution is additive and gated by **feature flags** (§3).
- `waddle.v0` remains served for as long as any deployed client negotiates it
  (see the serving guarantee in §3). A new package is a parallel surface, not
  a replacement event.

## 2. What is breaking

| Breaking (requires `waddle.v1`) | Non-breaking (allowed within `waddle.v0`) |
|---|---|
| Renumbering, retyping, or reusing any field number or field name | Adding new fields to existing messages (fresh numbers) |
| Reusing an enum value number or name; lifting a `reserved` entry | Adding new enum values (fresh numbers) |
| Any semantic change to an existing FSM transition guard (`docs/FSM.md` guard tables), even with the wire shape unchanged | Adding new `oneof` arms (fresh numbers) |
| Changing an existing golden fixture (§6) | Adding new messages, RPCs, or media-plane data topics behind a **new feature flag** |
| Changing a pinned convention: nanosecond time base and `_ns`/`_unix_ns`/`_client_ns` suffixes, **wxyz** quaternion order, SI units, normative `Composite` part order, canonical fixture JSON form (§7, `fixtures/README.md`) | Adding fixtures, scenarios, and documentation (append-only) |
| Removing a field or enum value outside an explicit feature sunset (§3); the removed entry becomes `reserved` forever (§4) | Reserving the number and name of a field removed via an explicit sunset (§4) |

If a change does not appear in either column, treat it as breaking until a
reviewed PR against this document says otherwise.

## 3. Feature flags

Protocol evolution is negotiated by **named feature flags, never release
numbers** (design rationale §2.7). SDKs pinned inside a robot image for a
year keep working because negotiation is per connection.

Mechanics (normative):

- A feature flag is an opaque string from the registry below. The client
  declares the flags it speaks in `RegisterRequest.feature_flags` and again —
  re-negotiable mid-session — in `NegotiateRequest.feature_flags`. The server
  returns the subset it will serve in
  `RegisterResponse.accepted_feature_flags` /
  `NegotiateResponse.accepted_feature_flags`.
- Behavior gated by a flag MUST NOT be exercised on a connection that did not
  accept that flag. In particular, the control plane MUST NOT plan against or
  emit any message, field, or behavior a connection did not declare: **old
  SDKs never see undeclared features planned against them.**
- The registry is **additive-only within v0**. A published flag never changes
  meaning; new behavior means a new flag. Flag names follow
  `waddle.v0.<area>[.<detail>]`. Adding a registry entry is a reviewed PR to
  this file.

### Initial registry (v0)

| Flag | Gates |
|---|---|
| `waddle.v0.core` | The baseline surface: descriptors, control (actions, verbs, handoff, faults), the episode FSM and `EpisodeEvent` stream, sidecar records, and the `Register`, `Negotiate`, `StreamObservations`, `GateActions`, `ClaimEpisode`, `HandoffLease` RPCs. MUST be declared by every conforming client; a connection without it is refused at `Register`. |
| `waddle.v0.media.teleop` | The media-plane data topics of media.proto: `TeleopStreamPacket`, `ClutchTransition`, `MarkEventPacket`, `OperatorTelemetry`. |
| `waddle.v0.reset` | The reset pipeline: the `RequestReset` RPC, `ResetRequest`, `ResetProgress`, `ResetResult`, and `ResetVerification` with its per-initiator modes (N12). |
| `waddle.v0.audit` | The audit slice (N13): `RecordingModeDeclaration.audit_quota_bp`, `AuditRecord`, and audit-label integrity enforcement (`Judgment.labeler_was_intervenor`). |
| `waddle.v0.heartbeat.proxy-signals` | `ProxySignals` and `VerbMeasurement` riding `HeartbeatPing` (N11). Without it, heartbeats are liveness-only and grant health falls back to safe-window measurement alone. |
| `waddle.v0.reset.phases` | `EPISODE_STATE_POST_RESET`, `PostResetResult` (`EpisodeEvent` arm 16), `StateTransition.outcome` set on a transition to `POST_RESET`, `Sidecar` fields 32–35 (`post_reset_declared`, `post_reset_failed`, `post_reset_result`, `post_reset_bounds`), FSM.md guard rows E14–E18/E14b (§1.3), the `post_reset_result` scenario inject kind, and snapshot paths `episode.post_reset_declared` / `episode.post_reset_failed` / `episode.pinned_outcome`. |
| `waddle.v0.reset.remote` | `GATE_MODE_RESET`, `ResetWindowEvent` (`EpisodeEvent` arm 17), `ResetWindowDirective` (`GateServerMessage` arm 6), `ResetKind`, FSM.md guard rows E19–E22 and C6/C7 (§1.4), the `reset_window_engage` / `reset_window_complete` scenario inject kinds, `NOOP_REASON_RESET_ACTIVE`, and the top-level `reset_window` snapshot document. Independent of and composable with `waddle.v0.reset.phases`: a remote actor may run either the pre-reset, the post-reset, or both. |
| `waddle.v0.plane.acks` | Directive acks (FSM.md §8): `DirectiveAck` (`GateClientMessage` arm 4) and the `directive_id` fields on `ClaimDirective`, `EpisodeDirective`, `ResetWindowDirective`, and `AgentTaskUpdate` (that last one only where the update drives the FSM at all — see §8). An ack is emitted only when the directive carried a `directive_id` AND this flag was negotiated on the connection; fire-and-forget remains valid for planes that don't set ids. Purely observability — no FSM guard row changes; the session accepts and rejects exactly what it did without the flag. |
| `waddle.v0.agent` | Agent-invited episodes: `AgentInviteEvent` (`EpisodeEvent` arm 18; append-only field 3 is generic string-map `task_metadata`), `AgentTaskUpdate` / `AgentTaskUpdateKind` (`GateServerMessage` arm 7), `NOOP_REASON_AGENT_EPISODE`, FSM.md guard rows E23–E26/E26b and C8 (§1.5), the `agent_task_update` scenario inject kind and the `episode_open` `agent_invite` key, snapshot paths `episode.agent_invited` / `episode.agent_engaged`, and the `agent_invite_timeout` timer effects. The invited agent claims, engages, streams chunks, and finishes through the EXISTING intervention machinery (`ClaimDirective`, `intervention_chunk`, `EpisodeDirective`) — this flag adds an invite and a status surface, never new authority. A client MAY declare this flag unconditionally (the reference SDK does: supporting *being* driven costs nothing until a customer asks). The gated behavior is the plane's: a plane that did not accept the flag never routes an invite, so `AgentInviteEvent` — which belongs on the episode's local timeline in either case, since it records that the customer asked — simply goes unanswered and E25's deadline closes the episode with ABORT. Composable with `waddle.v0.plane.acks`: `AgentTaskUpdate.directive_id` is acked exactly where the update decodes into a session event — a DENIED addressed to the ACTIVE episode (E26/E26b) — and is silently ignored elsewhere, since QUEUED, COMPLETED, and a DENIED naming some other episode are recorded without an FSM step to report on. |
| `waddle.v0.obs.stills` | Bounded-rate stills for agent perception on the control plane: `FrameStill` (`ObservationUpdate` payload arm 5) and `StreamPolicy.still_fps` (0/absent = no stills). The rate is bounded by the declared `still_fps`; never a video path — LiveKit media remains the only video transport. Independent of and composable with `waddle.v0.agent`. |
| `waddle.v0.parts` | Part-addressed control on a `Composite` declaration. Two behaviors: (a) honoring `Action.part` (control.proto field 11) at the intervention-chunk intake — a part-scoped step flattens and validates against **that declared part's** own space and dims, and dispatches part-tagged; (b) emitting a non-empty `ProprioSample.part` on the `StreamObservations` uplink. Conformance surface (`conformance/scenario-format.md`): part-carrying steps of the `intervention_chunk` inject kind, the `expect_output.part` and `expect_send.part` matchers (the second because the bypass pump dispatches without passing through `gate()`, so a part-addressed command sent during a stalled caller loop is unassertable without it), and the scope rule a `proprio_sample` inject's `ProprioSample.part` is compared under — a per-part sample answers to that part's own last command, a whole-robot sample answers to nothing while a part-scoped command supersedes part of it. **Declared iff the client's declared `ActionSpace` is `Composite`** — a single-part robot has no addressable parts, and `part == ""` (the sole/default part) is already core. No FSM guard row changes: the execution contract is FSM.md §4 ("move this part, hold the rest") and §5 (a part-scoped action does not cross-fade — the gate holds until the blend window closes), and every refusal reuses the existing once-per-claim-window `Fault{FAULT_KIND_VALIDATION_ERROR}` machinery. **Pre-flag behavior, both directions** (§3 forbids exercising either on a connection that did not accept the flag, so both are defined here rather than left to an implementation): (a) a part-scoped action keeps its pre-flag meaning — flattened against the whole declared space, so on any real multi-part robot the chunk is refused and faulted, deterministically and legibly; (b) the uplink carries only the sole/default sample, the one whose `part` is `""`. Samples for named parts are **withheld from the uplink entirely** — never relabeled `""` to get them through. Relabeling would put one arm's joint vector on the wire as the whole robot's, and would let each part's sample overwrite the others in a plane that keys freshness by part; a plane that sees no per-part proprio is looking at exactly the uplink it saw before this flag existed, which is the outcome §3 asks for. **Why a flag and not a defect fix:** the refusal in (a) is a defined behavior a plane may rely on, and §3's rule is that a plane MUST NOT plan a behavior the connection did not declare; "will execute" versus "will refuse" is exactly the discrimination the flag mechanism exists to make. Local recording is not connection-scoped: the MCAP archive always records `part`, on both flagged and unflagged connections — withholding is an uplink rule, never a recording one, so the archive of a bimanual session is per-part honest regardless of what the connection negotiated. **Uplink cost:** the `StreamObservations` cadence is per part, so a flagged connection carries the declared part count (plus the sole part, which is core) times the unflagged rate — three low-rate summaries per period on a two-part cell rather than one. Bounded by the DECLARATION, which is fixed for the session's lifetime and visible to the plane before it accepts the flag, and each sample is the same small summary it always was: this is the control plane's no-bandwidth rule respected, not excepted, since the bound is declared rather than driven by anything the session does. Media-plane part routing (`PartTarget.part`, `ClutchTransition.part`) is **not** gated here and is unimplemented in v0 — it gets its own detail flag when it lands. |

### Serving guarantee

Every shipped feature is **served indefinitely or explicitly sunset**. Sunset
is a published deprecation with a migration window and a dated removal; after
removal the flag is refused at negotiation (absent from
`accepted_feature_flags`), never silently ignored — a pinned SDK fails
legibly at `Register`/`Negotiate`, not mid-episode.

## 4. Reserved-field policy

A removed field or enum value becomes `reserved` — its **number AND its
name** — forever. Reservations are never lifted, in v0 or in any later
package that carries the message forward.

**Worked exemplar — `FaultKind` 15–17 (control.proto).** The internal fault
taxonomy carried three kinds (`COMMAND_HOLD`, `AUTO_RE_ENABLE`,
`ESTOP_RESUMED`) that are non-fatal state transitions, not faults. The
protocol moved them to `SafetyTransitionKind` (episode.proto), and
control.proto pins the vacated slots:

```protobuf
reserved 15, 16, 17;
reserved "FAULT_KIND_COMMAND_HOLD", "FAULT_KIND_AUTO_RE_ENABLE",
    "FAULT_KIND_ESTOP_RESUMED";
```

Values 1–14 and 18–19 keep their internal numeric identity so internal
adoption is a rename, not a migration. This is the template for every future
removal: relocate the semantics if they still exist, reserve number and name
where they were, never renumber the survivors.

## 5. Enum evolution

- Every enum MUST define `*_UNSPECIFIED = 0`. `UNSPECIFIED` is never a legal
  produced value unless the message's comments explicitly define it (e.g. a
  `Fault` with `FAULT_KIND_UNSPECIFIED` is a bug in the producer, never a
  success signal).
- Readers MUST tolerate unknown enum values: an unrecognized value means
  **"newer peer"**, never a parse error or a crash. On encountering one, a
  reader MUST (a) preserve it through re-serialization where its runtime
  allows, (b) apply the field's documented conservative fallback (an unknown
  `FaultKind` is still a fault; an unknown `TerminalOutcome` counts as
  neither success nor failure in metrics), and (c) never conflate it with
  `UNSPECIFIED`/absent.
- Writers MUST NOT emit enum values that belong to a feature flag the
  connection did not accept (§3).

## 6. Fixture stability

- Golden fixtures (`fixtures/wire/`, `fixtures/sidecars/`) and behavioral
  scenarios (`fixtures/behaviors/`) are **append-only**.
- **Changing an existing golden IS a breaking change, by definition.** The
  fixture is the pinned meaning of the schema: if an implementation and a
  golden disagree, the implementation is wrong.
- New FSM or gate behavior requires, in one change: a guard-table row in
  `docs/FSM.md`, at least one asserting scenario in `fixtures/behaviors/`,
  and a green run of the reference conformance runner
  (`conformance/README.md`).
- A golden discovered to contradict the schemas or normative docs is handled
  by **adding** a superseding fixture and marking the old one deprecated in
  its `description`; the old file is never edited or deleted within v0.

## 7. Time and unit rules (restated)

Normative home: the proto comments (descriptors.proto header) and
`GLOSSARY.md`. Restated here because every rule below is in the "breaking"
column of §2:

- All timestamps and durations are `int64` **nanoseconds**.
- `_ns` — session-monotonic nanoseconds; the single stream timeline.
- `_unix_ns` — wall-clock twin, captured **at stamp time** via the session's
  `ClockAnchor`; never derived after the fact (a host suspend between stamp
  and conversion silently corrupts offsets).
- `_client_ns` — a remote actor's own monotonic clock (media plane); exists
  for jitter monitoring and the echo triple only, and is **never recorded as
  session time**.
- Quaternions are **wxyz** (`Quat`, w first). Implementations with
  xyzw-ordered internals convert in exactly one place at their boundary,
  never on this wire; `fixtures/wire/action_chunk_ee_delta.json` exists to
  catch a transposed conversion.
- SI units, radians, meters; right-handed frames, z-up; an empty `frame_id`
  is a validation error, never a default.

## 8. Codec attestation surface (N15)

- `CodecDescriptor` (descriptors.proto) **is protocol surface** and evolves
  under this document's rules: `name`, `dialect`, `version`,
  `upstream_version`, `content_hash` (sha256 of the codec artifact),
  `signature`, and `signer_key_id`, declared at `Negotiate`. The signature
  covers the artifact bytes whose sha256 is `content_hash`; `signer_key_id`
  identifies the verification key and its registered scheme. Changing this
  attestation shape or its verification semantics is breaking.
- Codecs are version-pinned in configuration — no floating "latest" in the
  write path — and load-time certified in addition to per-session round-trip
  checks. A codec sits between a policy server and a robot: the
  highest-consequence position in the system for a buggy or malicious
  artifact.
- The codec **trait** (the Rust API in `waddle-codecs`) is **not** protocol
  surface. It is versioned independently by `waddle-codecs` (N4) and is
  declared **unstable until two external dialects exist** against it;
  out-of-tree codec authors track `waddle-codecs` releases, not this
  document.
