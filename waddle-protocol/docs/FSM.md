# FSM.md — state machines of the Waddle protocol

**Normative.** This document specifies the episode state machine, the claim
lifecycle, the lease machine, the intervention lifecycle, the chunk-handoff
sub-protocol, gate modes, and grant liveness. Each guard-table row is pinned
by at least one behavioral fixture in `../fixtures/behaviors/` (rightmost
column). Proto types referenced are from `proto/waddle/v0/`.

Two machines that prose often conflates are specified separately here, on
purpose: the **claim** (orchestration: who is assigned the episode) and the
**lease** (actuation: who may write to the robot). Winning a claim leads to
acquiring the lease; takeover and release are lease handoffs under an
existing claim.

---

## 1. Episode FSM

```
                       ┌──────────────────────────────────────────────┐
                       │                                              │
                       ▼                                              │
   RESETTING ──► READY ──► RUNNING ──► (INTERVENTION ⇄ RUNNING) ──► TERMINAL
       ▲                                     │                        ▲
       │                          engage → settle → release          │
       │                                     │                        │
       └──────── retake opens successor ◄────┴── retake ─────────────┘
                 (born claimed, claim held)      (this episode:
                                                  ABORTED_RETAKE)
```

States: `EpisodeState` — RESETTING, READY, RUNNING, INTERVENTION, TERMINAL.
Terminal outcomes: `TerminalOutcome` — SUCCESS, FAILURE, ABORT,
ABORTED_RETAKE. A sixth state, POST_RESET, exists only under flag
`waddle.v0.reset.phases` (§1.3); the diagram and guard table above are the
complete picture for connections that do not declare it.

v0 scope: **one active episode per session** (N18). TERMINAL is absorbing:
every event delivered to a terminal episode is rejected without a state
change.

### 1.1 Transition guards

| # | From | Trigger | Guard | To | Effects / emissions | Fixture |
|---|---|---|---|---|---|---|
| E1 | (open) | `episode_open` | no other episode active in session | RESETTING | `EpisodeEvent.state{→RESETTING}`; reset pipeline engaged | all |
| E2 | RESETTING | `reset_result{ok}` | mode = BLOCKING and `verification.verified` | READY | `state{→READY}`; `reset_verification` event | `retake_autonomous_blocking` |
| E3 | RESETTING | `reset_result{ok}` | mode = OPTIMISTIC_ASYNC (verification pending) | READY | `state{→READY}`; verification continues async | `retake_operator_optimistic` |
| E4 | RESETTING | `reset_result{ok}` + `verification{verified: false}` | mode = BLOCKING | RESETTING | stay; next `ResetStrategy` in priority order; `reset_verification{verified:false}` event | `reset_verification_failure` |
| E5 | RESETTING | reset strategies exhausted or reset aborted | — | TERMINAL{ABORT} | `state{→TERMINAL, ABORT}` | `reset_verification_failure` |
| E6 | READY | first gated action (`gate_tick`) or `start` | — | RUNNING | `state{→RUNNING}` | all |
| E7 | RUNNING | `engage{claim}` | claim GRANTED for this episode | INTERVENTION (phase ENGAGE) | handoff sub-protocol (§5) per declared `HandoffPolicy`; `intervention{ENGAGE}` | `handoff_*` |
| E8 | INTERVENTION | `release` | phase = SETTLE | RUNNING | reverse handoff; policy re-primed on fresh observations before un-claim; `intervention{RELEASE}`; `state{→RUNNING}` | `handoff_immediate_mid_chunk` |
| E9 | INTERVENTION | `retake` | phase = SETTLE (or ENGAGE after settle timeout) | TERMINAL{ABORTED_RETAKE} | `intervention{RETAKE}`; `state{→TERMINAL, ABORTED_RETAKE}`; effect `open_successor{claim held, born_claimed}` | `retake_*` |
| E10 | RUNNING or INTERVENTION | `judge_result` (done), `mark{END_*}`, `terminate`, or timeout timer | — | TERMINAL{outcome} | `state{→TERMINAL, outcome}`; async judging attaches labels later | `lease_denied_non_holder` |
| E11 | any non-TERMINAL | `estop` | — | TERMINAL{ABORT} | lease `RevokeAll`; `Fault{ESTOP}`; `state{→TERMINAL, ABORT}` | `estop_revokes_all_leases` |
| E12 | TERMINAL | any event | — | TERMINAL (rejected, unchanged) | none | proptest: terminal absorbing |
| E13 | any | late `verification_result{verified:false, invalidated_async}` | episode ran under OPTIMISTIC_ASYNC | unchanged | effect `set_flag{reset_unverified}` (PERMANENT); `reset_verification{invalidated_async}` event | `retake_operator_optimistic` |

### 1.2 Retake successors (N2, N12, N18)

`open_successor` opens a new episode that:

- carries `born_claimed = true` and `parent_episode_id` (its sidecar gets
  `RetakeLink` and `METRICS_CLASS_BORN_CLAIMED`);
- starts in RESETTING **under the still-held claim**;
- runs verification per initiator: a teleoperator retake gets
  OPTIMISTIC_ASYNC (the judge scores the reset from the live media stream
  during final settle; a late failure retro-flags `reset_unverified`); an
  autonomous retake gets BLOCKING.

The predecessor's outcome is always ABORTED_RETAKE and is never silently
folded into success-rate denominators.

### 1.3 Post-reset (flag `waddle.v0.reset.phases`)

Preface (normative): rows E14–E18 and E14b exist only when
`waddle.v0.reset.phases` is negotiated **and** the episode declares the
feature. An episode with no post-reset declared behaves exactly per
E1–E13; `EPISODE_STATE_POST_RESET` does not exist on undeclared connections
(pinned by `post_reset_skipped_when_undeclared`). E17 is the POST_RESET
specialization of E11 (a state that predates no existing row — the more
specific row governs; E11's behavior on every other pre-existing state is
unchanged).

| # | From | Trigger | Guard | To | Effects / emissions | Fixture |
|---|---|---|---|---|---|---|
| E14 | RUNNING or INTERVENTION | the E10 trigger set (judge done, mark END_*, terminate, timeout) | post-reset declared ∧ not yet entered | POST_RESET{outcome pinned} | `state{→POST_RESET, outcome}` (outcome final+immutable); claim released + gate→PASSTHROUGH exactly as E10 would; engage timers cancelled; post-reset pipeline engaged (hook or window) | `post_reset_happy`, `post_reset_from_intervention` |
| E15 | POST_RESET | `post_reset_result{ok}` | — | TERMINAL{pinned} | `post_reset{result, pinned_outcome}`; lease handback to loop client completes BEFORE the terminal transition when a reset claimant holds it; `state{→TERMINAL, pinned}` | `post_reset_happy` |
| E16 | POST_RESET | `post_reset_result{!ok}` / strategies exhausted / window timeout | — | TERMINAL{pinned, UNCHANGED} | `set_flag{post_reset_failed}` (PERMANENT); `post_reset{ok:false}`; `Fault` (kind: validation-appropriate, see impl); `state{→TERMINAL, pinned}` | `post_reset_failure_flags` |
| E17 | POST_RESET | `estop` | — | TERMINAL{pinned, UNCHANGED} | lease `RevokeAll`; `Fault{ESTOP}`; `set_flag{post_reset_failed}`; `reset_window{CANCELLED}` if window open; cancel `reset_window_timeout`; `state{→TERMINAL, pinned}` | `estop_during_post_reset` |
| E18 | INTERVENTION | `retake` | (E9 guard), post-reset declared or not | TERMINAL{ABORTED_RETAKE} | exactly E9 — retake BYPASSES POST_RESET (the successor's pre-reset handles the scene; E9's `release_claim=false` keeps claim+lease with the intervenor) | `retake_skips_post_reset` |
| E14b | POST_RESET | `terminate` / `mark{END_*}` / judge done / `retake` | — | POST_RESET (rejected) | none — outcome pinned; a late mark is recorded as a `mark` event only, never a transition | `post_reset_happy` |

Rationale E17: E11 verbatim ("any non-TERMINAL → TERMINAL{ABORT}") would
flip an earned SUCCESS to ABORT because cleanup was estopped, corrupting SR
denominators. E17 keeps every safety effect (RevokeAll, `Fault{ESTOP}`,
immediate TERMINAL) and preserves the pinned outcome; `post_reset_failed`
makes the incident permanent.

### 1.4 Remote reset windows (flag `waddle.v0.reset.remote`)

Preface (normative): rows E19–E22 exist only when `waddle.v0.reset.remote`
is negotiated **and** the reset in question (pre or post) declares a remote
actor. A remote reset window is a bounded period during which a claimed
teleoperator, site operator, or agent performs the reset directly through
the SDK; the claim granted for this purpose is the same `Claim`/`Lease`
machinery as everywhere else in this document, reused unchanged (rows C6/C7
in §2 document the claim side).

| # | From | Trigger | Guard | To | Effects / emissions | Fixture |
|---|---|---|---|---|---|---|
| E19 | entry to RESETTING (E1) or POST_RESET (E14) | — | that reset declared remote | unchanged | `reset_window{OPENED, kind, prompt, expected_actor}`; `arm_timer{reset_window_timeout}` | `remote_pre_reset_claim_engage_complete` |
| E20 | RESETTING or POST_RESET | `reset_window_engage{claim}` | claim GRANTED per C6 | unchanged | lease → claimant (L1 if vacant, else L6 handoff, fresh token); on lease applied: gate→RESET (`GateModeChange{→RESET}`), `reset_window{ENGAGED, claim_id}`. Actuation authorization = held lease + gate RESET; the SDK pump drives `send`; caller ticks get `NoopMarker{RESET_ACTIVE}` | `remote_pre_reset_claim_engage_complete`, `remote_reset_caller_tick_noop` |
| E21 | RESETTING or POST_RESET | `reset_window_complete{claim, result}` | claim active per C6 | (deferred) | `reset_window{COMPLETED, result}`; cancel timer; lease handback (L6); AFTER handback applies: gate→PASSTHROUGH, `claim{RELEASED}` (C7), then the result applies as if from the pipeline (E2–E5 in RESETTING; E15/E16 in POST_RESET) | `remote_pre_reset_claim_engage_complete` |
| E22 | RESETTING or POST_RESET | `timer{reset_window_timeout}` | window not COMPLETED | pre: TERMINAL{ABORT} (E5); post: TERMINAL{pinned} + `set_flag{post_reset_failed}` (E16) | `reset_window{TIMED_OUT}`; claim released; lease handback | `remote_post_reset_timeout` |
| E19b | RESETTING or POST_RESET | `reset_result` (pre) / `post_reset_result` (post) | a reset window is open (E19) | unchanged (rejected) | none — an open window owns this reset exclusively; the pipeline-hook completion path (E2–E5 / E15–E16) is illegal until the window closes (E21/E22). Not reachable through a config-correct runtime (a reset phase's config declares either a hook pipeline or a remote window, never both, and the runtime never injects a hook result while a remote window is declared) — guarded here anyway per the hollow-frontend rule (waddle-fsm is the sole enforcer; a future caller/runtime bug must not silently abandon the window/claim/lease) | `remote_window_owns_pipeline_result` |

Pinned in prose: `engage` stays E7/RUNNING-only (a reset claimant never
enters INTERVENTION; `InterventionPhase` is untouched). `clutch` during
reset windows stays a recorded edge, not a claim (current behavior,
unchanged). E13 (late verification) applies unchanged in POST_RESET.

Engage atomicity (E20/E21 interaction, normative): E20's lease routing is
asynchronous — between `reset_window_engage`'s acceptance and the minted
lease applying, the engage is *in flight* and the window has not observably
ENGAGED. A `reset_window_complete` arriving in that interval (a plane
sending ENGAGE and COMPLETE back-to-back; a CANCEL, which decodes to the
same event) is **rejected**, not honored: a window that never observably
ENGAGED has nothing to honorably complete, and honoring it would close the
window and release the reset claim underneath the in-flight lease
operation. The plane retries after it observes `reset_window{ENGAGED}`.
Symmetrically, a minted engage lease whose reset claim (or window) is gone
by the time it applies — e.g. a legal `claim_released` raced the mint — is
discarded: the lease does not move, a `lease{DENIED}` records the stale
mint, and the still-open window remains serviceable by a fresh claim (C6).
This pins the atomicity of E20's engage; it adds no new states or rows.

### 1.5 Agent-invited episodes (flag `waddle.v0.agent`)

Preface (normative): rows E23–E26/E26b and C8 exist only when
`waddle.v0.agent` is negotiated **and** the episode was opened agent-invited
(the customer asked Waddle to drive; the open carries
`agent_invite{prompt, timeout_ns}`). An agent-invited episode is otherwise a
NORMAL episode: E7 engage, intervention chunks, E10 termination, and the
reset phases all apply verbatim. The two terminating triggers this section
adds — E25's invite timeout and E26's pre-engage DENIED — are **members of
E10's trigger set** with a fixed outcome (ABORT); with `post_reset`
declared, E14 therefore governs them from RUNNING exactly as it governs
every other E10 trigger, so they terminate through the episode's normal
termination routing (TERMINAL{ABORT} per E10, POST_RESET{ABORT pinned} per
E14), never around it. The invited agent claims with the same
`Claim`/`Lease` machinery as every other actor — C8 restricts admission,
nothing else. Exactly two things differ: the caller's own `gate()` ticks
never dispatch while no claim is engaged (E24), and only `ACTOR_KIND_AGENT`
claims are admitted (C8). C8 constrains only the grants C1 governs: a claim
landing in RESETTING or POST_RESET with a window OPEN is a **reset** claim
and stays C6's business, so an agent-invited episode with a declared
TELEOPERATOR reset window admits its teleoperator exactly as any other
episode does (`agent_invite_denied_in_post_reset` pins this).

The **invite is open** from E23 until the first of: an agent claim ENGAGEs
(E7 on this episode), or the episode leaves {RESETTING, READY, RUNNING} by
any row (E5, E10, E11, E14, E25, E26; transitions within that set — E2–E4,
E6 — do not close it). On an agent-invited episode, every row that closes
the invite carries `cancel_timer{agent_invite_timeout}` (if still armed) as
an additional effect: E7 as below, and E5/E10/E11/E14/E25/E26 alike. A
`timer{agent_invite_timeout}` delivered after the invite has closed (an
implementation's expiry racing the cancellation) is **discarded** — no
transition, no event; a stale expiry can never abort a pinned outcome.

E7 on an agent-invited episode additionally emits
`cancel_timer{agent_invite_timeout}` and latches `episode.agent_engaged`
(true from the first agent ENGAGE onward; it never resets within the
episode — a release/re-engage cycle does not re-arm the invite timer).

E25 and E26 — and ONLY those two rows — latch `episode.invite_aborted`
before routing to termination: it marks a close produced by the invite
machinery itself (deadline elapsed, or DENIED while open), a legitimate
agent outcome. Every other close of an agent-invited episode (E5 reset
failure, E10 triggers, E11 estop, E14) leaves it false, so an embedder's
blocking ask-an-agent call can distinguish "the ask was declined or went
unanswered" from "the episode broke for unrelated reasons" without parsing
termination reasons. Emission-invisible state, like `agent_engaged`.

E24's Noop plan is **derived state scoped to the episode it was derived
for**: it holds while THAT episode is in {RESETTING, READY, RUNNING,
INTERVENTION} with no engaged claim, and ends the instant the episode leaves
that set — including into POST_RESET, where the caller drives the cleanup.
A successor episode that was not opened agent-invited dispatches its
caller's ticks normally, and that holds for a born-claimed retake successor
(C5) too: the surviving claim carries the predecessor's gate arrangement
across the boundary, never its invite (`agent_invite_retake_successor`).
An implementation that caches the plan MUST re-derive it whenever the
episode state behind these conditions moves, not only when the gate mode
does.

| # | From | Trigger | Guard | To | Effects / emissions | Fixture |
|---|---|---|---|---|---|---|
| E23 | (open) | `episode_open{agent_invite}` | E1 guard (no other episode active in session) | RESETTING | E1's effect set, plus emission `agent_invite{prompt, timeout_ns}` and `arm_timer{agent_invite_timeout, deadline = open + timeout_ns}` | `agent_invite_happy` |
| E24 | RESETTING, READY, RUNNING, or INTERVENTION | `gate_tick` | episode agent-invited ∧ no engaged claim (gate mode PASSTHROUGH; INTERVENTION with the gate still PASSTHROUGH is the *engage window* — the handoff is in flight and nothing is engaged yet) | unchanged | the caller's action NEVER dispatches: the gate plan is Noop with reason `NOOP_REASON_AGENT_EPISODE` (no fault, no state change). With an engaged claim, ordinary intervention semantics apply unchanged — substitution flows through `gate()` as ever. POST_RESET and TERMINAL are outside this row: the run is over, and its cleanup (or the successor) is the caller's to drive | `agent_caller_tick_noop`, `agent_invite_retake_successor` |
| E25 | RESETTING, READY, or RUNNING | `timer{agent_invite_timeout}` | invite open (no agent claim has ENGAGEd; E7 cancels this timer) | from RESETTING or READY: TERMINAL{ABORT}; from RUNNING: TERMINAL{ABORT} per E10, or POST_RESET{ABORT pinned} per E14 when post-reset declared ∧ not yet entered | termination carries detail "no agent engaged"; the taken route's effect set applies verbatim (E10's, or E14's with the outcome pinned to ABORT) | `agent_invite_timeout`, `agent_invite_timeout_post_reset` |
| E26 | RESETTING, READY, or RUNNING | `agent_update{DENIED}` | invite open (no agent claim has ENGAGEd) | routes exactly as E25: from RESETTING or READY TERMINAL{ABORT}; from RUNNING per E10, or per E14 when post-reset declared ∧ not yet entered | `cancel_timer{agent_invite_timeout}`; termination carries the update's detail; the taken route's effect set applies verbatim | `agent_invite_denied` |
| E26b | any non-TERMINAL | `agent_update{DENIED}` | invite not open (an agent claim has ENGAGEd, or the episode already left {RESETTING, READY, RUNNING} — e.g. a plane DENIED racing a pre-engage E14 into POST_RESET) | unchanged (rejected) | none — a late DENIED is recorded as an event only, never a transition; a pinned outcome is untouched | `agent_invite_denied_after_engage`, `agent_invite_denied_in_post_reset` |

QUEUED and COMPLETED updates (`AgentTaskUpdate`, services.proto) are
informational on every state: recorded, never a transition. The invite
emission and timer are episode-open effects (E23); nothing else about
episode open changes.

---

## 2. Claim lifecycle

States (per claim): REQUESTED → GRANTED | DENIED; GRANTED → RELEASED.

| # | From | Trigger | Guard | To | Notes | Fixture |
|---|---|---|---|---|---|---|
| C1 | — | `claim_request` | episode in RUNNING or INTERVENTION-eligible; no conflicting active claim | REQUESTED | `claim{REQUESTED}` event | `handoff_*` |
| C2 | REQUESTED | control-plane decision | planner grants | GRANTED | `claim{GRANTED}`; engage may begin | `handoff_*` |
| C3 | REQUESTED | control-plane decision | conflicting claim, no grant | DENIED | `claim{DENIED}` | — |
| C4 | GRANTED | `release` completes or episode reaches TERMINAL (except via retake) | — | RELEASED | `claim{RELEASED}` | `handoff_immediate_mid_chunk` |
| C5 | GRANTED | `retake` | — | GRANTED (survives) | the claim is NOT released; the successor episode is born claimed under it | `retake_operator_optimistic` |
| C6 | — | `claim_granted` | episode in RESETTING or POST_RESET ∧ window OPENED ∧ actor matches expected (a TELEOPERATOR window also admits SITE_OPERATOR; an AGENT window admits AGENT only) ∧ no active claim | GRANTED | `claim{GRANTED}` — a real `Claim`; the N18 one-claim rule applies (flag `waddle.v0.reset.remote`; see §1.4) | `remote_pre_reset_claim_engage_complete`, `remote_reset_wrong_actor_denied` |
| C7 | GRANTED (reset claim) | E21 / E22 / `estop` | — | RELEASED | `claim{RELEASED, "reset window closed"}` (flag `waddle.v0.reset.remote`; see §1.4) | `remote_pre_reset_claim_engage_complete` |
| C8 | — | `claim_granted` | episode agent-invited ∧ actor matches expected (an agent-invited episode admits `ACTOR_KIND_AGENT` only; any other actor's grant is rejected, `claim{DENIED}`) ∧ C1's episode-state and no-conflicting-claim conditions unchanged | GRANTED | `claim{GRANTED}` — a real `Claim`; the N18 one-claim rule applies (flag `waddle.v0.agent`; see §1.5) | `agent_invite_happy`, `agent_invite_wrong_actor_denied` |

**Every claim event names its claimant.** The `Claim` carried by
`claim{REQUESTED|GRANTED|DENIED|RELEASED}` MUST carry the claimant's
`ActorRef` whole — the kind AND the id the granting side stamped, display
name included when it has one. `Claim.source_name` names the intervention
*stream* ("teleop", "leader_arm", "waddle-agent"), never the actor, so a
consumer given only `source_name` cannot attribute the claim to anyone: the
journal, the sidecar's claim and provenance spans, and every downstream
judge read the actor off these events. A claim granted LOCALLY (a clutch
edge, C-section below) has no stamped identity and carries kind only.
Fixtures: `claim_events_name_the_claimant`,
`agent_claim_events_name_the_agent`.

**Self-initiated claims** (`Claim.self_initiated`): a local source's clutch
edge (`clutch{engaged}`) both requests and grants the claim in one step — the
platform records the intervention rather than fighting it (the engaged clutch
is the authorization; `ProvenanceTag.bypass_approval` may be set, which
bypasses approval gates but never the envelope, the lease, or the e-stop).

---

## 3. Lease machine

States: `Vacant` | `Held{lease_id, holder_client_id}`. Adopted from the
production single-writer broker; enforcement point per N7
(`LeaseEnforcement`).

| # | From | Event | Guard | To | Result | Fixture |
|---|---|---|---|---|---|---|
| L1 | Vacant | `LeaseAcquire{client}` | — | Held{fresh, client} | granted, fresh token | — |
| L2 | Held | `LeaseAcquire{client}` | client == holder | Held (unchanged) | granted, **same** token ("already held") — idempotent | proptest |
| L3 | Held | `LeaseAcquire{other}` | client ≠ holder | Held (unchanged) | denied ("held by …") | — |
| L4 | Held | `LeaseRelease{lease_id}` | token matches | Vacant | released | — |
| L5 | Held | `LeaseRelease{other}` | token mismatch | Held (unchanged) | denied ("stale or wrong") | — |
| L6 | Held | `LeaseHandoff{from, to}` | from == current token | Held{**fresh** token, to} | atomic handoff; old holder's in-flight streaming ended; staleness watchdog re-arms on new holder's first streaming command | `handoff_*` |
| L7 | Held | `LeaseHandoff{stale, to}` | from ≠ current token | Held (unchanged) | denied | proptest |
| L8 | any | `RevokeAll` (estop) | — | Vacant | every previously issued token is dead | `estop_revokes_all_leases` |

Non-holder `VERB_SEND` yields `VerbResult{ok:false, fault.kind:
FAULT_KIND_LEASE_DENIED}`. Under supervision this is a **pause signal**
(re-acquire or hand off), never a fault-equivalent abort
(`lease_denied_non_holder`).

**Advisory enforcement** (N7/N14): where nothing physically stops the
integrator's loop from writing during a takeover, the planner prefers
HOLD_FIRST handoffs and the implementation runs dual-write detection during
bypass: sustained divergence between commanded trajectory and proprioception
⇒ `request_verb{HOLD}` + `DualWriteDetected` event with a persisted trace
(`dual_write_detection`).

---

## 4. Intervention lifecycle

Phases within INTERVENTION: **engage → settle → release | retake**
(`InterventionPhase`).

| # | Phase | Entered on | Exited on | Notes |
|---|---|---|---|---|
| I1 | ENGAGE | claim granted + engage trigger | handoff sub-protocol completes (lease handed to intervenor) | per declared `HandoffPolicy` (§5) |
| I2 | SETTLE | first intervention action flowing (or explicit settle) | `release` or `retake` | the intervenor stabilizes the scene; settle timeout may arm a timer whose expiry permits retake from ENGAGE |
| I3 | RELEASE | `release` requested | policy re-primed on fresh observations, lease handed back, claim released | mirrors ENGAGE in reverse (§5) |
| I4 | RETAKE | `retake` requested | episode → TERMINAL{ABORTED_RETAKE}; successor opened born-claimed | claim survives (C5) |

---

## 5. Handoff sub-protocol (per `HandoffPolicy`)

Common preconditions: claim GRANTED; lease handoff is L6 (atomic, fresh
token). The soon-to-be-idle writer is stopped **before** the lease moves; the
new writer starts only **after** a granted handoff — there is no window with
two writers. If the handoff is denied or fails, both writers remain stopped
and the failure is emitted (fail-closed).

### IMMEDIATE{blend_ns}

```
policy source paused ─► lease handoff ─► intervention stream live
        │                                        │
        └── executing chunk DROPPED; gate cross-fades from the last
            commanded point into the intervention stream over blend_ns
            using the space's declared Interpolation. Provenance flips
            to the intervenor exactly at blend start.
```

If the intervention action's flattened width doesn't match the declared
action space, the gate holds instead of cross-fading — a dims mismatch is
never zip-truncated into a shorter, meaningless action. Media intake is the
primary check (drops the mismatched action before it reaches the gate,
faulting `FAULT_KIND_VALIDATION_ERROR` once per claim window); the gate's
own refusal to blend mismatched lengths is defense in depth.

Fixtures: `handoff_immediate_mid_chunk`, `teleop_dims_mismatch_holds`.

### CHUNK_BOUNDARY{max_wait_ns}

The executing chunk finishes (capped at `max_wait_ns`; 0 = full remaining
horizon), then the lease moves, then the intervention stream is live.
Fixture: `handoff_chunk_boundary`.

### HOLD_FIRST

`request_verb{HOLD}` is issued and a successful `VerbResult` is required
**before** the lease handoff; the intervenor starts from rest. The
conservative default for advisory-lease integrations and high-rate/heavy
arms. Fixture: `handoff_hold_first`.

### Release (all policies)

Release mirrors engage in reverse: the intervention stream stops; the policy
is **re-primed on fresh observations** (a new chunk computed from a current
`t_obs_ns` — never resuming a stale pre-engage chunk); the lease hands back;
the claim releases; blending per policy applies symmetrically.

### Bypass mode (claimed-while-stalled)

If the integrator's loop stalls while a claim is active (no `gate_tick`
within the stall threshold), intervention actions MUST NOT be starved by the
stalled loop: the implementation drives the declared `send` verb directly
from its own thread, and `gate()` returns `NoopMarker{BYPASS_ACTIVE}` to any
late caller ticks so the loop stays coherent as a spectator. The caller MUST
NOT dispatch NOOP-marked actions. `GateModeChange{→BYPASS}` is emitted on
entry and `{→INTERVENTION}` on exit when caller ticks resume. Fixture:
`claimed_while_stalled`.

### Delta-space restriction

Delta spaces (`EEPoseDelta`) compose against the chunk-start state snapshot;
a chunk entered at index > 0 has no valid compose base. Conforming
implementations refuse mid-chunk splice entry for delta spaces (engage under
IMMEDIATE degrades to HOLD_FIRST for a delta space).

---

## 6. Gate modes

| From | To | Trigger | Fixture |
|---|---|---|---|
| PASSTHROUGH | INTERVENTION | engage completes (lease handed to intervenor) | `handoff_*` |
| INTERVENTION | PASSTHROUGH | release completes | `handoff_immediate_mid_chunk` |
| INTERVENTION | BYPASS | stall detected while claimed (no gate tick within threshold) | `claimed_while_stalled` |
| BYPASS | INTERVENTION | caller ticks resume | `claimed_while_stalled` |
| PASSTHROUGH | RESET | remote reset window engage completes (E20: lease handed to the reset claimant) — flag `waddle.v0.reset.remote` | `remote_pre_reset_claim_engage_complete` |
| RESET | PASSTHROUGH | remote reset window complete/timeout handback completes (E21/E22) — flag `waddle.v0.reset.remote` | `remote_pre_reset_claim_engage_complete` |

BYPASS and RESET never inter-transition.

Every change emits `GateModeChange`. Holds (HOLD_FIRST engage, tripwire
holds) are an output condition (`gate()` returns Hold/Noop), not a gate mode.

The gate is the **only** point where Waddle touches the integrator's loop:
nominally a passthrough that (1) stamps and logs the (obs, action) pair,
(2) checks local tripwires, (3) consults claim state — and returns the
action unmodified. Under a claim it returns intervention actions tagged with
their provenance; the intervention segment is labeled at write time.

---

## 7. Grant liveness (N6, N11)

Per (verb, send_interface): `GrantStatus` ACTIVE ⇄ DEMOTED → REVOKED.

Rules:

1. Grant health is **inferred from proxy signals** (`ProxySignals`:
   control-plane RTT, gate-tick jitter, host load, callback dispatch time) —
   never from live verb invocation. Actual verbs are re-measured only in safe
   windows (`MeasurementWindow`: DOCTOR, BETWEEN_EPISODES, DURING_RESET).
2. **Demotion never interrupts an active lease.** A `GrantStatusChange`
   carries `effective_t_ns`; it takes effect at the next planning decision.
3. Transitions carry **hysteresis**: a signal hovering at the declared bound
   must not flap the planner. Re-promotion requires either sustained recovery
   past the hysteresis band or a safe-window re-measurement.
4. Every change is actor-visible: emitted as an `EpisodeEvent` and carried on
   `HeartbeatAck.grant_changes`; never applied silently.

Fixture: `grant_demotion_hysteresis`.

---

## 8. Degraded operation (backend partition)

On loss of control-plane connectivity (`partition_start`):

- Local tripwires and the heartbeat watchdog keep running; a sustained
  heartbeat loss fires the staleness tripwire, which requests HOLD through
  the declared verbs (fail safe locally).
- Cloud-dependent grants degrade to observe-only (DEMOTED with reason
  `partition`).
- `EpisodeEvent`s buffer locally (bounded, drop-oldest with a high-water
  event) and replay **in order** on reconnect (`partition_end`).

Fixture: `backend_partition_degradation`.

**Directive acks (flag `waddle.v0.plane.acks`).** A plane directive
(`ClaimDirective`, `EpisodeDirective`, `ResetWindowDirective`) that carries a
`directive_id`, on a connection that negotiated the flag, is answered with
exactly one `DirectiveAck` (`GateClientMessage` arm 4): `accepted=true` when
the session FSM applied every event the directive decoded into,
`accepted=false` with the FSM's rejection reason (guard-row language, e.g.
"engage outside RUNNING (E7)", "terminate rejected in POST_RESET (E14b)")
when any was rejected — a directive that decodes into more than one event
(a claim GRANT, a reset-window ENGAGE) acks once, rejected if any event was,
with the first rejection's reason. This is observability only, **not a guard
row**: the FSM accepts and rejects exactly what it did before the flag
existed, a NACKed directive changed no state, and acks never appear on the
`EpisodeEvent` stream or in sidecars. Directives without a `directive_id`
stay fire-and-forget; a directive too malformed to decode into session
events at all produces no ack.
