# Finite reference playback

Binding API 24 adds opt-in `PlaybackPolicy.part_hold` under the separate
`execution.part_hold_playback` fact. For supported named buffers, cancellation
or ordinary execution failure invokes only that part's existing owner Hold,
after its in-flight callback drains. Other buffers keep their own outcomes.
The default remains site-wide cleanup. Coordinated/whole-site buffers cannot
select part Hold, and native admission requires a local part-Hold callback.

Scoped cleanup adds `part` and optional structured `evidence` to its receipt.
The original motion failure remains separate; a missing/failed Hold response
is unknown cleanup, never physical-stop proof or permission to retry motion.
Registered site Hold/e-stop, control takeover, episode closure and shutdown
retain their global behavior. A global stop arriving during scoped cleanup
drains all buffers and supplies global receipts before terminal publication.
Consumers still own collision proof and retention of uncertain operation state;
this primitive adds no planner, writer thread or hardware isolation guarantee.

Binding API 23 adds `execution.parallel_joint_playback` for independent named
buffers. Distinct parts can execute concurrently on the existing serialized
control worker, with their own local clocks, completion and retained handles.
The worker interleaves callbacks; a blocking device callback still delays other
parts. Duplicate scopes are Busy, and a whole-site buffer conflicts with every
named buffer. Normal completion of one part leaves other buffers running.

Registered Hold/e-stop callbacks remain site-wide. When cancellation or a failure
uses that cleanup path, every active or queued sibling is cancelled too. Cleanup
receipts reach queued handles even when the stop precedes their worker admission;
old queued buffers never resume afterward. New admission is fenced while cleanup
runs. The originating failure remains distinct from sibling cancellation and
from the cleanup result. Selectively scoped cleanup requires the separate API 24 policy and fact above.
Consumers must preserve application collision proof before selecting parallel
playback. Episode closure drains all retained loops.

Named `Run.step_parts` commands can address idle scopes while another part plays.
Any addressed overlap refuses the entire call before writing. Whole-site `step`
stays Busy while any buffer is active. The public native conflict query
`Session.playback_active_on(episode_id, part)` is status, not a reservation or
permission to bypass the ordinary gate/owner envelope.

Binding API 22 adds `CoordinatedJointMotion(parts, references, policy)` for the
complete joint-position Site. `parts` must name every part in declaration order;
positions, velocity hints, polynomial rows and policy indices use that flattened
layout. Every part must advertise `execution.coordinated_joint_playback` and all
other policy support. A named `FiniteJointMotion` retains its original behavior.

The group exclusively occupies the whole-site scope and uses one native clock,
curve phase and acknowledgement. Local observations copy every participating part;
completion evaluates the selected joint rows together. Native admission enforces
the slowest participating command cadence (and the composite cadence). Each full
fixed-reference interval must respect that cadence; a late acknowledged write
cannot trigger faster catch-up writes. Original lateness/deadline limits still
apply rather than extending the motion budget. Each reference crosses the
ordinary episode gate and all addressed owner envelopes
before the first driver write. Those writes are sequential: if a later write
raises, earlier side effects remain possible, the original failure stays uncertain
and ordinary site Hold/drain follows. Cancellation waits for an in-flight reference
before cleanup. This group interface adds no planning, application geometry
validation or physical synchronization guarantees. Scalar
gripper outcome/force policies are refused for a group; ordinary held-jaw behavior
and generic feedback/curve policies retain their existing contracts.

If supervision selects a command for one part in place of a whole-site reference,
its velocity hint and required neighbor checks use that selected part. An accepted
substitution ends the original buffer as `superseded` without sending another Hold
over the selected stream. Owner refusal retains its original evidence and normal
Hold/drain.

Binding API 19 adds `feedback_reference`, advertised by
`execution.playback_feedback_reference`. Each snapshot pairs its `feedback` with
the last acknowledged reference before that observation was acquired. It has the
same fields as `last_reference`, but later writes may advance `last_reference`
without changing this pair. `feedback_reference` is `None` before the first
acknowledgement. Its offset is global curve phase for paced execution; its
positions are commanded values, not measured state. This avoids interpreting
delayed status receipt or a newer command as the reference for an older sample.
SDK observation time still does not establish motor acquisition time.

Binding API 18 adds opt-in future curve suffixes, described below. Existing
policies remain immutable by default and retain their previous behavior.

Binding API 17 adds optional `PlaybackPolicy.checkpoint_offsets_ns`, gated by
`execution.checkpointed_joint_playback`. Up to 256 strictly increasing offsets
in the inclusive buffer range retain diagnostic measurements on the same local
worker. Empty offsets preserve existing behavior; gripper outcome policies cannot
request checkpoints.

Each `checkpoints` snapshot entry contains the requested `offset_ns`, actual
`acknowledged_offset_ns`, session-clock `acknowledged_ns`, local `elapsed_ns`, and
the complete fresh `feedback` observed after that acknowledgement. Paced curves
use acknowledged curve phase, never elapsed wall time. A sample may cross several
markers; those entries truthfully share the same measurement. The worker does not
interpolate measurements, pause at a marker or claim intermediate arrival.
Evidence survives delayed polling and terminal failure/cancellation; unreached
markers have no entry. Observation timestamps do not establish motor acquisition
time. Checkpoint recording changes no gate, tracking bound, timeout or stop path.

Binding API 16 adds optional `PlaybackPolicy.reference_only`, gated by
`execution.reference_joint_playback`. It ends with `references_dispatched` only
after every reference is acknowledged and fresh post-endpoint feedback passes
normal tracking checks. It makes no arrival, stationarity or grasp claim and
cannot be combined with a gripper outcome policy. Start, gate, pacing, tracking,
deadline and stop behavior are unchanged. Snapshots retain
`last_acknowledged_ns` on the SDK session clock so callers can budget subsequent
goal checks from device acknowledgement rather than remote reply receipt.
No new thread, writer or authority is created; ordinary policies remain unchanged.

Binding API 15 adds `PlaybackPolicy.minimum_settle_samples` (default one), gated
by `execution.sampled_joint_playback` when nondefault. Both the count of fresh
eligible observations and `settle_ns` must qualify. Duplicate timestamps do not
count; velocity or required-goal loss resets eligibility. Encoder windows retain
at least the requested count and three samples. Gripper policies keep their
separate outcome-specific rules and require the default count. These are SDK
observation samples, not a claim of independent motor acquisition or physical
performance.

`waddle_runtime::playback` accounts for a fully received, bounded sequence of
joint-position references on a local session clock. `Session.start_playback`
executes an admitted buffer on the existing serialized `waddle-verbs` thread.
The Python `Run.start_playback` interface uses this worker; current connectors
do not automatically use it. It opens no device and creates no additional thread,
claim or lease.

## Input and timing

`FiniteReferences` reuses the existing validated `ActionChunk` and `Step` types.
Its caller must first validate the chunk against the declared joint-position
action space. One buffer addresses one scope, contains 1–16,385 references with
1–128 rows, starts at offset zero and has strictly increasing offsets ending
within 120 seconds. Its horizon equals the final offset. Values and optional
velocity hints must be finite and rectangular; the final hint must be at rest.
Grippers represented by a joint row are valid; gripper sidechannel actions are
not part of this interface.

By default each reference uses hold interpolation. This mode never fits a curve,
rescales time, drops a reference or substitutes a new target. Applications must
prepare and validate the exact reference sequence they intend to execute.
The intervention jitter buffer retains its existing latest-wins semantics;
finite reference accounting does not change that stream.

`PlaybackPolicy` supplies positive start, final-position and stationary-velocity
tolerances for selected moving rows. Other rows remain in the full command and
still cross the owner's envelope; for example, a held jaw's contact shortfall
need not become an arm arrival condition. The policy also supplies a positive
settling interval, an initial timeout of at most 180 seconds, and permitted reference
lateness of at most one second. These are caller-selected bounds, not measured
hardware performance guarantees. No default widens an owner's limits.

## Optional acknowledgement-paced curves

`PlaybackPolicy.paced_curve` selects local evaluation of a supplied `PacedCurve`
(Python: `PacedJointCurve`). Check the separate per-part
`execution.paced_joint_playback` support fact. The curve contains exact boundary
times/positions and six normalized polynomial coefficients per segment, in
segment → increasing power → selected coordinate order. Coordinates follow the
parent policy's `indices`; unselected command rows retain their input targets.
The caller remains responsible for checking the complete polynomial's limits,
geometry and dependency identity. This API does not plan, fit, certify collision
clearance or correct a goal. Optional measured holding below can approach an
unchanged held target without changing the selected polynomial coordinates.

Timing is bounded to 120 seconds and 16,384 segments. Native admission checks
dimensions, finite values, increasing times, declared boundaries, stationary
final velocity and agreement with the supplied nominal joint references. Boundary
consistency uses 1e-9 coordinate units, nominal reference agreement uses 1e-8,
and the curve duration must agree with the integer chunk horizon within 1 ns.
These numerical consistency checks do not widen the owner's safety envelope.
Velocity hints, if enabled, must agree with the nominal polynomial and be zero
on unselected rows. Every generated command still passes the ordinary episode
gate and full owner checks.

`period_ns` is the minimum interval between local submissions; its cadence cannot
exceed the declared command rate. Each successful acknowledgement advances nominal
curve time by at most that period. If the next sample exceeds 90% of the supplied
per-coordinate `position_error`, bounded bisection selects a shorter step on the
same polynomial, or repeats the prior reference. Leaving the supplied
`tracking_tolerance` or the admissible prior-reference window fails explicitly.
Partial steps scale analytical velocity by actual phase advance; repeated
references have zero feedforward. No reference is skipped to catch up.

Optional `sample_times_s` retains an exact nominal grid for rest-to-rest quintics.
The grid includes all segment boundaries, never has an interval longer than
`period_ns`, and leaves a partially reached sample pending until acknowledged.
Without it, the next sample is one period beyond the last acknowledged phase.
By default the original absolute timeout bounds both traversal and settling. The fixed-mode
`max_lateness_ns` guard is not used for acknowledged progression: late feedback
consumes the same timeout and slows progress instead of jumping ahead.

The worker uses a locally stamped submission start when the adapter returns
`DispatchedAt(started_ns)`; otherwise it conservatively spaces from the completed
acknowledgement. Delayed or unconfirmed writes cannot be replayed. Native snapshots
retain `curve_progress` and the complete `last_reference` only after a confirmed
dispatch. `references` counts input references; `dispatched` counts actual
acknowledgements and can be larger with polynomial evaluation/partial steps.
The optional curve does not provide dynamic world updates, contact detection,
force-grasp completion or coordinated parts.

### Optional pacing credit

Binding API 21 adds `PlaybackPolicy.credit_pacing_delay`, requiring
`execution.paced_playback_deadline` and reference-only polynomial playback.
Default false preserves the absolute wall-time limit. Enable it only when the
caller selects a curve-progress budget rather than an explicit wall deadline.
The original `timeout_ns` remains the initial budget; the effective deadline
adds `curve_progress.pacing_extension_ns` on the SDK's session clock.

The worker earns credit only after acknowledging at least one complete
`period_ns` of reference progress (within 1 ns). It adds the positive difference
between local elapsed acknowledgement time and that phase advance. Partial
progress must accumulate a full tick before earning credit; a repeated phase
cannot renew the budget. Exact constant trailing segments, identified by zero
nonconstant polynomial coefficients, earn no credit and reset the accounting
anchor. A completely stationary curve earns none. Interior stationary segments
before the final moving segment remain within the caller's moving prefix.

The native deadline check and worker wake-up use the same accumulated credit.
No unacknowledged write, cloud polling interval, planned update or withdrawal
earns credit. A completed local callback may supply the acknowledgement after
the previous deadline; its completed moving tick is then accounted before the
next deadline check. A stalled callback still follows the existing owner and
supervision contracts; this policy adds no callback preemption. Accepted curve
revisions retain global phase and cumulative credit. The original finish budget,
tracking/envelope checks, failures and Hold/e-stop/drain behavior stay intact.
Credit may extend effective wall time beyond the initial 180-second admission
bound; callers needing a strict total time must leave this option false.

## Optional future curve updates

`PlaybackPolicy.allow_curve_updates=True` requires the separate
`execution.revisable_joint_playback` fact and reference-only paced execution.
Gripper outcome policies, checkpoints, exact sample grids and `toward_goal`
velocity hints cannot be combined with updates. Coordinate order, held targets,
feedback policy, pacing/tracking limits and the original timeout policy stay
fixed for the execution.

`RevisablePlaybackHandlePort.update_curve(PlaybackCurveUpdate(...))` queues one
request without waiting for device I/O. The new curve starts at zero;
`switch_ns` is a future offset on the original execution's **acknowledged curve
phase**, not a wall clock. `expected_revision` must match the active revision
(initially zero). The switch must still lie on the incumbent curve when the local
worker checks it. The combined end must stay within 120 seconds. Dimensions and
polynomial admission are unchanged; position, velocity and acceleration must
agree at the handoff within 1e-8 coordinate units and their time derivatives.
The caller must check the complete new curve's limits, geometry and dependencies.
The SDK does not certify those checks or plan a replacement.

The returned request ID identifies the latest `curve_update` receipt: `pending`,
`accepted` or `rejected`, with the original refusal text. Acceptance schedules
one future suffix; it is not execution or arrival. `curve_progress` retains
`revision` and `pending_revision`; the active revision advances only when an
acknowledged reference crosses the switch. Global phase never resets. Another
suffix cannot replace an accepted pending suffix. Serialize requests and inspect
their receipts; this bounded status keeps the latest request, not a history.

Stale, discontinuous or incompatible updates leave the incumbent intact. A
queued request rejected by cancellation/closure cannot resurrect execution or
alter a later run. The same worker retains ordinary feedback, tracking, pacing,
gate and Hold/e-stop behavior. An accepted suffix never renews the deadline.
Without an accepted replacement the incumbent continues to its checked resting
endpoint. A lost response requires reconciliation from the retained handle;
never automatically replay an update. Stop/cleanup receipts remain distinct from
measured physical stopping.

These contracts supply future references, not live collision-world validation,
reactive goal logic, multi-arm coordination or physical/WAN qualification.

### Withdrawing a future suffix

Binding API 20 separately advertises `execution.withdrawable_joint_playback`.
`WithdrawablePlaybackHandlePort.withdraw_curve(request_id, expected_revision)`
queues withdrawal of the original accepted update ID while its incumbent
revision remains active. It returns a **new** request ID. The latest
`curve_update` receipt includes `withdraws_request_id` and the ordinary
`pending`/`accepted`/`rejected` result. Only `accepted` confirms removal.
The original trajectory then continues with its exact acknowledged phase,
pacing, held coordinates and deadline. Withdrawal does not stop motion or
validate that the incumbent is still safe in a changed environment.

The same serialized worker processes withdrawal between device submissions.
An in-flight handoff finishes first; if it activated the suffix, withdrawal
refuses. Wrong update IDs, stale revisions and terminal executions also refuse.
A delayed request cannot remove a newer suffix even when that suffix uses the
same incumbent revision. Rejected intervening updates do not change the
accepted suffix's identity. Whole-execution cancellation drains queued requests
through the ordinary Hold path. A lost response requires reconciliation, never
automatic replay or an assumption that the suffix was removed. A caller that
needs a new checked continuation must wait for confirmed withdrawal and admit
that continuation through the ordinary update checks.

## Optional measured execution policy

`PlaybackPolicy.feedback` accepts `FeedbackPolicy` (Python:
`PlaybackFeedbackPolicy`) with the separate `execution.feedback_joint_playback`
support fact and binding API 12. Omission preserves existing behavior.

- `held_groups` contains `HeldJointGroup` values: unselected row indices, positive
  per-row `position_error`, and a local `period_ns` between 0.1 ms and 1 s. Each
  group approaches its unchanged input target with 90% headroom. Opposing measured
  velocity reserves another period of travel. All coordinates in a group scale
  together; an already-admissible target retains its exact bits. This adjustment
  never changes the selected curve's coordinates or bypasses the owner's envelope.
- `measured_indices` copies current measurements for unselected rows, for example
  a jaw already held by its driver. It does not activate a force mode or prove
  contact. Held groups, measured rows and selected rows must be disjoint, with
  no repeated or undeclared indices. All unselected input targets remain constant
  throughout the buffer and their hints must be zero.
- `velocity_hints` is `curve`, `toward_reference`, or `toward_goal`. The first
  retains the original hints, including paced partial-step scaling. The second
  suppresses hints opposing current reference error and permits later reversals.
  The third latches each hint off after measured position reaches/crosses the
  final goal in that hint's direction. It stays off for the rest of the buffer.

An optional `position_window` replaces instantaneous velocity stationarity with
encoder stability over `settle_ns`. Its positive `max_drift` follows selected-row
order. At least three fresh samples must cover the full interval; the sample
immediately before its leading boundary is retained. The optional `max_gap_ns`
resets history after an excessive gap. Repeated timestamps reset history;
decreasing/invalid timestamps remain terminal errors. History is bounded to
16,385 samples; exceeding that bound fails instead of discarding required evidence.

Normally only post-endpoint, in-tolerance samples are eligible. `allow_settled`
also admits stationary misses. `during_motion=True` explicitly permits eligible
pre-endpoint history; the final reference must still be acknowledged and followed
by fresh feedback before completion. `consecutive_samples` (default 1) can require
additional consecutive qualifying windows after that acknowledgement. A single
in-tolerance sample cannot relabel a stationary miss as arrival: the full arrival
interval must remain in tolerance. Raw velocity is diagnostic and still must be
finite. The selected deadline policy remains binding.

Snapshots expose `settling` with sample count, span, per-row position spans, raw
velocity peak and consecutive qualifying count. Its time source is the SDK
observation timestamp, not a claim about motor acquisition timing. Contact/force
classification, geometry/world validation and dynamic updates remain separate.

## Optional gripper execution

`PlaybackPolicy.gripper` selects `GripperPolicy` (Python:
`GripperPlaybackPolicy`) with binding API 13 and `execution.gripper_playback`.
It selects exactly the Site's declared jaw row and affine physical opening
mapping (`closed_position`, `open_position`, `closed_m`, `open_m`). Reversed
action directions are supported. Unselected nominal targets remain constant;
optional held groups can still approach them. A paced polynomial, encoder
position window, nonzero hints and stationary-miss completion cannot be combined
with this jaw policy.

`period_ns` spaces commands from **completed acknowledgements**, using the local
clock and no catch-up. It cannot exceed the declared command rate. Nominal jaw
targets advance once per acknowledgement; after their end, correction continues
toward the final target under the original absolute timeout. Each correction
reserves 10% of `position_error` plus one period of opposing measured travel.
`tracking_tolerance` checks the preceding actual command, except while the
declared contact/force condition accounts for a blocked jaw. Ordinary owner
limits remain authoritative for every command. The fixed-offset lateness guard
does not apply in this mode.

- Position arrival requires three consecutive fresh, stationary, in-tolerance
  observations after the nominal sequence has been dispatched. Exhausting the
  sequence alone is not completion.
- Optional `GripperContactPolicy` requires a closing target, minimum physical
  opening and measured closing progress, plus shortfalls from both the final goal
  and the actual preceding command. At least two samples must cover `window_ns`
  with opening drift within `max_drift_m`; the sample at/before its leading
  boundary is retained. This yields `contact_stall` even if instantaneous velocity
  is noisy. It does not identify the object or measure force.
- Optional `GripperForcePolicy` carries one constant `target_n`, `tolerance_n`
  and `closed_tolerance_m`. The part must support force control and sourced force
  observation. Positive force immediately pursues the final jaw target using 50%
  position-error headroom. Three consecutive fresh, stationary force measurements
  at least `target_n - tolerance_n` yield `force_reached`; source and estimated
  status remain attached. Opening at/below the closed stop plus tolerance fails
  with `gripper_empty`, even with high motor effort. Missing force feedback fails
  with `gripper_force_unavailable`. These conditions do not calibrate motor effort
  as fingertip force. Positive force and contact classification cannot coexist.
- A force target of zero explicitly releases the driver's force mode and uses
  positional/contact completion. No force policy leaves the ordinary driver mode
  unchanged. Successful force completion retains the hold; explicit release,
  Hold/e-stop and teardown use the driver's existing lifecycle.

Repeated or pre-acknowledgement feedback resets completion history. The snapshot's
`gripper` evidence retains target/measured/commanded opening, progress, shortfalls,
window span/drift and arrival/force sample counts. Failure and uncertain writes
retain their original outcome and normal native Hold/drain; no reference retries.
Geometry checks, object identity and dependent action orchestration belong to the
consumer. This API does not qualify a physical grasp or multi-part execution.

## Driving the state

Construct `Playback` with the local session-monotonic start time. For each poll,
provide time from that same clock and measured positions/velocities with their
original observation timestamp. Remote emission time is not the execution clock.

- `Dispatch(index)` means attempt that reference through the existing episode
  gate and owner envelope. It grants no actuation authority. Call
  `acknowledge(index, completed_ns)` only after a confirmed dispatch of that exact
  reference. A refusal, substituted action or unknown write cannot be acknowledged
  as the requested reference.
- `WaitUntil(session_ns)` means the next reference is not due yet.
- `Observe` requests newer measured state. Feedback at or before the previous
  acknowledgement cannot admit the next reference or establish completion.
- `Complete(Arrived)` requires all dispatch acknowledgements and fresh,
  stationary, in-tolerance measurements for the configured settling interval.
- `Complete(Settled)` is an explicitly enabled stationary miss, distinct from
  arrival. The final observation alone cannot relabel a miss as arrived.

The pure accounting interface is useful for deterministic tests. Actual native
execution uses the session interface below; callers must not build a second
frontend timing loop around it.

## Native session execution

`Session.start_playback(episode_id, wire_chunk, policy, io)` validates the complete
chunk against the session's declared action space before admitting it. It accepts
one joint-position scope, including a named part or a complete composite of
joint-position parts. It refuses mixed action spaces, mixed scopes, inert actions and gripper side
channels. A registered Hold callback is required. Distinct named scopes may have
outstanding loops; overlapping scopes or an outstanding control/safety request
return `Busy`. This is execution serialization, not an additional actuation lease.

Implement `PlaybackIo` as local observation and dispatch adapters. `observe`
returns fresh, original session-stamped positions and velocities. `dispatch`
must use that episode's ordinary gate and owner's complete safety envelope.
It returns `Dispatched` only for a confirmed write of the exact requested
reference. A gate substitution, Hold or Noop follows its normal dispatch
semantics and returns `Superseded`, ending this buffer without overriding the
selected stream. Failed or uncertain writes return the original `PlaybackIoError`
detail, opaque structured evidence and uncertainty flag. Neither callback may
wait on a remote planner or synchronously wait for a verb on the same thread.
The session clock determines reference deadlines; observation sampling between
references uses the declared scope rate.
Deadline and reference-lateness checks still occur by their configured bounds
when that feedback rate is slower.

The returned `ExecutionHandle` retains an ID, confirmed dispatch count, latest
feedback and eventual result. `snapshot` is read-only; `wait` blocks on completion
and language bindings must release their language lock while waiting. `cancel`
ends future references and the worker requests the registered Hold. An in-flight
callback finishes before that Hold; no buffered reference follows it. A terminal
handle cannot cancel a successor or rewrite its own result. Cancellation racing
an arrival waits for local cleanup instead of publishing arrival.

Ordinary control verbs, priority Hold and e-stop cancel the buffer and use the
same dispatch thread. Failed cleanup cannot erase an original observation/write
failure. Session episode termination drains playback before terminal/reset
processing; a stale episode exits without holding a successor. Shutdown joins
the dispatch path before hardware may close. A stuck device callback still needs
the owner's ordinary watchdog and fault handling; this API cannot forcibly make
an uncertain write safe.

A terminal execution result means the reference loop has drained. It is not a
claim that a safety verb physically stopped the device: the original verb outcome
and owner measurements remain separate evidence. Binding API 14 includes the
completed stop callback in `cleanup`, separately from the original motion result.
It records `request` (`hold` or `estop`), `outcome` (`acknowledged` or `unknown`),
the original callback error text, `completed_ns` on the session clock, and
`measured_stationary: false`. The worker publishes this receipt only after the
callback returns. An uncertain write stays uncertain even when the stop succeeds.
Missing cleanup is not confirmation: successful motion, a superseding stream,
an ended scope or cancellation before execution can have no stop receipt.
`Session.hold` and `Session.estop` still acknowledge only that a request was queued;
posting another request cannot turn missing/failed cleanup into confirmed cleanup.

## Python Run execution

The optional `FinitePlaybackRunPort` in `waddle_sdk.runtime` exposes
`start_playback(FiniteJointMotion)`. Check the addressed part's
`execution.finite_joint_playback` support fact. The local Site implementation
declares it for a joint-position part with registered Send and Hold verbs;
the fact never grants permission or bypasses native gating.

Build the immutable request using `JointReference(offset_ns, command)` and
`PlaybackPolicy`. Commands use `JointPositionCommand` in the part's declared
joint order and units, with optional declared velocity feedforward. The first
reference must match measured start within the supplied tolerances. The final
hint must be zero. Supply the entire validated sequence before admission; this
interface performs no planning or fitting. By default it holds each reference
until the next one; the optional paced curve evaluates supplied coefficients
as described above. Per-reference force modifiers are refused; a constant force
uses the optional gripper policy above. An existing force
mode survives an unchanged caller action under the ordinary owner rules.

The returned handle provides:

- `snapshot()`: an independent dictionary with `id`, `episode_id`, `dispatched`,
  `references`, `feedback`, `last_reference`, `curve_progress`, `status`, and
  `failure` and optional `cleanup`. Feedback retains `observed_ns`,
  positions, velocities and optional sourced gripper force. Status is `running`,
  `arrived`, `settled`, `contact_stall`, `force_reached`, `references_dispatched`,
  or `failed`. `last_acknowledged_ns` is the SDK session-clock completion of the
  latest acknowledged owner submission, or `None` before any acknowledgement.
  For `references_dispatched` it stamps the final reference; the following
  feedback has a strictly later SDK observation timestamp. Optional
  `gripper` and `settling` fields retain their measured evidence.
- `wait()`: the same snapshot after native drain, releasing the Python interpreter
  lock while waiting; it creates no polling thread.
- `cancel(reason)`: native cancellation and the registered Hold path, without
  replaying or changing a terminal result.

Failure includes a stable `code`, original `detail`, `outcome_unknown` and
structured `evidence` for owner failures. Observation and dispatch failures remain
distinct. Native failures retain codes such as `stale_start`, `late_reference`,
`tracking`, `deadline`, `invalid_feedback`, `cancelled` and `superseded`; late references also
retain their index and lateness. A raising write is conservatively uncertain,
even when cleanup later succeeds. There is no automatic retry.

Each reference crosses the same native episode gate and complete owner envelope
as `Run.step`, including configured cross-part checks. Gate-selected substitutions
are applied normally and end this buffer as superseded. Direct steps and another
buffer return Busy when their addressed scope overlaps active playback. Run exit always drains,
including an already terminal episode, before site teardown may park or close
hardware. Unconfirmed drain retains site ownership. Hardware watchdogs and
physical stop measurements remain required; draining a callback is not physical
stop proof. A connector must supply its explicit disconnect cancellation policy;
this local API does not observe network liveness.

Binding API 14 introduced retained cleanup receipts; the optional additions
above require their separately listed binding APIs and capability facts. The
current Python/native binding is API 23. Install matching SDK/media wheels from
one revision in an idle environment; never replace a running hardware owner's
native module to test this API.

## Failure and evidence

Stale starts, expired deadlines, late references, invalid feedback, mismatched
acknowledgements and cancellation latch terminal errors. There is no skip-ahead
or retry. An outstanding unacknowledged write stays uncertain rather than being
offered again. Merely exhausting the buffer establishes neither arrival nor
physical task success. `dispatched()` counts confirmed receipts only.

`waddle-runtime/tests/finite_playback.rs` exercises accounting, declared-space
admission and episode drain. Native worker tests in
`waddle-runtime/src/playback/worker_tests.rs` use an advancing fake clock, the
real native gate and callback barriers to check execution, stop ordering,
supersession, retained faults and shutdown. No hardware or remote planner is
involved. These tests do not establish device timing, WAN performance or physical
acceptance.
