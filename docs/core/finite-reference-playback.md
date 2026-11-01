# Finite reference playback

## Optional command history

`PlaybackHistoryPort.history()` returns a detached diagnostic copy of the latest
256 playback callback attempts. Read it separately from `snapshot()` so ordinary
status polling does not resend the journal. The optional device relay exposes
the same data through `playback_history`, bound to the retained run and handle.
Older handles may not provide this optional method.

The `waddle.playback-history/v1` document identifies the run, native execution ID, joint ordering, total
and dropped counts, and each requested reference with its measured input. Start,
return and read stamps carry SDK session-monotonic nanoseconds and their epoch
twins captured by the native clock. A pending call has no returned stamp; errors
retain their original diagnostics. Superseded calls do not acknowledge the
requested reference. Cancellation drains an admitted call before normal Hold.

Each stamp also includes a separate `host_monotonic_ns` sample taken in the SDK
process. This permits comparison with optional camera content intervals from that
same SDK host and run. It does not convert session time, synchronize hosts, or
establish an exposure interval for cameras that do not supply one.

This records command calls, not physical arrival or camera exposure. Cleanup
writes and another supervision stream are excluded. Journal truncation is explicit.
Do not compare these session timestamps with another host clock or infer missing
commands from cloud polling; camera alignment requires the same clock identity.
The recorder makes no execution decisions and introduces no additional worker.

## Measured starting position

Binding API 41 adds `execution.expected_start_playback` and optional
`PlaybackPolicy.expected_start`. Supply a finite vector in the complete declared
joint order when the expected measured starting position differs from the first
command. The worker compares fresh feedback against that vector using the
original `indices` and `start_tolerance`; omission retains the first-command
comparison. Admission still occurs locally when a queued motion activates.

This vector is never commanded. It does not waive target limits, tracking bounds,
step limits, deadlines, or normal Hold on refusal. A caller planning bounded
recovery from an out-of-limit measurement must still provide legal commands and
establish the recovery path. Planning and collision proofs remain outside the SDK.
Check the separate support fact before supplying this option. Default policies
omit the field on the device relay, preserving their existing wire shape.

## SDK-owned queue handoff

Binding API 32 adds `execution.dependent_playback`. A caller can submit a checked
motion before its predecessor completes. The existing local worker waits for
the explicitly accepted result, checks the new motion's measured start, then
executes and completes it without another cloud request:

```python
from waddle_sdk.runtime import PlaybackDependency

first = run.start_playback(first_motion)
second = run.start_playback(
    second_motion,
    after=[PlaybackDependency(first, ("arrived",))],
    start_before_ns=latest_start_in_sdk_session_clock,
)
result = second.wait()
```

Dependencies must belong to this SDK instance and run. Accepted outcomes are
`arrived`, `settled`, `contact_stall`, `force_reached`, or
`references_dispatched`; choose only outcomes that justify the next motion.
Dispatching references alone does not establish arrival. Failed or unaccepted
predecessors produce `dependency_failed` without issuing child commands.
Original predecessor failure and cleanup evidence remain on that handle.

Binding API 40 adds `execution.transit_playback`. An admitted finite successor
that explicitly accepts `references_dispatched` can waive its predecessor's
intermediate settling. It must have exactly that predecessor as its dependency,
address the same named arm, remain uncancelled and meet its start deadline.
Gripper policies, other-arm and coordinated boundaries do not qualify. Both
references must end at rest. The predecessor must finish all references, retain
fresh feedback/tracking checks and satisfy its required goal tolerance; a report
allowing stationary misses can cross without measured arrival. Its result stays
`references_dispatched`, with the chosen successor ID in `transit_successor`.
The successor still performs ordinary measured-start checks and final settling.
Cancellation before this local decision removes the waiver. No cloud finish call,
second queue, additional worker or replay is introduced.

`waiting_for` contains predecessor IDs until activation. Status stays `running`
while queued; `dispatched` remains zero. The queue reserves all selected parts,
so overlapping work must depend on the complete preceding reservation chain.
Unrelated named parts may continue independently. There may be at most 16 waiting
dependent motions and 16 predecessors per motion. No planner, extra worker, or
automatic retry is added.

`start_before_ns` is optional and uses acquired SDK session time, never the
cloud clock. If it expires before activation, the result is `start_expired`.
Motion duration and completion allowances begin with activation. An existing
watchdog keeps its original absolute deadline even while queued. Cancelling or
rejecting a motion before activation does not Hold its running predecessor;
after activation, ordinary measured-start checks and cleanup apply. Site-wide
Hold/e-stop and owner closure still drain active and queued work together.

Metal remains responsible for IK, collision proofs, accepted predecessor outcomes
and plan validity. Queue admission does not validate stale world geometry or
authorize a motion after an unknown response; reconcile the retained handle and
never replay an uncertain submission. Existing calls without queue arguments
retain their behavior. Check the separate support fact before using this option.

## Input-limited reference streams

Binding API 26 adds `execution.streamed_joint_playback`. Set
`PlaybackPolicy.timeout_ns=None` only for an explicit reference stream, with
`reference_only=True`, `allow_curve_updates=True` and a `PlaybackWatchdog`.
Pacing credit is not applicable. The watchdog interval must be positive and at
most 180 seconds; callers should choose the required input-expiry interval.
Each curve retains its existing 120-second bound, ordinary tracking/owner gates
and exact interpolation, and must end with zero velocity and acceleration.

After the endpoint is acknowledged, status stays `running` and
`curve_progress.endpoint_acknowledged` is true. Fresh feedback and tracking
supervision continue on the existing worker; no arrival or task success is
implied. A revision-checked update may resume from that exact resting endpoint.
Moving handoffs keep the existing future-boundary PVA checks. If planning is
late, the incumbent reaches rest; no unchecked continuation or replay occurs.
The stream's cumulative phase may exceed 120 seconds, while only its incumbent
and single pending finite curve are retained. Withdrawing a pending resting
continuation restores the supervised resting state.

Only accepted watchdog renewal extends input lifetime. Status polling and curve
updates cannot do so. Cancellation, expiry, faults and disconnect retain ordinary
Hold/drain and unknown outcomes. A stream cannot revive after termination.
Integer `timeout_ns` values keep finite execution, the original deadline and
`references_dispatched` completion unchanged. Check support before selecting the
optional stream policy; application input handling and physical stopping still
require their own qualification.

## Optional playback watchdog

### Input arrival watchdog

Named single-arm and composite sites advertise `execution.part_hold_playback`
when their ordinary action/Hold grants permit the existing local callback.
An individual buffer cancellation may retain its live input only after a
successful **scoped** Hold. This supports replacing checked trajectories during
one still-active press. It does not renew the input or change its deadline.
Failed cleanup, other execution failures and global Stop fence the input;
release, expiry and run closure remain permanent even if scoped cancellation
finishes later. A completed Hold receipt is not measured physical stationarity.

The Rust `Session.start_playback_input(episode_id, timeout_ns)` creates a
run-bound `PlaybackInput` before planning starts. Attach it as
`PlaybackQueue.input` when submitting the checked references, leaving
`PlaybackPolicy.watchdog` unset. Creating or renewing an input commands no
hardware and adds no execution thread.

Binding API 42 exposes the same handle through public
`InputPlaybackRunPort.start_playback_input(timeout_ns)` and
`PlaybackInputPort`, under `execution.input_watchdog_playback`. Pass that handle
as `Run.start_playback(motion, input=control)`; it must belong to that exact run.
The device relay retains input handles by run and ID. Input renewal bypasses
ordinary I/O admission; release additionally uses the existing reserved Stop
capacity. Neither method waits for feedback, planning or execution-status reads.

```python
control = run.start_playback_input(250_000_000)
# Each newly received human input may call control.refresh(next_sequence).
# Do not renew from an automatic timer or from status/planning activity.
handle = run.start_playback(checked_motion, input=control)
control.release("operator released")
result = handle.wait()  # inspect normal cleanup before claiming completed Hold
```

The caller remains responsible for forwarding input while planning and admission
are pending, and for releasing a late creation reply after the press has ended.
Creating this SDK primitive alone does not establish that a cloud application
has implemented those paths or passed actual WAN qualification.

`refresh(sequence)` records acceptance on the SDK's session clock. Supply a
strictly increasing sequence for each newly accepted input message. Repeating
or reordering a sequence returns the retained state without extending its
deadline. Planning, feedback and status polling never renew it. This allows
input to arrive while a planner or execution-status response is delayed.

The timeout applies from **SDK input acceptance**, not physical button time or
cloud feedback time. It therefore cannot bound internet transit delay. Expiry
is checked before every renewal; a late message cannot revive an expired input.
`release(reason)` permanently fences that identity, including plans that have
not arrived. Site-wide Hold/e-stop, run closure and session shutdown also fence
retained inputs. A new press requires a new input. The native registry retains
at most 128 live inputs and reclaims expired, released or dropped entries.

The existing execution worker checks the bound input before I/O and after
observation and performs its ordinary Hold/cleanup on release or expiry.
In-flight callbacks still drain; input release is not completed stop or proof of
physical stationarity. Original motion deadlines, measured-feedback checks,
owner limits, cancellation and fault evidence still apply. Admission requires
the same SDK and run; a bound input cannot also be renewed through
`ExecutionHandle.refresh_watchdog`.

### Feedback-stamped watchdog

Binding API 25 adds optional `PlaybackWatchdog(observed_ns, timeout_ns)` under
`execution.watchdog_playback`. The existing serialized native worker checks
expiry before I/O and after observation; expiry retains `watchdog_expired` and
ordinary selected Hold/cleanup. `WatchdogPlaybackHandlePort.refresh_watchdog`
queues an acquired SDK feedback stamp, with pending/accepted/rejected evidence.
Only the native worker accepts renewal. Stale/future stamps refuse, duplicates
cannot extend time, and delayed requests cannot revive an expired buffer.
For paced curves, the worker also samples between command ticks when their
interval exceeds half the watchdog timeout. This supplies fresh feedback for
accepted input without increasing the curve's write rate. Sampling and status
reads alone never renew the watchdog.
No motion timeout extension, authority, clock or writer is added. Blocked callbacks still drain
before Hold; this is neither physical-stop proof nor a complete teleoperation
input policy. See the finite-playback contract and watchdog tests.

A watchdog requires `observed_ns >= 0` and `0 < timeout_ns <= policy.timeout_ns`
(or at most 180 seconds for an explicit stream),
without integer overflow. Its initial stamp and renewals use this SDK session's
observation clock, never the caller's clock or request-arrival time. The worker
checks the incumbent deadline before applying a pending renewal, including after
blocked observation. A blocked write can finish, but no next reference follows
expiry; its original write failure takes precedence if it fails.

`snapshot()["watchdog"]` retains accepted `observed_ns`, `timeout_ns` and
`deadline_ns`. `watchdog_refresh` retains the requested stamp, status and accepted
deadline or refusal text. Only one renewal may be pending; repeat submission of
the same stamp is idempotent. Refused future stamps do not prevent a later valid
renewal. A terminal buffer cannot renew. Original motion timeout, pacing credit,
curves and ownership remain unchanged. Default `None` adds no liveness check.
The caller still owns its human-input/deadman policy and must stop renewing when
that input expires; this primitive does not interpret human input.

```python
from dataclasses import replace
from waddle_sdk.runtime import PlaybackWatchdog

# motion is an already validated FiniteJointMotion; session/run are already open.
observed = session.observe()
motion = replace(motion, policy=replace(
    motion.policy, watchdog=PlaybackWatchdog(observed.session_ns, 250_000_000)
))
handle = run.start_playback(motion)
# While application input remains valid, use newly acquired feedback timestamps:
# handle.refresh_watchdog(session.observe().session_ns)
# Reconcile watchdog_refresh in the retained snapshot; never assume queueing
# extended the deadline. Expiry needs a new explicit application action.
```

`Run.set_playback_bounds` supplies optional run-wide joint envelopes through
`BoundedPlaybackRunPort` and `execution.joint_playback_bounds`. Its mapping binds
declared part names to `JointPositionBounds(lower, upper)` in each part's full
joint order and position units. Installation validates and copies the complete
mapping at the normal dispatch boundary. It performs no motion or hardware read.
An empty mapping removes the envelope; a new Run starts without one. Native
binding API 24 is unchanged.

While any envelope is installed, the existing local worker reads every bounded
part before playback feedback and again before each run command dispatch. It checks
measured positions and the final gate-selected targets against inclusive bounds.
This includes direct run commands and supervision substitutions. A missing
or failed required read, nonfinite value, or range violation requests the ordinary
site-wide Hold, including when individual buffers selected `part_hold`. Retain
the original scoped fault and the completed global cleanup receipt separately.
Direct run refusals use `SubmitResult.gate == "playback_bounds_refusal"`: their
Hold request is queued, not a completed owner-Hold acknowledgement.
This is a shared dependency failure; unrelated ordinary part failures retain
their existing scope when no shared envelope is installed.

The caller owns geometry/planning, serializes envelope replacements, and waits
for configuration acknowledgement before dependent admission. An uncertain reply
leaves configuration unknown; it does not authorize continuing dependent motion.
Replacing bounds does not change an accepted curve, its clock or timeout. The
envelope never widens existing site/owner limits. Explicit control verbs retain
their takeover and drain semantics; they are not trajectory references. A successful setter is neither a collision proof
nor evidence of physical stopping.

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

Binding API 27 additionally supports an ordered subset under the separate
`execution.coordinated_subset_playback` fact. Select at least two distinct declared
joint parts; all rows use only their concatenated layout. Native intake validates
the complete selected layout and its slowest command cadence. The group reserves
all selected members until terminal cleanup, while disjoint named buffers may run
on the same serialized worker. Normal completion does not write or hold unrelated
parts. Configured shared bounds and neighbor-collision checks remain dependencies.
Each selected part crosses the existing native gate before the complete decision
crosses all owner envelopes; a takeover replaces the group without a partial caller
write. Group failure/cancel still uses site-wide Hold, and group `part_hold` remains
unsupported. Neither parallel scopes nor shared cadence imply physical atomicity.

Binding API 22's original `CoordinatedJointMotion(parts, references, policy)` uses the
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
The worker schedules fresh feedback reads within the cadence interval, using
the read cost observed during that execution. Faster reads cannot dispatch early;
slower reads still consume the original timeout. This does not cache observations,
renew input liveness or change the owner's dispatch checks.
The optional curve does not provide dynamic world updates, contact detection,
force-grasp completion or coordinated parts.

### Optional pacing credit

Binding API 21 adds `PlaybackPolicy.credit_pacing_delay`, requiring
`execution.paced_playback_deadline` and reference-only polynomial playback.
Binding API 38 also permits explicit-goal polynomial playback with the same
accounting; target messages and goal dwell never earn additional credit.
Binding API 39 also permits revisable settled playback, with the same accounting.
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
`execution.revisable_joint_playback` fact and paced execution. Reference-only
buffers retain their existing contract. A buffer with an explicit measured goal
additionally requires `execution.revisable_goal_playback` (binding API 33).
Binding API 39 permits `allow_settled=True` without an explicit goal under
`execution.revisable_settled_playback`. Native completion scores the accepted
curve's final joint positions, then reports arrival or a stationary miss after
the full endpoint is acknowledged. It never completes in an intermediate rest.
The caller may use either settling method; `settle_ns=0` allows one fresh
post-acknowledgement stationary velocity sample, while position windows preserve
their span and sample count. A checked replacement retains the same worker,
execution identity, deadline, tracking limits and cancellation behavior.
This mode excludes checkpoints and requires a resting endpoint on each curve.
Gripper outcome policies, exact sample grids and `toward_goal` velocity hints
cannot be combined with updates. Checkpointed goal buffers accept updates only
after all original markers have been observed; reference-only buffers still
exclude checkpoints. Coordinate order, held targets,
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
velocity peak and consecutive qualifying count. Binding API 29 adds `observed_ns`,
identifying the SDK sample used to update the window, including empty resets.
Feedback and its window diagnostics publish together. Its time source is the SDK
observation timestamp, not a claim about motor acquisition timing. Contact/force
classification, geometry/world validation and dynamic updates remain separate.

`execution.reference_settling` permits these optional diagnostics during
reference-only playback, including its final fresh sample. Select the ordinary
feedback position-window policy and its explicit drift/gap bounds. `during_motion`
and `allow_settled` retain their normal eligibility meanings. The result remains
`references_dispatched`; a consumer must separately establish a fresh window,
the required sample count/span, drift bounds and its actual goal. Window diagnostics
never turn a reference receipt into an arrival or physical-stop guarantee.

### SDK-owned goal completion

Binding API 31 adds `PlaybackPolicy.goal` and `settle_timeout_ns`, gated by
`execution.goal_playback`. A `PlaybackGoal` carries exactly one of:

- `joint_positions`: the original target in selected-index order, evaluated with
  the existing per-joint `position_tolerance`. A corrected command endpoint can
  differ from this target.
- `pose`: a TCP pose in the SDK owner's base frame. Position uses
  `position_tolerance_m`; rotation uses `orientation_tolerance_rad`, or explicit
  `None` for position-only completion. This additionally requires
  `execution.cartesian_goal_playback` for the owner's default TCP, derived from
  opened local FK and a frame. Missing/wrong-frame feedback fails through normal
  local cleanup.

For a different selected tool, `execution.kinematic_goal_playback` accepts
`PlaybackGoal.kinematics`: a `PlaybackKinematics` reference TCP pose plus at most
128 `PlaybackKinematicJoint` entries, ordered from base to tool. Each identifies
a local measured joint index, hinge/slide axis, hinge origin, scale and offset.
The reference pose, axes and origins use the part's base frame at the same
reference configuration. The scalar displacement is `scale * measured + offset`;
the caller includes any reference-joint subtraction in that offset. FK composes
these scalar transforms in order before the reference pose. A slide translates
along its axis; a hinge rotates about its axis through its origin.

The existing worker evaluates this immutable numeric chain from the same local
joint sample used for settling and retains that TCP pose in feedback. It needs
no optional model engine or remote callback. The selected goal geometry does not
change owner workspace, collision, joint or dispatch gates. The caller remains
responsible for binding the chain to its selected robot/tool model. Other parts'
coordinates cannot be frozen into a local part's chain as if they were measured.
Omitting `kinematics` preserves the original declared-TCP wire contract.

The same worker evaluates arrival and settling for each fresh local sample.
`arrived` means the requested goal satisfies the configured settling policy;
`settled` is a separately allowed stationary miss, not successful arrival.
Failure/cancellation retains the original error and cleanup receipt. No caller
polling, remote arrival decision or `finish_references` request is needed.
Delayed status reads return the immutable outcome rather than rerunning checks
against the caller's clock. Planning a correction is a caller decision and
requires a checked trajectory.

With `execution.revisable_goal_playback`, the caller may opt into curve updates
on this same execution. The original goal, result identity and total deadline
remain fixed. A checked correction can continue from the acknowledged resting
endpoint, preserving position, velocity and acceleration there. Initial and
replacement curves must end with zero velocity and acceleration. A pending
accepted correction must finish before goal completion; merely observing the
goal cannot discard accepted future work. Withdrawing that pending correction
restores the incumbent's completion rules. Expired executions cannot be revived
by a delayed update, including one queued during an observation callback.

Binding API 38 adds `execution.updatable_goal_playback`. Set `goal.allow_updates`
explicitly on a pose goal with independent dwell, then call
`handle.update_goal(PlaybackGoalUpdate(expected_revision, pose))`. Revision zero
identifies the admitted target. The same local worker validates the current
revision and the unchanged base frame before accepting a new pose. It changes no
trajectory, FK, tolerance, stationarity policy, deadline or authority. Independent
dwell history persists; the next fresh goal score resets it only outside the
configured outer band. Target resolution and trajectory planning stay with the
caller.

Only one goal request may await processing. The returned request ID identifies
`goal_update`, which records the requested pose, expected revision and pending,
accepted or rejected result. Accepted receipts include the new revision.
`goal_target` contains the accepted pose/revision used for retained feedback and
is published with that feedback and dwell progress; it is absent before the first
sample. A queued update may lose to cancellation, failure or completion and be
rejected. Report the target retained in the terminal snapshot, never a newer
unaccepted intent. Delayed updates cannot renew either deadline or change a
terminal result. Reconcile a lost response from the same handle; do not replay.
Default fixed goals keep their previous behavior and wire encoding.

Binding API 37 adds optional `PlaybackGoalDwell(duration_ns,
exit_tolerance_multiplier)` under `execution.dwell_goal_playback`. Goal dwell
starts inside nominal tolerance, survives small crossings, and resets outside
the selected outer band. Arrival still requires current nominal tolerance and
independent stationarity. For example, a 300 ms dwell with multiplier 2 preserves
the timer through slight threshold jitter without declaring arrival outside the
nominal band. SDK observation time drives the timer, even during motion; measured
motion still prevents completion. Snapshots expose `goal_dwell.observed_ns`,
`since_ns` and `restarts` separately from encoder-window evidence.

With explicit positive goal dwell, `settle_ns=0` permits immediate velocity-based
stationarity; minimum sample counts and encoder-window bounds still apply. Default
goal policies retain their full in-tolerance settling interval. Goal dwell does
not alter trajectories, deadlines or cancellation and does not itself update a
moving target. Metal's target resolution and correction strategy remain external.

Binding API 36 adds opt-in `complete_in_resting_tail` under
`execution.resting_goal_playback` for revisable paced goals. Once every remaining
polynomial segment is constant, its resting reference has been acknowledged and
all requested checkpoints have been observed, the local worker can settle the
goal without dispatching the rest repeatedly or waiting for a remote finish
decision. A pending accepted suffix must execute or be withdrawn first, even
when the current measured pose already satisfies the goal. Goal loss, feedback
failure, cancellation and the original deadline remain authoritative. Default
false preserves the complete requested reference duration. A completed result
can therefore report `arrived` while `curve_progress.endpoint_acknowledged` is
false: the unused remainder contained only constant rest, not undispatched motion.

With `execution.playback_timing` (binding API 35), `elapsed_ns` is the duration
from native activation to the retained feedback sample. It is `None` while queued
or when an execution ends before activation. Queue wait and cloud reply delay do
not enter this duration; a terminal snapshot retains its final value. Planners
can subtract it from their supplied execution budget without comparing clocks.
It does not renew the local deadline or establish freshness of a delayed sample.

With `execution.goal_progress` (binding API 34), snapshots expose `settled_miss`:
the latest fresh local sample completes the configured settling test outside the
original goal. It is progress, not a terminal result or permission to move. Both
velocity dwell and position-window checks use the original local sample count,
drift and gap limits. Motion, stale feedback or loss of that settling evidence
clears the signal; reading status never renews it.

Set `allow_settled=False` to keep a stationary miss open for correction within
the original deadline. The SDK continues local supervision and measured goal
completion while the caller plans. A dependent buffer waits for this execution's
final accepted outcome across all corrections; no remote finish or handoff is
required. Original checkpoint evidence is retained and never repeated. A caller
must reconcile accepted revisions and keep its geometry proof valid; the SDK
adds no planner. Without this support, a correction is a separately admitted
execution, so dependents must not mistake an intermediate result for final arrival.

`settle_timeout_ns`, when supplied, bounds observation after the last reference
acknowledgement. It cannot extend the original finite motion timeout. Omitting
these fields preserves the existing joint-endpoint completion and wire format.
Explicit goals cannot be combined with reference-only or scalar gripper policies;
gripper completion retains its existing local policy.

### Deferred reference completion

Binding API 30 adds `PlaybackPolicy.defer_reference_completion`, gated by
`execution.deferred_reference_completion`. It requires reference-only paced
execution with a finite timeout, no curve updates and no gripper outcome policy.
After the fixed curve's endpoint is acknowledged, the same worker retains the
reservation and continues local observation and optional window diagnostics.
It sends no further references. This lets a remote planner score its goal using
locally collected evidence without treating WAN polling intervals as sensor gaps.

Call `DeferredPlaybackHandlePort.finish_references()` on that retained handle
after endpoint acknowledgement, then wait for the terminal snapshot. Request
acceptance alone is not completion. The worker checks fresh feedback, tracking,
window diagnostics and the unchanged deadline before returning
`references_dispatched`. Final drift or a sample gap remains in the returned
window; finishing does not certify arrival or stationarity. Early finish refuses;
repeating a successfully completed finish is idempotent and cannot replay motion.
Failed or cancelled buffers cannot be revived.

Cancellation, global controls, run closure and failures retain normal local
Hold/drain. The original timeout bounds deferred observation; a missing finish
does not create an indefinite reservation. A correction requires confirmed
completion of the previous buffer before admitting another. The default is
false and is omitted on the relay wire for older receivers.

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
The contact sample count, velocity-stationary sample count and local
contact-compatibility decision are retained on both success and failure. These
are diagnostics from the existing observation window; they do not change
completion or cleanup.

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
Binding API 28 adds the optional `control.completed_stop` support fact and
`CompletedStopPort.stop(reason, estop=False)`, implemented by `SiteSession.stop`.
It waits for a fresh registered Hold/e-stop callback on the existing serialized
worker and returns `request`, `outcome`, `error`, `completed_ns` and
`measured_stationary: false`. It also works after a buffer is terminal; the old
buffer's result and cleanup remain immutable. A request made during an existing
callback waits for the next callback. A callback failure returns `unknown` with
the original error. Transport loss or a caller timeout does not establish cleanup;
retain ownership until a completed receipt or independently verified owner closure.
The receipt proves callback completion, not measured physical stationarity.

Site-wide cleanup and completed-stop receipts retain optional structured
`evidence` supplied by the owner callback, matching scoped Hold diagnostics.
Python `RuntimeFault` code, detail, retryability, context and causes survive this
boundary. All declared parts receive Hold/e-stop despite an earlier failure;
a single fault passes through unchanged, while multiple failures retain each
part and complete fault. Unknown cleanup never becomes confirmed by recording
these diagnostics.

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
current Python/native binding is API 26. Install matching SDK/media wheels from
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

When a paced-curve feedback read finishes just before its due write, the worker
retains that sample only through the short remaining cadence wait (bounded by
the observed read cost and sampling interval). It keeps the original observation
stamp and rechecks cancellation, owner and watchdog state before dispatch. A late
wakeup reacquires feedback; gripper and long cadence waits continue sampling.
