# Finite reference playback

`waddle_runtime::playback` accounts for a fully received, bounded sequence of
joint-position references on a local session clock. `Session.start_playback`
executes an admitted buffer on the existing serialized `waddle-verbs` thread.
**The Python Site/Run API does not yet expose it, and no current connector
automatically uses it.** It opens no device and creates no additional thread,
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

Each reference uses hold interpolation. This module never fits a new curve,
rescales time, drops a reference or substitutes a new target. Applications must
prepare and validate the exact reference sequence they intend to execute.
The intervention jitter buffer retains its existing latest-wins semantics;
finite reference accounting does not change that stream.

`PlaybackPolicy` supplies positive start, final-position and stationary-velocity
tolerances for selected moving rows. Other rows remain in the full command and
still cross the owner's envelope; for example, a held jaw's contact shortfall
need not become an arm arrival condition. The policy also supplies a positive
settling interval, a total timeout of at most 180 seconds, and permitted reference
lateness of at most one second. These are caller-selected bounds, not measured
hardware performance guarantees. No default widens an owner's limits.

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
one joint-position scope, including one named part of a composite robot. It
refuses a whole composite space, mixed scopes, inert actions and gripper side
channels. A registered Hold callback is required. One reference loop can be
outstanding; another admission or an outstanding control/safety request returns
`Busy`. This is execution serialization, not an additional actuation lease.

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
and owner measurements remain separate evidence. Public Python owner adaptation,
support facts and connector transport integration are still pending.

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
