# Finite reference playback accounting

`waddle_runtime::playback` accounts for a fully received, bounded sequence of
joint-position references on a local session clock. It is the native foundation
for buffered execution. **The Python Site/Run API does not yet expose it, and no
current connector automatically uses it.** It opens no device and creates no
thread, claim or lease.

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

The execution owner must keep local observation and playback advancing without
network requests. It must cancel/drain on run closure, preserve original hardware
faults and apply the ordinary Hold/e-stop behavior. These owner and public-API
integration points remain separate work; this accounting type alone is not a
supervised execution service.

## Failure and evidence

Stale starts, expired deadlines, late references, invalid feedback, mismatched
acknowledgements and cancellation latch terminal errors. There is no skip-ahead
or retry. An outstanding unacknowledged write stays uncertain rather than being
offered again. Merely exhausting the buffer establishes neither arrival nor
physical task success. `dispatched()` counts confirmed receipts only.

`waddle-runtime/tests/finite_playback.rs` exercises these rules with explicit
timestamps, without wall-clock races or hardware. It does not establish device
timing, network performance or physical acceptance.
