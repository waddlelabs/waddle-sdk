# Session and lease lifecycle

The Python API never hands a lease token to application code. `waddle-core` owns the
claim, lease, handoff, and timeline machines; `SiteSession` and `Run` expose their
observable results. The exact transition and guard rules are in the
[normative FSM](core/fsm.md).

## From configuration to an open session

```python
import waddle_sdk

site = waddle_sdk.load_site("site.yaml")
with site.open() as session:
    with session.run(task="inspect the fixture", actor="policy") as run:
        while not run.done:
            observation = run.observe()
            action = policy(observation)
            result = run.step(action, observation)
            if not result.dispatched:
                run.hold(result.detail or "command withheld")
                run.finish("abort", result.detail or "command withheld")
                break
            if task_complete(observation):
                run.finish("success")
```

The boundaries have distinct effects:

| Operation | Hardware effect |
|---|---|
| `load_site()` | Validates configuration and confined paths; opens nothing |
| `site.open()` | Returns an unopened context; opens nothing |
| Enter the site context | Constructs and opens drivers/cameras, then starts the runtime and recording |
| `session.run()` | Returns an unopened run context |
| Enter the run context | Starts one episode through the native state machine |
| Exit an unfinished run | Terminates it with `abort` |
| Exit the site context | Finalizes recording, stops pumps/runtime threads, and closes hardware |

Adapter modules and part factories must therefore remain non-opening. The actual bus or
camera connection belongs in a `Rig` arm/camera builder called while the site context
is entered.

## Grant, claim, lease, envelope

These names are not interchangeable:

- A **grant** is permission for one callable the opened session actually registered.
- A **claim** assigns the episode to a claimant.
- A **lease** is the whole-robot, single-writer right to actuate.
- An **envelope** is the site owner's hard-safety gate chain.

Posture determines which control verbs a rig registers. A supervised rig registers
send, hold, and e-stop; a monitor rig registers only e-stop. Posture does not decide
who may act. The native authority machinery makes the same claim and lease decisions
for every frontend.

## Normal run: the caller holds the lease

With no engaged claim, `Run.step()` submits the caller's action to the native gate. A
pass decision then crosses the owner envelope and reaches the addressed driver.
`SubmitResult` reports what happened:

- `dispatched` is true only if a complete command reached the driver path.
- `gate` names the native decision, or `owner_refusal` when the selected command
  failed the envelope.
- `part` identifies a part-addressed native decision when present.
- `detail` carries a concise public reason where the Python layer has one.

The envelope validates width, finiteness, joint limits, per-command travel, optional
workspace bounds, and configured static/body-collision rules. It rejects the complete
target and moves none of the addressed set. It never clamps.

## Engage: fixed hold-first handoff

The current public SDK has no handoff policy selector. Handoff is always **hold-first**:

1. Native core requests the registered hold verb.
2. The driver path must report a successful hold.
3. Only then may the lease move to the claimant.
4. If hold fails, the lease does not move; the protocol never creates two writers.

While another stream holds the lease, a caller's `Run.step()` can return a substituted
native action or no dispatch. The caller must treat the result as authoritative and
must not write around the SDK. If a caller loop stalls during an intervention, native
core may enter bypass and drive the same registered send path from its owned thread.
That path still crosses the `Arm` envelope; bypass is about liveness, not safety or
authority escape.

Release stops the intervention stream before the lease returns to the caller. Retake,
reset-window, and terminal ordering remain native FSM behavior. The Python binding
does not mirror those state machines.

## Hold and e-stop

`run.hold(reason)` and `session.hold(reason)` request the core-owned energized hold
path. A reason must be non-empty and becomes a public event. Hold does not clear an
owner-side e-stop latch.

`session.estop(reason)` requests e-stop through native core and records the public
reason. Adapter `Driver.estop()` must latch locally; later writes remain refused until
the site operator follows the adapter's explicit recovery path and calls its
owner-side `re_enable()`. `re_enable()` is not a protocol resume grant.

The physical stop and its latency remain the site owner's responsibility. Neither a
Python call nor a network path replaces it.

## Finish and cleanup

`Run.finish(outcome, reason="")` accepts `success`, `failure`, or `abort`. Exiting a
run without a terminal result records `abort`, including when the body raises. A
session allows only one active run.

On site exit, the SDK stops its service thread, finalizes the active run if needed,
finalizes recording, and closes the rig. Rig and camera construction is unwind-safe:
if a later part fails to open, every earlier opened resource is closed before the
error leaves the context.

## What is normative

This page explains the current Python view. The
[FSM](core/fsm.md)
defines exact episode, claim, lease, handoff, gate, reset, partition, and terminal
behavior. The
[glossary](core/glossary.md)
defines the terms, and the
[versioning rules](core/versioning.md)
define how peers negotiate additive behavior. Those documents win if an explanatory
example here ever disagrees.
