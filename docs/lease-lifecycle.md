# The lease, from your loop's point of view

Every supervision story Waddle tells turns on one object: the **lease**, the
single-writer right to command the robot. Claims, handoff policies, reset
windows and agent-invited episodes all exist to move that one right between
your program and whoever is supervising it, and to leave a record of each move
afterwards. This page follows the lease through a whole session from your side
of the API: who holds it in each phase, what `ep.gate()` hands your loop while
they do, and what the SDK guarantees and does not guarantee at each step.

The normative rules are in
[`waddle-protocol/docs/FSM.md`](../waddle-protocol/docs/FSM.md), and the guard
rows cited below (E7, C6, L6 and friends) are pointers into it. Where this page
and FSM.md disagree, FSM.md wins; where a word is in question,
[`GLOSSARY.md`](../waddle-protocol/docs/GLOSSARY.md) wins over both.

## Four words that are not synonyms

A **grant** is a permission you extend to Waddle. In the Python SDK you never
write a grant down: it is derived from which verbs you register on
`waddle_sdk.Control`, one grant per callable. Register `send` and you have said
"something other than my policy may command this arm"; leave it out and you
have said the opposite, on the wire, to everyone.

A **claim** is orchestration: who has been assigned this episode. A claim is
requested and granted by the supervision plane (or granted locally by a
teleoperator's clutch edge, which both requests and grants in one step), it
names its claimant, and it is the thing that survives a retake.

A **lease** is actuation: who may write to the robot right now. It is whole
robot and single writer, always. Winning a claim leads to acquiring the lease;
a takeover is a lease handoff under a claim that already exists (FSM.md §3).
There is never a moment with two writers: the outgoing writer is stopped before
the lease moves, and a failed handoff leaves both stopped rather than both
running.

An **envelope** is your hard safety gate chain: joint limits, keep-outs,
watchdogs, and the physical stop somebody can hit. Waddle is always subject to
it and is **never the provider of it**. Nothing on this page changes that, and
no lease, claim or grant can bypass it.

## The map

| Phase | Who holds the lease | What `ep.gate()` returns to your loop |
|---|---|---|
| Rollout (RUNNING, no claim) | your loop | `Pass`: your exact action object |
| Intervention (claim engaged) | the claimant | `Substitute` or `Blend`: a fresh float64 array, or `None` on a hold |
| Intervention with your loop stalled (BYPASS) | the claimant | `None`, and the intervenor's actions go straight to your `send` verb |
| Reset window, pre or post (gate mode RESET) | the reset claimant | `None` |
| Agent-invited episode before any engage | nobody | `None` |

`None` always means the same thing: you must not send this tick. `ep.last_gate`
tells you which decision produced it (`kind` is `"pass"`, `"substitute"`,
`"blend"`, `"noop"` or `"hold"`), and the reason behind a no-op is recorded on
the episode timeline rather than surfaced as a Python attribute.

## Rollout: your loop holds the lease

In an ordinary episode nobody has claimed anything, and the **gate** (the one
point where Waddle touches your loop) is a passthrough. It stamps and records
the observation and action pair, checks local tripwires, consults claim state
and hands your action back unchanged:

```python
with waddle_sdk.rollout(task="fold the towel") as ep:
    while not ep.done:
        obs = get_obs()
        action = policy(obs)
        action = ep.gate(action, obs)
        if action is not None:
            send(action)
```

`Pass` returns your own object, identity preserved, so the fast path costs you
no copy. `ep.last_gate.provenance` reads `"policy"`. Everything is being
recorded whether or not a supervision plane is connected: with
`recording_dir` set you get a sidecar and an MCAP per episode, and
`ep.records_dropped` is non-zero only if the recorder fell behind your loop,
which means training data was lost.

The lease in this phase is yours by default rather than by acquisition. That
is worth stating plainly because of what comes next: when a claimant engages,
the lease moves away from your loop, and your loop finds out by the return
value of `gate()` changing, not by an exception or a callback.

## Intervention: the claimant holds it

An intervention begins when the plane grants a claim on a RUNNING episode and
engages it (FSM.md E7). What happens in the moments around the handoff is
decided by the `handoff=` policy you declared at `waddle_sdk.init`, and it is the
one lifecycle decision that is genuinely yours to make.

`waddle_sdk.Handoff.HOLD_FIRST` is the default and the conservative choice. Waddle
calls the `hold` verb you registered and requires a successful result
**before** the lease moves, so the intervenor starts from rest. This is why
`hold` is not optional once a session has a visible engage path, which means a
wired media plane or a registered `hold` or `send`: the engage would have
nothing to call. Such a session declaring this policy with no `hold` verb is
refused at `waddle_sdk.init`, naming the verb, rather than stalling at the first
clutch press with nothing to diagnose.

`waddle_sdk.Handoff.IMMEDIATE(blend_ms=...)` drops the executing chunk and
cross-fades from the last commanded point into the intervenor's stream over
the blend window, using the interpolation your action space declares.
Provenance flips to the intervenor at blend start, not at blend end: the whole
window is theirs in the record.

`waddle_sdk.Handoff.CHUNK_BOUNDARY(max_wait_ms=...)` lets the executing chunk
finish first, capped by the wait you declare.

Whichever you declare, the lease handoff itself is atomic and mints a fresh
token (FSM.md L6). Your loop keeps ticking; `gate()` starts returning
`Substitute` (a fresh float64 array carrying the intervenor's command) or
`Blend` (the same, with `ep.last_gate.progress` in `[0, 1]` while a cross-fade
is open). You send exactly what you are handed, and nothing when you are
handed `None`.

Two shapes of substitute deserve their own sentence, because both are arrays
that are not the width you expect. A **gripper-only** action arrives as an
empty array with `ep.last_gate.gripper` set: that means "hold the arm where it
is, move the gripper", in your declared gripper units. On a `waddle_sdk.Composite`
declaration (a bimanual cell, say) a substitute arrives as a **dict keyed by
declared part**, `{"right": ndarray}` for an action addressing one arm, or
every declared part for a whole-robot one. The parts absent from that dict are
commanded nothing at all: "move this part, hold the rest". Waddle does not
fill in the unaddressed parts, and you must not either, because passing your
own policy's values through for them would resume the paused policy's
actuation in the middle of somebody else's intervention, recorded under their
provenance.

There is one window where a part-addressed action is dropped rather than
executed: while an `IMMEDIATE{blend_ns}` cross-fade is open and the point it
fades from does not command that same part. A cross-fade needs two endpoints
of the same scope, and manufacturing one would fabricate a trajectory nobody
sent. Your loop sees `None` for the length of the window, the held ticks are
recorded as such, and the next action to come due after the window closes
substitutes normally. A sender that needs instant part-scoped takeover
declares `HOLD_FIRST` or a zero blend; FSM.md §5 states the full rule.

### When your loop goes quiet

An intervention must not be starved by a stalled integrator loop, so if no
`gate()` tick lands for longer than the stall threshold (half a second in
waddle-core today) while a claim is active, the gate enters **bypass**: core
drives the `send` verb you registered directly, from its own thread, and any
late tick from your loop gets `None` so your loop stays coherent as a
spectator. This is the one path on which an intervention action reaches the
robot without passing through `gate()`, and everything a gate return would
have carried rides the direct dispatch instead, the addressed part included.

Bypass is also why `send` is required on any session with a live intervention
path, independently of your handoff policy. The rule is stated once in
waddle-core and enforced at `waddle_sdk.init`.

### Release, and retake

A release mirrors the engage in reverse (FSM.md E8): the intervention stream
stops, your policy is re-primed on fresh observations rather than resuming a
stale pre-engage chunk, the lease hands back to your loop, and the claim
releases. `gate()` returns `Pass` again and the episode is RUNNING.

A **retake** is the other exit. When the intervenor judges the attempt
unsalvageable, they terminate the episode `aborted_retake` and a successor
episode opens under the still-held claim (FSM.md E9, C5). The claim survives
and the lease is not handed back: the point of a retake is to keep the robot
under the intervenor's control across the episode boundary, so the successor
is **born claimed** and starts by resetting the scene. The predecessor's
outcome is never silently folded into a success-rate denominator.

Be aware of what this looks like from Python today, because it is the one
place in this lifecycle where the SDK surface is thinner than the protocol.
Your `ep.done` flips true when the successor replaces your episode, so your
`with` block exits cleanly and does not abort anything. The SDK does not hand
you a handle to the successor, and because a session runs one active episode
at a time, calling `waddle_sdk.rollout()` again while that successor is live
raises `RuntimeError: an episode is already active (one active episode per
session)`. Your next attempt begins once the successor closes.

## Before the episode: pre-reset and the reset window

`waddle_sdk.rollout()` blocks until the scene is reset and, under the default
blocking verification mode, until the reset is verified. That call never
yields an invalid scene; if every configured strategy is exhausted it raises
`RuntimeError` instead.

A pre-reset is either a **scripted hook** you pass as a callable, run locally
in your own process, or a **remote reset window** declared as
`waddle_sdk.TeleopReset(prompt, timeout_s=...)` or
`waddle_sdk.AgentReset(prompt, timeout_s=...)`. A hook borrows nothing: your
process resets your scene and the lease never moves.

A window is a bounded period in which somebody else resets the scene through
the SDK, and it borrows the lease exactly like an intervention does, using the
same claim and lease machinery (FSM.md §1.4). The window opens on entry to
RESETTING and a timer arms. A claim is granted only to the actor kind the
window expects: a teleoperator window also admits a site operator, the
customer-side human physically at the cell, while an agent window admits an
agent and nothing else (FSM.md C6). On engage the lease hands to the claimant
and the gate goes to mode RESET, at which point any tick from your loop
returns `None` and is recorded as a no-op with reason `RESET_ACTIVE`. The
claimant's actions reach your robot through the same `send` verb an
intervention uses.

When the claimant signals complete, the ordering matters and is guaranteed:
the lease hands back and the claim releases **first**, the gate returns to
passthrough, and only then does the reset result apply as if it had come from
a local hook (FSM.md E21). Your loop is holding the lease again before the
episode ever reaches READY, let alone RUNNING. If the window instead runs out
its timer, the lease still hands back and the claim still releases; the
episode aborts, and `waddle_sdk.rollout()` raises (FSM.md E22).

Two honest limits. A declared window needs a connected supervision plane to
grant and complete it, so `waddle_sdk.init(transport=waddle_sdk.Grpc(url, token))` is
not optional here: with no plane declared the window opens and can only run
out its timeout. And a remote window can only be overridden per episode if the
session declared a remote reset for some phase at `init`, because the feature
is negotiated once, when the session is built.

## After the episode: post-reset and the pinned outcome

If you declare a `post_reset`, the episode does not go straight to TERMINAL.
It enters POST_RESET, and the first thing that happens there is that the
terminal outcome is **pinned** (FSM.md E14). Pinned means final and immutable:
the cleanup that follows cannot change it. This is the whole reason the phase
exists. An earned success must not become an abort because a cleanup script
failed or somebody hit the stop during tidying, since that would corrupt the
success-rate denominators the record exists to support.

`ep.done` flips true at that instant, before the cleanup has run, and
`ep.outcome` already reads the pinned value rather than `None`. So the
ordinary loop exits when the outcome is decided, not when the tidying
finishes. Calling `ep.terminate(...)` explicitly blocks through the cleanup;
exiting the `with` block some other way sees the episode already done and does
nothing, so it can never abort an in-flight post-reset. If the cleanup fails,
by a false hook result, an exhausted window, a timeout or a stop, the outcome
still does not move: `ep.post_reset_failed` latches true permanently and the
failure is on the timeline.

A post-reset can be a remote window too, with the same shape as the pre-reset
one and the same ordering guarantee at the end: when a reset claimant holds
the lease, the handback completes **before** the episode transitions to
TERMINAL (FSM.md E15). By the time the episode is over, the lease is back with
your loop. Your next `waddle_sdk.rollout()` waits for the cleanup to finish rather
than racing it.

One asymmetry worth knowing: a retake skips POST_RESET entirely (FSM.md E18).
The successor's own pre-reset handles the scene, and the claim and lease stay
with the intervenor across the boundary.

## Agent-invited episodes: handing over the whole rollout

`waddle_sdk.agent("clear the table and stack the cups")` opens an episode that is
**agent-invited**: you have asked Waddle to drive this one, and the call blocks
until the episode reaches an outcome. An agent here is a hosted actor, never a
component of Waddle; the word means the thing that claims and drives, and
nothing else.

An agent-invited episode is an ordinary episode with exactly two differences
(FSM.md §1.5). Only agent claims are admitted, so a teleoperator cannot quietly
take an episode you asked an agent for. And until an agent engages, your own
`gate()` ticks never dispatch: they return `None`, recorded with reason
`AGENT_EPISODE`. In practice you are not ticking anyway, because your thread is
blocked inside `waddle_sdk.agent()`, which is precisely why the call blocks instead
of handing you an episode handle it would be wrong to use.

That blocked thread is what makes the engaged agent's actions arrive through
bypass. With no gate ticks at all, the first engage trips the stall detector
immediately, the gate enters BYPASS, and core drives the `send` verb you
registered from its own thread. There is no separate agent actuation path: it
is the same claim, the same lease handoff under your declared policy, and the
same verb a teleoperator's intervention would use.

The refusals are deliberately early and loud. If the session declared no
transport there is nobody to ask, and the call raises rather than waiting out
a deadline. If the session registered no way for an agent to actuate (a `send`
verb, plus a `hold` under the default `HOLD_FIRST` handoff), the call raises
with the missing verb named, because an invite that no engage could ever carry
would otherwise stall with nothing to diagnose. A plane that never negotiated the agent feature
is not an error: it simply never routes the invite, and you get the ordinary
abort at the deadline.

Everything else comes back in the result. `AgentResult.outcome` is the pinned
outcome in the same spelling `ep.outcome` uses, including `"aborted_retake"`
when a retake replaced the episode; `episode_id` finds the run in your
recordings; `detail` carries the plane's last word, which is where a declined
task explains itself. A deadline that elapses with nobody engaged, or a task
the plane declines before any engage, both abort the episode cleanly through
the episode's normal termination routing, post-reset included, and both come
back as `outcome == "abort"` rather than as an exception. A denial that
arrives after an agent has already engaged is recorded and ignored: a pinned
outcome is never rewritten by a late message.

## A session that never lends the lease

Not every session should be able to hand the robot to anyone. The `monitor`
posture in [`waddle_sdk.robots`](../sdk/README.md#postures) registers your `estop`
alone: no `send`, so no send grant is declared and the plane has nothing to
plan an intervention against; no `hold`, and no media plane, so there is no
engage path in the session at all. `waddle_sdk.agent()` on such a session refuses
up front with the missing verb named. Watching is undiminished, since
proprioception, low-rate stills and the local archive do not need any of that.

The point is that the session says what it will not do on the wire, instead of
accepting motion it intends to drop. A posture is not an authority decision and
adds none: who may command a robot, when, and under what claim is waddle-core's
answer and is identical under either posture. The
[posture table in `sdk/README.md`](../sdk/README.md#postures) has the full
comparison.

## What binds every phase

**Grants bound everything above.** A verb you did not register is a permission
you never extended, and no phase, claim or feature flag can conjure it. Grant
health is also continuously validated from proxy signals rather than by
invoking your verbs behind your back, and a demotion never interrupts an active
lease: it takes effect at the next planning decision, and it is always visible
as an event (FSM.md §7).

**Advisory enforcement is the honest default.** `lease_enforcement="advisory"`,
the default at `waddle_sdk.init`, records that nothing physically prevents your
loop from writing during a takeover, because the write path is in-process
callables you own. The alternative, `"enforced"`, is for integrations where a
mux or proxy physically owns the only write path. Under advisory enforcement
the planner prefers hold-first handoffs, and the discipline that makes the
lease real is yours to keep: send exactly what `gate()` returns and nothing
when it returns `None`. The protocol's own answer to advisory enforcement is
dual-write detection, sustained divergence between what was commanded and what
proprioception reports. Be aware of where that stands: waddle-core implements
the detector and the conformance suite pins its behaviour, but the live session
this SDK opens does not yet feed it, so the proprioception you report with
`session.report_proprio(...)` is recorded and uplinked rather than compared.
Do not treat an advisory lease as a guard you can lean on.

**Your envelope and your physical stop are the floor.** Waddle never provides
either. A directly initiated human action may bypass a motion-approval gate,
but never the envelope, never the lease and never the e-stop. When a stop does
fire, every lease token is revoked at once and the episode goes terminal, with
the one exception described above: in POST_RESET the pinned outcome survives
and the incident is recorded as a post-reset failure instead (FSM.md E11, E17).

**Parts address, they do not authorise.** A part named in a `Composite`
declaration is an addressing axis on actions and proprioception. Claims, leases
and handoffs stay whole robot and single writer in v0: there is exactly one
holder at any instant, no matter how many arms the declaration names.

**A partition degrades loudly.** If the control plane goes away, the session
does not quietly carry on as though nothing had changed: a sustained heartbeat
loss requests a hold through the `hold` verb you declared and says why on the
timeline, cloud-dependent grants demote with reason `partition` and are
restored on reconnect, and episode events buffer locally and replay in order
once the connection heals (FSM.md §8). Tripwires, the Waddle-side watchdogs
that request holds through your declared verbs, run alongside that in
waddle-core when a session declares them; this SDK declares none of its own
today, so the heartbeat path above is what you get for free.

## Where to read more

[`FSM.md`](../waddle-protocol/docs/FSM.md) is normative and is the place to go
for the exact guard rows: §1 for the episode machine and its reset and agent
extensions, §2 for claims, §3 for the lease, §4 for the intervention lifecycle
and part-scoped actions, §5 for the handoff sub-protocol, §6 for gate modes.
Each row is pinned by at least one conformance fixture, named in the rightmost
column.

[`GLOSSARY.md`](../waddle-protocol/docs/GLOSSARY.md) is normative for the
vocabulary, [`VERSIONING.md`](../waddle-protocol/docs/VERSIONING.md) for how
feature flags such as `waddle.v0.reset.remote`, `waddle.v0.agent` and
`waddle.v0.parts` are negotiated, and [`sdk/README.md`](../sdk/README.md) for
the Python surface, the postures and the envelope-ownership doctrine.
[`sdk/examples/toy_robot.py`](../sdk/examples/) runs the offline, connected and
agent shapes of this lifecycle in one file, with no hardware and no plane
required.
