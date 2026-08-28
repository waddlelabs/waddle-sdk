# SDK concepts and ownership

## Layer boundary

`waddle-sdk` is the customer-side supervision runtime. It owns hardware adapters, cameras, hard-safety enforcement from site-owner facts, session timing, raw observations, local recording, and the protocol-facing runtime. A higher layer may plan work or expose product tools, but it must consume the SDK contract rather than gaining another hardware path.

The owner envelope is the hard, non-bypassable safety chain. Waddle is always subject to it and never supplies it. A tripwire is weaker: it observes conditions and requests a declared hold.

## Vocabulary

- **grant:** a permission the integrator extends for a control verb.
- **claim:** orchestration-level assignment of an episode or work item.
- **lease:** the actuation-level single-writer right.
- **gate:** the one point where supervision touches the integrator loop.
- **episode:** one rollout attempt, from verified reset to terminal outcome.
- **capability:** a robot skill; never permission or version negotiation.
- **feature flag:** a connection-scoped protocol-evolution agreement.
- **site operator:** the customer-side human at the machine.
- **teleoperator:** the human driving through the work plane.

In a checkout, `waddle-protocol/docs/GLOSSARY.md` is the complete normative vocabulary.

## Lifecycle

A `Site` is validated configuration and does not open hardware. `Site.open()` returns a context; hardware opens in `SiteSession.__enter__` and closes deterministically on exit. A `Run` is an episode within that open session. The Rust FSM owns claims, leases, handoffs, intervention phases, and terminal transitions.

Construction and authorization probes must remain non-opening. Hardware adapters open only in the opening phase. Half-open failures close every resource that did open. Closing cameras must unblock capture; closing live robot resources must leave the machine in the documented owner-safe state.

## Safety and posture

The ordinary `Arm` seam rejects a complete target and never clamps it into a command nobody requested. Owner inputs include joint limits, per-command step caps, optional workspace bounds, and optional static collision rules. Workspace checks require usable FK; configured collision checks require conservative geometry in a compatible frame and fail closed when unavailable.

`monitor` and `supervised` describe which verbs a session registers. They do not decide authority. Only `"sim"` identifies a harmless twin; every other driver `kind` is treated conservatively as live hardware.

## Time, media, and recording

Stream time is session-monotonic nanoseconds. A `ClockAnchor` pairs it with wall time, and the wall-clock twin is captured at stamp time. Camera RGB and aligned raw metric depth form one immutable, paired sample. Raw depth stays local; a derived preview may use the media plane. The local recorder is authoritative for full-rate evidence.

The control plane carries low-rate control and bounded declared observations, not continuous media. A negotiated feature flag belongs to one connection and must not leak queued messages onto a later connection with different negotiation.
