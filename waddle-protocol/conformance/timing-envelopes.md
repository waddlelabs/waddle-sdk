# Timing envelopes — conformance tier 3 (N3, N16)

Fixtures verify **logic, not physics** (N3). This document defines the bench
dimensions measured per frontend and the rules under which the resulting
numbers may be published (N16). Vocabulary per `../docs/GLOSSARY.md`.

## Bench dimensions

Every bench runs as a sustained soak (hours, not seconds) and reports jitter
percentiles — p50 / p95 / p99 / max, mirroring `JitterStats`
(services.proto) — never a single best-case figure.

| Bench | Definition |
|---|---|
| **gate passthrough latency** | time from `gate(action, obs)` entry to return in `GATE_MODE_PASSTHROUGH` with no claim active, measured at the declared `ActionSpace.rate_hz` under steady load. This is the fast path; it MUST NOT degrade with session length. |
| **engage-to-first-intervention-action** | time from receipt of the engaging `ClaimDirective` (or local clutch engagement on `waddle/v0/teleop/clutch`) to the first intervention action returned by the gate. Measured separately per `HandoffPolicy` arm: `Immediate`, `ChunkBoundary` (report against `max_wait_ns`), `HoldFirst` (report engage-to-hold and hold-to-first-action separately). |
| **blend-window adherence** | error between the declared `HandoffPolicy.Immediate.blend_ns` window and the observed cross-fade duration, plus compliance of every blended step with the space's declared step ceilings (`max_linear_step_m`, `max_angular_step_rad`). |
| **deadman/staleness cutoff** | time from the last valid teleop packet (or a `ClutchTransition` release) to the gate ceasing to emit intervention actions and requesting hold. The integration's staleness watchdog is the fail-safe under measurement; the console's synthesized release is UX and MUST NOT be what passes this bench. |
| **hold round-trip** | time from `VerbRequest{VERB_HOLD}` issuance to the integration's acknowledging `VerbResult`. **Safety-adjacent**: publication rule below applies in full. |

Every published number MUST state its conditions: hardware (CPU, memory,
OS/kernel), frontend and version, `waddle-protocol` release, declared
`rate_hz` and action space, load profile, network conditions where a WAN or
media plane is in the loop, soak duration, and sample count.

## Publication rule (N16) — normative

- All published numbers are **observed bench measurements under stated
  conditions**. They are not specifications, not guarantees, not warranties,
  and no published envelope creates one.
- Every published envelope table MUST carry the following language verbatim:

  > These figures are observed bench measurements under the stated
  > conditions. They are not a specification or warranty of performance.
  > Waddle's detection and intervention are best-effort supervision, never a
  > safety system; the envelope belongs to whoever owns the hardware. The
  > only binding figures for a deployment are those measured on that
  > deployment by `waddle doctor`.

- **Safety-adjacent numbers — hold round-trip and e-stop latency — are
  binding only as measured per deployment by `waddle doctor` on the actual
  rig, never from a brochure.** `Grant.declared_latency_bound_ns` is the
  integrator's declaration; doctor measures it in a safe window
  (`MEASUREMENT_WINDOW_DOCTOR`), and heartbeat proxy signals maintain
  confidence between safe-window re-measurements (N11). A bench number for
  these dimensions exists to size the fleet, not to bound any deployment.

## Envelope table

Template. One table per frontend per `waddle-protocol` release. No row may
carry a number that was not produced by the bench it names.

| Bench | Conditions | Measured envelope (p50/p95/p99/max) | Date | Hardware |
|---|---|---|---|---|
| gate passthrough latency | TBD (bench not yet run) | TBD (bench not yet run) | — | — |
| engage-to-first-intervention-action — `Immediate` | TBD (bench not yet run) | TBD (bench not yet run) | — | — |
| engage-to-first-intervention-action — `ChunkBoundary` | TBD (bench not yet run) | TBD (bench not yet run) | — | — |
| engage-to-first-intervention-action — `HoldFirst` | TBD (bench not yet run) | TBD (bench not yet run) | — | — |
| blend-window adherence | TBD (bench not yet run) | TBD (bench not yet run) | — | — |
| deadman/staleness cutoff | TBD (bench not yet run) | TBD (bench not yet run) | — | — |
| hold round-trip | TBD (bench not yet run) | TBD (bench not yet run) | — | — |

> These figures are observed bench measurements under the stated conditions.
> They are not a specification or warranty of performance. Waddle's detection
> and intervention are best-effort supervision, never a safety system; the
> envelope belongs to whoever owns the hardware. The only binding figures for
> a deployment are those measured on that deployment by `waddle doctor`.
