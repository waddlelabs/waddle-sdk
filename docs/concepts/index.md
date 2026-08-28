# Concepts

## The boundary

Waddle SDK sits beside the customer's devices. It opens robot and camera drivers,
enforces the site owner's envelope before a command reaches a driver, stamps samples,
records the full-rate local archive, and runs the authority machinery in
`waddle-core`.

Higher layers see one structural `SdkRuntimePort`. They may describe the site,
observe it, open a run, submit an action, request hold or e-stop, and read ordered
events. They do not receive a driver, a hardware handle, an authority token, or a raw
depth stream.

| Layer | Owns |
|---|---|
| Site owner and adapter | Device integration, physical stop, measured limits, safe recovery, and conservative geometry |
| Waddle SDK | Hardware/camera lifecycle, owner-envelope enforcement, authority runtime, clocks, and raw local recording |
| Consumer above the SDK | Tasks, planning, perception, and product workflow, constrained by the SDK declaration and grants |

The Python package is a hollow frontend over the Rust core. It translates Python data
and composes owner-side hardware objects, but it does not reimplement claim, lease,
handoff, or timeline decisions.

## Four words that must stay distinct

- A **grant** is permission for one registered control verb.
- A **claim** assigns an episode to a claimant.
- A **lease** is the whole-robot, single-writer right to actuate.
- An **envelope** is the owner's hard-safety gate chain.

The [normative glossary](../core/glossary.md)
defines these and the rest of the public vocabulary.

## Declarative sites

A `site.yaml` declares topology and reviewed configuration. A part entry names an
importable factory; the SDK does not maintain a vendor registry. Configuration may be
loaded and factories may be constructed without opening a device. Hardware opens only
when entering `Site.open()`'s context.

Physical relationships stay explicit:

- `parts.*.base_frame` is the frame in which an opened arm reports poses.
- `cameras.*.mount` says whether a camera is fixed to the scene or moves with one
  named part.
- `workspace_bounds` and `envelope` are owner-reviewed safety declarations, not
  suggestions inferred from device discovery.

## Control and media are separate

The control plane carries declarations, bounded observations, actions, events, and
negotiated feature flags. Continuous video belongs on the media plane. Raw metric
depth stays process-local; a camera may publish a derived color preview separately.

## Time has two coordinates

Every stream sample uses session-monotonic nanoseconds for ordering. An atomic clock
anchor relates that timeline to Unix time. Implementations stamp the two together;
they never reconstruct one later from the other. In the Rust implementation,
`waddle-ingest` is the only production crate allowed to read operating-system clocks.

## Support is not permission

An opened Python session derives a `waddle.sdk.support/v1` matrix. Rows report facts
such as forward kinematics, body geometry, camera intrinsics, and declared limits.
They do not grant motion or claim that a robot skill is available. A consumer must
intersect support facts with the exact action declaration, live grants, artifacts,
and its own implementation health.
