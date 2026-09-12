# Site lifecycle and manifest

## Lifecycle

The contexts are nested intentionally:

1. `load_site(path)` parses and validates `waddle.site/v1` without hardware access.
2. `Site.open()` constructs an unopened `SiteSession` context.
3. `SiteSession.__enter__()` resolves named secrets and builds declaration-only rigs.
   After connector authorization, it opens optional simulation worlds, drivers,
   cameras, native threads, and recording.
4. `SiteSession.run()` constructs an unopened `Run`; entering it starts one episode.
5. `Run.observe()` returns a composite robot/camera snapshot and paired time.
6. `Run.step()` asks the native gate, then applies the owner's envelope before any
   driver write.
7. `Run.finish()` records `success`, `failure`, or `abort`. Leaving an unfinished run
   records `abort` automatically.
8. Session exit stops services, finalizes recording, and closes every opened resource,
   including after a partial-open failure.

Only one run may be active in one `SiteSession`.

## Minimal manifest shape

```yaml
api_version: waddle.site/v1
kind: Site
metadata:
  id: inspection-cell
parts:
  arm:
    driver: customer_robot.backend:arm
    posture: supervised
    base_frame: arm_base
    connection:
      device: /dev/customer-arm
    joint_limits: {}
cameras: {}
frames: {}
calibration:
  artifacts: calib/
workspace_bounds: {}
envelope:
  static_keepouts: []
  self_collision: {}
recording:
  root: data/
  format: mcap
```

The bundled Draft 2020-12 schema rejects unknown fields. Relative paths are normalized
and confined beneath the manifest directory. Credential-shaped values must use a named
reference such as `{secret: ARM_TOKEN}` and are resolved only while the site opens.

## Topology and envelope

Each manifest part calls exactly one `PartConfig` factory. That factory returns a lazy
`Rig` with one bare action space and, when opened, exactly one `Arm`. The site layer
combines those parts into a composite declaration. All parts currently need the same
control rate and posture.

A shared simulator instead declares `worlds.*.driver` and lets each simulated part or
camera name that world. The SDK opens the world once and still presents its devices
through the ordinary runtime. See [simulation backends](../porting/simulation.md).

`base_frame` must match the frame reported by the opened arm. Camera mounts refer to
the scene or one declared part; they do not imply a transform. A configured workspace
requires forward kinematics. Static keep-outs or self/cross-part collision rules
require conservative geometry in compatible frames and fail closed when it is missing.

The complete JSON Schema ships at `waddle_sdk/schemas/site-v1.schema.json` in the
installed distribution.

For URDF-first simulation, `waddle-sdk sim init` creates the separate strict
`waddle.scene/v1` authoring document and `sim compile` emits this ordinary Site form.
Camera placement, lights, materials/coatings, deterministic variation, MuJoCo, the
ROS 2 adapter used by Gazebo/Isaac Sim, and third-party backend entry points are covered
in [Simulation backends](../porting/simulation.md).

## YAM initializer workspace preset

A mounted arm may need room on both sides of its base. The optional
`waddle_sdk.robots.safety_presets_for_driver` contract exposes the YAM tabletop
starting bounds as `min: [-0.7, -0.7, 0.0]` and `max: [0.7, 0.7, 1.0]`, in metres
in each arm's declared base frame. The preset has no static keepouts or
self-collision configuration. The site operator must review mounting, table,
tool, and neighboring-arm clearance before copying these values into a new site.
Existing `site.yaml` bounds remain unchanged, and importing the preset opens no
hardware. A preset is a configuration suggestion and grants no runtime capability.

Site opening now coordinates ownership across applications. See
[site ownership and publisher evidence](ownership-and-media.md) for lock scope,
uncertain teardown and the optional native media-track contract.

## Independent named parts

`SiteSession.observe_parts(parts=None)` returns fresh successful measurements in
`Observation.parts` and exact failures in `Observation.faults`, keyed by part.
Omitting names reads every declared part. `observe()` retains its all-or-error
contract; missing measurements are never replaced with cached or invented values.
The site reporting loop retains individual device failures while continuing other
parts; a blocked device call or shared simulator failure is not isolated.
Background part failures remain in `SiteSession.events()` as `robot.part_fault`
events containing `part` and the original structured `fault`, including its cause
chain. Later successful reads do not erase that history. Unchanged repeated fault
payloads are coalesced per part; applications choose their response policy.

`Run.step_parts({part: JointPositionCommand(...)}, observation)` returns one
`SubmitResult` per addressed part. Each command crosses the native gate and owner
envelope and is recorded with its declared part identity. Driver exceptions do not
erase neighboring receipts. This is not an atomic multi-part transaction. Dispatch
receipts establish neither arrival nor a confirmed physical stop. A failed write
can have an unknown physical outcome, even when no successful dispatch was reported.

The optional `NamedPartsObservationPort` and `NamedPartsRunPort` contracts are
advertised as `observation.named_parts` and `action.named_parts` support facts.
The existing site lease, explicit Hold/e-stop and native supervision remain shared.
Configured cross-part collision checks still require fresh neighbor geometry; a
missing required neighbor can refuse an otherwise healthy part's command. Applications
own trajectory scheduling, completion and cancellation; this API adds no automatic
sibling cancellation, motion replay or fault-domain policy.
