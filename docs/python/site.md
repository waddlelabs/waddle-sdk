# Site lifecycle and manifest

## Lifecycle

The contexts are nested intentionally:

1. `load_site(path)` parses and validates `waddle.site/v1` without hardware access.
2. `Site.open()` constructs an unopened `SiteSession` context.
3. `SiteSession.__enter__()` resolves named secrets, builds the rigs, then opens
   drivers, cameras, native threads, and recording.
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

`base_frame` must match the frame reported by the opened arm. Camera mounts refer to
the scene or one declared part; they do not imply a transform. A configured workspace
requires forward kinematics. Static keep-outs or self/cross-part collision rules
require conservative geometry in compatible frames and fail closed when it is missing.

The complete JSON Schema ships at `waddle_sdk/schemas/site-v1.schema.json` in the
installed distribution.
