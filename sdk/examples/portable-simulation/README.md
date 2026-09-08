# Portable RGB-D simulation

This is a complete backend-neutral source scene: one URDF arm, reviewed joint
limits, visual appearance, a visual-only tool coating, conservative body spheres,
scene and wrist RGB-D cameras, two lights, and environment geometry. Seeded color
and intensity ranges are resolved into recorded build evidence.

From `sdk/`:

```bash
uv run waddle-sdk sim validate examples/portable-simulation/scene.yaml
uv run waddle-sdk sim compile examples/portable-simulation/scene.yaml \
  --backend mujoco --output /tmp/waddle-portable-demo
MUJOCO_GL=egl uv run waddle-sdk sim run examples/portable-simulation/scene.yaml \
  --backend mujoco --output /tmp/waddle-portable-run
```

The build directory contains `site.yaml`, `world.xml`, the normalized URDF and
assets, simulator-ground-truth scene/wrist transforms under `calib/`, and
`resolved-scene.json`. Metal opens the generated `site.yaml` through the ordinary
SDK port and autoloads those calibration artifacts; it has no simulation-specific
attachment or operator calibration step.

To begin with another arm, use:

```bash
uv run waddle-sdk sim init path/to/arm.urdf --tool-link tool0 \
  --output /tmp/my-portable-scene
```

Review the generated owner envelope and tool transform before supervised motion.
The initializer intentionally does not guess workspace bounds or collision spheres.
