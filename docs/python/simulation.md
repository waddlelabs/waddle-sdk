# Physics simulation sites

Physics uses the same `Site`, `SdkRuntimePort`, joint-position, gripper, RGB-D,
kinematics, body-geometry, recording, hold, and e-stop contracts as physical devices.
Only the site adapters and their configuration select an engine. Programs must use
capability facts; importing a simulator does not establish sensor or motion support.

## Reference engines and scenes

| Selector | Role | Installation |
| --- | --- | --- |
| `mujoco` | Small, fast reference engine; extends the existing SDK MuJoCo driver | `pip install 'waddle-sdk[mujoco]'` |
| `isaac` | NVIDIA USD/PhysX and RTX rendering | Separate Isaac Sim 6.0.1 Python 3.12 installation; pass its interpreter as `worker_python` |
| `sapien` | PhysX manipulation and Vulkan RGB-D; a foundation for ManiSkill assets | `pip install 'waddle-sdk[sapien]'` on a supported platform |

MuJoCo and SAPIEN can run headlessly with working EGL/Vulkan drivers. The worker
currently requires POSIX. Isaac has its own GPU, driver, Python, and license
requirements. Installation and license acceptance are operator actions; opening a
site never downloads packages, accepts a license, or substitutes a mock engine.

Each engine accepts `yam` or `xarm7` and these environments:

- `two_cubes`: two free rigid cubes on a table, with frictional grasp contacts.
- `bottle_cap`: a fixed bottle fixture and a cap with coupled passive rotation and
  axial travel (5 mm/revolution). This reference thread model has three turns of
  travel; the cap remains on its axial guide, rather than becoming a free body.
- `drawer`: a fixed cabinet and a physical drawer with 220 mm of passive travel.

The reference robot models preserve the live adapters' joint names, order, limits,
radian units, FK, and normalized hand action (0 closed, 1 open). YAM uses the SDK's
pinned I2RT chain and tool convention. xArm7 uses the vendor's pinned kinematic
origins with the standard gripper TCP. Sources and licensing are recorded alongside
`waddle_sdk/simulators/data/`.

These are deliberately simple reference geometries, with approximate masses and
inertias, position servos, and gravity compensation. They are **not calibrated
hardware dynamics models**. A different physical hand, TCP offset, link revision,
or camera mounting requires a matching embodiment configuration. No simulation
result certifies clearance, grasp forces, or success on hardware.

## Create and open a site

Use a new directory and write the two returned documents:

```python
import json
from pathlib import Path
import yaml
from waddle_sdk import load_site
from waddle_sdk.simulators import make_site

root = Path("cube-site")
root.mkdir()
site, simulation = make_site(
    "cube-site", backend="mujoco", robot="yam", environment="two_cubes",
    width=640, height=480,
)
(root / "site.yaml").write_text(yaml.safe_dump(site, sort_keys=False))
(root / "simulation.json").write_text(json.dumps(simulation, indent=2))
with load_site(root / "site.yaml").open(console=False) as session:
    print(session.observe())
```

`make_site()` is non-opening. Its site has one part, `arm`, and two cameras, `scene`
and `wrist`. Every backend uses those same names and profiles. Names, resolutions,
intrinsics, and optical transforms are explicit configuration, not engine defaults.
Edit camera declarations and their matching simulation profiles together. The
reference scene has a single robot; multi-arm/custom arrangements can use ordinary
external SDK adapter packages without an SDK registry change.

## Sensor and frame contract

RGB is RGB8, depth is Z16 axial optical depth, and each pair comes from one frozen
physics state. `depth_scale_mm=1` means millimetres per integer; zero denotes missing
or out-of-range depth. Intrinsics are rectified pinhole intrinsics. The default
resolution is 640×480 at a requested 15 Hz, with explicit focal lengths and principal
point. Actual frame rate depends on rendering performance.

Optical frames use +X right, +Y down, +Z forward. Robot poses use metres and wxyz
quaternions, in `yam_base` or `xarm_base`. A scene-camera transform is
`base_from_camera`; a wrist transform is `tcp_from_camera` and follows the robot.
Renderer-specific camera axes, depth conventions, joint ordering, and two-finger
coordinates are converted inside the engine adapter. No privileged object pose,
segmentation, teleport, or task-completion operation is exposed through the runtime.

## Ownership and extension

Reference scenes implement the existing [shared-world contract](../porting/simulation.md).
The ordinary `worlds.cell` declaration owns one lazy worker and exposes part/camera
facets. `Site` opens it after authorization, resets the whole scene once per episode,
and closes it after devices. Each opening gets independent physics state. Worker
isolation keeps incompatible engine/Python runtimes out of the control process.

These presets add coupled grippers and articulated objects beyond the generic URDF
compiler's supported scene subset. For custom URDF bundles, use the existing portable
scene compiler; for an already configured Isaac stage, the existing ROS 2 backend is
also available. No second SDK lifecycle or agent API is introduced.

The worker continuously advances physics, serializes sensor/control requests, and
fails closed on startup or connection loss. Recovery requires reopening the site;
it never replays a motion after reconnecting. Site camera declarations are compared
to simulation sensor profiles before exposing the camera. Joint owner limits may tighten the
reference limits, but must include the reference home pose.

External simulations can implement the existing [robot](../porting/robot.md) and
[camera](../porting/camera.md) factories. The bundled worker protocol and native
`Engine` classes are private implementation details, not another agent API.

## Research basis

The three engines cover complementary execution needs:
[MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html) for a compact reference,
[Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
for NVIDIA rendering and USD tooling, and
[SAPIEN/ManiSkill](https://maniskill.readthedocs.io/en/latest/) for manipulation assets.
[SimFoundry](https://research.nvidia.com/labs/gear/simfoundry/) is a scene-creation
pipeline to evaluate as an asset source, rather than a fourth interchangeable
physics runtime.
