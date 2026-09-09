# Native MuJoCo bottle-cap geometry

The cap is a 25 g free rigid body throughout the simulation. MuJoCo's installed
first-party nut/bolt SDFs supply thread contact; `metric_thread.cc` forwards their
static distance evaluations with uniform scale 0.05. The nominal pitch is
4.166667 mm/turn. A rigid roof closes the cap. No callback applies contact forces,
attaches objects, or changes a constraint when the cap leaves the thread.

The native internal surfaces use SDF/SDF contact. Robot-facing collision surfaces
are offline CoACD partitions of the same marching-cubes geometry. Their masks
avoid duplicate internal contacts and mesh/SDF contact between these surfaces.
The roof also contacts the robot and scenery. Visuals use the original SDF mesh;
convex collision pieces remain hidden. This is an illustrative threaded fixture,
not a calibrated commercial bottle, seal, material or mass distribution.

`cap.xml` contains the native assembly, collision mappings and explicit inertia.
`manifest.json` records geometry provenance, conversion settings and every native
asset hash. Geometry was generated using MuJoCo 3.11.0, CoACD 1.0.14 and trimesh
5.1.0. The installed first-party plugin must retain its documented radius API.
Native tests also check thread behavior on supported runtime versions.

The wrapper is compiled once in each bottle-cap worker with C++17 against that
interpreter's installed MuJoCo headers and library. The private build directory
is removed after the library is mapped. Nothing downloads or changes an installed
library. Set `CXX` to select a compiler; other reference environments do not build
this plugin. Native worker processes currently require POSIX.

Rebuild these assets from the SDK root, with the packages above available:

```bash
PYTHONPATH=sdk/python python tools/vendor_thread_model.py --output /tmp/thread-assets
```

Inspect the generated hashes and meshes before running without `--output` to
replace the packaged assets. The renderer and native manipulation tests must
pass after a geometry change. No simulator is imported during site declaration.

Geometry attribution: Google DeepMind MuJoCo first-party SDF plugins, distributed
under Apache-2.0; a copy is in LICENSE. The dimensional wrapper uses the public
plugin API and contains no copied thread formula.

- https://github.com/google-deepmind/mujoco/blob/3.11.0/plugin/sdf/nut.cc
- https://github.com/google-deepmind/mujoco/blob/3.11.0/plugin/sdf/bolt.cc
- https://github.com/google-deepmind/mujoco/blob/3.11.0/model/plugin/sdf/nutbolt.xml
- https://mujoco.readthedocs.io/en/stable/programming/extension.html

SAPIEN/Isaac reference caps currently retain their finite guided model; this
MuJoCo asset does not establish free removal on those backends.
