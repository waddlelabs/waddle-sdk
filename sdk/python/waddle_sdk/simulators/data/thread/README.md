# Native bottle-cap geometry

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

SAPIEN uses `nut-physx.stl` and `bolt-physx.stl` for native GPU PhysX contact.
These higher-resolution surfaces are sampled from the same first-party SDFs
with 0.15 mm spacing using scikit-image's Lewiner marching cubes, simplified
with Manifold's 0.025 mm tolerance, and cleaned with MeshLab's T-vertex filters.
Their final float32 topology is watertight with no duplicate or zero-area faces;
the manifest records versions, parameters and hashes. The original coarse
visualization meshes are inadequate for this small thread clearance.

Rebuild the PhysX surfaces separately with the versions above and NumPy 2.5.3:

```bash
PYTHONPATH=sdk/python python tools/vendor_physx_thread_model.py --output /tmp/physx-thread-assets
```

The generator calls the installed native SDF in a temporary C++ sampler. It
contains no thread formula and reproduces both packaged STL hashes. Inspect
the output before omitting `--output` to update the packaged surfaces and manifest.

The PhysX assembly reads its poses, roof, mass, COM and inertia from `cap.xml`.
SAPIEN's native mesh loader/cooker consumes the packaged surfaces; its worker
does not need MuJoCo or a compiler. Isaac still retains its finite guided model.
The GPU fixture does not itself establish workspace or agent task acceptance.
