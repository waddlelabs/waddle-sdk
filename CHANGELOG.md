# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Fixed

- Keep small chocolate cylinders held during lift/carry on MuJoCo 3.13.0 by
  selecting iterative multicontact with a 1 micrometre contact margin in scene
  revision 1.0.2. Preserve geometry, material friction, mass, and robot force.

### Changed

- Version shampoo packing as 1.0.3: lower source walls to 60 mm and regenerate
  its stable mixed-orientation gravity pile; raise the receiver pedestal to
  60 mm and taper its six pockets continuously over 80 mm. Preserve bottle
  dimensions, material contacts and actuator limits. Update the native transport
  witness and require floor support and settling in offset-drop checks.

- Version shampoo packing as 1.0.2: raise the receiving box by 40 mm for upright YAM access, and use explicit bottle/finger material pairs with a 3 mm torsional patch. Preserve pad sliding friction, force limits, stiffness, gravity pile and other contact pairs.

- Version shampoo packing as scene 1.0.1 with physical tapered pocket entrances: 62 mm mouth to a snug 46 mm bottom, keeping upright seating dimensions and task-neutral runtime.

- Tighten the chocolate grid to 12 mm clear gaps (32 mm centers) in scene
  1.0.3, keeping twenty 20 mm cylinders and the validated contact margin.

- Move the chocolate-packing camera above the box in scene revision 1.0.1,
  keeping both trays visible while reducing pocket-mouth depth parallax.

- Make the near-base, wider YAM `pick_lift` cube distribution the built-in
  noncanonical evaluation behavior under scene revision 1.1.0; an external pose
  profile is no longer needed.
- Revise the `candy-bin-transfer` scene to version 1.2.0 with an adjacent wide
  blue destination bin. Its physical floor can retain all eighteen 24 mm cubes
  in one evolving scene, enabling continuous transfer-throughput evaluation
  without per-cube resets.

### Added

- Add the shampoo-packing scene with twenty gravity-settled thick cylinders, a reproducible pile asset, and six upright receiving pockets.

- Add native small-cylinder grasp acceptance through descent, lift, sustained
  bilateral carry, and release, including partial closure and a missed-grasp
  negative control across supported MuJoCo runtimes.

- Add the S20 `chocolate-packing` MuJoCo scene with twenty spaced small cylinders,
  a physical six-pocket box, a closer scene camera, and native grasp/drop tests.
  Trusted continuation reset preserves remaining source pieces and retires packed
  pieces so the next box is empty without restoring the supply.
- Retain validated explicit YAM `pick_lift` pose profiles for replaying frozen
  evaluations that selected one.
- Add the single-arm `candy-bin-transfer` MuJoCo task environment: eighteen
  independently simulated candy cubes start in a shallow clutter tray beside
  a physical destination bin. Native tests cover camera visibility,
  stable clutter, released bin containment, reset, and all three reference robot
  families without adding task policy or evaluator state to the public SDK.
- Extend native initial-collision acceptance to every matched dual-arm task cell.
  D01-D11 now reject penetration between either robot and the tabletop or any
  task prop for SO-101, YAM, and xArm7, alongside the existing inter-arm and
  single-arm checks.
- Add a native-mechanics coverage guard for all 30 MuJoCo evaluation task
  environments. Every catalog scene must remain assigned to the isolated
  single-arm conformance path or an explicit dual-arm physics acceptance group.
- Add evaluator-only per-physics-step contact summaries to reference MuJoCo task
  snapshots. Named geometry pairs retain first/latest time, minimum distance,
  maximum normal force, and sample count until reset, allowing a lower-rate trusted
  scorer to observe transient contact and peak-force events without changing the
  participant interface.
- Add an engine-neutral `SimulationVariation` profile to trusted evaluator reset,
  with independent canonical, bounded, and held-out pose, appearance, physics,
  and geometry dimensions. Reference MuJoCo task scenes deterministically vary
  material/lighting, mass/friction/damping, and task-prop scale, restore canonical
  model arrays on ordinary reset, and record a stable resolved-variation digest
  only in privileged snapshots.
- Scale the reference task layout to each robot family's reachable workspace.
  Drawer fixtures keep the existing full-size-arm placement while moving inward
  for SO-101; control, door, socket, rack, USB, and dual-arm workpieces now have
  collision-free reachable starts. A higher drawer/rack scene-camera mount exposes
  the open drawer interior and rear rack row. Native acceptance checks the revised
  visibility, resting mechanics, and all affected task placements.
- Add SDK-owned monotonic start/completion bounds to `SubmitResult`. The start is
  captured after acquiring the shared dispatch boundary, allowing a timed caller
  to distinguish dispatch work from prior lock contention while retaining the
  existing per-part gate, fault, and non-arrival semantics.
- Add explicit rolling/torsional contact friction and small passive free-body
  damping to loose horizontal cylinders in the insertion and clear-test-tube
  fixtures. Their canonical tabletop poses now remain at rest for a six-second
  native soak instead of gaining rolling energy and leaving the evaluation
  workspace before a participant acts.
- Apply stable, bounded XY and yaw variation when the trusted evaluator resets any
  of the 29 reference task scenes. The same seed reproduces the same world across
  worker restarts, seed zero selects the canonical pose, assembled free fixtures
  move as rigid groups, and ordinary reset restores the canonical interactive scene.
- Add all six hard two-arm MuJoCo development environments: oriented tool
  handover, movable-receiver peg insertion, balanced two-handle lift, loaded tray
  transport, friction-fit test-tube uncapping/return, and target-bottle retrieval
  from clutter. Native tests cover visibility and task mechanics across every
  matched reference family; coordinated multi-part execution remains a separate
  downstream capability requirement for D07-D10.
- Add the hard single-arm USB-insertion and clear-test-tube loading development
  environments. The close-clearance connector has a native missing-quadrant key
  and internal port tab; four translucent physical tubes fit distinct named
  two-row rack collars. Tests cover correct/reversed insertion, contact load,
  ordered slot placement, visibility, stability, and reset for every family.
- Add four hard single-arm MuJoCo development environments for a released
  three-cube stack, close-clearance peg insertion, retrieving an object from a
  physical drawer, and storing an object before closing that drawer. Native tests
  cover visibility, aperture contact, composed object/articulation motion,
  containment, stability, and reset for all reference families.
- Add four medium two-arm MuJoCo development environments for block handover,
  stabilizing a movable drawer cabinet, holding a movable container during
  placement, and stabilizing a friction-fit box while removing its lid. Native
  tests cover both-arm startup, overhead visibility, free fixtures, containment,
  load transfer, separation, goal placement, and reset for all reference families.
- Add the single-arm `open-hinged-door` MuJoCo development environment. A passive
  lever retracts its sliding bolt through a native joint constraint; the extended
  bolt contacts a fixed strike and blocks the same opening load that moves the
  released door. No callback changes the latch or door state.
- Add five medium single-arm MuJoCo development environments for distractor
  selection, drawer closing, two-cube stacking, ring placement, and hook use.
  Their ordinary reset path restores declared passive-joint initial positions;
  native checks cover task-object visibility and mechanics for every reference
  family without embedding robot routes or success logic.
- Add the two-arm `split_workspace_sorting` MuJoCo development environment for
  every reference family. It presents two independently reachable cubes and two
  opposite-side bins through the ordinary dual-arm runtime, scene camera, wrist
  cameras, reset, and evaluator snapshot contracts.
- Add five MuJoCo development task environments for target contact, cube lift,
  bin placement, region pushing, and control operation. They use ordinary SDK
  robot/camera/reset paths, expose task-relevant objects to every reference
  embodiment's scene camera, and include native mechanics and visibility checks.
- Add a separately retained `SimulationAdministration` capability for trusted
  evaluators. It binds only to complete supported simulation sites, snapshots
  finite privileged state, performs deterministic seeded reset without replacing
  the active SDK run, advances an episode revision, and fails closed on partial
  support or uncertain reset. The reference MuJoCo worker reports named joints,
  bodies, velocities, contacts, and simulation time through this optional facet.
- Bind reference MuJoCo evaluator snapshots to provider/revision, robot family/
  embodiment revision, arm count, environment, scene revision, and asset revision.
  Newly generated simulation documents carry the reference revisions; older
  documents remain interactive but lack the exact task-evaluation identity.
- Add SO-101 to the pinned reference simulation family, including its maintained
  CAD/inertias, five arm joints, normalized physical gripper mapping, RGB-only
  scene/wrist cameras, camera mount, source hashes and license.
- Add matched two-arm MuJoCo sites for SO-101, YAM and xArm7 in one shared world,
  with independent named parts, base placements, per-arm control/reset/TCP state,
  one scene camera and one wrist camera per arm. Isaac and SAPIEN remain explicit
  one-arm reference backends.
- Generate non-opening planner model sources for every reference robot from the
  same pinned chain, limits, base/TCP frames and collision links as the runtime
  assembly. Record the fixed-open-hand, no-prop and no-neighbor geometry scope.
- Accept an exact per-camera physical profile when generating a simulation site,
  keeping each camera's stream, rectified intrinsics, depth capability and mount
  transform identical across the public site and native simulator configuration.

- Include Isaac in the existing rendering-preset, episode-reset, shared-world
  lifecycle/e-stop and custom camera/part-name checks. Use its explicitly selected
  licensed worker interpreter; assertions and other backend behavior are unchanged.
  Include native Isaac cube mass/inertia in the shared physical-property checks.
  Add SAPIEN to the shared-world reopen/e-stop test as well.

- A freely removable MuJoCo reference cap with native first-party nut/bolt SDF
  contacts, dimensional scaling, a rigid roof and hash-bound convex collision
  surfaces. The cap remains a 25 g free body throughout unscrewing and removal.
  A C++17 compiler is required only when opening a MuJoCo bottle-cap scene;
  compilation uses a private temporary directory and the installed native API.
  Include an offline asset rebuild tool and native axial-retention/free-exit tests.
  Isaac's equivalent USD assembly has separate licensed runtime acceptance.

- Reference scene `render_quality` presets (`fast`, `standard`, `high`) using
  native renderer settings without changing robot, physics, or camera contracts.
  Existing scene files retain standard rendering.

- Replace primitive simulation robots with pinned manufacturer YAM/xArm7 URDF
  assemblies, actual visual meshes, masses, COMs and full inertia tensors. Match
  the live YAM linear hand and xArm G2 gripper with native coupled joints,
  nonlinear jaw mapping, shared convex hand collisions and mesh-derived
  planning bounds. Reuse the optional trajectory-velocity driver port. Include
  licenses, source hashes and an offline vendoring tool.

- Reference YAM and xArm7 workspaces for MuJoCo, Isaac Sim, and SAPIEN, with shared
  scene/wrist RGB-D profiles, normalized grippers, two cubes, a threaded cap, and a drawer.
  Optional engines run in isolated workers behind the existing shared-world lifecycle.
- Optional per-row `Arm.position_error_caps` and YAM
  `max_joint_position_error_rad` let a reviewed owner envelope bound servo
  tracking error independently of command cadence. Defaults, gripper bounds,
  simulated speed and vendor dispatch remain unchanged. Open descriptions and
  `limits.position_error` support report the actual ordered allowance; refusal
  context retains it alongside the target and latest measurement. This adds no
  interpolation, convergence, or physical velocity/torque guarantee.

- Background part-read failures persist as structured `robot.part_fault` runtime
  events while healthy streams continue. Live comparisons retain selected-part
  fault history and reject recovered transient failures before further commands
  or successful completion.

- SDK-owned live two-arm acceptance measures device feedback, independent arrival
  and reuse, neighbor reference retention, timing under load, envelope refusal and
  explicit Hold/e-stop response. Reviewed profiles select hardware and test-only
  feedback probes; evidence retains original faults and per-arm startup details.

- Optional named-part observations and supervised joint submissions retain healthy results beside exact device faults. Sparse commands preserve native scope/recording, owner envelopes and configured neighbor collision dependencies; existing composite submissions retain their contract.


- Optional YAM `arm_gains` configures independent six-joint KP/KD values while
  preserving hand gains and recovery behavior. Paired raw/SDK benchmarks apply
  the same settings and distinguish requested gains from predicted MIT encoding.
  Explicit and scaled gains are validated before CAN startup; unrepresentable
  requests now fail instead of being silently clamped by the vendor codec.

- Real-camera RGB/depth publication and reconnect tests through the native SDK
  and configured LiveKit service, using isolated camera ownership without opening
  physical arms. Reports distinguish source dimensions from adaptive video sizes.

- Opt-in paired YAM benchmarks run pristine I2RT and SDK processes with matching
  controller settings and sampled trajectories. Reports preserve measured joint
  and TCP arrival, timing, vendor diagnostics, failures and cleanup; explicit
  comparison margins enforce accuracy and settling-time noninferiority.
  Optional reviewed rest targets return healthy trials to supported parking before
  backend hand-off. Parking remains separate measured evidence; parking failures
  block the next backend, and original motion/background faults prohibit further
  trajectory commands.
  Both backends now require observable post-start CAN feedback before timing a
  trajectory, preserving startup evidence separately and refusing stale startup
  within a bounded deadline without moving or changing envelope checks.
  Comparison rejects nonzero child exits, missing or unequal manifest hashes,
  and missing, duplicated or reordered phases even when recorded arrivals pass.
  Optional per-case `minimum_settle_s` retains a target-commanded observation
  window before arrival/Hold without extending the total settling deadline or
  bypassing faults, tracking checks or three consecutive arrival observations.
  Optional reviewed `approach_rad` adds reference-entry, approach and reference
  preparation through the same motion loop. Each must arrive before advancing;
  failures retain their evidence, allow only healthy configured parking and fence
  the next backend's target trial. Comparison checks exact preparation order and
  targets without relaxing initial-state, arrival or performance limits.

- Unify software and live behavior tests under `sdk/tests`; `pytest --live` selects
  viable camera/robot checks from metadata and optional motion profiles, reports
  missing prerequisites and stays disabled in CI/releases.
- Explicit `SiteSession.close(torque_release_authorized=True)` for authorized
  headless teardown, sharing context-exit resource and ownership handling.

- Shared site-ID process ownership in `Site.open()`, acquired before adapter
  construction and retained after uncertain teardown. Applications using the
  same site ID now coordinate through the same SDK lock.
- Optional `MediaRuntimePort` and `SiteSession.media_tracks()` expose native
  publisher identities, per-stream last-attempt status and drop counts without
  reconstructing track facts from observations.

- Shared optional `ModelSourceProvider` and immutable `ModelSources` bundle for
  complete source geometry, assets, named bindings and attribution. Any robot
  adapter can expose the same non-opening module extension without registry edits;
  absent support and selected failures remain distinct.

- Public non-opening robot/camera metadata helpers: named part action spaces,
  validated physical gripper mappings with reversed action ranges, effective
  camera declarations and intrinsics with explicit optional depth scaling.
- The YAM `model_sources` extension provides verified source geometry, attribution,
  TCP attachment and coupled finger travel from the exact optional I2RT pin,
  without opening hardware or selecting a planner.

- Export public `waddle_sdk.FEATURES` as the selected native core's feature set,
  allowing applications to check media support without importing SDK internals.
- Add opt-in public SDK camera/LiveKit acceptance using scoped grants, actual RGB
  and depth-display frames, native signaling reconnect with the same SiteSession
  and Run, and independent viewer rejoin. Document matching media dependencies
  and the explicit test lifecycle without changing hardware/media lifecycle behavior.

- Add manifest-selected shared simulation worlds through the public
  `WorldConfig`/`SimulationBackend` contract. World-backed parts and cameras reuse the
  ordinary SDK runtime, support matrix, owner envelope, observations, and RGB-D sample
  path consumed by applications.
- Add a shared-world MuJoCo reference backend with one physics state for all attached
  parts and cameras, exactly-once world stepping, episode reset, derived camera
  intrinsics, and aligned metric Z16 rendering.
- Add the strict `waddle.scene/v1` portable URDF scene contract, non-overwriting
  `waddle-sdk sim backends|init|validate|compile|run` workflow, seeded value resolution,
  replayable file-hash evidence, and a checked scene/wrist RGB-D example. The MuJoCo
  compiler covers robot placement, reviewed joint limits, scene geometry, camera
  optical poses, lights, materials, visual/physical coatings, and link-local safety
  spheres while emitting an ordinary `site.yaml`.
- Extend portable scenes with fixed geometry groups and free rigid bodies whose
  explicit mass, center of mass, full inertia, collision shapes, named state, and
  deterministic reset compile into the ordinary shared-world MuJoCo backend.
- Add parent-order-independent body composition and passive bounded slide/hinge
  joints with explicit initial state, axis/anchor, damping, friction, and optional
  springs. Portable hinge values are explicitly compiled as radians.
- Allow portable scenes to declare complete versioned runtime identity. The compiler
  adds and verifies the actual arm count, the backend adds its provider revision, and
  identity remains outside the ordinary participant runtime description.
- Add installed simulator extension points through the
  `waddle_sdk.simulation_backends` and `waddle_sdk.simulation_compilers` entry-point
  groups while retaining explicit `module:callable` targets.
- Add a lazy ROS 2 shared-world backend for simulator graphs. It maps reviewed joint
  declarations to ros2_control or `JointState` position topics, pairs RGB/depth by
  acquisition time, derives active intrinsics from `CameraInfo`, and supports an
  optional `std_srvs/Empty` reset service. This provides the ordinary SDK boundary for
  ROS-connected Gazebo and Isaac Sim processes.

### Changed

- Batch reference-world 2 ms physics substeps between state reports at twice the
  declared command rate, with a 100 Hz floor. Carry fractional substeps forward
  instead of advancing extra physics time when reporting intervals do not divide
  the native timestep.
- Share SAPIEN's original gain, force and inertia budget across opposing jaw
  drives while retaining native jaw coupling and xArm passive linkage closures.
- Add a bundled CC0 Poly Haven table texture and native material/key/fill lighting
  to MuJoCo and SAPIEN reference scenes, without changing geometry or RGB-D units.

- Base the reference simulation implementations on maintained 2026 I2RT,
  Menagerie, mjlab, ManiSkill and Isaac Lab sources. Use native URDF importers,
  xArm linkage closures and threaded-cap constraints; remove custom servo-target
  clipping, custom solver overrides and cap force callbacks. Reuse the SDK
  shared-world physics clock instead of running an independent worker clock. Derive the LINEAR_4310 pinch
  offset from the manufacturer's corrected grasp site without changing SDK FK.
- Owner-envelope refusals preserve their originating safety fault, exact reason,
  part, measured/commanded values and bounds in the submission receipt and event.
  Independent hold failures remain separate diagnostics.

- Native binding API 4 adds publisher snapshots; rebuild base and media extensions
  together. Camera/arm/world teardown failures now prevent site ownership release.

- YAM source loading implements the shared adapter contract and returns complete
  articulated MJCF, preserving physical slide coordinates and coupling. Remove
  the unpublished YAM-specific bundle fields from the public source boundary.
- Document and regression-test xArm G2's physical millimetre API boundary. Vendor
  code owns nonlinear motor-pulse conversion; parallel opening mappings stay linear.

- Camera depth resolution shares metadata validation and refuses malformed
  distortion coefficients before calling a vendor resolver.
- Correct the documented root-export regression to include the existing public
  `FEATURES` export.

- Make `AGENTS.md` the canonical guide and `CLAUDE.md` its relative symlink; keep
  build and public-contract guidance together, with publication-confirmed changelog
  archiving. Clarify public runtime documentation, examples, diagnostic messages,
  and historical notes without changing SDK APIs or execution behavior.
- Keep public rationale and amendment history while removing unrelated
  implementation and naming roadmaps; clarify its informative status.
- Document the optional MuJoCo prerequisite for the complete scene-compiler test
  suite and headless Linux rendering configuration.

- Set YAM absolute arm gravity factors to `[1.0, 1.1, 1.2, 1.3, 1.0, 1.0]`, using
  rounded two-arm bench calibration for joints 3/4. A validated, declaration-frozen
  `gravity_comp_factor` option overrides all six values per arm. Pass it into I2RT
  before its servo starts; retain the vendor gripper factor, PD gains, and friction.

- Expand the YAM tabletop initializer preset and example to workspace bounds
  `[-0.7, -0.7, 0.0]` through `[0.7, 0.7, 1.0]` metres in each arm base frame.
  Existing site declarations remain authoritative and are not rewritten.

- Preserve the existing `waddle_sdk.robots.mujoco:arm` factory as a compatible
  private-world adapter while documenting `waddle_sdk.robots.mujoco:backend` for
  multipart and RGB-D scenes.
- Require MuJoCo 3.5 or newer for the portable `MjSpec` scene compiler and renderer;
  the existing runtime remains lazy behind the `[mujoco]` extra.

### Fixed

- Gate the native camera-profile mismatch test on its optional MuJoCo runtime,
  keeping the default SDK test suite valid without simulation extras.
- Refuse evaluator resets whose compiled MuJoCo contacts contain robot/table,
  robot/prop, or dual-arm cross-arm penetration. The trusted administration layer
  fences a refused world and requires the site to reopen before further use.
- Build YAM planning geometry from the same deterministic spatial convex hulls
  of complete decomposed source pieces used by the other reference families.
  This preserves forearm recess clearance that a single convex import of each
  concave source mesh filled, while retaining known native collision rejection.
- Replace coarse per-band planner collision boxes for SO-101 and xArm7 with
  deterministic convex hulls of complete source collision pieces. Compact the
  hull boundary without shrinking it, preserve readable geom names and provenance,
  and stop rejecting a collision-free xArm drawer pull while retaining a known
  base-to-link collision.
- Bound real-time worker catch-up work while retaining fixed native timesteps.
  Slow physics now warns and discards wall-clock lag instead of accumulating
  an unbounded backlog that disconnects controls and cameras. Explicit rollouts
  still execute their complete requested duration.
- Replace SAPIEN's finite bottle-cap guide with a free rigid cap using packaged
  first-party thread surfaces and native GPU PhysX contact. Add optional CUDA
  worker state handling and the `sapien-gpu` extra; preserve ordinary SDK
  control, camera and reset contracts. Cube and drawer scenes retain CPU physics.

- Replace Isaac's finite cap guide with a packaged free-body USD assembly using
  the same native PhysX thread surfaces, roof and inertia as SAPIEN. Select GPU
  dynamics/broadphase for the bottle scene and remove the obsolete shared guide.
  Add an offline USD builder and source-surface/inertial/composition checks.
  Extend physical cap assertions to Isaac; licensed native execution remains
  required before claiming contact, reset, rendering or workspace acceptance.

- Initialize SAPIEN reference prop poses before scene insertion so GPU PhysX
  retains the declared placement. Add a native GPU regression that detects
  cubes incorrectly spawning at the origin. The reference workspace still uses
  CPU physics; this corrects its shared prop importer for GPU integration.

- Keep Isaac's xArm gripper loop-closing joints outside the articulation tree,
  as required by PhysX, while retaining both physical constraints. Test the
  complete imported manufacturer joint trees and loop-anchor alignment through
  standalone OpenUSD and the URDF converter used by Isaac. Imported full-inertia
  checks also detect the upstream 0.1.3 principal-axis bug; document the verified
  0.3.3 upstream correction without adding an inertia-rewriting workaround.

- Correct Isaac's USD camera aperture-offset signs so off-center site intrinsics
  project into the declared image coordinates. Add a standalone OpenUSD projection
  and deprojection regression; Isaac rendered RGB-D still requires native validation.

- Place new reference drawer cameras in front of the cabinet so the handle face
  is visible at the robot's home pose. Preserve existing scene configurations;
  native RGB-D tests check the handle's actual front plane for both robot models.

- Convert G2's 50 N jaw-force rating through its actual linkage to a 4.2039 N m
  native motor limit; the former 50 N m reference setting could grossly overload
  the hand. Retain MuJoCo's reference passive-link armature and joint-limit
  response so loaded fingers preserve the four-bar geometry and reported jaw
  width. Native MuJoCo/SAPIEN regressions cover grasp retention, arm tracking,
  and reopening without changing public controls or motion tolerances.

- Give SAPIEN's passive screw cap the same 0.001 N m resistance budget as
  MuJoCo through a native force-limited velocity damper. Its default joint
  friction let the cap unwind under gravity after the fingers released it.
  Native tests now check release away from the end stop and rotation in both
  directions, so a limit cannot conceal missing thread resistance.

- Set SAPIEN robot collision margins to 2 mm per shape for 2 ms stepping.
  The native centimetre margin generated over a thousand speculative contacts
  between the closed xArm hand's convex pieces, slowing physics beyond real
  time and starving ordinary control. Collision meshes, exclusions, rest offsets
  and material friction remain unchanged.

- Use 2 ms native substeps in SAPIEN reference scenes, matching the other engines.
  The previous 10 ms step left coupled hand/contact solves with residual joint
  velocities that could prevent a stationary grasp from completing its lift.
  Finer integration improves convergence while retaining motor gains and force
  limits. SDK command rates and camera profiles remain independent.

- Preserve native SAPIEN jaw coupling under contact. Equal position targets
  alone let the YAM jaws disagree by nearly 4 mm and shift the TCP under an
  asymmetric handle load. The native URDF relation, with the existing shared
  actuator budget, keeps the jaws coupled without changing controller gains or
  consumer completion thresholds.

- Give the 60 g, 50 mm reference cubes their uniform-body inertia in every
  backend. SAPIEN now imports single-body props through its existing URDF
  loader, retaining declared mass, COM and inertia instead of overriding mass
  alone after its builder calculated inertia at a different density.

- Prioritize queued simulation state/control requests ahead of camera captures,
  preserving one serialized native transaction. Real-time SDK pump ticks no
  longer send redundant clock requests; native reads, writes and captures
  already integrate elapsed time. This reduces camera-induced command latency.
- Decompose concave manufacturer arm and housing collision meshes through the
  existing CoACD build step, alongside fingers. Preserve physical recesses that
  a single convex hull filled, causing false YAM forearm/hand collisions during
  wrist motion. Convex source meshes and visual geometry remain unchanged.
- Build conservative planning bounds per physical link, independent of how its
  collision mesh is partitioned. This avoids redundant overlapping spheres and
  keeps native convex decomposition from multiplying planning work.
- Balance MuJoCo hand actuation with Menagerie's native fixed tendon, preserving
  the single motor's total force, gain and reflected inertia. This prevents the
  YAM jaws from diverging under handle contact and displacing the TCP.
- Use MuJoCo's documented elliptic friction cone and impedance ratio 10 across
  reference scenes to reduce slow contact creep. Native xArm handle-grasp
  retention is now checked alongside joint motion and RGB-D conformance.
- Apply Menagerie's published finger-pad contact response to the original
  manufacturer collision meshes. Prevent excessive cube penetration and YAM
  jaw oscillation; add a native grasp-width and settling regression.
- Give the MuJoCo reference cap native dry thread resistance and check released
  axial retention, instead of letting gravity back-drive a frictionless helix.
- Give the reference drawer explicit passive joint damping, preventing an
  undamped pull from coasting into its limit and bouncing back after release.
  Preserve the passive joint and normal gripper contacts across engine adapters.

- Advance reference physics by elapsed monotonic time before reading state or
  changing targets, so rendering delays do not silently slow declared joint
  velocities or replay new targets into the past. Retain fixed engine substeps
  and explicit rollout stepping through `worlds.cell.options.real_time: false`.
  Use ManiSkill's standard 100 Hz SAPIEN physics cadence; SDK command and camera
  rates remain independent. New scene control defaults come from the physical
  SDK embodiment declarations instead of a simulator-wide 50 Hz / 0.5 rad/s.

- Place the reference YAM at a tabletop working pose with its TCP near
  `(0.36, 0, 0.14)` m and the hand pitched 45 degrees down. The prior pose lay
  near the inner boundary for horizontal approaches. The model and owner limits
  are unchanged; physical adapters retain their measured configuration. Place
  the bottle and cabinet clear of the starting hand and check native startup
  contacts for both robot models in every reference scene.

- Resume robot-pump cadence after a scheduling delay instead of replaying missed
  ticks under a newly issued target. Catch-up bursts after planning could advance
  simulated arms past their target, causing owner-envelope refusals and timeouts.

- Skip missed camera capture slots after slow rendering/publication instead of
  issuing catch-up bursts. Those bursts starved shared simulation physics at full
  RGB-D resolution and caused ordinary SAPIEN joint/orientation motion to time out.
  Preserve native physics timesteps, owner limits, and every acquired frame's
  normal publication/recording path.

- Preserve site-selected robot-part, base-frame, and wrist-camera owner names
  in reference simulation worlds, allowing the same program scopes as physical
  hardware. Reject binding two logical parts to the single reference robot.

- Correct MuJoCo principal-point signs so RGB pixels and axial depth agree
  with the declared optical intrinsics for off-center cameras. Native engine
  acceptance now checks projected RGB geometry and depth with unequal focal
  lengths, an off-center principal point, and submillimetre depth units.

- Preserve reference simulation state across control runs by default. Starting
  a new run after jog release no longer teleports the robot or rearranges props
  underneath an already observed command. Shared-world initialization owns the
  home pose; per-arm episode hooks do not home again. Rollouts can explicitly
  select `worlds.cell.options.reset_on_episode: true` for native scene reset.
- Reject invalid portable-scene robot base frames and unsafe URDF joint-limit
  widening before loading the optional MuJoCo compiler dependency.
- YAM's pinned I2RT adapter accepts the expected CAN reply in either the original
  10 ms receive or 9 ms recovery window instead of discarding a valid late reply.
  Unrelated IDs do not restart those deadlines; explicit encoder reply IDs,
  matching motor errors, retry counts and scoped communication failures remain
  intact. Both patched methods are signature-checked before installation.

- Replace a scheduler-dependent per-part uplink test with controlled reducer
  admission times, verifying independent 10 Hz budgets and nonstarvation without
  requiring unrelated streams to have identical phases.

- Preserve original runtime exception text, JSON metadata and nested causes through
  `RuntimeFault.from_exception`; existing typed faults pass through unchanged and
  only credential material is redacted. Teardown/reporting failures no longer
  replace the primary live motion failure.

- Refuse YAM reads and commands when its CAN/server writer has stopped or real
  feedback cache updates stall, even if vendor observation timestamps advance.

- Accept optional grasp metadata alongside the standard YAM physical gripper
  mapping when resolving model sources; retain rejection of changed jaw/action
  mappings and all source geometry/provenance validation.

- Keep MuJoCo renderer creation and destruction on its owning camera-pump thread so
  EGL/OpenGL RGB-D capture closes without cross-thread context failures.
- Transform link-local conservative collision-sphere offsets with each simulated
  body's pose instead of treating every sphere as body-origin centered.

## Released changelogs

- [`0.1.11` — 2026-08-30](docs/changelogs/CHANGELOG-0.1.11.md)
- [`0.1.10` — 2026-08-29](docs/changelogs/CHANGELOG-0.1.10.md)
- [`0.1.9` — 2026-08-28](docs/changelogs/CHANGELOG-0.1.9.md)
- [`0.1.8` — 2026-08-28](docs/changelogs/CHANGELOG-0.1.8.md)
- [`0.1.7` — 2026-08-28 (not published)](docs/changelogs/CHANGELOG-0.1.7.md)
- [`0.1.6` — 2026-08-28 (not published)](docs/changelogs/CHANGELOG-0.1.6.md)
- [`0.1.5` — 2026-08-27](docs/changelogs/CHANGELOG-0.1.5.md)
- [`0.1.4` — 2026-08-27 (withdrawn)](docs/changelogs/CHANGELOG-0.1.4.md)
- [`0.1.3` — 2026-08-27](docs/changelogs/CHANGELOG-0.1.3.md)
- [`0.1.2` — 2026-08-25](docs/changelogs/CHANGELOG-0.1.2.md)
- [`0.1.1` — 2026-08-24](docs/changelogs/CHANGELOG-0.1.1.md)
- [`0.1.0` — 2026-08-23](docs/changelogs/CHANGELOG-0.1.0.md)
