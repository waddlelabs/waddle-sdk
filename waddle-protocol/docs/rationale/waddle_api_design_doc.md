# Waddle API Design Doc

**Design Document — Draft v0.9**
*(v0.9: added §7, a third adversarial pass attacking the v0.8 amendments themselves; proposes N11–N19 (not yet applied) and a closed-side v1 cut list. v0.8: applied amendments N1–N10 into the body (§2.3, §2.4, §2.6, §2.7, §2.8, §3.1, §3.2, §3.6), marked inline with (N#) tags. v0.7: added §6, a final adversarial pass against the v0.6 design, adopting ten normative amendments (N1–N10) and a v1 cut list. v0.6: unified internal and external vocabulary into one glossary — permissions are **grants** (capability now means robot skills only, protocol evolution uses **feature flags**), the data product is the **Corpus** (`waddle.corpus`), and the intervention lifecycle (engage/settle/release/**retake**) is adopted from production into the protocol FSM. v0.5: added §3, the explicit component layout of every artifact. v0.4: adopted the final artifact naming — `waddle-protocol` / `waddle-core` / `waddle-sdk` / `waddle-cpp` / `waddle_ros` / `waddle-proxy` / `waddle-relay` — added the protocol glossary (§2.8), and added Appendix A: the rename plan for the existing internal cell codebase.)*

---

## 1. Philosophy and Functionality

### 1.1 What Waddle is

Waddle is a supervision layer for real-world robot policy rollouts. It attaches to a customer's *existing* stack — their robot, their cameras, their policy server, their control loop — and takes ownership of everything that happens *around* the policy's decisions:

- **Watching** — ingesting camera feeds, proprioception, and action streams; detecting stalls, anomalies, and impending failures in real time.
- **Intervening** — when a rollout goes wrong, dispatching a correction from the best available source: Waddle's remote teleoperators, Waddle's code-as-policy agents, or a customer-supplied source (e.g., a local leader arm).
- **Resetting** — returning the scene to a valid initial state between episodes, via code-as-policy agents, teleoperators, or customer-supplied routines, so rollouts can run unattended.
- **Judging** — annotating every episode with success/failure labels, dense reward signals, sub-task segmentation, and intervention markers, using VLM judges and learned classifiers.
- **Improving** — turning the resulting corpus (autonomous rollouts + human corrections + reward labels) into better policies, through a post-training toolkit: intervention-boundary smoothing, jitter cleanup, filtered behavior cloning, and online-RL recipes in the HIL-SERL / RLPD family.

The one-sentence analogy: **Weights & Biases instrumented your training loop; Waddle instruments your deployment loop.** The critical difference — and the source of nearly every design decision below — is that W&B is read-only, while Waddle must hold a *write path into the robot*. Intervention and reset are actuation, not telemetry.

### 1.2 The core loop

Waddle's product is a loop, and the API exists to let that loop run against any stack:

```
            ┌──────────────────────────────────────────────┐
            │                                              │
            ▼                                              │
   [reset scene] ──► [rollout under watch] ──► [judge & label]
        ▲                    │                             │
        │              failure/stall                       │
        │                    ▼                             ▼
        │             [intervene]                [dataset + metrics]
        │                    │                             │
        └────────────────────┘                   [optimize policy]
```

Every arrow in this loop is a product surface. Every trip around it produces training data. The interventions are DAgger-gold (on-policy failure states with expert corrections); the resets are free episode boundaries; the judgments are reward labels. Waddle is simultaneously an ops product (keep rollouts running unattended) and a data product (the corpus that comes out the other side).

### 1.3 Design principles

**P1 — Opinionated semantics, unopinionated transport.** Waddle takes hard positions on *meaning*: what an episode is, what an action space is, what units and frames and timestamps look like, what "intervention" and "reset" mean as protocol states. Waddle takes no position on *plumbing*: whether actions travel over CAN, a ROS topic, a vendor TCP SDK, or a custom USB protocol is the customer's business, expressed through a five-verb control interface they implement however they like.

**P2 — The grant lattice: every declaration unlocks a feature.** Waddle never demands full integration. A customer who provides only camera taps gets monitoring and episode labeling. Add a `hold()` callable → safety pauses. Add an `act()` callable → intervention takeover. Add a URDF → embodiment-portable teleoperation with Waddle-side retargeting and IK. Add calibrated camera intrinsics/extrinsics → 3D teleoperator overlays and geometric tripwires. There is no compatibility cliff — only a slope where more structure buys more product.

```
observe ──► pause ──► takeover ──► reset
(cameras)   (+hold)   (+act)       (+home / scene agents)
```

**P3 — Never in the hot path uninvited.** The customer's control loop is sacred. Supervision (VLM judging, anomaly detection) runs asynchronously against a shadow media stream, never inline. The single point where Waddle touches the loop — `ep.gate(action)` — is a local, nanosecond-scale passthrough in the nominal case. It becomes an action *source* only when an intervention has been explicitly claimed through the control plane. Latency added to a customer's 50 Hz loop is a bug of the highest severity.

**P4 — Open protocol, open thin client, closed brain.** The SDKs, the wire schemas, the adapters, and the local logging are Apache-2.0 on GitHub. Everything that *decides* — when to intervene, who intervenes, how to reset, whether the episode succeeded, how retargeting maps an operator rig into a customer's action space — lives behind the RPC boundary in Waddle's control plane (cloud, or a licensed on-prem relay container). The open SDK is genuinely useful standalone (local MCAP logging, episode bookkeeping, dataset export), which is what gets it embedded in READMEs; the paid product begins the moment supervision, intervention, or reset is invoked.

**P5 — Modularity as a first-class contract.** Intervention sources, reset strategies, and judges are plugin interfaces, not built-ins. Waddle ships defaults (teleop network, code-as-policy agents, VLM judges), but a customer can replace any slot with their own implementation and still get the orchestration, labeling, and data flywheel around it. The customer who "doesn't need our teleop layer" is not a lost customer — they are a customer for detection, orchestration, resets, labeling, and optimization.

**P6 — Every byte lands in formats the ecosystem already reads.** Rollouts log to MCAP (opens natively in Foxglove), export to LeRobotDataset (drops into any LeRobot training script) and Rerun `.rrd`. Waddle's value is the semantic layer — episodes, tasks, claims, judgments — not a captive file format.

### 1.4 What Waddle is not

Waddle is not a policy server (LeRobot async inference, openpi `serve_policy`, VLAgents own that layer — Waddle wraps or proxies them). It is not a middleware (ROS, dora, Zenoh own transport — Waddle bridges to them). It is not a visualization tool (Foxglove and Rerun own that — Waddle feeds them). And it is not a fleet manager in the AMR sense (Formant/InOrbit own navigation fleets — Waddle owns the manipulation-policy rollout loop). Positioning discipline here is what keeps the integration surface small.

---
## 2. Design Specification

### 2.1 System topology

```
 CUSTOMER SITE                                │        WADDLE CLOUD (closed)
                                              │
 ┌────────────────────────────┐               │   ┌─────────────────────────────┐
 │  Customer process           │               │   │  Control plane (gRPC)       │
 │  ┌───────────────────────┐ │   gRPC        │   │  • episode registry          │
 │  │ waddle-sdk (open)     │◄├───────────────┼──►│  • supervisor (VLM judges,   │
 │  │ • descriptors          │ │  control      │   │    anomaly detection)        │
 │  │ • gate() / rollout()   │ │  plane        │   │  • intervention orchestrator │
 │  │ • local tripwires      │ │               │   │  • reset planner             │
 │  │ • MCAP writer          │ │               │   │    (code-as-policy agents)   │
 │  └───────────┬───────────┘ │               │   │  • retargeting + IK service  │
 │              │ shared mem   │               │   └──────────────┬──────────────┘
 │  ┌───────────▼───────────┐ │  WebRTC       │                  │
 │  │ media taps (SDK-owned │◄├───────────────┼──► teleoperator consoles,
 │  │ capture threads)      │ │  media plane  │    labeling pipeline, dashboard
 │  └───────────────────────┘ │               │
 └────────────────────────────┘               │
        (optional) waddle-relay container:     │
        terminates media locally, buffers      │
        MCAP, runs local tripwires, licensed   │
        closed binary for on-prem / low-latency│
```

Two planes, deliberately separate. The **control plane** is gRPC with IDL-first protobuf schemas: typed, versioned, bidirectional streams for episode lifecycle, grant negotiation, intervention claims, and reset requests. The **media plane** is WebRTC (LiveKit): camera and depth streams flow to supervisors and teleoperators with sub-150 ms glass-to-glass latency; teleoperator action streams flow back over a WebRTC data channel. Nothing high-bandwidth ever touches gRPC; nothing stateful ever touches WebRTC.

The SDK runs *inside the customer's process* (Python) or *alongside it* (ROS node, relay container). It owns background capture threads for declared media taps, the MCAP writer, the local tripwire evaluator, and the `gate()` state machine. It contains no models and makes no decisions.

### 2.2 The resource model: declarations

Everything the customer tells Waddle at `init` is a typed descriptor defined in protobuf. Python/ROS/C++ SDKs are codegen plus sugar. Conventions are pinned, not negotiated: SI units, radians, meters, right-handed frames, z-up, quaternions `wxyz`, timestamps as nanoseconds from a monotonic clock synced at session start.

```protobuf
message RobotDescription {
  string name = 1;
  ActionSpace action_space = 2;          // REQUIRED: the one hard opinion
  optional bytes kinematics_urdf = 3;    // unlocks Waddle-side IK & retargeting
  optional FrameGraph frames = 4;        // named frames + static transforms
  repeated Grant grants = 5;             // negotiated, not assumed
  map<string,string> vendor = 15;        // never interpreted; logged verbatim
}

message ActionSpace {
  oneof space {
    JointPosition joint_position = 1;    // ordered names, limits, units
    JointVelocity joint_velocity = 2;
    EEPoseDelta   ee_delta        = 3;   // frame_id, rotation convention
    EEPoseAbs     ee_absolute     = 4;
    BaseTwist     base_twist      = 5;
    Composite     composite       = 6;   // named parts, each an ActionSpace
    Opaque        opaque          = 7;   // monitor-only escape hatch
  }
  double rate_hz = 10;
  ChunkingSemantics chunking = 11;       // horizon, replan policy, interpolation
  optional GripperSpec gripper = 12;     // parallel | suction | dexterous
}
```

Design notes:

- **`ActionSpace` is a closed enum with complete execution semantics.** This is the anti-RLDS decision: an action is never "a float tensor," because Waddle's teleop retargeting and reset agents must *write* into this space safely. The closed enum means Waddle implements retargeting once per canonical type (≈6 types) instead of once per robot: N robots × M types collapses to M.
- **`ChunkingSemantics` is part of the action space.** VLA policies emit 10–50-step chunks; whether a new chunk *replaces, blends with, or queues behind* the executing one, and how actions interpolate between policy rate and control rate, are properties Waddle must know to hand control back and forth mid-episode. Declared, not discovered.
- **`Composite` with named parts** makes a bimanual-plus-grippers robot `{left_arm: JointPosition(7), right_arm: JointPosition(7)}` rather than a mystery 14-vector.
- **`Opaque` is a real tier, not a failure.** Unknown encodings still get monitoring, episode bookkeeping, and video-based judging (those need only pixels). They don't get takeover until someone writes a mapping. Full functionality is the carrot for structured declaration.

Cameras follow the ONVIF playbook — standardize the profile, not the device:

```protobuf
message CameraDescription {
  string name = 1;                       // "wrist_left", "overhead"
  uint32 width = 2;  uint32 height = 3;  double fps = 4;
  Encoding encoding = 5;                 // RGB8 | BGR8 | Z16 | JPEG | H264
  optional Intrinsics intrinsics = 6;    // pinhole + distortion (ROS CameraInfo model)
  optional string frame_id = 7;          // extrinsics via FrameGraph
  StreamPolicy stream = 8;               // e.g. full-rate local, 15fps H264 uplink
}
```

Uncalibrated RGB unlocks monitoring and VLM judging. Intrinsics + a frame unlock 3D teleoperator overlays, workspace tripwires, and mark-based agent actuation. Depth streams are *ingested selectively* (downsampled on uplink, full-rate in local MCAP) — see §5 on bandwidth.

Non-camera sensors use a generic `TimeSeries` descriptor (name, dtype, shape, units, frame): joint states, F/T, IMU, without enumerating sensor types forever.

### 2.3 The control contract: five verbs

The write path is the smallest surface in the system:

```python
class Control:
    send(chunk: ActionChunk)   # stream actions into the robot (grants TAKEOVER, RESET)
    hold()                     # freeze safely, hold position     (grants PAUSE)
    resume()                   # release a hold
    home()                     # move to declared home pose       (grants RESET, partial)
    estop()                    # hardware or software stop        (declared, with latency bound)
```

Each verb is a customer-provided callable (Python), topic (ROS), or endpoint (relay). Grant negotiation at `init` records which verbs exist and what guarantees they carry — `hold` latency bound, whether `estop` is hardware — and the backend plans interventions strictly against declared grants. Two runtime rules keep grants honest:

- **Grants are live, not static** *(N6)*. A latency bound measured by `waddle doctor` on a quiet host is a claim the host's load average will eventually break, so the session heartbeat carries recently *measured* verb latencies, and the control plane **demotes** a grant (with an operator-visible event) when observed behavior violates its declaration. The grant lattice is a live lattice; the planner always plans against current, not onboarding-day, grants.
- **The lease enforcement point is declared** *(N7)*. Grant negotiation records whether the single-writer lease is **enforced** (a broker, a ROS mux with exclusive ownership of the command topic, a proxy owning the only socket to the robot client) or **advisory** (in-process callables, where nothing physically stops the customer's loop from writing during a takeover). The intervention planner treats advisory-lease integrations more conservatively (preferring `HOLD_FIRST` handoffs), and `waddle doctor` includes a NOOP-compliance test that verifies the customer's loop actually stands down during a simulated bypass.

The customer may register **multiple command interfaces** for `send`:

```python
control = waddle.Control(
    send={"joint_position": robot.command_joint_pos,       # policy's native space
          "ee_delta":       my_ik_streamer},               # unlocks Cartesian teleop
    hold=robot.hold, resume=robot.resume, home=robot.go_home,
)
```

Waddle's orchestrator picks the richest interface each consumer supports: teleop prefers `ee_delta` (natural for a human rig), reset agents may prefer `joint_position`. This is how "tap into *our* IK" (Company A) and "use the vendor's IK" (Company B) become the same declaration.

**Safety split.** `hold()` and tripwires have a *local* fast path: the SDK evaluates workspace bounds, joint-limit margins, force thresholds, and a control-plane heartbeat watchdog on-device, and can trigger the customer's `hold()` with no network round trip. Cloud supervision layers intelligence above; the site stays safe through a network partition. Fail safe locally, get smart remotely.

A vocabulary distinction the protocol enforces (see the glossary in §2.8): a **tripwire** is Waddle-side and *requests* safety actions through the declared verbs; an **envelope** is the hard, non-bypassable gate chain owned by whoever owns the hardware (the customer's safety layer, or the broker in a Waddle-operated cell). Waddle never claims to provide an envelope, and every intervention or reset action Waddle sends is subject to the owner's envelope like any other command. Keeping the two words distinct keeps the liability boundary legible.

### 2.4 The runtime model: episodes, gates, claims

An episode is a protocol-level state machine, mirrored between SDK and control plane:

```
RESETTING ─► READY ─► RUNNING ─► (INTERVENTION ⇄ RUNNING) ─► TERMINAL
                                       │                        │
                            engage → settle → release           │
                                       └─ retake ─► (new episode, claim held)
                                                                 │
                                         success | failure | abort
                                         (judged async, label attached)
```

The Python surface:

```python
waddle.init(project=..., robot=..., cameras=..., control=..., ...)

with waddle.rollout(task="fold the towel") as ep:   # ← blocks until scene reset completes
    while not ep.done:                              # ← flips on judge/operator/timeout
        obs    = get_obs()
        action = policy(obs)
        action = ep.gate(action, obs)               # ← the only touch point
        send(action)
```

`gate(action, obs)` does three things in one call: (1) timestamps and logs the (obs, action) pair to local MCAP; (2) checks local tripwires; (3) consults the claim state. Nominally it returns `action` unmodified in sub-microsecond time. When the control plane (supervisor decision) or a local source (leader-arm clutch, operator claim) has **claimed** the episode, `gate` returns intervention actions instead — pulled from a jitter-buffered stream on the data channel — and tags every returned action with its provenance (`policy | teleop | agent | custom`). The intervention segment is therefore *labeled at write time*: pre-annotated DAgger data, no post-hoc alignment.

(Protocol vocabulary: winning a **claim** — the orchestration-level assignment of an episode to an actor — leads to acquiring the **lease**, the actuation-level single-writer right on the robot; takeover and release are lease handoffs under an existing claim. §2.8 pins these terms.)

An intervention span follows the **intervention lifecycle**: **engage** (lease handoff per the declared `HandoffPolicy`) → **settle** (the intervenor stabilizes the scene) → either **release** (lease returns, policy re-primed on fresh observations, episode continues) or **retake** (the current episode is terminated and a new one opened *under the still-held claim* — for when the intervenor decides the attempt is unsalvageable and effectively performs the reset themselves). These four states and their names come directly from the production intervention system and are normative in the protocol FSM. Two accounting rules keep retake from becoming a statistics loophole *(N2)*: the terminated episode closes with the distinct terminal outcome **`aborted_retake`** (never silently folded into success-rate denominators — see §2.6), and the successor episode must still pass **reset verification** before entering RUNNING, or it carries a permanent `reset_unverified` flag in its sidecar.

**Chunk handoff.** Because policies emit chunks and interventions arrive mid-chunk, the claim protocol includes a declared `HandoffPolicy`: `IMMEDIATE` (drop remaining chunk, cross-fade over `blend_ms` using the space's interpolation rule), `CHUNK_BOUNDARY` (finish the executing chunk, then switch), or `HOLD_FIRST` (freeze via `hold()`, human takes over from rest). Release mirrors it in reverse, with the policy re-primed on fresh observations before un-claim. This contract lives in the protocol so the Python `with` block, the ROS mux, and a C++ client all behave identically.

**Resets happen *between* the `with` blocks.** `waddle.rollout()` does not yield until the reset pipeline reports the scene valid — Waddle owns everything outside the block; the block is the episode. `ep.done` flips when a judge, an operator, a timeout, or the customer (`ep.terminate(...)`) calls it.

**Integration idioms.** Three tiers, one protocol:

| Tier | Form | Lines | For whom |
|---|---|---|---|
| 1 | `ep.gate()` inline in the loop | ~6 | anyone with a hand-rolled loop |
| 2 | `@waddle.watching` decorator around an episode fn | ~2 | structured codebases |
| 3 | `waddle proxy` / `waddle-ros` mux | 0 (config) | LeRobot-async & openpi websocket users; ROS graphs |

Tier 3 exploits the fact that the policy-server pattern (LeRobot async inference's gRPC client/server; openpi's `WebsocketClientPolicy`) already routes every observation and every action chunk through one socket. `waddle proxy --policy ws://gpu:8000 --listen :9000` puts Waddle in the write path with a one-URL change. For ROS, `waddle-ros` is a YAML-configured node using the standard priority-mux idiom (à la `twist_mux`): Waddle publishes on a higher-priority command topic when it holds a claim.

### 2.5 The module system: plugin slots

Three slots, three small ABCs, all orchestrated by the (closed) planner but implementable by anyone:

```python
class InterventionSource(waddle.plugins.InterventionSource):
    name: str
    spaces: list[str]                    # which ActionSpace types it can emit
    def engaged(self) -> bool: ...       # local sources: clutch/deadman poll (each gate tick)
    def start(self, ctx: ClaimContext): ...
    def get_action(self, obs) -> Action: ...
    def stop(self): ...

class ResetStrategy(waddle.plugins.ResetStrategy):
    name: str
    def applicable(self, scene_report) -> bool: ...
    def execute(self, ctl: Control, ctx) -> ResetResult: ...

class Judge(waddle.plugins.Judge):
    name: str
    def on_episode(self, episode_handle) -> Labels: ...   # async, off the hot path
```

Registered at init, in priority order:

```python
waddle.init(...,
    interventions=[MyLeaderArm(), waddle.interventions.Teleop(), waddle.interventions.CodeAsPolicy()],
    resets=[waddle.resets.CodeAsPolicy(scene="tabletop"), waddle.resets.Teleop(),
            waddle.resets.Scripted(my_reset_fn)],
    judges=[waddle.judges.VLM(rubric="towel is folded within the blue square"),
            MyForceThresholdJudge()],
)
```

Waddle's supervisor decides *when* (detection is closed); the registry decides *who* (open). A customer opting out of Waddle teleop simply doesn't register it. Waddle-supplied defaults (`Teleop`, `CodeAsPolicy`, `VLM`) are thin open-source stubs whose `get_action`/`execute`/`on_episode` calls hit the closed control plane — the plugin ABC is exactly the open/closed seam.

### 2.6 The Corpus: data model and the flywheel

The project-level index of episodes — sidecars, labels, judge outputs, and archive references, queryable as one dataset — is the **Corpus**, the same name (and lineage) as the internal component that owns episode indexing today. `waddle.corpus` is its client API. Every sidecar carries **`robot_id`** (and **`cell_id`** where applicable) as first-class fields, so fleet-level queries ("SR across all cells running task X") are joins, not archaeology *(N10)*.

**Recording modes.** What persists beyond the sidecar is a pluggable slot chosen at init: **`Local`** (batteries included — Waddle writes full MCAP episodes to local disk with a retention policy; the default for customers with no logging infrastructure, and the mode the cell's broker recorder reference-implements), **`Reference`** (the customer's existing recorder keeps the bulk bytes; Waddle emits only sidecars whose entries carry references — stream id, time range on the session timeline, content hash — resolved at read time through a small open resolver interface), and **`SidecarOnly`** (semantic records only; nothing bulk persists). Independent of mode, the relay/SDK keeps a short rolling ring buffer and persists **incident clips** around events — intervention spans, tripwire fires, judge-flagged failures, reset verifications — because supervision needs replays even when the customer owns the archive.

Local layout, one MCAP per episode plus a manifest:

```
waddle/<project>/<session>/
  ep_000341.mcap          # /camera/*, /robot/joint_states, /waddle/actions (tagged
  ep_000341.json          #   with provenance), /waddle/events (claims, tripwires)
  manifest.jsonl          # task, outcome, labels, judge scores, intervention spans,
                          #   robot_id / cell_id
```

**Outcome accounting.** Terminal outcomes are `success | failure | abort | aborted_retake`. Retaken episodes are never silently dropped: every Corpus summary reports the retake count, and success rate is always presented both including and excluding retakes, so an operator's judgment call can move a metric only in the open *(N2)*.

Exports: `ds.to_lerobot(path)` (LeRobotDataset, training-ready), `waddle export --format rrd` (Rerun), and MCAP opens in Foxglove untouched. The annotation pipeline (closed) attaches: success/failure with confidence, dense task-progress reward from the VLM judge, sub-task segmentation, failure taxonomy, and intervention-boundary markers.

**Judge integrity.** Because the judge labels the data that trains the policy the judge then evaluates, three guards are mandatory rather than optional *(N9)*: every project maintains a held-out, **human-labeled audit slice** (teleoperators double as labelers); judge versions are **pinned per project** — a judge upgrade is an explicit re-baselining event, never a silent change to historical metrics; and the **judge/human disagreement rate** is itself a first-class Corpus metric surfaced to the customer.

The post-training toolkit (open-source interfaces, some closed-hosted implementations):

```python
ds = waddle.corpus.load("kitchen-pilot")
ds = waddle.corpus.smooth_handoffs(ds, blend_s=0.4)     # de-spike policy→human boundaries
ds = waddle.corpus.dejitter(ds, method="awe")           # waypoint-extraction cleanup of VLA jitter
ds_int = ds.filter(provenance="teleop")               # corrections only

waddle.optimize.filtered_bc(ds, base="lerobot/smolvla_base", weight="intervention")  # IWR/Sirius-style
waddle.optimize.hil_serl(robot_hooks=..., classifier=ds.judge_head(), ...)           # online RL, live rig
waddle.optimize.rlpd(offline=ds, ...)                                                # offline+online mix
```

Every transform and optimizer **declares its data requirements** and fails loudly at load time against the project's recording mode ("this project records SidecarOnly; `filtered_bc` requires Local or Reference-with-resolver") — `SidecarOnly` is the metrics/ops tier, not the data tier, and the product tiering says so explicitly *(N8)*.

This is the retention argument: even as the customer's policy improves and interventions get rarer, the corpus and the recipes keep compounding.

### 2.7 Conformance, versioning, and the proprietary boundary

- **`waddle doctor`** exercises a declared integration end-to-end: round-trips a no-op chunk, measures `hold()` latency, checks timestamp monotonicity and clock skew (NTP/PTP report), validates joints against the URDF, reprojection-sanity-checks each calibrated camera, and — for advisory-lease integrations — runs the **NOOP-compliance test**, verifying the customer's loop actually stands down during a simulated bypass *(N7)*. It prints a grant report ("monitor ✓, pause 38 ms ✓, takeover in ee_delta ✓, lease: advisory, reset: home+scripted") and doubles as onboarding UX and support-ticket deflector. Doctor-time measurements seed, and heartbeat measurements maintain, the live grants of §2.3.
- **Golden MCAP fixtures** per canonical action space and camera profile; community adapters certify against them in CI. This is how third parties extend the open SDK without the closed backend ever seeing a malformed stream. Fixtures verify *logic*, not *physics* — the conformance program therefore has a third tier: **timing/soak benches** (hardware-in-loop where feasible) with published per-frontend timing envelopes for blend windows, deadman cutoffs, and hold latency *(N3)*. A deployment's only binding conformance statement remains `waddle doctor` on the actual rig.
- **Version by feature flags, not release numbers.** SDKs pinned inside a robot image for a year keep working because negotiation is per-connection; new features (tactile streams, dexterous-hand spaces) are simply never declared by old SDKs and never planned on by the backend.
- **What is closed, precisely:** failure/anomaly detection models, VLM judges and reward heads, the intervention orchestrator and teleoperator network, retargeting + IK-as-a-service, code-as-policy reset agents, the dashboard, and the relay binary. What is open: schemas, SDKs, adapters, gate/claim/reset protocol, local logging, export, plugin ABCs, and reference implementations of the data-cleanup transforms. The judge never ships to the edge — even latency-sensitive success detection runs on the relay as a licensed binary, not in the pip package.

### 2.8 Artifact family, naming, and glossary

**The artifact family.** The protocol sits above everything as the source of truth; one Rust core is its reference implementation; every frontend is a hollow binding over that core. The naming follows Rust/ecosystem convention (`core`, not `lib*`, at the crate level — the linker emits `libwaddle.so`/`.dylib` from the `cdylib` anyway, so the `libwaddle` brand exists exactly where it's meaningful: the embeddable C ABI artifact).

| Artifact | Name | License / distribution | Contents |
|---|---|---|---|
| Protocol | `waddle-protocol` | open (repo + crates) | protobuf/IDL schemas, the claim/lease FSM spec, the sidecar schema, **conformance fixtures** (wire captures, golden sidecars, behavioral scenarios). The standard itself — implementable without `waddle-core`. |
| Rust core | `waddle-core` | open (crate) | reference implementation: episode/claim FSM, gate, tripwire engine, codecs, media plane, sidecar writer, ring buffer, clock sync. Emits the `libwaddle` C ABI (`cdylib` + cbindgen). |
| Python SDK | **PyPI distribution `waddle-sdk`**, **`import waddle`**, **CLI `waddle`** | open | PyO3/maturin wheels over `waddle-core`, plus descriptors, sugar (`rollout`/`watching`), adapters (LeRobot, gym), plugin ABCs, `waddle.corpus` / `waddle.optimize`. The CLI uses entry-point subcommand discovery so other packages (including the closed cell package) can register subcommands into the same `waddle …` namespace. |
| C++ binding | `waddle-cpp` | open | thin header + lib over the `libwaddle` C ABI. |
| ROS 2 | `waddle_ros` package, **`waddle_gate`** node | open | lifecycle node (C++ over the C ABI): YAML-declared topics, priority-mux / `ros2_control`-switch takeover, services for control verbs and episode lifecycle. **Never named "bridge"** — see below. |
| Proxy | `waddle-proxy` (`waddle proxy`) | open binary | policy-server impersonation: codecs + the shared semantic core; zero-code integration tier. |
| Edge container | `waddle-relay` | **closed**, licensed binary | media termination on-LAN, local judging/detection modules, buffering through partitions, host-level config. |
| Cloud | control plane | closed SaaS | the brain: supervisor, orchestrator, reset planner, retargeting/IK service, dashboard, flywheel. |

**The hollow-frontend rule, restated as a naming consequence:** if `waddle-sdk`, `waddle-cpp`, or `waddle_ros` contains an `if` statement about claims, leases, handoffs, or timelines, it is a bug — that logic lives in `waddle-core` (or, for independent implementations, is specified by `waddle-protocol` and verified by its fixtures).

**Reserved-word policy.** Two words are deliberately *not* used in any public artifact or doc: **"bridge"** (it names the internal cell orchestration server, and in public robotics vocabulary it means a protocol translator — rosbridge, foxglove_bridge — which would misidentify the proxy/ROS node) and **"broker"** (it names the internal safety owner). Both remain proud internal process names; public surfaces say *relay* and *control plane*. Conversely, "agent" is avoided as a component name everywhere (Datadog-style "Agent" would collide fatally with the literal agents Waddle hosts).

**The protocol glossary.** Frozen in `waddle-protocol` v0. This is the **single vocabulary, internal and external** — the same word means the same thing in the cell codebase, the protocol, the SDKs, and customer-facing docs. Where the production system and the greenfield spec had different words for one concept, the production (bridge/broker) term won; where the production system had a concept without a name, the new term becomes canonical internally too.

| Term | Meaning | Strength / owner |
|---|---|---|
| **gate** | the single point where Waddle touches the customer's loop; passthrough nominally, action source under a claim | SDK/core |
| **claim** | orchestration-level assignment of an episode or work item to an actor (operator, agent, custom source) — the `work_claim` of the work plane | control plane / work plane |
| **lease** | actuation-level single-writer right on the robot; takeover = lease handoff under an existing claim (`handoff_lease`). The **enforcement point** — *enforced* (broker, ROS mux, proxy) vs *advisory* (in-process callables) — is recorded at grant negotiation *(N7)* | hardware owner (customer stack, or broker in Waddle-operated cells) |
| **grant** | a permission the integrator extends to Waddle — pause, takeover, reset, per-verb guarantees. Declared at init, negotiated per-connection, **validated continuously at runtime** (heartbeat-measured verb latencies; violated grants are demoted with an operator-visible event); the **grant lattice** is the slope from observe to reset. The noun sense is canonical *(N1)*: the work plane's `work_grant` is the verb — awarding a claim — not an instance of this Grant | integrator declares; control plane plans against |
| **envelope** | the hard, non-bypassable safety gate chain (limits, keep-outs, e-stop, watchdogs) | hardware owner; Waddle is always subject to it, never the provider of it |
| **tripwire** | Waddle-side local watchdog (bounds, margins, heartbeats, deadmen) that *requests* holds through declared verbs — the canonical name for what the cell calls caller-side softeners | SDK / relay; weaker than an envelope by definition |
| **episode** | one rollout attempt: reset-verified start → terminal outcome (`success \| failure \| abort \| aborted_retake`); the unit of the sidecar. An episode run with supervision enabled is called a *supervised rollout* in prose (descriptive, not a distinct protocol object) | protocol |
| **intervention lifecycle** | engage → settle → release \| retake; **retake** = terminate the episode and open a new one under the still-held claim | protocol (from the production `InterventionLifecycle`) |
| **provenance** | per-action origin tag (`policy \| teleop \| agent \| custom:<name>`), written at gate time; carries authorization semantics (the `operator_initiated` stamp generalizes to a provenance attribute: may bypass approval, never the envelope) | protocol |
| **sidecar** | the small semantic record per episode (boundaries, task, claims, provenance spans, events, labels); bulk bytes may live in customer storage via references | protocol |
| **corpus** | the project-level index of episodes, sidecars, labels, and archive references; the queryable data product (`waddle.corpus`) | control plane (Local mode: cell/SDK) |
| **capability** | *reserved for robot skills* in the CapabilityLibrary sense (code-as-policy actions a cell can perform). Never used for permissions (those are **grants**) or protocol versioning (those are **feature flags**) | cell / control plane |
| **feature flag** | a protocol-evolution unit negotiated per-connection; how pinned SDKs and an evolving backend coexist | protocol |

---
## 3. Component Layout

This section makes the artifact family of §2.8 concrete: what each repository/crate/package actually contains, how it is laid out, what its build emits, and which invariants it owns. The governing picture is a strict dependency DAG — nothing ever depends "sideways" on a sibling frontend, and nothing open depends on anything closed:

```
                        waddle-protocol            (schemas + fixtures; depends on nothing)
                              │  buf codegen
                              ▼
                         waddle-core               (Rust workspace; the only implementation
                              │                     of FSMs, gate, codecs, transport)
        ┌──────────────┬──────┼───────────┬──────────────────┐
        ▼              ▼      ▼           ▼                  ▼
   waddle-ffi     waddle-sdk  waddle-  waddle-relay      (closed modules
   → libwaddle    (PyO3)      proxy    (open chassis      link against the
        │                     (binary)  + closed mods)     relay chassis)
   ┌────┴─────┐
   ▼          ▼
waddle-cpp  waddle_ros
(header)    (C++ node)
```

Build/release pipeline, end to end: `waddle-protocol` publishes versioned schema releases (buf lint + breaking-change CI); `waddle-core` vendors the generated types via `prost`; `maturin` builds `waddle-sdk` wheels (manylinux x86_64 + aarch64, macOS); `cbindgen` emits the C header and the `cdylib` ships as prebuilt `libwaddle` artifacts per platform triple; `waddle-proxy` and the relay chassis are `cargo build --release` static binaries; `waddle_ros` builds under colcon against the prebuilt `libwaddle`. Every artifact's CI runs the same `waddle-protocol` conformance suite — that, not shared code, is what keeps the frontends' *logic* identical; timing equivalence is covered separately by the soak/latency tier *(N3)*.

### 3.1 `waddle-protocol` — the standard

Depends on nothing; everything depends on it. Implementable without `waddle-core` (that is the point).

```
waddle-protocol/
  proto/waddle/v0/
    descriptors.proto      # RobotDescription, ActionSpace (+ChunkingSemantics, GripperSpec),
                           #   CameraDescription, TimeSeries, FrameGraph, Grant
    control.proto          # ActionChunk, the five verbs, HandoffPolicy, EStop declaration
    episode.proto          # episode FSM states & events, Claim, Lease, provenance tags,
                           #   terminal outcomes incl. aborted_retake, reset_unverified
                           #   flag (N2), ResetRequest/ResetResult, reset-verification record
    sidecar.proto          # the per-episode semantic record (§2.6): robot_id/cell_id (N10),
                           #   boundaries, task, claim/lease spans, provenance spans,
                           #   events, labels, refs
    services.proto         # gRPC: Register, Negotiate, StreamObservations, GateActions,
                           #   ClaimEpisode, HandoffLease, RequestReset, Heartbeat
                           #   (carries measured verb latencies for live grants, N6)
    media.proto            # WebRTC data-topic payloads: teleop action stream, clutch/
                           #   episode-mark events, operator telemetry
  fixtures/
    wire/                  # golden wire captures per codec dialect × upstream version
    sidecars/              # golden sidecar records per scenario
    behaviors/             # scripted behavioral scenarios: claimed-while-stalled,
                           #   mid-chunk handoff per HandoffPolicy, backend-partition
                           #   degradation, reset-verification failure, retake
  conformance/             # runner harness: point any implementation at it, get a report;
                           #   plus the timing/soak bench definitions and per-frontend
                           #   timing envelopes (N3)
  docs/
    GLOSSARY.md            # §2.8's table, normative
    FSM.md                 # state diagrams with transition guards, prose semantics
    VERSIONING.md          # feature-flag policy; what "breaking" means here
```

Owns: the meaning of every message and state; the conformance suite; the glossary. Explicitly does *not* contain: any executable networking code, any Python/C++ — generated code is a build product of downstream repos, never checked in here.

### 3.2 `waddle-core` — the reference implementation

A Cargo workspace of small crates with enforced layering (inner crates do no I/O; outer crates own runtimes). This is where the hollow-frontend rule is made structural: everything a binding might be tempted to reimplement lives here exactly once.

```
waddle-core/
  crates/
    waddle-types/        # prost-generated types + canonical conversions, unit/frame
                         #   convention enforcement. No I/O, no clocks.
    waddle-fsm/          # episode/claim/lease state machines as pure transition
                         #   functions. No I/O. Property-tested + fixture-tested;
                         #   this crate IS the behavioral conformance target.
    waddle-gate/         # the gate engine: passthrough fast path, provenance
                         #   tagging, claim-state consultation, teleop jitter-buffer
                         #   consumption, chunk-handoff blending per HandoffPolicy,
                         #   NOOP-marker emission in bypass mode.
    waddle-tripwire/     # tripwire evaluator + control-plane heartbeat watchdog on
                         #   dedicated OS threads; verb-invocation requests out.
    waddle-ingest/       # shm ring buffers, zero-copy frame path, per-source clock
                         #   offset estimation, mapping onto the session monotonic
                         #   timeline. Owns all timestamps.
    waddle-media/        # LiveKit/WebRTC: track publication per StreamPolicy,
                         #   H.264 encode, data-channel teleop stream intake.
    waddle-controlplane/ # tonic gRPC client: session, grant negotiation,
                         #   reconnect/backoff, offline event buffering.
    waddle-sidecar/      # sidecar writer; Local-mode MCAP recorder; event ring
                         #   buffer + incident-clip persistence; Reference-mode
                         #   ref emission.
    waddle-codecs/       # Codec trait (decode/encode/declares), lerobot-async and
                         #   openpi dialects, round-trip certification (§ proxy),
                         #   per-upstream-version fixtures.
    waddle-runtime/      # composition root: owns the tokio runtime + thread pools,
                         #   wires the above into a Session object. The only crate
                         #   bindings and binaries talk to.
    waddle-ffi/          # C ABI over waddle-runtime via cbindgen → libwaddle.
```

Two invariants worth stating as layout, not just intention. **Threading:** `waddle-runtime` owns its own tokio runtime and all capture/tripwire/media threads; nothing in the core ever executes on a caller's thread except the synchronous `gate()` fast path, which is a lock-free read of claim state plus a ring-buffer write. **FFI surface (`waddle-ffi`):** opaque handles only (`WaddleSession*`, `WaddleEpisode*`), plain C structs for data, frames crossing as shm handles + descriptors rather than pixel buffers, every function returning a status code with a last-error string accessor. The C ABI is semver'd independently of the internal crates; internal refactors never move it once stable — and it is explicitly **unstable until both `waddle-sdk` and `waddle_ros` consume it in anger** *(N5)*: stability is declared as an event after two real consumers have shaped it, not assumed from birth.

### 3.3 `waddle-sdk` — the Python frontend

One repo, mixed Rust/Python, built by maturin. PyPI distribution `waddle-sdk`, import `waddle`, console script `waddle`.

```
waddle-sdk/
  rust/                    # the PyO3 shim crate: waddle-runtime in, pyclass handles out.
  python/waddle/
    __init__.py            # init, rollout, watching, log, Control, EStop, Handoff
    descriptors.py         # Robot, JointSpace/EEDelta/Composite/Opaque, Camera,
                           #   TimeSeries, Gripper, Stream, Chunking — sugar that
                           #   compiles to waddle-types protos
    plugins/__init__.py    # InterventionSource, ResetStrategy, Judge ABCs
    interventions.py       # Teleop, CodeAsPolicy — open stubs whose calls hit the
    resets.py              #   control plane; Scripted wraps a user callable
    judges.py              # VLM stub + Labels types
    tripwires.py           # Workspace, JointLimitMargin, … — declarations compiled
                           #   down to waddle-tripwire configs (evaluation is core-side)
    recording.py           # Local | Reference(resolver) | SidecarOnly + resolver ABC
    adapters/
      lerobot.py           # wrap(robot), Robot.from_lerobot(cfg)
      gym.py               # env wrapper
    corpus/                # load(), Episode/Dataset views over sidecars (+resolvers),
                           #   smooth_handoffs, dejitter, to_lerobot, to_rrd
    optimize/              # filtered_bc, hil_serl, rlpd — interfaces + reference
                           #   recipes (torch optional extra)
    cli/                   # `waddle` entry point; subcommand discovery via the
                           #   "waddle.commands" entry-point group (doctor, proxy
                           #   launcher, episode marks, export); cell package
                           #   registers `waddle cell …` here
    _core.pyi              # typed surface of the PyO3 module
```

Boundary rules, enforced in review: no claim/lease/handoff/timeline logic in Python (hollow-frontend); `gate()` is exactly one FFI call; user taps registered as callables are invoked from core-owned threads that acquire the GIL only for the duration of the call and hand the returned buffer straight into `waddle-ingest` (users are steered toward core-native capture — e.g. the RealSense helper — when rates are high); `waddle.corpus`/`waddle.optimize` never import the core at all — they read sidecars and archives, so they work on machines with no robot and no Rust.

### 3.4 `waddle-cpp` — the C++ frontend

Deliberately the smallest artifact: a distribution of `libwaddle` plus ergonomics. No logic.

```
waddle-cpp/
  include/waddle/waddle.h     # generated C header (cbindgen), the actual contract
  include/waddle/waddle.hpp   # header-only RAII wrapper: Session/Episode objects,
                              #   RAII claim guards, std::span views over shm frames
  cmake/                      # find_package(waddle) config; fetches prebuilt
                              #   libwaddle per triple, or builds from source
  examples/                   # inline-gate loop; custom-transport Control impl
```

Shipped as versioned prebuilt archives (header + `libwaddle` per platform) so embedded/vendor-controller integrations never need a Rust toolchain. An idiomatic deeper C++ API is explicitly deferred until a customer demands it.

### 3.5 `waddle_ros` — the ROS 2 runtime target

Not a language binding — a node in the customer's graph, built on the C ABI. Colcon workspace, released via the ROS index, targeting current LTS distros; also shipped as a container for non-builders.

```
waddle_ros/
  waddle_gate/               # the lifecycle node (C++ over libwaddle)
    src/gate_node.cpp        # subscriptions → waddle-ingest; command mux; services
    src/mux.cpp              # priority arbitration; ros2_control controller-switch
                             #   takeover path where a controller_manager exists
    launch/gate.launch.py
    config/example.yaml      # the entire integration surface — see below
  waddle_msgs/               # minimal msgs: EpisodeEvent, Claim, Provenance stamp
                             #   (sensor/control data reuses std ROS msgs untouched)
  waddle_ros_tests/          # launch_testing against protocol behavior fixtures
```

The YAML *is* the declaration layer (`waddle.init` equivalent): camera topics (`image_raw`/`camera_info` pairs — intrinsics and TF extrinsics harvested automatically, which auto-unlocks the calibrated rungs of the grant lattice), joint-state topic, command topic + mux priorities or `ros2_control` controller names, verb services (`hold`/`resume`/`home` service names, e-stop declaration), action-space block, project/task/judges. Node interfaces: subscribes to declared sensor/command topics; publishes the muxed command topic and `~/events`; offers `~/start_episode`, `~/end_episode`, `~/claim_status` services. Because the node is out-of-process from any policy code, this is structurally the native-path integration — its grant report says so.

### 3.6 `waddle-proxy` — the zero-code binary

One static binary (cargo, from `waddle-core` crates — no code of its own beyond CLI and wiring), one config file.

```
waddle-proxy
  ├─ subcommands:  run (default) · codecs (list dialects+versions) · verify
  │                (round-trip certification against a live upstream) · record
  │                (passthrough + capture only)
  ├─ waddle.yaml:  project/task/api key · upstream URL + codec dialect ·
  │                ActionSpace mapping for the action tensor · which obs keys
  │                are cameras (+optional intrinsics) · judges · HandoffPolicy ·
  │                recording mode · episode-boundary source (endpoint | heuristic)
  └─ local surface: :port/episode (start/end/task marks), :port/status, :port/healthz
```

Internals are §2.4's semantic core behind the codec seam: certified-total codecs unlock chunk substitution; framing-only codecs degrade the session to observe-only. Dialects come from the independently-versioned `waddle-codecs` *(N4)*, so upstream schema churn (a LeRobot release, an openpi protocol tweak) ships as a codec update on its own cadence, never forcing a `waddle-core` release or an SDK wheel rebuild. Both faces (listen/upstream) speak gRPC or WebSocket per dialect.

### 3.7 `waddle-relay` — the closed edge container

An open *chassis* (the same `waddle-core` crates: ingest, media termination, sidecar, tripwires, control-plane client) with **closed modules** linked in at Waddle's build: local judge/detection inference, incident triage, licensed activation. Distributed only as a container/appliance image, never as source or crates.

```
waddle-relay (container)
  /waddle/relay              # single binary: chassis + closed modules
  /waddle/models/            # encrypted judge/detector weights, license-gated
  /waddle/config/relay.yaml  # site config: which SDK/proxy sessions to accept,
                             #   uplink policy, retention, LAN discovery
  ports: gRPC (SDK/proxy sessions in) · WebRTC (media) · :healthz
  volumes: incident-clip cache · offline sidecar buffer
```

The relay is the one place closed code runs on customer infrastructure; the layout keeps that surface to a single auditable image whose open chassis behavior is covered by the public conformance suite.

### 3.8 Where each §2 concept lives

| Concept (§2) | Specified in | Implemented in | Exposed by |
|---|---|---|---|
| Descriptors (§2.2) | `descriptors.proto` | `waddle-types` | `descriptors.py`, YAML (proxy/ROS) |
| Five verbs (§2.3) | `control.proto` | user side; invoked via `waddle-runtime` | `Control`, ROS services, proxy synth |
| Episode/claim/lease FSM (§2.4) | `episode.proto` + `FSM.md` | `waddle-fsm` | `rollout()`, mux, proxy |
| `gate()` + handoff (§2.4) | `control.proto` (HandoffPolicy) | `waddle-gate` | `ep.gate()`, mux arbitration, chunk substitution |
| Tripwires vs envelope (§2.3) | `GLOSSARY.md` | `waddle-tripwire` (tripwires only) | `tripwires.py`, YAML |
| Plugin slots (§2.5) | service RPCs | control plane (closed) + user impls | `waddle.plugins` ABCs |
| Sidecar & recording modes (§2.6) | `sidecar.proto` | `waddle-sidecar` | `recording.py`, `waddle.corpus` |
| Conformance (§2.7) | `fixtures/`, `conformance/` | every artifact's CI | `waddle doctor`, `waddle-proxy verify` |

---
## 4. Four Integrations, Concretely

Each scenario uses real, publicly available libraries for the customer's side of the code (`i2rt`, `pyrealsense2`, `xarm-python-sdk`, `pyorbbecsdk`, `lerobot`). Customer-side API symbols follow those projects' public examples; pin exact signatures against the versions in your environment. Everything under `waddle.*` is the API this document proposes.

A map of which slots each company exercises:

| | A (bimanual YAM lab) | B (xArm manufacturer) | C (custom-everything startup) | D (SO-101 + LeRobot) |
|---|---|---|---|---|
| Action space | `Composite{2× JointPosition}` | `JointPosition(7)` | `JointPosition(6)` over custom USB | inherited from LeRobot adapter |
| Teleop retarget path | **their IK** via `ee_delta` iface | vendor IK, but **teleop opted out** | **Waddle IK** from their URDF | Waddle default (SO-101 known) |
| Intervention source | Waddle teleop | **custom leader arm plugin** | Waddle teleop + code-as-policy | Waddle defaults |
| Reset | code-as-policy + teleop fallback | scripted (vendor motions) + Waddle agent | **custom scripted** + teleop fallback | Waddle code-as-policy |
| Judging | VLM + custom F/T judge | VLM rubric | VLM (RGB only) | VLM |
| Integration tier | 1 (inline gate) | 1 (inline gate) | 1 (inline, raw taps) | 3 (wrap / proxy) then 1 |
| Optimization | HIL-SERL | filtered BC | export → their trainer | filtered BC on SmolVLA |

### 4.1 Company A — mature lab, bimanual YAM, 3× RealSense, in-house IK

The interesting demand: *"our teleop system should tap into their IK."* Their policy runs in joint space; their IK is better than anything generic (they've tuned null-space behavior for the YAM's kinematics). So they register **two command interfaces** — Waddle's teleoperator rig emits `ee_delta` streams per arm, their IK turns those into joint targets, and Waddle never needs to know how.

```python
"""Company A: bimanual YAM + 3x RealSense D435, in-house IK, full Waddle stack."""
import numpy as np
import pyrealsense2 as rs
from i2rt.robots.get_robot import get_yam_robot   # i2rt SDK: joint-space YAM control

import waddle

# ── 1. Hardware, exactly as they already run it ─────────────────────────────
left  = get_yam_robot(channel="can_left")          # 6 DOF + linear parallel gripper
right = get_yam_robot(channel="can_right")

def make_rs_tap(serial: str):
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    profile = pipe.start(cfg)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    def tap():                                     # SDK calls this from a capture thread
        f = pipe.wait_for_frames()
        return {"color": np.asanyarray(f.get_color_frame().get_data()),
                "depth": np.asanyarray(f.get_depth_frame().get_data()),
                "t_ns":  int(f.get_timestamp() * 1e6)}
    return tap, intr

tap_over, intr_over = make_rs_tap("827312070XXX")
tap_wl,   intr_wl   = make_rs_tap("827312071XXX")
tap_wr,   intr_wr   = make_rs_tap("827312072XXX")

# ── 2. Their IK, exposed as an ee_delta command interface ───────────────────
from lab.ik import BimanualIK                      # in-house; tuned null-space for YAM
ik = BimanualIK(urdf="yam_bimanual.urdf")

def ee_delta_send(chunk: waddle.ActionChunk):
    """Waddle teleop emits per-arm SE(3) deltas; their IK owns the joint solution."""
    for step in chunk.steps:
        q = ik.solve_delta(left=step["left_ee"], right=step["right_ee"],
                           q_now=np.r_[left.get_joint_pos(), right.get_joint_pos()])
        left.command_joint_pos(q[:7]); right.command_joint_pos(q[7:])

def joint_send(chunk: waddle.ActionChunk):         # native space: policy + reset agent
    for step in chunk.steps:
        left.command_joint_pos(step["left"]); right.command_joint_pos(step["right"])

# ── 3. Declarations ─────────────────────────────────────────────────────────
robot = waddle.Robot(
    name="yam-bimanual-01",
    action_space=waddle.Composite(
        left =waddle.JointSpace(joints=[f"l{i}" for i in range(6)] + ["l_grip"],
                                units="rad", rate_hz=50,
                                gripper=waddle.Gripper.parallel(dim=-1, open=1.0, closed=0.0)),
        right=waddle.JointSpace(joints=[f"r{i}" for i in range(6)] + ["r_grip"],
                                units="rad", rate_hz=50,
                                gripper=waddle.Gripper.parallel(dim=-1, open=1.0, closed=0.0)),
        chunking=waddle.Chunking(horizon=20, replan="IMMEDIATE", interp="linear"),
    ),
    kinematics="yam_bimanual.urdf",                # unlocks 3D overlays + geometric tripwires
)

control = waddle.Control(
    send={"joint_position": joint_send,
          "ee_delta":       ee_delta_send},        # ← teleop taps THEIR IK here
    hold=lambda: (left.hold(), right.hold()),
    resume=lambda: None,
    home=lambda: (left.command_joint_pos(HOME_L), right.command_joint_pos(HOME_R)),
    handoff=waddle.Handoff.IMMEDIATE(blend_ms=300),
)

class SpikeJudge(waddle.plugins.Judge):            # custom judge alongside Waddle's VLM
    name = "ft_spike"
    def on_episode(self, ep):
        ft = ep.series("/robot/ft_wrist")
        return waddle.Labels(flags=["force_spike"] if (np.abs(ft.values) > 40.0).any() else [])

waddle.init(
    project="towel-folding-pilot", api_key="wd_live_…", mode="relay",   # on-prem relay: video stays on LAN
    robot=robot, control=control,
    cameras={
        "overhead":    waddle.Camera(tap=tap_over, intrinsics=intr_over, frame_id="cam_overhead",
                                     stream=waddle.Stream(local="full", uplink="h264@15fps")),
        "wrist_left":  waddle.Camera(tap=tap_wl, intrinsics=intr_wl, frame_id="l_wrist"),
        "wrist_right": waddle.Camera(tap=tap_wr, intrinsics=intr_wr, frame_id="r_wrist"),
    },
    series={"/robot/ft_wrist": waddle.TimeSeries(shape=(6,), units="N,Nm", tap=lab_ft_tap)},
    interventions=[waddle.interventions.Teleop(rig="bimanual", space="ee_delta")],
    resets=[waddle.resets.CodeAsPolicy(scene="tabletop_softgoods"),
            waddle.resets.Teleop()],               # human fallback when the agent punts
    judges=[waddle.judges.VLM(rubric="towel folded into quarters inside the marked zone"),
            SpikeJudge()],
    tripwires=[waddle.tripwires.Workspace(aabb=[[-.6,-.5,0],[.6,.5,.7]]),   # local, no-network
               waddle.tripwires.JointLimitMargin(rad=0.05)],
)

# ── 4. Their rollout loop, 6 lines changed ──────────────────────────────────
for _ in range(200):                               # overnight unattended eval
    with waddle.rollout(task="fold the towel into quarters") as ep:   # blocks through reset
        while not ep.done:
            obs    = collect_obs(left, right)      # their code, unchanged
            action = policy.infer(obs)             # their VLA, wherever it runs
            action = ep.gate(action, obs)          # log + tripwires + claim check
            joint_send_single(action)              # their transport, unchanged

report = waddle.corpus.load("towel-folding-pilot").summary()   # SR, MTTI, failure taxonomy

# ── 5. Two weeks later: policy improvement from the same corpus ────────────
ds = waddle.corpus.load("towel-folding-pilot")
ds = waddle.corpus.smooth_handoffs(ds, blend_s=0.4)  # kill the takeover spikes
waddle.optimize.hil_serl(dataset=ds, robot_hooks=waddle.hooks.from_init(),
                         judge_head=ds.judge_head("vlm"))     # live online-RL rig
```

What A exercised: dual command interfaces (the "tap our IK" ask costs them one dict entry), composite bimanual space, on-prem relay for video governance, a custom judge coexisting with the VLM, local tripwires, code-as-policy reset with human fallback, and HIL-SERL on the collected corpus.

### 4.2 Company B — industrial, single xArm, 2× Orbbec, vendor IK, **their own leader arm**

The interesting demand: *"we don't need your teleop layer."* Correct response: don't sell them teleop — sell them detection, orchestration, resets, labeling, and the flywheel *around* their leader arm. Their leader arm becomes an `InterventionSource` plugin; Waddle still decides when intervention is needed, claims the episode, tags provenance, and labels the data. Crucially, the leader arm can also *self-initiate* a claim via its clutch — the operator on the floor grabs the leader, and Waddle records it as an intervention rather than fighting it.

```python
"""Company B: xArm7 + 2x Orbbec, vendor IK, custom leader-arm interventions."""
import numpy as np
from xarm.wrapper import XArmAPI                    # xArm-Python-SDK
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat

import waddle

# ── 1. Hardware as they already run it ──────────────────────────────────────
arm = XArmAPI("192.168.1.221")
arm.motion_enable(True); arm.set_mode(1); arm.set_state(0)     # mode 1: joint servo streaming

def make_orbbec_tap(index: int):
    pipe, cfg = Pipeline(), Config()
    profiles = pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    cfg.enable_stream(profiles.get_video_stream_profile(1280, 720, OBFormat.RGB, 30))
    pipe.start(cfg)
    def tap():
        frames = pipe.wait_for_frames(100)
        c = frames.get_color_frame()
        img = np.frombuffer(c.get_data(), np.uint8).reshape(720, 1280, 3)
        return {"color": img, "t_ns": c.get_timestamp_us() * 1000}
    return tap

# ── 2. Their leader arm as a first-class InterventionSource ─────────────────
class LeaderArm(waddle.plugins.InterventionSource):
    """Kinematically-matched leader; clutch on the handle. Vendor IK untouched."""
    name, spaces = "leader_arm", ["joint_position"]
    def __init__(self, dev="/dev/ttyUSB0"):
        self.leader = factory_leader.connect(dev)          # their hardware, their code
    def engaged(self) -> bool:                             # polled every gate() tick
        return self.leader.clutch_pressed()                # operator grabs it → claim
    def start(self, ctx):
        self.leader.sync_to(ctx.robot_qpos)                # avoid a jump at handoff
    def get_action(self, obs) -> waddle.Action:
        return waddle.Action(joint_position=self.leader.read_qpos())
    def stop(self):
        self.leader.release()

# ── 3. Declarations — note what is ABSENT: no Waddle teleop registered ──────
waddle.init(
    project="cnc-tending", api_key="wd_live_…",
    robot=waddle.Robot(
        name="xarm7-cell-3",
        action_space=waddle.JointSpace(
            joints=[f"j{i}" for i in range(1, 8)], units="rad", rate_hz=100,
            chunking=waddle.Chunking(horizon=16, replan="CHUNK_BOUNDARY", interp="cubic"),
            gripper=waddle.Gripper.parallel(dim=-1, open=850, closed=0)),  # xArm gripper units, mapped
        kinematics="xarm7.urdf",
    ),
    control=waddle.Control(
        send={"joint_position": lambda ch: [arm.set_servo_angle_j(s.q, is_radian=True)
                                            for s in ch.steps]},
        hold=lambda: arm.set_state(3),                     # xArm pause state
        resume=lambda: arm.set_state(0),
        home=lambda: arm.move_gohome(wait=True),
        handoff=waddle.Handoff.HOLD_FIRST(),               # industrial: freeze, then human
    ),
    cameras={"cell": waddle.Camera(tap=make_orbbec_tap(0)),
             "gripper": waddle.Camera(tap=make_orbbec_tap(1))},
    interventions=[LeaderArm()],                           # ← their plugin; Waddle teleop opted out
    resets=[waddle.resets.Scripted(lambda ctl, ctx: run_vendor_reset_motion(arm)),
            waddle.resets.CodeAsPolicy(scene="machine_tending")],  # agent when script isn't enough
    judges=[waddle.judges.VLM(rubric="part seated in chuck; door closed; no part on table")],
)

# ── 4. Rollouts. Detection is Waddle's; hands are theirs. ──────────────────
with waddle.rollout(task="load blank into chuck") as ep:
    while not ep.done:
        obs = {"qpos": np.array(arm.get_servo_angle(is_radian=True)[1])}
        a   = ep.gate(vla_client.infer(obs), obs)
        # If Waddle's supervisor flags a stall, it pages the floor operator (webhook)
        # and the episode enters HOLD; the operator grabs the leader arm, clutch
        # engages, gate() streams leader actions — tagged provenance="custom:leader_arm".
        arm.set_servo_angle_j(a.q, is_radian=True)

# ── 5. Their payoff: corrections become training data with zero labeling ────
ds = waddle.corpus.load("cnc-tending")
ds = waddle.corpus.smooth_handoffs(ds, blend_s=0.3)
waddle.optimize.filtered_bc(ds, base="their-pi0-finetune",
                            weight="intervention")         # IWR-style upweighting
```

What B exercised: full teleop opt-out with a ~20-line plugin, operator-initiated claims (clutch), `HOLD_FIRST` handoff for an industrial safety culture, vendor motions as a `Scripted` reset with an agent behind it, webhook paging instead of a remote operator network, and the labeling/flywheel value prop standing entirely on its own.

### 4.3 Company C — custom OS, custom RGB cameras, custom USB protocol

The stress test for P1: *nothing* in their stack is standard. That's fine, because Waddle never asked for standard plumbing — it asked for standard *declarations*. They know their joint layout (so the action space is `JointPosition`, not `Opaque`), they can produce a URDF (so Waddle's closed retargeting/IK service can drive teleop even though Waddle has never seen this robot), and their custom USB transport hides entirely inside the five verbs.

```python
"""Company C: custom OS layer, custom RGB cameras, custom USB wire protocol."""
import numpy as np
import waddle
from ourstack import bus, camd                      # their proprietary layer

# ── 1. Their transport, wrapped in the five verbs. Waddle never sees USB. ───
dev = bus.open("/dev/ourbot0")                      # custom framed-packet protocol

def send(chunk: waddle.ActionChunk):
    for step in chunk.steps:
        dev.write(bus.pack_joint_target(step.q, step.t_ns))   # their framing

control = waddle.Control(
    send={"joint_position": send},
    hold=lambda: dev.write(bus.HOLD),
    resume=lambda: dev.write(bus.RESUME),
    home=lambda: dev.write(bus.pack_joint_target(HOME_Q)),
    estop=waddle.EStop(fn=lambda: dev.write(bus.ESTOP), hardware=False, latency_ms=15),
)

# ── 2. Custom cameras: a tap is just a callable returning pixels + a clock ──
def cam_tap(cam_id: int):
    stream = camd.subscribe(cam_id)                 # their camera daemon
    def tap():
        frame = stream.next()                       # RGB only, no depth, no intrinsics
        return {"color": frame.rgb, "t_ns": frame.mono_ns}
    return tap

# ── 3. Declarations. The URDF is the key that unlocks Waddle-side teleop. ───
waddle.init(
    project="ourbot-alpha", api_key="wd_live_…",
    robot=waddle.Robot(
        name="ourbot-proto-4",
        action_space=waddle.JointSpace(joints=[f"a{i}" for i in range(6)],
                                       units="rad", rate_hz=30,
                                       chunking=waddle.Chunking(horizon=8, replan="IMMEDIATE",
                                                                interp="linear")),
        kinematics=open("ourbot.urdf", "rb").read(),   # → Waddle IK + retargeting, closed side
    ),
    control=control,
    cameras={"front": waddle.Camera(tap=cam_tap(0)),   # uncalibrated RGB: monitoring + VLM
             "wrist": waddle.Camera(tap=cam_tap(1))},  #   judging work; 3D overlays don't (yet)
    interventions=[waddle.interventions.Teleop(space="ee_delta"),   # Waddle IK: ee_delta→their joints
                   waddle.interventions.CodeAsPolicy()],
    resets=[waddle.resets.Scripted(lambda ctl, ctx: our_reset_routine(dev)),   # their routine, slot-in
            waddle.resets.Teleop()],
    judges=[waddle.judges.VLM(rubric="all three blocks inside the tray")],
)

# `waddle doctor` matters most for exactly this customer:
#   $ waddle doctor
#   ✓ joint round-trip (send→state echo)        ✓ hold() latency: 21 ms
#   ✓ URDF joints match declaration (6/6)       ✗ clock skew cam0 ↔ control: 47 ms  → fix PTP
#   grant report: monitor ✓ pause ✓ takeover(ee_delta via waddle-ik) ✓ reset(scripted+teleop) ✓

# ── 4. Rollout loop, same six lines as everyone else ────────────────────────
with waddle.rollout(task="clear the tray") as ep:
    while not ep.done:
        obs = {"qpos": dev.read_state().q}
        a   = ep.gate(policy(obs), obs)
        dev.write(bus.pack_joint_target(a.q))

# ── 5. They train in their own stack — export, don't capture ────────────────
waddle.corpus.load("ourbot-alpha").to_lerobot("exports/ourbot_alpha")   # or raw MCAP
```

What C exercised: fully opaque transport behind the five verbs, tap-based cameras with no vendor assumptions, teleop unlocked purely by a URDF (Waddle-side IK — the closed retargeting service earning its keep), a custom scripted reset slotted ahead of teleop fallback, `waddle doctor` as the onboarding path for weird stacks, and clean export into their own training pipeline. Note what they *didn't* get without calibration: no 3D overlays, no geometric tripwires — the lattice, not a cliff.

### 4.4 Company D — SO-101s + stock LeRobot, two RGB cameras

The beginner case must be near-zero effort, so it gets the adapter treatment: Waddle already knows the SO-101 (URDF, motor layout, gripper) and already knows LeRobot's `Robot` interface, so `waddle.lerobot.wrap()` builds the descriptors, taps the cameras from the LeRobot config, and gates `send_action` — one line around their existing object.

```python
"""Company D: SO-101 follower + stock LeRobot + SmolVLA, minimal footprint."""
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

import waddle

# ── 1. Standard LeRobot setup, straight from the docs ──────────────────────
cfg = SO101FollowerConfig(
    port="/dev/ttyACM0", id="follower_1",
    cameras={"wrist": OpenCVCameraConfig(index_or_path=0, fps=30, width=640, height=480),
             "scene": OpenCVCameraConfig(index_or_path=2, fps=30, width=640, height=480)},
)
robot = SO101Follower(cfg); robot.connect()
policy = SmolVLAPolicy.from_pretrained("lerobot/smolvla_base")

# ── 2. One line. The adapter reads cfg: cameras, motors, gripper, URDF. ─────
waddle.init(project="so101-first-evals", api_key="wd_live_…")
robot = waddle.lerobot.wrap(robot,
    judges=[waddle.judges.VLM(rubric="red cube inside the white bowl")])
# defaults filled in: interventions=[Teleop(so101 profile), CodeAsPolicy()],
# resets=[CodeAsPolicy(scene="tabletop")], tripwires from SO-101 joint limits.

# ── 3. Their loop is character-for-character the LeRobot tutorial loop ──────
for i in range(50):
    with waddle.rollout(task="put the red cube in the bowl") as ep:
        while not ep.done:
            obs    = robot.get_observation()        # wrapped: frames auto-tapped + logged
            action = policy.select_action(obs)
            robot.send_action(action)               # wrapped: gate() runs inside
print(waddle.corpus.load("so101-first-evals").summary())
#  50 episodes · SR 62% → 71% after interventions · 9 teleop claims · 50 auto-resets

# ── 4. First taste of the flywheel, three lines ─────────────────────────────
ds = waddle.corpus.smooth_handoffs(waddle.corpus.load("so101-first-evals"), blend_s=0.3)
waddle.optimize.filtered_bc(ds, base="lerobot/smolvla_base",
                            out="hf://them/smolvla_so101_v2")
```

And for LeRobot users on **async inference** (policy on a GPU box, robot client on a laptop), the zero-code path — no Python at all:

```bash
# GPU box (unchanged):     lerobot policy server on :8080
# Laptop, was:             robot_client → gpu-box:8080
$ waddle proxy --upstream grpc://gpu-box:8080 --listen :9000 \
               --config waddle.yaml            # project, task, judges, api key
# Laptop, now:             robot_client → laptop:9000
```

The proxy sees every observation and action chunk, so it is *already in the write path*: supervision, takeover (substitute the chunk), episode segmentation, and labeling all work with a one-URL change. The same binary speaks the openpi `WebsocketClientPolicy` protocol for that ecosystem.

What D exercised: adapter-inferred declarations (they authored zero descriptors), wrapped-object gating (their tutorial loop unmodified), all-default modules, the proxy as a no-code alternative, and a three-line path from "first evals" to "fine-tuned checkpoint."

---
## 5. Adversarial Stress Test — First Pass (v0.1 design)

An honest attempt to break the design, attack by attack, with verdicts.

### 5.1 Attacks on the abstraction

**"The gate assumes a user-owned Python loop. Much of robotics doesn't have one."** True and important. ROS control graphs, event-driven executors, and vendor runtimes have no loop to put `ep.gate()` in. The mitigation is that the *protocol* — not the `with` block — is the product: the ROS priority-mux node and the proxy implement identical claim/handoff semantics with no user loop at all. But this means maintaining two integration idioms forever, and the docs must route people correctly or the first impression is "this doesn't fit my architecture." Verdict: survivable, but the mux and proxy cannot be second-class citizens; they need the same conformance fixtures and the same launch-day polish as the Python SDK.

**"A closed ActionSpace enum will meet a robot it can't describe."** Guaranteed. Dexterous hands, soft robots, tendon-driven systems, multi-rate composites (100 Hz arm + 10 Hz base), torque/impedance control. The `Opaque` tier catches them for monitoring, but there's a real risk Opaque becomes a ghetto where half the market lives with half the product. Two mitigations: (a) `Composite` with per-part rates covers more than it first appears; (b) feature-flag versioning lets new canonical types ship without breaking pinned SDKs. But the strategic cost of adding a type is real — every canonical type is a promise that Waddle teleop and reset agents can *write* into it. Verdict: accept the constraint knowingly; the enum's narrowness is the price of the M-not-N×M retargeting economics, and the roadmap should sequence new types by revenue, not elegance.

**"Waddle sits at policy rate, so it can't help at servo rate."** Correct: `gate()` at 30–100 Hz is fine; a 500 Hz–1 kHz torque loop is not a place for a Python SDK, and Waddle should say so explicitly rather than discover it in a customer's incident report. Impedance-controlled contact-rich tasks where the failure *is* at servo rate (a bad force spike inside one chunk) will be caught only by local tripwires or after the fact. Verdict: declared scope limit. The relay's C++ tripwire evaluator can later move down the stack; the SDK never should.

**"Resets are the weakest promise."** The hardest attack, because it targets the value prop rather than the API. Code-as-policy scene reset on *arbitrary* scenes is unsolved — your in-house results are promising, but "we reset anything" is not yet true, and a failed reset silently corrupts every downstream eval statistic (episodes start from invalid states and the SR numbers lie). Mitigations built into the design: reset strategies are an ordered list with teleop and scripted fallbacks; the reset pipeline ends with a *reset verification* judgment (same VLM machinery) before `waddle.rollout()` yields; and reset failures are surfaced as first-class events, not swallowed. Verdict: the architecture is honest about it, but sales must be too — sell "unattended for tabletop families we've certified, human-fallback elsewhere," and let the certified-scene list grow.

### 5.2 Attacks on the runtime

**"Teleop actions through the gate means teleop at the mercy of their loop."** If the customer's loop blocks (policy inference hiccup, camera stall), intervention actions stop flowing to the robot — during the exact failure that prompted the intervention. Mitigation: on claim, the SDK may *bypass* the gate and drive `control.send` directly from its own thread (the gate then returns `Action.NOOP` markers so the loop stays coherent). This is a mode switch with subtle consequences — the customer's loop is now a spectator writing no-ops — and it must be specified precisely in the handoff contract, including what `send` implementations must do about interleaving. Verdict: solvable, but this is the hairiest part of the protocol; specify and conformance-test it before any adapter code exists (the golden-fixture suite should include a "claimed while loop stalled" scenario).

**"Chunk handoff will cause visible jerks or worse."** Blending between a policy chunk and a human stream in joint space can transit configurations neither author intended; `IMMEDIATE` with a bad `blend_ms` on a fast arm is a safety event. Mitigation: `HOLD_FIRST` as the conservative default for `rate_hz > 50` or payloads above a threshold; blending only where the declared interpolation rule is defined and joint-limit margins are tripwired. Verdict: contained, but the handoff contract is where a demo either impresses or scares a customer — invest in it disproportionately.

**"The cloud e-stop is a liability bomb."** If any safety action routes through WAN, a partition during a bad grasp becomes a courtroom exhibit. The design already splits this (local tripwires + heartbeat watchdog trigger customer-provided `hold()` with zero network), but the *marketing* must never imply Waddle is a safety system — it is a supervision system that *uses* the customer's safety interfaces. Verdict: fine technically; the contract language and the `EStop(hardware=..., latency_ms=...)` declaration exist precisely to keep responsibilities legible. ISO 10218/15066 compliance stays the customer's.

**"Clock skew will silently poison the data."** Three RealSenses, a control PC, and a teleop stream on different clocks: intervention boundaries land tens of ms off, and the flywheel trains on misaligned (obs, action) pairs. Verdict: real and boring, which is why `waddle doctor` measures skew at onboarding and the session handshake establishes clock offsets; per-message source clocks plus SDK-side mapping into one monotonic timeline is non-negotiable in the schema (already pinned: t_ns from a synced monotonic clock, verified per-session).

**"Depth and multi-camera bandwidth will melt the uplink."** 3× 848×480 RGBD at 30 fps is >1 Gbps raw. Verdict: already handled structurally — `StreamPolicy` separates local-full-rate MCAP from downsampled H.264 uplink, and the relay terminates media on-LAN; teleoperators get the live stream, the cloud gets what judging needs, the archive keeps everything for post-hoc labeling.

### 5.3 Attacks on the business design

**"Open protocol + open SDK = a competitor implements your control plane."** The Sentry problem: someone ships a backend speaking the Waddle protocol. Verdict: accept it — the moat was never the schema. It's (a) the retargeting/IK library that grows with every embodiment onboarded, (b) the judge and detection models trained on the intervention corpus, (c) the teleoperator network and its ops tooling, and (d) the certified-scene reset library. All four compound with usage; a protocol-compatible competitor starts at zero on each. The open protocol is what makes the SDK adoptable enough to feed them.

**"Company B pays you nothing for the hardest part."** B opted out of teleop — the expensive human network — and uses detection, orchestration, resets, and labeling. Is that a good customer? Verdict: yes, deliberately. B's leader-arm corrections still flow through Waddle's labeling and flywheel, which is the data business; and B is one `interventions=[...]` list edit from adding Waddle teleop for night shifts. The modularity that lets B opt out is the same modularity that makes expansion frictionless.

**"The SDK-in-a-robot-image long tail will strangle protocol evolution."** A customer pins waddle-sdk 0.4 in a robot image for 18 months. Verdict: this is why versioning is feature flags negotiated per-connection rather than semver cliffs — but it constrains the *backend* forever (every shipped feature must be served indefinitely or explicitly sunset). Budget for it culturally: the protocol team says no a lot.

### 5.4 Benefits and drawbacks, summarized

**Benefits.** One protocol serves four wildly different stacks with the same six-line loop (§4's matrix); the grant lattice converts partial integrations into partial revenue instead of failed sales; the closed enum makes teleop and resets embodiment-portable at cost O(action-space types), which is the core scaling economics; interventions are provenance-tagged at write time, making the data flywheel a side effect of ops rather than a separate product; the open/closed seam falls exactly on the plugin ABCs and RPC boundary, so "open enough to build on, closed enough to sell" is a line in the architecture rather than a license negotiation; and MCAP/LeRobotDataset/Rerun outputs mean Waddle composes with the ecosystem instead of fighting Foxglove and LeRobot for territory they already won.

**Drawbacks.** Two integration idioms (gate vs. mux/proxy) must be maintained at equal quality forever; the ActionSpace enum will always lag the frontier of embodiments, and each addition is an expensive promise; reset generality — the headline value prop — is the least mature technology in the system and needs honest scoping; the claimed-while-stalled and mid-chunk-handoff corners of the runtime contract are genuinely hard to specify and will consume disproportionate engineering; Python-rate operation excludes servo-rate failure modes by construction; and the relay container (needed for latency, bandwidth, and data governance) drags a closed binary into customer infrastructure, with all the operational support burden that implies.

**Open questions worth resolving before writing adapter code.** (1) The exact `send()` interleaving contract when the SDK bypasses a stalled gate. (2) Whether reset verification blocks `rollout()` entry on judge latency (seconds) or optimistically yields with an async invalidation path. (3) Whether Company-B-style local sources should be allowed to *deny* Waddle-initiated claims (union rules, safety culture) — i.e., is the priority list advisory or binding. (4) Minimum viable canonical-type set at launch: is `BaseTwist` in or out for v1.


---

## 6. Adversarial Stress Test — Final Pass (v0.6)

A second red-team, run against the design *as it now stands* — the component layout (§3), the unified vocabulary, the recording modes, the codec architecture, and the retake semantics all postdate the first pass and deserve their own attacks. Where an attack found an actual defect rather than a tradeoff, the fix is recorded in §6.6 as a normative amendment.

### 6.1 Attacks on the unified vocabulary

**"You resolved the capability collision and created a grant collision."** True. The glossary defines **grant** as a permission the integrator extends to Waddle; but the wire-frozen work plane uses `work_grant` for a different act — the lobby *granting a claim* to a worker. So "grant" now carries a noun sense (declared permission) and a verb sense (awarding a claim), both live in the same codebase. Verdict: livable but must be pinned before it spreads — the noun sense is canonical; `work_grant` is read as the verb ("to grant a claim") and is explicitly *not* an instance of the glossary's Grant. If ambiguity ever bites in practice, the escape hatch is renaming the protocol message (`Grant` → `StandingGrant`) rather than touching wire-visible RPCs. Recorded as amendment N1.

**"Retake is a statistics loophole."** The sharpest new attack. Retake — terminate the episode, open a new one under the held claim — was adopted from production because operators genuinely need it. But it creates two integrity holes. First, *outcome accounting*: if retaken episodes silently vanish, success rate is biased upward (the worst attempts get laundered into retakes); if they count as failures, an operator's judgment call moves the customer's headline metric. Second, *reset validity*: the new episode's "reset" was performed ad hoc by the intervenor mid-claim, bypassing the reset-verification judgment that `waddle.rollout()` otherwise guarantees — quietly weakening the very invariant the first pass identified as protecting all downstream eval statistics. Verdict: real defect. Fix (amendment N2): a retaken episode closes with the distinct terminal outcome `aborted_retake`, reported as its own line in every summary (never folded into SR's denominator silently — the Corpus reports SR both including and excluding retakes); and the successor episode must still pass reset verification before entering RUNNING, or be permanently flagged `reset_unverified` in its sidecar.

**"The glossary has near-synonyms, and it's frozen 'forever' pre-product-market-fit."** *Supervised rollout* and *episode* are one concept viewed from two heights, and freezing vocabulary before a single external customer has used it risks enshrining words that turn out to confuse. Verdict: mostly accepted — the freeze is what makes internal/external unification worth anything, and v0 explicitly permits additive evolution via feature flags. But trim now while it's free: *supervised rollout* is demoted to prose (a description of an episode run with supervision), not a defended glossary entry.

### 6.2 Attacks on the component layout

**"The conformance suite tests logic, not physics."** §3 leans hard on "every artifact's CI runs the same conformance suite — that, not shared code, guarantees identical behavior." But wire fixtures and behavioral scripts verify *transition correctness*, not timing: a ROS mux under a congested executor, a PyO3 tap fighting a customer's GIL, a jitter buffer on a lossy WAN can all pass fixtures and still behave differently where it matters (blend windows, deadman cutoffs, hold latency). Verdict: fixtures are necessary, not sufficient. Amendment N3: the conformance story gets a third tier — soak/latency benches (hardware-in-loop where feasible) with published timing envelopes per frontend — and the docs stop implying fixtures alone guarantee equivalence. `waddle doctor` on the actual rig remains the only conformance statement that binds a specific deployment.

**"Codecs inside waddle-core chain your release cadence to LeRobot's."** The §3 sign-off note put `waddle-codecs` in the core workspace so relay and SDK proxy-modes reuse dialects for free. The cost surfaces now: a LeRobot schema bump forces a waddle-core release, which forces SDK wheel rebuilds, which pinned robot images won't take — the exact long-tail §5 warned about, now on someone else's schedule. Verdict: defect in packaging, not concept. Amendment N4: `waddle-codecs` moves out of the core's semver — an independently versioned crate (and, for the proxy, loadable at startup) so dialect churn ships without touching `waddle-core`. The reuse argument survives; only the coupling dies.

**"You're freezing a C ABI before it has two consumers."** The FFI's "internal refactors never move it" promise, combined with proxy-first sequencing, means the ABI would be designed when only the proxy (which doesn't use it) exists. Datadog's lesson was that the FFI boundary is where the bodies are buried; freezing it from imagination repeats the greenfield-guessing mistake §5.3 flagged. Verdict: process fix, not design fix. Amendment N5: the C ABI is explicitly *unstable* until both `waddle-sdk` and `waddle_ros` consume it in anger; stability is declared afterward, as an event, not assumed from birth.

**"The relay is your IP's weakest point, and 'open chassis' is a trust claim, not a mechanism."** §3.7 ships judge/detector weights to customer infrastructure encrypted and license-gated — but edge-deployed weights are extractable by a sufficiently motivated actor with root on their own hardware; that's a DRM problem, and DRM loses. Meanwhile "the open chassis behavior is covered by the public conformance suite" is unverifiable for a closed binary — customers must take Waddle's word that the shipped relay *is* the audited chassis. Verdict: two accepted risks to manage, not eliminate. Crown-jewel models stay cloud-side; the relay hosts *distilled, deployment-scoped* models whose extraction loses Waddle a component, not the moat (the moat argument of §5.3 — compounding corpus, ops network, retargeting library — already doesn't rest on any single checkpoint). For the trust gap: publish reproducible-build hashes of the chassis portion where feasible, and accept that enterprise security review, not open source, is the assurance mechanism for the rest.

### 6.3 Attacks on the runtime guarantees as now specified

**"Grants are static claims about dynamic properties."** `hold()` measured at 21 ms by `waddle doctor` on a quiet Tuesday is a promise the robot host's load average will eventually break — and the backend *plans interventions against declared grants*, so a stale grant is a planning input that's silently false. Verdict: defect. Amendment N6: grants are validated continuously, not only at init — the heartbeat carries recent measured verb latencies, and the control plane demotes a grant (with an operator-visible event) when observed behavior violates its declaration. The grant lattice becomes a *live* lattice.

**"The SDK-tier lease is a polite fiction."** The glossary honestly assigns lease enforcement to the hardware owner — and in a Tier-1 Python integration, the "owner" is a convention: nothing physically stops the customer's loop from calling their own transport during bypass mode, producing the dual-writer scenario the lease exists to prevent. The broker enforces single-writer for real; a `Control(send=callable)` integration cannot. Verdict: the design is honest but under-explicit. Amendment N7: grant negotiation records the **lease enforcement point** per integration — *enforced* (broker, ROS mux with exclusive topic ownership, proxy owning the only socket) vs *advisory* (in-process callables) — the backend's intervention planner treats advisory-lease integrations more conservatively (prefer `HOLD_FIRST`), and `waddle doctor` gains a NOOP-compliance test that verifies the customer's loop actually stands down during a simulated bypass.

**"The recording modes and the flywheel contradict each other."** `waddle.optimize.filtered_bc` needs (observation, action) pairs; in `SidecarOnly` mode no observations persist anywhere, and in `Reference` mode they persist somewhere Waddle can't reach without a resolver. A customer sold "the flywheel" who chose `SidecarOnly` bought a toolkit that cannot run. Verdict: tension is real, resolution is documentation plus one guard. Amendment N8: `waddle.corpus` transforms and `waddle.optimize` recipes declare their data requirements and fail at load time with a mode-specific message ("this project records SidecarOnly; filtered_bc requires Local or Reference-with-resolver"); sales collateral maps modes to product tiers explicitly — SidecarOnly is the metrics/ops tier, not the data tier.

**"Monitor-only customers create liability without authority."** The grant lattice's proudest feature — revenue from observe-only integrations — means Waddle will sometimes *watch a failure it has no grant to prevent*, and detection itself creates exposure: "your supervisor saw the collision developing and the page arrived late." Observability vendors carry missed-alert risk; Waddle carries it about physical events. Verdict: accepted risk requiring contract language, not architecture — detection SLOs are explicitly best-effort with stated latency envelopes, alerts are advisory, and the envelope/tripwire vocabulary (Waddle requests, the owner enforces) exists precisely to keep this legible in a courtroom. The marketing rule from §5.2 hardens into policy: Waddle is never described as a safety system, *including in monitor-only mode*.

### 6.4 Attacks on the data product and the business

**"The flywheel is circular: the judge labels the data that trains the policy that the judge then evaluates."** If `filtered_bc` upweights what the VLM judge scores well, and the customer's headline SR is computed by the same judge, the system optimizes the judge, not the task — Goodhart with a robot attached. Worse, judge drift (a model update) silently moves every customer's historical metrics. Verdict: the most important data-integrity risk in the product. Mitigations, adopted as amendment N9: every project maintains a held-out, human-labeled audit slice (teleoperators double as labelers — the ops network is already paying for the humans); judge versions are pinned per project with explicit re-baselining events, never silent upgrades; and judge/human disagreement rate is itself a first-class Corpus metric surfaced to the customer. The audit slice is also the honest answer to "why trust your SR numbers" in enterprise sales.

**"The sidecar has no fleet identity."** Sessions imply one robot; nothing in §2.6's record pins *which* robot in a multi-cell or multi-arm-per-process deployment produced an episode, and "SR across the fleet" is the first dashboard any real customer asks for. Verdict: small defect, cheap now, expensive later. Amendment N10: the sidecar carries `robot_id` (and `cell_id` where applicable) as first-class fields from v0 — the internal schema already keys on `cell_id`, so this is unification, not invention.

**"You're planning a package rename during a live pilot."** Appendix A.5's sequencing is correct in the abstract and risky in the particular: the `waddle` → `waddle_cell` flip touches every deploy script, tunnel unit, and runbook on cell hosts — exactly the machinery a running customer pilot depends on. Verdict: add one sequencing constraint — the rename lands only in a window with no active external pilot (concretely: not during the final week of a pilot whose extension decision is pending), and the A.5 automation inventory is completed *before* the window is scheduled, not during it.

### 6.5 The meta-attack: this is a platform company's architecture and you are a startup

Step back and count what v0.6 specifies: a protocol with a conformance program, a ten-crate Rust core, three language frontends plus a ROS runtime target, two shipping binaries (one an appliance), three recording modes, a plugin system with three slots, a codec ecosystem with a version treadmill, a post-training toolkit, and a rename of the existing codebase. Every piece is individually justified, and together they describe roughly two years of platform engineering standing between today and the first dollar of SDK revenue — the classic failure mode of infrastructure-brained founders. The design's own best defense is the sequencing principle it already contains (proxy-first, ABI-unstable-until-consumed, certified-scene honesty), but the doc has never stated a v1 cut, so here is the proposed one, adversarially minimal:

**v1 ships:** `waddle-protocol` v0 (schemas + glossary + fixtures, no conformance *program*), `waddle-core` sufficient for the proxy, **`waddle-proxy` with exactly two codecs** (LeRobot-async, openpi), `waddle-sdk` Tier-1/Tier-2 for Python only, action spaces `JointPosition` + `EEDelta` + `Composite` + `Opaque`, `Local` recording only, Waddle teleop + `Scripted` resets + the VLM judge, `waddle doctor`, and `waddle.corpus` with export + `smooth_handoffs`. **v1 explicitly defers:** `waddle-cpp`, `waddle_ros`, the relay as a customer-installable product (it runs only in Waddle-operated deployments, where it's the productizing bridge), `Reference`/`SidecarOnly` modes, `BaseTwist`/velocity spaces, the codec plugin API, `hil_serl`/`rlpd` as products, and the CLI plugin merge. Every deferral has a named trigger (a paying customer who needs it), which converts this section from a wish-list haircut into a decision procedure.

### 6.6 Normative amendments adopted from this pass

*Status: all amendments below are applied in the body as of v0.8 — the sections listed under "Touches" now contain the normative text, marked inline with *(N#)* tags. This table remains as the audit record of what the pass changed and why.*

| # | Amendment | Touches |
|---|---|---|
| N1 | "Grant" noun sense is canonical; `work_grant` is the verb, not an instance; escape hatch is renaming the message, never the wire RPC | §2.8, A.3 |
| N2 | Retake closes with `aborted_retake` (always reported; SR shown with and without); successor episode requires reset verification or a permanent `reset_unverified` flag | §2.4, §2.6, `episode.proto` |
| N3 | Conformance gains a timing/soak tier with published per-frontend envelopes; fixtures no longer claimed sufficient | §3.1, §2.7 |
| N4 | `waddle-codecs` versioned independently of `waddle-core`; proxy loads dialects without a core release | §3.2, §3.6 |
| N5 | C ABI declared unstable until consumed by both `waddle-sdk` and `waddle_ros` | §3.2 |
| N6 | Grants validated continuously via heartbeat-carried verb latencies; runtime demotion with operator-visible events | §2.3, `services.proto` |
| N7 | Lease enforcement point (enforced vs advisory) recorded at grant negotiation; planner conservatism and a doctor NOOP-compliance test for advisory integrations | §2.3, §2.4 |
| N8 | Corpus transforms/optimizers declare data requirements and fail loudly per recording mode; modes mapped to product tiers | §2.6 |
| N9 | Held-out human-labeled audit slice per project; pinned judge versions with explicit re-baselining; disagreement rate as a Corpus metric | §2.6, control plane |
| N10 | `robot_id`/`cell_id` first-class in the sidecar from v0 | `sidecar.proto` |
| — | v1 cut list of §6.5 adopted as the sequencing baseline; deferrals require a named customer trigger to un-defer | roadmap |

The overall verdict after two passes: the architecture survives its own red team, but only *with* the amendments — the recurring theme of this pass is that v0.6's elegance kept making static promises (grants, leases, conformance, vocabulary) about dynamic realities (load, customers, upstream churn, operators under pressure), and the fixes all have the same shape: measure at runtime what was previously declared at init, and report honestly what was previously implied.

---

## 7. Adversarial Stress Test — Third Pass (v0.8)

The previous pass attacked the design; this one attacks the *fixes*. Amendments are code too — they can be unimplementable, interact badly, or shift risk instead of removing it. This pass also examines the one artifact no prior pass could: the amendment process itself. New amendments are numbered N11+ and are **proposed, not yet applied**.

### 7.1 Second-order attacks on the amendments

**"N6's live grants are partially unimplementable as written."** The amendment says the heartbeat carries "recently measured verb latencies" — but you cannot continuously measure the latency of a verb you dare not call. `hold()` on a robot mid-task is not a probe, it's an incident; there is no such thing as routinely invoking it to see how fast it is. Doctor measures verbs by actually calling them in a controlled window; a heartbeat cannot. Verdict: real defect in the fix. Amendment **N11**: the heartbeat carries *proxy signals* that are safely measurable at runtime — control-plane RTT, gate-tick jitter (a direct read on interpreter/executor health for advisory integrations), host load, callback dispatch time — and grant health is *inferred* from proxies with hysteresis; actual verbs are re-measured only in safe windows (between episodes, during resets, doctor re-runs). Two demotion mechanics follow: demotion never interrupts an active lease (it takes effect at the next planning decision — revoking a teleoperator's takeover grant mid-motion because the host got busy, possibly busy *because of the intervention's own video encoding*, would be the cure killing the patient), and demote/re-promote transitions carry hysteresis bands so a latency hovering at the bound cannot flap the planner.

**"N2's retake verification reintroduces the open question it was meant to close."** The first pass left open whether reset verification blocks `rollout()` entry on judge latency; N2 quietly reopened it for retakes with an unresolved "or": verification *before RUNNING*, or a permanent flag — who picks the branch, and when? If verification blocks, the operator who just hand-reset the scene stands idle for a judge round-trip, which teaches operators to hate the system; if it's async, the flag arrives after data was already written. Verdict: specify the branch per initiator. Amendment **N12**: operator-initiated retakes get *optimistic entry with async invalidation* — the judge scores the reset from the already-live media stream during the operator's final settle moments, and a late failure marks the episode `reset_unverified` retroactively (operator flow is sacred; the flag exists precisely so optimism is honest). Autonomous resets get *blocking* verification, because no human is waiting and episode validity is the whole point. This also closes first-pass open question (2) with the same rule.

**"N9's audit slice is biased, conflicted, and impossible in two of three recording modes."** Three separate holes. *Independence:* teleoperators double as labelers, but teleoperators are the intervenors — a human labeling an episode they intervened in is grading their own necessity. *Sampling:* the only bulk data that persists in Reference/SidecarOnly modes without a resolver is incident clips, which are event-triggered — i.e., precisely a biased sample; an audit slice built from judge-flagged moments cannot detect what the judge systematically misses. *Existence:* a SidecarOnly project retains no video at all, so its judge-produced SR is structurally unauditable — and the doc currently sells that tier metrics-first. Verdict: the most important catch of this pass. Amendment **N13**: audit labels must come from a labeler who was not the episode's intervenor; audit sampling is a *random retention quota* (a contractual opt-in for Reference/SidecarOnly projects: N% of episodes retained at random for audit, independent of events); projects that decline the quota have their judge metrics permanently marked **unaudited** in every Corpus surface; and slice size is tied to a stated confidence target rather than vibes — with real-world eval already suffering from N<25-per-condition statistics, an audit slice that cannot detect judge drift is theater.

**"N7's NOOP-compliance test commits the sin the pass diagnosed: a doctor-time test of a runtime property."** A loop that stands down in a simulated bypass can still double-write during a real panic — exception handlers, watchdog restarts, and human floor improvisation are different code paths from the rehearsed one. Verdict: the test is evidence, not enforcement, and advisory means advisory. Amendment **N14**: add *runtime* dual-write detection — during any bypass on an advisory-lease integration, the intervenor knows exactly what it commanded, so sustained divergence between commanded trajectory and proprioception means either the envelope clamped it or someone else is writing; on detection, freeze via `hold()`, alert, and record the event with the divergence trace. This converts the dual-writer scenario from silent corruption into a loud, diagnosable incident, which is the most an advisory lease can honestly offer.

**"N4's independently-shipping codecs opened a supply-chain hole in the action write path."** A codec loadable by the proxy at startup, on its own release cadence, is a plugin sitting between a policy server and a robot — the highest-consequence position in the system for a malicious or merely buggy artifact. Verdict: fixable with boring rigor. Amendment **N15**: codecs are signed and their versions pinned in `waddle.yaml` (no floating "latest" in the write path); load-time certification is mandatory in addition to the existing per-session round-trip check; and the codec *trait* gets its own stability declaration — N5 deferred C-ABI stability, but the trait is a de facto ABI for out-of-tree codecs and cannot inherit that deferral once third parties write against it.

**"N3's published timing envelopes contradict your own liability posture."** §6.3 hardened "detection is best-effort, never a safety system" into policy; N3 then proposes publishing per-frontend numbers like hold-latency envelopes — exactly the kind of specific figure that reads as a warranty in a deposition. Verdict: tension, not contradiction, resolvable by framing. Amendment **N16**: envelopes are published as *observed bench measurements* under stated conditions with explicit non-warranty language; safety-adjacent numbers (`hold`, `estop`) are additionally reported only per-deployment by `waddle doctor` — the binding number is always the one measured on the customer's rig, never the one in the brochure.

### 7.2 Interaction and residue attacks

**"The vocabulary unification missed 'operator.'"** N6 says demotion produces an "operator-visible event" — which operator? The doc uses the word for Waddle's work-plane teleoperators, Company B's floor staff, and the customer's ops team, three different humans with different consoles and different authority. Verdict: small, cheap, worth fixing before it fossilizes. Amendment **N17**: *teleoperator* = Waddle work-plane human; *site operator* = customer-side human at the cell; unqualified "operator" is banned in normative text.

**"Episodes can now be born claimed, and the metrics don't know it."** A retake successor starts life under an active claim, which quietly breaks metric definitions written for clean episodes: intervention-rate-per-episode counts it as instantly intervened, MTTI is undefined (time-to-intervention from an intervened start), and autonomy-duration stats get a zero-length head. Separately, the session model assumes one active episode — two independent tasks in one process (a two-station workcell, one SDK session) is currently unrepresentable, and `robot_id` (N10) made the gap visible without filling it. Verdict: scope explicitly rather than discover in a customer dashboard. Amendment **N18**: Corpus metric definitions handle born-claimed episodes as their own class (excluded from MTTI, counted in a `retake_continuation` rate); v0 formally scopes **one active episode per session**, with concurrent episodes a named-trigger deferral (first customer running parallel stations in one process).

### 7.3 The cut list, re-examined: it trimmed the cheap half

§6.5's v1 cut was adversarially minimal about the *open* column — and silent about the closed one. "Waddle teleop" appears as one line in v1, but that line contains the retargeting/IK service, the media plane, the operator console, and the ops network; shipping `EEDelta` as a v1 action space commits the hosted IK path by the back door (Company C's whole integration story leans on it). The two-year-platform risk the meta-attack worried about lives mostly in the closed column, which received no haircut. Verdict: the cut list needs a twin. Required companion artifact (roadmap, not numbered): a **closed-side v1 cut** written with the same named-trigger discipline — a plausible shape being: v1 teleop = the existing terminal + one hosted IK path (Pinocchio-based) covering `JointPosition` mirror and single-arm `EEDelta` for embodiments with customer-supplied URDFs, generic multi-embodiment retargeting deferred; v1 judging = one VLM judge configuration + the audit-slice machinery (N13 makes it non-optional); v1 reset agents = scripted + teleop only, code-as-policy resets remain Waddle-operated-cell-only until the certified-scene list exists. The honest headline: v1's constraint is closed-side ops capacity, not SDK surface, and the roadmap should say so.

### 7.4 The process attack: the spec is starting to read like a patch series

Applying N1–N10 inline was correct for auditability and has a cost: normative text now lives in two voices (§2's design prose and §6's amendment rationale) with slightly divergent phrasings, inline *(N#)* tags interrupt the read, and a new engineer cannot tell at a glance which sentence wins if they drift apart. The document is also 1,000+ lines maintained by hand outside version control, edited conversationally — the exact configuration in which specs rot. Verdict: one more pass of this kind and the doc becomes its own risk. Amendment **N19**: the next release is a *consolidation*: rewrite §2–§3 cleanly with amendments absorbed and tags dropped, archive §5–§7 as a rationale appendix (the audit trail survives, out of the normative path), move the document into the `waddle-protocol` repository, and from then on amend it by reviewed PR — the spec should live under the same discipline it prescribes for everything else.

### 7.5 Proposed amendments from this pass

| # | Amendment | Touches |
|---|---|---|
| N11 | Live grants run on safely-measurable proxy signals with hysteresis; verbs re-measured only in safe windows; demotion never interrupts an active lease | §2.3, `services.proto` |
| N12 | Retake reset-verification: optimistic entry + async invalidation for operator retakes; blocking for autonomous resets (closes first-pass open question 2) | §2.4, §2.6 |
| N13 | Audit labels from non-intervenor labelers; random retention quota as the audit sample (opt-in for Reference/SidecarOnly, else metrics marked **unaudited**); slice sized to a stated confidence target | §2.6, contracts |
| N14 | Runtime dual-write detection during advisory-lease bypass: commanded-vs-proprioception divergence ⇒ hold + alert + trace | §2.3, §2.4 |
| N15 | Codecs signed and version-pinned in config; mandatory load-time certification; codec trait gets its own stability declaration | §3.2, §3.6 |
| N16 | Timing envelopes published as non-warranty bench observations; safety-adjacent numbers binding only via per-deployment `waddle doctor` | §2.7, §3.1 |
| N17 | *Teleoperator* vs *site operator*; unqualified "operator" banned in normative text | §2.8 |
| N18 | Born-claimed episodes as a distinct metrics class; v0 scopes one active episode per session, concurrency deferred by named trigger | §2.6, `episode.proto` |
| N19 | Next release is a consolidation: absorb amendments, drop inline tags, archive stress-test sections as rationale, move the spec into `waddle-protocol` under PR review | doc process |
| — | Companion artifact required: a closed-side v1 cut list with named triggers (teleop/IK scope, judge scope, reset-agent scope) | roadmap |

### 7.6 Verdict after three passes

The pattern across passes is itself the finding. Pass one attacked promises and found the hard tradeoffs (resets, handoff, two idioms). Pass two attacked elegance and found static claims about dynamic realities; its fixes all had the shape *measure at runtime what was declared at init*. Pass three attacked those fixes and found the next shape: **several amendments moved risk into places with less scrutiny** — into the heartbeat (N6's unmeasurable verbs), into the audit slice (N9's biased sample), into the plugin loader (N4's write-path supply chain), into the brochure (N3's implied warranties). The meta-lesson to carry into implementation: every mitigation is a new component, and new components get red-teamed with the same energy as the original design — which is an argument for N19's consolidation-and-PR-review process being the real deliverable of this pass, since a spec that can only be stress-tested by marathon is a spec that will stop being stress-tested.

---

## Appendix A — Rename Plan for the Existing Internal Codebase

Context: the internal monorepo (also named `waddle`) already implements a substantial fraction of the closed side under different names — a Python package `waddle` with CLI `waddle` (`waddle ui`, `waddle ui-dev`, `waddle serve`, `waddle bridges`), a **broker** process owning hardware and safety (e-stop, lease, envelope, watchdog, MCAP recorder), a **bridge** process (FastAPI) hosting orchestration, the daemon plane (EventBus, TaskManager, Corpus, Inbox, CapabilityLibrary, Orchestrator), agent loops, and the teleop/intervention subsystem; a LiveKit-only teleop terminal web app; and a published `waddle-companion` phone relay on PyPI. This appendix reconciles that reality with the public naming in §2.8. The governing principle: **public names are ten-year commitments; internal renames are one-week refactors — the internal codebase cedes contested names to the public artifacts, and keeps everything else.**

### A.1 The one real rename: the Python package and CLI

The public SDK takes `import waddle` and the `waddle` console script. Therefore:

- **Package:** internal `waddle` → **`waddle_cell`**. The monorepo itself keeps the umbrella name `waddle` — repos are containers and never collide with import names. "Cell" is chosen because (a) it is the industry-standard unit for exactly what this package deploys — one robot + sensors + fixtures + safety envelope + controller (workcell in the ISO 10218 integration sense; eval cells in the AutoEval sense); (b) the codebase already voted for it (`cell_id`, broker client identities `cell-<cell_id>`, `cell_world`); (c) it scales with the business — a Waddle-operated deployment is N cells, and `cell_id` is already the key. Rejected alternatives: `waddle_bridge` (names the repo after one module, and semantically inverts the hierarchy — the broker outranks the bridge); `waddle_backend` (positional, and the position belongs to the cloud control plane the moment it exists; also miscues web-service instincts about a process that owns a 200 Hz e-stop poll).
- **CLI:** merge rather than rename. The open SDK owns the `waddle` console script and discovers subcommands via an entry-point group (e.g. `[project.entry-points."waddle.commands"]`). The cell package registers under a `cell` prefix: `waddle cell ui`, `waddle cell ui-dev`, `waddle cell serve`, `waddle cell bridges`. During migration, ship a transitional `waddle-cell` alias binary and leave shims on the old subcommands that print the new invocation.
- **Env vars:** the SDK claims the bare `WADDLE_*` namespace; cell-private variables move to `WADDLE_CELL_*` (`WADDLE_BRIDGE_HOST/PORT` → `WADDLE_CELL_BRIDGE_HOST/PORT`). Exception: `WADDLE_EVENTS_URL` is consumed by foreign-env tasks through the stdlib-only client — keep it working with a deprecation alias for one release cycle before flipping.
- **On-disk names** (`bridge_registry`, `tasks.db`, `events.jsonl`, `inbox.db`, `calib/<camera>.json`, `int-<hex>` episode ids) are internal artifacts with no public collision: **unchanged**. The public sidecar schema adopts `episode_id` as an opaque string, so `int-<hex>` ids flow through as-is.

### A.2 What keeps its name (most things)

- **`bridge` and `broker`** stay as the process names inside `waddle_cell` (`waddle_cell.ui_server`, `waddle_cell.broker`). They are good names for a load-bearing split — the broker enforces, the bridge requests and grants — and that split is precisely the envelope-vs-orchestration principle §2.3 exports. They simply never appear in public artifacts or docs (reserved-word policy, §2.8); externally, the bridge's productized descendant is the *relay* + *control plane*.
- **EventBus, TaskManager, Corpus, Inbox, CapabilityLibrary, Orchestrator**: unchanged as component names. `Corpus` is now *also* the public name of the data product (§2.6, `waddle.corpus`) — the internal component and the product concept share one word by design. `CapabilityLibrary` keeps exclusive rights to the word "capability" (robot skills); see A.3.
- **LiveKit work-plane RPC names** (`work_claim`, `work_answer`, `work_release`, `work_grant`) and **broker lease RPCs** (`handoff_lease`): unchanged — they already match the public glossary (see A.3), and they are wire-visible to deployed terminals. Same for participant identities (`ui-bridge`, `cell-<cell_id>`): these are identities, not names; keep them stable.
- **`waddle-companion`**: keeps its PyPI name. One phrasing change in docs going forward: call it the *phone companion*, not the *phone relay*, to avoid overloading "relay" now that `waddle-relay` is a product.
- **Teleop terminal** (`policy/teleop/web/`): stays in the monorepo for now; it was never inside the Python package, so the `waddle_cell` rename doesn't touch it. Flag: it deploys on a different cadence (Vercel) and is the seed of the operator-network client — likely its own repo when the SDK product matures.

### A.3 Vocabulary unification: one glossary, no translation table

As of v0.6 there is **no internal↔public vocabulary mapping to maintain** — §2.8's glossary is the single dictionary for the cell codebase, the protocol, the SDKs, and customer docs. The unification resolved as follows.

**Public adopts the production (bridge/broker) terms** wherever both had a word for the same concept: **claim** (from `work_claim` — the lobby is a claim broker), **lease** and lease handoff (from the broker's single-writer lease and `handoff_lease`; the protocol FSM adopts the broker's battle-tested semantics wholesale), **envelope** (the `Broker._handle_command` gate chain is the reference implementation; publicly the envelope belongs to whoever owns hardware), the **intervention lifecycle** with **engage/settle/release/retake** (from `InterventionLifecycle` — retake, a new episode under a held claim, was a state the greenfield FSM missed and is now normative in §2.4), **supervised rollout** (the public `waddle.rollout()` unit takes the internal name), **grant** (generalizing `work_grant`, the grant pages, and the authorization language — the bridge "requests and grants" — into the protocol's permission concept), and **Corpus** (the internal episode-index component name becomes the public data product and the `waddle.corpus` API).

**Internal adopts the new protocol terms** where the production system had a concept without a name: **tripwire** becomes the canonical internal word for the caller-side softeners (wall-slide clamp, hold-on-unreachable, the 0.5 s WAN deadman — advisory, hold-requesting, "the broker's gate chain is the floor"); **provenance** becomes the general form of the `operator_initiated` stamp (per-action origin + authorization semantics; "may bypass approval, never the envelope" is now a protocol invariant); **gate**, **sidecar**, and **episode** as specified in §2.8. The input-driven `TakeoverCoordinator` (rising-edge engage, ~0.75 s idle release) keeps its class name and is documented as the reference implementation of an *engagement-initiated claim* — the Company-B leader-arm semantics of §4.2.

**Collisions resolved by fiat, recorded here so they never reopen:** the word **capability** belongs exclusively to robot skills (the `CapabilityLibrary` sense) — permissions are **grants**, protocol evolution units are **feature flags**; within "grant" itself, the noun sense (declared permission) is canonical and the work plane's `work_grant` is the verb — awarding a claim — not an instance of the protocol's Grant; if that ambiguity ever bites, the escape hatch is renaming the protocol message (`Grant` → `StandingGrant`), never the wire RPC *(N1)*; the word **relay** belongs to `waddle-relay` — the phone app is the *companion*; **bridge** and **broker** are process names, not concepts, and follow A.2's scoping rather than the glossary.

**Deliberately not exported in v0:** the motion `AuthorizationStore` / standing authorizations with intersected envelopes (a candidate future protocol feature); the broker MCAP recorder remains the reference implementation of *Local* recording mode (§2.6) without protocol surface of its own.

### A.4 Registry and namespace actions (do these first; they're free)

1. Publish/reserve **PyPI `waddle-sdk`** (distribution) — the import name `waddle` ships inside it. PyPI `waddle` is squatted by a dormant AWS parameter-store tool (last release 2023, ~zero dependents); file a **PEP 541** transfer request in parallel and treat success as a bonus (`pip install waddle` becomes an alias; nothing else changes). Note `waddle-ai` (podcast toolkit) and `waddleml` (a W&B clone, of all things) are third-party — avoid those names entirely.
2. Reserve **crates.io**: `waddle-protocol`, `waddle-core`, `waddle-proxy`; **GitHub org** repos to match; container registry names for `waddle-proxy` / `waddle-relay`; the `waddle_ros` package name in the ROS index when ready.
3. Accept the low-probability import-name collision with the dormant PyPI `waddle` (both provide top-level `waddle`); document it in the SDK README's troubleshooting section rather than contorting the import name.

### A.5 Mechanics, sequencing, risks

Sequence: **(1)** reserve external names (A.4) — day zero, independent of everything; **(2)** delete the known scratchpad leftovers first (the temp `server.https` Vite block, the hand-minted `go.html`/`start.html` grant pages with the expired JWT, root `start.sh`) so the rename diff is clean; **(3)** mechanical package rename `waddle` → `waddle_cell` (imports, `pyproject`, test modules, `MANIFEST`/packaging globs), plus a grep pass for string-hardcoded `"waddle."` paths in configs, Modal endpoint wiring, Vercel functions, and systemd/tunnel launch commands; **(4)** land the CLI plugin architecture in the open SDK skeleton and re-register cell subcommands; **(5)** freeze the glossary (A.3) into `waddle-protocol` v0 *before any adapter code is written*.

Risks worth an explicit inventory before flipping: pinned automation on cell hosts invoking `waddle serve` (deploy scripts, Cloudflare tunnel units, operator runbooks); the Vite dev proxy's hardcoded bridge prefixes; and any external party already `pip install`-ing internal wheels by the old name. Wire-visible identifiers (LiveKit identities, RPC names, protobuf message names on data topics) are deliberately **out of scope** for the rename — they are protocol, not branding, and deployed terminals depend on them.

