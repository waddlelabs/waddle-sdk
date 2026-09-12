# Real-device behavior tests

All SDK tests live under `sdk/tests`. Ordinary `pytest` runs software tests and
skips the `live` group before any driver is opened. The live group uses the same
pytest runner and ordinary `-k`/`-m` selection:

```bash
# From sdk/, using the project's Python environment:
python -m pytest --live --collect-only -q
python -m pytest --live --live-site=/absolute/site.yaml -m 'live and not motion'
python -m pytest --live --live-config=/absolute/bench.json -m live -x
```

`--live` discovers configured CAN/USB identities using the public non-opening SDK
discovery API. It can select camera inspection directly from identified cameras.
Robot tests additionally require a site manifest, an authorized shutdown policy,
and, for motion, explicit named-joint targets and limits. A CAN interface alone
does not establish which robot is connected or that it can execute a command.
Missing prerequisites skip only dependent tests and appear in the discovery
summary. Driver/service failures during execution fail the test with their origin
information; discovery never converts a failing device into a successful test.
`--collect-only` enumerates metadata without opening drivers or sending commands.
`CI` and `GITHUB_ACTIONS` reject `--live`; release jobs run the normal suite.
Physical execution is sequential and stops at the first failure.

Configuration resolves from `--live-config`, `WADDLE_SDK_LIVE_CONFIG`, or
`.live-tests.json` in the working directory. `--live-site` or `WADDLE_LIVE_SITE`
can select a site independently. The JSON schema name is `waddle.live-bench/v1`.
Use `site`, `evidence_directory`, `parts`, `cameras`, `cases`, and explicit
`torque_release_authorized`. Each motion case names `case_id`, `part`,
`joint_names`, `start_rad`, `target_rad`, positive `velocity_rad_s`,
`acceleration_rad_s2`, `jerk_rad_s3`, `joint_tolerance_rad`, `tcp_tolerance_m`,
`settle_s`, and `min_displacement_m`. `max_tracking_error_rad` bounds deviation
from the streamed reference. `velocity_feedforward` explicitly selects that
command path. Targets are local rig inputs, not guessed from USB or CAN discovery.

Optional per-case `minimum_settle_s` defaults to zero. A positive value keeps the
final target commanded for at least that many seconds after its first successful
dispatch before accepting arrival or issuing Hold. The latest three observations
must still meet both arrival tolerances. `settle_s` remains the total deadline
after the planned trajectory; the minimum must be finite, nonnegative and no
greater than that budget. Allow room for command and sampling latency: equal
minimum and total budgets can time out. Gates, tracking checks and faults still
interrupt immediately. Reference, return and configured rest inherit the case's
minimum. Reports retain `minimum_settle_s` and `target_latched_elapsed_s`; the
latter is local command completion time, not motor acknowledgement.

Single-arm tests project the selected part into a public SiteSession and keep the
original site ID and ownership lock. An unrelated arm is never opened merely to
hold it during another arm's trial. The native gate and manifest envelope still
apply. Camera inspection
owns only the selected camera; missing arms cannot prevent camera checks. Motion
uses a quintic trajectory, measured joint arrival and three settled samples, then
returns to the configured reference. JSON reports retain samples, target errors,
encoder-derived TCP displacement, timing, outcome and cleanup errors. FK-derived
positions are not independent external metrology. Passing SDK motion alone does
not prove comparison with the raw vendor API; paired trial evidence is required.

For a YAM comparison, add a `comparison` object to the profile with `vendor` set
to `i2rt` and explicit nonnegative `initial_tolerance_rad`,
`joint_error_margin_rad`, `tcp_error_margin_m`, and `settling_margin_s`. Choose
these margins before measuring; the case's absolute arrival limits still apply.
The ordinary motion test then compares pristine raw I2RT and SDK child processes
sequentially. Other robot adapters keep the SDK-only measured test and can add a
small raw adapter using the same observation/command/hold/pose boundaries.
Both child processes must exit successfully and identify the same source manifest
hash. Each must report reference, target and return exactly once in that order;
reference-only diagnostics require only the reference phase. Missing paired
measurements fail comparison rather than omitting that phase from the verdict.

Each backend uses the same named case, quintic interpolation and external
command cadence. Starts come from fresh measured joints. Reports compare target
start states and reject differences beyond the declared bound, or trajectory
durations differing by more than one command period. `motion_elapsed_s` (also
`elapsed_s`) ends at the motion loop's terminal decision; `arrival_elapsed_s`
identifies the accepted observation after three consecutive in-tolerance samples
and any minimum settling window, or is null on nonarrival. The measured
endpoint is that target-phase observation. `hold_elapsed_s`, `total_elapsed_s`
and `post_hold_measured_rad` separately record cleanup latency and any later
drift. Settling after the trajectory is used for the configured timing regression
margin. This measures SDK overhead at a matched
external cadence, not I2RT's maximum native streaming throughput.

For a bounded reference diagnostic before full motion, run from `sdk/`:

```bash
PYTHONPATH=tests python -m live.sdk_live.paired --config /absolute/bench.json \
  --part left --case yaw-2cm --reference-only
```

This opt-in command opens hardware. CI is refused. It collects the other backend
after a healthy bounded nonarrival, records the failed absolute result, and
returns a failing exit code. A motor fault, unsafe tracking or uncertain shutdown
blocks the next owner. A failed reference never advances to a larger target.
The SDK benchmark also checks retained `robot.part_fault` events before commands
and after shutdown. A later healthy read cannot hide an earlier selected-part
failure: `part_faults` preserves the event cursor, timestamp, part and complete
fault, and that part receives no further diagnostic trajectories.
Reference-only evidence reports independently measured starting states and is
diagnostic evidence, not a matched target throughput baseline.

Profiles can declare `rest_positions.<part>` with `joint_names` and `position_rad`
arrays copied from a reviewed, physically supported parking pose. Names must match
that part's case joint set; order may differ. Configuration rejects duplicate names,
width mismatches and nonfinite or boolean positions before opening hardware.
After healthy bounded trials, including reference nonarrival, each paired backend
moves to its configured rest before closing. It uses the case's unchanged motion,
tracking, arrival and settling limits. `rest_trials` retains measured parking
evidence separately from comparison `trials`; `rest_error` retains an original
parking fault. Parking failure or missing required parking blocks the next backend
and fails the pair. A driver, gate or captured background fault stops subsequent
trajectory commands, including automatic return/parking. No rest is inferred when
the profile omits it, and a configured position does not itself prove support.

Raw I2RT acquires the SDK's existing site lock and checks the same model envelope
without constructing an SDK live driver. Fresh processes keep SDK receive/command
patches out of the raw run. The vendor defaults to gravity factors
`[1,1.1,1.1,1.2,1,1]`; SDK defaults are `[1,1.1,1.2,1.3,1,1]`. Both comparison
backends explicitly use the configured site factors or that unchanged SDK
default. Actual gains, gravity factors, friction, command cadence and hand limits
are recorded and must match. Optional `part_options` maps part names to reviewed
factory option overrides in the test projection. The raw run's freshly measured
gripper motor limits are reused by the SDK child, with the source report recorded,
so the pair uses one calibration and hand mapping.

Gain evidence distinguishes requested `kp`/`kd` from `encoded_kp`/`encoded_kd`,
computed with the pinned vendor's 12-bit MIT codec; these are not motor readback.
Both backends honor explicit per-part `arm_gains` with independent six-joint
P/D vectors and reject gain settings the codec would clamp. The original default
gains remain unchanged. See [gain configuration](../../../docs/porting/robot.md#yam-gain-configuration).

Releasing torque between processes requires a fixture or supported resting pose
that stays inside the original joint/TCP envelope. Support on foam alone does
not prove that condition. If the first measured pose is outside, the ordinary
envelope rejects the first target; neither backend corrects it automatically.
The site operator must re-establish a supported in-envelope setup. Pinned I2RT's
`get_yam_robot` performs optional jaw calibration before returning observations,
so an arm-pose precheck cannot precede that calibration through this API. Supply
verified per-unit `gripper_limits` to avoid recalibration; never guess them.

`paired-<run-id>.json` retains both backend reports and their trials; the
`paired-report.json` alias points to the latest pair. Backend reports and original
stdout/stderr logs remain alongside it. Evidence includes measured joints and
velocities, per-axis errors, sampled tracking error, command latency, TCP poses
and optional YAM orientation, outcome, and independent cleanup errors. Set
`minimum_orientation_rad` for orientation-only cases. The source manifest hash
and selected part configuration identify the tested setup. I2RT measured effort,
computed feedforward torque and CAN rate are explicitly labelled private test
diagnostics; public SDK observations do not claim those fields. Neither SDK
envelope timestamps nor vendor read timestamps prove CAN acquisition freshness.
Worker liveness and observed CAN cache replacement are checked, and original
background thread errors are preserved separately. Raw teardown joins the vendor
workers before closing the socket; it does not kill a stuck hardware process.

Before timing a YAM trial, both backends also require observable post-start
feedback. I2RT can initially publish computed feedforward alongside cached
pre-servo observations. A test-only readiness check observes the constructor's
holding command in the CAN command cache, two later complete CAN cache generations
(a previous scan may still await publication), and subsequent robot-state
ingestion. `startup_evidence` records these observations, elapsed time and any
original fault. The total deadline, including lock acquisition, is one second;
this is not a fixed sleep. Readiness sends no commands and tests no position or
effort threshold. It does not establish physical settling or a valid starting
pose: the unchanged envelope still checks the first trajectory target.

Prefer tests of complete behavior: sustained frame flow, measured motion and
accuracy, recovery/ownership, and rejection of incomplete hardware contracts.
Avoid checks whose only purpose is to pin source layout or duplicate an assertion
for different arbitrary strings. Preserve distinct failure modes when combining
cases. Never hide a failed physical trial by loosening its tolerance afterward.

Existing `test_livekit_public.py` also requires `--live` plus either the three
scoped LiveKit grant variables (`WADDLE_TEST_LIVEKIT_URL`,
`WADDLE_TEST_LIVEKIT_PUBLISHER_TOKEN`, `WADDLE_TEST_LIVEKIT_VIEWER_TOKEN`) or the
scoped URL with `WADDLE_TEST_LIVEKIT_API_KEY` and `WADDLE_TEST_LIVEKIT_API_SECRET`.
Only a selected media fixture uses signing credentials to create its own isolated
room and scoped grants, then deletes that room and records cleanup evidence.
Caller-provided grants never authorize deleting their room. Collection reports
credential availability without creating rooms or verifying service permission;
unrelated selected tests do not call LiveKit. With the same media configuration
and a configured site, `test_03_media.py` publishes each
selected physical camera through the native SDK media path, receives RGB and depth
preview frames, interrupts publisher signaling and rejoins the viewer. It projects
one camera with a mock motion context under the original site ownership ID, so
physical arms are never opened. Source resolution and raw metric depth remain
checked independently of WebRTC's adaptive received-frame dimensions. Reports
record frame counts and received sizes; passing synthetic transport alone does
not establish physical camera publication.

## Named-part acceptance

`test_04_named_parts.py` tests the SDK directly with two arms in one site/run.
Only `named_parts.parts` open, retaining the original site ID, lock and envelopes.
All cases need both discovered devices, authorized teardown and device feedback
probes. Execution checks the support matrix: named observations, named actions/send
permission where needed, FK for trajectories, and the selected stop grant. Missing
requirements skip the dependent test; malformed profiles and device faults fail.
This harness introduces no runtime or SDK contract changes.

```bash
python -m pytest --live --live-config=/absolute/bench.json \
  tests/live/test_04_named_parts.py --collect-only -q
python -m pytest --live --live-config=/absolute/bench.json \
  -m 'named_parts and not motion' -x
python -m pytest --live --live-config=/absolute/bench.json -m named_motion -x
```

Add an optional `named_parts` object to the existing bench JSON. Numeric limits
are mandatory, positive, finite, and chosen before execution for the actual bench:

| Field | Meaning |
|---|---|
| `parts` | Exactly two distinct names from top-level `parts`, first arm then neighbor. |
| `observation_s` | Read window, at least twice `max_feedback_gap_s`. |
| `max_observation_latency_s` | Maximum complete probe + named SDK read latency, including test instrumentation. |
| `max_feedback_gap_s` | Maximum observed time without a new device generation or ingested state; not exact acquisition age. |
| `mapping_tolerance_rad` | Bound between named public arm positions and probe positions; also the neighbor reference comparison bound. |
| `motion_cases` | Optional three `{part, case_id}` references: first arm, neighbor, first arm again. |
| `max_command_latency_s` | Maximum `step_parts` latency; required with motions. |
| `max_command_gap_s` | Maximum interval between submissions for each active arm; required with motions. Missed periods are never replayed. |
| `min_progress_rad` | Minimum measured neighbor progress after reusing the first arm; required with motions. |

The repeated case starts at the first case's target, matched by joint name. The
neighbor duration must exceed the first trajectory plus its settling budget. Both
parts need explicit top-level `rest_positions`. The test acquires reviewed
references, streams both arms, requires three measured joint/TCP arrival samples
on the first arm while the neighbor remains outside tolerance, then sends a sparse
command to reuse the arrived arm. It checks the neighbor's retained controller
reference against its last submission and verifies that the sparse command leaves
it intact. Later encoder progress and measured arrival provide physical evidence;
a retained reference alone cannot pass. Choose enough separation to observe these
events on the real bench.

Ordinary per-arm completion issues no Hold. Healthy trials return both arms to
reviewed rest positions with unchanged limits. Any failed reference, tracking or
latency violation, nonarrival or device/gate fault goes straight to existing site
teardown, without further trajectories or automatic parking. This test stops on
an error; injected fault isolation remains a deterministic software test, without
inducing motor or bus failures.

Reads sample both parts together and each exact subset, checking declared joint
order/width, base frames, finite positions/velocities and device feedback. YAM
includes a test-only probe of actual CAN cache replacement and robot-state
ingestion, bounded locks and worker liveness. Startup readiness/settings are
recorded per arm. SDK envelope timestamps prove ordering only. The controller
reference is local I2RT state, not motor acknowledgement.

Other adapters declare `feedback_probes: {"part": "module:factory"}` inside
`named_parts`. Collection checks declaration without importing or opening probes.
Execution calls `factory(bench, part)`; its read-only, bounded `sample()` returns
JSON-safe `source`, increasing integer `generation` and `ingestion`, `joint_names`,
corresponding `position_rad`, and `command_position_rad` for the retained local
reference. Counters reflect device updates/ingestion, never sample calls or SDK
envelope timestamps. Probes open no additional owner and send no commands. Probe
errors fail. A probe can compare a declared subset of axes (YAM compares six arm
joints, excluding the gripper's different motor coordinates); public shapes still
cover all axes.

Two optional profiles enable supervisory checks:

- `envelope`: `{part, joint_name, rejected_position_rad, observe_s,
  max_drift_rad}`. The finite reviewed target must be outside that joint's declared
  range. One named submission must retain a safety refusal while both measured
  arms stay within the drift bound. No return command follows this deliberate
  refusal. Select with `-m named_envelope`.
- `stops`: a list of `{method, supported, after_s, response_s, observe_s,
  min_velocity_rad_s, max_velocity_rad_s, max_drift_rad}`. `method` is `hold` or
  `estop`; `supported` must be explicitly `true` for reviewed physical support of
  both arms, including possible torque release. `after_s` must interrupt both
  trajectories. Both arms must be moving above `min_velocity_rad_s` before the
  stop. Afterward, measured speed must fall below `max_velocity_rad_s` within
  `response_s` and remain below it through `observe_s`, with drift bounded from
  the pre-stop pose. The observation window exceeds the response deadline; the
  moving threshold exceeds the stopped threshold. Each entry opens its own
  session. No targets, recovery or automatic parking follow the stop. Select with
  `-k explicit_stop`; missing profiles skip those cases.

`sdk-named-<run-id>.json` and latest `sdk-named-report.json` retain the source
manifest hash, configuration/support, raw named samples and device provenance,
command times/gaps and exact receipts, tracking/arrival/reuse evidence, stop
response/drift, original failures and independent shutdown errors. FK poses are
encoder-derived, not external metrology. Timing bounds establish acceptance under
configured two-arm load, not raw-vendor throughput comparison. Software plant and
native-mock results must never be reported as physical acceptance.
