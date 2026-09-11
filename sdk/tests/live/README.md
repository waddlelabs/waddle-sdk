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

Robot tests own one public SiteSession and use its native gate. Camera inspection
owns only the selected camera; missing arms cannot prevent camera checks. Motion
uses a quintic trajectory, measured joint arrival and three settled samples, then
returns to the configured reference. JSON reports retain samples, target errors,
encoder-derived TCP displacement, timing, outcome and cleanup errors. FK-derived
positions are not independent external metrology. Passing SDK motion alone does
not prove comparison with the raw vendor API; paired trial evidence is required.

Prefer tests of complete behavior: sustained frame flow, measured motion and
accuracy, recovery/ownership, and rejection of incomplete hardware contracts.
Avoid checks whose only purpose is to pin source layout or duplicate an assertion
for different arbitrary strings. Preserve distinct failure modes when combining
cases. Never hide a failed physical trial by loosening its tolerance afterward.

Existing `test_livekit_public.py` also requires `--live` plus its three scoped
LiveKit grant variables. Credential presence alone never starts that service test.
