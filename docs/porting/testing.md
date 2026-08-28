# Validation and commissioning

Build confidence in layers. Most failures should be found without a connected device.

## 1. Static and import checks

- Install the adapter into a clean environment with the supported SDK version.
- Import its package with vendor libraries absent unless the selected extra requires
  them.
- Validate representative `site.yaml` files against the shipped schema.
- Call the `PartConfig` factory and prove it opens no bus, starts no thread, and makes
  no motion.
- Assert every facts table has inspectable provenance.

## 2. Fake-vendor contract tests

Use a fake object at the vendor API boundary, not only the SDK's `SimDriver`.

- Driver read/write arrays use the declared width, order, and units.
- `hold()` cancels or arrests commanded motion according to the device contract.
- `estop()` latches, later writes refuse, and recovery is explicit.
- `close()` is deterministic and safe after partial construction.
- A failure opening a later resource closes everything already opened.
- Non-finite, wrong-width, out-of-limit, and over-step targets move nothing.
- Optional position/velocity control falls back to the identical position target.
- FK and body spheres are deterministic, frame-correct, and conservative.
- A camera close unblocks a capture waiting in another thread.
- Camera arrays, dtype, alignment, intrinsics, and point resolution match the declared
  stream.

## 3. Complete simulated site

Open the adapter through `load_site()` and `Site.open()`, not by directly driving its
classes. Exercise observation, one admitted command, one envelope refusal, hold,
e-stop, recovery behavior, terminal outcome, shutdown, and MCAP readback. This catches
declaration/composition errors that unit tests cannot.

For a multipart manifest, also verify that every factory returns one bare arm, all
rates and postures agree, base frames match, and a cross-part geometry refusal moves
no part.

## 4. Hardware-in-loop commissioning

Hardware tests are attended and site-specific. Before enabling motion, record:

- the exact unit, firmware, vendor package, adapter, and SDK versions;
- mounting, tooling, payload, floor/table, keep-outs, and measured joint limits;
- physical e-stop function and measured response;
- hold behavior and latency under expected load;
- command rate, watchdog behavior, bus loss, process loss, and reconnect behavior;
- close and power/torque behavior after normal and exceptional exit; and
- camera frame rate, alignment, shutdown, and recovery from a stalled stream.

Begin with the manufacturer's safe mode and the site's smallest reviewed motion.
Require explicit site-operator approval for each increase in scope. An automated test
passing is evidence, not certification, and it never authorizes live motion by itself.

## Acceptance record

A supported port should leave behind a reviewable record containing the facts sources,
manifest, fake-vendor results, complete simulated-site results, hardware unit identity,
commissioning observations, known omissions, supported OS/Python/vendor SDK versions,
and the maintainer responsible for future compatibility.
