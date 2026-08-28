# Testing and commissioning

## Non-opening tests

- Import every adapter module with vendor packages absent; no device, socket, thread, or subprocess may appear.
- Construct each robot `Rig`; prove the vendor opener has not run.
- Validate `site.yaml` schema and topology without calling `Site.open()`.
- Compare declaration and `Arm` joint names, order, width, limits, rate, base frame, and gripper row.
- Exercise invalid/missing hardware facts and require a fail-closed error.

Use `scripts/validate_adapter.py` for a conservative static first pass. It intentionally does not import adapter modules and cannot prove import safety, runtime behavior, or hardware safety.

## Fake-vendor contract tests

- Read/write exact vectors in declared order; reject wrong width and non-finite values at the envelope.
- Hold calls a real vendor-safe hold behavior, not a no-op.
- E-stop latches; writes stay refused until an explicit successful re-enable.
- Close is idempotent and releases half-open resources in reverse order.
- Driver calls remain safe across simultaneous read, dispatch, and shutdown.
- Every owner-envelope rejection holds and sends no partial/clamped command.
- A twin respects the same limits, caps, rate, and latch semantics as live hardware.
- Optional FK and collision geometry are deterministic and conservative.
- Camera frames have exact dtype/shape/alignment; close unblocks a blocked capture.
- An end-to-end fake session records actions, observations, provenance, and terminal outcome and supports MCAP readback.

## Site-operator commissioning handoff

Do not automate this phase. Prepare a reviewed checklist covering physical e-stop access, exclusion zone, mount/tool/payload facts, low-energy posture, power/torque behavior, bus loss, watchdog, hold, e-stop latch, recovery, close, and measured latency bounds.

Require explicit approval before opening hardware. Begin in monitor posture. When motion is separately approved, use the smallest reviewed displacement and speed, observe measured settling, and never retry motion automatically after uncertainty. Record the exact unit, firmware, vendor SDK, adapter, SDK version, site manifest digest, results, and approver.

Passing software tests is not certification. List every unverified physical fact and keep dependent behavior unavailable.
