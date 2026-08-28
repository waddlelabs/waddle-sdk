---
name: port-waddle-hardware
description: Scaffold, implement, review, or test a custom Waddle SDK robot, simulator, or camera adapter in Python. Use when adding vendor hardware, creating an external adapter package, declaring site hardware, auditing lazy lifecycle and cleanup, building fake-vendor tests, or preparing human-supervised commissioning. This skill is non-actuating and must stop before any live motion.
---

# Port Waddle Hardware

Build an external Python adapter against the installed SDK's public contracts. Do not modify central dispatch, add a vendor registry, import higher product layers, or create another control surface.

## Workflow

1. Read [robot-contract.md](references/robot-contract.md) for a robot or simulator, [camera-contract.md](references/camera-contract.md) for a camera, and [testing.md](references/testing.md) before commissioning work.
2. Inspect the installed SDK version and exact type signatures. Prefer `PartConfig` or `CameraConfig` factories.
3. Collect authoritative hardware facts before generating code: joint names/order, SI units, limits, rate, per-command caps, stop/recovery behavior, frames, tooling/payload, camera stream shape, and shutdown behavior. Record provenance beside each fact.
4. Scaffold only after those facts exist. From this skill directory, for example:

   ```bash
   python scripts/scaffold_adapter.py robot --name acme-arm --output /path/to/parent \
     --facts-source 'Acme Arm manual rev 4, table 2' \
     --joint shoulder:-2.0:2.0:0.02 --joint elbow:-1.5:1.5:0.02 \
     --rate-hz 50
   ```

   Use `simulator` with the same fact arguments, a required sourced `--home`, and an explicit synthetic/test-model source for a harmless twin. A live `robot` scaffold deliberately has no home fact because its generated `home()` refuses unattended motion. Use `camera` with `--width`, `--height`, `--fps`, and its stream-specification source. `--facts-source` is required and is recorded verbatim; the scaffold cannot prove that the citation is authoritative. The script refuses an existing target and never imports or opens hardware.
5. Replace the generated vendor-opening stub with a lazy vendor client. Keep all bus connections and threads inside `build_arms()` for robots or the site-open camera factory call. Never open anything during import or robot-factory construction.
6. Run fake-vendor tests. Then run static, non-opening validation:

   ```bash
   python scripts/validate_adapter.py /path/to/project --site site.example.yaml
   ```

   This validates the site schema and inspects adapter ASTs without importing adapter modules.
7. Add focused failure and cleanup tests before touching hardware.
8. Prepare a site-specific commissioning checklist with measured limits and hold/e-stop latency criteria.
9. **Stop and obtain explicit site-operator approval before opening live hardware or issuing any motion.** This skill contains no live-motion command or automatic commissioning script.

## Non-negotiable guardrails

- Never invent, infer, silently widen, or replace hardware limits. Preserve provenance for every physical number.
- Never label live hardware as `kind="sim"`; only a harmless twin may use it.
- Never put claim, lease, grant, authority, hosted-service, or protocol-FSM logic in an adapter.
- Never claim Waddle provides the owner envelope. The site owner supplies the hard-safety facts; all commands remain subject to them.
- Never clamp a rejected target into a different command. Refuse the complete command and hold.
- Never initiate live motion, homing, re-enable, or commissioning from this skill.
- Never require a vendor SDK at adapter-module import time. Import it only on the opening path and provide an actionable missing-extra error.
- Test fake vendor objects before hardware-in-loop work. Test half-open cleanup and blocking camera shutdown.
- Use the installed SDK's interfaces, not remembered signatures or snippets from another release.

## Completion report

Report implemented scopes, sourced versus still-missing hardware facts, fake-vendor coverage, static-validation results, optional facets intentionally omitted, and the exact live checks still requiring a site operator. Never call generated code certified merely because tests pass.
