---
name: waddle-sdk-contracts
description: Explain, inspect, or review Waddle SDK protocols and contracts, including ownership boundaries, vocabulary, lifecycle, clocks, recording, Rust-core layering, feature negotiation, Python bindings, and compatibility. Use for architecture questions, contract reviews, protocol changes, binding work, or deciding which layer owns behavior. Do not use it as a live robot-control surface.
---

# Waddle SDK Contracts

Use the contracts shipped with the installed SDK version rather than remembered APIs.

## Workflow

1. Identify the installed `waddle-sdk` version and, when available, the repository revision.
2. Classify the request as concept explanation, protocol review, Rust-core work, or Python-binding work.
3. Read the matching references before answering or editing:
   - Read [concepts.md](references/concepts.md) for ownership, vocabulary, lifecycle, safety, time, and recording.
   - Read [rust-core.md](references/rust-core.md) for crate boundaries, the gate, threading, and conformance.
   - Read [python-binding.md](references/python-binding.md) for the hollow frontend, public lifecycle, descriptors, runtime facets, and extension seams.
4. In a source checkout, treat `waddle-protocol/docs/GLOSSARY.md`, `FSM.md`, and `VERSIONING.md` as normative. Read the relevant file completely before changing its contract. Treat the bundled references as orientation, not a replacement for those sources.
5. State whether a claim is normative, implementation behavior, or explanatory guidance.
6. Verify exact signatures and feature names against the installed package or checked-out source before producing code.
7. For a behavior change, update the normative contract and conformance fixture together; never patch a language binding around the Rust core.

## Review guardrails

- Keep claims, leases, handoffs, grants, timelines, and gate decisions in the Rust core.
- Keep hardware, cameras, the owner envelope, paired timestamps, and raw recording below higher product layers.
- Never describe Waddle as providing the owner envelope. The SDK applies owner-supplied limits at the non-bypassable hardware seam.
- Use `grant` for permission, `claim` for work assignment, `lease` for actuation single-writer ownership, `capability` for robot skills, and `feature flag` for protocol evolution.
- Preserve session-monotonic nanoseconds and the wall-clock twin captured at stamp time. Never derive one from the other later.
- Keep media off the control plane except for explicitly declared, bounded protocol exceptions.
- Evolve protobuf fields append-only; reserve both removed field numbers and names.
- Treat omission of an optional facet as local degradation, not permission to invent support.

## Output expectations

Name the controlling contract and owning layer. Highlight safety or compatibility consequences. When proposing code, identify the normative text and conformance coverage that must move with it.
