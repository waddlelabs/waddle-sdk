# Waddle SDK

Waddle SDK is the customer-side supervision layer for real-world robot policy
rollouts. It owns the hardware and camera lifecycle, the site owner's hard-safety
envelope, authority enforcement, paired timestamps, and raw recording.

This documentation has four routes:

- [Concepts](concepts/index.md) explains the boundaries and vocabulary.
- [Protocol and Rust core](core/contracts.md) describes the normative contracts and
  their reference implementation.
- [Python](python/index.md) documents the first language frontend.
- [Port custom hardware](hardware-backends.md) is the implementation path for a new
  robot, simulator, or camera.

!!! important "The site owns hard safety"

    Waddle never supplies the owner's physical stop, joint and workspace limits,
    keep-outs, watchdogs, or safe commissioning procedure. A Waddle grant or lease
    cannot bypass that envelope.

## Source of truth

The protocol documents are normative. Start with the version-matched
[glossary](core/glossary.md), [FSM](core/fsm.md), and
[versioning rules](core/versioning.md).
If explanatory documentation conflicts with them, the normative document wins.
