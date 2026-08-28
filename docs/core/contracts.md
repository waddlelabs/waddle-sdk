# Contract hierarchy

Waddle's public behavior is specified in layers. Read the highest applicable source
first:

1. The [normative glossary](glossary.md)
   defines public terms.
2. The [normative FSM](fsm.md)
   defines episode, claim, lease, handoff, reset, and gate behavior.
3. The [versioning rules](versioning.md)
   define append-only protocol evolution and feature negotiation.
4. Protobuf schemas define the wire shape; golden fixtures and behavioral scenarios
   pin it to executable conformance.
5. The older rationale document explains design history. It is informative where the
   fresh normative documents are silent.

This site is explanatory. It links to the normative documents rather than copying
their guard tables, so there is one place to change protocol behavior.

## Stable contract rules

### Authority is native-core behavior

Claims, leases, handoffs, and timelines are implemented once in `waddle-core`.
Language bindings expose outcomes of those decisions; they do not add policy branches.
A lease is whole-robot and single-writer in v0, including a site with multiple parts.

### Handoff is hold-first

The current SDK public lifecycle exposes no handoff selector. An engage requests a
hold and requires it to succeed before the lease moves. If hold fails, the lease does
not move and no overlapping writer is created; callers must not claim the machine is
held unless hold success was confirmed.

### The envelope cannot be granted away

Every path to an SDK-managed driver crosses the owner's envelope. Targets are checked
as a whole and refused as a whole; clamping would manufacture a command the caller did
not send. Grants and negotiated features cannot weaken these checks.

### Protocol evolution is append-only

Fields and enum values are never reused or renumbered. Removed fields reserve both
number and name. A behavior-changing addition to `waddle.v0` needs an explicit
feature flag, normative text, and conformance coverage. A breaking change uses a new
protocol package.

### Feature negotiation is connection-scoped

Every connection registers again. Producers consult only the current connection's
accepted flags, and flagged messages are not buffered for replay onto a later
connection with a different answer.

### The control plane carries no unbounded media

Full-rate video and raw depth do not ride control RPCs. Any bounded exception must be
declared, negotiated, and rate-limited in the protocol.

## Changing a contract

A behavior change is incomplete unless its governing normative prose is updated and
at least one conformance scenario asserts it. Existing golden fixtures are append-only:
changing one is a breaking change, not a convenient test update. See the
[conformance format](https://github.com/waddlelabs/waddle-sdk/blob/main/waddle-protocol/conformance/scenario-format.md)
before adding a scenario.
