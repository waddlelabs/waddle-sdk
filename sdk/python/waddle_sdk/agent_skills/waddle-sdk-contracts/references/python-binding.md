# Python binding contract

## Hollow frontend

The Python package describes hardware and composes lifecycle resources. It must not reproduce Rust decisions about claims, leases, handoffs, feature negotiation, or episode timelines. If Python grows an `if` about those concepts, move the behavior to the core and expose only the narrow result needed by the binding.

The extension shim grows declarations and keyword arguments, not policy logic. Select optional native features through the package's declared feature set rather than probing imports.

## Public lifecycle

The root package intentionally keeps a small surface: `Site`, `SiteSession`, `Run`, `load_site`, transport declarations, outcomes, and typed manifest/connector errors. Driver extensions live under `waddle_sdk.robots` and `waddle_sdk.cameras`. Structural runtime DTOs and optional support, FK, and geometry ports live in `waddle_sdk.runtime`.

`load_site(path)` validates and confines configuration without opening adapters. `Site.open(...)` returns an unopened session context. The enter operation resolves factories, opens resources, registers the exact declarations and verbs, and starts owned pumps. Exit finalizes recording before hardware close and cleans half-open resources.

## Declarations and support

Pure-Python descriptors compile to canonical protocol JSON. They validate shape; the core validates protocol semantics. A declaration's joint order and widths must exactly match driver reads and writes. Composite declaration order defines flattened action layout.

After open, the SDK derives `waddle.sdk.support/v1` from actual registered verbs, declarations, and optional opened facets. Adapters do not author this matrix. Missing optional FK, body geometry, intrinsics, or velocity feedforward removes only dependent behavior unless the owner configured an envelope rule that requires it.

## Extension seams

A manifest robot factory accepts `PartConfig`, returns a declaration-only `Rig`, and defers bus/thread creation to `Rig.build_arms`. A manifest-loaded part factory returns one bare `Arm`; site composition assigns its manifest part name. A camera factory accepts `CameraConfig` and is called only during site open.

Use structural protocols rather than inheritance. Keep vendor imports lazy. Connection credentials stay in manifest secret references and are resolved only for opening. Package facts with provenance and test them against the pinned vendor artifact where possible.
