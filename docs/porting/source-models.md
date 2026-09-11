# Source models without opening hardware

Robot adapters share `waddle_sdk.robots.models.ModelSourceProvider`. An adapter
module may expose `model_sources(*, factory, part_name, part)` alongside its
ordinary hardware factory. The resolver `model_sources_for_driver(driver, ...)`
loads that same optional extension for every `module:factory` driver. Adding an
embodiment requires its adapter/assets/configuration, with no central vendor list.

The call receives the selected factory name, the configured part name and an
immutable snapshot of its manifest row. It may read installed source assets and
verify provenance. It must not construct/open a driver, download assets or start
services. Explicit externally supplied models can bypass source resolution.

Return `None` when the adapter/factory has no source support. Once a supported
source is selected, raise `ModelSourceError` on failure; do not replace it with a
different model or report absence. Import/provider errors remain typed failures.
Applications should isolate absent source support to model-dependent behavior;
it does not remove ordinary observation, hold or independent gripper support.

## Shared bundle

`ModelSources` contains:

- Source `format`, complete source-document bytes and relative-path asset bytes.
- Explicit primary joint names/order/units, source base body and TCP site.
- A declared `tcp_frame` identifier bound to that site. The application binds the
  source base body to the part's explicit configured base frame.
- License files and finite JSON provenance, including upstream revisions/hashes
  where available. Provenance must contain no secrets or execution authority.

The bundle snapshots and freezes metadata/assets, confines portable relative
paths and bounds their sizes. `model.xml` and `SOURCE.json` are reserved output
names. Consumers must check every referenced byte and explicitly reject source
formats or relationships they do not support. Source descriptions add no authority
to execute. One selected compiler/planner remains an application decision.

The initial reference source format is MJCF. The bundle does not hard-code a
joint count, arm/hand layout, mesh topology or finger mechanism. A complete source
retains its physical coordinates and relationships, including passive joints.
Applications own collision approximations and must declare their supported scope.
The existing [portable scene API](simulation.md) remains available for URDF scene
assembly and backend compilation; source loading does not open a simulation world.

A customer adapter can load its reviewed package assets using the ordinary
resource API:

```python
from importlib.resources import files
from waddle_sdk.robots.models import ModelSources


def model_sources(*, factory, part_name, part):
    if factory != "arm":
        return None
    resources = files("customer_robot") / "assets"
    # These bindings must match this adapter's actual reviewed source model.
    return ModelSources(
        format="mjcf",
        model=(resources / "arm.xml").read_bytes(),
        assets={"housing.stl": (resources / "housing.stl").read_bytes()},
        joint_names=("pan", "tilt"),
        joint_units=("rad", "rad"),
        base_body="mount",
        tcp_site="probe",
        tcp_frame=part_name + "_probe",
        licenses={"LICENSE.model": (resources / "LICENSE").read_bytes()},
        provenance={"source_revision": "reviewed-model-1"},
    )
```

Here the provider explicitly declares the `part_name + "_probe"` binding; generic
consumers do not invent it. A real adapter must also validate configuration that
changes source geometry before returning its bundle.

## YAM reference

YAM implements exactly this optional contract. It verifies the installed I2RT Git
pin and each consumed SHA256 record, combines the SDK URDF chain with the complete
vendor hand, and retains both physical slides and their coupling in MJCF. Its
missing vendor terminal mesh is replaced through the declared hand/TCP attachment,
inside the adapter. Neither callers nor the shared bundle need YAM-specific fields.
Mesh bytes remain in the optional vendor installation and are not shipped in the
SDK wheel. Missing or modified selected vendor sources raise `ModelSourceError`.

Tests in `sdk/tests/test_model_sources.py` cover independent adapter resolution,
absence, selected failures, frozen metadata and path/binding refusal.
`sdk/tests/test_yam_model_sources.py` retains vendor provenance/relationship checks.
