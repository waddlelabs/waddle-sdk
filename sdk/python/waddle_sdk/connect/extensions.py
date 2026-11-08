"""Optional application integrations for execution and existing workspace layouts.

The SDK supplies hosted device setup. Applications can register an execution
module with ``main(argv, *, prog)`` and, for device execution, async
``serve(workspace, *, stop, torque_release_authorized)``. Discovery does not load
extensions until a user selects their mode. Layout providers return a directory
only for an existing connection they own; new SDK connections use ``.waddle``.
"""

from importlib.metadata import entry_points
from pathlib import Path


def execution(mode):
    providers = tuple(entry_points(group="waddle_sdk.connect_execution", name=mode))
    if len(providers) != 1:
        raise ValueError(
            f"Execution mode {mode!r} requires exactly one installed setup extension"
        )
    return providers[0].load()


def connection_directory(workspace):
    root = Path(workspace).parent
    default = root / ".waddle"
    candidates = {default} if (default / "connection.json").exists() else set()
    for provider in entry_points(group="waddle_sdk.connection_layouts"):
        selected = provider.load()(Path(workspace))
        if selected is not None:
            candidates.add(Path(selected))
    if len(candidates) > 1:
        raise ValueError(
            "Multiple saved connection layouts; review the workspace before starting"
        )
    return next(iter(candidates), default)


def prepare_connection(endpoint, key_file):
    """Run optional installed system prerequisites before opening hardware."""
    for provider in entry_points(group="waddle_sdk.connect_prepare"):
        provider.load()(endpoint, key_file)
