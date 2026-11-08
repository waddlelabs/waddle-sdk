"""Device-side selection of a saved SDK site, without loading a planner."""

from dataclasses import dataclass
from pathlib import Path

import yaml
from waddle_sdk_relay.connection import ConnectionConfig as ConnectorConfig
from waddle_sdk_relay.connection import load_connection as load_saved_connection
from waddle_sdk_relay.connection import read_key as read_private_key
from waddle_sdk_relay.errors import RelayError as ToolError

from waddle_sdk import load_site

from .extensions import connection_directory

__all__ = ["ConnectorConfig", "load_connection", "load_workspace", "read_key"]


def load_connection(workspace):
    root = Path(workspace).parent
    return load_saved_connection(
        connection_directory(workspace) / "connection.json", root=root
    )


def read_key(*, key_file):
    return read_private_key(key_file)


class _Loader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        keys = [self.construct_object(key, deep=deep) for key, _ in node.value]
        if any(not isinstance(key, str) for key in keys) or len(set(keys)) != len(keys):
            raise ToolError(
                "workspace_configuration", "Mapping keys must be unique strings"
            )
        return super().construct_mapping(node, deep=deep)


@dataclass(frozen=True)
class DeviceWorkspace:
    path: Path
    workspace_id: str
    site: object


def load_workspace(path):
    """Read only the site binding; the planning host validates planning settings."""
    path = Path(path).expanduser().resolve(strict=True)
    with path.open("rb") as source:
        raw = source.read(65537)
    if len(raw) > 65536:
        raise ToolError(
            "workspace_configuration", "Workspace configuration exceeds 64 KiB"
        )
    try:
        data = yaml.load(raw, Loader=_Loader)
    except yaml.YAMLError as error:
        raise ToolError(
            "workspace_configuration", "Workspace YAML is invalid"
        ) from error
    if (
        not isinstance(data, dict)
        or type(data.get("version")) is not int
        or data["version"] != 1
        or not isinstance(data.get("id"), str)
        or not 1 <= len(data["id"]) <= 128
        or not isinstance(data.get("site"), str)
        or not data["site"]
    ):
        raise ToolError(
            "workspace_configuration",
            "A device workspace requires version, id and site",
        )
    selected = (path.parent / data["site"]).resolve(strict=True)
    if not selected.is_relative_to(path.parent):
        raise ToolError(
            "workspace_configuration", "The SDK site must be inside the workspace"
        )
    return DeviceWorkspace(path, data["id"], load_site(selected))
