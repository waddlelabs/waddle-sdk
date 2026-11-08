"""Account gateway attachment; this frontend does not implement an MCP server."""

import json


def hello(*, site_id, connector_id, rig):
    # Retain the existing gateway envelope. The cloud connector negotiates MCP;
    # this device only runs the binary SDK relay after authenticated attachment.
    return json.dumps(
        {
            "type": "hello",
            "protocol": "metal.mcp.v1",
            "workspace_id": None,
            "site_id": site_id,
            "connector_id": connector_id,
            "rig": rig,
            "supported_mcp_versions": ["2025-11-25"],
        },
        allow_nan=False,
        separators=(",", ":"),
    )
