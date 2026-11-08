"""Account gateway attachment; this frontend does not implement an MCP server."""

import json

from waddle_sdk_relay.protocol import PROTOCOL


def hello(*, site_id, connector_id, rig):
    # Retain the existing gateway envelope. The cloud connector negotiates MCP;
    # this device only runs the binary SDK relay after authenticated attachment.
    return json.dumps(
        {
            "type": "hello",
            "protocol": PROTOCOL,
            "workspace_id": None,
            "site_id": site_id,
            "connector_id": connector_id,
            "rig": rig,
            "supported_mcp_versions": ["2025-11-25"],
        },
        allow_nan=False,
        separators=(",", ":"),
    )
