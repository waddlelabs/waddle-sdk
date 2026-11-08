"""Attach the local executor to its authenticated cloud planning allocation."""

import asyncio
import json
import logging
from uuid import uuid4

from waddle_metal.contracts.connector import MCP_VERSIONS, encode
from waddle_metal.contracts.connector import PROTOCOL as METAL_PROTOCOL
from waddle_metal.contracts.sdk_bridge import PROTOCOL
from waddle_metal.errors import ToolError
from waddle_metal.transport.config import ConnectorConfig, load_connection, read_key
from waddle_metal.transport.planning_device import run
from waddle_metal.workspace.config import load_workspace
from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect

_LOG = logging.getLogger("waddle-connect.planning.websocket")
_LOG.disabled = True


async def serve(workspace, *, stop, torque_release_authorized=False):
    config = load_workspace(workspace)
    saved = load_connection(config.path)
    if config.site is None or saved is None or saved.rig is None:
        raise ValueError(
            "Cloud planning requires a configured SDK site and account rig"
        )
    connection = ConnectorConfig(
        saved.endpoint,
        read_key(key_file=saved.key_file),
        saved.allow_loopback_ws,
        rig=saved.rig,
    )
    while not stop.is_set():
        identity = str(uuid4())
        socket = await asyncio.to_thread(
            connect,
            saved.endpoint.removesuffix("/connect") + "/sdk",
            additional_headers={
                "Authorization": "Bearer " + connection.key,
                "X-Waddle-Execution": "hybrid",
            },
            subprotocols=[PROTOCOL],
            compression=None,
            proxy=None,
            open_timeout=5,
            close_timeout=2,
            max_size=4 * 1024 * 1024,
            max_queue=4,
            logger=_LOG,
        )
        try:
            if socket.subprotocol != PROTOCOL:
                raise ToolError(
                    "planning.protocol", "Gateway did not select the device protocol"
                )
            await asyncio.to_thread(
                socket.send,
                encode(
                    {
                        "type": "hello",
                        "protocol": METAL_PROTOCOL,
                        "workspace_id": None,
                        "site_id": config.site.id,
                        "connector_id": identity,
                        "rig": saved.rig,
                        "supported_mcp_versions": list(MCP_VERSIONS),
                    }
                ),
            )
            while not stop.is_set():
                try:
                    raw = await asyncio.to_thread(socket.recv, timeout=0.5)
                except TimeoutError:
                    continue
                message = json.loads(raw)
                if message.get("type") in {"waiting", "sdk_waiting"}:
                    print(
                        "Waiting for cloud planning. Hardware is still closed…",
                        flush=True,
                    )
                elif message == {"type": "sdk_attached"}:
                    print(
                        "Cloud GPU connected. Preparing the planner before hardware startup…",
                        flush=True,
                    )
                    await run(
                        socket,
                        config,
                        connection,
                        identity,
                        stop=stop,
                        torque_release_authorized=torque_release_authorized,
                    )
                    return
                else:
                    raise ToolError(
                        "planning.protocol", "Unexpected planner attachment response"
                    )
        except ConnectionClosed as error:
            if error.rcvd and error.rcvd.reason == "rig_bound":
                continue
            raise ToolError(
                "planning.transport_lost",
                "Planner connection ended; hardware was not restarted",
            ) from error
        finally:
            await asyncio.to_thread(socket.close)
