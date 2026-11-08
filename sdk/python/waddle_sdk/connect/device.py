"""The client computer owns SDK hardware; a cloud GPU worker runs Metal."""

import asyncio
import json
import logging
import signal
from pathlib import Path
from uuid import uuid4

from waddle_sdk_relay.device import Device
from waddle_sdk_relay.errors import RelayError as ToolError
from waddle_sdk_relay.protocol import PROTOCOL
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed

from .config import ConnectorConfig, load_connection, load_workspace, read_key
from .protocol import hello

_LOG = logging.getLogger("waddle-connect.sdk.websocket")
_LOG.disabled = True


class _ExactEndpoint(connect):
    def process_redirect(self, error):
        return error


async def serve(workspace, *, stop=None, torque_release_authorized=False):
    config = load_workspace(Path(workspace))
    saved = load_connection(config.path)
    if config.site is None or saved is None or saved.rig is None:
        raise ValueError(
            "Cloud setup requires a saved SDK site and account rig connection"
        )
    device = Device(
        config.site,
        config.path.parent,
        torque_release_authorized=torque_release_authorized,
    )
    while not device.opened_once and not (stop and stop.is_set()):
        try:
            connection = ConnectorConfig(
                saved.endpoint,
                read_key(key_file=saved.key_file),
                allow_loopback_ws=saved.allow_loopback_ws,
                rig=saved.rig,
            )
            async with _ExactEndpoint(
                saved.endpoint.removesuffix("/connect") + "/sdk",
                additional_headers={"Authorization": "Bearer " + connection.key},
                subprotocols=[PROTOCOL],
                compression=None,
                proxy=None,
                open_timeout=5,
                close_timeout=2,
                max_size=4 * 1024 * 1024,
                max_queue=4,
                logger=_LOG,
            ) as socket:
                if socket.subprotocol != PROTOCOL:
                    raise ToolError(
                        "sdk_bridge_protocol",
                        "Gateway did not select the SDK device protocol",
                    )
                await socket.send(
                    hello(
                        site_id=config.site.id, connector_id=str(uuid4()), rig=saved.rig
                    )
                )
                while True:
                    receiving = asyncio.create_task(socket.recv())
                    stopping = (
                        asyncio.create_task(stop.wait()) if stop is not None else None
                    )
                    try:
                        if stopping is not None:
                            done, _ = await asyncio.wait(
                                (receiving, stopping),
                                return_when=asyncio.FIRST_COMPLETED,
                            )
                            if stopping in done:
                                return
                        message = json.loads(await receiving)
                    finally:
                        receiving.cancel()
                        if stopping is not None:
                            stopping.cancel()
                        await asyncio.gather(
                            receiving,
                            *([stopping] if stopping is not None else []),
                            return_exceptions=True,
                        )
                    if message.get("type") == "waiting":
                        print(
                            "Waiting for this rig to be assigned to its app workspace…",
                            flush=True,
                        )
                    elif message.get("type") == "sdk_waiting":
                        print(
                            "Waiting for cloud Metal. Hardware is still closed…",
                            flush=True,
                        )
                    elif message == {"type": "sdk_attached"}:
                        print(
                            "Cloud Metal attached. Preparing your rig before hardware startup…",
                            flush=True,
                        )
                        await device.serve(socket, stop=stop)
                        return
                    else:
                        raise ToolError(
                            "sdk_bridge_protocol", "Unexpected SDK attachment response"
                        )
        except ConnectionClosed as error:
            reason = error.rcvd.reason if error.rcvd else ""
            if reason == "rig_bound" and not device.opened_once:
                continue
            raise ToolError(
                "sdk_transport_lost",
                "SDK/cloud connection ended; hardware was not restarted",
            ) from error


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(prog="waddle-connect-device")
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--release-torque-on-exit", action="store_true")
    parser.add_argument("--execution", choices=("cloud", "hybrid"), default="cloud")
    args = parser.parse_args(argv)

    async def run():
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        # asyncio supports add_signal_handler on the supported Unix clients.
        for kind in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(kind, stop.set)
        selected_serve = serve
        if args.execution == "hybrid":
            from .planning import serve as selected_serve
        await selected_serve(
            args.workspace,
            stop=stop,
            torque_release_authorized=args.release_torque_on_exit,
        )

    try:
        asyncio.run(run())
        return 0
    except (ValueError, OSError, ToolError) as error:
        print(f"waddle-connect: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
