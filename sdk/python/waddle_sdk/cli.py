"""Customer-side SDK connector command."""

from __future__ import annotations

import argparse
import getpass
import os
import signal
import sys
import threading
from collections.abc import Sequence

from . import Grpc, load_site
from ._hosted_ui import (
    UiInvitationConfig,
    UiInvitationError,
    WaddleUiInvitationClient,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="waddle-sdk")
    commands = parser.add_subparsers(dest="command", required=True)
    connect = commands.add_parser(
        "connect",
        help="connect one site to its authorized hosted workspace",
    )
    connect.add_argument("--site", required=True)
    connect.add_argument(
        "--target",
        default=os.environ.get(
            "WADDLE_CONNECTOR_TARGET", "https://connect.waddlelabs.ai:443"
        ),
        help="hosted waddle.v0 endpoint",
    )
    connect.add_argument(
        "--authorization-timeout",
        type=float,
        default=15.0,
        help="seconds to authenticate before refusing to open hardware",
    )
    connect.add_argument(
        "--api-url",
        default=os.environ.get("WADDLE_API_URL", "https://api.waddlelabs.ai"),
        help="hosted Waddle HTTP API used to derive the browser invitation",
    )
    connect.add_argument("--insecure", action="store_true", help=argparse.SUPPRESS)
    return parser


def _api_key() -> str:
    key = os.environ.get("WADDLE_API_KEY")
    if key is None:
        key = getpass.getpass("Waddle API key: ")
    if not key:
        raise SystemExit("WADDLE_API_KEY or a non-empty prompted API key is required")
    return key


def _connect(args: argparse.Namespace) -> int:
    site = load_site(args.site)
    api_key = _api_key()
    invitation_client = WaddleUiInvitationClient(
        UiInvitationConfig(
            api_url=args.api_url,
            api_key=api_key,
            workspace_id=site.id,
            allow_insecure=args.insecure,
        )
    )
    binding = invitation_client.resolve_binding()
    transport = Grpc(
        args.target,
        api_key,
        customer_id=binding.customer_id,
        project_id=binding.project_id,
        workspace_id=binding.workspace_id,
    )
    stop = threading.Event()

    def request_stop(_signum, _frame) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    with site.open(
        transport=transport,
        authorization_timeout_s=args.authorization_timeout,
    ):
        print(
            f"connected site {site.id!r} to "
            f"{binding.customer_id}/{binding.project_id}/{binding.workspace_id}",
            flush=True,
        )
        try:
            url = invitation_client.issue()
        except UiInvitationError as error:
            print(f"UI: unavailable ({error})", file=sys.stderr, flush=True)
        else:
            print(f"UI: {url}", flush=True)
        while not stop.wait(0.5):
            pass
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "connect":
        return _connect(args)
    raise AssertionError(f"unhandled command {args.command!r}")
