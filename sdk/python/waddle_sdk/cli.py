"""Customer-side SDK connector command."""

from __future__ import annotations

import argparse
import getpass
import os
import signal
import threading
from collections.abc import Sequence

from . import Grpc, load_site


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="waddle-sdk")
    commands = parser.add_subparsers(dest="command", required=True)
    connect = commands.add_parser(
        "connect",
        help="connect one site to its authorized hosted workspace",
    )
    connect.add_argument("--site", required=True)
    connect.add_argument("--customer", required=True)
    connect.add_argument("--project", required=True)
    connect.add_argument("--workspace", required=True)
    connect.add_argument(
        "--target",
        default=os.environ.get(
            "WADDLE_CONNECTOR_TARGET", "https://api.waddlelabs.ai:443"
        ),
        help="hosted waddle.v0 endpoint",
    )
    connect.add_argument(
        "--authorization-timeout",
        type=float,
        default=15.0,
        help="seconds to authenticate before refusing to open hardware",
    )
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
    transport = Grpc(
        args.target,
        _api_key(),
        customer_id=args.customer,
        project_id=args.project,
        workspace_id=args.workspace,
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
            f"{args.customer}/{args.project}/{args.workspace}",
            flush=True,
        )
        while not stop.wait(0.5):
            pass
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "connect":
        return _connect(args)
    raise AssertionError(f"unhandled command {args.command!r}")
