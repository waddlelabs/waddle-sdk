"""Hosted pairing and device launch, with optional local planning setup."""

import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

import httpx
from waddle_sdk_relay.errors import RelayError as ToolError

from .config import load_connection, load_workspace
from .connection import launch
from .dependencies import (
    HARDWARE,
    hardware_for_drivers,
    hardware_for_site,
    install_support,
)
from .setup import account_client, checked, configure, origin


def main(argv=None, *, prog="waddle-sdk connect"):
    parser = argparse.ArgumentParser(prog=prog)
    commands = parser.add_subparsers(dest="command", required=True)
    setup = commands.add_parser("setup", help="Configure and pair a rig with the app")
    setup.add_argument("--endpoint", default="https://app.waddlelabs.ai")
    setup.add_argument(
        "--key-file", type=Path, default=Path("~/.config/waddle/account.key")
    )
    setup.add_argument("--workspace-id")
    setup.add_argument("--directory", type=Path, default=Path("~/workspaces/my-rig"))
    setup.add_argument("--rig")
    setup.add_argument("--hardware", choices=tuple(HARDWARE), action="append")
    setup.add_argument("--device", choices=("linux", "linux-gpu", "jetson", "mac"))
    setup.add_argument("--execution", choices=("local", "cloud", "hybrid"))
    setup.add_argument("--planner-provider", choices=("auto", "curobo", "mujoco"))
    setup.add_argument("--site", type=Path)
    setup.add_argument("--simulation", action="store_true")
    setup.add_argument(
        "--no-start",
        action="store_true",
        help="Save configuration without opening hardware",
    )
    start = commands.add_parser("start", help="Start a previously configured rig")
    start.add_argument("--directory", type=Path, default=Path("~/workspaces/my-rig"))
    start.add_argument(
        "--release-torque-on-exit",
        action="store_true",
        help=(
            "Cloud/hybrid connection: confirm the arm is supported for torque release "
            "when this connection closes"
        ),
    )
    status = commands.add_parser(
        "check",
        aliases=["status"],
        help="Check saved configuration and app connection without opening hardware",
    )
    status.add_argument("--directory", type=Path, default=Path("~/workspaces/my-rig"))
    args = parser.parse_args(argv)
    try:
        directory = args.directory.expanduser().absolute()
        workspace = directory / "workspace.yaml"
        state_file = directory / ".metal" / "hosted.json"
        state = json.loads(state_file.read_text()) if state_file.exists() else {}
        if args.command == "start":
            saved = load_connection(workspace)
            if saved is None:
                raise ValueError("No saved app connection; rerun setup")
            return launch(
                workspace,
                workspace_id=state.get("workspace_id"),
                endpoint="https://"
                + saved.endpoint.removeprefix("wss://").removesuffix("/connect"),
                key_file=saved.key_file,
                release_torque_on_exit=args.release_torque_on_exit,
            )
        if args.command in {"check", "status"}:
            config = load_workspace(workspace)
            if state.get("execution") not in {"cloud", "hybrid"}:
                from waddle_metal.workspace.config import (
                    load_workspace as load_metal_workspace,
                )

                config = load_metal_workspace(workspace)
            saved = load_connection(workspace)
            if saved is None:
                raise ValueError("No saved app connection; rerun setup")
            endpoint = "https://" + saved.endpoint.removeprefix("wss://").removesuffix(
                "/connect"
            )
            with account_client(endpoint, saved.key_file) as client:
                account = checked(client.get("/api/v1/account"))
            rig = next((r for r in account["rigs"] if r["name"] == saved.rig), None)
            print(
                json.dumps(
                    {
                        "configuration": (
                            {
                                "workspace": str(workspace),
                                "site_id": config.site.id,
                                "execution": state["execution"],
                                "planner_validation": (
                                    "local execution and native validation; cloud search"
                                    if state["execution"] == "hybrid"
                                    else "on the cloud worker"
                                ),
                            }
                            if state.get("execution") in {"cloud", "hybrid"}
                            else config.describe()
                        ),
                        "rig": rig,
                        "note": "Hardware was not opened by this check.",
                    },
                    default=str,
                )
            )
            return 0
        args.execution = args.execution or state.get("execution") or "cloud"
        selected_planner = (
            None
            if args.execution == "cloud"
            else ("mujoco" if args.execution == "hybrid" else "auto")
        )
        if args.execution == "cloud" and args.planner_provider is not None:
            raise ValueError(
                "Cloud setup selects planning compute on the assigned server"
            )
        if args.execution == "hybrid" and args.planner_provider not in {None, "mujoco"}:
            raise ValueError(
                "Cloud setup prepares native models here and CUDA planning on the cloud GPU"
            )
        args.planner_provider = args.planner_provider or selected_planner
        if args.workspace_id is None and state_file.exists():
            args.workspace_id = json.loads(state_file.read_text()).get("workspace_id")
        args.endpoint = origin(args.endpoint)
        args.key_file = args.key_file.expanduser().absolute()
        with account_client(args.endpoint, args.key_file) as client:
            account = checked(client.get("/api/v1/account"))
        if state and state.get("execution", "local") != args.execution:
            saved = load_connection(workspace)
            rig = next(
                (r for r in account["rigs"] if saved and r["name"] == saved.rig), None
            )
            if rig and rig.get("connected"):
                raise ValueError(
                    "Stop the existing rig before changing where Metal runs"
                )
        if workspace.exists():
            if args.execution == "cloud":
                existing = load_workspace(workspace)
            else:
                from waddle_metal.workspace.config import (
                    load_workspace as load_metal_workspace,
                )

                existing = load_metal_workspace(workspace)
            if existing.site is None:
                raise ValueError("Hosted setup requires a configured SDK site")
            selections = getattr(existing, "planner_selections", {})
            planner = (
                "curobo"
                if any(row.get("provider") == "curobo" for row in selections.values())
                else (
                    "auto"
                    if any(row.get("provider") == "auto" for row in selections.values())
                    else "mujoco"
                )
            )
            install_support(
                hardware_for_site(existing.site.manifest),
                None
                if args.execution == "cloud"
                else ("mujoco" if args.execution == "hybrid" else planner),
                args.endpoint,
                args.key_file,
            )
        else:
            if args.execution != "cloud":
                from waddle_metal import cli as metal_cli

            hardware = args.hardware
            if args.simulation:
                hardware = sorted(set(hardware or []) | {"mujoco"})
            if args.site and hardware is None:
                from waddle_sdk import load_site

                source = args.site.expanduser()
                if source.is_dir():
                    source /= "site.yaml"
                hardware = hardware_for_site(load_site(source).manifest)
            install_support(
                hardware or [], args.planner_provider, args.endpoint, args.key_file
            )
            installed = set(hardware or [])

            def prepare_drivers(drivers):
                needed = set(hardware_for_drivers(drivers)) - installed
                if needed:
                    install_support(
                        sorted(installed | needed),
                        args.planner_provider,
                        args.endpoint,
                        args.key_file,
                        quiet=True,
                    )
                    installed.update(needed)

            if args.execution == "cloud":
                from .initialize import initialize as initialize_device

                built = initialize_device(
                    directory,
                    source_site=args.site,
                    simulation=args.simulation,
                    prepare_drivers=prepare_drivers,
                )
            else:
                from waddle_metal.initialization.cli import initialize

                command = [
                    "init",
                    str(directory),
                    "--host",
                    "none",
                    "--planner-provider",
                    args.planner_provider,
                ]
                if args.simulation:
                    command.append("--simulation")
                elif args.site:
                    command += ["--site", str(args.site.expanduser())]
                with TemporaryDirectory(prefix="waddle-setup-") as options_dir:
                    options_file = Path(options_dir) / "hosted.json"
                    options_file.write_text(
                        json.dumps(
                            {
                                "endpoint": args.endpoint,
                                "key_file": str(args.key_file),
                                **({"rig": args.rig} if args.rig else {}),
                                "workspace_id": args.workspace_id,
                                "execution": args.execution,
                            }
                        )
                    )
                    command[command.index("none")] = "hosted"
                    command += ["--host-options", str(options_file)]
                    built = initialize(
                        metal_cli.argument_parser().parse_args(command),
                        prepare_drivers=prepare_drivers,
                    )
            if built is None:
                return 0  # wizard cancelled
            workspace = Path(built["workspace"])
            directory = workspace.parent
        result = configure(
            workspace,
            options={
                "endpoint": args.endpoint,
                "key_file": str(args.key_file),
                **({"rig": args.rig} if args.rig else {}),
                "workspace_id": args.workspace_id,
                "execution": args.execution,
            },
        )
        print(
            "Saved app connection. Restart with: "
            + shlex.join(
                [
                    str(Path(sys.executable).with_name("waddle-connect")),
                    "start",
                    "--directory",
                    str(directory),
                ]
            )
        )
        if args.no_start:
            return 0
        if not sys.stdin.isatty() or input(
            "Start the rig now? This enables its configured hardware. [y/N] "
        ).strip().lower() not in {"y", "yes"}:
            return 0
        return launch(
            workspace,
            workspace_id=result["workspace_id"],
            endpoint=args.endpoint,
            key_file=args.key_file,
        )
    except ModuleNotFoundError as error:
        if error.name != "waddle_metal" and not (error.name or "").startswith(
            "waddle_metal."
        ):
            raise
        print(
            "waddle-connect: local/hybrid execution requires the optional "
            "waddle-connect[local] package. The base connector supports cloud setup.",
            file=sys.stderr,
        )
        return 2
    except ToolError as error:
        print(f"waddle-connect: {error.code}: {error.detail}", file=sys.stderr)
        return 2
    except httpx.HTTPError:
        print(
            "waddle-connect: could not reach the app; check the connection and rerun setup",
            file=sys.stderr,
        )
        return 2
    except (ValueError, OSError) as error:
        print(f"waddle-connect: {error}", file=sys.stderr)
        return 2
    except subprocess.CalledProcessError:
        print(
            "waddle-connect: installation failed; fix the error above and rerun setup",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
