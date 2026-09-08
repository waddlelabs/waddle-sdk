"""Customer-side SDK connector command."""

from __future__ import annotations

import argparse
import getpass
import json
import os
import signal
import sys
import threading
from collections.abc import Sequence

from . import Grpc, __version__, load_site
from ._hosted_ui import (
    UiInvitationConfig,
    UiInvitationError,
    WaddleUiInvitationClient,
)
from .agent_skills import bundled_skills, export_skill
from .scene import (
    SceneError,
    initialize_scene,
    load_scene,
    simulation_compiler_names,
)
from .simulation import simulation_backend_names


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

    skills = commands.add_parser(
        "skills",
        help="inspect or export version-matched coding-agent skills",
    )
    skill_commands = skills.add_subparsers(dest="skills_command", required=True)
    list_skills = skill_commands.add_parser(
        "list",
        help="list skills bundled with this SDK",
    )
    list_skills.add_argument("--json", action="store_true", dest="as_json")
    export = skill_commands.add_parser(
        "export",
        help="copy one portable skill into a chosen directory",
    )
    export.add_argument("name")
    export.add_argument(
        "--output",
        required=True,
        help="parent directory that will receive a new <skill-name> folder",
    )

    sim = commands.add_parser(
        "sim",
        help="validate, compile, or smoke-test a portable simulation scene",
    )
    sim_commands = sim.add_subparsers(dest="sim_command", required=True)
    backends = sim_commands.add_parser(
        "backends",
        help="list installed simulation runtimes and scene compilers",
    )
    backends.add_argument("--json", action="store_true", dest="as_json")
    initialize = sim_commands.add_parser(
        "init",
        help="create an editable portable scene bundle from one URDF",
    )
    initialize.add_argument("urdf")
    initialize.add_argument("--output", required=True)
    initialize.add_argument("--part", default="arm")
    initialize.add_argument("--tool-link")
    initialize.add_argument(
        "--color",
        nargs=4,
        type=float,
        metavar=("R", "G", "B", "A"),
        default=(0.9, 0.55, 0.1, 1.0),
    )
    initialize.add_argument(
        "--package",
        action="append",
        default=[],
        metavar="NAME=PATH",
        help="resolve one package://NAME asset root while copying the URDF",
    )
    validate = sim_commands.add_parser(
        "validate",
        help="validate a portable scene without importing a simulator",
    )
    validate.add_argument("scene")
    validate.add_argument("--json", action="store_true", dest="as_json")
    compile_command = sim_commands.add_parser(
        "compile",
        help="compile a portable scene into a new backend-native bundle",
    )
    compile_command.add_argument("scene")
    compile_command.add_argument("--backend", default="mujoco")
    compile_command.add_argument("--output", required=True)
    compile_command.add_argument("--seed", type=int)
    run = sim_commands.add_parser(
        "run",
        help="compile and locally open a portable scene once",
    )
    run.add_argument("scene")
    run.add_argument("--backend", default="mujoco")
    run.add_argument("--output", required=True)
    run.add_argument("--seed", type=int)
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


def _skills(args: argparse.Namespace) -> int:
    skills = bundled_skills()
    if args.skills_command == "list":
        if args.as_json:
            print(
                json.dumps(
                    {
                        "sdk_version": __version__,
                        "skills": [
                            {"name": skill.name, "description": skill.description}
                            for skill in skills
                        ],
                    },
                    sort_keys=True,
                )
            )
        else:
            print(f"waddle-sdk {__version__}")
            for skill in skills:
                print(f"{skill.name}\t{skill.description}")
        return 0
    if args.skills_command == "export":
        try:
            target = export_skill(args.name, args.output)
        except (FileExistsError, OSError, RuntimeError, ValueError) as error:
            raise SystemExit(str(error)) from error
        print(f"exported {args.name} from waddle-sdk {__version__} to {target}")
        return 0
    raise AssertionError(f"unhandled skills command {args.skills_command!r}")


def _sim(args: argparse.Namespace) -> int:
    try:
        if args.sim_command == "backends":
            runtimes = set(simulation_backend_names())
            compilers = set(simulation_compiler_names())
            rows = [
                {
                    "name": name,
                    "runtime": name in runtimes,
                    "compiler": name in compilers,
                }
                for name in sorted(runtimes | compilers)
            ]
            if args.as_json:
                print(json.dumps({"backends": rows}, sort_keys=True))
            else:
                for row in rows:
                    capabilities = ", ".join(
                        name
                        for name in ("runtime", "compiler")
                        if row[name]
                    )
                    print(f"{row['name']}\t{capabilities}")
            return 0
        if args.sim_command == "init":
            packages = {}
            for value in args.package:
                name, separator, path = value.partition("=")
                if not separator or not name or not path or name in packages:
                    raise SceneError(
                        "--package must be a unique non-empty NAME=PATH mapping"
                    )
                packages[name] = path
            scene_path = initialize_scene(
                args.urdf,
                output_dir=args.output,
                part_name=args.part,
                tool_link=args.tool_link,
                rgba=args.color,
                packages=packages,
            )
            print(f"scene: {scene_path}")
            print(
                f"next: waddle-sdk sim compile {scene_path} --backend mujoco "
                f"--output {scene_path.parent / 'build'}"
            )
            return 0
        scene = load_scene(args.scene)
        if args.sim_command == "validate":
            payload = {
                "api_version": scene.manifest["api_version"],
                "scene_id": scene.id,
                "robots": sorted(scene.manifest["robots"]),
                "cameras": sorted(scene.manifest.get("cameras", {})),
                "lights": sorted(scene.manifest.get("lights", {})),
            }
            if args.as_json:
                print(json.dumps(payload, sort_keys=True))
            else:
                print(
                    f"valid portable scene {scene.id!r}: "
                    f"{len(payload['robots'])} robot(s), "
                    f"{len(payload['cameras'])} camera(s), "
                    f"{len(payload['lights'])} light(s)"
                )
            return 0
        if args.sim_command in {"compile", "run"}:
            build = scene.compile(
                backend=args.backend,
                output_dir=args.output,
                seed=args.seed,
            )
            print(f"site: {build.site_path}")
            print(f"world: {build.world_path}")
            print(f"evidence: {build.evidence_path}")
            if args.sim_command == "run":
                site = load_site(build.site_path)
                with site.open(console=False) as session:
                    observation = session.observe()
                    summary = {
                        "site_id": site.id,
                        "parts": {
                            name: len(part.joint_position)
                            for name, part in observation.parts.items()
                        },
                        "cameras_ready": sorted(observation.cameras),
                    }
                    print(json.dumps(summary, sort_keys=True))
            return 0
    except (FileExistsError, OSError, RuntimeError, SceneError) as error:
        raise SystemExit(str(error)) from error
    raise AssertionError(f"unhandled sim command {args.sim_command!r}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "connect":
        return _connect(args)
    if args.command == "skills":
        return _skills(args)
    if args.command == "sim":
        return _sim(args)
    raise AssertionError(f"unhandled command {args.command!r}")
