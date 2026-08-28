#!/usr/bin/env python3
"""Create a fail-closed external Waddle adapter project without opening hardware."""

from __future__ import annotations

import argparse
import keyword
import math
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class JointFact:
    name: str
    lower: float
    upper: float
    step_cap: float


def _finite(raw: str, label: str) -> float:
    try:
        value = float(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{label} must be a number") from error
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"{label} must be finite")
    return value


def _joint(raw: str) -> JointFact:
    parts = raw.split(":")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("joint must be NAME:LOWER:UPPER:STEP_CAP")
    name = parts[0]
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
        raise argparse.ArgumentTypeError(f"invalid joint name {name!r}")
    lower = _finite(parts[1], f"{name} lower")
    upper = _finite(parts[2], f"{name} upper")
    cap = _finite(parts[3], f"{name} step cap")
    if lower >= upper:
        raise argparse.ArgumentTypeError(f"{name} lower must be less than upper")
    if cap <= 0.0 or cap > upper - lower:
        raise argparse.ArgumentTypeError(
            f"{name} step cap must be positive and no wider than its range"
        )
    return JointFact(name, lower, upper, cap)


def _positive(raw: str) -> float:
    value = _finite(raw, "value")
    if value <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def _nonempty(raw: str) -> str:
    if not raw.strip():
        raise argparse.ArgumentTypeError("value must be non-empty")
    return raw


def _package_name(name: str) -> str:
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", name):
        raise argparse.ArgumentTypeError(
            "name must start with a letter and contain only letters, digits, '-' or '_'"
        )
    package = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    if not package or package[0].isdigit() or keyword.iskeyword(package):
        raise argparse.ArgumentTypeError(
            "name must produce a Python package beginning with a letter"
        )
    return package


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="kind", required=True)
    for kind in ("robot", "simulator"):
        command = commands.add_parser(kind)
        command.add_argument("--name", required=True)
        command.add_argument("--output", required=True, type=Path)
        command.add_argument("--facts-source", required=True, type=_nonempty)
        command.add_argument("--joint", required=True, action="append", type=_joint)
        command.add_argument("--rate-hz", required=True, type=_positive)
        if kind == "simulator":
            command.add_argument("--home", required=True)
    camera = commands.add_parser("camera")
    camera.add_argument("--name", required=True)
    camera.add_argument("--output", required=True, type=Path)
    camera.add_argument("--facts-source", required=True, type=_nonempty)
    camera.add_argument("--width", required=True, type=int)
    camera.add_argument("--height", required=True, type=int)
    camera.add_argument("--fps", required=True, type=_positive)
    return parser


def _sdk_version() -> str:
    try:
        from waddle_sdk import __version__
    except (ImportError, RuntimeError) as error:
        raise SystemExit(
            "cannot determine the SDK version; run this skill with waddle-sdk installed"
        ) from error
    return __version__


def _replacements(args: argparse.Namespace) -> dict[str, str]:
    try:
        package = _package_name(args.name)
    except argparse.ArgumentTypeError as error:
        raise SystemExit(str(error)) from error
    values = {
        "__DIST_NAME__": args.name,
        "__PACKAGE_NAME__": package,
        "__SDK_VERSION__": _sdk_version(),
        "__FACTS_SOURCE__": repr(args.facts_source),
    }
    if args.kind in {"robot", "simulator"}:
        joints: list[JointFact] = args.joint
        if len({joint.name for joint in joints}) != len(joints):
            raise SystemExit("joint names must be unique")
        values.update(
            {
                "__JOINTS__": repr(tuple(joint.name for joint in joints)),
                "__LIMITS__": repr(tuple((j.lower, j.upper) for j in joints)),
                "__STEP_CAPS__": repr(tuple(j.step_cap for j in joints)),
                "__RATE_HZ__": repr(args.rate_hz),
                "__JOINT_LIMITS_YAML__": "\n".join(
                    f"      {j.name}: [{j.lower!r}, {j.upper!r}]" for j in joints
                ),
            }
        )
        if args.kind == "simulator":
            home = tuple(
                _finite(item.strip(), "home") for item in args.home.split(",")
            )
            if len(home) != len(joints):
                raise SystemExit(
                    "home must contain one comma-separated value per joint"
                )
            for joint, value in zip(joints, home, strict=True):
                if not joint.lower <= value <= joint.upper:
                    raise SystemExit(f"home for {joint.name!r} is outside its limits")
            values["__HOME__"] = repr(home)
    else:
        if args.width <= 0 or args.height <= 0:
            raise SystemExit("camera width and height must be positive")
        values.update(
            {
                "__WIDTH__": str(args.width),
                "__HEIGHT__": str(args.height),
                "__FPS__": repr(args.fps),
            }
        )
    return values


def _selected(name: str, kind: str) -> str | None:
    for candidate in ("robot", "simulator", "camera"):
        suffix = f".{candidate}.tmpl"
        if name.endswith(suffix):
            return name[: -len(suffix)] if candidate == kind else None
    return name[:-5] if name.endswith(".tmpl") else name


def main() -> int:
    args = _parser().parse_args()
    replacements = _replacements(args)
    target = args.output.expanduser() / args.name
    if target.exists() or target.is_symlink():
        raise SystemExit(f"refusing to overwrite existing target: {target}")
    target.mkdir(mode=0o755, parents=True)

    template = Path(__file__).resolve().parents[1] / "assets" / "python-adapter"
    for source in sorted(template.rglob("*")):
        if not source.is_file():
            continue
        relative_parts = [
            replacements["__PACKAGE_NAME__"] if part == "package_name" else part
            for part in source.relative_to(template).parts
        ]
        selected = _selected(relative_parts[-1], args.kind)
        if selected is None:
            continue
        relative_parts[-1] = selected
        destination = target.joinpath(*relative_parts)
        destination.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        text = source.read_text(encoding="utf-8")
        for needle, replacement in replacements.items():
            text = text.replace(needle, replacement)
        destination.write_text(text, encoding="utf-8")
        destination.chmod(0o644)
    print(f"created {args.kind} adapter scaffold at {target}")
    print("no adapter module was imported and no hardware was opened")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
