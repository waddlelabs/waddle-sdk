"""Portable, version-matched skill resources bundled with ``waddle-sdk``."""

from __future__ import annotations

import ast
import os
import stat
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class BundledSkill:
    """One validated skill available from the installed SDK wheel."""

    name: str
    description: str
    resource: Any


def _frontmatter(skill_md: Any) -> dict[str, object]:
    text = skill_md.read_text(encoding="utf-8")
    if not text.startswith("---\n"):
        raise RuntimeError(f"bundled skill {skill_md} has no YAML frontmatter")
    try:
        raw = text.split("---\n", 2)[1]
    except IndexError as error:
        raise RuntimeError(f"bundled skill {skill_md} has invalid frontmatter") from error
    parsed: dict[str, object] = {}
    for line in raw.splitlines():
        key, separator, value = line.partition(":")
        if not separator or key not in {"name", "description"}:
            continue
        scalar = value.strip()
        if scalar.startswith(("'", '"')):
            try:
                scalar = ast.literal_eval(scalar)
            except (SyntaxError, ValueError) as error:
                raise RuntimeError(
                    f"bundled skill {skill_md} has invalid {key} frontmatter"
                ) from error
        if not isinstance(scalar, str) or not scalar:
            raise RuntimeError(
                f"bundled skill {skill_md} has invalid {key} frontmatter"
            )
        parsed[key] = scalar
    return parsed


def bundled_skills() -> tuple[BundledSkill, ...]:
    """Return every skill shipped by this exact installed SDK."""

    root = resources.files(__package__)
    found: list[BundledSkill] = []
    for candidate in root.iterdir():
        if not candidate.is_dir():
            continue
        skill_md = candidate.joinpath("SKILL.md")
        if not skill_md.is_file():
            continue
        metadata = _frontmatter(skill_md)
        name = metadata.get("name")
        description = metadata.get("description")
        if name != candidate.name or not isinstance(description, str) or not description:
            raise RuntimeError(
                f"bundled skill directory {candidate.name!r} disagrees with SKILL.md"
            )
        found.append(BundledSkill(name, description, candidate))
    return tuple(sorted(found, key=lambda skill: skill.name))


def _source_mode(source: Any, relative: tuple[str, ...]) -> int:
    try:
        return stat.S_IMODE(os.stat(source).st_mode)
    except (OSError, TypeError, ValueError):
        if relative and relative[0] == "scripts":
            return 0o755
        return 0o644


def _copy_resource_tree(source: Any, destination: Path, relative: tuple[str, ...]) -> None:
    destination.mkdir(mode=0o755)
    for child in source.iterdir():
        child_relative = (*relative, child.name)
        child_destination = destination / child.name
        if child.is_dir():
            _copy_resource_tree(child, child_destination, child_relative)
            continue
        if not child.is_file():
            raise RuntimeError(f"unsupported bundled skill resource {child}")
        with child.open("rb") as input_stream, child_destination.open("xb") as output:
            while block := input_stream.read(1024 * 1024):
                output.write(block)
        child_destination.chmod(_source_mode(child, child_relative))


def export_skill(name: str, output: str | os.PathLike[str]) -> Path:
    """Copy one bundled skill beneath *output* without replacing any target."""

    selected = {skill.name: skill for skill in bundled_skills()}.get(name)
    if selected is None:
        available = ", ".join(skill.name for skill in bundled_skills())
        raise ValueError(f"unknown bundled skill {name!r}; available: {available}")

    parent = Path(output).expanduser()
    parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    target = parent / selected.name
    if target.exists() or target.is_symlink():
        raise FileExistsError(f"refusing to overwrite existing skill target: {target}")
    _copy_resource_tree(selected.resource, target, ())
    return target


__all__ = ["BundledSkill", "bundled_skills", "export_skill"]
