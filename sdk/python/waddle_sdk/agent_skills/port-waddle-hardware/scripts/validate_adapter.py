#!/usr/bin/env python3
"""Conservatively screen a Waddle adapter without importing its modules.

Static analysis is incomplete and does not prove import or runtime safety.
"""

from __future__ import annotations

import argparse
import ast
import keyword
from collections.abc import Mapping
from pathlib import Path

class ValidationError(Exception):
    pass


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    parser.add_argument("--site", required=True, type=Path)
    parser.add_argument("--source-root", default="src", type=Path)
    return parser


def _module_path(source_root: Path, module: str) -> Path:
    segments = module.split(".")
    if not segments or any(
        not segment.isidentifier() or keyword.iskeyword(segment)
        for segment in segments
    ):
        raise ValidationError(
            f"adapter module {module!r} must contain only strict dotted Python "
            "identifiers"
        )
    root = source_root.expanduser().resolve()
    if not root.is_dir():
        raise ValidationError(f"adapter source root is not a directory: {root}")
    candidate = root.joinpath(*segments).resolve(strict=False)
    if not candidate.is_relative_to(root):
        raise ValidationError(f"adapter module {module!r} escapes source root {root}")
    module_file = candidate.with_suffix(".py")
    package_file = candidate / "__init__.py"
    for path in (module_file, package_file):
        if path.is_file():
            resolved = path.resolve()
            if not resolved.is_relative_to(root):
                raise ValidationError(
                    f"adapter module {module!r} escapes source root {root}"
                )
            return resolved
    raise ValidationError(f"cannot find local source for adapter module {module!r}")


class _ImportTimeCallVisitor(ast.NodeVisitor):
    """Find call expressions and call-like declarations evaluated at import."""

    _SAFE_DECORATORS = frozenset({"classmethod", "property", "staticmethod"})
    _SAFE_BASES = frozenset({"ABC", "Protocol"})

    def __init__(self) -> None:
        self.lines: set[int] = set()

    def visit_Call(self, node: ast.Call) -> None:  # noqa: N802 - ast visitor API
        self.lines.add(node.lineno)
        self.generic_visit(node)

    def visit_Lambda(self, node: ast.Lambda) -> None:  # noqa: N802
        # Creating a lambda evaluates no body. Calls around the lambda are
        # visited by its parent expression.
        del node

    def _visit_decorator(self, decorator: ast.expr) -> None:
        # Applying a decorator is itself a call even when its expression is a
        # bare name. Only the built-in descriptor decorators used by ordinary
        # structural drivers are admitted without a finding.
        if isinstance(decorator, ast.Name) and decorator.id in self._SAFE_DECORATORS:
            return
        self.lines.add(decorator.lineno)
        self.visit(decorator)

    def _visit_function_header(
        self, node: ast.FunctionDef | ast.AsyncFunctionDef
    ) -> None:
        # Defining a function evaluates decorators, defaults and annotations;
        # its body remains deferred until somebody calls it.
        for decorator in node.decorator_list:
            self._visit_decorator(decorator)
        for argument in (
            *node.args.posonlyargs,
            *node.args.args,
            *node.args.kwonlyargs,
        ):
            if argument.annotation is not None:
                self.visit(argument.annotation)
        if node.args.vararg is not None and node.args.vararg.annotation is not None:
            self.visit(node.args.vararg.annotation)
        if node.args.kwarg is not None and node.args.kwarg.annotation is not None:
            self.visit(node.args.kwarg.annotation)
        for default in (*node.args.defaults, *node.args.kw_defaults):
            if default is not None:
                self.visit(default)
        if node.returns is not None:
            self.visit(node.returns)
        for type_parameter in getattr(node, "type_params", ()):
            self.visit(type_parameter)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:  # noqa: N802
        self._visit_function_header(node)

    def visit_AsyncFunctionDef(  # noqa: N802
        self, node: ast.AsyncFunctionDef
    ) -> None:
        self._visit_function_header(node)

    def visit_ClassDef(self, node: ast.ClassDef) -> None:  # noqa: N802
        # Bases, metaclass keywords and decorators are evaluated before class
        # construction. The class body itself then executes as an import-time
        # scope, including nested control flow and method declaration headers.
        for decorator in node.decorator_list:
            self._visit_decorator(decorator)
        for base in node.bases:
            if not (
                isinstance(base, ast.Name) and base.id in self._SAFE_BASES
            ):
                # Class construction can invoke a base's __init_subclass__
                # even when the base expression contains no explicit Call.
                self.lines.add(base.lineno)
            self.visit(base)
        for class_keyword in node.keywords:
            if class_keyword.arg == "metaclass":
                # The selected metaclass is called to construct the class.
                self.lines.add(class_keyword.value.lineno)
            self.visit(class_keyword.value)
        for type_parameter in getattr(node, "type_params", ()):
            self.visit(type_parameter)
        for statement in node.body:
            self.visit(statement)


def _module_scope_calls(tree: ast.Module) -> list[int]:
    visitor = _ImportTimeCallVisitor()
    visitor.visit(tree)
    return sorted(visitor.lines)


def _factory(tree: ast.Module, attribute: str, path: Path) -> ast.FunctionDef:
    functions = {
        node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    function = functions.get(attribute)
    if function is None:
        raise ValidationError(f"{path}: no top-level factory {attribute!r}")
    parameters = {
        argument.arg for argument in (*function.args.args, *function.args.kwonlyargs)
    }
    if "config" not in parameters:
        raise ValidationError(f"{path}:{function.lineno}: factory must accept config")
    return function


def _driver_targets(document: object) -> list[tuple[str, str]]:
    if not isinstance(document, Mapping):
        raise ValidationError("site manifest must be a mapping")
    targets: list[tuple[str, str]] = []
    for section, kind in (("parts", "robot"), ("cameras", "camera")):
        rows = document.get(section, {})
        if not isinstance(rows, Mapping):
            raise ValidationError(f"site {section} must be a mapping")
        for name, row in rows.items():
            if not isinstance(row, Mapping) or not isinstance(row.get("driver"), str):
                raise ValidationError(f"site {section}.{name} must declare driver")
            targets.append((kind, row["driver"]))
    return targets


def _validate_site_schema(site: Path) -> object:
    try:
        from waddle_sdk import load_site
    except ImportError as error:
        raise ValidationError("install waddle-sdk to validate its site schema") from error
    try:
        loaded = load_site(site)
    except Exception as error:
        raise ValidationError(f"site validation failed: {error}") from error
    return loaded.describe()


def main() -> int:
    args = _parser().parse_args()
    project = args.project.expanduser().resolve()
    site = args.site if args.site.is_absolute() else project / args.site
    source_root = (
        args.source_root
        if args.source_root.is_absolute()
        else project / args.source_root
    )
    document = _validate_site_schema(site)

    inspected = 0
    try:
        for kind, target in _driver_targets(document):
            if target.startswith("waddle_sdk."):
                continue
            module, separator, attribute = target.partition(":")
            if not separator or not module or not attribute:
                raise ValidationError(
                    f"custom driver {target!r} must use explicit module:factory form"
                )
            path = _module_path(source_root, module)
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            calls = _module_scope_calls(tree)
            if calls:
                joined = ", ".join(str(line) for line in calls)
                raise ValidationError(
                    f"{path}: module-scope calls at lines {joined}; keep adapter import "
                    "declarative and move opening work into the lifecycle"
                )
            function = _factory(tree, attribute, path)
            called_names = {
                node.func.id
                for node in ast.walk(function)
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            } | {
                node.func.attr
                for node in ast.walk(function)
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            }
            if kind == "robot" and "Rig" not in called_names:
                raise ValidationError(
                    f"{path}:{function.lineno}: robot factory must construct a Rig"
                )
            if kind == "camera":
                classes = {
                    node.name: {
                        member.name
                        for member in node.body
                        if isinstance(member, ast.FunctionDef)
                    }
                    for node in tree.body
                    if isinstance(node, ast.ClassDef)
                }
                if not any(
                    {"capture", "close"} <= methods for methods in classes.values()
                ):
                    raise ValidationError(
                        f"{path}: camera adapter must define capture() and close()"
                    )
            inspected += 1
    except (OSError, SyntaxError, ValidationError) as error:
        raise SystemExit(f"adapter validation failed: {error}") from error

    if inspected == 0:
        raise SystemExit("adapter validation failed: site names no local custom adapter")
    print(f"validated {inspected} custom adapter factory/factories against {site}")
    print(
        "validation was static and incomplete: adapter modules were not imported, "
        "hardware was not opened, and runtime safety was not proven"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
