"""Shared camera-factory loading for site and inspection lifecycles."""

from __future__ import annotations

import importlib
import inspect
from collections.abc import Callable

from .base import CameraDriver
from .site import CameraConfig


class CameraFactoryError(ValueError):
    """A camera driver reference or factory does not satisfy the SDK contract."""


def resolve_camera_factory(spec: str) -> Callable[..., object]:
    """Resolve one lazy ``module[:attribute]`` camera driver reference."""

    if ":" in spec:
        module_name, attribute = spec.split(":", 1)
    else:
        module_name = spec
        leaf = module_name.rsplit(".", 1)[-1]
        attribute = "".join(piece.capitalize() for piece in leaf.split("_")) + "Driver"
    if not module_name or not attribute:
        raise CameraFactoryError(f"driver {spec!r} must name module:attribute")
    try:
        module = importlib.import_module(module_name)
        try:
            target = getattr(module, attribute)
        except AttributeError:
            candidates = [
                getattr(module, name)
                for name in getattr(module, "__all__", ())
                if name.endswith("Driver") and callable(getattr(module, name, None))
            ]
            if len(candidates) != 1:
                raise
            target = candidates[0]
    except (ImportError, AttributeError) as exc:
        raise CameraFactoryError(
            f"cannot load camera driver ({type(exc).__name__})"
        ) from exc
    if not callable(target):
        raise CameraFactoryError(f"driver {spec!r} is not callable")
    return target


def open_camera_driver(
    target: Callable[..., object], config: CameraConfig
) -> CameraDriver:
    """Call one current or legacy camera factory and check its result."""

    parameters = inspect.signature(target).parameters
    if "config" in parameters:
        result = target(config=config)
    else:
        kwargs = {
            **dict(config.options),
            **dict(config.connection),
            "width": int(config.stream["width"]),
            "height": int(config.stream["height"]),
            "fps": config.stream["fps"],
        }
        try:
            inspect.signature(target).bind(**kwargs)
        except TypeError as exc:
            raise CameraFactoryError(
                f"camera {config.name!r} driver arguments do not match its factory"
            ) from exc
        result = target(**kwargs)
    if not isinstance(result, CameraDriver):
        raise CameraFactoryError(
            f"camera {config.name!r} driver must provide capture() and close()"
        )
    return result


__all__ = ["CameraFactoryError", "open_camera_driver", "resolve_camera_factory"]
