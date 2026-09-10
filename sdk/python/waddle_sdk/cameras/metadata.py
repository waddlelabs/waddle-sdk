"""Non-opening normalization of public camera descriptions and intrinsics."""

import math
from collections.abc import Mapping

from ..descriptors import Intrinsics


def _finite(value):
    try:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )
    except OverflowError:
        return False


def camera_declarations(description: Mapping) -> dict[str, dict]:
    """Return effective named declarations, retaining incomplete optional metadata.

    Open runtime declarations take precedence over configuration. Site opening
    already applies explicit site intrinsics over adapter-provided intrinsics.
    Mounts come only from explicit configuration; no relationships are inferred.
    """
    live = {row["name"]: row for row in description.get("robot", {}).get("cameras", [])}
    configured = description.get("cameras", {})
    rows = {}
    for name in sorted(set(live) | set(configured)):
        actual, config = live.get(name, {}), configured.get(name, {})
        intrinsics = actual.get("intrinsics", config.get("intrinsics"))
        if isinstance(intrinsics, dict):
            intrinsics = {
                key: intrinsics[key]
                for key in (
                    "fx",
                    "fy",
                    "cx",
                    "cy",
                    "distortion",
                    "model",
                    "distortionModel",
                    "distortion_model",
                    "depthScaleMm",
                    "depth_scale_mm",
                )
                if key in intrinsics
            }
        rows[name] = {
            "frame_id": actual.get("frameId", config.get("frame_id")),
            "mount": config.get("mount"),
            "intrinsics": intrinsics,
            "width": actual.get("width", config.get("stream", {}).get("width")),
            "height": actual.get("height", config.get("stream", {}).get("height")),
            "fps": actual.get("fps", config.get("stream", {}).get("fps")),
        }
    return rows


class CameraMetadataError(ValueError):
    """Invalid or missing optional intrinsics, with a stable machine-readable code."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


def camera_intrinsics(raw: object, *, require_depth_scale: bool = False) -> Intrinsics:
    """Parse manifest or wire intrinsics without guessing a depth scale.

    RGB-only intrinsics remain valid. Metric depth consumers explicitly require
    a finite positive scale. Distortion model identity is retained, including the
    protobuf ``model`` spelling; this function never rectifies an image.
    """
    if not isinstance(raw, Mapping):
        raise CameraMetadataError(
            "intrinsics_missing", "Camera intrinsics are not declared"
        )
    values = [raw.get(key) for key in ("fx", "fy", "cx", "cy")]
    scale = raw.get("depthScaleMm", raw.get("depth_scale_mm"))
    checked = values + ([scale] if require_depth_scale or scale is not None else [])
    if (
        any(not _finite(v) for v in checked)
        or values[0] <= 0
        or values[1] <= 0
        or (scale is not None and scale <= 0)
    ):
        raise CameraMetadataError(
            "intrinsics_invalid",
            "Camera requires finite focal lengths, principal point and positive depth scale when used",
        )
    distortion = raw.get("distortion", ())
    if (
        not isinstance(distortion, (list, tuple))
        or len(distortion) > 16
        or any(not _finite(v) for v in distortion)
    ):
        raise CameraMetadataError(
            "intrinsics_invalid", "Camera distortion coefficients are invalid"
        )
    model = raw.get(
        "model", raw.get("distortionModel", raw.get("distortion_model", "unspecified"))
    )
    try:
        result = Intrinsics(
            *values,
            depth_scale_mm=scale,
            distortion=tuple(distortion),
            distortion_model=model,
        )
        result._compile()  # Validate the public descriptor's enum spelling too.
    except (TypeError, ValueError, AttributeError) as error:
        raise CameraMetadataError(
            "intrinsics_invalid", "Camera distortion model is invalid"
        ) from error
    return result
