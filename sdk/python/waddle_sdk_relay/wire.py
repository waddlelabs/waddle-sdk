"""Bounded copies of public SDK values; no pickle, imports or executable payloads."""

import enum
import math
import zlib
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
from types import SimpleNamespace

import msgpack
import numpy as np
from waddle_sdk import runtime
from waddle_sdk.cameras import CameraCaptureState, CameraSample

from .errors import RelayError

MAX_BYTES = 4 * 1024 * 1024
MAX_ARRAY_BYTES = 16 * 1024 * 1024
_TYPES = {
    cls.__name__: cls
    for cls in (
        runtime.Observation,
        runtime.PartObservation,
        runtime.GripperForce,
        runtime.JointPositionCommand,
        runtime.SubmitResult,
        runtime.RuntimeFault,
        runtime.RuntimeFaultCause,
        runtime.RuntimeEvent,
        runtime.Pose,
        runtime.BodySphere,
        runtime.SupportMatrix,
        runtime.SupportRow,
        CameraSample,
        CameraCaptureState,
    )
}
# Older installed SDKs retain their existing ports. Only builds exposing the
# optional execution facet can encode/decode its finite values.
for _name in (
    "FiniteJointMotion",
    "CoordinatedJointMotion",
    "JointReference",
    "PlaybackPolicy",
    "PacedJointCurve",
    "PlaybackCurveUpdate",
    "HeldJointGroup",
    "PlaybackFeedbackPolicy",
    "PositionWindow",
    "GripperPlaybackPolicy",
    "GripperContactPolicy",
    "GripperForcePolicy",
):
    if (_type := getattr(runtime, _name, None)) is not None:
        _TYPES[_name] = _type
_DTYPES = {"|u1", "<u2", "<f8", "<f4"}


def _default(value):
    if isinstance(value, enum.Enum):
        return value.value
    if isinstance(value, np.ndarray):
        if value.dtype.str not in _DTYPES or value.nbytes > MAX_ARRAY_BYTES:
            raise ValueError("SDK array type or size exceeds the bridge contract")
        payload = (value.dtype.str, value.shape, zlib.compress(value.tobytes(), 1))
        return msgpack.ExtType(1, msgpack.packb(payload, use_bin_type=True))
    if isinstance(value, Mapping):
        return dict(value)
    if isinstance(value, CameraSample):
        data = {
            field.name: getattr(value, field.name)
            for field in fields(value)
            if field.name not in {"point_resolver", "content_timing"}
        }
        # Host monotonic camera clocks do not transfer to another host. Paired
        # session/Unix clocks, source pixels and acquisition sequence stay exact.
        data["remote_point_resolver"] = value.point_resolver is not None
        return msgpack.ExtType(2, encode(("CameraSample", data)))
    if is_dataclass(value) and type(value).__name__ in _TYPES:
        data = {field.name: getattr(value, field.name) for field in fields(value)}
        if (
            type(value).__name__ == "PlaybackPolicy"
            and data.get("minimum_settle_samples") == 1
        ):
            # Omitted default remains decodable by older playback SDKs. A
            # nondefault count requires the separately declared support fact.
            data.pop("minimum_settle_samples")
        if (
            type(value).__name__ == "PlaybackPolicy"
            and data.get("reference_only") is False
        ):
            # An older receiver must reject an enabled completion mode, but can
            # continue decoding ordinary measured-completion policies.
            data.pop("reference_only")
        if type(value).__name__ == "PlaybackPolicy" and not data.get(
            "checkpoint_offsets_ns"
        ):
            # Ordinary policies remain compatible; checkpoint capture requires
            # its separately declared native support fact.
            data.pop("checkpoint_offsets_ns", None)
        if (
            type(value).__name__ == "PlaybackPolicy"
            and data.get("allow_curve_updates") is False
        ):
            data.pop("allow_curve_updates")
        if (
            type(value).__name__ == "PlaybackPolicy"
            and data.get("credit_pacing_delay") is False
        ):
            data.pop("credit_pacing_delay")
        if type(value).__name__ == "PlaybackPolicy" and data.get("part_hold") is False:
            data.pop("part_hold")
        if isinstance(value, runtime.SubmitResult):
            # These optional clocks belong to the SDK host, not this executor.
            data["dispatch_started_monotonic_s"] = None
            data["dispatch_completed_monotonic_s"] = None
        return msgpack.ExtType(2, encode((type(value).__name__, data)))
    if hasattr(value, "session_ns") and hasattr(value, "unix_ns"):
        return msgpack.ExtType(
            3, msgpack.packb((value.session_ns, value.unix_ns), use_bin_type=True)
        )
    raise TypeError(f"Unsupported SDK bridge value: {type(value).__name__}")


def _extension(code, payload):
    if code == 1:
        dtype, shape, compressed = msgpack.unpackb(payload, raw=False)
        if (
            dtype not in _DTYPES
            or not isinstance(shape, list)
            or not 1 <= len(shape) <= 4
            or any(
                type(width) is not int or not 0 <= width <= MAX_ARRAY_BYTES
                for width in shape
            )
        ):
            raise ValueError("Invalid SDK array declaration")
        size = math.prod(shape) * np.dtype(dtype).itemsize
        if size > MAX_ARRAY_BYTES:
            raise ValueError("SDK array exceeds the decoded limit")
        inflater = zlib.decompressobj()
        raw = inflater.decompress(compressed, size + 1)
        if len(raw) != size or not inflater.eof or inflater.unused_data:
            raise ValueError("SDK array does not match its exact declared length")
        result = np.frombuffer(raw, dtype=dtype).reshape(shape)
        result.setflags(write=False)
        return result
    if code == 2:
        name, values = decode(payload)
        if name not in _TYPES or not isinstance(values, dict):
            raise ValueError("Unknown SDK value type")
        resolver = (
            values.pop("remote_point_resolver", False)
            if name == "CameraSample"
            else False
        )
        if name == "RuntimeFault":
            values["code"] = runtime.FaultCode(values["code"])
        if name == "SupportRow":
            values["facts"] = tuple(
                runtime.SupportFact(fact) for fact in values["facts"]
            )
        value = _TYPES[name](**values)
        if resolver:
            object.__setattr__(value, "_remote_point_resolver", True)
        return value
    if code == 3:
        session_ns, unix_ns = msgpack.unpackb(payload, raw=False)
        if (
            type(session_ns) is not int
            or session_ns < 0
            or type(unix_ns) is not int
            or unix_ns <= 0
        ):
            raise ValueError("Invalid paired SDK stamp")
        return SimpleNamespace(session_ns=session_ns, unix_ns=unix_ns)
    raise ValueError("Unknown SDK wire extension")


def encode(value):
    result = msgpack.packb(value, default=_default, use_bin_type=True)
    if len(result) > MAX_BYTES:
        raise RelayError("sdk_bridge_limit", "SDK message exceeds the 4 MiB limit")
    return result


def decode(raw):
    if not isinstance(raw, bytes) or len(raw) > MAX_BYTES:
        raise RelayError("sdk_bridge_protocol", "Expected a bounded binary SDK message")
    try:
        return msgpack.unpackb(raw, raw=False, strict_map_key=True, ext_hook=_extension)
    except (ValueError, TypeError, KeyError, OverflowError, zlib.error) as error:
        raise RelayError("sdk_bridge_protocol", "Malformed SDK value") from error
