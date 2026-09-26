"""Optional local image-content clocks, independent of paired stream stamps."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraContentTiming:
    """Conservative content intervals on this host's ``time.monotonic_ns`` clock.

    Each interval contains the acquisition times of all pixels in that plane,
    including exposure/readout and clock-mapping uncertainty. A driver must not
    infer a lower bound from a buffered read's start or from frame delivery.
    ``sensor_exposure`` requires a verified mapping from the sensor clock;
    ``simulated_state`` bounds the native state snapshot used to render pixels.
    ``clock_revision`` identifies the mapping and changes when that mapping does.
    These local values never replace the session/Unix pair or imply wire support.
    """

    kind: str
    clock_revision: str
    rgb_monotonic_ns: tuple[int, int]
    depth_monotonic_ns: tuple[int, int] | None = None

    def __post_init__(self):
        if self.kind not in ("sensor_exposure", "simulated_state"):
            raise ValueError("camera content timing kind is invalid")
        if (
            not isinstance(self.clock_revision, str)
            or not 1 <= len(self.clock_revision) <= 128
            or any(ord(c) < 32 for c in self.clock_revision)
        ):
            raise ValueError("camera content timing needs a bounded clock revision")
        for field in ("rgb_monotonic_ns", "depth_monotonic_ns"):
            value = getattr(self, field)
            if value is None and field == "depth_monotonic_ns":
                continue
            if (
                not isinstance(value, (tuple, list))
                or len(value) != 2
                or any(type(x) is not int or not 0 <= x < 2**63 for x in value)
                or value[0] > value[1]
            ):
                raise ValueError(
                    f"camera {field} needs ordered nonnegative nanoseconds"
                )
            object.__setattr__(self, field, tuple(value))


def validate_content_timing(value, depth):
    if value is None:
        return
    if not isinstance(value, CameraContentTiming):
        raise TypeError("content_timing must be CameraContentTiming or None")
    if (value.depth_monotonic_ns is None) != (depth is None):
        raise ValueError(
            "camera content timing must cover exactly its RGB/depth planes"
        )
