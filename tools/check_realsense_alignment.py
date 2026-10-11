"""Exercise native RGB-D alignment using only a software device.

Run once with each reference/candidate binding in a separate process. Compare
the generated JSON pixel hashes, and retain both timing/progress measurements.
This does not enumerate USB, open a camera, or qualify physical control timing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np


def exercise(output: Path, frames: int) -> dict:
    import pyrealsense2 as rs

    if output.exists():
        raise ValueError("Refusing to overwrite alignment evidence")
    width, height = 640, 480
    device = rs.software_device()
    device.create_matcher(rs.matchers.default)
    sensors, profiles = [], []
    for index, (name, kind, format_, bpp) in enumerate(
        [
            ("Depth", rs.stream.depth, rs.format.z16, 2),
            ("Color", rs.stream.color, rs.format.rgb8, 3),
        ]
    ):
        sensor = device.add_sensor(name)
        stream = rs.video_stream()
        stream.type, stream.fmt = kind, format_
        stream.uid, stream.width, stream.height = index, width, height
        stream.bpp, stream.fps = bpp, 30
        intrinsics = rs.intrinsics()
        intrinsics.width, intrinsics.height = width, height
        intrinsics.fx, intrinsics.fy = 400.0 + 10 * index, 405.0 + 10 * index
        intrinsics.ppx, intrinsics.ppy = 318.0 + index, 239.0 + index
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0.0] * 5
        stream.intrinsics = intrinsics
        profile = sensor.add_video_stream(stream).as_video_stream_profile()
        sensors.append(sensor)
        profiles.append(profile)
    sensors[0].add_read_only_option(rs.option.depth_units, 0.001)
    extrinsics = rs.extrinsics()
    extrinsics.rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    extrinsics.translation = [0.025, -0.005, 0.0]
    profiles[0].register_extrinsics_to(profiles[1], extrinsics)
    syncer = rs.syncer(4)
    started = []
    stop = threading.Event()
    ready = threading.Event()
    progress = [0]

    def observer():
        ready.set()
        while not stop.is_set():
            progress[0] += 1
            time.sleep(0.0001)

    worker = threading.Thread(target=observer, name="alignment-progress")
    previous_switch_interval = sys.getswitchinterval()
    hashes, measurements = [], []
    y, x = np.indices((height, width), dtype=np.uint16)
    align = rs.align(rs.stream.color)
    try:
        for sensor, profile in zip(sensors, profiles, strict=True):
            sensor.open(profile)
            sensor.start(syncer)
            started.append(sensor)
        worker.start()
        if not ready.wait(1.0):
            raise RuntimeError("Progress observer did not start")
        # Keep ordinary Python's periodic GIL handoff outside each short native
        # call. The sleeping observer can advance only when the binding yields.
        sys.setswitchinterval(5.0)
        for number in range(frames + 1):
            depth = ((x + 3 * y + number * 11) % 2500 + 250).astype(np.uint16)
            depth[(x + y + number) % 31 == 0] = 0
            rgb = np.stack(
                [
                    (x + number) % 256,
                    (y + number * 3) % 256,
                    (x + y + number * 7) % 256,
                ],
                axis=-1,
            ).astype(np.uint8)
            buffers = [depth, rgb]
            for index, sensor in enumerate(sensors):
                frame = rs.software_video_frame()
                frame.pixels = bytearray(buffers[index].tobytes())
                frame.stride = width * (2 if index == 0 else 3)
                frame.bpp = 2 if index == 0 else 3
                frame.frame_number = number
                frame.timestamp = number * 1000.0 / 30.0
                frame.domain = rs.timestamp_domain.hardware_clock
                frame.profile = profiles[index]
                if index == 0:
                    frame.depth_units = 0.001
                sensor.on_video_frame(frame)
            if number == 0:
                # Upstream's software syncer initially emits each unknown stream
                # separately. Drain that documented bootstrap before comparison.
                for _ in sensors:
                    syncer.wait_for_frames(1000)
                continue
            acquired = syncer.wait_for_frames(1000)
            if not acquired.get_depth_frame() or not acquired.get_color_frame():
                raise RuntimeError(
                    "Software device did not produce a complete RGB-D sample"
                )
            before = progress[0]
            begin = time.perf_counter_ns()
            aligned = align.process(acquired)
            elapsed = time.perf_counter_ns() - begin
            during = progress[0] - before
            if elapsed >= 5_000_000_000:
                raise RuntimeError(
                    "Native call exceeded the observer's GIL measurement bound"
                )
            actual_depth = np.asarray(aligned.get_depth_frame().get_data())
            actual_rgb = np.asarray(aligned.get_color_frame().get_data())
            if actual_depth.shape != depth.shape or actual_rgb.shape != rgb.shape:
                raise RuntimeError("Aligned software-device grids changed")
            if not np.array_equal(actual_rgb, rgb):
                raise RuntimeError("Alignment changed the color-grid RGB pixels")
            hashes.append(
                {
                    "frame": number,
                    "rgb": hashlib.sha256(actual_rgb.tobytes()).hexdigest(),
                    "depth": hashlib.sha256(actual_depth.tobytes()).hexdigest(),
                }
            )
            measurements.append(
                {"alignment_ms": elapsed / 1e6, "observer_progress": during}
            )
    finally:
        sys.setswitchinterval(previous_switch_interval)
        stop.set()
        if worker.ident is not None:
            worker.join(timeout=1.0)
            if worker.is_alive():
                raise RuntimeError("Progress observer did not drain")
        for sensor in reversed(started):
            sensor.stop()
            sensor.close()
    result = {
        "native_version": rs.__version__,
        "binding": rs.__file__,
        "frames": frames,
        "pixels": hashes,
        "measurements": measurements,
        "alignment_ms": {
            key: float(value)
            for key, value in zip(
                ["p50", "p95", "max"],
                np.percentile(
                    [row["alignment_ms"] for row in measurements], [50, 95, 100]
                ),
                strict=True,
            )
        },
        "calls_with_observer_progress": sum(
            row["observer_progress"] > 0 for row in measurements
        ),
        "hardware_opened": False,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--frames", default=50, type=int, choices=range(1, 201), metavar="1..200"
    )
    args = parser.parse_args()
    result = exercise(args.output, args.frames)
    print(
        json.dumps(
            {
                key: value
                for key, value in result.items()
                if key not in {"pixels", "measurements"}
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
