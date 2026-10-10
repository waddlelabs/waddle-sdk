"""Private preview child, invoked as a script before importing heavy libraries.

This process receives RGB slots and a media grant. It has no hardware/site
configuration, SDK session, control transport, callbacks, or recording authority.
"""

import fcntl
import json
import mmap
import os
import resource
import signal
import socket
import struct
import sys
import time


def main():
    channel = socket.socket(fileno=int(sys.argv[1]))
    config = json.loads(channel.recv(16 * 1024))
    channel.setblocking(False)
    slots = []
    publisher = None
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    def report(value):
        try:
            channel.send(json.dumps(value).encode())
        except (BlockingIOError, OSError):
            pass  # bounded optional metadata; never stall video on its observer

    try:
        if config["worker_cpu_ids"] is not None:
            os.sched_setaffinity(0, config["worker_cpu_ids"])
        os.nice(10)
        os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
        memory_bound = config["worker_memory_mb"] * 1024 * 1024
        resource.setrlimit(resource.RLIMIT_AS, (memory_bound, memory_bound))
        for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
            os.environ[variable] = "1"
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        if os.getppid() != config["parent_pid"]:
            return
        # Affinity/priority precede NumPy, native-library initialization and
        # WebRTC thread creation. Native child threads inherit this placement.
        sys.path.insert(0, config["package_root"])
        import numpy as np

        from waddle_sdk._native import core

        cameras = [(c["name"], c["width"], c["height"]) for c in config["cameras"]]
        publisher = core.PreviewPublisher(
            cameras,
            config["url"],
            config["token"],
            config["preview_width"],
            config["preview_fps"],
            config["preview_max_kbps"],
            config["demand_driven"],
            config["testing_loopback"],
        )
        header = struct.Struct("!QQ")
        rows = []
        for camera in config["cameras"]:
            size = header.size + camera["width"] * camera["height"] * 3
            slots.append(
                (camera, mmap.mmap(camera["fd"], size, access=mmap.ACCESS_READ))
            )
            rows.append(
                {
                    "camera_id": camera["name"],
                    "stream": "rgb",
                    "track_name": camera["name"],
                    "status": "pending",
                    "frames_dropped": 0,
                    "last_sequence": 0,
                }
            )
        report({"tracks": rows})
        last_report = time.monotonic()
        while not stopping and os.getppid() == config["parent_pid"]:
            for (camera, memory), row in zip(slots, rows, strict=True):
                try:
                    fcntl.lockf(camera["fd"], fcntl.LOCK_SH | fcntl.LOCK_NB)
                except BlockingIOError:
                    continue
                try:
                    sequence, timestamp = header.unpack_from(memory)
                    if sequence == 0 or sequence == row["last_sequence"]:
                        continue
                    image = np.frombuffer(
                        memory, dtype=np.uint8, offset=header.size
                    ).copy()
                finally:
                    fcntl.lockf(camera["fd"], fcntl.LOCK_UN)
                row["frames_dropped"] += max(0, sequence - row["last_sequence"] - 1)
                row["last_sequence"] = sequence
                try:
                    publisher.publish_frame(
                        camera["name"],
                        image.reshape(camera["height"], camera["width"], 3),
                        timestamp,
                    )
                    row["status"] = "published"
                    row.pop("error", None)
                except Exception as error:  # noqa: BLE001 -- preserve the exact scoped native publication failure
                    row["status"] = "publish_failed"
                    row["frames_dropped"] += 1
                    row["error"] = str(error)
            if time.monotonic() - last_report >= 1.0:
                for row in rows:
                    row["wants_frame"] = publisher.wants_frame(row["camera_id"])
                report({"tracks": rows})
                last_report = time.monotonic()
            time.sleep(min(0.1, 1.0 / config["preview_fps"]))
    except Exception as error:  # noqa: BLE001 -- resource/native failure disables only this hardware-free child
        report({"error": f"media_worker_failed: {type(error).__name__}: {error}"})
    finally:
        for camera, memory in slots:
            memory.close()
            os.close(camera["fd"])
        if publisher is not None:
            publisher.close()
        channel.close()


if __name__ == "__main__":
    main()
