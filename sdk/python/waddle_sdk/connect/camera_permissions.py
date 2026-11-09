"""Prepare Linux USB access for detected RealSense cameras, without opening them."""

import os
import re
import shlex
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

from waddle_sdk_relay.errors import RelayError


def _devices(sys_root, dev_root):
    for device in sorted(sys_root.glob("*")):
        try:
            if device.joinpath("idVendor").read_text().strip().lower() != "8086":
                continue
            if "realsense" not in device.joinpath("product").read_text().lower():
                continue
            product = device.joinpath("idProduct").read_text().strip().lower()
            if not re.fullmatch("[0-9a-f]{4}", product):
                continue
            bus = int(device.joinpath("busnum").read_text())
            number = int(device.joinpath("devnum").read_text())
        except (OSError, ValueError):
            continue
        yield device, product, dev_root / f"{bus:03d}" / f"{number:03d}"


def prepare_realsense_usb_access(
    *,
    sys_root=Path("/sys/bus/usb/devices"),
    dev_root=Path("/dev/bus/usb"),
    rules_root=Path("/etc/udev/rules.d"),
    interactive=False,
):
    """Install video-group rules only where the current user lacks USB access.

    Existing administrator access policy remains sufficient when it already
    works. No camera stream, USB reset, or robot bus is opened here.
    """
    if sys.platform != "linux":
        return
    # A piped installer must fail promptly instead of asking sudo to read from
    # an unrelated terminal. The TUI suspends before this interactive callback.
    interactive = interactive and sys.stdin.isatty()
    blocked = [
        row
        for row in _devices(sys_root, dev_root)
        if not os.access(row[2], os.R_OK | os.W_OK)
    ]
    if not blocked:
        return
    import grp

    try:
        video_gid = grp.getgrnam("video").gr_gid
    except KeyError as error:
        raise RelayError(
            "setup.camera_permissions", "The Linux video group is missing."
        ) from error
    if os.geteuid() != 0 and video_gid not in {*os.getgroups(), os.getegid()}:
        raise RelayError(
            "setup.camera_permissions",
            "RealSense needs read/write USB access. Ask an administrator to add your "
            "account to the video group, then log out and reconnect before retrying setup.",
        )
    if interactive:
        print(
            "RealSense needs USB read/write access. Preparing video-group permissions "
            "for detected cameras; sudo may ask for your password.",
            flush=True,
        )
    prefix = [] if os.geteuid() == 0 else (["sudo"] if interactive else ["sudo", "-n"])

    def run(label, *args):
        if interactive:
            print(label, flush=True)
        try:
            result = subprocess.run(
                prefix + list(args),
                check=False,
                capture_output=not interactive,
                text=True,
                timeout=120 if interactive else 15,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RelayError(
                "setup.camera_permissions", label + " " + str(error)
            ) from error
        if result.returncode:
            raise RelayError(
                "setup.camera_permissions",
                "Cannot prepare RealSense USB access: "
                + (
                    result.stderr or result.stdout or f"exit status {result.returncode}"
                ).strip()
                + ". In another terminal run `"
                + shlex.quote(sys.executable)
                + " -m waddle_sdk.connect.camera_permissions`, "
                "then retry Add or Preview. "
                "That command can prompt for sudo to install camera rules.",
            )

    with TemporaryDirectory(prefix="waddle-camera-rules-") as temporary:
        for product in sorted({row[1] for row in blocked}):
            rule = (
                "# RealSense USB access for members of the video group.\n"
                f'SUBSYSTEM=="usb", ATTR{{idVendor}}=="8086", ATTR{{idProduct}}=="{product}", '
                'GROUP="video", MODE="0660"\n'
            )
            destination = rules_root / f"99-waddle-realsense-{product}.rules"
            if destination.exists():
                if destination.read_text() != rule:
                    raise RelayError(
                        "setup.camera_permissions",
                        f"Existing rule differs; review {destination}",
                    )
            else:
                source = Path(temporary) / destination.name
                source.write_text(rule)
                run(
                    "Installing camera USB access rule…",
                    "install",
                    "-m",
                    "0644",
                    str(source),
                    str(destination),
                )
        run(
            "Reloading camera USB access rules…", "udevadm", "control", "--reload-rules"
        )
        for device, _, _ in blocked:
            run(
                "Applying camera USB access…",
                "udevadm",
                "trigger",
                "--action=change",
                str(device),
            )
        run("Checking camera USB access…", "udevadm", "settle", "--timeout=5")
    remaining = [
        str(row[2]) for row in blocked if not os.access(row[2], os.R_OK | os.W_OK)
    ]
    if remaining:
        raise RelayError(
            "setup.camera_permissions",
            "RealSense USB access is still unavailable after applying rules: "
            + ", ".join(remaining),
        )
    if interactive:
        print("Camera USB access ready. Continuing setup…", flush=True)


if __name__ == "__main__":
    prepare_realsense_usb_access(interactive=True)
    print("RealSense USB access checked. Retry Add or Preview in setup.")
