"""Install the selected SDK extras and the release's existing planner pins."""

import os
import shutil
import subprocess
import sys
from importlib.metadata import PackageNotFoundError, requires, version
from pathlib import Path
from tempfile import TemporaryDirectory

from waddle_sdk_relay.errors import RelayError as ToolError

from .setup import account_client, checked

HARDWARE = {
    "mujoco": ["waddle-sdk[mujoco]"],
    "yam": [
        "i2rt @ git+https://github.com/i2rt-robotics/i2rt@570ef66681ff12bd8298aba34084307cfecc9f05"
    ],
    "realsense": ["waddle-sdk[realsense]"],
    "usb": ["waddle-sdk[usb]"],
    "orbbec": ["waddle-sdk[orbbec]"],
    "xarm": ["waddle-sdk[xarm]"],
    "alicia": ["waddle-sdk[alicia]"],
    "alicia-d": ["waddle-sdk[alicia-d]"],
}


def hardware_for_drivers(drivers):
    selected = set()
    for driver in drivers:
        for name in ("yam", "xarm", "alicia", "alicia-d", "mujoco"):
            if driver.split(":", 1)[0] == "waddle_sdk.robots." + name.replace("-", "_"):
                selected.add(name)
        for name in ("realsense", "usb", "orbbec"):
            if driver.split(":", 1)[0] == "waddle_sdk.cameras." + name:
                selected.add(name)
    return sorted(selected)


def hardware_for_site(site):
    return hardware_for_drivers(
        row.get("driver", "")
        for section in ("parts", "cameras")
        for row in site.get(section, {}).values()
    )


def install_support(hardware, planner, endpoint, key_file, *, quiet=False):
    if sys.platform == "darwin" and "yam" in hardware:
        raise ToolError(
            "setup.hardware_platform",
            "YAM uses Linux SocketCAN. Connect it through a Linux computer or Jetson; "
            "you can use the app from your Mac. "
            "Select the installed SDK driver for other hardware.",
        )
    if planner is not None:
        raise ValueError(
            "Local planning support requires its installed setup extension"
        )
    uv = shutil.which("uv")
    if not uv:
        local_uv = Path(sys.executable).parents[2] / "tools" / "uv"
        if local_uv.is_file() and os.access(local_uv, os.X_OK):
            uv = str(local_uv)
    if not uv:
        raise ValueError("uv is unavailable; rerun the app's setup command")
    # Preserve the installed SDK's runtime constraints while adding hardware.
    requirements = ["waddle-sdk[media]"] + [
        requirement
        for requirement in requires("waddle-sdk")
        if "extra ==" not in requirement
    ]
    selected = set(hardware)
    for name, distribution in (
        ("yam", "i2rt"),
        ("realsense", "pyrealsense2"),
        ("usb", "opencv-python-headless"),
        ("orbbec", "pyorbbecsdk2"),
    ):
        try:
            version(distribution)
        except PackageNotFoundError:
            continue
        selected.add(name)
    for name in sorted(selected):
        requirements.extend(HARDWARE[name])
    with (
        TemporaryDirectory(prefix="waddle-dependencies-") as directory,
        account_client(endpoint, key_file) as client,
    ):
        response = client.get("/api/v1/packages/files/requirements-build.txt")
        if not response.is_success:
            checked(response)
        constraints = Path(directory) / "requirements-build.txt"
        constraints.write_text(response.text)
        command = [
            uv,
            "pip",
            "install",
            "--python",
            sys.executable,
            "--build-constraint",
            str(constraints),
        ]
        environment = os.environ.copy()
        environment.setdefault("UV_CONCURRENT_BUILDS", "1")
        environment.setdefault("CMAKE_BUILD_PARALLEL_LEVEL", "2")
        environment["UV_INDEX_WADDLE_USERNAME"] = "key"
        environment["UV_INDEX_WADDLE_PASSWORD"] = key_file.read_text().strip()
        command += ["--index", "waddle=" + endpoint + "/api/v1/packages/simple"]
        if not quiet:
            print("Installing selected camera/robot support…", flush=True)
        # Resolve the hardware and planner together. A second independent
        # install can otherwise upgrade NumPy/Click past I2RT's requirements.
        for invocation in (
            command + requirements,
            [uv, "pip", "check", "--python", sys.executable],
        ):
            try:
                subprocess.run(
                    invocation,
                    check=True,
                    env=environment,
                    capture_output=quiet,
                    text=True,
                )
            except subprocess.CalledProcessError as error:
                if not quiet:
                    raise
                detail = (error.stderr or error.stdout or str(error)).strip()
                # The index credential is passed only through the environment;
                # keep package-manager diagnostics safe even on an unusual failure.
                detail = detail.replace(
                    environment["UV_INDEX_WADDLE_PASSWORD"], "[redacted]"
                )
                raise ToolError("setup.dependencies", detail) from error

    if "realsense" in selected:
        from .camera_permissions import prepare_realsense_usb_access

        prepare_realsense_usb_access(interactive=not quiet and sys.stdin.isatty())
