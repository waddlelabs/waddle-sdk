"""An explicitly requested camera snapshot through the SDK inspection lifecycle."""

import json

from PIL import Image
from rich.text import Text


def snapshot(driver, connection, stream):
    selected = json.loads(connection)
    for adapter, selector in (("realsense", "serial"), ("usb", "device")):
        if driver == "waddle_sdk.cameras." + adapter and not selected.get(selector):
            raise ValueError(
                f"Select the camera's {selector} before requesting a preview"
            )
    # Lazy import keeps ordinary setup usable with older SDKs; the action gives
    # an explicit upgrade message rather than opening a camera another way.
    try:
        from waddle_sdk.cameras.inspection import CameraInspectionSpec, inspect_cameras
    except ImportError as error:
        raise ValueError(
            "Camera preview needs an SDK build with camera inspection support"
        ) from error
    spec = CameraInspectionSpec(
        name="setup-preview",
        driver=driver,
        connection=selected,
        stream={key: int(value) for key, value in stream.items()},
    )
    with inspect_cameras([spec]) as inspection:
        frame = inspection.wait(spec.name, timeout_s=5)
        if frame is None:
            reason = (
                str(dict(inspection.errors))
                if inspection.errors
                else "no RGB frame within 5 seconds"
            )
            raise ValueError("Camera preview unavailable: " + reason)
        # Retain only a small copied RGB snapshot. Closing inspection releases
        # the camera before returning to the wizard or preparing a robot owner.
        image = Image.fromarray(frame.rgb).convert("RGB")
        image.thumbnail((60, 28))
    pixels = image.load()
    result = Text()
    for y in range(0, image.height - 1, 2):
        for x in range(image.width):
            upper = "#{:02x}{:02x}{:02x}".format(*pixels[x, y])
            lower = "#{:02x}{:02x}{:02x}".format(*pixels[x, y + 1])
            result.append("▀", style=f"{upper} on {lower}")
        result.append("\n")
    result.append("Snapshot · camera closed. Check the view, then select its mount.")
    return result
