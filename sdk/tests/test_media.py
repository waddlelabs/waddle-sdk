"""The Python-facing frame-ingestion surface: `session.
publish_frame(camera, frame)` over the `_testing` loopback media plane."""

import time

import numpy as np
import pytest

import waddle
import waddle._testing


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()


def _robot(camera: waddle.Camera, n_joints: int = 3) -> waddle.Robot:
    return waddle.Robot(
        name="pytest-media-bot",
        robot_id="py-media-01",
        cell_id="cell-py-media",
        action_space=waddle.JointSpace(
            joints=[f"j{i}" for i in range(n_joints)], rate_hz=50
        ),
        cameras={"overhead": camera},
    )


def _control() -> waddle.Control:
    return waddle.Control(send=lambda chunk: None, hold=lambda: None, resume=lambda: None)


def test_publish_frame_round_trips_through_the_loopback_far_end():
    camera = waddle.Camera(width=4, height=4, fps=30)
    session = waddle.init("py-media-roundtrip", _robot(camera), _control(), _testing=True)

    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    frame[:] = 7
    session.publish_frame("overhead", frame)

    deadline = time.monotonic() + 2.0
    got: list[bytes] = []
    while time.monotonic() < deadline:
        got = waddle._testing.frames(session, "overhead")
        if got:
            break
        time.sleep(0.005)
    assert got, "no frame observed on the loopback far end"
    assert got[-1] == bytes([7] * (4 * 4 * 3))


def test_publish_frame_wrong_dtype_raises_type_error():
    camera = waddle.Camera(width=4, height=4, fps=30)
    session = waddle.init("py-media-dtype", _robot(camera), _control(), _testing=True)

    wrong_dtype = np.zeros((4, 4, 3), dtype=np.float32)
    with pytest.raises(TypeError):
        session.publish_frame("overhead", wrong_dtype)


def test_publish_frame_wrong_shape_raises_type_error():
    camera = waddle.Camera(width=4, height=4, fps=30)
    session = waddle.init("py-media-shape", _robot(camera), _control(), _testing=True)

    # Missing the channel axis entirely.
    wrong_shape = np.zeros((4, 4), dtype=np.uint8)
    with pytest.raises(TypeError):
        session.publish_frame("overhead", wrong_shape)

    # Right rank, wrong channel count.
    wrong_channels = np.zeros((4, 4, 4), dtype=np.uint8)
    with pytest.raises(TypeError):
        session.publish_frame("overhead", wrong_channels)


def test_publish_frame_rejects_fortran_ordered_arrays():
    # numpy's own "contiguous" check is layout-agnostic (accepts Fortran/
    # column-major too), which would silently transpose the row-major pixel
    # bytes this method assumes instead of raising — publish_frame must
    # reject this specific case rather than accept and corrupt it.
    camera = waddle.Camera(width=4, height=4, fps=30)
    session = waddle.init("py-media-fortran", _robot(camera), _control(), _testing=True)

    fortran_frame = np.asfortranarray(np.zeros((4, 4, 3), dtype=np.uint8))
    assert fortran_frame.flags["F_CONTIGUOUS"]
    assert not fortran_frame.flags["C_CONTIGUOUS"]
    with pytest.raises(TypeError, match="C-contiguous"):
        session.publish_frame("overhead", fortran_frame)


def test_publish_frame_unknown_camera_raises():
    camera = waddle.Camera(width=4, height=4, fps=30)
    session = waddle.init("py-media-unknown", _robot(camera), _control(), _testing=True)

    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    with pytest.raises(RuntimeError, match="unknown camera"):
        session.publish_frame("not-a-camera", frame)


def test_publish_frame_fps_throttle_is_observable():
    camera = waddle.Camera(
        width=2,
        height=2,
        fps=30,
        stream_policy=waddle.StreamPolicy(
            uplink=waddle.Uplink(fps=5.0, encoding="rgb8")
        ),
    )
    session = waddle.init("py-media-throttle", _robot(camera), _control(), _testing=True)

    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    # ~200ms of pushes at the declared 5fps (200ms period) admits only a
    # couple of frames, nowhere near the 40 pushed.
    for _ in range(40):
        session.publish_frame("overhead", frame)
        time.sleep(0.005)
    time.sleep(0.05)  # let the uplink pump drain
    got = len(waddle._testing.frames(session, "overhead"))
    assert 1 <= got <= 10, f"expected roughly 5fps worth of frames, got {got}"


def test_media_and_testing_are_mutually_exclusive():
    camera = waddle.Camera(width=4, height=4, fps=30)
    with pytest.raises(ValueError, match="media and _testing"):
        waddle.init(
            "py-media-conflict",
            _robot(camera),
            _control(),
            media=waddle.LiveKit(url="wss://example.invalid", token="tok"),
            _testing=True,
        )


def test_media_livekit_raises_a_clean_not_compiled_error():
    camera = waddle.Camera(width=4, height=4, fps=30)
    with pytest.raises(RuntimeError, match="not compiled"):
        waddle.init(
            "py-media-livekit",
            _robot(camera),
            _control(),
            media=waddle.LiveKit(url="wss://example.invalid", token="tok"),
        )
