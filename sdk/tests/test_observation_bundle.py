"""One read-only relay request preserves frame identity and workspace errors."""

from types import SimpleNamespace

import pytest
from waddle_sdk.runtime import Observation
from waddle_sdk_relay.device import Device
from waddle_sdk_relay.errors import RelayError


def device_at(root):
    device = Device(None, root)
    reads = []
    observation = Observation(1, 2, {}, {})
    device.sdk = SimpleNamespace(
        describe=lambda: {"revision": len(reads)},
        support=lambda: "support",
        observe_parts=lambda parts=None: reads.append(parts) or observation,
    )
    return device, reads, observation


def test_bundle_reads_once_retains_identity_and_workspace_error(tmp_path):
    device, reads, observation = device_at(tmp_path)
    result = device.dispatch(
        "observation_bundle",
        {"workspace": [{"operation": "read", "path": "missing.json"}]},
    )
    assert reads == [None]
    assert result["support"] == "support"
    assert result["observation"] is observation
    assert device._acquired(result["observation_id"]) is observation
    assert result["workspace"][0]["error"]["code"] == "workspace.not_found"
    again = device.dispatch("observation_bundle", {})
    assert again["observation_id"] != result["observation_id"]
    assert reads == [None, None]


@pytest.mark.parametrize(
    "read_request",
    [
        {"operation": "write", "path": "file", "content": "bad"},
        {"operation": "read", "path": ".connection/key"},
        {"operation": "read", "path": "../outside"},
    ],
)
def test_bundle_rejects_mutation_and_private_paths_before_observing(
    tmp_path, read_request
):
    device, reads, _ = device_at(tmp_path)
    with pytest.raises((RelayError, ValueError)):
        device.dispatch("observation_bundle", {"workspace": [read_request]})
    assert not reads
    assert not list(tmp_path.iterdir())


def test_bundle_bounds_workspace_requests(tmp_path):
    device, reads, _ = device_at(tmp_path)
    with pytest.raises(RelayError):
        device.dispatch(
            "observation_bundle",
            {"workspace": [{"operation": "list", "path": "."}] * 9},
        )
    assert not reads
