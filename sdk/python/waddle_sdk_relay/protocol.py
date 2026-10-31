"""SDK device/cloud attachment identifiers, independent of hardware imports."""

PROTOCOL = "waddle.sdk-bridge.v1"
MAX_PENDING = 128
MAX_CALLS = 16
MAX_SAFETY_CALLS = 2
SAFETY_METHODS = frozenset({"hold", "estop", "stop"})
