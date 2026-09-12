"""Test-only acquisition evidence. Envelope timestamps are never freshness proof."""

import importlib
import time

from waddle_sdk.runtime import FaultCode, RuntimeFault


def factory_path(config, manifest, part):
    explicit = config["named_parts"].get("feedback_probes", {}).get(part)
    if explicit:
        return explicit
    if manifest["parts"][part].get("driver") == "waddle_sdk.robots.yam:arm":
        return "live.sdk_live.feedback:YamFeedback"
    return None


def create(path, bench, part):
    module, name = path.split(":", 1)
    return getattr(importlib.import_module(module), name)(bench, part)


class YamFeedback:
    """Observe actual CAN cache replacement and later robot-state ingestion.

    Retain the objects, not their recycled IDs. Counters are locally observed
    generations, not motor sequence numbers or exact acquisition timestamps.
    """

    def __init__(self, bench, part):
        self.robot = bench.session._managed.arms[part].driver._robot
        self.part = part
        self.timeout = bench.profile["max_feedback_gap_s"]
        self.cache = self.ingested = None
        self.generation = self.ingestion = 0

    def sample(self):
        robot, chain = self.robot, self.robot.motor_chain
        if not chain.running or not robot._server_thread.is_alive():
            raise RuntimeFault(
                FaultCode.MOTOR_FAILURE,
                "I2RT feedback worker stopped",
                context={"part": self.part, "operation": "live_feedback_probe"},
            )
        # The same bounded lock order as the vendor server. Public reads below
        # independently exercise the SDK driver's stale-feedback checks.
        started = time.monotonic()
        if not robot._state_lock.acquire(timeout=self.timeout):
            raise TimeoutError(f"{self.part}: robot-state probe lock timed out")
        try:
            state = robot._joint_state
            positions = state.pos.copy().tolist()
            if not robot._command_lock.acquire(
                timeout=max(0, self.timeout - (time.monotonic() - started))
            ):
                raise TimeoutError(f"{self.part}: command probe lock timed out")
            try:
                commanded = robot._commands.pos.copy().tolist()
            finally:
                robot._command_lock.release()
            if not chain.state_lock.acquire(
                timeout=max(0, self.timeout - (time.monotonic() - started))
            ):
                raise TimeoutError(f"{self.part}: CAN probe lock timed out")
            try:
                cache = chain.state
            finally:
                chain.state_lock.release()
        finally:
            robot._state_lock.release()
        if cache is not None and cache is not self.cache:
            self.cache = cache
            self.generation += 1
        if state is not self.ingested:
            self.ingested = state
            self.ingestion += 1
        return {
            "source": "test-only I2RT CAN cache and ingested robot-state replacement",
            "generation": self.generation,
            "ingestion": self.ingestion,
            # I2RT's final axis uses motor coordinates; only compare arm axes.
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "position_rad": positions[:6],
            "command_position_rad": commanded[:6],
            "command_provenance": "I2RT local controller reference, not motor acknowledgement",
        }
