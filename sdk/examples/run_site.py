"""Run one bounded local episode against the simulated example site."""

from __future__ import annotations

from pathlib import Path

import numpy as np

import waddle_sdk


def main() -> int:
    site = waddle_sdk.load_site(Path(__file__).with_name("site.yaml"))
    with site.open(console=False) as session:
        with session.run(
            task={"id": "observe-and-hold"},
            actor={"id": "example"},
        ) as run:
            observation = run.observe()
            action = np.concatenate(
                [part.joint_position for part in observation.parts.values()]
            )
            result = run.step(action, observation)
            run.finish("success" if result.dispatched else "failure")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
