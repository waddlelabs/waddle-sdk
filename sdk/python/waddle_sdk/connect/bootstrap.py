"""Apply an installed device runtime before importing any native libraries."""

import importlib.util
import os
import sys


def main(argv=None):
    arguments = sys.argv[1:] if argv is None else argv
    runtime = importlib.util.find_spec("waddle_jetson_runtime")
    if (
        runtime is not None
        and os.environ.get("WADDLE_JETSON_RUNTIME") != runtime.origin
    ):
        from waddle_jetson_runtime import environment

        try:
            selected = environment()
        except ValueError as error:
            print(f"waddle-connect: {error}", file=sys.stderr)
            return 2
        selected = {**os.environ, **selected, "WADDLE_JETSON_RUNTIME": runtime.origin}
        os.execve(
            sys.executable,
            [sys.executable, "-m", "waddle_sdk.connect.bootstrap", *arguments],
            selected,
        )
    from .cli import main as connect_main

    return connect_main(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
