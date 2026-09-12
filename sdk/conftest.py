"""Expose the same pytest plugin when invoking tests from the Python SDK root."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "tests"))
pytest_plugins = ("live.discovery",)
