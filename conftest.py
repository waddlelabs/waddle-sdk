"""Register checkout-local test discovery before pytest parses optional bench paths."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "sdk/tests"))
pytest_plugins = ("live.discovery",)
