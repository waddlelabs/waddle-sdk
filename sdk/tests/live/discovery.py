"""Small pytest policy over existing non-opening SDK hardware discovery."""

import os
from collections import Counter
from importlib.util import find_spec
from pathlib import Path

import pytest
from waddle_sdk import load_site
from waddle_sdk.cameras.inspection import CameraInspectionSpec
from waddle_sdk.discovery import discover_hardware

from .sdk_live.config import load, reject_ci


def pytest_addoption(parser):
    group = parser.getgroup("live devices")
    group.addoption(
        "--live", action="store_true", help="Run viable real-device tests locally"
    )
    group.addoption(
        "--live-config",
        help="Optional bench JSON; defaults to WADDLE_SDK_LIVE_CONFIG or .live-tests.json",
    )
    group.addoption(
        "--live-site",
        help="Site manifest; defaults to WADDLE_LIVE_SITE or bench config",
    )


def pytest_configure(config):
    for name, text in (
        ("live", "real devices/services, never automatic CI"),
        ("hardware", "opens physical devices"),
        ("motion", "commands physical motion"),
        ("requires_env", "named environment prerequisites for a live service test"),
        ("requires_site", "needs an explicitly configured site manifest"),
        ("named_parts", "two selected arms and device feedback probes"),
        ("named_motion", "reviewed independent motion and timing profile"),
        ("named_envelope", "reviewed out-of-envelope refusal profile"),
    ):
        config.addinivalue_line("markers", f"{name}: {text}")
    config.live_bench = {"parts": [], "cameras": [], "cases": []}
    config.live_missing = {
        "site": "no configured site (--live-site or WADDLE_LIVE_SITE)"
    }
    if not config.getoption("--live"):
        return
    config.option.maxfail = 1
    try:
        reject_ci()
        path = config.getoption("--live-config") or os.environ.get(
            "WADDLE_SDK_LIVE_CONFIG"
        )
        if not path and Path(".live-tests.json").is_file():
            path = ".live-tests.json"
        values = load(path) if path else {}
        site_path = (
            config.getoption("--live-site")
            or os.environ.get("WADDLE_LIVE_SITE")
            or values.get("site")
        )
        report = discover_hardware()
        config.live_warnings = report.warnings
        if not site_path:
            specs = {
                row.identifier: CameraInspectionSpec.from_candidate(row)
                for row in report.candidates
                if row.kind == "camera" and row.driver and row.confidence == "confirmed"
            }
            config.live_bench.update(cameras=list(specs), camera_specs=specs)
            return
        site = load_site(site_path)
        if site.manifest.get("worlds"):
            raise ValueError(
                "Live tests require a physical site, not a simulation world"
            )
        values["site"] = str(site.path)
        values.setdefault("parts", list(site.manifest["parts"]))
        values.setdefault("cameras", list(site.manifest["cameras"]))
        values.setdefault("cases", [])
        values.setdefault(
            "evidence_directory", str(Path(".pytest_cache/live").resolve())
        )
        missing = {}
        for kind in ("parts", "cameras"):
            for name in values[kind]:
                row = site.manifest[kind][name]
                connection = row.get("connection", {})
                identity = {
                    key: value
                    for key, value in connection.items()
                    if key in ("channel", "serial", "device", "port")
                }
                matched = any(
                    (candidate.driver == row["driver"] or candidate.kind == "transport")
                    and all(
                        candidate.connection.get(key) == value
                        for key, value in identity.items()
                    )
                    for candidate in report.candidates
                )
                if not identity or not matched:
                    missing[f"{kind}:{name}"] = (
                        f"{name}: no matching hardware discovery evidence for configured connection"
                    )
        if values.get("torque_release_authorized") is not True:
            missing["shutdown"] = (
                "robot lifecycle needs explicit torque_release_authorized in bench profile"
            )
        config.live_bench, config.live_missing = values, missing
        config.live_warnings = report.warnings
    except (KeyError, ValueError, OSError, TypeError) as error:
        raise pytest.UsageError(f"Live discovery: {error}") from error


def pytest_generate_tests(metafunc):
    # Scope parametrization to this behavior group; ordinary tests are untouched.
    if Path(__file__).parent not in Path(metafunc.definition.path).parents:
        return
    values = metafunc.config.live_bench
    if "named_stop" in metafunc.fixturenames:
        stops = values.get("named_parts", {}).get("stops", []) or [None]
        metafunc.parametrize(
            "named_stop",
            stops,
            ids=[row["method"] if row else "unconfigured" for row in stops],
        )
    for name, key in (("part", "parts"), ("camera", "cameras"), ("case", "cases")):
        if name in metafunc.fixturenames:
            rows = values[key] or [None]
            ids = [
                f"{row['part']}-{row['case_id']}"
                if isinstance(row, dict)
                else row or "unconfigured"
                for row in rows
            ]
            metafunc.parametrize(name, rows, ids=ids)


def reason(item):
    if not item.config.getoption("--live"):
        return "live tests require --live"
    required = (
        [name for marker in item.iter_markers("requires_env") for name in marker.args]
        if hasattr(item, "iter_markers")
        else []
    )
    if required:
        absent = [name for name in required if not os.environ.get(name)]
        provisionable = set(absent) <= {
            "WADDLE_TEST_LIVEKIT_URL",
            "WADDLE_TEST_LIVEKIT_PUBLISHER_TOKEN",
            "WADDLE_TEST_LIVEKIT_VIEWER_TOKEN",
        } and all(
            os.environ.get(name)
            for name in (
                "WADDLE_TEST_LIVEKIT_URL",
                "WADDLE_TEST_LIVEKIT_API_KEY",
                "WADDLE_TEST_LIVEKIT_API_SECRET",
            )
        )
        if absent and not provisionable:
            return "missing " + ", ".join(absent)
    missing = item.config.live_missing
    params = getattr(getattr(item, "callspec", None), "params", {})
    needs_site = (
        item.get_closest_marker("requires_site")
        if hasattr(item, "get_closest_marker")
        else None
    )
    if needs_site and "site" in missing:
        return missing["site"]
    if "camera" in params:
        return missing.get(f"cameras:{params['camera']}") or (
            "no configured camera" if params["camera"] is None else None
        )
    if required and not params and not needs_site:
        return None
    if "site" in missing:
        return missing["site"]
    named = (
        item.get_closest_marker("named_parts")
        if hasattr(item, "get_closest_marker")
        else None
    )
    if named:
        from .sdk_live.feedback import factory_path

        profile = item.config.live_bench.get("named_parts")
        if not profile:
            return "two-arm acceptance needs a named_parts profile"
        for part in profile["parts"]:
            if f"parts:{part}" in missing:
                return missing[f"parts:{part}"]
        if "shutdown" in missing:
            return missing["shutdown"]
        if item.get_closest_marker("named_motion") and not profile.get("motion_cases"):
            return "named motion needs reviewed motion_cases and timing limits"
        if item.get_closest_marker("named_envelope") and not profile.get("envelope"):
            return "named envelope test needs a reviewed refusal target"
        if "named_stop" in params and params["named_stop"] is None:
            return "named stopping needs reviewed stops with physical support"
        site = load_site(item.config.live_bench["site"])
        for part in profile["parts"]:
            if factory_path(item.config.live_bench, site.manifest, part) is None:
                return f"{part}: device acquisition freshness needs a feedback probe"
        return None
    # Single-part tests retain the original site lock ID as well.
    selected_part = params.get("part") or (params.get("case") or {}).get("part")
    if f"parts:{selected_part}" in missing:
        return missing[f"parts:{selected_part}"]
    if (
        params.get("case")
        and item.config.live_bench.get("comparison", {}).get("vendor") == "i2rt"
    ):
        site = load_site(item.config.live_bench["site"])
        if (
            site.manifest["parts"][selected_part].get("driver")
            != "waddle_sdk.robots.yam:arm"
        ):
            return "raw i2rt comparison requires a YAM arm"
        if find_spec("i2rt") is None:
            return "raw i2rt comparison requires the optional I2RT package"
    if "shutdown" in missing:
        return missing["shutdown"]
    if "case" in params and params["case"] is None:
        return "motion needs configured target/limit cases"
    if "part" in params and params["part"] is None:
        return "no configured robot part"
    return None


def pytest_collection_modifyitems(config, items):
    counts = Counter()
    for item in items:
        grouped = Path(__file__).parent in item.path.parents
        marked = (
            item.get_closest_marker("live")
            if hasattr(item, "get_closest_marker")
            else None
        )
        if not grouped and not marked:
            continue
        item.add_marker(pytest.mark.live)
        if grouped:
            item.add_marker(pytest.mark.hardware)
        unavailable = reason(item)
        counts[unavailable or "eligible"] += 1
        if unavailable:
            item.add_marker(pytest.mark.skip(reason=unavailable))
    if (
        config.getoption("--live")
        and config.getoption("numprocesses", default=0)
        and any(item.get_closest_marker("hardware") for item in items)
    ):
        raise pytest.UsageError(
            "Live hardware tests require sequential execution; omit -n"
        )
    config.live_selection = counts


def pytest_terminal_summary(terminalreporter, config):
    if not config.getoption("--live"):
        return
    terminalreporter.section("Live discovery (metadata; execution rechecks devices)")
    for label, count in sorted(config.live_selection.items()):
        terminalreporter.write_line(f"{count}: {label}")
    for warning in getattr(config, "live_warnings", ()):
        terminalreporter.write_line(warning)
    if all(
        os.environ.get(name)
        for name in (
            "WADDLE_TEST_LIVEKIT_URL",
            "WADDLE_TEST_LIVEKIT_API_KEY",
            "WADDLE_TEST_LIVEKIT_API_SECRET",
        )
    ):
        terminalreporter.write_line(
            "LiveKit signing credentials configured; only selected media fixtures issue grants. "
            "Discovery does not verify service permission."
        )
