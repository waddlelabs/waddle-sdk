"""Drift guards for the strict public SDK contract."""

from __future__ import annotations

import ast
from pathlib import Path

import waddle_sdk
import yaml

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]


SDK = Path(__file__).resolve().parents[1]
REPO = SDK.parent
README = (SDK / "README.md").read_text(encoding="utf-8")
EXAMPLES = (SDK / "examples" / "README.md").read_text(encoding="utf-8")


def test_examples_publish_only_the_site_lifecycle():
    program = (SDK / "examples" / "run_site.py").read_text(encoding="utf-8")
    ast.parse(program)
    assert "waddle_sdk.load_site(" in program
    assert "with site.open(" in program
    assert "with session.run(" in program
    for removed in ("waddle_sdk.init(", "waddle_sdk.agent(", "waddle_sdk.ui()"):
        assert removed not in program
        assert removed not in EXAMPLES


def test_camera_install_commands_are_held_to_package_metadata():
    with (SDK / "pyproject.toml").open("rb") as stream:
        extras = tomllib.load(stream)["project"]["optional-dependencies"]

    assert extras["orbbec"] == ["pyorbbecsdk2"]
    assert extras["realsense"] == ["pyrealsense2"]
    assert extras["usb"] == ["opencv-python-headless>=4.8"]
    assert set(extras["cameras"]) == set(
        extras["orbbec"] + extras["realsense"] + extras["usb"]
    )
    for extra in ("orbbec", "realsense", "usb", "cameras"):
        assert f"waddle-sdk[{extra}]" in README
    assert "waddle-sdk[cameras,media]" in README


def test_readme_holds_the_site_and_runtime_boundaries():
    prose = " ".join(README.split())
    assert 'waddle_sdk.load_site("site.yaml")' in README
    assert "SdkRuntimePort" in README
    assert "Guided calibration orchestration belongs to Metal" in prose
    assert "hold-first" in prose
    assert "waddle.execution.v1" not in README
    assert "waddle_sdk.ui()" not in README
    assert "waddle_sdk.agent(" not in README
    assert "waddle_sdk.init(" not in README
    assert "ConnectorRegistrationError" in README
    assert "ConnectorCompatibilityWarning" in README
    assert set(waddle_sdk.__all__) == {
        "ConnectorCompatibilityWarning",
        "ConnectorRegistrationError",
        "Grpc",
        "LiveKit",
        "ManifestError",
        "ManifestPathError",
        "ManifestSyntaxError",
        "ManifestValidationError",
        "Outcome",
        "Run",
        "Site",
        "SiteSession",
        "load_site",
    }
    for removed in (
        "Control",
        "Handoff",
        "init",
        "shutdown",
        "rollout",
        "agent",
        "ui",
        "task_session",
        "calibration_click",
        "calibration_updates",
        "request_workspace_artifact",
        "execution_backends",
    ):
        assert not hasattr(waddle_sdk, removed)
    assert not (SDK / "python" / "waddle_sdk" / "_ui.py").exists()
    assert not (SDK / "python" / "waddle_sdk" / "_services.py").exists()
    assert not (SDK / "python" / "waddle_sdk" / "_testing.py").exists()


def test_release_is_gated_and_the_distribution_pair_is_atomic():
    ci_path = REPO / ".github" / "workflows" / "ci.yml"
    release_path = REPO / ".github" / "workflows" / "release.yml"
    ci = ci_path.read_text(encoding="utf-8")
    release_text = release_path.read_text(encoding="utf-8")
    jobs = yaml.safe_load(release_text)["jobs"]

    assert jobs["quality"]["uses"] == "./.github/workflows/ci.yml"
    assert jobs["skills-bundle"]["needs"] == ["quality"]
    assert jobs["wheels"]["needs"] == ["quality"]
    assert jobs["media-wheels"]["needs"] == ["quality"]
    release_gate = ["wheels", "media-wheels", "skills-bundle"]
    assert jobs["publish-sdk"]["needs"] == release_gate
    assert jobs["publish-media"]["needs"] == release_gate
    assert jobs["publish-github-release"]["needs"] == [
        "publish-sdk",
        "publish-media",
        "skills-bundle",
    ]
    assert jobs["publish-github-release"]["if"] == "github.ref_type == 'tag'"
    assert jobs["publish-sdk"]["environment"] == "pypi"
    assert jobs["publish-media"]["environment"] == "pypi-media"
    assert "publish-teleop" not in jobs
    expected_platforms = {
        ("linux-x86_64", "ubuntu-24.04", "x86_64"),
        ("linux-aarch64", "ubuntu-24.04-arm", "aarch64"),
        ("macos-arm64", "macos-latest", "aarch64"),
        ("macos-x86_64", "macos-15-intel", "x86_64"),
        ("windows-x64", "windows-latest", "x64"),
    }
    media_platforms = jobs["media-wheels"]["strategy"]["matrix"]["platform"]
    base_platforms = jobs["wheels"]["strategy"]["matrix"]["platform"]
    assert {
        (item["name"], item["runner"], item["target"]) for item in media_platforms
    } == expected_platforms
    assert media_platforms == base_platforms
    assert "MACOSX_DEPLOYMENT_TARGET=12.3" in release_text
    assert "waddle-sdk-skills-${GITHUB_REF_NAME}.zip" in release_text
    assert "zipfile.ZIP_STORED" in release_text
    assert "Cannot prove that GitHub Release" in release_text
    assert '404) ;;' in release_text
    assert '"skills", "list", "--json"' in release_text
    assert '"skills", "export", skill' in release_text
    release_commands = "\n".join(
        str(step.get("run", ""))
        for job in jobs.values()
        for step in job.get("steps", ())
        if isinstance(step, dict)
    )
    assert "--clobber" not in release_commands
    assert "continue-on-error" not in str(jobs)

    for gate in (
        "uv run --no-sync pytest",
        "uv build --wheel --out-dir dist",
        '"skills", "list", "--json"',
        '"skills", "export", skill',
        "cargo test --workspace --locked",
        "cargo clippy --workspace --all-targets --locked -- -D warnings",
        "cargo test -p waddle-controlplane --features tonic-transport --locked",
        "cargo test -p waddle-media --features livekit --locked",
        "cargo clippy -p waddle-runtime --features grpc,livekit --all-targets --locked -- -D warnings",
        "python -m pip install --require-hashes -r docs/requirements.txt",
        "cargo doc --workspace --exclude xtask --no-deps --locked",
        "python -m mkdocs build --strict",
    ):
        assert gate in ci


def test_public_docs_are_strict_versioned_and_current():
    mkdocs = yaml.safe_load((REPO / "mkdocs.yml").read_text(encoding="utf-8"))
    readthedocs = yaml.safe_load(
        (REPO / ".readthedocs.yaml").read_text(encoding="utf-8")
    )

    assert mkdocs["strict"] is True
    snippets = next(
        extension["pymdownx.snippets"]
        for extension in mkdocs["markdown_extensions"]
        if isinstance(extension, dict) and "pymdownx.snippets" in extension
    )
    assert snippets == {"base_path": ["."], "check_paths": True}
    assert mkdocs["plugins"][1]["mkdocstrings"]["handlers"]["python"]["options"][
        "allow_inspection"
    ] is False
    assert readthedocs["build"]["tools"] == {"python": "3.12", "rust": "1.96"}
    assert readthedocs["mkdocs"]["configuration"] == "mkdocs.yml"
    assert readthedocs["python"]["install"] == [
        {"requirements": "docs/requirements.txt"}
    ]

    nav = str(mkdocs["nav"])
    assert "Agent skills" in nav
    assert "Port custom hardware" in nav
    assert "connect" not in nav.lower()
    assert "mcp" not in nav.lower()
    assert "Normative glossary" in nav
    assert "Normative FSM" in nav
    assert "Normative versioning rules" in nav

    published = [
        REPO / "docs" / "index.md",
        REPO / "docs" / "lease-lifecycle.md",
        REPO / "docs" / "hardware-backends.md",
        *(REPO / "docs" / "concepts").glob("*.md"),
        *(REPO / "docs" / "core").glob("*.md"),
        *(REPO / "docs" / "python").glob("*.md"),
        *(REPO / "docs" / "porting").glob("*.md"),
    ]
    prose = "\n".join(path.read_text(encoding="utf-8") for path in published)
    assert "github.com/waddlelabs/waddle-sdk/blob/main/waddle-protocol/docs" not in prose
    for stale in (
        "waddle-sdk connect",
        "waddle_sdk.init(",
        "waddle_sdk.rollout(",
        "waddle_sdk.agent(",
        "waddle_sdk.Handoff",
        "MCP",
    ):
        assert stale not in prose

    requirements = (REPO / "docs" / "requirements.txt").read_text(encoding="utf-8")
    assert "--hash=sha256:" in requirements


def test_published_docs_do_not_expose_internal_process_vocabulary():
    published_markdown = [
        REPO / "README.md",
        SDK / "README.md",
        REPO / "waddle-protocol" / "docs" / "GLOSSARY.md",
        REPO / "waddle-protocol" / "docs" / "FSM.md",
        REPO / "waddle-protocol" / "docs" / "VERSIONING.md",
        REPO / "docs" / "index.md",
        REPO / "docs" / "lease-lifecycle.md",
        REPO / "docs" / "hardware-backends.md",
        *(REPO / "docs" / "concepts").glob("*.md"),
        *(REPO / "docs" / "core").glob("*.md"),
        *(REPO / "docs" / "python").glob("*.md"),
        *(REPO / "docs" / "porting").glob("*.md"),
        *(
            SDK / "python" / "waddle_sdk" / "agent_skills"
        ).rglob("*.md"),
    ]
    public_api_text = [
        *(path.read_text(encoding="utf-8") for path in published_markdown),
        *(
            path.read_text(encoding="utf-8")
            for path in (REPO / "waddle-protocol" / "proto").rglob("*.proto")
        ),
        *(
            "\n".join(
                line
                for line in path.read_text(encoding="utf-8").splitlines()
                if line.lstrip().startswith(("///", "//!"))
            )
            for path in (REPO / "waddle-core").rglob("*.rs")
            if "target" not in path.parts
        ),
    ]
    rendered_sources = "\n".join(public_api_text).lower()
    for restricted in ("bri" + "dge", "bro" + "ker"):
        assert restricted not in rendered_sources
    assert "reserved words" not in rendered_sources


def test_generated_extension_ignore_uses_the_current_package_name():
    ignore = (REPO / ".gitignore").read_text(encoding="utf-8")
    assert "sdk/python/waddle_sdk/_core*.so" in ignore
    assert "sdk/python/waddle/_core*.so" not in ignore
