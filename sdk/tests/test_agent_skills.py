from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

import waddle_sdk
from waddle_sdk import cli
from waddle_sdk.agent_skills import bundled_skills


SKILL_ROOT = (
    Path(__file__).resolve().parents[1]
    / "python"
    / "waddle_sdk"
    / "agent_skills"
)
PORTING = SKILL_ROOT / "port-waddle-hardware"
SCAFFOLD = PORTING / "scripts" / "scaffold_adapter.py"
VALIDATE = PORTING / "scripts" / "validate_adapter.py"


def test_bundled_skills_are_complete_and_sorted() -> None:
    skills = bundled_skills()
    assert [skill.name for skill in skills] == [
        "port-waddle-hardware",
        "waddle-sdk-contracts",
    ]
    for skill in skills:
        assert skill.description
        assert skill.resource.joinpath("SKILL.md").is_file()
        assert skill.resource.joinpath("agents", "openai.yaml").is_file()


def test_skill_metadata_and_resource_links_are_strict() -> None:
    skill_roots = sorted(
        path
        for path in SKILL_ROOT.iterdir()
        if path.is_dir() and (path / "SKILL.md").is_file()
    )
    assert [path.name for path in skill_roots] == [
        "port-waddle-hardware",
        "waddle-sdk-contracts",
    ]
    for skill_root in skill_roots:
        skill_md = skill_root / "SKILL.md"
        text = skill_md.read_text(encoding="utf-8")
        _opening, frontmatter, _body = text.split("---", 2)
        metadata = yaml.safe_load(frontmatter)
        assert set(metadata) == {"name", "description"}
        assert metadata["name"] == skill_root.name
        assert isinstance(metadata["description"], str) and metadata["description"]
        assert len(text.splitlines()) <= 500
        assert "[TODO" not in text and "TODO:" not in text

        agent = yaml.safe_load(
            (skill_root / "agents" / "openai.yaml").read_text(encoding="utf-8")
        )
        assert set(agent) == {"interface"}
        interface = agent["interface"]
        assert set(interface) == {
            "display_name",
            "short_description",
            "default_prompt",
        }
        assert 1 <= len(interface["display_name"]) <= 64
        assert 1 <= len(interface["short_description"]) <= 64
        assert f"${skill_root.name}" in interface["default_prompt"]

        for markdown in skill_root.rglob("*.md"):
            for target in re.findall(r"\[[^]]+\]\(([^)]+)\)", markdown.read_text()):
                if target.startswith(("#", "http://", "https://")):
                    continue
                resolved = (markdown.parent / target.split("#", 1)[0]).resolve()
                assert resolved.is_relative_to(skill_root.resolve())
                assert resolved.exists(), f"broken skill link: {markdown} -> {target}"


def test_skills_list_reports_installed_sdk_version(capsys) -> None:
    assert cli.main(["skills", "list", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["sdk_version"] == waddle_sdk.__version__
    assert [row["name"] for row in payload["skills"]] == [
        "port-waddle-hardware",
        "waddle-sdk-contracts",
    ]


def test_export_preserves_bytes_and_script_modes_and_refuses_overwrite(
    tmp_path: Path,
    capsys,
) -> None:
    assert (
        cli.main(
            [
                "skills",
                "export",
                "port-waddle-hardware",
                "--output",
                str(tmp_path),
            ]
        )
        == 0
    )
    target = tmp_path / "port-waddle-hardware"
    assert target == tmp_path / "port-waddle-hardware"
    assert (
        capsys.readouterr().out
        == f"exported port-waddle-hardware from waddle-sdk "
        f"{waddle_sdk.__version__} to {target}\n"
    )
    for source in PORTING.rglob("*"):
        if source.is_file():
            exported = target / source.relative_to(PORTING)
            assert exported.read_bytes() == source.read_bytes()
    assert os.access(target / "scripts" / "scaffold_adapter.py", os.X_OK)
    assert os.access(target / "scripts" / "validate_adapter.py", os.X_OK)
    with pytest.raises(SystemExit, match="refusing to overwrite"):
        cli.main(
            [
                "skills",
                "export",
                "port-waddle-hardware",
                "--output",
                str(tmp_path),
            ]
        )


def _run(*arguments: str, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, *arguments],
        cwd=cwd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


@pytest.mark.parametrize(
    ("kind", "name", "arguments"),
    [
        (
            "robot",
            "acme-arm",
            [
                "--facts-source",
                "Acme Arm manual rev 4, table 2",
                "--joint",
                "shoulder:-2:2:0.02",
                "--joint",
                "elbow:-1.5:1.5:0.01",
                "--rate-hz",
                "50",
            ],
        ),
        (
            "simulator",
            "acme-sim",
            [
                "--facts-source",
                "Synthetic one-axis test model v1",
                "--joint",
                "axis:-1:1:0.01",
                "--home",
                "0",
                "--rate-hz",
                "20",
            ],
        ),
        (
            "camera",
            "acme-camera",
            [
                "--facts-source",
                "Acme Camera stream profile rev 2",
                "--width",
                "64",
                "--height",
                "48",
                "--fps",
                "30",
            ],
        ),
    ],
)
def test_scaffolds_validate_statically_and_pass_fake_vendor_tests(
    tmp_path: Path,
    kind: str,
    name: str,
    arguments: list[str],
) -> None:
    result = _run(
        str(SCAFFOLD),
        kind,
        "--name",
        name,
        "--output",
        str(tmp_path),
        *arguments,
    )
    assert "no hardware was opened" in result.stdout
    project = tmp_path / name
    package = name.replace("-", "_")
    facts_source = arguments[arguments.index("--facts-source") + 1]
    assert (
        f"FACTS_SOURCE = {facts_source!r}"
        in (project / "src" / package / "backend.py").read_text(encoding="utf-8")
    )
    assert (
        f'waddle-sdk=={waddle_sdk.__version__}'
        in (project / "pyproject.toml").read_text(encoding="utf-8")
    )
    validation = _run(
        str(VALIDATE),
        str(project),
        "--site",
        "site.example.yaml",
    )
    assert "adapter modules were not imported" in validation.stdout
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(project / "src")
    subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(project / "tests")],
        cwd=project,
        env=environment,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


@pytest.mark.parametrize("unsafe", ["../escape", "nested/name", ".", "a.b"])
def test_scaffold_rejects_names_that_can_escape_the_output(
    tmp_path: Path, unsafe: str
) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            str(SCAFFOLD),
            "simulator",
            "--name",
            unsafe,
            "--output",
            str(tmp_path),
            "--facts-source",
            "Synthetic traversal-name test model",
            "--joint",
            "axis:-1:1:0.01",
            "--home",
            "0",
            "--rate-hz",
            "20",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert completed.returncode != 0
    assert not (tmp_path.parent / "escape").exists()


@pytest.mark.parametrize(
    "driver",
    [
        "/tmp/waddle-sdk-validator-escape:arm",
        "validator_sim..backend:arm",
    ],
)
def test_static_validator_rejects_unsafe_module_paths(
    tmp_path: Path, driver: str
) -> None:
    _run(
        str(SCAFFOLD),
        "simulator",
        "--name",
        "validator-sim",
        "--output",
        str(tmp_path),
        "--facts-source",
        "Synthetic validator-path test model",
        "--joint",
        "axis:-1:1:0.01",
        "--home",
        "0",
        "--rate-hz",
        "20",
    )
    project = tmp_path / "validator-sim"
    site = project / "site.example.yaml"
    site.write_text(
        site.read_text(encoding="utf-8").replace(
            "validator_sim.backend:arm", driver
        ),
        encoding="utf-8",
    )
    completed = subprocess.run(
        [
            sys.executable,
            str(VALIDATE),
            str(project),
            "--site",
            "site.example.yaml",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert completed.returncode != 0
    assert "strict dotted Python identifiers" in completed.stdout


@pytest.mark.parametrize(
    "import_time_code",
    [
        """
def configure_hardware(cls):
    return cls

@configure_hardware
class DecoratedAtImport:
    pass
""",
        """
def configure_hardware():
    return object()

if True:
    class NestedAtImport:
        opened = configure_hardware()
""",
        """
def configure_hardware():
    return object()

def configured_at_import(
    value: configure_hardware() = configure_hardware(),
):
    return value

class BasedAtImport(configure_hardware(), metaclass=configure_hardware()):
    pass
""",
    ],
)
def test_static_validator_rejects_calls_in_all_import_time_scopes(
    tmp_path: Path, import_time_code: str
) -> None:
    _run(
        str(SCAFFOLD),
        "simulator",
        "--name",
        "import-scan-sim",
        "--output",
        str(tmp_path),
        "--facts-source",
        "Synthetic import-scan test model",
        "--joint",
        "axis:-1:1:0.01",
        "--home",
        "0",
        "--rate-hz",
        "20",
    )
    project = tmp_path / "import-scan-sim"
    backend = project / "src" / "import_scan_sim" / "backend.py"
    backend.write_text(
        backend.read_text(encoding="utf-8") + import_time_code,
        encoding="utf-8",
    )
    completed = subprocess.run(
        [
            sys.executable,
            str(VALIDATE),
            str(project),
            "--site",
            "site.example.yaml",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert completed.returncode != 0
    assert "module-scope calls" in completed.stdout


@pytest.mark.parametrize(
    "facts_arguments",
    [[], ["--facts-source", "   "]],
)
def test_scaffold_requires_an_explicit_facts_source(
    tmp_path: Path, facts_arguments: list[str]
) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            str(SCAFFOLD),
            "simulator",
            "--name",
            "missing-source-sim",
            "--output",
            str(tmp_path),
            *facts_arguments,
            "--joint",
            "axis:-1:1:0.01",
            "--home",
            "0",
            "--rate-hz",
            "20",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert completed.returncode != 0
    assert "--facts-source" in completed.stdout
    assert not (tmp_path / "missing-source-sim").exists()


def test_simulator_scaffold_requires_a_sourced_home(tmp_path: Path) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            str(SCAFFOLD),
            "simulator",
            "--name",
            "missing-home-sim",
            "--output",
            str(tmp_path),
            "--facts-source",
            "Synthetic missing-home test model",
            "--joint",
            "axis:-1:1:0.01",
            "--rate-hz",
            "20",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert completed.returncode != 0
    assert "--home" in completed.stdout
    assert not (tmp_path / "missing-home-sim").exists()


@pytest.mark.parametrize(
    ("name", "home"),
    [("lower-home-sim", "-1"), ("near-upper-home-sim", "0.99")],
)
def test_generated_simulator_tests_accept_home_near_source_bounds(
    tmp_path: Path, name: str, home: str
) -> None:
    _run(
        str(SCAFFOLD),
        "simulator",
        "--name",
        name,
        "--output",
        str(tmp_path),
        "--facts-source",
        "Synthetic boundary-home test model",
        "--joint",
        "axis:-1:1:0.01",
        "--home",
        home,
        "--rate-hz",
        "20",
    )
    project = tmp_path / name
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(project / "src")
    subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(project / "tests")],
        cwd=project,
        env=environment,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
