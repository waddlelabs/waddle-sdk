"""Manifest parsers preserve validation and refuse Python YAML constructors."""

import pytest
import yaml


@pytest.mark.parametrize("python_fallback", [False, True])
@pytest.mark.parametrize("kind", ["site", "scene"])
def test_safe_parsers_keep_manifest_semantics(
    tmp_path, monkeypatch, python_fallback, kind
):
    from test_scene_compiler import _write_scene
    from test_site_api import _write_site
    from waddle_sdk import scene, site

    if not python_fallback and not hasattr(yaml, "CSafeLoader"):
        pytest.skip("optional LibYAML is unavailable")
    module, write = (site, _write_site) if kind == "site" else (scene, _write_scene)
    path = write(tmp_path)
    load = module.load_site if kind == "site" else module.load_scene
    expected = dict(load(path).manifest)
    if python_fallback:
        monkeypatch.delattr(yaml, "CSafeLoader", raising=False)
    assert dict(load(path).manifest) == expected
    assert expected == yaml.safe_load(path.read_text())
    invalid = expected | {"unexpected": True}
    path.write_text(yaml.safe_dump(invalid))
    validation_error = (
        site.ManifestValidationError if kind == "site" else scene.SceneValidationError
    )
    with pytest.raises(validation_error):
        load(path)
    syntax_error = (
        site.ManifestSyntaxError if kind == "site" else scene.SceneSyntaxError
    )
    for document in ("!!python/object/new:builtins.list []", "key: [unterminated"):
        path.write_text(document)
        with pytest.raises(syntax_error):
            load(path)
