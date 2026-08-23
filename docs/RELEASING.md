# Releasing `waddle-sdk`

How a version of the Python frontend gets to PyPI. The pipeline is
[`.github/workflows/release.yml`](../.github/workflows/release.yml); this page is the
checklist around it, plus the live trusted-publisher configuration that every release
depends on.

## What ships

Two distributions, built from one source tree (CLAUDE.md, "Two distributions from one
source tree"):

| Distribution | Project file | Module | Cargo features |
| --- | --- | --- | --- |
| `waddle-sdk` | `sdk/pyproject.toml` | `waddle_sdk._core` | `pyo3/extension-module`, `grpc` |
| `waddle-sdk-teleop` | `sdk/teleop/pyproject.toml` | `waddle_teleop._core` | the same, plus `livekit` |

Both are one build of `sdk/rust/Cargo.toml`, so they cannot disagree on a version, and
`waddle_sdk._native` refuses a mismatched pair at import rather than loading a core built
from other sources. The companion is installed as the extra — `pip install
'waddle-sdk[teleop]'` — never by name.

**Wheels only, no sdist, on purpose.** Both `[tool.maturin] manifest-path`s point at
`sdk/rust/Cargo.toml`, whose dependencies are path deps into
`../../waddle-core/crates/*`. Those escape both pyproject directories, so any sdist
maturin could produce would be an archive nobody can build. Every supported interpreter
is covered by one abi3 wheel per platform instead (pyo3 `abi3-py310`: a `cp310-abi3`
wheel installs on 3.10+).

## PyPI trusted publisher configuration

Both projects already exist on PyPI: the successful `v0.0.0` release on 2026-08-05
claimed `waddle-sdk` and `waddle-sdk-teleop` and converted their pending publishers into
ordinary trusted publishers. The matching GitHub environments also already exist.
There is no API token or publishing secret in this repository.

The live publisher identities must remain exactly:

| Field | `waddle-sdk` | `waddle-sdk-teleop` |
| --- | --- | --- |
| Owner | `waddlelabs` | `waddlelabs` |
| Repository name | `waddle-sdk` | `waddle-sdk` |
| Workflow name | `release.yml` | `release.yml` |
| Environment name | `pypi` | **`pypi-teleop`** |

Owner and repository are the GitHub coordinates. Workflow name is the file name, not
the workflow's display name, and environment names are case-sensitive. The environments
differ because PyPI keys a publisher on the owner/repository/workflow/environment tuple;
each publish job therefore keeps its own environment and downloads only its own
distribution's prefixed artifacts. Renaming the workflow, repository, owner, or either
environment requires updating the corresponding trusted publisher on PyPI before the
next release.

If a `waddlelabs` PyPI organization is wanted later, create it and transfer both
projects in; the trusted publisher configuration travels with each project.

## Cutting a release

Everything below happens on `main`, with the tree clean and the full local gate green.

1. **Bump the version in both places.** The version lives in
   `sdk/rust/Cargo.toml` (`[package] version`), and maturin derives both wheels' version
   from it. The exception is the extra's pin:

   - `sdk/rust/Cargo.toml` → `version = "X.Y.Z"`
   - `sdk/pyproject.toml` → `teleop = ["waddle-sdk-teleop==X.Y.Z"]`

   That literal is the ONE version maturin cannot derive (PEP 621 has no dynamic
   optional-dependencies). Forget it and `pip install 'waddle-sdk[teleop]'` resolves the
   *previous* release, `waddle_sdk._native` sees the mismatch, and the install silently has
   no LiveKit. `sdk/tests/test_features.py::test_the_teleop_extra_pins_this_builds_version`
   fails until the two agree, and the publish job re-checks the built wheels against the
   tag.

2. **Run the gates** (CLAUDE.md, "Build & test") — at minimum, from `sdk/`:

   ```sh
   uv sync --dev && uv run pytest
   cargo clippy --manifest-path rust/Cargo.toml --all-targets -- -D warnings
   cargo fmt --manifest-path rust/Cargo.toml --check
   ```

   and the `waddle-core` workspace's own tests/clippy/fmt, including the feature-gated
   passes. `.github/workflows/ci.yml` runs the complete gate on every push and pull
   request; the release workflow calls that same reusable workflow before either wheel
   build and before either publishing identity can run.

3. **Stow the changelog.** Move the `[Unreleased]` content into
   `docs/changelogs/CHANGELOG-X.Y.Z.md`, reset root `CHANGELOG.md` to `[Unreleased]`
   plus the pointer list (standing obligation 2 in CLAUDE.md).

4. **Commit, tag, push:**

   ```sh
   git commit -am "release: vX.Y.Z"
   git tag vX.Y.Z
   git push origin main
   git push origin vX.Y.Z     # <- this is what triggers the pipeline
   ```

   The tag is the trigger (`on: push: tags: ['v*']`). Pushing the branch alone builds
   nothing.

5. **Watch the run** (`gh run watch`, or the Actions tab). It builds five default wheels
   (linux x86_64/aarch64, macOS arm64/x86_64, Windows x64) and the teleop wheel, imports
   each one before uploading it, then publishes each distribution from its own job —
   `publish-sdk` (environment `pypi`) and `publish-teleop` (environment `pypi-teleop`).
   Both must be green for the release to be complete.

6. **Verify from the outside**, on a machine that is not this checkout:

   ```sh
   pip install waddle-sdk
   python -c "import waddle_sdk, waddle_sdk._native as n; print(waddle_sdk.__version__, sorted(n.FEATURES))"
   # -> X.Y.Z ['grpc']

   pip install 'waddle-sdk[teleop]'                       # linux x86_64 only, for now
   python -c "import waddle_sdk, waddle_sdk._native as n; print(waddle_sdk.__version__, sorted(n.FEATURES))"
   # -> X.Y.Z ['grpc', 'livekit']
   ```

## What is honestly supported, right now

- `pip install waddle-sdk` — linux x86_64, linux aarch64, macOS arm64, macOS x86_64,
  Windows x64. Python 3.10+ everywhere (one abi3 wheel per platform).
- `pip install 'waddle-sdk[teleop]'` — **linux x86_64 only.** The teleop matrix stays at
  one platform until the libwebrtc side of the build is audited on the others: its
  extension links a prebuilt libwebrtc downloaded at build time, and whether the result
  survives `auditwheel` (and the equivalent on macOS/Windows) has not been proven on any
  other target. On every other platform the extra simply fails to resolve — which is the
  intended failure: loud, at install time, rather than a session that quietly has no
  media plane. Say this in the release notes.
- Free-threaded interpreters (3.13t/3.14t) are not built: abi3 does not cover them.

## When something goes wrong

- **The teleop job fails (usually auditwheel rejecting the libwebrtc-linked
  extension).** Neither distribution is published: both publish jobs wait on the
  complete default matrix and teleop wheel, and no leg uses `continue-on-error`. Fix the
  build (a newer `manylinux` container, or `before-script-linux` installing what the C++
  side wants) and rerun before publishing this version.
- **A build platform fails.** The whole `wheels` matrix gates `publish-sdk`, so a
  failing leg stops the default publish. Fix it, or drop that leg from the matrix for
  this release and say so. Never publish a partial set silently.
- **The publish step fails after some files uploaded.** PyPI does not allow overwriting
  a file, and neither `skip-existing` nor any other paper-over is enabled here. Bump to
  the next patch version and release again; do not try to re-upload.
- **Re-running after a fixed publish failure** (nothing uploaded yet): the
  `workflow_dispatch` trigger builds and publishes exactly what is on the ref you run it
  from. Use it deliberately — it will happily try to publish an already-published
  version, and fail.
- **PyPI rejects the OIDC exchange** ("invalid-publisher", or a 403 on upload). The
  identity it matches is the whole tuple: owner, repository, workflow *file* name, and
  environment. Check the failing job's `environment:` against the table above —
  `publish-sdk` must be `pypi` and `publish-teleop` must be `pypi-teleop`, and renaming
  either here means editing the trusted publisher on PyPI too.
