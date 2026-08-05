# Releasing `waddle-sdk`

How a version of the Python frontend gets to PyPI. The pipeline is
[`.github/workflows/release.yml`](../.github/workflows/release.yml); this page is the
checklist around it, plus the one-time account setup that has to happen before the
first release can work at all.

## What ships

Two distributions, built from one source tree (CLAUDE.md, "Two distributions from one
source tree"):

| Distribution | Project file | Module | Cargo features |
| --- | --- | --- | --- |
| `waddle-sdk` | `sdk/pyproject.toml` | `waddle._core` | `pyo3/extension-module`, `grpc` |
| `waddle-sdk-teleop` | `sdk/teleop/pyproject.toml` | `waddle_teleop._core` | the same, plus `livekit` |

Both are one build of `sdk/rust/Cargo.toml`, so they cannot disagree on a version, and
`waddle._native` refuses a mismatched pair at import rather than loading a core built
from other sources. The companion is installed as the extra — `pip install
'waddle-sdk[teleop]'` — never by name.

**Wheels only, no sdist, on purpose.** Both `[tool.maturin] manifest-path`s point at
`sdk/rust/Cargo.toml`, whose dependencies are path deps into
`../../waddle-core/crates/*`. Those escape both pyproject directories, so any sdist
maturin could produce would be an archive nobody can build. Every supported interpreter
is covered by one abi3 wheel per platform instead (pyo3 `abi3-py310`: a `cp310-abi3`
wheel installs on 3.10+).

## One-time PyPI setup (Vincent, before the first release)

Neither project exists on PyPI yet. Both names get **claimed by the first CI publish**,
via a *pending* trusted publisher — no placeholder upload, no API token, ever.

1. Create a PyPI account at <https://pypi.org/account/register/> and enable 2FA
   (PyPI requires it for uploads; a TOTP app plus the recovery codes stored somewhere
   that is not this repo).
2. Go to <https://pypi.org/manage/account/publishing/> — "Add a new **pending**
   publisher", GitHub tab — and add it **twice**, once per project name, with exactly
   these values:

   | Field | First entry | Second entry |
   | --- | --- | --- |
   | PyPI Project Name | `waddle-sdk` | `waddle-sdk-teleop` |
   | Owner | `waddlelabs` | `waddlelabs` |
   | Repository name | `waddle-sdk` | `waddle-sdk` |
   | Workflow name | `release.yml` | `release.yml` |
   | Environment name | `pypi` | `pypi` |

   (Owner and repository are the GitHub coordinates —
   `github.com/waddlelabs/waddle-sdk`. Workflow name is the *file* name, not the `name:`
   inside it. Environment name must match the `environment: pypi` on the publish job;
   it is case-sensitive.)
3. Optional, in GitHub: **Settings → Environments → New environment → `pypi`**. The
   workflow works without it being pre-created, but creating it is where a required
   reviewer / approval gate would go if releases should ever pause for a human.
4. Nothing else. There is no token to generate, no secret to add to the repo, and
   nothing to paste anywhere.

The first successful run converts both pending publishers into ordinary trusted
publishers attached to the now-existing projects. If a `waddlelabs` PyPI **organization**
is wanted later, create it and transfer both projects in — the trusted publisher config
travels with the project, so the pipeline keeps working untouched.

## Cutting a release

Everything below happens on `main`, with the tree clean and the full local gate green.

1. **Bump the version in both places.** The version lives in
   `sdk/rust/Cargo.toml` (`[package] version`), and maturin derives both wheels' version
   from it. The exception is the extra's pin:

   - `sdk/rust/Cargo.toml` → `version = "X.Y.Z"`
   - `sdk/pyproject.toml` → `teleop = ["waddle-sdk-teleop==X.Y.Z"]`

   That literal is the ONE version maturin cannot derive (PEP 621 has no dynamic
   optional-dependencies). Forget it and `pip install 'waddle-sdk[teleop]'` resolves the
   *previous* release, `waddle._native` sees the mismatch, and the install silently has
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
   passes.

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
   each one it can, then publishes everything in a single step.

6. **Verify from the outside**, on a machine that is not this checkout:

   ```sh
   pip install waddle-sdk
   python -c "import waddle, waddle._native as n; print(waddle.__version__, sorted(n.FEATURES))"
   # -> X.Y.Z ['grpc']

   pip install 'waddle-sdk[teleop]'                       # linux x86_64 only, for now
   python -c "import waddle, waddle._native as n; print(waddle.__version__, sorted(n.FEATURES))"
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
  extension).** The release stops, by design — it is not `continue-on-error`, because a
  publish that ships half of what the extra resolves to is worse than no publish. Either
  fix it (a newer `manylinux` container, or `before-script-linux` installing what the
  C++ side wants), or consciously ship default-only for this release: drop
  `teleop-wheel` from the `publish` job's `needs`, note it in the release notes, and
  remember that `[teleop]` then resolves to the previous release or to nothing.
- **A build platform fails.** Same shape: fix it, or drop that leg from the matrix for
  this release and say so. Never publish a partial set silently.
- **The publish step fails after some files uploaded.** PyPI does not allow overwriting
  a file, and neither `skip-existing` nor any other paper-over is enabled here. Bump to
  the next patch version and release again; do not try to re-upload.
- **Re-running after a fixed publish failure** (nothing uploaded yet): the
  `workflow_dispatch` trigger builds and publishes exactly what is on the ref you run it
  from. Use it deliberately — it will happily try to publish an already-published
  version, and fail.
- **PyPI rejects the OIDC exchange for one of the two projects.** The minted token
  covers the projects the publisher identity is registered for, which is why both go up
  in one step. If that ever changes, split the publish into two
  `pypa/gh-action-pypi-publish` steps with their own `packages-dir` (one per
  distribution) in the same job.
