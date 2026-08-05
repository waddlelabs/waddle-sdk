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
   | Environment name | `pypi` | **`pypi-teleop`** |

   Owner and repository are the GitHub coordinates —
   `github.com/waddlelabs/waddle-sdk`. Workflow name is the *file* name, not the `name:`
   inside it. Environment names are case-sensitive and must match the `environment:` on
   the corresponding publish job.

   **The two environments differ because they have to.** PyPI keys a pending trusted
   publisher on the (owner, repository, workflow, environment) tuple and refuses a
   second registration of the same tuple under a different project name — "A pending
   trusted publisher matching this configuration has already been registered for a
   different project name". So the two projects cannot both publish from `pypi`; and
   since a GitHub job carries exactly one environment, that is also why
   `.github/workflows/release.yml` has two publish jobs (`publish-sdk` → `pypi`,
   `publish-teleop` → `pypi-teleop`) with artifact names prefixed per distribution, so
   each job downloads only what its identity is allowed to upload.
3. Optional, in GitHub: **Settings → Environments → New environment**, once for `pypi`
   and once for `pypi-teleop`. The workflow works without them being pre-created, but
   they are where a required reviewer / approval gate would go if releases should ever
   pause for a human.
4. Nothing else. There is no token to generate, no secret to add to the repo, and
   nothing to paste anywhere.

The first successful run converts each pending publisher into an ordinary trusted
publisher attached to the now-existing project. If a `waddlelabs` PyPI **organization**
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
   each one before uploading it, then publishes each distribution from its own job —
   `publish-sdk` (environment `pypi`) and `publish-teleop` (environment `pypi-teleop`).
   Both must be green for the release to be complete.

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
  extension).** Nothing teleop is published — it is not `continue-on-error`, because a
  green job that uploaded no companion wheel would be a release claiming an extra it did
  not ship. But note the shape the two-environment split forces: `publish-sdk` does not
  wait on the teleop build, so **the default wheels still go out** and the release
  becomes default-only on its own. Either fix the build (a newer `manylinux` container,
  or `before-script-linux` installing what the C++ side wants) and re-release, or accept
  it — and say so in the release notes, because `pip install 'waddle-sdk[teleop]'` then
  resolves to the previous release, or to nothing at all on the first one. If a release
  should instead be all-or-nothing, add `teleop-wheel` to `publish-sdk`'s `needs`.
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
