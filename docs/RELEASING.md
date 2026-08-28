# Releasing `waddle-sdk`

How a version of the Python frontend gets to PyPI and its matching Agent Skills get
to GitHub Releases. The pipeline is
[`.github/workflows/release.yml`](../.github/workflows/release.yml); this page is the
checklist around it, plus the live publishing configuration and recovery rules that
every release depends on.

## What ships

Two distributions, built from one source tree (CLAUDE.md, "Two distributions from one
source tree"):

| Distribution | Project file | Module | Cargo features |
| --- | --- | --- | --- |
| `waddle-sdk` | `sdk/pyproject.toml` | `waddle_sdk._core` | `pyo3/extension-module`, `grpc` |
| `waddle-sdk-media` | `sdk/media/pyproject.toml` | `waddle_media._core` | the same, plus `livekit` |

Both are one build of `sdk/rust/Cargo.toml`, so they cannot disagree on a version, and
`waddle_sdk._native` refuses a mismatched pair at import rather than loading a core built
from other sources. The companion is installed as the extra — `pip install
'waddle-sdk[media]'` — never by name.

The same release also carries two portable Agent Skills from the canonical
`sdk/python/waddle_sdk/agent_skills` package data:

- `waddle-sdk-contracts`
- `port-waddle-hardware`

The workflow archives those directories as `waddle-sdk-skills-vX.Y.Z.zip`, with a
top-level `SDK_VERSION` file, and publishes the archive plus
`waddle-sdk-skills-vX.Y.Z.zip.sha256` on the matching GitHub Release. The archive is
deterministic: its order, timestamps, permissions, compression, and generated version
file are fixed. The workflow rejects unexpected skill directories, symlinks, generated
Python bytecode, unfinished TODOs, missing `SKILL.md` files, frontmatter-name drift,
version/tag drift, duplicate archive members, and CRC/checksum failures.

Release assets are immutable by operator contract. The workflow fails if the GitHub
Release already exists and never passes an asset-overwrite option. Do not replace a
published archive in place; correct it in a new SDK patch release.

**Wheels only, no sdist, on purpose.** Both `[tool.maturin] manifest-path`s point at
`sdk/rust/Cargo.toml`, whose dependencies are path deps into
`../../waddle-core/crates/*`. Those escape both pyproject directories, so any sdist
maturin could produce would be an archive nobody can build. Every supported interpreter
is covered by one abi3 wheel per platform instead (pyo3 `abi3-py310`: a `cp310-abi3`
wheel installs on 3.10+).

## PyPI trusted publisher configuration

`waddle-sdk` already exists on PyPI. The `waddle-sdk-media` pending trusted publisher
and matching GitHub `pypi-media` environment are configured for its first release; a
successful first publish converts that pending publisher into an ordinary trusted
publisher. There is no API token or publishing secret in this repository. The retired
`waddle-sdk-teleop` project is not part of the new release pair.

The live publisher identities must remain exactly:

| Field | `waddle-sdk` | `waddle-sdk-media` |
| --- | --- | --- |
| Owner | `waddlelabs` | `waddlelabs` |
| Repository name | `waddle-sdk` | `waddle-sdk` |
| Workflow name | `release.yml` | `release.yml` |
| Environment name | `pypi` | **`pypi-media`** |

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
   - `sdk/pyproject.toml` → `media = ["waddle-sdk-media==X.Y.Z"]`

   That literal is the ONE version maturin cannot derive (PEP 621 has no dynamic
   optional-dependencies). Forget it and `pip install 'waddle-sdk[media]'` resolves the
   *previous* release, `waddle_sdk._native` sees the mismatch, and the install silently has
   no LiveKit. `sdk/tests/test_features.py::test_the_media_extra_pins_this_builds_version`
   fails until the two agree, and the publish job re-checks the built wheels against the
   tag.

2. **Run the gates** (CLAUDE.md, "Build & test") — at minimum, from `sdk/`:

   ```sh
   uv sync --dev && uv run pytest
   cargo clippy --manifest-path rust/Cargo.toml --all-targets -- -D warnings
   cargo fmt --manifest-path rust/Cargo.toml --check
   ```

   and the `waddle-core` workspace's own tests/clippy/fmt, including the feature-gated
   passes. Also build the public documentation exactly as CI does, from the repository
   root:

   ```sh
   python -m pip install --require-hashes -r docs/requirements.txt
   (cd waddle-core && RUSTDOCFLAGS="-D warnings" cargo doc --workspace --exclude xtask --no-deps --locked)
   mkdir -p docs/reference/rust
   cp -R waddle-core/target/doc/. docs/reference/rust/
   python -m mkdocs build --strict
   ```

   `.github/workflows/ci.yml` runs the complete Python, Rust, and strict documentation
   gates on every push and pull request. The release workflow calls that same reusable
   workflow before any wheel or Agent Skills build and before either publishing
   identity can run.

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

   The tag is the release trigger (`on: push: tags: ['v*']`). Pushing the branch alone
   runs CI but does not build release matrices, publish packages, or create a release.

5. **Watch the run** (`gh run watch`, or the Actions tab). After the reusable quality
   gate, it builds five default wheels and five media wheels (linux x86_64/aarch64,
   macOS arm64/x86_64, Windows x64), imports each one natively, and independently builds
   and validates the deterministic skills archive. Both PyPI jobs require all ten
   wheels and the skills archive: `publish-sdk` uses environment `pypi`, while
   `publish-media` uses `pypi-media`. Only after both PyPI publishes succeed does the
   tag-only `publish-github-release` job create the GitHub Release and attach the skills
   archive and checksum. All three publish jobs must be green for the release to be
   complete.

6. **Verify from the outside**, on a machine that is not this checkout:

   ```sh
   pip install waddle-sdk
   python -c "import waddle_sdk, waddle_sdk._native as n; print(waddle_sdk.__version__, sorted(n.FEATURES))"
   # -> X.Y.Z ['grpc']

   pip install 'waddle-sdk[media]'
   python -c "import waddle_sdk, waddle_sdk._native as n; print(waddle_sdk.__version__, sorted(n.FEATURES))"
   # -> X.Y.Z ['grpc', 'livekit']

   gh release download vX.Y.Z --repo waddlelabs/waddle-sdk --pattern 'waddle-sdk-skills-vX.Y.Z.zip*'
   sha256sum --check waddle-sdk-skills-vX.Y.Z.zip.sha256
   python -m zipfile --test waddle-sdk-skills-vX.Y.Z.zip
   unzip -p waddle-sdk-skills-vX.Y.Z.zip waddle-sdk-skills-vX.Y.Z/SDK_VERSION
   # -> X.Y.Z
   ```

   Confirm the archive contains both `waddle-sdk-contracts/SKILL.md` and
   `port-waddle-hardware/SKILL.md` below its one versioned top-level directory. Also
   confirm Read the Docs built the tag and that `stable` now resolves to the intended
   newest release; `latest` continues to document `main`.

## What is honestly supported, right now

- `pip install waddle-sdk` — linux x86_64, linux aarch64, macOS arm64, macOS x86_64,
  Windows x64. Python 3.10+ everywhere (one abi3 wheel per platform).
- `pip install 'waddle-sdk[media]'` — linux x86_64, linux aarch64, macOS arm64,
  macOS x86_64, and Windows x64. Each wheel links LiveKit's target-specific prebuilt
  libwebrtc and is imported on the same native architecture before publishing. macOS
  media wheels declare a 12.3+ deployment floor because the SDK links ScreenCaptureKit.
  The Windows media build selects Rust's static CRT to match LiveKit's prebuilt
  libwebrtc archive; do not remove that target-specific release setting or mix `/MT`
  and `/MD` objects in the extension.
  Windows ARM64 remains unsupported until both distributions have native wheel and import
  coverage; do not infer it from LiveKit merely publishing a libwebrtc archive.
- Free-threaded interpreters (3.13t/3.14t) are not built: abi3 does not cover them.
- `waddle-sdk-skills-vX.Y.Z.zip` — the two portable Agent Skills shipped in the base
  wheel, for explicit export or manual installation into a compatible coding-agent
  client. The archive itself grants no device access and contains no MCP server or live
  hardware-control tool.

## When something goes wrong

- **The documentation gate fails.** No release build or publisher runs. Fix Rustdoc
  warnings, MkDocs warnings, stale links, or the hash-locked documentation environment;
  do not weaken `RUSTDOCFLAGS=-D warnings`, `--locked`, `--require-hashes`, or MkDocs
  `--strict` for a release.
- **The Agent Skills bundle fails validation.** Neither PyPI publisher runs. Fix the
  canonical skill tree or version/tag mismatch and start a clean release run. Do not
  hand-build an archive around the failed gate.
- **A GitHub Release already exists when the tag run starts.** The skills job fails
  before either PyPI publish. Determine who created the release. If it is a real or
  externally consumed release, cut a new patch version. Only an empty, accidental
  release record may be removed and retried; leave the Git tag intact.
- **The media matrix fails (usually auditwheel or a platform linker rejecting the libwebrtc-linked
  extension).** Neither distribution is published: both publish jobs wait on the
  complete default and media matrices and the skills bundle, and no leg uses
  `continue-on-error`. Fix the build (a newer `manylinux` container, or
  `before-script-linux` installing what the C++ side wants) and rerun before publishing
  this version.
- **A build platform fails.** The whole `wheels` matrix gates `publish-sdk`, so a
  failing leg stops the default publish. Fix it, or drop that leg from the matrix for
  this release and say so. Never publish a partial set silently.
- **The publish step fails after some files uploaded.** PyPI does not allow overwriting
  a file, and neither `skip-existing` nor any other paper-over is enabled here. Bump to
  the next patch version and release again; do not try to re-upload.
- **Re-running after a fixed publish failure** (nothing uploaded yet): the
  `workflow_dispatch` trigger builds and publishes exactly what is on the ref you run it
  from. A branch dispatch retains the skills bundle only as a workflow artifact; a tag
  dispatch creates the GitHub Release after PyPI. Use dispatch deliberately — it will
  happily try to publish an already-published Python version, and fail.
- **PyPI rejects the OIDC exchange** ("invalid-publisher", or a 403 on upload). The
  identity it matches is the whole tuple: owner, repository, workflow *file* name, and
  environment. Check the failing job's `environment:` against the table above —
  `publish-sdk` must be `pypi` and `publish-media` must be `pypi-media`, and renaming
  either here means editing the trusted publisher on PyPI too.
- **PyPI succeeded but `publish-github-release` failed before creating a release.** Use
  GitHub Actions' **Re-run failed jobs** on that same run while its `release-skills`
  artifact is retained. This retries only the final job with the exact bytes already
  gated; do not dispatch the whole workflow, which would attempt forbidden PyPI
  re-uploads.
- **The final job left an incomplete GitHub Release record.** Inspect its assets and
  compare the published checksum first. The workflow will never overwrite it. If the
  exact archive and checksum are both present and validate, no recovery mutation is
  needed. Otherwise delete only that incomplete GitHub Release record (never the tag,
  and never use `--cleanup-tag`), then use **Re-run failed jobs** on the original run so
  it reuses the retained, gated artifact. If that artifact has expired, stop and choose
  an attended recovery from the exact tag; do not improvise an asset replacement or
  rerun the PyPI publishers.
