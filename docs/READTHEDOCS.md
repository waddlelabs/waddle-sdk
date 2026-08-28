# Read the Docs project setup

`.readthedocs.yaml`, `mkdocs.yml`, and `requirements.txt` make each build reproducible,
but the following one-time project settings live in the Read the Docs control plane.
They are operator setup, not part of the public navigation.

## Initial project settings

1. Import `waddlelabs/waddle-sdk` and retain its GitHub integration/webhook.
2. Set the default branch to `main`.
3. Keep the built-in `latest` version active; Read the Docs builds it from the default
   branch (`main`).
4. Enable **Build pull requests for this project** under
   **Settings → Pull request builds** so every PR receives a preview URL and build
   status. A pull request opened before this setting was enabled needs a new commit to
   trigger its first build.

## Version automation

Create this rule in **Admin → Automation Rules**:

| Version type | Match | Action |
|---|---|---|
| Tag | SemVer versions, or regex `^v[0-9]+\.[0-9]+\.[0-9]+$` | Activate version |

Read the Docs' `stable` special version then resolves to the newest active release tag,
while `latest` continues to resolve to `main`; do not create a redundant raw `main`
version. Automation rules apply only to versions created after the rule. If the current
release tag already exists, activate it once under **Versions**, then set the project's
default displayed version to `stable` under **Settings**. Do not activate arbitrary
development branches. Release tags are immutable; rebuild rather than retarget a tag.

## Verification

After configuring the project, verify one build of `latest`, one tagged `stable`
build, and one pull-request preview. Each must install `docs/requirements.txt` with
hash checking, generate warning-clean Rust API docs from the same revision, and run
MkDocs in strict mode. No command in this note performs the external setup.
