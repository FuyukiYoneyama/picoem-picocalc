# Versioning and release pairing

This repository is an independent PicoCalc backend distribution. Its upstream
history is retained, but upstream commit numbers and the `main` branch are not
user-facing release markers.

## What users should download

Use a GitHub Release and its immutable SemVer tag. Do not use the moving
`main` branch as a reproducible dependency.

```text
main             development head; no stability promise
ordinary commit  development history
vX.Y.Z tag       immutable source point
GitHub Release   user-facing notes and compatibility information
```

The first public technical-preview pair is intended to use `v0.1.0`. A tag is
not a release until it has been deliberately created and published; ordinary
pushes do not create a release.

The backend release number is independent from the PicoCalc BSP version and
from the report/machine-API schema versions. Those values are recorded
separately in the paired `picocalc_emu` release notes.

## Pairing with `picocalc_emu`

For a supported PicoCalc firmware run, use the exact backend tag and commit
listed by the matching `picocalc_emu` Release. Do not substitute the current
backend `main` for a registry-pinned commit.

The pairing record contains at least:

- `picocalc_emu` tag and full commit SHA
- this backend tag and full commit SHA
- BSP version, report schema, and machine API schema
- Rust/toolchain requirements and local validation results
- known limitations and whether an optional historical external workspace is needed

The target registry remains the executable authority: it pins the backend
commit, firmware artifact, scenario, and expected report. A Git tag is a human
entry point; the full SHA is the reproducibility anchor.

## Getting a tagged checkout

Use Git so that provenance and submodule metadata are retained:

```sh
git clone --branch v0.1.0 --recursive \
  https://github.com/FuyukiYoneyama/picoem-picocalc.git
```

GitHub's `Code -> Download ZIP` is suitable for source browsing, but it does
not contain Git metadata and is not the recommended input for provenance-bound
PicoCalc validation.

## Version changes

- PATCH: compatible fixes and documentation changes
- MINOR: backward-compatible capabilities or target additions
- MAJOR: incompatible changes to the backend interface or the contracts consumed
  by `picocalc_emu`

Commits and pushes may be made in small batches during development. Create a
new tag only after the local release gate passes and the paired emulator commit
has been recorded. Never move or force-push a published tag; issue a new patch
release if a published release is wrong.

The canonical cross-repository release policy is maintained in
[`picocalc_emu/docs/VERSIONING.md`](https://github.com/FuyukiYoneyama/picocalc_emu/blob/main/docs/VERSIONING.md).
