# picoem-picocalc v0.2.0 — RP2040 SIO divider correction

Date: 2026-10-12. This is a paired source release and public technical preview.
Use the immutable annotated `v0.2.0` tag in both repositories. The GitHub Release
notes give the exact final commit SHAs and downloadable local validation logs.

## Corrected divider

The implementation commit is `d91f2bb7189b3a534cec01f8ae3670fd3a22297a`.
Serial and Threaded now share an RP2040 per-core divider model:

- Every operand alias write restarts calculation, including identical writes.
- The last written alias determines signedness; results become ready after eight cycles.
- Only QUOTIENT reads clear DIRTY; REMAINDER reads preserve it.
- Direct result writes cancel pending calculations and set READY and DIRTY.
- State survives Serial-to-Threaded promotion and successive worker invocations.

Direct `Sio` callers must advance time with `advance_divider_cycles` before
expecting a completed result. CPU execution advances the divider automatically.
See the [correction record](rp2040_sio_divider_fix_20261012.md) for the
6144 arithmetic comparisons and 104 pinned SDK 64-bit division calls.
The inspected earlier revisions `32d27ff`, `f810d059` and `121d649` remain affected.

## Migration from v0.1.0

The September baseline restoration removed later backend facilities, including
machine API, preview/audio analysis, heartbeat and SD/loader extensions. This
release contains that restored baseline plus the divider fix. It does not offer
all facilities present in the earlier tag or in historical accepted revisions.
The consumer retains tools and records for those exact pinned revisions.

The version is `v0.2.0` because the full change set includes incompatible scope
changes during the `0.y.z` technical-preview phase. It is not a compatible patch
upgrade. Report schema remains 8; the machine API schema-1 implementation is
**unavailable in this tagged backend**. Use the exact historical backend pin
when reproducing a target that requires an omitted facility.

No firmware target is promoted to this release merely by tagging it. The paired
consumer registry retains its historical `backend.accepted` SHAs. New application
BINs can use this corrected backend through direct `picocalc-run` execution;
formal registered regression continues to use the target's exact backend.

## Local checks

| Check | Result |
| --- | --- |
| Published local gate: board, harness and RP2040 tests | 1382 passed (including doc tests) |
| Threading/testing RP2040 tests | 1492 passed |
| Release runner build | passed |
| PicoCalc board/harness formatting | passed |
| PicoCalc board/harness Clippy, all targets, warnings denied | passed |
| Divider library/integration-test Clippy with threading/testing | passed |
| Combined idle-profiler/behavior-trace tests and library/harness Clippy | passed |
| Paired consumer portable verification / Python tool tests | 86 / 126 passed |

Logs and command manifests are attached to both GitHub Releases as
`picocalc-v0.2.0-validation.tar.gz`. The preparation gates ran from the divider
implementation commit with documentation/license changes in progress. No runtime
source was changed for release preparation. A clean final-commit build and source
identity smoke check are required before publication and recorded in that asset.

Test environment: Python 3.12.3, Rust/Cargo 1.97.1, x86_64 Linux.
Rust workspace minimum is 1.88. Normal Cargo tests use the committed SDK fixture
and require no external SDK. Rebuilding the fixture uses SDK 2.2.0 at
`a1438dff1d38bd9c65dbd693f0e5db4b9ae91779` and Arm GCC 13.2.1.

## Distribution and limits

The restored notices/license texts match the unchanged RP bootrom, OneROM,
SeaBIOS and epio/apio assets from `v0.1.0`. The new SDK fixture has its BSD-3-Clause
license and provenance in the fixture directory and root `NOTICE`.
Repository tags are independent of crate versions and BSP version. This action
publishes source releases, not packages to crates.io.

Serial remains the PicoCalc firmware reference. Threaded divider regressions do
not establish equivalence of all PicoCalc workloads. General cycle-timing and
peripheral limits remain in `tech_debt.md`. No hardware comparison, ClockCalc
rerun or new application/hardware acceptance is claimed by this release.

## Get the corrected source

```sh
git clone --branch v0.2.0 --recurse-submodules \
  https://github.com/FuyukiYoneyama/picoem-picocalc.git
cd picoem-picocalc
cargo build --locked --release -p picocalc-harness
```

Use `picocalc-run` as described in the paired
[consumer release record](https://github.com/FuyukiYoneyama/picocalc_emu/blob/v0.2.0/docs/RELEASE_V0.2.0.md).
A tag is immutable; existing `v0.1.0` tags are preserved.
