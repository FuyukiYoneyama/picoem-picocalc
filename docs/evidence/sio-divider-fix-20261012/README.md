# RP2040 SIO divider correction regression evidence — 2026-10-12

See [the correction record](../../rp2040_sio_divider_fix_20261012.md).
This validates the emulator's register/CPU/SDK behavior independently of apps.
It contains no ClockCalc run or hardware comparison.

## Identity and replay

The base backend revision is `121d649930d625924fa7608a3b72ed622098f522`.
The correction was tested in a modified working tree; this bundle does not
claim a clean published backend identity. `metadata.json` records commands,
results and source hashes. `source.patch` includes all tracked and new
changes under `crates/rp2040-emu`, including the regression code and SDK
fixture. Documentation changes elsewhere are not part of that source patch.

The metadata's `committed=false` and `pushed=false` describe the state when
this evidence was recorded, before publication; they are preserved historical
fields. The correction record's containing Git commit identifies the published
source. Its crate files can be compared with the recorded source hashes.

To replay, use a separate checkout at the base revision, apply `source.patch`,
and execute the commands in `metadata.json`. Offline Cargo execution requires
the locked dependencies to be cached. Threaded tests require x86_64 Linux or
Windows with at least three available host cores. Rust 1.97.1 was used here.
Verify this bundle with `sha256sum -c SHA256SUMS` from this directory.

The fixture builder's source and bytecode provenance are under
`crates/rp2040-emu/tests/fixtures/sio_divider_sdk`. Tests consume its tracked
BIN; rebuilding the fixture requires the pinned SDK and Arm GNU toolchain.
The final fixture generator update normalizes the BIN file mode. Its bytes
were verified unchanged from the BIN executed by the tests.

## Logs and limitations

- `rp2040-default.log`: default configuration, 1269 passed, zero failures.
- `rp2040-all.log`: `threading,testing`, 1492 passed, zero failures.
- `divider-regression.log`: eight specification-based register/CPU tests.
- `sdk-regression.log`: two SDK regression tests, covering 104 actual
  signed/unsigned 64-bit calls across both cores, two quanta and both models.
- `clippy-focused.log`: strict production library and new integration test.
- `clippy-all.log`: all targets, allowing only the existing `identity_op` lint.
- `clippy-initial.log`: initial failure before moving the new test module;
  it also records the six existing `BASE + 0x000` identity warnings. This is
  retained as a failure record, not reported as a passing strict all-target run.
- `rustfmt.log`: empty on success for the changed Rust files.
- `sdk-build.log`: fixture build output, empty on success.
- `SHA256SUMS`: hashes of every bundle file except itself.

Results read while READY=0 are undefined by the hardware specification and
are not used as acceptance values. Timing checks use the emulator's existing
instruction scheduling model; no physical device or consumer firmware result
is established by this bundle. The earlier diagnostic bundle is preserved
separately and still records the pre-fix mismatches.
