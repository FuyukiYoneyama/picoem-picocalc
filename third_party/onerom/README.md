# OneROM firmware fixtures

The harness redistributes six locally built OneROM RP2350 firmware images:

- `crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin`
- `crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-1541.bin`
- `crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-seabios-cpu.bin`
- `crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0-cpu.bin`
- `crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0.bin`
- `crates/picoem-harness/fixtures/onerom-fire-32-a-rp2350-seabios.bin`

The firmware source is the upstream [OneROM](https://github.com/piersfinlayson/one-rom)
software/firmware project. Its software and firmware are licensed under the MIT
license; the complete upstream text is preserved in [`LICENSE-MIT`](LICENSE-MIT).
Copyright belongs to Piers Finlayson and contributors. OneROM hardware files
are not redistributed here.

The fixture binaries are fixed test inputs, not a vendored OneROM source tree.
Their SHA-256 values and historical build recipes are recorded in
[`crates/picoem-harness/fixtures/README.md`](../../crates/picoem-harness/fixtures/README.md)
and the corresponding `wrk_journals/` entries. The historical build journals
clone the upstream `main` branch but do not preserve a OneROM source commit;
therefore these binaries are byte-pinned fixtures and this repository does not
claim a clean source-to-byte rebuild for the historical images.
