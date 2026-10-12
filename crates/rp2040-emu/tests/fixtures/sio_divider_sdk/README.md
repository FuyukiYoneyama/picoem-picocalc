# Pico SDK hardware divider regression fixture

`fixture.bin` is linked at SRAM `0x20000000`. The Rust regression tests execute
the actual Cortex-M0+ instructions from the unmodified Pico SDK
`divider_hardware.S` at commit `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`.
SDK source and helpers are BSD-3-Clause, copyright Raspberry Pi; their source
identities are recorded in `provenance.json`.

`harness.S` loads two 64-bit inputs and a signedness selector from `0x20030000`,
calls `__aeabi_uldivmod` or `__aeabi_ldivmod`, and writes quotient, remainder
and a completion marker at `0x20030020`. The fixture has no app, board, display
or bootrom dependency. Both cores and Serial/Threaded execution are tested
with quanta 1 and 128, exact host integer references and finite budgets.
The tests seed a dirty pending divider before the call and check its restored
operands/results. Threaded tests split execution across worker invocations.

`pico.h` supplies only assembly configuration: ordinary symbol names, one
text section, SDK state save/restore enabled, and zero-divisor callback hooks
disabled. No SDK arithmetic instructions are replaced. Zero-divisor register
behavior is checked separately in `tests/sio_divider.rs`; these SDK tests use
nonzero divisors and include signed overflow.

Rebuild with an existing checkout of the pinned SDK and the Arm GNU toolchain:

```sh
python3 crates/rp2040-emu/tests/fixtures/sio_divider_sdk/build.py --sdk /path/to/pico-sdk
```

The builder checks the SDK revision and the six included source/header files
against that revision, builds in a temporary directory, and updates the BIN,
symbol list and provenance. `provenance.json` records input/BIN/ELF hashes,
compiler version and commands. The temporary ELF is not retained; its recorded
hash identifies the linker output, and the source/commands reproduce it using
the recorded toolchain. Normal Rust tests use the checked-in BIN and do not
require the SDK checkout or Arm compiler.
