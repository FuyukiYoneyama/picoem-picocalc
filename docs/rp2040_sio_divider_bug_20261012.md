# RP2040 SIO divider does not recalculate on dividend writes

Recorded: 2026-10-12. **Status: open; source defect identified, fix not applied.**

## Scope and impact

The RP2040 SIO model starts division when a divisor register is written but
does not start it when a dividend register is written. A divisor-first
sequence therefore leaves quotient/remainder based on the preceding dividend.
Both unsigned and signed dividend aliases are affected.

Inspected revisions:

- Public backend: `32d27ff4179a993ae9fac6ff4a1d5e8999570fa9`.
- Backend used for ClockCalc observation:
  `f810d059958773d1b42a1c6d03cc15183cdc1a4f`.

Both revisions contain this omission in
[`Sio::divider_write`](../crates/rp2040-emu/src/bus/sio.rs). No claim is made
about all other revisions, RP2350, or threaded-mode behavior; the firmware
observations below used Serial execution.

SDK 64-bit division uses a divisor-first sequence to calculate a reciprocal.
An incorrect reciprocal can make its quotient-correction loop run for a very
long time. A consumer can appear to stop updating its display or handling keys
without reporting an emulator exception or unsupported MMIO.

## Expected register behavior and actual implementation

Primary reference inspected locally: Raspberry Pi Pico SDK commit
`a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`,
[`hardware/regs/sio.h`](https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/sio.h).
The SIO register documentation specifies that writing either operand starts
a new calculation. Unsigned and signed dividend registers alias the same
operand and select the calculation type.

| SIO register | Offset from `0xd0000000` | Current `divider_write` behavior |
| --- | --- | --- |
| DIV_UDIVIDEND | `0x060` | Stores dividend, selects unsigned; no `compute_division` call |
| DIV_UDIVISOR | `0x064` | Stores divisor, selects unsigned, calls `compute_division` |
| DIV_SDIVIDEND | `0x068` | Stores dividend, selects signed; no `compute_division` call |
| DIV_SDIVISOR | `0x06c` | Stores divisor, selects signed, calls `compute_division` |

The SDK's
[`divider_hardware.S`](https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2_common/pico_divider/divider_hardware.S),
label `y64_x32`, writes the normalized divisor first, then writes
`0xffffffff` to DIV_UDIVIDEND before reading DIV_QUOTIENT. The second write
does not update the result in this model.

This is a functional operand-trigger defect. Existing divider timing and
dirty-flag limitations in [tech_debt.md](../tech_debt.md) remain separate items.

## Minimal reproduction derived from the source

The following sequences are proposed regression cases, **not newly executed
tests**. Each uses the same core for all accesses and allows the hardware's
documented calculation time before reading results. Initial seeding avoids
relying on divider reset contents.

Unsigned:

```text
write32(0xd0000060, 0)       # seed dividend
write32(0xd0000064, 3)       # result now 0 / 3
write32(0xd0000060, 10)      # must start 10 / 3
wait for completion
read32(0xd0000070)          # expected quotient 3; current source leaves 0
read32(0xd0000074)          # expected remainder 1; current source leaves 0
```

Signed:

```text
write32(0xd0000068, 0)
write32(0xd000006c, 3)
write32(0xd0000068, 0xfffffff6)  # -10
wait for completion
read32(0xd0000070)              # expected -3 (0xfffffffd); source leaves 0
read32(0xd0000074)              # expected -1 (0xffffffff); source leaves 0
```

## Firmware observation and evidence limits

During ClockCalc Calculator-to-Clock Esc diagnosis, the clean f810d059 runner
executed the actual 0.8.78 BIN with `--board picocalc --lcd-variant pio-rgb565
--keyboard --i2c-profile picocalc-rtc-env-v1`, quantum 1 and the existing
`i2c-ext-e5-env-v1.json` fixture. At 2,680 ms the scenario failed: the clock
time region had zero non-black pixels. The final PC was `0x10016e40`, in an
SDK division correction loop. No exception, emulator error or unsupported
MMIO was reported. LCD decoding recorded COLMOD `0x65`, SLPOUT once and
display_on=true; those fields do not establish successful physical display.

The UART trace recorded BMP280 ADC pressure/temperature `415148`/`519888`,
`t_fine=128422`, temperature `2508` (0.01 C units), then no subsequent sensor
completion. The temperature/pressure calibration matches the fixture.

A preceding 0.8.77 diagnostic run stopped in the same division routine at
`0x10016cca`. Its backend was dirty **only for a post-run register/stack dump**;
the divider implementation was unchanged. Saved LR `0x1000cab9` was mapped
in that ELF to the BMP280 pressure-compensation caller. Keep this run distinct
from the clean 0.8.78 evidence.

**Inference:** the identified operand-trigger defect explains the observed
pressure-division stall. There has been no run with a corrected divider, so
causal confirmation and recovery remain unverified. This does not establish
the cause of the user's physical LCD failure.

Raw reports, UART, scenarios, screenshots, the diagnostic patch and a hash
manifest are retained by the consumer repository in
[ClockCalc divider evidence](https://github.com/FuyukiYoneyama/picocalc_emu/tree/main/firmware-validation/evidence/clockcalc-sio-divider-20261012-01).
The 0.8.73 baseline's broad non-black-pixel assertion passed despite retained
calculator graphics in the supposed clock image. That pass is insufficient
as a visual acceptance baseline.

## Proposed correction and acceptance before closing

1. Start calculation for both dividend aliases as well as both divisor aliases,
   preserving signed/unsigned selection and per-core divider state.
2. Cover dividend-first and divisor-first writes, repeated dividend updates,
   unsigned/signed alias selection, zero divisors and per-core isolation.
3. Exercise the SDK `y64_x32` sequence and a 64-bit division workload.
4. With a clean corrected backend, replay the saved ClockCalc conditions;
   require sensor completion, the expected clock glyphs and later input
   processing. A region merely containing non-black pixels is insufficient.
5. Record the correction commit and raw before/after evidence. Update the
   appropriate accepted backend pin only after its acceptance requirements
   are satisfied.

The user canceled emulator execution during diagnosis and requested this
documentation commit. No correction, regression run, target-pin change or
hardware operation is part of this documentation task.
