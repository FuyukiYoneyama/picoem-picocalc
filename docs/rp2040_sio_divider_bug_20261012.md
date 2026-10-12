# RP2040 SIO divider does not recalculate on dividend writes

Recorded and reviewed: 2026-10-12. **Status: fixed and regression-checked.**
The correction is identified by the Git commit containing the correction record.
See [the emulator correction and acceptance record](rp2040_sio_divider_fix_20261012.md).
The observations below describe the pre-fix revisions and retained history.

## Scope and impact

The pre-fix RP2040 SIO model starts division when a divisor register is written but
does not start it when a dividend register is written. A divisor-first
sequence therefore leaves quotient/remainder based on the preceding dividend.
Both unsigned and signed dividend aliases are affected.

Inspected revisions:

- Public backend: `32d27ff4179a993ae9fac6ff4a1d5e8999570fa9`.
- Backend used for ClockCalc observation:
  `f810d059958773d1b42a1c6d03cc15183cdc1a4f`.

Both revisions contain this omission in
[`Sio::divider_write` at the pre-fix revision](https://github.com/FuyukiYoneyama/picoem-picocalc/blob/121d649930d625924fa7608a3b72ed622098f522/crates/rp2040-emu/src/bus/sio.rs).
The follow-up review also checked public main `121d649930d625924fa7608a3b72ed622098f522`
and its separate [`WorkerBus::sio_write32` at that revision](https://github.com/FuyukiYoneyama/picoem-picocalc/blob/121d649930d625924fa7608a3b72ed622098f522/crates/rp2040-emu/src/threaded/bus.rs)
implementation. Both implementations were directly exercised as described
below. Their divider blocks are unchanged from `32d27ff` and `f810d059`;
other code in those revisions differs. No claim is made about all other
revisions or RP2350. The saved firmware observations used Serial execution;
the initial WorkerBus diagnostic did not start CPU worker threads. The later
correction record includes actual CPU/worker and SDK division execution.

SDK 64-bit division uses a divisor-first sequence to calculate a reciprocal.
An incorrect reciprocal can make its quotient-correction loop run for a very
long time. A consumer can appear to stop updating its display or handling keys
without reporting an emulator exception or unsupported MMIO.

## Expected register behavior and pre-fix implementation

Primary reference inspected locally: Raspberry Pi Pico SDK commit
`a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`,
[`hardware/regs/sio.h`](https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/sio.h).
The SIO register documentation specifies that writing either operand starts
a new calculation. Unsigned and signed dividend registers alias the same
operand and select the calculation type.

| SIO register | Offset from `0xd0000000` | Pre-fix `divider_write` behavior |
| --- | --- | --- |
| DIV_UDIVIDEND | `0x060` | Stores dividend, selects unsigned; no `compute_division` call |
| DIV_UDIVISOR | `0x064` | Stores divisor, selects unsigned, calls `compute_division` |
| DIV_SDIVIDEND | `0x068` | Stores dividend, selects signed; no `compute_division` call |
| DIV_SDIVISOR | `0x06c` | Stores divisor, selects signed, calls `compute_division` |

The SDK's
[`divider_hardware.S`](https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2_common/pico_divider/divider_hardware.S),
label `y64_x32`, writes the normalized divisor first, then writes
`0xffffffff` to DIV_UDIVIDEND before reading DIV_QUOTIENT. The second write
does not update the result in the pre-fix model.

This was a functional operand-trigger defect. The timing and DIRTY defects
were also corrected; their status is recorded in [tech_debt.md](../tech_debt.md)
and the correction record linked above.

## Minimal reproduction derived from the source

The following sequences were initially proposed without execution. The
follow-up direct register diagnostic now reproduces their stale results in
both implementations. These hardware-oriented sequences allow the documented
calculation time before reading; the diagnostic calls the pre-fix synchronous
model directly and does not test timing. Initial seeding avoids relying on
divider reset contents.

Unsigned:

```text
write32(0xd0000060, 0)       # seed dividend
write32(0xd0000064, 3)       # result now 0 / 3
write32(0xd0000060, 10)      # must start 10 / 3
wait for completion
read32(0xd0000070)          # expected quotient 3; pre-fix source leaves 0
read32(0xd0000074)          # expected remainder 1; pre-fix source leaves 0
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
time region had zero non-black pixels. The report records final PC
`0x10016e40`. The original investigation associated that address with an SDK
division correction loop, subject to the mapping evidence limit below.
No exception, emulator error or unsupported
MMIO was reported. LCD decoding recorded COLMOD `0x65`, SLPOUT once and
display_on=true; those fields do not establish successful physical display.

The UART trace recorded BMP280 ADC pressure/temperature `415148`/`519888`,
`t_fine=128422`, temperature `2508` (0.01 C units), then no subsequent sensor
completion. The temperature/pressure calibration matches the fixture.

A preceding 0.8.77 diagnostic report records final PC `0x10016cca`.
Its backend was dirty **only for a post-run register/stack dump**;
the divider implementation was unchanged. The original investigation
associated that PC with the same division routine and saved LR `0x1000cab9`
with the BMP280 pressure-compensation caller. Keep this run distinct from the
clean 0.8.78 evidence.

**Mapping evidence limit:** the retained reports/register dump establish the
recorded addresses, but the matching ELF, its SHA-256, map/symbol table and
address-mapping/disassembly output were not preserved in the evidence bundle.
The function associations above are recorded results of the original
investigation and cannot be independently checked from the saved bundle.
They were not re-established by the follow-up register diagnostic. If a
matching ELF is recovered, retain its hash and mapping command/output; a new
firmware build's addresses cannot substitute for the original mapping.

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

## Original proposed checks (historical)

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

The original evidence/documentation commit followed cancellation of firmware
emulation. The follow-up review below executed only register diagnostics;
no emulator correction was applied at that stage. The subsequent correction
and acceptance results are recorded separately in the link at the top.

## Follow-up verification: pre-fix direct register observations

[Program, lockfile, raw output and provenance](evidence/sio-divider-review-20261012/README.md)
are retained for main `121d649`. An external Cargo project in `/tmp` depended
on the unmodified `rp2040-emu` library with `threading` enabled. It called
Serial `Sio` and Threaded `WorkerBus` directly on x86_64 Linux, with no CPU
instructions, scheduler execution, firmware image or physical hardware.

Each implementation received 17 checks. Serial had 6 matching observations
and 11 mismatches; WorkerBus had 7 matching observations and 10 mismatches.
These counts are diagnostic cases, not distinct bugs or a target pass rate.
The program exits 0 after recording observations; `matches_spec=false` in
the output explicitly records a specification mismatch.

| Case | Expected quotient / remainder | Observed in both implementations |
| --- | --- | --- |
| Unsigned, dividend then divisor: 10 / 3 | 3 / 1 | 3 / 1 |
| Unsigned, divisor then dividend: 10 / 3, seeded with 0 / 3 | 3 / 1 | 0 / 0 |
| Signed, divisor then dividend: -10 / 3, seeded with 0 / 3 | -3 / -1 | 0 / 0 |
| Change dividend from 10 to 20 with divisor 3 | 6 / 2 | 3 / 1 |
| SDK reciprocal operand pattern: 0xffffffff / 65537 | 65535 / 0 | 0 / 0 |
| Unsigned zero divisor written before dividend 42 | 0xffffffff / 42 | 0xffffffff / 0 |
| Signed zero divisor written before dividend -42 | 1 / -42 | -1 / 0 |

Mixed signed/unsigned dividend aliases also leave the old results. Control
checks for dividend-first signed division, divisor updates, signed overflow
and the selected independent core-0/core-1 calculations match expectations.
The reciprocal check exercises register ordering only; it does not execute
SDK assembly or the entire 64-bit division routine.

The [replay instructions](evidence/sio-divider-review-20261012/README.md)
select the backend checkout and diagnostic bundle independently: `121d649`
contains the library source but predates the retained diagnostic files.
The program targets the synchronous model. After introducing calculation
latency, use a separate diagnostic that advances emulated cycles and waits
for READY before result reads; the unchanged program's mismatch counts
cannot assess that model's conformance.

The consumer's existing 32 artifact hashes and I2C fixture hash were verified,
and the four reports' identities/verdicts agree with the preserved manifest.
Visual inspection confirms that baseline-v073's supposed clock screen retains
calculator keys, while framing-v078 has no time glyphs. Those checks strengthen
the evidence accounting, but add no corrected-firmware result.

## Additional verified pre-fix defects in divider status handling

The pinned SDK's `SIO_DIV_QUOTIENT` and `SIO_DIV_CSR_DIRTY` descriptions specify
that reading **QUOTIENT** clears DIRTY. Reading REMAINDER leaves DIRTY set.
An operand write sets DIRTY and starts calculation; direct result writes set
READY/DIRTY and terminate any calculation in progress.

| Operation | Specification | Serial diagnostic | WorkerBus diagnostic |
| --- | --- | --- | --- |
| Read QUOTIENT once after 100 / 7 | DIRTY=0 | DIRTY=1 | DIRTY=1 |
| Read REMAINDER twice after 100 / 7 | DIRTY=1 | DIRTY=0 | DIRTY=1 |
| Consume REMAINDER then QUOTIENT, then write dividend | DIRTY 0 then 1 | 0 then 0 | 1 then 1 |

Pre-fix Serial `divider_result_read` counted two reads of either result rather
than clearing on QUOTIENT. WorkerBus result reads never cleared DIRTY. In the last
WorkerBus check, DIRTY was already incorrectly set before the dividend write;
its final value cannot establish whether that write itself sets DIRTY.
These status defects can affect SDK interrupt context save/restore decisions.
They are not independently established causes of the saved ClockCalc stall.

The old debt description saying that *any* result read clears DIRTY was wrong
and has been corrected. The pre-fix Serial test
`divider_dirty_flag_clears_after_two_reads` encoded incorrect behavior; the
correction replaces it with QUOTIENT-specific clearing checks.

Both pre-fix models report READY=1 immediately and calculate synchronously on
divisor writes. The pinned SDK `hardware/divider.h` documents an 8-cycle
calculation and readable results on the ninth cycle. This timing gap was
confirmed by source inspection, not a cycle-boundary experiment. The stale
operand result reproduced above is caused by the missing trigger; immediate
READY alone does not imply a stale result after a correctly triggered write.

## Original remediation plan and comparison requirements (historical)

1. **Correct operand triggers in both runtime paths.** In Serial
   `divider_write` and Threaded `sio_write32`, each of the four operand aliases
   must store the value, select the operation's signedness, and start a new
   calculation for the accessing core. Adding calls only in Serial leaves
   WorkerBus defective. Preserve wrapping arithmetic, divide-by-zero behavior
   and direct result-register writes. A local, minimal correction is sufficient;
   sharing the arithmetic/trigger helper can be considered to prevent drift.
2. **Correct status behavior with explicit checks.** Clear DIRTY only on
   QUOTIENT reads; repeated REMAINDER reads must preserve it. Every operand
   write must set it. Replace the two-read test, and add interrupt
   save/restore and direct result-write checks. Do not accept Serial/Threaded
   agreement alone: both currently share errors, so use specification-derived
   expected values.
3. **Track timing as a separate requirement.** For cycle fidelity, model an
   8-cycle completion deadline per core, READY transitions, restart after a
   new operand write, and cancellation by result-register writes. Wire this
   to the correct clock in both execution models. Functional trigger checks
   can establish that correction before timing fidelity is completed, with
   the remaining timing limitation stated explicitly.
   Adapt the diagnostic to advance emulated cycles and require READY=1 before
   result assertions; the retained direct-call program reads immediately.
   Polling READY alone does not advance this program's emulated clock.
4. **Validate SDK callers.** Run actual SDK signed/unsigned 64-bit division
   across the `y64_x32`, `y64_x48` and `y64_x64` paths. Include positive/negative
   inputs, large operands, zero-divisor policy, overflow, repeated operands
   and interrupt context restore. Set a completion budget to detect correction
   loops that fail to make progress. Compare quotient/remainder to an
   independent exact integer reference, then run meaningful existing RP2040
   regressions under both supported execution models.
5. **Confirm the ClockCalc inference separately.** Use the identical BIN,
   fixture, runner settings and scenario for before/after runs. Preserve BIN
   and full dirty firmware source or a reproducible clean build plus toolchain,
   so new evidence is independently reproducible. If the old BIN cannot be
   recovered, label a new firmware baseline as a different comparison.
   Preserve the matching ELF's SHA-256 and the commands/output mapping PC/LR
   addresses to symbols or disassembly, alongside the BIN identity.
   Require pressure-compensation completion and its fixture-derived value,
   expected clock glyphs at fixed RTC times, subsequent time changes, and
   Escape returning to a usable calculator. Improve the scenario before
   acceptance: assert specific glyph regions/patterns and observable input
   effects, not just a minimum non-black count. A delivered key event alone
   does not prove that the application handled it.

A workaround that writes dividend before divisor in a custom caller can
avoid this trigger defect locally. Editing ClockCalc application expressions,
disabling the pressure sensor or adding delays cannot establish corrected
SDK division behavior. Correcting the backend is the proposed permanent
action. The [correction record](rp2040_sio_divider_fix_20261012.md) defines
emulator acceptance using register/CPU/SDK regressions independently of any
application observations. The firmware diagnosis above remains historical.
