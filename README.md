# picoem-picocalc

An independent PicoCalc-oriented derivative of [`0x4D44/picoem`](https://github.com/0x4D44/picoem), retaining the upstream Git history and dual MIT OR Apache-2.0 licensing.

The maintained public distribution is
[`FuyukiYoneyama/picoem-picocalc`](https://github.com/FuyukiYoneyama/picoem-picocalc).
Links to `0x4D44/picoem` identify the upstream origin, not this derivative's
distribution location.

Public-project guidance is in [`CONTRIBUTING.md`](CONTRIBUTING.md),
[`SECURITY.md`](SECURITY.md), and [`NOTICE`](NOTICE). The repository does not
promise a response SLA, and development-probe identifiers remain per-machine
operator configuration rather than checked-in project data.
Users should select a GitHub Release and immutable SemVer tag rather than the
moving `main` branch. The backend/emulator pairing and exact commit policy is
documented in [`docs/VERSIONING.md`](docs/VERSIONING.md) and the canonical
[`picocalc_emu` versioning policy](https://github.com/FuyukiYoneyama/picocalc_emu/blob/main/docs/VERSIONING.md).

The upstream Git history is intentionally retained. The current tree removes
operator-specific probe identifiers and paths from public instructions, but
historical commits are not rewritten by this release-preparation work.

## Derivative origin and direction

- **Upstream:** `0x4D44/picoem`
- **Initial upstream base:** `8e20a82e64ef0a416e21af20edd964138ecd41be`
- **Repository model:** independent derivative rather than a GitHub fork-network repository
- **Primary consumer:** [`FuyukiYoneyama/picocalc_emu`](https://github.com/FuyukiYoneyama/picocalc_emu)

This derivative is intended to provide the RP2040 firmware-execution backend for `picocalc_emu`. The current `picocalc-run` interface direct-boots the raw Pico SDK BIN generated alongside the ELF and the UF2 deployed to hardware, while connecting the emulated RP2040 to deterministic PicoCalc board-device models. ELF input must first be converted with `objcopy`; direct UF2 loading is not implemented.

Development priorities are:

1. Keep the existing RP2040 CPU, dual-core, SIO, PIO, DMA, peripheral, and differential-test assets intact.
2. Treat the single-host-thread `ExecutionModel::Serial` path as the correctness reference. Threaded execution remains optional until it is demonstrably equivalent for PicoCalc workloads.
3. Add reusable external-device interfaces needed by PicoCalc, including LCD, keyboard, SD card, PSRAM, audio, GPIO stimulus, UART capture, SPI devices, and I2C devices.
4. Support deterministic scenario execution and artifacts such as framebuffer images, UART logs, traces, filesystem results, and test reports.
5. Preserve upstream attribution, licenses, and history. General fixes suitable for upstream may be prepared separately for contribution to `0x4D44/picoem`.

This firmware backend complements rather than replaces the faster host-device-model backend implemented in `picocalc_emu`. PicoCalc-specific behavior should remain outside the generic RP2040 core wherever a board-level adapter or external-device model is sufficient.

Upstream changes are fetched through the local `upstream` remote and incorporated selectively after review and regression testing. They are not merged automatically.

## Original project scope

Cycle-accurate emulators for the Raspberry Pi **RP2354 / RP2350** (dual Arm Cortex-M33 @ 150 MHz, 520 KB SRAM, FPU, coprocessors, PIO) and **RP2040** (dual Arm Cortex-M0+ @ 133 MHz, 264 KB SRAM, PIO), written in Rust.

The original project goal is a small, clean, verifiable emulator core that can boot the real Pi bootroms, run ARMv8-M (M33) or ARMv6-M (M0+) firmware with accurate cycle timing, and serve as a reusable library crate for downstream projects.

```
picoem-picocalc (this repo)      — PicoCalc-oriented derivative and test workspace
  ├─► rp2350-emu                 — RP2350 / RP2354 emulator library (Cortex-M33)
  ├─► rp2040-emu                 — RP2040 emulator library (Cortex-M0+)
  └─► onerom-emu                 — OneROM firmware running on rp2350-emu
        └─► mddosem              — DOS emulator, uses OneROM as BIOS
```

## Status

The upstream repository is a personal research project and does not promise issue-triage, pull-request review, or ongoing feature-development service levels. This derivative is maintained independently for PicoCalc integration; users should not expect the upstream author to support derivative-specific changes.

Feature coverage as of the published versions — Arm-mode only; Hazard3 RISC-V cores on RP2350 are out of scope:

| Subsystem | RP2350 / RP2354 | RP2040 |
|---|---|---|
| CPU core ISA | Cortex-M33 (ARMv8-M Mainline); differential-tested vs QEMU | Cortex-M0+ (ARMv6-M); differential-tested vs QEMU `cortex-m0` |
| FPU (VFPv5 single-precision) | Working; lazy context save | N/A |
| Coprocessors (GPIO/CP0, DCP, RCP) | Working | N/A |
| Dual-core + SIO (spinlocks, FIFOs, interpolators) | Working | Working (+ hardware divider) |
| Bus fabric | Working; some timing edge cases open | Working; simplifications open (see `tech_debt.md`) |
| Clock tree (ROSC / XOSC / PLL / dividers) | Working | Working |
| Exceptions / NVIC / fault delivery | Working | Working |
| Memory | 32 KB ROM, 520 KB SRAM, XIP flash | 16 KB ROM, 264 KB SRAM (no onboard flash) |
| PIO blocks | Working | Working |
| Pacer (wall-clock real-time pacing) | Working | Working |
| UART / SPI / I2C / DMA / timers | Mixed implementation; see `tech_debt.md` | Implemented for the exercised PicoCalc paths; residual timing gaps remain |
| GDB RSP debug server | Stub | Stub |
| TrustZone (SAU / ACCESSCTRL) | Design seams only — v1 treats everything as Secure | N/A |

Open cycle-timing gaps and post-Phase-7 residuals are tracked in `tech_debt.md`.

### PicoCalc integration status

The RP2040 PicoCalc path can direct-boot a Pico SDK BIN and attach the board display,
8 MiB PSRAM, I2C keyboard controller, and a pre-formatted SPI0 SD card. The in-memory test card is
64 MiB; its filesystem defaults to FAT32 to match the format expected for PicoCalc's bundled 32 GB
card, while `--sd-format fat16` selects the compatibility profile. Both formats pass the same BSP
filesystem smoke, while the SPI block model itself remains filesystem-independent. Both LCD transports
used by the canonical BSP are modelled: SPI1/RGB666 for compatibility and PIO0/RGB565 for
the recommended configuration. The scenario runner can inject timed keys and assert UART,
pixel, and framebuffer-region conditions while firmware is executing.

The primary reference for keyboard-controller behavior is ClockworkPi's official
[`PicoCalc/Code/picocalc_keyboard`](https://github.com/clockworkpi/PicoCalc/tree/master/Code/picocalc_keyboard)
STM32F103R8T6 firmware. In this workspace it is checked out at
an arbitrary local checkout of that repository. R1 conformance is complete: the Rust
device pins the consumer-visible `0x01..0x0e` replies, 31-event FIFO, official matrix/button map,
state and modifier transformations, strict hold/repeat thresholds, both overflow policies,
backlights, battery, reset, C64, and power-off behavior to that source. It deliberately does not
simulate STM32 GPIO electrical scanning, debounce and 16 ms polling, or the PMU battery/power-key
lifecycle. CFG/INT/DEB/FRQ remain internal because the official `receiveEvent()` switch does not
expose them over I2C.

```bash
cargo run --release -p picocalc-harness --bin picocalc-run -- \
  --bin /absolute/path/to/picocalc_app.bin \
  --bootrom "$PWD/roms/rp2040/bootrom-rp2040-b2.bin" \
  --board picocalc --lcd-variant pio-rgb565 \
  --psram --sd --keyboard --scenario /absolute/path/to/scenario.json \
  --snapshot-dir /tmp/picocalc-snapshots --json /tmp/picocalc-report.json
```

For long-running firmware sessions, the runner can emit best-effort progress lines to stderr without
changing the deterministic report or verdict inputs. The two options are deliberately an explicit pair:

```bash
cargo run --release -p picocalc-harness --bin picocalc-run -- \
  --run-id run-a --progress-interval 10 \
  --bin /absolute/path/to/picocalc_app.bin \
  --bootrom "$PWD/roms/rp2040/bootrom-rp2040-b2.bin" \
  --board picocalc --lcd-variant pio-rgb565 --psram --sd --keyboard \
  --scenario /absolute/path/to/scenario.json \
  --snapshot-dir /tmp/picocalc-runs/run-a/snapshots \
  --json /tmp/picocalc-runs/run-a/report.json
```

The normal `picocalc_emu/tools/picocalc.py test --mode firmware` entry point enables a ten-second
heartbeat by default and supplies `<target>-<wrapper-pid>` when `--run-id` is omitted. Use
`--no-progress` to disable it. Allocate a distinct output directory for every concurrent run; the
runner does not create a central run registry or protect shared artifact paths. Heartbeat lines are
diagnostic stderr only, and `event=finish` carries the runner's authoritative `exit` value (`0=pass`,
`1=fail`, `2=cannot_judge`). WSL supervisors should check `finish` and the report before trusting an
outer shell exit code. See the canonical [concurrent-run guidance](https://github.com/FuyukiYoneyama/picocalc_emu/blob/main/docs/CONCURRENT_RUNS.md).

For the PicoCalc PWM5_CC path, the runner can also write a separate deterministic level-analysis
artifact and an optional unnormalised listening WAV. The frozen NEXT-2 contracts remain the
48 kHz stereo path; streams with another observed timer rate are emitted as
the additive audio-analysis schema 2 instead of being forced into that historical contract:

```bash
cargo run --release -p picocalc-harness --bin picocalc-run -- \
  <the normal PicoCalc firmware arguments> \
  --audio-analysis /tmp/picocalc-audio-analysis.json \
  --audio-wav /tmp/picocalc-audio-raw.wav
```

The analysis reconstructs signed 16-bit samples from the post-quantizer 8-bit duty stream and
reports peak, whole-stream and maximum complete 1024-frame-block RMS, active-frame ratio, DC offset, PWM
rail ratio, and the longest consecutive rail run. A rail sample is not automatically a firmware
clip and is not an automatic failure. Project policy belongs in `picocalc_emu`'s schema-2 quality
contract: it rejects needlessly low level while allowing bounded saturation and only rejecting
extreme rail occupancy. The WAV preserves the observed digital level; it is never normalised and
does not model the amplifier, physical volume control, speaker, enclosure, or room.

The DMA arbitration rules, timer-miss field meanings, board-less audio capture,
and the limits of this digital observation are documented in
[`docs/DMA_AUDIO_OBSERVABILITY.md`](docs/DMA_AUDIO_OBSERVABILITY.md). That
document is part of the public emulator contract; it also records the
quantum-invariance test and the diagnostic-only UART-marker profile option.

NEXT-4 also exposes the same persistent machine session as a deterministic JSON Lines API.
Artifact and device options are fixed at process startup; one stdin line produces one stdout line,
while UART and diagnostics never contaminate stdout. `run` is always bounded and subscriptions are
pulled at command boundaries rather than delivered by a wall-clock thread:

```bash
cargo run --release -p picocalc-harness --bin picocalc-run -- \
  --machine-api --bin /absolute/path/to/picocalc_app.bin \
  --bootrom "$PWD/roms/rp2040/bootrom-rp2040-b2.bin" \
  --board picocalc --lcd-variant pio-rgb565 --psram --sd --keyboard \
  --snapshot-dir /tmp/picocalc-machine-snapshots
```

The schema-1 operations are `run`, `step`, `run_until`, `input`, `observe`, `subscribe`, and
`snapshot`. `step` means one scheduler dispatch, not one CPU instruction or one master cycle, and
returns the actual cycle count. `snapshot` means a framebuffer snapshot, not a restorable emulator
checkpoint. The normative wire schema and fail-closed rules are documented in
[`picocalc_emu/docs/HEADLESS_MACHINE_API.md`](https://github.com/FuyukiYoneyama/picocalc_emu/blob/main/docs/HEADLESS_MACHINE_API.md).

The command above is authoritative only when its structured report is checked against the
target's declared expectations. The cross-repository acceptance order, source/toolchain pins,
and current hardening work are owned by
[`picocalc_emu/docs/MILESTONES.md`](https://github.com/FuyukiYoneyama/picocalc_emu/blob/main/docs/MILESTONES.md),
not duplicated here.

Report schema 8 makes that judgement normative. Schema 7 introduced the fail-closed verdict;
schema 8 additionally binds each report to the commit and dirty state compiled into the runner via
`backend_build`. A conformance invocation supplies an accepted stop with `--expect-stop` and may
repeat `--expect-uart` for required firmware markers. A raw cycle-limit run without any acceptance
criterion is `cannot_judge`, not pass. Exception, emulator error, unsupported or truncated MMIO,
keyboard event loss, scenario failure, stop mismatch, and missing UART markers cannot silently
pass. The report's `verdict.status` and process status use the same mapping: 0=`pass`, 1=`fail`,
2=`cannot_judge`. A scenario infrastructure fault also exits 2; an assertion, timeout, or
incomplete scenario exits 1. Registered cross-repository runs should normally use
`picocalc_emu/tools/picocalc.py test --mode firmware --target ...`, which verifies schema 8,
backend/source identity, artifact hashes, device arguments, and report expectations together.

### OPT0-A idle profiling

The Serial runner has an opt-in, diagnostic-only idle profiler for the emulator optimization
program. It is excluded from default builds so that the reference and wall-time measurement path
does not acquire a disabled profiler branch. Build it explicitly and add an output path:

```bash
cargo run --release -p picocalc-harness --features idle-profiler -- \
  <the normal picocalc-run arguments> \
  --idle-profile /tmp/picocalc-idle-profile.json
```

The separate schema-1 profile records total and per-core cycles, the both-core-blocked upper bound,
a conservative proven-safe lower bound, power-of-two cumulative `S(K)` distributions, and
overlapping blocker cycles and episode counts for PIO, DMA, PWM, SysTick, UART, SPI, I2C, ADC,
timer, and pending IRQ state. `instrumented=true` and `valid_for_wall_time=false` are normative:
the JSON is for prioritizing optimization work and must not be used as a performance result. The
ordinary schema-8 report and its acceptance rules are unchanged.

The companion cost microbenchmark uses the same diagnostic feature:

```bash
cargo build --locked --release -p picocalc-harness \
  --features idle-profiler --bin opt0-idle-cost
taskset -c 0 target/release/opt0-idle-cost \
  --iterations 1000000 --samples 10 --json /tmp/opt0-idle-cost.json
```

It compares loop overhead, a current conservative idle probe, the existing quantum-1 blocked step,
and quiescent `tick_peripherals(L)` for several values of `L`. Its schema explicitly reports that
active-source next-event calculation, boundary event firing, clock update, IRQ routing, and wake
checks are not yet fully costed. Consequently its break-even value is an optimistic lower bound and
is not sufficient by itself to choose an optimization.

## Quick Start

Clone with submodules — the workspace member `epio-sys` references vendored upstream sources via git submodules. A normal clone works for everything else, but `cargo build -p epio-sys` requires submodules to be initialised:

```bash
git clone --recursive https://github.com/FuyukiYoneyama/picoem-picocalc.git
# or, after a non-recursive clone:
git submodule update --init
```

`epio-sys` is excluded from the workspace's `default-members` and additionally requires `clang` to be on `PATH`, so a plain `cargo build --release` at the workspace root works on hosts without `clang` or initialised submodules — opt in explicitly with `cargo build -p epio-sys` once those prerequisites are in place.

```bash
# Build everything (release profile is strongly recommended — debug is slow)
cargo build --release

# RP2350 / RP2354 interactive TUI (dual Cortex-M33)
cargo run -p rp2350-emu-tui --release              # blinky (default)
cargo run -p rp2350-emu-tui --release -- lcd       # LCD demo
cargo run -p rp2350-emu-tui --release -- benchmark # throughput benchmark
cargo run -p rp2350-emu-tui --release -- blinky    # (explicit)

# RP2040 interactive TUI (dual Cortex-M0+)
cargo run -p rp2040-emu-tui --release              # blinky (default)

# Load your own firmware
cargo run -p rp2350-emu-tui --release -- path/to/firmware.bin
cargo run -p rp2040-emu-tui --release -- path/to/firmware.bin
```

The RP2350 TUI has panels for CPU status, GPIO state, an LCD device emulator, an ISA trace view, and a live benchmark panel. The RP2040 TUI has the same shape minus the FPU / DCP / RCP / NS panels, and its ISA panel carries M0+-specific cycle numbers.

Bundled ROMs under `roms/rp2350/` (`blinky.bin`, `benchmark.bin`, `lcd_demo.bin`, `dualcore.bin`) and `roms/rp2040/` (`blinky.bin`) are generated from the `gen_*.py` scripts in the same directories. The real Pi bootroms are checked in as `roms/rp2350/bootrom-combined.bin` and `roms/rp2040/bootrom.bin`.

## Workspace Layout

Eleven crates under `crates/`:

- **`picoem-common`** — shared primitives: `Memory`, `ClockTree`, `Pacer`, PIO primitive types (`PioBlock` / `StateMachine`), divider/FIFO, and portable threaded primitives. Both chip crates depend on this.
- **`picoem-devices`** — reusable off-chip device models shared by emulators and harnesses.
- **`rp2350-emu`** — the RP2350 / RP2354 emulator core library (CPUs, bus, memory, clocks, SIO, PIO, FPU, coprocessors, pacer).
- **`rp2350-emu-tui`** — interactive TUI (ratatui + crossterm) for `rp2350-emu`, with panels and a device frontend (LCD, benchmark).
- **`rp2040-emu`** — the RP2040 emulator core library (dual Cortex-M0+, bus, memory, clocks, SIO, PIO).
- **`rp2040-emu-tui`** — interactive TUI for `rp2040-emu`.
- **`picoem-harness`** — all differential and hardware-in-the-loop test binaries. Binaries are chip-suffixed (`qemu_diff_m33` / `qemu_diff_m0plus`, `probe_diff_rp2350` / `probe_diff_rp2040`, etc.).
- **`picoem-debug`** — GDB RSP server and trace tooling. Stubbed.
- **`picocalc-board`** — PicoCalc pin map and external LCD, keyboard, PSRAM, SD, audio-observation, framebuffer, and report-input observations.
- **`picocalc-harness`** — the `picocalc-run` firmware runner and JSON scenario engine.
- **`epio-sys`** — optional native FFI for the reference PIO simulator; excluded from `default-members`.

The interactive UIs are `rp2350-emu-tui` and `rp2040-emu-tui`. The headless PicoCalc entry point is
`picocalc-run` from `picocalc-harness`. The workspace has no top-level binary.

## Testing

The emulators are validated by independent oracles, each catching different bug classes.

GitHub Actions provides the R4 backend quality gate as three independent jobs on pinned
Ubuntu 24.04 and Rust 1.97.1: tests plus a release runner build, scoped rustfmt, and scoped Clippy
with warnings denied. The formatting and lint boundary is `picocalc-board` plus
`picocalc-harness`; inherited upstream crates are not reformatted as part of PicoCalc work.
`rp2040-emu` remains in the test job because it is the core used by the runner. Passing this CI
does not automatically promote the current backend commit into a firmware target: accepted commits
remain explicit in `picocalc_emu/reference-projects/firmware-targets.json`, and historical targets
continue to run from their pinned checkout.

### 1. Unit tests

```bash
cargo test                      # all crates
cargo test -p rp2350-emu        # RP2350 / RP2354 core only
cargo test -p rp2040-emu        # RP2040 core only
cargo test -p picocalc-board    # PicoCalc board/device models
cargo test -p picocalc-harness  # runner/scenario parser and execution contract
cargo test <name_substring>      # filtered
```

Instruction semantics, decode edge cases, exception mechanics, PIO, and clock-tree config live in each core crate's `tests.rs` / `pio_tests.rs` / `tests/firmware.rs`.

### 2. QEMU differential harness (per chip)

Each oracle spawns a QEMU reference CPU, connects over GDB, runs the same instruction in both QEMU and the emulator, then diffs R0–R15 + xPSR (masking architecturally unpredictable flag fields).

```bash
# RP2350 / Cortex-M33 oracle (GDB port 3333)
cargo run -p picoem-harness --release --bin qemu_diff_m33
cargo run -p picoem-harness --release --bin qemu_diff_m33 -- --fuzz 100000
cargo run -p picoem-harness --release --bin qemu_diff_m33 -- --fuzz 100000 --seed <S>

# RP2040 / Cortex-M0+ oracle (GDB port 3334, uses QEMU `cortex-m0` — see `tech_debt.md`)
cargo run -p picoem-harness --release --bin qemu_diff_m0plus
cargo run -p picoem-harness --release --bin qemu_diff_m0plus -- --fuzz 100000
cargo run -p picoem-harness --release --bin qemu_diff_m0plus -- --fuzz 100000 --seed <S>
```

Requires `qemu-system-arm` on `PATH`.

On Windows, a running `.exe` is locked against overwrite, so a long fuzz session will block concurrent `cargo build --release` (or any build that relinks the harness). Copy the binary out before a long run — e.g. `cp target/release/qemu_diff_m33.exe /tmp/fuzzer.exe && /tmp/fuzzer.exe --fuzz 100000`. The overnight drivers under `fuzz-runs/` handle this automatically. See `CLAUDE.md` for the full note.

### 3. Hardware-in-the-loop (real silicon)

Drive a real RP2354 board over SWD via a Pi Pico debug probe, single-step it, and diff against the emulator. Catches behaviours QEMU doesn't model correctly — e.g. pipeline effects, peripheral timing. `bank_conflict_test_rp2350` characterises SRAM bank-contention timing on silicon for reference; the emulator itself does **not** model bank contention on RP2350 (see `CLAUDE.md` / `wrk_journals/2026.04.15 - JRN - Contention Modelling Declined.md` for rationale).

```bash
# Same instruction-level test suite as qemu_diff_m33 but against silicon
cargo run -p picoem-harness --release --bin probe_diff_rp2350

# Register / DWT cycle-counter sanity checks
cargo run -p picoem-harness --release --bin probe_verify_rp2350

# SRAM bank-conflict timing characterisation
cargo run -p picoem-harness --release --bin bank_conflict_test_rp2350
```

Requires a Pi Pico configured as a `probe-rs`-compatible debug probe wired to an RP2354 target (for the `*_rp2350` binaries) or a Pico V1 / RP2040 target (for `probe_diff_rp2040`). On a host with both probes attached, disambiguate with `--probe <VID:PID:SERIAL>` — `probe-rs list` shows the available serials.

### 4. Paced benchmark and full integration

```bash
cargo run -p picoem-harness --release --bin paced_bench_rp2350
cargo run -p picoem-harness --release --bin full_test_rp2350
```

Measures real-time throughput with wall-clock pacing and runs a larger integration smoke. Useful for regression-checking performance work.

### Coverage

```bash
cargo llvm-cov
```

## Design Documents

Phase HLDs live under `wrk_docs/`. Filenames follow `YYYY.MM.DD - HLD - <topic> V<N>.md`. Start with `2026.04.12 - RP2350 Emulator HLD.md` for the original design (with an errata note at the top tracking the post-restructure crate renames), then the phase docs (bus fabric, interrupts, dual-core, PIO, coprocessors/FPU) for subsystem detail. Newer dated versions supersede earlier drafts of the same phase. The workspace restructure is documented in `2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

Per-session journals (investigations, performance work, review cycles) live under `wrk_journals/`. Known cycle-timing gaps against real silicon and post-Phase-7 residuals are in `tech_debt.md`.

## Requirements

- Rust (edition 2024, stable)
- `qemu-system-arm` for `qemu_diff_m33` / `qemu_diff_m0plus` (any reasonably recent release; the M0+ oracle uses `cortex-m0` because QEMU 10.2 doesn't ship a `cortex-m0plus` model — see `tech_debt.md`)
- A Pi Pico debug probe + RP2354 target board for the `probe_*_rp2350` harnesses (optional)

The `pacer` module uses an `rdtsc` backend on `x86_64` and an `Instant` backend elsewhere, so serial emulator builds and TUIs support macOS, Linux, and Windows. The threaded worker runtime remains limited to `x86_64` Windows/Linux.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

This repository redistributes third-party content under their respective licenses — the Raspberry Pi RP2350 bootrom (BSD-3-Clause), the composite RP2040 B2 bootrom (BSD-3-Clause plus the separate `mufplib` terms), the PicoGUS firmware (GPL-2.0-or-later), OneROM RP2350 firmware fixtures (MIT), the SeaBIOS payload used by OneROM fixtures (LGPL-3.0), the `epio`/`apio` MIT submodules used by the PIO differential helper, and a vendored fork of probe-rs (MIT OR Apache-2.0). See [NOTICE](NOTICE) for the full list and attribution.

## Trademarks

*Raspberry Pi*, *RP2350*, *RP2354*, *RP2040*, and *Pico* are trademarks of Raspberry Pi Ltd. *Arm*, *Cortex-M0+*, *Cortex-M33*, *Armv6-M*, *Armv8-M*, and *NEON* are trademarks or registered trademarks of Arm Limited (or its subsidiaries) in the US and/or elsewhere. *Sound Blaster* is a trademark of Creative Technology Ltd. *AdLib*, *Gravis Ultrasound*, and *MT-32* are trademarks of their respective owners. *Monkey Island* is a trademark of Lucasfilm Entertainment Company Ltd. LLC.

This project is an independent emulator and is not affiliated with, endorsed by, or sponsored by any of the above trademark holders. All trademarks are used for identification purposes only.
