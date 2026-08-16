# RP2040 DMA and PicoCalc audio observability

This document describes the public DMA and audio-observation behaviour of the
RP2040 backend. It is part of the emulator contract: source changes that alter
the model or the report fields must update this document and its tests in the
same change.

## Scope

The Serial RP2040 model advances DMA with the following arbitration rules:

- eligible windows use one arbitration decision per system clock;
- `CHn_CTRL.HIGH_PRIORITY` channels are considered before normal channels;
- within either tier, the lowest channel number wins;
- FORCE DREQ, DMA timer DREQ, chaining, and ring addressing use the same DMA
  transfer path; and
- a ready non-timer DREQ keeps the window on the per-system-clock path because
  it can compete with a timer event.

An event-driven timer path is used only when the current window is eligible for
it. This preserves timer-event positions while avoiding unnecessary per-clock
work in windows that cannot contain a competing ready request. A peripheral
DREQ that becomes ready inside a bulk window is not inferred unless the window
is already forced onto the per-clock path.

The `rp2040-emu` Serial model remains the correctness reference. These rules
are an emulator model, not a claim that every unobserved silicon arbitration
detail has been proven.

## Digital audio capture

The PicoCalc harness observes DMA-origin writes to PWM slice 5's CC register.
It does not model the analogue PWM output, amplifier, speaker, enclosure, room,
or human volume control. The optional WAV is therefore an unnormalised digital
observation suitable for comparing emulator runs and for listening checks.

Audio capture can be requested without attaching the LCD/keyboard board model:

```bash
cargo run --release -p picocalc-harness --bin picocalc-run -- \
  --bin /absolute/path/to/picocalc_app.bin \
  --bootrom /absolute/path/to/bootrom-rp2040-b2.bin \
  --board none \
  --audio-analysis /tmp/picocalc-audio-analysis.json \
  --audio-wav /tmp/picocalc-audio-raw.wav
```

The audio sink is enabled when an audio count/hash expectation, analysis path,
or WAV path is supplied. The analysis reconstructs signed 16-bit stereo PCM
from the observed 8-bit PWM duty stream and records the observed timer-derived
sample rate. It reports structural counters such as DMA writes, wrong width or
TREQ, due-cycle gaps, block boundaries, and unexpected gaps, plus level metrics
such as peak, RMS, DC offset, active-frame ratio, and rail occupancy.

### Timer-miss fields

The audio report also contains these additive diagnostic fields:

| Field | Meaning |
|---|---|
| `timer_event_count` | Timer DREQ events generated for the observed timer. |
| `timer_miss_count` | Events left unconsumed by the DMA arbitration model. |
| `timer_miss_audio_not_busy` | Events generated while the audio DMA channel was not busy. |
| `timer_miss_other_dma_selected` | Events that lost arbitration to another ready DMA channel. |
| `timer_miss_no_dma_selected` | Events for which no DMA channel was selected. |
| `timer_miss_multiple_due_in_window` | Events coalesced beyond one service in an eligible window. |

These are emulator-side timer/DMA observations. They are not a direct count of
firmware audio-ring underruns. A firmware UART message such as an underrun
counter must be captured and interpreted separately. In particular, a timer
miss while the audio channel is not busy can be a startup or teardown condition
and must not automatically be reported as a firmware defect.

The frozen NEXT-2 48 kHz stereo contract remains separate from observed-rate
audio analysis. A stream using another timer rate is represented by the
additive audio-analysis schema already described in the root README; it is not
silently coerced to 48 kHz.

## Starting an event-horizon profile after a firmware marker

The diagnostic-only `event-horizon-profiler` feature can defer the running
event profile until a UART marker has appeared. This is useful when startup
initialisation would otherwise dominate the profile:

```bash
cargo run --release -p picocalc-harness --features event-horizon-profiler -- \
  --bin /absolute/path/to/picocalc_app.bin \
  --bootrom /absolute/path/to/bootrom-rp2040-b2.bin \
  --board none \
  --event-horizon-profile /tmp/event-horizon.json \
  --event-horizon-profile-after-uart '[READY]'
```

The report records the activation mode, marker, and virtual start cycle. The
start cycle is the cycle at which the runner recognises the marker in drained
UART bytes and enables the profile; it is not an assertion of the exact cycle
at which firmware wrote the final UART byte. The feature is diagnostic-only
and must not be used as a wall-clock performance result. It cannot be combined
with the JSON machine API.

## Quantum-invariance test

The public integration test compares the same DMA workloads at quantum 1, 16,
and 64 while keeping the **actual** master-cycle boundary fixed. `Emulator::run`
is specified as running for at least the requested number of cycles and may
overshoot at an instruction boundary, so the fixture selects a common boundary
for all three quanta rather than comparing state at different virtual times:

```bash
cargo test -p rp2040-emu --test dma_quantum_invariance
```

The workloads cover FORCE transfer, timer-paced transfer, two-channel FORCE
competition, chain plus read-ring operation, a fixed-destination timer-paced
PWM audio fixture, and five HIGH_PRIORITY／timer contention cases (including
same-cycle timer tie-break, audio-versus-FORCE, and chain-induced priority-tier
change). The test directly compares destination data, channel
addresses/count/control/busy state, DMA interrupt/NVIC state, timer registers and
accumulators, cumulative event/miss classifications, and the complete audio
PCM/due-cycle/block/latency observation. Chain and read-ring behaviour are
exercised through explicit end-state assertions.

`timer_due_cycle` and `timer_window_*` are deliberately window-local fields:
the final tick window is partitioned differently at different quanta. The test
checks their internal consistency (including due-cycle presence when a window
contains events) rather than falsely requiring those last-window values to be
equal. Audio-selected due cycles remain a cumulative digest and are compared
exactly. The 10/10 workload comparison is a local validation result; it does not
yet make the backend eligible for a promoted PicoCalc target pin.

Passing this test means that the tested model is invariant for those workloads
under the tested scheduler quanta. It does not certify the complete PicoCalc
firmware, analogue sound quality, or physical RP2040 behaviour. Full firmware
acceptance still requires the normal schema-8 report, scenario expectations,
artifact identity checks, and (where applicable) a hardware comparison.

## Public-change rule

Changes to DMA arbitration, audio-sink report fields, or diagnostic activation
must update all of the following in one reviewable change:

1. the implementation and unit/integration tests;
2. this document and the relevant crate/root README section;
3. `CHANGELOG.md`, including any compatibility or diagnostic-only limitation;
4. the report/schema or cross-repository contract when a field is normative.

Generated reports, WAV files, and local firmware/ROM images remain evidence
artefacts and are not part of this public emulator contract unless explicitly
added through the repository's fixture policy.
