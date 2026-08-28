# PicoCalc harness fixtures

The CLI end-to-end tests generate their RP2040 raw flash fixture at test time;
no generated binary is checked in.  The generator is deliberately small and
is part of `cli_audio_e2e.rs` so that the source, instruction encodings, and
generation procedure are reviewed together.

The fixture is an original test program written for this repository.  It is
covered by the repository's MIT OR Apache-2.0 license.  It releases the
RP2040 DMA, TIMER, UART0, and PWM reset gates, emits an `AUDIO_FIXTURE` UART
marker, configures a four-word timer-paced DMA transfer to PicoCalc PWM slice
5 CC, and then loops.  The test invokes the real `picocalc-run` binary with
`--board none --audio-analysis --audio-wav` and checks the resulting schema-8
report, observed sample rate, WAV header, and PCM digest.

The program uses only Thumb-1 literal loads, word stores, and a self-branch;
the test does not require an ARM cross compiler or an external firmware
workspace.

## Machine API and preview fixtures

`machine-api-schema1-golden.jsonl` is a checked-in JSONL exchange transcript for
the established machine API schema 1. The integration test sends its
`observe`, `step`, `subscribe`, `run`, `input`, `run_until`, and `snapshot`
requests to the real runner and compares each response with the fixture. It is
a compatibility guard; the preview-only observation domain is tested
separately so the existing transcript remains unchanged.

The preview UART RX positive/overrun test generates its small Thumb-1 echo
firmware in `preview_api_e2e.rs` at test time. It is intentionally not an
opaque checked-in binary: the source documents the synthetic boot handoff and
instruction encodings, and the temporary image is removed after the test.
