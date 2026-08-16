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
