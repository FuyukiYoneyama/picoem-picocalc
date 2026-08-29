# Validated realtime preview backend (VRP-2)

This repository contains the authoritative `picocalc-run` process used by the
validated realtime preview project.  The backend is still a headless emulator;
it does not contain a GUI and it does not claim realtime-1x qualification.

## Process contract

Build the runner from a clean, pinned backend checkout.  The preview mode is
selected with `--preview-api` and uses stdin/stdout as a local duplex pipe:

```text
runner stdout  -> framed PCRP messages -> preview frontend
preview stdin  -> framed PCRP commands -> runner
runner stderr  -> diagnostics only
```

`--preview-api` is a separate mode.  It cannot be combined with a scenario,
machine API, report/artifact output, conformance expectation, or diagnostic
profiler.  Those restrictions keep preview presentation from changing the
authoritative batch path.

The frozen wire contract is maintained by the `picocalc_emu` repository:

- `docs/validated-realtime-preview/preview-ipc-schema-v1.json`
- `docs/validated-realtime-preview/preview-ipc-fixture-v1.json`

The runner validates magic, version, direction, sequence, payload bounds,
canonical JSON, and binary payload lengths before dispatching a command.  An
unknown, truncated, out-of-order, or malformed frame is a protocol error and
terminates with exit code 2; bytes are never reinterpreted as another command.
Only an explicit `quit` command is a clean preview termination.

## Supported backend messages

Startup emits `hello` and an initial `status`.  The loop then emits:

- incremental UART0 TX bytes (`u64 virtual cycle + u8 byte`);
- status snapshots containing virtual cycle/time, pacer ratio/lag/behind count,
  framebuffer update count, UART RX/TX/drop state, and the versioned
  observation projection/digest for UART, framebuffer, unsupported-MMIO, and
  audio-sink state.  The projection's audio member is the complete bounded
  DMA-to-PWM surface shared with schema-8 `audio_sink`; post-quantizer
  loudness/rail metrics remain in the separate `--audio-analysis` artifact and
  are not silently folded into the VRP-2 digest;
- RGB565 framebuffer snapshots when the LCD model changes;
- `error` messages for rejected key input, disabled/overrun UART RX, or a full
  keyboard queue;
- `goodbye` after a clean `quit`.

Input commands are `key_event`, `uart_rx`, `reset`, and `quit`.  Host arrival
time is not written into the emulated clock.  UART input goes to the guest's
  PL011 RX FIFO and is independent of PicoCalc keyboard events or process
  stdin.  `reset` resets CPU/peripheral state and re-enters the selected boot
  handoff while retaining attached flash and SD media.

PCM framing is now emitted by the preview runner as bounded 128-frame source
blocks.  The backend uses a bounded audio tap and asynchronous PCRP output
writer; a slow host reader or player cannot block emulated virtual time.  The
host frontend owns resampling, playback, and presentation-drop accounting.
`audio.state=not_streamed` remains the legacy authoritative digest field, while
host monitor state is reported separately.  The backend is still not an audio
player and does not claim speaker-quality or hardware-audio qualification.

The existing JSON Lines machine API can request the same projection with
`{"schema":1,"id":"obs","op":"observe","domains":["preview"]}`.
The response carries `schema_version`, `virtual_cycle`, `projection`, and its
canonical `digest_sha256`.  This is an observation surface only.  The
`--replay-scenario` option drives a registered scenario to completion before a
machine-API `observe` request or preview status is served; it is intended for
the `picocalc.py preview-digest-gate` comparison and is not a replacement for
the schema-8 batch report.  A board-backed synthetic same-cycle comparison is
covered by the local smoke gate.  Registered-target admission and the
four-way digest decision remain owned by `picocalc_emu`.

## Verification boundary

The VRP-2 implementation is protected locally by:

```sh
cargo test -p picoem-common --locked
cargo test -p rp2040-emu --locked
cargo test -p picocalc-harness --locked
cargo test -p picocalc-harness --test preview_api_e2e --locked
cargo test -p picocalc-harness --test machine_api_schema1_golden --locked
cargo clippy -p picocalc-harness --tests --locked -- -D warnings
```

The preview E2E test speaks the wire directly, confirms UART direction and
clean quit, injects an unknown message kind to verify fail-closed exit, and
compares a board-backed synthetic UART fixture against the batch report and
machine API at one exact virtual cycle. The comparison covers the complete
report-compatible UART, initial RGB565 framebuffer, unsupported-MMIO, and
audio-sink DMA-to-PWM observation projection. This is a local backend smoke gate; it is
not target admission, realtime qualification, or a GitHub Actions trigger.
The `machine_api_schema1_golden` test replays the repository-owned
`crates/picocalc-harness/tests/fixtures/machine-api-schema1-golden.jsonl`
transcript against the real runner and protects the established schema-1
`run`/`step`/`run_until`/`input`/`observe`/`subscribe`/`snapshot` responses.
The UART RX positive/overrun case is part of `preview_api_e2e`; its generated
fixture is temporary and does not change the authoritative firmware report.

## Current qualification status

VRP-2 provides a deterministic preview backend API on the existing Serial
emulator.  The board-backed report-compatible cross-API smoke gate, the
machine-API schema-1 golden transcript, and the directional UART RX
positive/overrun evidence are present. It does not yet provide:

- the real registered-target digest result (the backend now exposes the
  replay-only boundary consumed by `picocalc.py preview-digest-gate`, but the
  currently recorded VRP-2 targets predate complete `audio_sink` observation
  and the gate must not fill that data by inference);
- a GUI, PicoCalc device skin, or automatic UART window (VRP-3);
- a formally qualified host PCM monitor: VRP-4's bounded tap, asynchronous
  transport, variable-rate resampling, and drop states are implemented and
  locally tested, but the registered-target off/on/forced-drop evidence gate is
  still pending;
- a `REALTIME OK` or `realtime-1x-qualified` capability (VRP-5 and
  `VRP-NES-0` are still required).

Do not add the preview mode to the normal validation target registry or change
the firmware report schema without a new versioned contract and evidence in
`picocalc_emu`.
