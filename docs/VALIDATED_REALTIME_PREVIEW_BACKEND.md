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
  audio-sink state;
- RGB565 framebuffer snapshots when the LCD model changes;
- `error` messages for rejected key input, disabled/overrun UART RX, or a full
  keyboard queue;
- `goodbye` after a clean `quit`.

Input commands are `key_event`, `uart_rx`, `reset`, and `quit`.  Host arrival
time is not written into the emulated clock.  UART input goes to the guest's
  PL011 RX FIFO and is independent of PicoCalc keyboard events or process
  stdin.  `reset` resets CPU/peripheral state and re-enters the selected boot
  handoff while retaining attached flash and SD media.

PCM framing is reserved by the schema, but VRP-2 reports
`audio.state=not_streamed`; bounded host audio transport is the separate VRP-4
work package.  The backend therefore must not be described as an audio player.

The existing JSON Lines machine API can request the same projection with
`{"schema":1,"id":"obs","op":"observe","domains":["preview"]}`.
The response carries `schema_version`, `virtual_cycle`, `projection`, and its
canonical `digest_sha256`.  This is an observation surface only: VRP-2 still
needs a deterministic same-cycle runner comparison and admission gate before
the projection can be called a qualification result.

## Verification boundary

The VRP-2 implementation is protected locally by:

```sh
cargo test -p picoem-common --locked
cargo test -p rp2040-emu --locked
cargo test -p picocalc-harness --locked
cargo test -p picocalc-harness --test preview_api_e2e --locked
cargo clippy -p picocalc-harness --locked -- -D warnings
```

The preview E2E test speaks the wire directly, confirms UART direction and
clean quit, injects an unknown message kind to verify fail-closed exit, and
compares a board-backed synthetic UART fixture against the batch report and
machine API at one exact virtual cycle. The comparison covers the complete
report-compatible UART, initial RGB565 framebuffer, unsupported-MMIO, and
audio-sink observation projection. This is a local backend smoke gate; it is
not target admission, realtime qualification, or a GitHub Actions trigger.

## Current qualification status

VRP-2 provides a deterministic preview backend API on the existing Serial
emulator.  The board-backed report-compatible cross-API smoke gate is present,
but it does not yet provide:

- target-admission integration for the cross-API boundary digest (the shared
  `src/session.rs` owns the session state and stepping boundary, and the
  preview/machine APIs expose a versioned projection and digest; the smoke
  fixture is synthetic and is not a registered validation target);
- a GUI, PicoCalc device skin, or automatic UART window (VRP-3);
- bounded host PCM streaming (VRP-4);
- a `REALTIME OK` or `realtime-1x-qualified` capability (VRP-5 and
  `VRP-NES-0` are still required).

Do not add the preview mode to the normal validation target registry or change
the firmware report schema without a new versioned contract and evidence in
`picocalc_emu`.
