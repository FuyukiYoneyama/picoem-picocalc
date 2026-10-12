# RP2040 SIO divider direct register diagnostic — 2026-10-12

This is diagnostic evidence for [the open issue](../../rp2040_sio_divider_bug_20261012.md).
**The recorded backend is the unmodified pre-fix revision.** The later
[emulator correction](../../rp2040_sio_divider_fix_20261012.md) has separate
regression evidence; these original observations are retained. No ClockCalc
firmware, CPU instruction stream or worker threads were run.

## Inputs and results

- Backend main: `121d649930d625924fa7608a3b72ed622098f522`, clean at execution.
- Host: x86_64 Linux; Rust 1.97.1; offline Cargo debug build with `threading`.
- `main.rs`: calls the actual Serial `Sio` and Threaded `WorkerBus` methods.
  WorkerBus objects share state but are called sequentially on the host thread.
  Arithmetic cases seed 0 / 3 before writing their operands; result observation
  reads REMAINDER then QUOTIENT. No real divider latency is exercised.
- `results.jsonl`: 17 cases per implementation; Serial 11 mismatches,
  WorkerBus 10 mismatches. The other observations match their specified values.
- `metadata.json`: execution identity, source/SDK hashes, scope and counts.
- `Cargo.toml` / `Cargo.lock`: exact external probe manifest and dependency lock.
  The manifest records the original machine path; adapt it when replaying.
- `build.log`: stderr from the final invocation after refining the last DIRTY
  check to record both the value before and after the dividend write. The first
  build resolved the lockfile offline; the final invocation used `--locked`.
- `SHA256SUMS`: hashes of these evidence files, excluding itself.

Exit 0 means observations were recorded. It does **not** mean conformance:
each JSON row carries `matches_spec`. Arithmetic `actual` and `expected`
contain `[quotient,remainder]` encoded as u32. DIRTY checks use `[dirty,0]`,
except `dividend_write_after_result_consumption`, which uses
`[dirty_before,dirty_after]`. DIRTY is represented by bit mask 2.

The final DIRTY case requires a clean state after result consumption. WorkerBus
already fails that precondition, so `[2,2]` must not be read as proof that its
dividend write sets DIRTY correctly.

## Replay with an isolated project

Use two independently selected inputs:

- `SIO_REVIEW_BACKEND`: an existing backend checkout at
  `121d649930d625924fa7608a3b72ed622098f522`, used as the library dependency.
- `SIO_REVIEW_EVIDENCE`: this diagnostic directory from the checkout or bundle
  containing the follow-up review, including `main.rs`, `Cargo.lock`,
  `metadata.json` and `SHA256SUMS`. The backend commit above predates these
  artifacts and does **not** contain this directory. Obtain the diagnostic
  files separately and verify them with `sha256sum -c SHA256SUMS` from their
  directory. Retain the bundle's checksum list with the replay output.

The following creates only a temporary Cargo project; it leaves both input
directories unchanged. It checks the backend revision before preparing the
project. Offline replay requires cached locked dependencies and x86_64
Linux or Windows for the exposed WorkerBus API; this shell example uses Linux.

```sh
set -eu
export SIO_REVIEW_BACKEND=/path/to/backend-at-121d649
export SIO_REVIEW_EVIDENCE=/path/to/checkout-containing-this-review/docs/evidence/sio-divider-review-20261012
SIO_REVIEW_TMP=$(mktemp -d /tmp/sio-divider-review.XXXXXX)
export SIO_REVIEW_TMP
python3 - <<'PY'
import json, os, shutil, subprocess
from pathlib import Path
p = Path(os.environ['SIO_REVIEW_TMP'])
e = Path(os.environ['SIO_REVIEW_EVIDENCE'])
backend = Path(os.environ['SIO_REVIEW_BACKEND']).resolve()
recorded = json.loads((e / 'metadata.json').read_text())['backend_commit']
actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=backend, text=True).strip()
if actual != recorded:
    raise SystemExit(f'Backend revision mismatch: expected {recorded}, got {actual}')
(p / 'src').mkdir()
shutil.copyfile(e / 'main.rs', p / 'src/main.rs')
shutil.copyfile(e / 'Cargo.lock', p / 'Cargo.lock')
crate = str(backend / 'crates/rp2040-emu')
(p / 'Cargo.toml').write_text(
    '[package]\nname="sio-divider-review"\nversion="0.0.0"\nedition="2024"\n'
    '\n[dependencies]\nrp2040-emu={path=' + json.dumps(crate) + ',features=["threading"]}\n')
PY
CARGO_TARGET_DIR="$SIO_REVIEW_TMP/target" cargo run --offline --locked \
  --manifest-path "$SIO_REVIEW_TMP/Cargo.toml" \
  > "$SIO_REVIEW_TMP/results.jsonl" 2> "$SIO_REVIEW_TMP/build.log"
```

Retain the printed/recorded observations and source identity before removing
temporary build output. Successful register checks alone do not establish
firmware recovery, concurrent runtime behavior or 8-cycle timing fidelity.

## Applicability after a correction

The retained `main.rs` is a diagnostic for the pre-fix **synchronous** divider
model. It reads results immediately after operand writes and never advances
emulated cycles. It can check an operand-trigger or DIRTY correction only
while the divider still calculates synchronously; record any such corrected
backend's identity and output separately from the historical replay above.

Once calculation latency is implemented, immediate result reads can occur
while READY=0, when result contents are undefined. The unchanged program's
`matches_spec` rows then cannot serve as conformance results. Use a separate
diagnostic that advances the appropriate emulated clock, checks READY with a
completion budget, and reads REMAINDER/QUOTIENT only after READY=1. Polling
registers alone will not advance time in this direct-call diagnostic. Test
the 8-cycle READY boundaries separately, and preserve the new program and
source identity with its results. That diagnostic has not been implemented
or executed as part of this documentation correction.
