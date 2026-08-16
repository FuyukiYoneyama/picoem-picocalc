//! `picocalc-run` — headless RP2040 firmware runner for the PicoCalc
//! firmware-conformance track (Gate 1 of `picocalc_emu`'s
//! `docs/IMPLEMENTATION_PLAN.md`).
//!
//! One command in, one JSON report + one raw UART byte stream out. No
//! TUI, no wall-clock pacing, no interactive state — the same inputs
//! must produce byte-identical outputs on every run, because the
//! reports are the evidence artefacts a Gate is accepted on.
//!
//! Boot sequence (plan §3.1 "shortcut"):
//!
//! 1. Build a Serial-model emulator with the firmware pre-loaded as the
//!    XIP flash image (appears at `0x1000_0000`).
//! 2. `load_bootrom` — the 16 KB RP2040 B2 bootrom is *loaded but never
//!    executed*. It is there so SDK firmware can resolve ROM function
//!    table pointers (`rom_func_lookup`). The real bootrom would sample
//!    QSPI pads we do not model and park in USB-MSC boot forever.
//! 3. `reset`, then `direct_boot_from_flash(0x100)` — seeds SP / PC /
//!    VTOR straight from the SDK vector table at flash offset `0x100`,
//!    exactly what boot2 does on silicon.
//!
//! Stop reasons: `cycle_limit` (budget exhausted — only acceptable when
//! explicitly named by the conformance contract),
//! `pc_match` (`--stop-pc` reached, used for "did we get to `main`?"),
//! `exception` (core 0 entered NMI or HardFault), `error` (both cores
//! halted, or the emulator returned an error).
//!
//! Determinism rules: the report carries no wall-clock time, no
//! absolute paths (basenames only), and no host-dependent values. The
//! unsupported-MMIO list is sorted by `(addr, pc)`.

use std::collections::BTreeSet;
use std::io::{BufRead, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use picocalc_board::sha256::sha256_hex;
use picocalc_board::{
    Framebuffer, KeyEvent, KeyState, Keyboard, KeyboardWire, LcdPioWire, SdCard, SdCardWire,
    SdFormat, St7365p, St7365pWire, pins,
};
#[cfg(feature = "behavior-trace")]
use rp2040_emu::{BehaviorEventDomain, BehaviorTraceSnapshot};
use rp2040_emu::{Config, Emulator, EmulatorBuilder};
#[cfg(feature = "idle-profiler")]
use rp2040_emu::{
    CumulativeHistogramSnapshot, IDLE_HISTOGRAM_BUCKETS, IDLE_PROFILE_SCHEMA_VERSION,
    IdleBlockerCycles, IdleBlockerEpisodes, IdleHorizonEvents, IdleProfileSnapshot,
};
#[cfg(feature = "event-horizon-profiler")]
use rp2040_emu::{
    DecodeProfileSnapshot, RUNNING_EVENT_PROFILE_SCHEMA_VERSION, RunningBoundaryEvents,
    RunningEventProfileSnapshot,
};

mod machine_protocol;
mod scenario;

/// Report schema version. Bump on any breaking field change.
///
/// * 1 — Gate 1: boot / UART / unsupported-MMIO report.
/// * 2 — Gate 2: optional `lcd` + `framebuffer` sections, `board` field.
/// * 3 — Gate 3: optional `psram` section (`--psram` / `--psram-verify-range`).
/// * 5 — Milestone 3: optional `scenario` section (`--scenario`), and the
///   `scenario_done` stop reason.
/// * 6 — R1: optional `sd` section with the provisioned filesystem format.
/// * 7 — R1: top-level verdict, reasons, and explicit stop/UART expectations.
/// * 8 — R2: compile-time backend identity and dirty-state provenance.
const SCHEMA_VERSION: u32 = 8;

/// Default 16 KB RP2040 bootrom image, relative to the repo root.
const DEFAULT_BOOTROM_PATH: &str = "roms/rp2040/bootrom-rp2040-b2.bin";

/// pico-sdk convention: boot2 occupies the first 256 flash bytes, the
/// application vector table starts right after.
const SDK_VTOR_FLASH_OFFSET: u32 = 0x100;

/// Default cycle budget (plan Gate 1 CLI contract).
const DEFAULT_CYCLE_LIMIT: u64 = 1_000_000_000;

/// System clock seed for the clock tree. Firmware reprograms PLL /
/// dividers itself; this only seeds the pre-PLL state. Matches the
/// value the existing RP2040 harness binaries use.
const DEFAULT_SYS_CLK_HZ: u32 = 125_000_000;

/// Master-clock cycles per `Emulator::step` when `--stop-pc` is *not*
/// in play. Larger quantum = fewer dispatch round-trips.
const QUANTUM_FREE_RUN: u32 = 64;

/// Master-clock cycles per `Emulator::step` when `--stop-pc` is set.
/// With a quantum of 1 the serial scheduler retires exactly one
/// instruction per core per step, so the post-step PC observation sees
/// every architectural instruction boundary — a quantum of 64 would
/// stride straight past the target address.
const QUANTUM_PC_WATCH: u32 = 1;

/// Master-clock cycles per `Emulator::step` when a board with off-chip
/// devices is attached.
///
/// **Why 16 is safe, and why it is not "as small as possible".**
/// Framing correctness needs one property: when the SSP drains a word to
/// the panel, the CS/DC levels the CPU intended for *that* word must
/// still be the ones on the pads. `Bus::tick_peripherals` samples the
/// pads immediately before the drain, and the firmware cannot have moved
/// on: every path in `lcdspi.c` waits for `SSPSR.BSY` to fall (pico-sdk
/// `spi_write_blocking`, and `spi_finish` for the `spi_write_fast`
/// bursts) before touching a control line, and BSY stays asserted while
/// the TX FIFO is non-empty. So the CPU is architecturally parked in a
/// poll loop across the whole drain window regardless of quantum size.
///
/// 16 is therefore a margin, not a requirement: it keeps IRQ-delivery
/// latency and the PWM-interrupt cadence tight enough that the audio ISR
/// firing mid-transfer stays representative, while still amortising the
/// dispatch overhead over a run measured in tens of millions of cycles.
/// Override with `--quantum`.
const QUANTUM_BOARD: u32 = 16;

/// Master-clock cycles per `Emulator::step` when `--psram` is attached
/// (and no explicit `--quantum` overrides it).
///
/// Unlike the LCD's SSP (where the CPU polls `SSPSR.BSY` and is
/// architecturally parked for the whole drain — see [`QUANTUM_BOARD`]),
/// the PSRAM is driven by a free-running PIO state machine over DMA:
/// the CPU only polls the *DMA channel's* busy flag, which says nothing
/// about where the PIO program counter is mid-transfer. `Bus::step`'s
/// slow path takes a single `bus.gpio_in` snapshot at the top of
/// `tick_pio_and_route_irqs` and reuses it for the *entire* quantum's
/// `pio.step_n(cycles, gpio_in)` call — `update_gpio()` (which feeds
/// `Psram::tick` and splices its MISO bit back in) only runs once, at
/// the end. With `clkdiv=1.0` (picocalc_helloworld's setting — one PIO
/// instruction per sysclk) any quantum > 1 would let every SCK/CS edge
/// inside it go unseen by the PSRAM model until the quantum boundary,
/// exactly the failure mode `tech_debt.md`'s "PSRAM PIO-integration
/// tests cover only 1 edge/quantum" entry warns about. Forcing 1 here
/// keeps every edge synchronous with the PSRAM's `tick()` — matching
/// the same choice already made by the `onerom_*` and
/// `picogus_diff_rp2040` harnesses for the same reason.
const QUANTUM_PSRAM: u32 = 1;

/// How often (in steps) the UART TX log is drained during free-run.
/// Draining is cheap (a `mem::take` of a `Vec`), but not free at one
/// call per cycle.
const UART_DRAIN_INTERVAL: u64 = 256;

/// Backend commit, embedded at build time when the environment
/// provides it (`PICOEM_BACKEND_COMMIT=$(git rev-parse HEAD) cargo
/// build …`), overridable per-run with `--backend-commit`. Never
/// shelled out to at run time: the report must be a pure function of
/// its inputs.
const BUILT_BACKEND_COMMIT: &str = env!("PICOEM_BUILT_COMMIT");

fn built_backend_dirty() -> bool {
    env!("PICOEM_BUILT_DIRTY") == "true"
}

fn validate_backend_identity(expected: &str, built: &str, dirty: bool) -> Result<(), String> {
    if built != expected {
        return Err(format!(
            "runner was built from backend {built} but {expected} was required"
        ));
    }
    if dirty {
        return Err("runner was built from a dirty backend worktree".to_string());
    }
    Ok(())
}

/// Which off-chip board model to attach. `None` keeps the Gate 1
/// behaviour: a bare RP2040 with nothing on its pins.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Board {
    None,
    PicoCalc,
}

impl Board {
    fn as_str(self) -> &'static str {
        match self {
            Board::None => "none",
            Board::PicoCalc => "picocalc",
        }
    }

    fn parse(s: &str) -> Result<Board, String> {
        match s {
            "none" => Ok(Board::None),
            "picocalc" => Ok(Board::PicoCalc),
            other => Err(format!(
                "unknown --board '{other}' (expected none|picocalc)"
            )),
        }
    }
}

/// Which display transport the firmware uses.
///
/// The panel is the same part in both cases; only how bytes reach it
/// differs. Variant A is the official sample's hardware SPI1 path with
/// an RGB666 three-byte container. Variant B is the Canonical BSP
/// default: a PIO0 shift program, RGB565, two bytes per pixel.
#[derive(Clone, Copy, PartialEq, Eq)]
enum LcdVariant {
    A,
    B,
}

impl LcdVariant {
    fn as_str(self) -> &'static str {
        match self {
            LcdVariant::A => "hwspi-rgb888",
            LcdVariant::B => "pio-rgb565",
        }
    }

    fn parse(s: &str) -> Result<LcdVariant, String> {
        match s {
            "hwspi-rgb888" | "a" | "A" => Ok(LcdVariant::A),
            "pio-rgb565" | "b" | "B" => Ok(LcdVariant::B),
            other => Err(format!(
                "unknown --lcd-variant '{other}' (expected hwspi-rgb888|pio-rgb565)"
            )),
        }
    }
}

struct Args {
    bin: PathBuf,
    bootrom: PathBuf,
    cycles: u64,
    stop_pc: Option<u32>,
    json: Option<PathBuf>,
    uart: Option<PathBuf>,
    flash_image_out: Option<PathBuf>,
    expected_backend_commit: Option<String>,
    board: Board,
    lcd_variant: LcdVariant,
    fb_png: Option<PathBuf>,
    quantum: Option<u32>,
    psram: bool,
    psram_verify_range: Option<(u32, u32)>,
    keyboard: bool,
    sd: bool,
    sd_image: Option<PathBuf>,
    sd_image_out: Option<PathBuf>,
    sd_format: SdFormat,
    keys: Option<String>,
    scenario: Option<PathBuf>,
    snapshot_dir: PathBuf,
    machine_api: bool,
    run_id: Option<String>,
    progress_interval: Option<u64>,
    expected_stop: Option<StopReason>,
    expected_uart: Vec<String>,
    expected_audio_sink_count: Option<u64>,
    expected_audio_sink_sha256: Option<String>,
    audio_analysis: Option<PathBuf>,
    audio_wav: Option<PathBuf>,
    #[cfg(feature = "idle-profiler")]
    idle_profile: Option<PathBuf>,
    #[cfg(feature = "behavior-trace")]
    behavior_trace: Option<PathBuf>,
    #[cfg(feature = "event-horizon-profiler")]
    event_horizon_profile: Option<PathBuf>,
    #[cfg(feature = "event-horizon-profiler")]
    event_horizon_profile_after_uart: Option<String>,
}

const PROGRESS_CLOCK_CHECK_DISPATCHES: u64 = 256;

fn validate_run_id(run_id: &str) -> Result<(), String> {
    if run_id.is_empty() {
        return Err("--run-id must not be empty".to_string());
    }
    if run_id.len() > 64 {
        return Err("--run-id must be at most 64 ASCII characters".to_string());
    }
    if !run_id
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b':' | b'-'))
    {
        return Err(
            "--run-id may contain only ASCII letters, digits, '.', '_', ':' and '-'".to_string(),
        );
    }
    Ok(())
}

fn validate_progress_interval(seconds: u64) -> Result<(), String> {
    if seconds == 0 {
        return Err("--progress-interval must be >= 1 second".to_string());
    }
    if Instant::now()
        .checked_add(Duration::from_secs(seconds))
        .is_none()
    {
        return Err("--progress-interval is too large for the host monotonic clock".to_string());
    }
    Ok(())
}

/// Best-effort wall-clock progress reporting. This type deliberately lives
/// outside the report/verdict path: heartbeat output is diagnostic metadata,
/// never an emulator observation or an acceptance input.
struct ProgressReporter {
    run_id: String,
    pid: u32,
    interval: Duration,
    started_at: Instant,
    next_deadline: Instant,
    dispatches_since_check: u64,
    sequence: u64,
    enabled: bool,
}

impl ProgressReporter {
    fn new(run_id: String, interval_seconds: u64) -> Result<Self, String> {
        validate_progress_interval(interval_seconds)?;
        let started_at = Instant::now();
        let interval = Duration::from_secs(interval_seconds);
        let next_deadline = started_at.checked_add(interval).ok_or_else(|| {
            "--progress-interval is too large for the host monotonic clock".to_string()
        })?;
        Ok(Self {
            run_id,
            pid: std::process::id(),
            interval,
            started_at,
            next_deadline,
            dispatches_since_check: 0,
            sequence: 0,
            enabled: true,
        })
    }

    fn write_line(&mut self, line: String) {
        if !self.enabled {
            return;
        }
        let mut stderr = std::io::stderr().lock();
        if writeln!(stderr, "{line}")
            .and_then(|_| stderr.flush())
            .is_err()
        {
            // A closed pipe or redirected stderr must not change the
            // firmware verdict. Stop attempting diagnostics after the first
            // failure, but leave the run itself untouched.
            self.enabled = false;
        }
    }

    fn start(&mut self, budget: u64) {
        self.write_line(format!(
            "[PICOCALC][RUN] event=start run={} pid={} budget={budget}",
            self.run_id, self.pid
        ));
    }

    fn maybe_emit(&mut self, cycles: u64, budget: u64) {
        if !self.enabled {
            return;
        }
        self.dispatches_since_check = self.dispatches_since_check.saturating_add(1);
        if self.dispatches_since_check < PROGRESS_CLOCK_CHECK_DISPATCHES {
            return;
        }
        self.dispatches_since_check = 0;

        let now = Instant::now();
        if now < self.next_deadline {
            return;
        }

        self.sequence = self.sequence.saturating_add(1);
        let elapsed_s = now.duration_since(self.started_at).as_secs_f64();
        let rate_mcycles_s = if elapsed_s > 0.0 {
            cycles as f64 / elapsed_s / 1_000_000.0
        } else {
            0.0
        };
        let pct = if budget > 0 {
            cycles as f64 * 100.0 / budget as f64
        } else {
            0.0
        };
        self.write_line(format!(
            "[PICOCALC][RUN] event=heartbeat run={} pid={} seq={} cycles={} budget={} pct={pct:.3} elapsed_s={elapsed_s:.3} rate_mcycles_s={rate_mcycles_s:.3}",
            self.run_id, self.pid, self.sequence, cycles, budget
        ));

        // Do not emit a burst after a long host stall. Advance the deadline
        // past the current time while retaining the requested cadence.
        while self.next_deadline <= now {
            let Some(next_deadline) = self.next_deadline.checked_add(self.interval) else {
                self.enabled = false;
                break;
            };
            self.next_deadline = next_deadline;
        }
    }

    fn finish(&mut self, outcome: &RunOutcome, status: Verdict) {
        if !self.enabled {
            return;
        }
        let now = Instant::now();
        let elapsed_s = now.duration_since(self.started_at).as_secs_f64();
        self.write_line(format!(
            "[PICOCALC][RUN] event=finish run={} pid={} cycles={} elapsed_s={elapsed_s:.3} stop={} exit={}",
            self.run_id,
            self.pid,
            outcome.cycles,
            outcome.stop_reason.as_str(),
            status.exit_code()
        ));
    }
}

/// Parse a `start:len` range, e.g. `0:10000` or `0x100:0x2000` (either
/// side may be hex with a `0x` prefix, or plain decimal).
fn parse_range(raw: &str) -> Result<(u32, u32), String> {
    let (start_raw, len_raw) = raw
        .split_once(':')
        .ok_or_else(|| format!("invalid range '{raw}' (expected START:LEN)"))?;
    let parse_num = |s: &str| -> Result<u32, String> {
        let digits = s.trim_start_matches("0x").trim_start_matches("0X");
        if digits.len() != s.len() {
            u32::from_str_radix(digits, 16).map_err(|e| format!("invalid hex '{s}': {e}"))
        } else {
            s.parse::<u32>()
                .map_err(|e| format!("invalid number '{s}': {e}"))
        }
    };
    Ok((parse_num(start_raw)?, parse_num(len_raw)?))
}

fn validate_sd_selection(
    sd: bool,
    sd_image: Option<&Path>,
    sd_image_out: Option<&Path>,
    format_explicit: bool,
) -> Result<(), String> {
    if sd && sd_image.is_some() {
        return Err("--sd and --sd-image are mutually exclusive".to_string());
    }
    if sd_image_out.is_some() && sd_image.is_none() {
        return Err("--sd-image-out requires --sd-image".to_string());
    }
    if format_explicit && !sd {
        return Err("--sd-format requires --sd".to_string());
    }
    Ok(())
}

fn print_usage() {
    eprintln!(
        "Usage:\n  \
         picocalc-run --bin <firmware.bin> [options]\n\
         \n\
         --bin <path>             Required. Raw RP2040 flash image (.bin), loaded at\n\
                                  0x1000_0000 and direct-booted from offset 0x100.\n\
         --bootrom <path>         16 KB RP2040 bootrom image, loaded but never executed.\n\
                                  Default: {DEFAULT_BOOTROM_PATH}\n\
         --cycles <N>             Cycle budget. Exceeding it gives cycle_limit, which is\n\
                                  not a pass unless explicitly expected. Default: {DEFAULT_CYCLE_LIMIT}\n\
         --stop-pc <hex>          Stop with stop_reason=pc_match when core 0's PC equals\n\
                                  this address (e.g. 0x10000ca8). Forces a 1-cycle step\n\
                                  quantum so every instruction boundary is observed.\n\
         --json <path>            Write the JSON report here. Default: stdout.\n\
         --uart <path>            Write raw UART0 TX bytes here. Default: discarded\n\
                                  (byte count + sha256 still reported).\n\
         --backend-commit <str>   Require the runner's compile-time Git identity to match.\n\
         --board <none|picocalc>  Attach an off-chip board model. 'picocalc' hangs the\n\
                                  ST7365P display off SPI1 (CS=GP13, DC=GP14, RST=GP15)\n\
                                  and adds the 'lcd' + 'framebuffer' report sections.\n\
                                  Default: none\n\
         --fb-png <path>          Write the 320x320 viewport as an 8-bit RGB PNG.\n\
                                  Requires --board picocalc.\n\
         --quantum <N>            Master-clock cycles per step. Overrides the default\n\
                                  ({QUANTUM_FREE_RUN} free-run, {QUANTUM_BOARD} with a board,\n\
                                  {QUANTUM_PC_WATCH} with --stop-pc, {QUANTUM_PSRAM} with --psram).\n\
         --psram                  Attach the off-chip SPI PSRAM (APS6404L, 8 MiB) wired the\n\
                                  way PicoCalc solders it (CS=GP20, SCK=GP21, MOSI=GP2,\n\
                                  MISO=GP3). Works with or without --board picocalc. Forces\n\
                                  a 1-cycle step quantum (see --quantum) unless overridden.\n\
         --psram-verify-range <START:LEN>\n\
                                  After the run, check PSRAM buffer bytes [START, START+LEN)\n\
                                  against the `addr & 0xFF` pattern picocalc_helloworld's\n\
                                  psram_test() writes, and report matched/mismatched counts.\n\
                                  START/LEN accept hex (0x-prefixed) or decimal. Requires\n\
                                  --psram.\n\
         --keyboard               Attach the PicoCalc keyboard controller on I2C1.\n\
         --keys <string>          Queue these characters as key events before the run\n\
                                  starts. Implies --keyboard. For input that has to be\n\
                                  timed against what the program is doing, use --scenario.\n\
         --sd                     Attach an SD card on SPI0, pre-formatted FAT32 by default.\n\
         --sd-image <path>        Attach a non-empty 512-byte-aligned RAW SD image read-only;\n\
                                  emulated writes use a sector copy-on-write overlay.\n\
         --sd-image-out <path>    Atomically export the RAW image plus COW writes after the run.\n\
                                  Requires --sd-image and must differ from the input path.\n\
         --sd-format <fat32|fat16>\n\
                                  Initial filesystem profile. FAT32 is the default, matching\n\
                                  PicoCalc's bundled 32 GB card. Requires --sd.\n\
         --scenario <path>        Run a JSON scenario: timed key input, and pixel / region\n\
                                  / UART assertions checked inside the run loop. Adds the\n\
                                  'scenario' report section. Exit 1 if any step fails.\n\
                                  Milliseconds are virtual, derived from the system clock\n\
                                  the firmware has programmed.\n\
         --snapshot-dir <path>    Where scenario 'snapshot' steps write their PNGs.\n\
                                  Default: the current directory.\n\
         --machine-api            NEXT-4 JSON Lines API on stdin/stdout. Uses the same\n\
                                  startup artifact/device options; no scenario/final report.\n\
         --run-id <ID>            Optional diagnostic ID; requires --progress-interval.\n\
         --progress-interval <N> Emit stderr heartbeat lines every N seconds (opt-in).\n\
         --expect-stop <reason>   Required stop: cycle_limit, pc_match, or scenario_done.\n\
         --expect-uart <text>     Required UART substring. Repeat for each marker.\n\
         --expect-audio-sink-count <N>\n\
                                  Require exactly N DMA-origin PWM5_CC writes.\n\
         --expect-audio-sink-sha256 <hex>\n\
                                  Require the little-endian PWM5_CC stream SHA-256.\n\
         --audio-analysis <path> Write deterministic digital-level metrics reconstructed\n\
                                  from the 8-bit stereo PWM duty stream; may be used without\n\
                                  --board picocalc for audio-only capture.\n\
         --audio-wav <path>      Write the same unnormalised reconstructed stream as\n\
                                  an observed-rate stereo signed-16 WAV for listening; may be\n\
                                  used without --board picocalc for audio-only capture.\n\
         --flash-image-out <path> Export the final 2 MiB XIP image after SSI erase/program.\n\\
         -h, --help               This message."
    );
    #[cfg(feature = "idle-profiler")]
    eprintln!(
        "         --idle-profile <path>   OPT0-A diagnostic JSON (not valid for wall-time measurement)."
    );
    #[cfg(feature = "behavior-trace")]
    eprintln!(
        "         --behavior-trace <path> OPT0-B correctness artifact with streaming event hashes.\n\
                                          Not valid for wall-time measurement."
    );
    #[cfg(feature = "event-horizon-profiler")]
    eprintln!(
        "         --event-horizon-profile <path>\n\
                                          OPT2-D running-boundary/decode opportunity profile.\n\
                                          Not valid for wall-time measurement."
    );
    #[cfg(feature = "event-horizon-profiler")]
    eprintln!(
        "         --event-horizon-profile-after-uart <text>\n\
                                          Defer OPT2-D until this UART marker is observed.\n\
                                          Requires --event-horizon-profile."
    );
}

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut bin: Option<PathBuf> = None;
    let mut bootrom: Option<PathBuf> = None;
    let mut cycles: Option<u64> = None;
    let mut stop_pc: Option<u32> = None;
    let mut json: Option<PathBuf> = None;
    let mut uart: Option<PathBuf> = None;
    let mut flash_image_out: Option<PathBuf> = None;
    let mut expected_backend_commit: Option<String> = None;
    let mut board = Board::None;
    // Variant B is the Canonical BSP default; variant A is what the
    // official sample uses. Selected explicitly so a report always
    // records which transport produced it.
    //
    // The choice is not cosmetic. Variant B attaches a pin-watching
    // device, which switches the serial loop to per-cycle GPIO
    // observation -- correct for B, but a large slowdown for firmware
    // that never drives the display from PIO. Running the official
    // sample without naming variant A costs roughly a third of the
    // reachable cycles.
    let mut lcd_variant = LcdVariant::B;
    let mut fb_png: Option<PathBuf> = None;
    let mut quantum: Option<u32> = None;
    let mut psram = false;
    let mut psram_verify_range: Option<(u32, u32)> = None;
    let mut keyboard = false;
    let mut sd = false;
    let mut sd_image: Option<PathBuf> = None;
    let mut sd_image_out: Option<PathBuf> = None;
    let mut sd_format = SdFormat::default();
    let mut sd_format_explicit = false;
    let mut keys: Option<String> = None;
    let mut scenario: Option<PathBuf> = None;
    let mut snapshot_dir: Option<PathBuf> = None;
    let mut machine_api = false;
    let mut run_id: Option<String> = None;
    let mut progress_interval: Option<u64> = None;
    let mut expected_stop: Option<StopReason> = None;
    let mut expected_uart: Vec<String> = Vec::new();
    let mut expected_audio_sink_count: Option<u64> = None;
    let mut expected_audio_sink_sha256: Option<String> = None;
    let mut audio_analysis: Option<PathBuf> = None;
    let mut audio_wav: Option<PathBuf> = None;
    #[cfg(feature = "idle-profiler")]
    let mut idle_profile: Option<PathBuf> = None;
    #[cfg(feature = "behavior-trace")]
    let mut behavior_trace: Option<PathBuf> = None;
    #[cfg(feature = "event-horizon-profiler")]
    let mut event_horizon_profile: Option<PathBuf> = None;
    #[cfg(feature = "event-horizon-profiler")]
    let mut event_horizon_profile_after_uart: Option<String> = None;

    let mut i = 0;
    while i < argv.len() {
        let flag = argv[i].as_str();
        let mut value = |name: &str| -> Result<String, String> {
            i += 1;
            argv.get(i)
                .cloned()
                .ok_or_else(|| format!("{name} requires a value"))
        };
        match flag {
            "--bin" => bin = Some(PathBuf::from(value("--bin")?)),
            "--bootrom" => bootrom = Some(PathBuf::from(value("--bootrom")?)),
            "--cycles" => {
                let raw = value("--cycles")?;
                let cleaned = raw.replace('_', "");
                cycles = Some(
                    cleaned
                        .parse::<u64>()
                        .map_err(|e| format!("invalid --cycles '{raw}': {e}"))?,
                );
            }
            "--stop-pc" => {
                let raw = value("--stop-pc")?;
                let digits = raw.trim_start_matches("0x").trim_start_matches("0X");
                stop_pc = Some(
                    u32::from_str_radix(digits, 16)
                        .map_err(|e| format!("invalid --stop-pc '{raw}' (expected hex): {e}"))?,
                );
            }
            "--json" => json = Some(PathBuf::from(value("--json")?)),
            "--uart" => uart = Some(PathBuf::from(value("--uart")?)),
            "--flash-image-out" => {
                if flash_image_out.is_some() {
                    return Err("--flash-image-out may be specified only once".to_string());
                }
                flash_image_out = Some(PathBuf::from(value("--flash-image-out")?));
            }
            "--backend-commit" => expected_backend_commit = Some(value("--backend-commit")?),
            "--board" => board = Board::parse(&value("--board")?)?,
            "--lcd-variant" => lcd_variant = LcdVariant::parse(&value("--lcd-variant")?)?,
            "--fb-png" => fb_png = Some(PathBuf::from(value("--fb-png")?)),
            "--quantum" => {
                let raw = value("--quantum")?;
                let n = raw
                    .parse::<u32>()
                    .map_err(|e| format!("invalid --quantum '{raw}': {e}"))?;
                if n == 0 {
                    return Err("--quantum must be >= 1".to_string());
                }
                quantum = Some(n);
            }
            "--psram" => psram = true,
            "--keyboard" => keyboard = true,
            "--sd" => sd = true,
            "--sd-image" => sd_image = Some(PathBuf::from(value("--sd-image")?)),
            "--sd-image-out" => sd_image_out = Some(PathBuf::from(value("--sd-image-out")?)),
            "--sd-format" => {
                let raw = value("--sd-format")?;
                sd_format = raw.parse::<SdFormat>()?;
                sd_format_explicit = true;
            }
            "--keys" => keys = Some(value("--keys")?),
            "--scenario" => scenario = Some(PathBuf::from(value("--scenario")?)),
            "--snapshot-dir" => snapshot_dir = Some(PathBuf::from(value("--snapshot-dir")?)),
            "--machine-api" => machine_api = true,
            "--run-id" => {
                if run_id.is_some() {
                    return Err("--run-id may be specified only once".to_string());
                }
                let id = value("--run-id")?;
                validate_run_id(&id)?;
                run_id = Some(id);
            }
            "--progress-interval" => {
                if progress_interval.is_some() {
                    return Err("--progress-interval may be specified only once".to_string());
                }
                let raw = value("--progress-interval")?;
                let interval = raw.parse::<u64>().map_err(|error| {
                    format!(
                        "invalid --progress-interval '{raw}' (expected integer seconds): {error}"
                    )
                })?;
                validate_progress_interval(interval)?;
                progress_interval = Some(interval);
            }
            "--expect-stop" => {
                if expected_stop.is_some() {
                    return Err("--expect-stop may be specified only once".to_string());
                }
                expected_stop = Some(StopReason::parse_acceptable(&value("--expect-stop")?)?);
            }
            "--expect-uart" => {
                let marker = value("--expect-uart")?;
                if marker.is_empty() {
                    return Err("--expect-uart marker must not be empty".to_string());
                }
                expected_uart.push(marker);
            }
            "--expect-audio-sink-count" => {
                if expected_audio_sink_count.is_some() {
                    return Err("--expect-audio-sink-count may be specified only once".to_string());
                }
                let raw = value("--expect-audio-sink-count")?;
                let count = raw
                    .parse::<u64>()
                    .map_err(|e| format!("invalid --expect-audio-sink-count '{raw}': {e}"))?;
                if count == 0 {
                    return Err("--expect-audio-sink-count must be > 0".to_string());
                }
                expected_audio_sink_count = Some(count);
            }
            "--expect-audio-sink-sha256" => {
                if expected_audio_sink_sha256.is_some() {
                    return Err("--expect-audio-sink-sha256 may be specified only once".to_string());
                }
                let digest = value("--expect-audio-sink-sha256")?.to_ascii_lowercase();
                if digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
                    return Err("--expect-audio-sink-sha256 must be 64 hex characters".to_string());
                }
                expected_audio_sink_sha256 = Some(digest);
            }
            "--audio-analysis" => {
                if audio_analysis.is_some() {
                    return Err("--audio-analysis may be specified only once".to_string());
                }
                audio_analysis = Some(PathBuf::from(value("--audio-analysis")?));
            }
            "--audio-wav" => {
                if audio_wav.is_some() {
                    return Err("--audio-wav may be specified only once".to_string());
                }
                audio_wav = Some(PathBuf::from(value("--audio-wav")?));
            }
            #[cfg(feature = "idle-profiler")]
            "--idle-profile" => idle_profile = Some(PathBuf::from(value("--idle-profile")?)),
            #[cfg(feature = "behavior-trace")]
            "--behavior-trace" => behavior_trace = Some(PathBuf::from(value("--behavior-trace")?)),
            #[cfg(feature = "event-horizon-profiler")]
            "--event-horizon-profile" => {
                event_horizon_profile = Some(PathBuf::from(value("--event-horizon-profile")?))
            }
            #[cfg(feature = "event-horizon-profiler")]
            "--event-horizon-profile-after-uart" => {
                let marker = value("--event-horizon-profile-after-uart")?;
                if marker.is_empty() {
                    return Err(
                        "--event-horizon-profile-after-uart marker must not be empty".to_string(),
                    );
                }
                event_horizon_profile_after_uart = Some(marker);
            }
            "--psram-verify-range" => {
                let raw = value("--psram-verify-range")?;
                psram_verify_range = Some(parse_range(&raw)?);
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
        i += 1;
    }

    if fb_png.is_some() && board != Board::PicoCalc {
        return Err("--fb-png requires --board picocalc".to_string());
    }
    if psram_verify_range.is_some() && !psram {
        return Err("--psram-verify-range requires --psram".to_string());
    }
    validate_sd_selection(
        sd,
        sd_image.as_deref(),
        sd_image_out.as_deref(),
        sd_format_explicit,
    )?;
    // Queueing keys implies the controller they arrive through.
    if keys.is_some() {
        keyboard = true;
    }
    if snapshot_dir.is_some() && scenario.is_none() && !machine_api {
        return Err("--snapshot-dir requires --scenario or --machine-api".to_string());
    }
    if expected_stop == Some(StopReason::PcMatch) && stop_pc.is_none() {
        return Err("--expect-stop pc_match requires --stop-pc".to_string());
    }
    if expected_stop == Some(StopReason::ScenarioDone) && scenario.is_none() {
        return Err("--expect-stop scenario_done requires --scenario".to_string());
    }
    if scenario.is_some() && stop_pc.is_some() {
        return Err("--scenario and --stop-pc define competing successful stops".to_string());
    }
    if scenario.is_some()
        && expected_stop.is_some()
        && expected_stop != Some(StopReason::ScenarioDone)
    {
        return Err("--scenario only permits --expect-stop scenario_done".to_string());
    }
    if stop_pc.is_some() && expected_stop.is_some() && expected_stop != Some(StopReason::PcMatch) {
        return Err("--stop-pc only permits --expect-stop pc_match".to_string());
    }
    if expected_audio_sink_count.is_some() != expected_audio_sink_sha256.is_some() {
        return Err(
            "--expect-audio-sink-count and --expect-audio-sink-sha256 must be used together"
                .to_string(),
        );
    }
    match (&run_id, progress_interval) {
        (Some(_), None) => return Err("--run-id requires --progress-interval".to_string()),
        (None, Some(_)) => {
            return Err("--progress-interval requires --run-id".to_string());
        }
        _ => {}
    }
    if machine_api {
        let conflicts = [
            (scenario.is_some(), "--scenario"),
            (stop_pc.is_some(), "--stop-pc"),
            (json.is_some(), "--json"),
            (uart.is_some(), "--uart"),
            (flash_image_out.is_some(), "--flash-image-out"),
            (fb_png.is_some(), "--fb-png"),
            (expected_stop.is_some(), "--expect-stop"),
            (!expected_uart.is_empty(), "--expect-uart"),
            (
                expected_audio_sink_count.is_some(),
                "audio sink expectations",
            ),
            (audio_analysis.is_some(), "--audio-analysis"),
            (audio_wav.is_some(), "--audio-wav"),
            (keys.is_some(), "--keys"),
            (sd_image_out.is_some(), "--sd-image-out"),
            (run_id.is_some(), "--run-id"),
            (progress_interval.is_some(), "--progress-interval"),
        ];
        if let Some((_, name)) = conflicts.into_iter().find(|(present, _)| *present) {
            return Err(format!("--machine-api cannot be combined with {name}"));
        }
    }
    #[cfg(feature = "idle-profiler")]
    if machine_api && idle_profile.is_some() {
        return Err("--machine-api cannot be combined with --idle-profile".to_string());
    }
    #[cfg(feature = "behavior-trace")]
    if machine_api && behavior_trace.is_some() {
        return Err("--machine-api cannot be combined with --behavior-trace".to_string());
    }
    #[cfg(feature = "event-horizon-profiler")]
    if machine_api && event_horizon_profile.is_some() {
        return Err("--machine-api cannot be combined with --event-horizon-profile".to_string());
    }
    #[cfg(feature = "event-horizon-profiler")]
    if machine_api && event_horizon_profile_after_uart.is_some() {
        return Err(
            "--machine-api cannot be combined with --event-horizon-profile-after-uart".to_string(),
        );
    }
    #[cfg(all(feature = "idle-profiler", feature = "behavior-trace"))]
    if idle_profile.is_some() && behavior_trace.is_some() {
        return Err(
            "--idle-profile and --behavior-trace are separate diagnostic modes".to_string(),
        );
    }
    #[cfg(feature = "event-horizon-profiler")]
    if event_horizon_profile.is_some() && (idle_profile.is_some() || behavior_trace.is_some()) {
        return Err(
            "--event-horizon-profile is a separate diagnostic mode from --idle-profile/--behavior-trace"
                .to_string(),
        );
    }
    #[cfg(feature = "event-horizon-profiler")]
    if event_horizon_profile_after_uart.is_some() && event_horizon_profile.is_none() {
        return Err(
            "--event-horizon-profile-after-uart requires --event-horizon-profile".to_string(),
        );
    }

    Ok(Args {
        bin: bin.ok_or_else(|| "missing required --bin <path>".to_string())?,
        bootrom: bootrom.unwrap_or_else(|| PathBuf::from(DEFAULT_BOOTROM_PATH)),
        cycles: cycles.unwrap_or(DEFAULT_CYCLE_LIMIT),
        stop_pc,
        json,
        uart,
        flash_image_out,
        expected_backend_commit,
        board,
        lcd_variant,
        fb_png,
        quantum,
        psram,
        psram_verify_range,
        keyboard,
        sd,
        sd_image,
        sd_image_out,
        sd_format,
        keys,
        scenario,
        snapshot_dir: snapshot_dir.unwrap_or_else(|| PathBuf::from(".")),
        machine_api,
        run_id,
        progress_interval,
        expected_stop,
        expected_uart,
        expected_audio_sink_count,
        expected_audio_sink_sha256,
        audio_analysis,
        audio_wav,
        #[cfg(feature = "idle-profiler")]
        idle_profile,
        #[cfg(feature = "behavior-trace")]
        behavior_trace,
        #[cfg(feature = "event-horizon-profiler")]
        event_horizon_profile,
        #[cfg(feature = "event-horizon-profiler")]
        event_horizon_profile_after_uart,
    })
}

/// Why the run stopped. `as_str` values are part of the report schema.
#[derive(Clone, Copy, PartialEq, Eq)]
enum StopReason {
    CycleLimit,
    PcMatch,
    Exception,
    Error,
    /// Every scenario step finished. A normal stop, and the usual one
    /// under `--scenario`: there is nothing left to observe, so burning
    /// the rest of the cycle budget would only cost wall time.
    ScenarioDone,
}

impl StopReason {
    fn as_str(self) -> &'static str {
        match self {
            StopReason::ScenarioDone => "scenario_done",
            StopReason::CycleLimit => "cycle_limit",
            StopReason::PcMatch => "pc_match",
            StopReason::Exception => "exception",
            StopReason::Error => "error",
        }
    }

    fn parse_acceptable(raw: &str) -> Result<Self, String> {
        match raw {
            "cycle_limit" => Ok(StopReason::CycleLimit),
            "pc_match" => Ok(StopReason::PcMatch),
            "scenario_done" => Ok(StopReason::ScenarioDone),
            other => Err(format!(
                "invalid --expect-stop '{other}' (expected cycle_limit|pc_match|scenario_done)"
            )),
        }
    }
}

/// Result of the run loop — everything the report needs that isn't
/// already known from the arguments.
struct RunOutcome {
    stop_reason: StopReason,
    cycles: u64,
    /// Virtual time at the stop, from [`VirtualClock`]. Firmware time,
    /// not host time — nothing here reads a wall clock.
    elapsed_ns: u64,
    pc: u32,
    exception: Option<&'static str>,
    error: Option<String>,
    uart_bytes: Vec<u8>,
}

/// Process result. `CannotJudge` is distinct from a negative firmware
/// verdict: it means no acceptance contract was supplied, or the harness
/// itself could not produce a trustworthy judgement.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Verdict {
    Pass,
    Fail,
    CannotJudge,
}

impl Verdict {
    fn as_str(self) -> &'static str {
        match self {
            Verdict::Pass => "pass",
            Verdict::Fail => "fail",
            Verdict::CannotJudge => "cannot_judge",
        }
    }

    fn exit_code(self) -> u8 {
        match self {
            Verdict::Pass => 0,
            Verdict::Fail => 1,
            Verdict::CannotJudge => 2,
        }
    }
}

struct VerdictReport {
    status: Verdict,
    reasons: Vec<&'static str>,
    expected_stop: Option<StopReason>,
    required_uart_markers: Vec<String>,
    missing_uart_markers: Vec<String>,
}

impl VerdictReport {
    fn to_json(&self) -> String {
        let strings = |items: &[String]| {
            items
                .iter()
                .map(|item| json_string(item))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let reasons = self
            .reasons
            .iter()
            .map(|reason| json_string(reason))
            .collect::<Vec<_>>()
            .join(", ");
        format!(
            "  \"verdict\": {{\"status\": {}, \"reasons\": [{}], \
             \"expected_stop_reason\": {}, \"required_uart_markers\": [{}], \
             \"missing_uart_markers\": [{}]}},\n",
            json_string(self.status.as_str()),
            reasons,
            self.expected_stop
                .map(|reason| json_string(reason.as_str()))
                .unwrap_or_else(|| "null".to_string()),
            strings(&self.required_uart_markers),
            strings(&self.missing_uart_markers),
        )
    }
}

#[allow(clippy::too_many_arguments)]
fn judge_run(
    outcome: &RunOutcome,
    unsupported_count: usize,
    unsupported_truncated: bool,
    key_events_dropped: u64,
    keyboard_protocol_errors: u64,
    scenario_passed: Option<bool>,
    scenario_fault: bool,
    expected_stop: Option<StopReason>,
    expected_uart: &[String],
) -> VerdictReport {
    let mut reasons = Vec::new();
    if outcome.exception.is_some() {
        reasons.push("exception");
    }
    if outcome.error.is_some() {
        reasons.push("emulator_error");
    }
    if unsupported_count > 0 {
        reasons.push("unsupported_mmio");
    }
    if unsupported_truncated {
        reasons.push("unsupported_mmio_log_truncated");
    }
    if key_events_dropped > 0 {
        reasons.push("keyboard_events_dropped");
    }
    if keyboard_protocol_errors > 0 {
        reasons.push("keyboard_protocol_error");
    }
    if scenario_passed == Some(false) && !scenario_fault {
        reasons.push("scenario_failed");
    }
    if scenario_fault {
        reasons.push("scenario_unrunnable");
    }
    if !scenario_fault && expected_stop.is_some_and(|expected| outcome.stop_reason != expected) {
        reasons.push("stop_reason_mismatch");
    }

    let missing_uart_markers = expected_uart
        .iter()
        .filter(|marker| {
            let bytes = marker.as_bytes();
            bytes.is_empty()
                || !outcome
                    .uart_bytes
                    .windows(bytes.len())
                    .any(|window| window == bytes)
        })
        .cloned()
        .collect::<Vec<_>>();
    if !scenario_fault && !missing_uart_markers.is_empty() {
        reasons.push("missing_uart_markers");
    }

    let has_judged_failure = reasons
        .iter()
        .any(|reason| *reason != "scenario_unrunnable");
    let status = if has_judged_failure {
        Verdict::Fail
    } else if scenario_fault {
        Verdict::CannotJudge
    } else if expected_stop.is_none() && scenario_passed.is_none() {
        reasons.push(if expected_uart.is_empty() {
            "no_acceptance_criteria"
        } else {
            "no_accepted_stop_reason"
        });
        Verdict::CannotJudge
    } else {
        Verdict::Pass
    };
    VerdictReport {
        status,
        reasons,
        expected_stop,
        required_uart_markers: expected_uart.to_vec(),
        missing_uart_markers,
    }
}

fn apply_audio_sink_expectation(verdict: &mut VerdictReport, audio_sink: Option<&AudioSinkReport>) {
    if audio_sink.is_some_and(|report| report.expectation_failed()) {
        verdict.status = Verdict::Fail;
        verdict.reasons.push("audio_sink_mismatch");
    }
}

/// ARMv6-M IPSR exception numbers that mean "the firmware has fallen
/// over". Ordinary IRQs (>= 16), SVCall, PendSV and SysTick are normal
/// operation and must not stop the run.
fn fatal_exception_name(core: usize, ipsr: u32) -> Option<&'static str> {
    match (core, ipsr) {
        (0, 2) => Some("NMI"),
        (0, 3) => Some("HardFault"),
        (1, 2) => Some("core1 NMI"),
        (1, 3) => Some("core1 HardFault"),
        _ => None,
    }
}

/// Basename of `path` as a lossy string. The report must not carry
/// absolute paths (plan §3.3).
fn basename(path: &Path) -> String {
    path.file_name()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.to_string_lossy().into_owned())
}

fn load_image(path: &Path, what: &str) -> Result<Vec<u8>, String> {
    let bytes =
        std::fs::read(path).map_err(|e| format!("reading {what} {}: {e}", path.display()))?;
    if bytes.len() >= 4 && bytes[0..4] == [0x7F, b'E', b'L', b'F'] {
        return Err(format!(
            "{} is an ELF file — convert first: `arm-none-eabi-objcopy -O binary in.elf out.bin`",
            path.display()
        ));
    }
    Ok(bytes)
}

/// How execution was started. Reported so a run can never be mistaken
/// for a direct boot that did not happen.
///
/// `DirectBootFromFlash` is the plan §3.1 shortcut and the only path
/// used for pico-sdk images. `BootromResetVector` is the fallback for
/// hand-assembled images (e.g. `roms/rp2040/blinky.bin`, which puts
/// code at flash+0 and pairs with the synthetic `roms/rp2040/
/// bootrom.bin` whose reset vector points straight at it) — there is no
/// SDK vector table at flash+0x100 to seed from. It is *not* the real
/// bootrom's USB-MSC path: nothing in the bootrom image executes; the
/// reset vector is taken from ROM word 1 by `Emulator::reset`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum BootMode {
    DirectBootFromFlash,
    BootromResetVector,
}

impl BootMode {
    fn as_str(self) -> &'static str {
        match self {
            BootMode::DirectBootFromFlash => "direct_boot_from_flash",
            BootMode::BootromResetVector => "bootrom_reset_vector",
        }
    }
}

/// Build the emulator and perform the direct-boot handoff.
#[allow(clippy::type_complexity)]
fn boot(
    firmware: Vec<u8>,
    bootrom: &[u8],
    step_quantum: u32,
    board: Board,
    lcd_variant: LcdVariant,
    psram: bool,
    keyboard: Option<Arc<Mutex<Keyboard>>>,
    sd: Option<Arc<Mutex<SdCard>>>,
) -> Result<(Emulator, BootMode, Option<Arc<Mutex<St7365p>>>), String> {
    let mut builder = EmulatorBuilder::new(Config {
        sys_clk_hz: DEFAULT_SYS_CLK_HZ,
    })
    .step_quantum(step_quantum)
    .flash(firmware);

    // PSRAM attaches independently of `--board` — PicoCalc's PSRAM is
    // wired to pio1 regardless of whether the LCD model is present.
    if psram {
        builder = builder.psram(pins::psram_picocalc());
    }

    let mut emu = builder.build().expect("Serial build is infallible");

    // Board models go on before reset: `SpiRegs::reset` deliberately
    // keeps an attached device (a soldered part survives an MCU reset),
    // so either order works — attaching first just makes the panel
    // observe the firmware's very first pin move.
    // One panel, two possible transports. Variant A (the official
    // sample) drives it from SPI1; variant B (the Canonical BSP default)
    // drives it from a PIO0 program, where nothing passes through a
    // controller FIFO and the traffic is only visible on the pads. The
    // two wires stay separate — the plan forbids folding them into one
    // transfer path — but they share the panel model, because the
    // display and its command set are the same part either way.
    let lcd = match board {
        Board::None => None,
        Board::PicoCalc => {
            let lcd = Arc::new(Mutex::new(St7365p::new()));
            match lcd_variant {
                LcdVariant::A => {
                    emu.bus
                        .attach_spi_device(
                            pins::LCD_SPI_INSTANCE,
                            Box::new(St7365pWire::new(lcd.clone())),
                        )
                        .map_err(|i| format!("no SPI instance {i} on RP2040"))?;
                    // Variant A normally writes through SPI1, but its RAMRD
                    // diagnostic temporarily deinitialises SPI1 and drives
                    // the same pads from SIO. The SPI wire receives FIFO
                    // frames; this pin wire receives only the SIO pad edges
                    // and returns panel MISO through GPIO_IN. `update_gpio`
                    // does not synthesize SPI-peripheral SCK/MOSI edges, so
                    // attaching both paths cannot double-count normal writes.
                    emu.bus
                        .attach_pin_device(Box::new(LcdPioWire::new(lcd.clone())));
                }
                LcdVariant::B => {
                    emu.bus
                        .attach_pin_device(Box::new(LcdPioWire::new(lcd.clone())));
                }
            }
            Some(lcd)
        }
    };

    // The keyboard/power controller hangs off I2C1 regardless of the
    // display model, same as the real mainboard.
    if let Some(kbd) = keyboard {
        emu.bus
            .attach_i2c_device(
                pins::KEYBOARD_I2C_INSTANCE,
                Box::new(KeyboardWire::new(kbd)),
            )
            .map_err(|i| format!("no I2C instance {i} on RP2040"))?;
    }

    // The card sits on SPI0. Card detect is an input to the chip, so it
    // is forced low here rather than driven by the device: the slot
    // reports "occupied" for as long as a card is attached.
    if let Some(card) = sd {
        emu.bus
            .attach_spi_device(pins::SD_SPI_INSTANCE, Box::new(SdCardWire::new(card)))
            .map_err(|i| format!("no SPI instance {i} on RP2040"))?;
        let detect = 1u32 << pins::SD_PIN_DETECT;
        emu.bus.external_gpio_in_mask |= detect;
        emu.bus.external_gpio_in_override &= !detect;
    }

    // Loaded, never executed — see the module docs.
    emu.load_bootrom(bootrom);
    emu.reset();

    // Sanity-check the vector table before seeding from it: a garbage
    // SP/PC pair means the image is not an SDK flash image and the run
    // would produce meaningless output.
    let sp = emu.bus.memory.xip_read32(SDK_VTOR_FLASH_OFFSET);
    let pc = emu.bus.memory.xip_read32(SDK_VTOR_FLASH_OFFSET + 4);
    let sp_in_sram = (0x2000_0000..=0x2004_2000).contains(&sp);
    let pc_in_flash = (0x1000_0000..0x1020_0000).contains(&(pc & !1));
    if !(sp_in_sram && pc_in_flash) {
        // Hand-assembled image with no SDK vector table. Seeding SP/PC
        // from flash+0x100 would install garbage, so stay on whatever
        // `reset()` already pulled from ROM words 0/1. Reported as
        // `bootrom_reset_vector` so the distinction is never implicit.
        eprintln!(
            "picocalc-run: flash+{SDK_VTOR_FLASH_OFFSET:#x} is not an SDK vector table \
             (SP={sp:#010x}, PC={pc:#010x}) — booting from the bootrom reset vector instead"
        );
        return Ok((emu, BootMode::BootromResetVector, lcd));
    }
    emu.direct_boot_from_flash(SDK_VTOR_FLASH_OFFSET);
    Ok((emu, BootMode::DirectBootFromFlash, lcd))
}

/// Human-readable park state of one core, for the stalled-clock report.
fn park_state(emu: &Emulator, core: usize) -> &'static str {
    match (emu.cores[core].is_halted(), emu.bus.wfe_waiting[core]) {
        (true, true) => "halted+wfe",
        (true, false) => "halted",
        (false, true) => "wfe",
        (false, false) => "running",
    }
}

/// Emulated cycles converted to virtual nanoseconds.
///
/// The system clock is not a constant: firmware boots on ROSC and moves
/// to a PLL, so a fixed divisor would put every timestamp taken before
/// the switch off by a factor of twenty. Instead the conversion is
/// re-based whenever `clk_sys` changes — time already elapsed keeps the
/// rate it was measured at, and only the new stretch uses the new rate.
struct VirtualClock {
    epoch_cycles: u64,
    epoch_ns: u64,
    hz: u64,
}

impl VirtualClock {
    fn new(hz: u32) -> Self {
        Self {
            epoch_cycles: 0,
            epoch_ns: 0,
            hz: u64::from(hz).max(1),
        }
    }

    fn ns_at(&self, cycles: u64) -> u64 {
        let elapsed = u128::from(cycles.saturating_sub(self.epoch_cycles));
        self.epoch_ns + (elapsed * 1_000_000_000 / u128::from(self.hz)) as u64
    }

    /// The cycle count at which the clock will read `ns`, saturating at
    /// `u64::MAX` so a far-future deadline simply never arrives.
    fn cycles_at(&self, ns: u64) -> u64 {
        let ahead = u128::from(ns.saturating_sub(self.epoch_ns));
        let cycles = ahead * u128::from(self.hz) / 1_000_000_000;
        self.epoch_cycles
            .saturating_add(u64::try_from(cycles).unwrap_or(u64::MAX))
    }

    /// Adopt a new rate from `cycles` onwards. No-op if unchanged.
    fn rebase(&mut self, cycles: u64, hz: u32) -> bool {
        let hz = u64::from(hz).max(1);
        if hz == self.hz {
            return false;
        }
        self.epoch_ns = self.ns_at(cycles);
        self.epoch_cycles = cycles;
        self.hz = hz;
        true
    }
}

/// Board models the scenario engine can reach into mid-run.
#[derive(Default)]
struct BoardHandles {
    lcd: Option<Arc<Mutex<St7365p>>>,
    keyboard: Option<Arc<Mutex<Keyboard>>>,
    sd: Option<Arc<Mutex<SdCard>>>,
}

/// One persistent, deterministic headless machine session.
///
/// This is the shared execution boundary for the batch scenario runner and
/// NEXT-4's JSONL adapter.  Keeping UART accumulation and the virtual clock
/// here prevents either client from inventing subtly different stepping,
/// clock-rebase, or observation semantics.
struct MachineSession {
    emu: Emulator,
    vclock: VirtualClock,
    uart_bytes: Vec<u8>,
    dispatches: u64,
    board: BoardHandles,
    sticky_stop: Option<SessionStop>,
    #[cfg(feature = "event-horizon-profiler")]
    event_profile_after_uart: Option<String>,
    #[cfg(feature = "event-horizon-profiler")]
    event_profile_start_cycle: Option<u64>,
}

#[derive(Clone)]
enum SessionStop {
    Exception(&'static str),
    Error(String),
}

impl MachineSession {
    #[inline]
    fn new(emu: Emulator, board: BoardHandles) -> Self {
        let vclock = VirtualClock::new(emu.bus.clock_tree.sys_clk_hz);
        Self {
            emu,
            vclock,
            uart_bytes: Vec::new(),
            dispatches: 0,
            board,
            sticky_stop: None,
            #[cfg(feature = "event-horizon-profiler")]
            event_profile_after_uart: None,
            #[cfg(feature = "event-horizon-profiler")]
            event_profile_start_cycle: None,
        }
    }

    #[inline(always)]
    fn cycles(&self) -> u64 {
        self.emu.clock.cycles
    }

    #[inline(always)]
    fn elapsed_ns(&self) -> u64 {
        self.vclock.ns_at(self.cycles())
    }

    fn fatal_exception(&self) -> Option<&'static str> {
        self.emu
            .cores
            .iter()
            .enumerate()
            .find_map(|(core, state)| fatal_exception_name(core, state.regs.xpsr & 0x1FF))
    }

    fn refresh_sticky_stop(&mut self) {
        if self.sticky_stop.is_none()
            && let Some(exception) = self.fatal_exception()
        {
            self.sticky_stop = Some(SessionStop::Exception(exception));
        }
    }

    fn stopped(&self) -> Option<&SessionStop> {
        self.sticky_stop.as_ref()
    }

    #[inline(always)]
    fn drain_uart(&mut self) {
        self.uart_bytes
            .extend_from_slice(&self.emu.drain_uart0_tx_log());
        #[cfg(feature = "event-horizon-profiler")]
        self.maybe_start_event_profile();
    }

    #[cfg(feature = "event-horizon-profiler")]
    fn arm_event_profile_after_uart(&mut self, marker: String) {
        self.event_profile_after_uart = Some(marker);
    }

    #[cfg(feature = "event-horizon-profiler")]
    fn maybe_start_event_profile(&mut self) {
        let Some(marker) = self.event_profile_after_uart.as_deref() else {
            return;
        };
        if self
            .uart_bytes
            .windows(marker.len())
            .any(|window| window == marker.as_bytes())
        {
            self.emu
                .enable_running_event_profiler()
                .expect("deferred event profiler is enabled on Serial emulator");
            self.event_profile_start_cycle = Some(self.cycles());
            self.event_profile_after_uart = None;
        }
    }

    fn poll_scenario(&mut self, engine: &mut scenario::Engine) {
        self.drain_uart();
        engine.poll(&scenario::Observation {
            now_ns: self.elapsed_ns(),
            cycles: self.cycles(),
            lcd: self.board.lcd.as_deref(),
            keyboard: self.board.keyboard.as_deref(),
            uart: &self.uart_bytes,
        });
    }

    /// Execute one scheduler dispatch without crossing a proven idle
    /// external boundary. Returns `(cycles_consumed, clock_rate_changed)`.
    #[inline(always)]
    fn advance_once(&mut self, external_event_cycle: u64) -> Result<(u64, bool), String> {
        let consumed = self.emu.step_until(external_event_cycle).map_err(|error| {
            let message = error.to_string();
            self.sticky_stop = Some(SessionStop::Error(message.clone()));
            message
        })?;
        self.dispatches = self.dispatches.saturating_add(1);
        if self.dispatches.is_multiple_of(UART_DRAIN_INTERVAL) {
            self.drain_uart();
        }
        let rebased = self
            .vclock
            .rebase(self.emu.clock.cycles, self.emu.bus.clock_tree.sys_clk_hz);
        Ok((consumed, rebased))
    }

    fn mark_clock_stalled(&mut self) {
        let detail = format!(
            "clock stalled: core0 {}, core1 {} — no wake source can fire while the master clock is frozen",
            park_state(&self.emu, 0),
            park_state(&self.emu, 1)
        );
        self.sticky_stop = Some(SessionStop::Error(detail));
    }

    fn finish(
        &mut self,
        stop_reason: StopReason,
        exception: Option<&'static str>,
        error: Option<String>,
    ) -> RunOutcome {
        self.drain_uart();
        RunOutcome {
            stop_reason,
            cycles: self.cycles(),
            elapsed_ns: self.elapsed_ns(),
            pc: self.emu.cores[0].regs.pc(),
            exception,
            error,
            uart_bytes: std::mem::take(&mut self.uart_bytes),
        }
    }
}

fn run_loop(
    machine: &mut MachineSession,
    cycle_limit: u64,
    stop_pc: Option<u32>,
    mut engine: Option<&mut scenario::Engine>,
    mut progress: Option<&mut ProgressReporter>,
) -> RunOutcome {
    // Keep the established batch hot loop byte-for-byte simple. The
    // persistent API uses `MachineSession::advance_once`; the scenario client
    // owns the same session state but retains its historical local dispatch
    // counter so NEXT-4 does not add protocol bookkeeping to conformance runs.
    let mut steps = 0u64;
    // Comparing cycles rather than converting to nanoseconds keeps the
    // per-step check to one integer compare; the division only happens
    // at a poll or a clock change.
    let mut next_poll_cycles = match engine.as_deref() {
        Some(e) => machine.vclock.cycles_at(e.next_poll_ns()),
        None => u64::MAX,
    };

    loop {
        // Pre-step observations. Checking before the first step means a
        // `--stop-pc` equal to the reset vector matches immediately, and
        // that a scenario's first step sees the machine at reset.
        if let Some(e) = engine.as_deref_mut()
            && machine.cycles() >= next_poll_cycles
        {
            // The engine may test the UART stream, so it must see every
            // byte sent so far — not just those the periodic drain has
            // collected.
            machine.poll_scenario(e);
            #[cfg(feature = "behavior-trace")]
            {
                // The scenario file digest identifies the complete input
                // program in the behavior projection. This event records
                // when that program was observed/applied in virtual time.
                let mut payload = Vec::with_capacity(25);
                payload.extend_from_slice(&(e.results().len() as u64).to_be_bytes());
                payload.push(u8::from(e.is_done()));
                payload.extend_from_slice(&e.next_poll_ns().to_be_bytes());
                payload.extend_from_slice(&(machine.uart_bytes.len() as u64).to_be_bytes());
                machine
                    .emu
                    .record_behavior_event(BehaviorEventDomain::ScenarioInput, 1, &payload);
            }
            if e.is_done() {
                return machine.finish(StopReason::ScenarioDone, None, None);
            }
            next_poll_cycles = machine.vclock.cycles_at(e.next_poll_ns());
            // A poll that changed nothing would otherwise re-fire every
            // step until virtual time moved on.
            next_poll_cycles = next_poll_cycles.max(machine.cycles() + 1);
        }

        if let Some(target) = stop_pc
            && machine.emu.cores[0].regs.pc() == target
        {
            return machine.finish(StopReason::PcMatch, None, None);
        }
        for (core, state) in machine.emu.cores.iter().enumerate() {
            if let Some(name) = fatal_exception_name(core, state.regs.xpsr & 0x1FF) {
                return machine.finish(StopReason::Exception, Some(name), None);
            }
        }
        if machine.cycles() >= cycle_limit {
            return machine.finish(StopReason::CycleLimit, None, None);
        }

        let external_event_cycle = next_poll_cycles.min(cycle_limit);
        let consumed = match machine.emu.step_until(external_event_cycle) {
            Ok(consumed) => consumed,
            Err(error) => {
                return machine.finish(StopReason::Error, None, Some(error.to_string()));
            }
        };
        steps += 1;

        if consumed == 0 {
            // Every core is halted or parked on WFE, so the master
            // clock cannot advance and the cycle budget can never be
            // reached — stepping again would spin forever. Report the
            // per-core park state: "halted" (BKPT / core 1 never
            // launched) and "wfe" (waiting for an event that only a
            // peripheral tick can deliver) have very different causes.
            let detail = format!(
                "clock stalled: core0 {}, core1 {} — no wake source can fire \
                 while the master clock is frozen",
                park_state(&machine.emu, 0),
                park_state(&machine.emu, 1)
            );
            return machine.finish(StopReason::Error, None, Some(detail));
        }

        if steps.is_multiple_of(UART_DRAIN_INTERVAL) {
            machine.drain_uart();
        }

        if let Some(reporter) = progress.as_mut() {
            reporter.maybe_emit(machine.cycles(), cycle_limit);
        }

        // Firmware reprograms the clock tree during init; from here on,
        // virtual milliseconds have to mean what the firmware thinks they
        // mean. The pending poll deadline is expressed in nanoseconds, so
        // it moves with the rebase rather than being stranded at the old
        // rate.
        if machine.vclock.rebase(
            machine.emu.clock.cycles,
            machine.emu.bus.clock_tree.sys_clk_hz,
        ) && let Some(e) = engine.as_deref()
        {
            next_poll_cycles = machine
                .vclock
                .cycles_at(e.next_poll_ns())
                .max(machine.cycles() + 1);
        }
    }
}

const MACHINE_MAX_DISPATCHES: u64 = 1_000_000;
const MACHINE_MAX_CYCLES: u64 = 100_000_000_000;
const MACHINE_MAX_REQUEST_BYTES: usize = 1_048_576;
const MACHINE_MAX_EVENT_BYTES: usize = 1_048_576;

#[derive(Default)]
struct MachineApiState {
    subscriptions: BTreeSet<String>,
    next_event_sequence: u64,
    uart_cursor: usize,
    framebuffer_sha: Option<String>,
    stop_seen: bool,
}

fn protocol_error(
    code: machine_protocol::ErrorCode,
    message: impl Into<String>,
) -> machine_protocol::ProtocolError {
    machine_protocol::ProtocolError::new(code, message)
}

fn invalid_request(message: impl Into<String>) -> machine_protocol::ProtocolError {
    protocol_error(machine_protocol::ErrorCode::InvalidRequest, message)
}

fn request_object(
    request: &serde_json::Value,
) -> Result<&serde_json::Map<String, serde_json::Value>, machine_protocol::ProtocolError> {
    request
        .as_object()
        .ok_or_else(|| invalid_request("request root must be an object"))
}

fn required_u64(
    object: &serde_json::Map<String, serde_json::Value>,
    field: &str,
    min: u64,
    max: u64,
) -> Result<u64, machine_protocol::ProtocolError> {
    let value = object
        .get(field)
        .and_then(serde_json::Value::as_u64)
        .ok_or_else(|| {
            invalid_request(format!("field '{field}' must be a non-negative integer"))
        })?;
    if !(min..=max).contains(&value) {
        return Err(invalid_request(format!(
            "field '{field}' must be in {min}..={max}"
        )));
    }
    Ok(value)
}

fn string_array(
    object: &serde_json::Map<String, serde_json::Value>,
    field: &str,
) -> Result<Vec<String>, machine_protocol::ProtocolError> {
    let values = object
        .get(field)
        .and_then(serde_json::Value::as_array)
        .ok_or_else(|| invalid_request(format!("field '{field}' must be an array")))?;
    if values.is_empty() {
        return Err(invalid_request(format!(
            "field '{field}' must not be empty"
        )));
    }
    values
        .iter()
        .enumerate()
        .map(|(index, value)| {
            value.as_str().map(str::to_string).ok_or_else(|| {
                invalid_request(format!("field '{field}[{index}]' must be a string"))
            })
        })
        .collect()
}

fn session_stop_json(stop: Option<&SessionStop>) -> serde_json::Value {
    match stop {
        None => serde_json::Value::Null,
        Some(SessionStop::Exception(exception)) => serde_json::json!({
            "reason": "exception",
            "exception": exception,
        }),
        Some(SessionStop::Error(error)) => serde_json::json!({
            "reason": "error",
            "error": error,
        }),
    }
}

fn framebuffer_json(
    machine: &MachineSession,
) -> Result<serde_json::Value, machine_protocol::ProtocolError> {
    let lcd = machine.board.lcd.as_ref().ok_or_else(|| {
        protocol_error(
            machine_protocol::ErrorCode::UnsupportedObservation,
            "framebuffer observation requires --board picocalc",
        )
    })?;
    let framebuffer = lcd
        .lock()
        .map_err(|_| {
            protocol_error(
                machine_protocol::ErrorCode::ModelError,
                "LCD mutex poisoned",
            )
        })?
        .framebuffer();
    Ok(serde_json::json!({
        "width": framebuffer.width,
        "height": framebuffer.height,
        "rgb565_sha256": framebuffer.rgb565_sha256(),
        "non_black_pixels": framebuffer.non_black_pixels(),
    }))
}

fn observe_domain(
    machine: &mut MachineSession,
    domain: &str,
) -> Result<serde_json::Value, machine_protocol::ProtocolError> {
    match domain {
        "machine" => {
            machine.refresh_sticky_stop();
            Ok(serde_json::json!({
                "cycle": machine.cycles(),
                "virtual_ns": machine.elapsed_ns(),
                "core0": {
                    "pc": machine.emu.cores[0].regs.pc(),
                    "park": park_state(&machine.emu, 0),
                },
                "core1": {
                    "pc": machine.emu.cores[1].regs.pc(),
                    "park": park_state(&machine.emu, 1),
                },
                "stop": session_stop_json(machine.stopped()),
            }))
        }
        "uart" => {
            machine.drain_uart();
            Ok(serde_json::json!({
                "bytes": machine.uart_bytes.len(),
                "sha256": sha256_hex(&machine.uart_bytes),
                "text": String::from_utf8_lossy(&machine.uart_bytes),
            }))
        }
        "framebuffer" => framebuffer_json(machine),
        "keyboard" => {
            let keyboard = machine.board.keyboard.as_ref().ok_or_else(|| {
                protocol_error(
                    machine_protocol::ErrorCode::UnsupportedObservation,
                    "keyboard observation requires --keyboard",
                )
            })?;
            let keyboard = keyboard.lock().map_err(|_| {
                protocol_error(
                    machine_protocol::ErrorCode::ModelError,
                    "keyboard mutex poisoned",
                )
            })?;
            Ok(serde_json::json!({
                "queued": keyboard.queued(),
                "delivered": keyboard.key_events_delivered,
                "dropped": keyboard.key_events_dropped,
                "overwritten": keyboard.key_events_overwritten,
                "caps_lock": keyboard.caps_lock,
                "num_lock": keyboard.num_lock,
            }))
        }
        "sd" => {
            let card = machine.board.sd.as_ref().ok_or_else(|| {
                protocol_error(
                    machine_protocol::ErrorCode::UnsupportedObservation,
                    "SD observation requires --sd",
                )
            })?;
            let card = card.lock().map_err(|_| {
                protocol_error(machine_protocol::ErrorCode::ModelError, "SD mutex poisoned")
            })?;
            Ok(serde_json::json!({
                "format": card.format().as_str(),
                "commands_seen": card.commands_seen,
                "blocks_read": card.blocks_read,
                "blocks_written": card.blocks_written,
                "unknown_commands": card.unknown_commands,
            }))
        }
        "unsupported_mmio" => Ok(serde_json::json!({
            "entries": machine.emu.bus.unsupported_mmio_log(),
            "truncated": machine.emu.bus.unsupported_mmio_log_truncated(),
        })),
        other => Err(protocol_error(
            machine_protocol::ErrorCode::UnsupportedObservation,
            format!("unknown observation domain '{other}'"),
        )),
    }
}

fn run_machine_budget(
    machine: &mut MachineSession,
    max_cycles: u64,
) -> Result<serde_json::Value, machine_protocol::ProtocolError> {
    machine.refresh_sticky_stop();
    if machine.stopped().is_some() {
        return Err(protocol_error(
            machine_protocol::ErrorCode::MachineStopped,
            "machine is already stopped",
        ));
    }
    let start = machine.cycles();
    let target = start
        .checked_add(max_cycles)
        .ok_or_else(|| invalid_request("max_cycles overflows the master-cycle counter"))?;
    let reason = loop {
        machine.refresh_sticky_stop();
        if machine.stopped().is_some() {
            break "stopped";
        }
        if machine.cycles() >= target {
            break "cycle_budget";
        }
        match machine.advance_once(target) {
            Ok((0, _)) => {
                machine.mark_clock_stalled();
                break "stopped";
            }
            Ok(_) => {}
            Err(_) if machine.stopped().is_some() => break "stopped",
            Err(error) => {
                return Err(protocol_error(
                    machine_protocol::ErrorCode::ModelError,
                    error,
                ));
            }
        }
    };
    machine.drain_uart();
    Ok(serde_json::json!({
        "reason": reason,
        "advanced_cycles": machine.cycles().saturating_sub(start),
        "stop": session_stop_json(machine.stopped()),
    }))
}

fn condition_holds(
    machine: &mut MachineSession,
    condition: &serde_json::Value,
) -> Result<bool, machine_protocol::ProtocolError> {
    let object = condition
        .as_object()
        .ok_or_else(|| invalid_request("field 'condition' must be an object"))?;
    let kind = object
        .get("kind")
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| invalid_request("field 'condition.kind' must be a string"))?;
    let allowed: &[&str] = match kind {
        "pc_equals" | "cycle_at_least" => &["kind", "value"],
        "uart_contains" => &["kind", "text"],
        "pixel_equals" => &["kind", "x", "y", "value"],
        "region_hash_equals" => &["kind", "x", "y", "w", "h", "sha256"],
        other => {
            return Err(invalid_request(format!("unknown condition kind '{other}'")));
        }
    };
    let mut unknown = object
        .keys()
        .filter(|field| !allowed.contains(&field.as_str()))
        .cloned()
        .collect::<Vec<_>>();
    unknown.sort();
    if !unknown.is_empty() {
        return Err(invalid_request(format!(
            "condition contains unknown field(s): {}",
            unknown.join(", ")
        )));
    }
    let number = |field: &str| -> Result<u64, machine_protocol::ProtocolError> {
        object
            .get(field)
            .and_then(serde_json::Value::as_u64)
            .ok_or_else(|| {
                invalid_request(format!(
                    "field 'condition.{field}' must be a non-negative integer"
                ))
            })
    };
    match kind {
        "pc_equals" => {
            let pc = u32::try_from(number("value")?)
                .map_err(|_| invalid_request("field 'condition.value' does not fit in u32"))?;
            Ok(machine.emu.cores[0].regs.pc() == pc)
        }
        "cycle_at_least" => Ok(machine.cycles() >= number("value")?),
        "uart_contains" => {
            let text = object
                .get("text")
                .and_then(serde_json::Value::as_str)
                .filter(|text| !text.is_empty())
                .ok_or_else(|| {
                    invalid_request("field 'condition.text' must be a non-empty string")
                })?;
            machine.drain_uart();
            Ok(machine
                .uart_bytes
                .windows(text.len())
                .any(|window| window == text.as_bytes()))
        }
        "pixel_equals" => {
            let x = usize::try_from(number("x")?)
                .map_err(|_| invalid_request("condition.x is too large"))?;
            let y = usize::try_from(number("y")?)
                .map_err(|_| invalid_request("condition.y is too large"))?;
            let value = u16::try_from(number("value")?)
                .map_err(|_| invalid_request("condition.value does not fit in RGB565"))?;
            let lcd = machine.board.lcd.as_ref().ok_or_else(|| {
                protocol_error(
                    machine_protocol::ErrorCode::UnsupportedObservation,
                    "pixel condition requires --board picocalc",
                )
            })?;
            let lcd = lcd.lock().map_err(|_| {
                protocol_error(
                    machine_protocol::ErrorCode::ModelError,
                    "LCD mutex poisoned",
                )
            })?;
            let pixel = lcd
                .gram_pixel(x, y)
                .ok_or_else(|| invalid_request("pixel coordinate is outside the framebuffer"))?;
            Ok(pixel == value)
        }
        "region_hash_equals" => {
            let x = usize::try_from(number("x")?)
                .map_err(|_| invalid_request("condition.x is too large"))?;
            let y = usize::try_from(number("y")?)
                .map_err(|_| invalid_request("condition.y is too large"))?;
            let w = usize::try_from(number("w")?)
                .map_err(|_| invalid_request("condition.w is too large"))?;
            let h = usize::try_from(number("h")?)
                .map_err(|_| invalid_request("condition.h is too large"))?;
            if w == 0 || h == 0 {
                return Err(invalid_request(
                    "condition region dimensions must be non-zero",
                ));
            }
            let expected = object
                .get("sha256")
                .and_then(serde_json::Value::as_str)
                .filter(|value| {
                    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
                })
                .ok_or_else(|| {
                    invalid_request("field 'condition.sha256' must be 64 hexadecimal characters")
                })?;
            let lcd = machine.board.lcd.as_ref().ok_or_else(|| {
                protocol_error(
                    machine_protocol::ErrorCode::UnsupportedObservation,
                    "region condition requires --board picocalc",
                )
            })?;
            let lcd = lcd.lock().map_err(|_| {
                protocol_error(
                    machine_protocol::ErrorCode::ModelError,
                    "LCD mutex poisoned",
                )
            })?;
            let framebuffer = lcd.framebuffer();
            if x.checked_add(w)
                .is_none_or(|right| right > framebuffer.width)
                || y.checked_add(h)
                    .is_none_or(|bottom| bottom > framebuffer.height)
            {
                return Err(invalid_request(
                    "condition region is outside the framebuffer",
                ));
            }
            let mut bytes = Vec::with_capacity(w.saturating_mul(h).saturating_mul(2));
            for row in y..y + h {
                for column in x..x + w {
                    bytes
                        .extend_from_slice(&lcd.gram_pixel(column, row).unwrap_or(0).to_le_bytes());
                }
            }
            Ok(sha256_hex(&bytes).eq_ignore_ascii_case(expected))
        }
        _ => unreachable!("condition kind was validated above"),
    }
}

fn run_machine_until(
    machine: &mut MachineSession,
    condition: &serde_json::Value,
    max_cycles: u64,
    poll_cycles: u64,
) -> Result<serde_json::Value, machine_protocol::ProtocolError> {
    machine.refresh_sticky_stop();
    if machine.stopped().is_some() {
        return Err(protocol_error(
            machine_protocol::ErrorCode::MachineStopped,
            "machine is already stopped",
        ));
    }
    let start = machine.cycles();
    let target = start
        .checked_add(max_cycles)
        .ok_or_else(|| invalid_request("max_cycles overflows the master-cycle counter"))?;
    let mut next_poll = start;
    let reason = loop {
        machine.refresh_sticky_stop();
        if machine.stopped().is_some() {
            break "stopped";
        }
        if machine.cycles() >= next_poll {
            if condition_holds(machine, condition)? {
                break "condition";
            }
            next_poll = machine.cycles().saturating_add(poll_cycles).min(target);
        }
        if machine.cycles() >= target {
            break "cycle_budget";
        }
        let boundary = next_poll.min(target);
        match machine.advance_once(boundary) {
            Ok((0, _)) => {
                machine.mark_clock_stalled();
                break "stopped";
            }
            Ok(_) => {}
            Err(_) if machine.stopped().is_some() => break "stopped",
            Err(error) => {
                return Err(protocol_error(
                    machine_protocol::ErrorCode::ModelError,
                    error,
                ));
            }
        }
    };
    machine.drain_uart();
    Ok(serde_json::json!({
        "reason": reason,
        "condition_met": reason == "condition",
        "advanced_cycles": machine.cycles().saturating_sub(start),
        "stop": session_stop_json(machine.stopped()),
    }))
}

fn inject_input(
    machine: &mut MachineSession,
    request: &serde_json::Value,
) -> Result<serde_json::Value, machine_protocol::ProtocolError> {
    machine.refresh_sticky_stop();
    if machine.stopped().is_some() {
        return Err(protocol_error(
            machine_protocol::ErrorCode::MachineStopped,
            "machine is already stopped",
        ));
    }
    let object = request_object(request)?;
    let has_text = object.contains_key("text");
    let has_events = object.contains_key("events");
    if has_text == has_events {
        return Err(invalid_request(
            "input requires exactly one of 'text' or 'events'",
        ));
    }
    let keyboard = machine.board.keyboard.as_ref().ok_or_else(|| {
        protocol_error(
            machine_protocol::ErrorCode::UnsupportedObservation,
            "input requires --keyboard",
        )
    })?;
    let mut parsed = Vec::new();
    if let Some(text) = object.get("text") {
        let text = text
            .as_str()
            .filter(|text| !text.is_empty())
            .ok_or_else(|| invalid_request("field 'text' must be a non-empty string"))?;
        for character in text.chars() {
            let code = u8::try_from(character as u32).map_err(|_| {
                invalid_request(format!("character {character:?} is not an 8-bit key code"))
            })?;
            parsed.push(KeyEvent::pressed(code));
            parsed.push(KeyEvent::released(code));
        }
    } else {
        let events = object
            .get("events")
            .and_then(serde_json::Value::as_array)
            .filter(|events| !events.is_empty())
            .ok_or_else(|| invalid_request("field 'events' must be a non-empty array"))?;
        for (index, event) in events.iter().enumerate() {
            let event = event.as_object().ok_or_else(|| {
                invalid_request(format!("field 'events[{index}]' must be an object"))
            })?;
            if event
                .keys()
                .any(|field| field != "state" && field != "code")
            {
                return Err(invalid_request(format!(
                    "field 'events[{index}]' contains an unknown field"
                )));
            }
            let state = match event.get("state").and_then(serde_json::Value::as_str) {
                Some("pressed") => KeyState::Pressed,
                Some("held") => KeyState::Held,
                Some("released") => KeyState::Released,
                _ => {
                    return Err(invalid_request(format!(
                        "field 'events[{index}].state' must be pressed, held, or released"
                    )));
                }
            };
            let code = event
                .get("code")
                .and_then(serde_json::Value::as_u64)
                .and_then(|value| u8::try_from(value).ok())
                .ok_or_else(|| {
                    invalid_request(format!("field 'events[{index}].code' must be in 0..=255"))
                })?;
            parsed.push(KeyEvent { state, code });
        }
    }
    let mut keyboard = keyboard.lock().map_err(|_| {
        protocol_error(
            machine_protocol::ErrorCode::ModelError,
            "keyboard mutex poisoned",
        )
    })?;
    let dropped_before = keyboard.key_events_dropped;
    for event in &parsed {
        keyboard.push_event(*event);
    }
    let dropped = keyboard.key_events_dropped.saturating_sub(dropped_before);
    Ok(serde_json::json!({
        "status": if dropped == 0 { "accepted" } else { "dropped" },
        "events": parsed.len(),
        "dropped": dropped,
        "queued": keyboard.queued(),
    }))
}

fn snapshot_machine(
    machine: &MachineSession,
    request: &serde_json::Value,
    snapshot_dir: &Path,
) -> Result<serde_json::Value, machine_protocol::ProtocolError> {
    let object = request_object(request)?;
    let lcd = machine.board.lcd.as_ref().ok_or_else(|| {
        protocol_error(
            machine_protocol::ErrorCode::UnsupportedObservation,
            "snapshot requires --board picocalc",
        )
    })?;
    let framebuffer = lcd
        .lock()
        .map_err(|_| {
            protocol_error(
                machine_protocol::ErrorCode::ModelError,
                "LCD mutex poisoned",
            )
        })?
        .framebuffer();
    let png = object
        .get("png")
        .map(|value| {
            let name = value
                .as_str()
                .filter(|name| !name.is_empty())
                .ok_or_else(|| invalid_request("field 'png' must be a non-empty basename"))?;
            let path = Path::new(name);
            if path.is_absolute()
                || path.components().count() != 1
                || path.file_name().and_then(|value| value.to_str()) != Some(name)
            {
                return Err(invalid_request(
                    "field 'png' must be a basename without path separators or '..'",
                ));
            }
            let output = snapshot_dir.join(path);
            framebuffer.write_png(&output).map_err(|error| {
                protocol_error(
                    machine_protocol::ErrorCode::ModelError,
                    format!("writing snapshot {name}: {error}"),
                )
            })?;
            Ok::<_, machine_protocol::ProtocolError>(name.to_string())
        })
        .transpose()?;
    Ok(serde_json::json!({
        "width": framebuffer.width,
        "height": framebuffer.height,
        "rgb565_sha256": framebuffer.rgb565_sha256(),
        "non_black_pixels": framebuffer.non_black_pixels(),
        "png": png,
    }))
}

fn collect_subscription_events(
    machine: &mut MachineSession,
    state: &mut MachineApiState,
) -> Result<Vec<serde_json::Value>, machine_protocol::ProtocolError> {
    let mut events = Vec::new();
    for topic in state.subscriptions.clone() {
        let data = match topic.as_str() {
            "uart" => {
                machine.drain_uart();
                let bytes = &machine.uart_bytes[state.uart_cursor..];
                if bytes.len() > MACHINE_MAX_EVENT_BYTES {
                    return Err(protocol_error(
                        machine_protocol::ErrorCode::EventOverflow,
                        format!(
                            "UART event contains {} bytes, limit is {MACHINE_MAX_EVENT_BYTES}",
                            bytes.len()
                        ),
                    ));
                }
                if bytes.is_empty() {
                    continue;
                }
                let offset = state.uart_cursor;
                state.uart_cursor = machine.uart_bytes.len();
                serde_json::json!({
                    "offset": offset,
                    "bytes": bytes,
                    "text": String::from_utf8_lossy(bytes),
                })
            }
            "stop" => {
                machine.refresh_sticky_stop();
                if machine.stopped().is_none() || state.stop_seen {
                    continue;
                }
                state.stop_seen = true;
                session_stop_json(machine.stopped())
            }
            "framebuffer" => {
                let current = framebuffer_json(machine)?;
                let sha = current
                    .get("rgb565_sha256")
                    .and_then(serde_json::Value::as_str)
                    .expect("framebuffer result always has sha256")
                    .to_string();
                if state.framebuffer_sha.as_deref() == Some(&sha) {
                    continue;
                }
                state.framebuffer_sha = Some(sha);
                current
            }
            _ => unreachable!("subscribe validates every topic"),
        };
        let sequence = state.next_event_sequence;
        state.next_event_sequence = state.next_event_sequence.saturating_add(1);
        events.push(serde_json::json!({
            "sequence": sequence,
            "topic": topic,
            "cycle": machine.cycles(),
            "data": data,
        }));
    }
    Ok(events)
}

fn dispatch_machine_request(
    machine: &mut MachineSession,
    state: &mut MachineApiState,
    request: &serde_json::Value,
    header: &machine_protocol::RequestHeader,
    snapshot_dir: &Path,
) -> Result<(serde_json::Value, bool), machine_protocol::ProtocolError> {
    let object = request_object(request)?;
    match header.op.as_str() {
        "run" => {
            machine_protocol::reject_unknown_top_level_fields(request, &["max_cycles"])?;
            let max_cycles = required_u64(object, "max_cycles", 1, MACHINE_MAX_CYCLES)?;
            Ok((run_machine_budget(machine, max_cycles)?, true))
        }
        "step" => {
            machine_protocol::reject_unknown_top_level_fields(request, &["count"])?;
            let count = object
                .get("count")
                .map(|_| required_u64(object, "count", 1, MACHINE_MAX_DISPATCHES))
                .transpose()?
                .unwrap_or(1);
            machine.refresh_sticky_stop();
            if machine.stopped().is_some() {
                return Err(protocol_error(
                    machine_protocol::ErrorCode::MachineStopped,
                    "machine is already stopped",
                ));
            }
            let start = machine.cycles();
            let mut completed = 0;
            while completed < count {
                machine.refresh_sticky_stop();
                if machine.stopped().is_some() {
                    break;
                }
                match machine.advance_once(u64::MAX) {
                    Ok((0, _)) => {
                        machine.mark_clock_stalled();
                        break;
                    }
                    Ok(_) => completed += 1,
                    Err(_) if machine.stopped().is_some() => break,
                    Err(error) => {
                        return Err(protocol_error(
                            machine_protocol::ErrorCode::ModelError,
                            error,
                        ));
                    }
                }
            }
            machine.drain_uart();
            Ok((
                serde_json::json!({
                    "requested_dispatches": count,
                    "completed_dispatches": completed,
                    "advanced_cycles": machine.cycles().saturating_sub(start),
                    "stop": session_stop_json(machine.stopped()),
                }),
                true,
            ))
        }
        "run_until" => {
            machine_protocol::reject_unknown_top_level_fields(
                request,
                &["condition", "max_cycles", "poll_cycles"],
            )?;
            let condition = object
                .get("condition")
                .ok_or_else(|| invalid_request("missing required field 'condition'"))?;
            let max_cycles = required_u64(object, "max_cycles", 1, MACHINE_MAX_CYCLES)?;
            let poll_cycles = required_u64(object, "poll_cycles", 1, max_cycles)?;
            Ok((
                run_machine_until(machine, condition, max_cycles, poll_cycles)?,
                true,
            ))
        }
        "input" => {
            machine_protocol::reject_unknown_top_level_fields(request, &["text", "events"])?;
            Ok((inject_input(machine, request)?, true))
        }
        "observe" => {
            machine_protocol::reject_unknown_top_level_fields(request, &["domains"])?;
            let domains = string_array(object, "domains")?;
            let mut result = serde_json::Map::new();
            for domain in domains {
                if result.contains_key(&domain) {
                    return Err(invalid_request(format!(
                        "observation domain '{domain}' is duplicated"
                    )));
                }
                result.insert(domain.clone(), observe_domain(machine, &domain)?);
            }
            Ok((serde_json::Value::Object(result), false))
        }
        "subscribe" => {
            machine_protocol::reject_unknown_top_level_fields(request, &["domains"])?;
            let domains = string_array(object, "domains")?;
            let mut subscriptions = BTreeSet::new();
            for domain in domains {
                if !matches!(domain.as_str(), "uart" | "stop" | "framebuffer") {
                    return Err(protocol_error(
                        machine_protocol::ErrorCode::UnsupportedObservation,
                        format!("unsupported subscription topic '{domain}'"),
                    ));
                }
                if !subscriptions.insert(domain.clone()) {
                    return Err(invalid_request(format!(
                        "subscription topic '{domain}' is duplicated"
                    )));
                }
            }
            machine.drain_uart();
            state.uart_cursor = machine.uart_bytes.len();
            state.stop_seen = machine.stopped().is_some();
            state.framebuffer_sha = if subscriptions.contains("framebuffer") {
                Some(
                    framebuffer_json(machine)?
                        .get("rgb565_sha256")
                        .and_then(serde_json::Value::as_str)
                        .expect("framebuffer result always has sha256")
                        .to_string(),
                )
            } else {
                None
            };
            state.subscriptions = subscriptions;
            Ok((
                serde_json::json!({
                    "domains": state.subscriptions,
                    "next_event_sequence": state.next_event_sequence,
                }),
                false,
            ))
        }
        "snapshot" => {
            machine_protocol::reject_unknown_top_level_fields(request, &["png"])?;
            Ok((snapshot_machine(machine, request, snapshot_dir)?, false))
        }
        other => Err(protocol_error(
            machine_protocol::ErrorCode::UnsupportedOperation,
            format!("unknown operation '{other}'"),
        )),
    }
}

fn run_machine_api(machine: &mut MachineSession, snapshot_dir: &Path) -> Result<(), String> {
    let stdin = std::io::stdin();
    let mut stdout = std::io::BufWriter::new(std::io::stdout().lock());
    let mut state = MachineApiState::default();
    for line in stdin.lock().lines() {
        let line = line.map_err(|error| format!("reading machine API stdin: {error}"))?;
        if line.len() > MACHINE_MAX_REQUEST_BYTES {
            let error = invalid_request(format!(
                "request contains {} bytes, limit is {MACHINE_MAX_REQUEST_BYTES}",
                line.len()
            ));
            stdout
                .write_all(
                    machine_protocol::error_response_line(None, machine.cycles(), &error)
                        .as_bytes(),
                )
                .map_err(|write| format!("writing machine API response: {write}"))?;
            stdout
                .flush()
                .map_err(|flush| format!("flushing machine API response: {flush}"))?;
            continue;
        }
        let parsed = match machine_protocol::parse_request_line(&line) {
            Ok(value) => value,
            Err(error) => {
                stdout
                    .write_all(
                        machine_protocol::error_response_line(None, machine.cycles(), &error)
                            .as_bytes(),
                    )
                    .map_err(|write| format!("writing machine API response: {write}"))?;
                stdout
                    .flush()
                    .map_err(|flush| format!("flushing machine API response: {flush}"))?;
                continue;
            }
        };
        let correlation = machine_protocol::correlation_id(&parsed);
        let header = match machine_protocol::parse_request_header(&parsed) {
            Ok(header) => header,
            Err(error) => {
                stdout
                    .write_all(
                        machine_protocol::error_response_line(
                            correlation.as_ref(),
                            machine.cycles(),
                            &error,
                        )
                        .as_bytes(),
                    )
                    .map_err(|write| format!("writing machine API response: {write}"))?;
                stdout
                    .flush()
                    .map_err(|flush| format!("flushing machine API response: {flush}"))?;
                continue;
            }
        };
        let response =
            match dispatch_machine_request(machine, &mut state, &parsed, &header, snapshot_dir) {
                Ok((result, state_changed)) => {
                    let events = if state_changed {
                        match collect_subscription_events(machine, &mut state) {
                            Ok(events) => events,
                            Err(error) => {
                                let line = machine_protocol::error_response_line(
                                    Some(&header.id),
                                    machine.cycles(),
                                    &error,
                                );
                                stdout.write_all(line.as_bytes()).map_err(|write| {
                                    format!("writing machine API response: {write}")
                                })?;
                                stdout.flush().map_err(|flush| {
                                    format!("flushing machine API response: {flush}")
                                })?;
                                continue;
                            }
                        }
                    } else {
                        Vec::new()
                    };
                    machine_protocol::success_response_line(
                        &header.id,
                        machine.cycles(),
                        result,
                        events,
                    )
                }
                Err(error) => machine_protocol::error_response_line(
                    Some(&header.id),
                    machine.cycles(),
                    &error,
                ),
            };
        stdout
            .write_all(response.as_bytes())
            .map_err(|error| format!("writing machine API response: {error}"))?;
        stdout
            .flush()
            .map_err(|error| format!("flushing machine API response: {error}"))?;
    }
    Ok(())
}

/// JSON string escaping per RFC 8259 §7.
fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{08}' => out.push_str("\\b"),
            '\u{0c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

fn json_string(s: &str) -> String {
    format!("\"{}\"", json_escape(s))
}

fn json_array_strings(values: &[String]) -> String {
    values
        .iter()
        .map(|value| json_string(value))
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(feature = "idle-profiler")]
fn u64_json_array(values: &[u64]) -> String {
    values
        .iter()
        .map(u64::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(feature = "idle-profiler")]
fn histogram_json(value: &CumulativeHistogramSnapshot) -> String {
    format!(
        "{{\"episodes_ge\": [{}], \"cycle_mass_ge\": [{}]}}",
        u64_json_array(&value.episodes_ge),
        u64_json_array(&value.cycle_mass_ge),
    )
}

#[cfg(feature = "idle-profiler")]
fn source_cycles_json(value: &IdleBlockerCycles) -> String {
    format!(
        concat!(
            "{{\"pio\": {}, \"dma\": {}, \"pwm\": {}, \"systick\": {}, ",
            "\"uart\": {}, \"spi\": {}, \"i2c\": {}, \"adc\": {}, ",
            "\"timer\": {}, \"pending_irq\": {}}}"
        ),
        value.pio,
        value.dma,
        value.pwm,
        value.systick,
        value.uart,
        value.spi,
        value.i2c,
        value.adc,
        value.timer,
        value.pending_irq,
    )
}

#[cfg(feature = "idle-profiler")]
fn source_episodes_json(value: &IdleBlockerEpisodes) -> String {
    format!(
        concat!(
            "{{\"pio\": {}, \"dma\": {}, \"pwm\": {}, \"systick\": {}, ",
            "\"uart\": {}, \"spi\": {}, \"i2c\": {}, \"adc\": {}, ",
            "\"timer\": {}, \"pending_irq\": {}}}"
        ),
        value.pio,
        value.dma,
        value.pwm,
        value.systick,
        value.uart,
        value.spi,
        value.i2c,
        value.adc,
        value.timer,
        value.pending_irq,
    )
}

#[cfg(feature = "idle-profiler")]
fn horizon_events_json(value: &IdleHorizonEvents) -> String {
    format!(
        concat!(
            "{{\"pio\": {}, \"dma\": {}, \"pwm\": {}, \"systick\": {}, ",
            "\"uart\": {}, \"spi\": {}, \"i2c\": {}, \"adc\": {}, ",
            "\"timer\": {}, \"pending_irq\": {}, \"external\": {}}}"
        ),
        value.pio,
        value.dma,
        value.pwm,
        value.systick,
        value.uart,
        value.spi,
        value.i2c,
        value.adc,
        value.timer,
        value.pending_irq,
        value.external,
    )
}

#[cfg(feature = "idle-profiler")]
#[allow(clippy::too_many_arguments)]
fn build_idle_profile_report(
    backend_commit: &str,
    backend_dirty: bool,
    firmware_name: &str,
    firmware_sha: &str,
    step_quantum: u32,
    outcome: &RunOutcome,
    profile: &IdleProfileSnapshot,
) -> String {
    let thresholds: [u64; IDLE_HISTOGRAM_BUCKETS] = std::array::from_fn(|i| 1u64 << i);
    format!(
        concat!(
            "{{\n",
            "  \"schema_version\": {},\n",
            "  \"kind\": \"rp2040_serial_idle_profile\",\n",
            "  \"backend_build\": {{\"commit\": {}, \"dirty\": {}}},\n",
            "  \"firmware\": {{\"basename\": {}, \"sha256\": {}}},\n",
            "  \"execution_model\": \"Serial\",\n",
            "  \"instrumented\": true,\n",
            "  \"valid_for_wall_time\": false,\n",
            "  \"step_quantum\": {},\n",
            "  \"stop_reason\": {},\n",
            "  \"run_cycles\": {},\n",
            "  \"histogram_thresholds_cycles\": [{}],\n",
            "  \"counters\": {{\n",
            "    \"step_calls\": {},\n",
            "    \"total_master_cycles\": {},\n",
            "    \"core0_executed_cycles\": {},\n",
            "    \"core1_executed_cycles\": {},\n",
            "    \"both_blocked_cycles\": {},\n",
            "    \"proven_safe_cycles\": {},\n",
            "    \"zero_progress_blocked_steps\": {},\n",
            "    \"core0_halted_blocked_cycles\": {},\n",
            "    \"core0_wfe_blocked_cycles\": {},\n",
            "    \"core1_halted_blocked_cycles\": {},\n",
            "    \"core1_wfe_blocked_cycles\": {}\n",
            "  }},\n",
            "  \"blocked_lengths\": {},\n",
            "  \"proven_safe_lengths\": {},\n",
            "  \"event_bounded_safe_lengths\": {},\n",
            "  \"horizon_boundary_events\": {},\n",
            "  \"initial_horizon_distances\": {},\n",
            "  \"blocker_cycles\": {},\n",
            "  \"blocker_episodes\": {},\n",
            "  \"stationary_source_cycles\": {},\n",
            "  \"stationary_source_episodes\": {},\n",
            "  \"exact_bulk_source_cycles\": {},\n",
            "  \"exact_bulk_source_episodes\": {}\n",
            "}}\n"
        ),
        IDLE_PROFILE_SCHEMA_VERSION,
        json_string(backend_commit),
        backend_dirty,
        json_string(firmware_name),
        json_string(firmware_sha),
        step_quantum,
        json_string(outcome.stop_reason.as_str()),
        outcome.cycles,
        u64_json_array(&thresholds),
        profile.step_calls,
        profile.total_master_cycles,
        profile.core0_executed_cycles,
        profile.core1_executed_cycles,
        profile.both_blocked_cycles,
        profile.proven_safe_cycles,
        profile.zero_progress_blocked_steps,
        profile.core0_halted_blocked_cycles,
        profile.core0_wfe_blocked_cycles,
        profile.core1_halted_blocked_cycles,
        profile.core1_wfe_blocked_cycles,
        histogram_json(&profile.blocked_lengths),
        histogram_json(&profile.proven_safe_lengths),
        histogram_json(&profile.event_bounded_safe_lengths),
        horizon_events_json(&profile.horizon_boundary_events),
        histogram_json(&profile.initial_horizon_distances),
        source_cycles_json(&profile.blockers),
        source_episodes_json(&profile.blocker_episodes),
        source_cycles_json(&profile.stationary_sources),
        source_episodes_json(&profile.stationary_source_episodes),
        source_cycles_json(&profile.exact_bulk_sources),
        source_episodes_json(&profile.exact_bulk_source_episodes),
    )
}

#[cfg(feature = "event-horizon-profiler")]
fn running_boundary_events_json(value: &RunningBoundaryEvents) -> String {
    format!(
        concat!(
            "{{\"cpu_mmio\": {}, \"gpio_in\": {}, \"fifo_dreq\": {}, ",
            "\"irq_exception\": {}, \"pio_device\": {}, \"dma_dreq\": {}, ",
            "\"timer_systick_pwm\": {}, \"serial\": {}, \"clock\": {}, ",
            "\"external\": {}}}"
        ),
        value.cpu_mmio,
        value.gpio_in,
        value.fifo_dreq,
        value.irq_exception,
        value.pio_device,
        value.dma_dreq,
        value.timer_systick_pwm,
        value.serial,
        value.clock,
        value.external,
    )
}

#[cfg(feature = "event-horizon-profiler")]
fn decode_profile_json(value: &DecodeProfileSnapshot) -> String {
    let lookup_hits_by_region = format!(
        "{{\"rom\": {}, \"immutable_xip_flash_aliases\": {}, \"xip_sram\": {}, \"sram\": {}, \"other\": {}}}",
        value.lookup_hits_by_region.rom,
        value.lookup_hits_by_region.immutable_xip_flash_aliases,
        value.lookup_hits_by_region.xip_sram,
        value.lookup_hits_by_region.sram,
        value.lookup_hits_by_region.other,
    );
    let lookup_misses_by_region = format!(
        "{{\"rom\": {}, \"immutable_xip_flash_aliases\": {}, \"xip_sram\": {}, \"sram\": {}, \"other\": {}}}",
        value.lookup_misses_by_region.rom,
        value.lookup_misses_by_region.immutable_xip_flash_aliases,
        value.lookup_misses_by_region.xip_sram,
        value.lookup_misses_by_region.sram,
        value.lookup_misses_by_region.other,
    );
    let immutable_xip_hit_run_termination_counters = format!(
        "{{\"post_execute_next_pc_redirect\": {}, \"xip_miss\": {}, \"region_exit\": {}, \"prefetch_exception\": {}, \"fault\": {}}}",
        value
            .immutable_xip_hit_run_termination_counters
            .post_execute_next_pc_redirect,
        value.immutable_xip_hit_run_termination_counters.xip_miss,
        value.immutable_xip_hit_run_termination_counters.region_exit,
        value
            .immutable_xip_hit_run_termination_counters
            .prefetch_exception,
        value.immutable_xip_hit_run_termination_counters.fault,
    );
    let decode_cache_invalidation_observations = format!(
        "{{\"entry_address_count\": {}, \"rom\": {}, \"xip\": {}, \"sram\": {}, \"bulk\": {}, \"all\": {}}}",
        value
            .decode_cache_invalidation_observations
            .entry_address_count,
        value.decode_cache_invalidation_observations.rom,
        value.decode_cache_invalidation_observations.xip,
        value.decode_cache_invalidation_observations.sram,
        value.decode_cache_invalidation_observations.bulk,
        value.decode_cache_invalidation_observations.all,
    );

    format!(
        concat!(
            "{{\"cacheable_hits\": {}, \"cacheable_misses\": {}, ",
            "\"noncacheable_fetches\": {}, ",
            "\"cacheable_hits_narrow\": {}, \"cacheable_hits_wide\": {}, ",
            "\"lookup_hits_by_region\": {}, ",
            "\"lookup_misses_by_region\": {}, ",
            "\"sequential_cache_hit_runs\": {}, ",
            "\"immutable_xip_hit_runs\": {}, ",
            "\"immutable_xip_hit_run_termination_counters\": {}, ",
            "\"decode_cache_invalidation_observations\": {}}}"
        ),
        value.cacheable_hits,
        value.cacheable_misses,
        value.noncacheable_fetches,
        value.cacheable_hits_narrow,
        value.cacheable_hits_wide,
        lookup_hits_by_region,
        lookup_misses_by_region,
        histogram_json(&value.sequential_cache_hit_runs),
        histogram_json(&value.immutable_xip_hit_runs),
        immutable_xip_hit_run_termination_counters,
        decode_cache_invalidation_observations,
    )
}

#[cfg(all(feature = "event-horizon-profiler", test))]
#[allow(clippy::too_many_arguments)]
fn build_running_event_profile_report(
    backend_commit: &str,
    backend_dirty: bool,
    firmware_name: &str,
    firmware_sha: &str,
    step_quantum: u32,
    outcome: &RunOutcome,
    profile: &RunningEventProfileSnapshot,
) -> String {
    build_running_event_profile_report_with_activation(
        backend_commit,
        backend_dirty,
        firmware_name,
        firmware_sha,
        step_quantum,
        None,
        None,
        outcome,
        profile,
    )
}

#[cfg(feature = "event-horizon-profiler")]
#[allow(clippy::too_many_arguments)]
fn build_running_event_profile_report_with_activation(
    backend_commit: &str,
    backend_dirty: bool,
    firmware_name: &str,
    firmware_sha: &str,
    step_quantum: u32,
    activation_marker: Option<&str>,
    activation_cycle: Option<u64>,
    outcome: &RunOutcome,
    profile: &RunningEventProfileSnapshot,
) -> String {
    let boundary = &profile.boundary;
    let thresholds: [u64; IDLE_HISTOGRAM_BUCKETS] = std::array::from_fn(|i| 1u64 << i);
    let activation = match activation_marker {
        Some(marker) => format!(
            "{{\"mode\":\"after_uart\",\"marker\":{},\"start_cycle\":{}}}",
            json_string(marker),
            activation_cycle.unwrap_or(0),
        ),
        None => "{\"mode\":\"from_start\"}".to_string(),
    };
    format!(
        concat!(
            "{{\n",
            "  \"schema_version\": {},\n",
            "  \"kind\": \"rp2040_serial_running_event_horizon_profile\",\n",
            "  \"backend_build\": {{\"commit\": {}, \"dirty\": {}}},\n",
            "  \"firmware\": {{\"basename\": {}, \"sha256\": {}}},\n",
            "  \"execution_model\": \"Serial\",\n",
            "  \"instrumented\": true,\n",
            "  \"valid_for_wall_time\": false,\n",
            "  \"observed_gaps_are_safe_windows\": false,\n",
            "  \"fallback_occupancy_is_safe_window\": false,\n",
            "  \"decode_hit_runs_are_speedup_prediction\": false,\n",
            "  \"immutable_xip_hit_runs_are_speedup_prediction\": false,\n",
            "  \"conservative_horizon_complete_for_current_model\": true,\n",
            "  \"step_quantum\": {},\n",
            "  \"activation\": {},\n",
            "  \"stop_reason\": {},\n",
            "  \"run_cycles\": {},\n",
            "  \"histogram_thresholds_cycles\": [{}],\n",
            "  \"counters\": {{\"running_steps\": {}, \"total_running_cycles\": {}, ",
            "\"boundary_steps\": {}, \"no_known_horizon_steps\": {}, ",
            "\"no_known_horizon_cycles\": {}, \"candidate_dispatches\": {}, ",
            "\"candidate_cycles\": {}}},\n",
            "  \"observed_inter_boundary_dispatches\": {},\n",
            "  \"observed_inter_boundary_cycles\": {},\n",
            "  \"observed_candidate_dispatches\": {},\n",
            "  \"observed_candidate_cycles\": {},\n",
            "  \"conservative_horizon_distances\": {},\n",
            "  \"boundary_events\": {},\n",
            "  \"one_cycle_fallback_cycles\": {},\n",
            "  \"one_cycle_fallback_signatures\": {{\"bit_order\": ",
            "[\"pio\", \"uart\", \"dma\", \"any_other\"], ",
            "\"steps\": [{}], \"cycle_mass\": [{}]}},\n",
            "  \"decode_opportunity_by_core\": [\n",
            "    {},\n",
            "    {}\n",
            "  ]\n",
            "}}\n"
        ),
        RUNNING_EVENT_PROFILE_SCHEMA_VERSION,
        json_string(backend_commit),
        backend_dirty,
        json_string(firmware_name),
        json_string(firmware_sha),
        step_quantum,
        activation,
        json_string(outcome.stop_reason.as_str()),
        outcome.cycles,
        u64_json_array(&thresholds),
        boundary.running_steps,
        boundary.total_running_cycles,
        boundary.boundary_steps,
        boundary.no_known_horizon_steps,
        boundary.no_known_horizon_cycles,
        boundary.candidate_dispatches,
        boundary.candidate_cycles,
        histogram_json(&boundary.observed_inter_boundary_dispatches),
        histogram_json(&boundary.observed_inter_boundary_cycles),
        histogram_json(&boundary.observed_candidate_dispatches),
        histogram_json(&boundary.observed_candidate_cycles),
        histogram_json(&boundary.conservative_horizon_distances),
        running_boundary_events_json(&boundary.boundary_events),
        horizon_events_json(&boundary.one_cycle_fallback_cycles),
        u64_json_array(&boundary.one_cycle_fallback_signatures.steps),
        u64_json_array(&boundary.one_cycle_fallback_signatures.cycle_mass),
        decode_profile_json(&profile.decode_by_core[0]),
        decode_profile_json(&profile.decode_by_core[1]),
    )
}

/// Everything the report says about the attached panel. Snapshotted out
/// of the shared model once the run has finished so the JSON builder
/// stays a pure function of plain data.
struct LcdReport {
    reset_pulses: u32,
    swreset: u32,
    slpout: u32,
    slpin: u32,
    dispon: u32,
    dispoff: u32,
    inverted: bool,
    sleeping: bool,
    display_on: bool,
    madctl: u8,
    colmod: u8,
    caset: u32,
    raset: u32,
    ramwr: u32,
    ramrd: u32,
    pixels_written: u64,
    pixels_dropped: u64,
    orphan_data_bytes: u64,
    unknown_commands: Vec<(u8, u32)>,
}

impl LcdReport {
    fn snapshot(lcd: &St7365p) -> Self {
        Self {
            reset_pulses: lcd.reset_pulses,
            swreset: lcd.swreset_count,
            slpout: lcd.slpout_count,
            slpin: lcd.slpin_count,
            dispon: lcd.dispon_count,
            dispoff: lcd.dispoff_count,
            inverted: lcd.inverted,
            sleeping: lcd.sleeping,
            display_on: lcd.display_on,
            madctl: lcd.madctl,
            colmod: lcd.colmod_reg,
            caset: lcd.caset_count,
            raset: lcd.raset_count,
            ramwr: lcd.ramwr_count,
            ramrd: lcd.ramrd_count,
            pixels_written: lcd.pixels_written,
            pixels_dropped: lcd.pixels_dropped,
            orphan_data_bytes: lcd.orphan_data_bytes,
            unknown_commands: lcd
                .unknown_commands()
                .iter()
                .map(|u| (u.code, u.count))
                .collect(),
        }
    }

    fn to_json(&self) -> String {
        let mut s = String::new();
        s.push_str("  \"lcd\": {\n");
        s.push_str(&format!("    \"reset_pulses\": {},\n", self.reset_pulses));
        s.push_str(&format!(
            "    \"init\": {{\"swreset\": {}, \"slpout\": {}, \"slpin\": {}, \
             \"dispon\": {}, \"dispoff\": {}}},\n",
            self.swreset, self.slpout, self.slpin, self.dispon, self.dispoff
        ));
        s.push_str(&format!(
            "    \"state\": {{\"sleeping\": {}, \"display_on\": {}, \"inverted\": {}}},\n",
            self.sleeping, self.display_on, self.inverted
        ));
        s.push_str(&format!(
            "    \"madctl\": \"{:#04x}\",\n    \"colmod\": \"{:#04x}\",\n",
            self.madctl, self.colmod
        ));
        s.push_str(&format!(
            "    \"caset\": {}, \"raset\": {}, \"ramwr\": {}, \"ramrd\": {},\n",
            self.caset, self.raset, self.ramwr, self.ramrd
        ));
        s.push_str(&format!(
            "    \"pixels_written\": {}, \"pixels_dropped\": {}, \"orphan_data_bytes\": {},\n",
            self.pixels_written, self.pixels_dropped, self.orphan_data_bytes
        ));
        s.push_str("    \"unknown_commands\": [");
        if self.unknown_commands.is_empty() {
            s.push_str("]\n");
        } else {
            s.push('\n');
            for (i, (code, count)) in self.unknown_commands.iter().enumerate() {
                s.push_str(&format!(
                    "      {{\"code\": \"{code:#04x}\", \"count\": {count}}}"
                ));
                if i + 1 < self.unknown_commands.len() {
                    s.push(',');
                }
                s.push('\n');
            }
            s.push_str("    ]\n");
        }
        s.push_str("  },\n");
        s
    }
}

/// The `framebuffer` report section.
struct FramebufferReport {
    width: usize,
    height: usize,
    rgb565_sha256: String,
    non_black_pixels: usize,
    png_basename: Option<String>,
}

impl FramebufferReport {
    fn to_json(&self) -> String {
        let mut s = String::new();
        s.push_str("  \"framebuffer\": {");
        s.push_str(&format!(
            "\"width\": {}, \"height\": {}, \"rgb565_sha256\": {}, \"non_black_pixels\": {}, \
             \"png\": {}}},\n",
            self.width,
            self.height,
            json_string(&self.rgb565_sha256),
            self.non_black_pixels,
            match &self.png_basename {
                Some(name) => json_string(name),
                None => "null".to_string(),
            }
        ));
        s
    }
}

/// Result of checking a PSRAM buffer range against the `addr & 0xFF`
/// pattern that picocalc_helloworld's `psram_test()` writes in its
/// 8-bit pass (`main.c`: `psram_write8(psram_spi, addr, addr & 0xFF)`).
struct PsramVerifyReport {
    start: u32,
    len: u32,
    matched: u64,
    mismatched: u64,
    /// `(addr, expected, actual)` of the first mismatch, if any.
    first_mismatch: Option<(u32, u8, u8)>,
}

fn verify_psram_range(buffer: &[u8], start: u32, len: u32) -> PsramVerifyReport {
    let mut matched: u64 = 0;
    let mut mismatched: u64 = 0;
    let mut first_mismatch: Option<(u32, u8, u8)> = None;
    let size = buffer.len() as u32;
    for i in 0..len {
        let addr = start.wrapping_add(i);
        let off = (addr & (size - 1)) as usize;
        let expected = (addr & 0xFF) as u8;
        let actual = buffer[off];
        if actual == expected {
            matched += 1;
        } else {
            mismatched += 1;
            if first_mismatch.is_none() {
                first_mismatch = Some((addr, expected, actual));
            }
        }
    }
    PsramVerifyReport {
        start,
        len,
        matched,
        mismatched,
        first_mismatch,
    }
}

/// The `psram` report section.
/// The `pio` report section (Gate 7).
///
/// Variant B of the PicoCalc display driver pushes pixels through a PIO
/// state machine rather than the SSP, and firmware waits on `FSTAT`
/// before touching CS or DC. When a run stalls in that wait, the useful
/// question is whether the state machine is enabled and moving at all.
struct PioReport {
    blocks: Vec<(usize, u32, [bool; 4], [u8; 4])>,
}

impl PioReport {
    fn collect(bus: &mut rp2040_emu::Bus) -> Self {
        let mut blocks = Vec::new();
        for (index, base) in [
            (0usize, rp2040_emu::bus::PIO0_BASE),
            (1usize, rp2040_emu::bus::PIO1_BASE),
        ] {
            let fstat = bus.read32(base + 0x004);
            let mut enabled = [false; 4];
            let mut pcs = [0u8; 4];
            for sm in 0..4 {
                enabled[sm] = bus.pio[index].sm[sm].enabled();
                pcs[sm] = bus.pio[index].sm[sm].pc();
            }
            if enabled.iter().any(|e| *e) || fstat != 0x0F00_0F00 {
                blocks.push((index, fstat, enabled, pcs));
            }
        }
        Self { blocks }
    }

    fn to_json(&self) -> String {
        let mut s = String::new();
        s.push_str("  \"pio\": [");
        for (i, (index, fstat, enabled, pcs)) in self.blocks.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&format!(
                "\n    {{\"block\": {index}, \"fstat\": \"0x{fstat:08x}\", \"sm_enabled\": [{}], \"sm_pc\": [{}]}}",
                enabled
                    .iter()
                    .map(|e| e.to_string())
                    .collect::<Vec<_>>()
                    .join(", "),
                pcs.iter()
                    .map(|p| p.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
        }
        if self.blocks.is_empty() {
            s.push_str("],\n");
        } else {
            s.push_str("\n  ],\n");
        }
        s
    }
}

/// The `pwm` report section (Gate 5).
///
/// `picocalc_helloworld` initialises the two audio slices but never
/// starts sample playback, so the acceptance condition is that the
/// configuration is observable — not that anything is audible.
struct PwmReport {
    configured_slices: Vec<(usize, u32, u16, u32)>,
    inte: u8,
}

impl PwmReport {
    fn collect(bus: &rp2040_emu::Bus) -> Self {
        let pwm = bus.pwm();
        let mut configured_slices = Vec::new();
        for index in 0..rp2040_emu::peripherals::pwm::PWM_SLICE_COUNT {
            if let Some(slice) = pwm.slice(index) {
                let touched = slice.csr != 0 || slice.top != TOP_RESET_VALUE || slice.cc != 0;
                if touched {
                    configured_slices.push((index, slice.csr, slice.top, slice.cc));
                }
            }
        }
        Self {
            configured_slices,
            inte: pwm.inte(),
        }
    }

    fn to_json(&self) -> String {
        let mut s = String::new();
        s.push_str("  \"pwm\": {\n");
        s.push_str(&format!("    \"inte\": \"0x{:02x}\",\n", self.inte));
        s.push_str("    \"configured_slices\": [");
        for (i, (index, csr, top, cc)) in self.configured_slices.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&format!(
                "\n      {{\"slice\": {index}, \"csr\": \"0x{csr:08x}\", \"top\": {top}, \"cc\": \"0x{cc:08x}\"}}"
            ));
        }
        if self.configured_slices.is_empty() {
            s.push_str("]\n");
        } else {
            s.push_str("\n    ]\n");
        }
        s.push_str("  },\n");
        s
    }
}

/// Digital sample stream observed at the DMA-to-PWM boundary.
struct AudioSinkReport {
    snapshot: rp2040_emu::AudioSinkSnapshot,
    expected_count: Option<u64>,
    expected_sha256: Option<String>,
    status: &'static str,
}

impl AudioSinkReport {
    fn status_for_expected(
        snapshot: &rp2040_emu::AudioSinkSnapshot,
        expected_count: Option<u64>,
        expected_sha256: Option<&str>,
    ) -> &'static str {
        match (expected_count, expected_sha256) {
            (Some(count), Some(digest)) => {
                if snapshot.status == "pass"
                    && snapshot.dma_write_count == count
                    && snapshot.pcm_sha256 == digest
                {
                    "pass"
                } else {
                    "fail"
                }
            }
            (None, None) => snapshot.status,
            _ => "fail",
        }
    }

    fn collect(
        bus: &rp2040_emu::Bus,
        expected_count: Option<u64>,
        expected_sha256: Option<&str>,
    ) -> Self {
        let snapshot = bus.audio_sink_snapshot();
        let status = Self::status_for_expected(&snapshot, expected_count, expected_sha256);
        Self {
            snapshot,
            expected_count,
            expected_sha256: expected_sha256.map(str::to_owned),
            status,
        }
    }

    fn expectation_failed(&self) -> bool {
        self.expected_count.is_some() && self.status != "pass"
    }

    fn analysis_json(
        &self,
        backend_commit: &str,
        backend_dirty: bool,
        firmware_name: &str,
        firmware_sha256: &str,
    ) -> String {
        // Schema 1 is the frozen NEXT-2 48 kHz artifact. A non-48 kHz
        // stream uses the additive generic schema 2. Block lengths remain
        // visible in the report's audio_sink projection.
        let schema_version = if self.snapshot.sample_rate_hz == 48_000 {
            1
        } else {
            2
        };
        format!(
            concat!(
                "{{\n",
                "  \"schema_version\": {},\n",
                "  \"boundary\": \"dma_to_pwm5_cc\",\n",
                "  \"interpretation\": \"digital_level_only_not_speaker_loudness\",\n",
                "  \"backend_build\": {{\"commit\": {}, \"dirty\": {}}},\n",
                "  \"firmware\": {{\"file\": {}, \"sha256\": {}}},\n",
                "  \"observation_status\": {},\n",
                "  \"pcm_sha256\": {},\n",
                "  \"pcm_format\": {},\n",
                "  \"sample_rate_hz\": {},\n",
                "  \"channel_count\": {},\n",
                "  \"frame_count\": {},\n",
                "  \"window_frames\": {},\n",
                "  \"active_abs_threshold\": {},\n",
                "  \"peak_abs_left\": {},\n",
                "  \"peak_abs_right\": {},\n",
                "  \"stream_rms\": {},\n",
                "  \"max_window_rms\": {},\n",
                "  \"dc_offset_left\": {},\n",
                "  \"dc_offset_right\": {},\n",
                "  \"active_frame_count\": {},\n",
                "  \"active_frame_ratio_ppm\": {},\n",
                "  \"rail_sample_count\": {},\n",
                "  \"rail_sample_ratio_ppm\": {},\n",
                "  \"max_consecutive_rail_frames\": {},\n",
                "  \"out_of_range_duty_sample_count\": {},\n",
                "  \"rail_interpretation\": \"post_quantizer_pwm_rail_usage_not_source_clip_count\"\n",
                "}}\n"
            ),
            schema_version,
            json_string(backend_commit),
            backend_dirty,
            json_string(firmware_name),
            json_string(firmware_sha256),
            json_string(self.snapshot.status),
            json_string(&self.snapshot.pcm_sha256),
            json_string(self.snapshot.reconstructed_pcm_format),
            self.snapshot.sample_rate_hz,
            self.snapshot.channel_count,
            self.snapshot.analysis_frame_count,
            self.snapshot.analysis_window_frames,
            self.snapshot.active_abs_threshold,
            self.snapshot.peak_abs_left,
            self.snapshot.peak_abs_right,
            self.snapshot.stream_rms,
            self.snapshot.max_window_rms,
            self.snapshot.dc_offset_left,
            self.snapshot.dc_offset_right,
            self.snapshot.active_frame_count,
            self.snapshot.active_frame_ratio_ppm,
            self.snapshot.rail_sample_count,
            self.snapshot.rail_sample_ratio_ppm,
            self.snapshot.max_consecutive_rail_frames,
            self.snapshot.out_of_range_duty_sample_count,
        )
    }

    fn to_json(&self) -> String {
        let words = |values: &[u32]| {
            values
                .iter()
                .map(|value| json_string(&format!("0x{value:08x}")))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let option_u64 = |value: Option<u64>| {
            value
                .map(|number| number.to_string())
                .unwrap_or_else(|| "null".to_string())
        };
        format!(
            concat!(
                "  \"audio_sink\": {{\n",
                "    \"status\": {},\n",
                "    \"dma_write_count\": {},\n",
                "    \"target_write_attempt_count\": {},\n",
                "    \"other_pwm_cc_write_count\": {},\n",
                "    \"wrong_width_count\": {},\n",
                "    \"wrong_treq_count\": {},\n",
                "    \"missing_due_cycle_count\": {},\n",
                "    \"pcm_sha256\": {},\n",
                "    \"expected_count\": {},\n",
                "    \"expected_sha256\": {},\n",
                "    \"first_words\": [{}],\n",
                "    \"last_words\": [{}],\n",
                "    \"timer_index\": {},\n",
                "    \"treq\": {},\n",
                "    \"timer_fraction\": \"{}/{}\",\n",
                "    \"sample_rate_hz\": {},\n",
                "    \"timer_event_count\": {},\n",
                "    \"timer_miss_count\": {},\n",
                "    \"timer_miss_audio_not_busy\": {},\n",
                "    \"timer_miss_other_dma_selected\": {},\n",
                "    \"timer_miss_no_dma_selected\": {},\n",
                "    \"timer_miss_multiple_due_in_window\": {},\n",
                "    \"timer_due_cycle_sha256\": {},\n",
                "    \"block_start_count\": {},\n",
                "    \"block_frame_min\": {},\n",
                "    \"block_frame_max\": {},\n",
                "    \"malformed_block_count\": {},\n",
                "    \"block_boundary_gap_count\": {},\n",
                "    \"block_boundary_gap_min_cycles\": {},\n",
                "    \"block_boundary_gap_max_cycles\": {},\n",
                "    \"block_boundary_gap_sha256\": {},\n",
                "    \"gap_5208_count\": {},\n",
                "    \"gap_5209_count\": {},\n",
                "    \"unexpected_gap_count\": {},\n",
                "    \"service_latency_min_cycles\": {},\n",
                "    \"service_latency_max_cycles\": {},\n",
                "    \"service_latency_sha256\": {}\n",
                "  }},\n"
            ),
            json_string(self.status),
            self.snapshot.dma_write_count,
            self.snapshot.target_write_attempt_count,
            self.snapshot.other_pwm_cc_write_count,
            self.snapshot.wrong_width_count,
            self.snapshot.wrong_treq_count,
            self.snapshot.missing_due_cycle_count,
            json_string(&self.snapshot.pcm_sha256),
            option_u64(self.expected_count),
            self.expected_sha256
                .as_deref()
                .map(json_string)
                .unwrap_or_else(|| "null".to_string()),
            words(&self.snapshot.first_words),
            words(&self.snapshot.last_words),
            self.snapshot.timer_index,
            self.snapshot.treq,
            self.snapshot.timer_fraction_x,
            self.snapshot.timer_fraction_y,
            self.snapshot.sample_rate_hz,
            self.snapshot.timer_event_count,
            self.snapshot.timer_miss_count,
            self.snapshot.timer_miss_audio_not_busy,
            self.snapshot.timer_miss_other_dma_selected,
            self.snapshot.timer_miss_no_dma_selected,
            self.snapshot.timer_miss_multiple_due_in_window,
            json_string(&self.snapshot.timer_due_cycle_sha256),
            self.snapshot.block_start_count,
            option_u64(self.snapshot.block_frame_min),
            option_u64(self.snapshot.block_frame_max),
            self.snapshot.malformed_block_count,
            self.snapshot.block_boundary_gap_count,
            option_u64(self.snapshot.block_boundary_gap_min_cycles),
            option_u64(self.snapshot.block_boundary_gap_max_cycles),
            json_string(&self.snapshot.block_boundary_gap_sha256),
            self.snapshot.gap_5208_count,
            self.snapshot.gap_5209_count,
            self.snapshot.unexpected_gap_count,
            option_u64(self.snapshot.service_latency_min_cycles),
            option_u64(self.snapshot.service_latency_max_cycles),
            json_string(&self.snapshot.service_latency_sha256),
        )
    }
}

fn write_audio_wav(path: &Path, samples: &[i16], sample_rate_hz: u32) -> Result<(), String> {
    if !samples.len().is_multiple_of(2) {
        return Err("audio PCM capture is not interleaved stereo".to_string());
    }
    if sample_rate_hz == 0 {
        return Err("audio PCM capture has no observed sample rate".to_string());
    }
    let data_size = u32::try_from(samples.len().saturating_mul(2))
        .map_err(|_| "audio WAV exceeds the RIFF 32-bit size limit".to_string())?;
    let riff_size = 36u32
        .checked_add(data_size)
        .ok_or_else(|| "audio WAV exceeds the RIFF 32-bit size limit".to_string())?;
    let byte_rate = sample_rate_hz
        .checked_mul(4)
        .ok_or_else(|| "audio WAV sample rate exceeds the RIFF byte-rate limit".to_string())?;
    let file = std::fs::File::create(path)
        .map_err(|e| format!("creating audio WAV {}: {e}", path.display()))?;
    let mut output = BufWriter::new(file);
    output
        .write_all(b"RIFF")
        .and_then(|_| output.write_all(&riff_size.to_le_bytes()))
        .and_then(|_| output.write_all(b"WAVEfmt "))
        .and_then(|_| output.write_all(&16u32.to_le_bytes()))
        .and_then(|_| output.write_all(&1u16.to_le_bytes()))
        .and_then(|_| output.write_all(&2u16.to_le_bytes()))
        .and_then(|_| output.write_all(&sample_rate_hz.to_le_bytes()))
        .and_then(|_| output.write_all(&byte_rate.to_le_bytes()))
        .and_then(|_| output.write_all(&4u16.to_le_bytes()))
        .and_then(|_| output.write_all(&16u16.to_le_bytes()))
        .and_then(|_| output.write_all(b"data"))
        .and_then(|_| output.write_all(&data_size.to_le_bytes()))
        .map_err(|e| format!("writing audio WAV header {}: {e}", path.display()))?;
    for sample in samples {
        output
            .write_all(&sample.to_le_bytes())
            .map_err(|e| format!("writing audio WAV samples {}: {e}", path.display()))?;
    }
    output
        .flush()
        .map_err(|e| format!("flushing audio WAV {}: {e}", path.display()))
}

fn write_flash_image(path: &Path, image: &[u8]) -> Result<(), String> {
    let temporary = path.with_extension(format!("tmp-{}", std::process::id()));
    std::fs::write(&temporary, image)
        .map_err(|error| format!("writing flash image {}: {error}", temporary.display()))?;
    std::fs::rename(&temporary, path)
        .map_err(|error| format!("publishing flash image {}: {error}", path.display()))
}

/// TOP register reset value; a slice still holding it was never given a
/// wrap point.
const TOP_RESET_VALUE: u16 = 0xFFFF;

/// The attached SD card and the initial filesystem profile supplied to it.
struct SdReport {
    format: SdFormat,
    block_count: usize,
    commands_seen: u64,
    blocks_read: u64,
    blocks_written: u64,
    unknown_commands: Vec<(u8, u32)>,
    raw: Option<picocalc_board::sdcard::RawMetadata>,
}

impl SdReport {
    fn snapshot(card: &SdCard) -> Self {
        let mut unknown_commands = card.unknown_commands.clone();
        unknown_commands.sort_by_key(|&(command, _)| command);
        Self {
            format: card.format(),
            block_count: card.block_count(),
            commands_seen: card.commands_seen,
            blocks_read: card.blocks_read,
            blocks_written: card.blocks_written,
            unknown_commands,
            raw: card.raw_metadata(),
        }
    }

    fn to_json(&self) -> String {
        let mut s = String::new();
        s.push_str("  \"sd\": {\n");
        s.push_str(&format!(
            "    \"attached\": true, \"format\": {}, \"block_size\": {}, \"block_count\": {},\n",
            json_string(self.format.as_str()),
            picocalc_board::sdcard::BLOCK_SIZE,
            self.block_count
        ));
        s.push_str(&format!(
            "    \"commands_seen\": {}, \"blocks_read\": {}, \"blocks_written\": {},\n",
            self.commands_seen, self.blocks_read, self.blocks_written
        ));
        if let Some(raw) = self.raw.as_ref() {
            s.push_str(&format!(
                "    \"raw_image\": {{\"bytes\": {}, \"blocks\": {}, \"dirty_blocks\": {}, \"source_sha256\": {} }},\n",
                raw.bytes,
                raw.blocks,
                raw.dirty_blocks,
                json_string(&raw.source_sha256),
            ));
        }
        s.push_str("    \"unknown_commands\": [");
        if self.unknown_commands.is_empty() {
            s.push_str("]\n");
        } else {
            for (index, (command, count)) in self.unknown_commands.iter().enumerate() {
                if index == 0 {
                    s.push('\n');
                }
                s.push_str(&format!(
                    "      {{\"command\": {}, \"count\": {}}}",
                    command, count
                ));
                if index + 1 < self.unknown_commands.len() {
                    s.push(',');
                }
                s.push('\n');
            }
            s.push_str("    ]\n");
        }
        s.push_str("  },\n");
        s
    }
}

/// The `keyboard` report section (Gate 4).
struct KeyboardReport {
    attached: bool,
    addr: u16,
    reg_selects: u64,
    key_events_delivered: u64,
    key_events_remaining: usize,
    key_events_dropped: u64,
    key_events_overwritten: u64,
    battery_reads: u64,
    backlight_writes: u64,
    backlight: u8,
    lcd_backlight: u8,
    config: u8,
    interrupt_status: u8,
    caps_lock: bool,
    num_lock: bool,
    unknown_reg_selects: u64,
    unknown_reg_writes: u64,
    last_unknown_reg: Option<u8>,
}

impl KeyboardReport {
    fn to_json(&self) -> String {
        let mut s = String::new();
        s.push_str("  \"keyboard\": {\n");
        s.push_str(&format!(
            "    \"attached\": {}, \"addr\": \"0x{:02x}\",\n",
            self.attached, self.addr
        ));
        s.push_str(&format!(
            "    \"reg_selects\": {}, \"key_events_delivered\": {}, \"key_events_remaining\": {},\n",
            self.reg_selects, self.key_events_delivered, self.key_events_remaining
        ));
        s.push_str(&format!(
            "    \"key_events_dropped\": {}, \"key_events_overwritten\": {},\n",
            self.key_events_dropped, self.key_events_overwritten
        ));
        s.push_str(&format!(
            "    \"battery_reads\": {}, \"backlight_writes\": {}, \"backlight\": {}, \"lcd_backlight\": {},\n",
            self.battery_reads, self.backlight_writes, self.backlight, self.lcd_backlight
        ));
        s.push_str(&format!(
            "    \"config\": \"0x{:02x}\", \"interrupt_status\": \"0x{:02x}\", \"caps_lock\": {}, \"num_lock\": {},\n",
            self.config, self.interrupt_status, self.caps_lock, self.num_lock
        ));
        match self.last_unknown_reg {
            Some(reg) => s.push_str(&format!(
                "    \"unknown_reg_selects\": {}, \"unknown_reg_writes\": {}, \"last_unknown_reg\": \"0x{:02x}\"\n",
                self.unknown_reg_selects, self.unknown_reg_writes, reg
            )),
            None => s.push_str(&format!(
                "    \"unknown_reg_selects\": {}, \"unknown_reg_writes\": {}, \"last_unknown_reg\": null\n",
                self.unknown_reg_selects, self.unknown_reg_writes
            )),
        }
        s.push_str("  },\n");
        s
    }
}

struct PsramReport {
    attached: bool,
    tick_count: u64,
    cs_falling_count: u64,
    bytes_written: u64,
    bytes_read: u64,
    cmd_write_count: u64,
    cmd_fast_read_count: u64,
    cmd_reset_enable_count: u64,
    cmd_reset_count: u64,
    cmd_unknown_count: u64,
    verify: Option<PsramVerifyReport>,
}

impl PsramReport {
    fn to_json(&self) -> String {
        let mut s = String::new();
        s.push_str("  \"psram\": {\n");
        s.push_str(&format!("    \"attached\": {},\n", self.attached));
        s.push_str(&format!(
            "    \"tick_count\": {}, \"cs_falling_count\": {},\n",
            self.tick_count, self.cs_falling_count
        ));
        s.push_str(&format!(
            "    \"bytes_written\": {}, \"bytes_read\": {},\n",
            self.bytes_written, self.bytes_read
        ));
        s.push_str(&format!(
            "    \"cmd_counts\": {{\"write\": {}, \"fast_read\": {}, \"reset_enable\": {}, \
             \"reset\": {}, \"unknown\": {}}},\n",
            self.cmd_write_count,
            self.cmd_fast_read_count,
            self.cmd_reset_enable_count,
            self.cmd_reset_count,
            self.cmd_unknown_count
        ));
        s.push_str("    \"verify\": ");
        match &self.verify {
            None => s.push_str("null\n"),
            Some(v) => {
                s.push_str(&format!(
                    "{{\"range\": \"{:#010x}:{:#x}\", \"matched\": {}, \"mismatched\": {}, \
                     \"first_mismatch\": {}}}\n",
                    v.start,
                    v.len,
                    v.matched,
                    v.mismatched,
                    match v.first_mismatch {
                        Some((addr, expected, actual)) => format!(
                            "{{\"addr\": \"{addr:#010x}\", \"expected\": \"{expected:#04x}\", \
                             \"actual\": \"{actual:#04x}\"}}"
                        ),
                        None => "null".to_string(),
                    }
                ));
            }
        }
        s.push_str("  },\n");
        s
    }
}

struct FlashReport {
    image_bytes: usize,
    erase_count: u64,
    program_count: u64,
    program_bytes: u64,
    command_counts: Vec<(u8, u32)>,
    unknown_commands: Vec<(u8, u32)>,
    errors: Vec<String>,
}

impl FlashReport {
    fn to_json(&self) -> String {
        let unknown = self
            .unknown_commands
            .iter()
            .map(|(command, count)| format!("{{\"command\": {command}, \"count\": {count}}}"))
            .collect::<Vec<_>>()
            .join(", ");
        let commands = self
            .command_counts
            .iter()
            .map(|(command, count)| format!("{{\"command\": {command}, \"count\": {count}}}"))
            .collect::<Vec<_>>()
            .join(", ");
        format!(
            "  \"flash\": {{\"image_bytes\": {}, \"erase_count\": {}, \"program_count\": {}, \"program_bytes\": {}, \"command_counts\": [{}], \"unknown_commands\": [{}], \"errors\": [{}]}},\n",
            self.image_bytes,
            self.erase_count,
            self.program_count,
            self.program_bytes,
            commands,
            unknown,
            json_array_strings(&self.errors),
        )
    }
}

#[allow(clippy::too_many_arguments)]
fn build_report(
    backend_commit: &str,
    backend_dirty: bool,
    firmware_name: &str,
    firmware_sha: &str,
    bootrom_name: &str,
    bootrom_sha: &str,
    boot_mode: BootMode,
    step_quantum: u32,
    cycle_limit: u64,
    stop_pc: Option<u32>,
    outcome: &RunOutcome,
    unsupported: &[(u32, u32, u64)],
    unsupported_truncated: bool,
    uart_sha: &str,
    board: Board,
    lcd_variant: LcdVariant,
    lcd: Option<&LcdReport>,
    fb: Option<&FramebufferReport>,
    psram: Option<&PsramReport>,
    flash: Option<&FlashReport>,
    sd: Option<&SdReport>,
    keyboard: Option<&KeyboardReport>,
    pwm: Option<&PwmReport>,
    audio_sink: Option<&AudioSinkReport>,
    pio: Option<&PioReport>,
    scenario: Option<(&scenario::Engine, String)>,
    verdict: &VerdictReport,
) -> String {
    let mut s = String::new();
    s.push_str("{\n");
    s.push_str(&format!("  \"schema_version\": {SCHEMA_VERSION},\n"));
    s.push_str(&format!(
        "  \"backend_commit\": {},\n",
        json_string(backend_commit)
    ));
    s.push_str(&format!(
        "  \"backend_build\": {{\"commit\": {}, \"dirty\": {}}},\n",
        json_string(backend_commit),
        backend_dirty
    ));
    s.push_str(&format!(
        "  \"firmware\": {{\"basename\": {}, \"sha256\": {}}},\n",
        json_string(firmware_name),
        json_string(firmware_sha)
    ));
    s.push_str(&format!(
        "  \"bootrom\": {{\"basename\": {}, \"sha256\": {}, \"executed\": false}},\n",
        json_string(bootrom_name),
        json_string(bootrom_sha)
    ));
    s.push_str("  \"execution_model\": \"Serial\",\n");
    s.push_str(&format!("  \"board\": {},\n", json_string(board.as_str())));
    // Which display transport produced this run. Recorded because the
    // two variants reach very different cycle counts for the same
    // firmware — a report without it invites the reader to compare runs
    // that are not comparable.
    s.push_str(&format!(
        "  \"lcd_variant\": {},\n",
        json_string(lcd_variant.as_str())
    ));
    s.push_str(&format!(
        "  \"boot\": {{\"mode\": {}, \"vtor_flash_offset\": \"{:#06x}\"}},\n",
        json_string(boot_mode.as_str()),
        SDK_VTOR_FLASH_OFFSET
    ));
    s.push_str(&format!("  \"step_quantum\": {step_quantum},\n"));
    s.push_str(&format!("  \"cycle_limit\": {cycle_limit},\n"));
    s.push_str(&format!(
        "  \"stop_pc\": {},\n",
        match stop_pc {
            Some(pc) => json_string(&format!("{pc:#010x}")),
            None => "null".to_string(),
        }
    ));
    s.push_str(&format!(
        "  \"stop_reason\": {},\n",
        json_string(outcome.stop_reason.as_str())
    ));
    s.push_str(&format!("  \"cycles\": {},\n", outcome.cycles));
    // Firmware time, from the clock the firmware programmed. Integer
    // microseconds: nanoseconds would imply a precision the quantised
    // step loop does not have.
    s.push_str(&format!(
        "  \"elapsed_us\": {},\n",
        outcome.elapsed_ns / 1_000
    ));
    s.push_str(&format!(
        "  \"pc\": {},\n",
        json_string(&format!("{:#010x}", outcome.pc))
    ));
    s.push_str(&format!(
        "  \"exception\": {},\n",
        match outcome.exception {
            Some(name) => json_string(name),
            None => "null".to_string(),
        }
    ));
    s.push_str(&format!(
        "  \"error\": {},\n",
        match &outcome.error {
            Some(msg) => json_string(msg),
            None => "null".to_string(),
        }
    ));
    s.push_str(&verdict.to_json());

    s.push_str("  \"unsupported_mmio\": [");
    if unsupported.is_empty() {
        s.push_str("],\n");
    } else {
        s.push('\n');
        for (i, (addr, pc, count)) in unsupported.iter().enumerate() {
            s.push_str(&format!(
                "    {{\"addr\": \"{addr:#010x}\", \"pc\": \"{pc:#010x}\", \"count\": {count}}}"
            ));
            if i + 1 < unsupported.len() {
                s.push(',');
            }
            s.push('\n');
        }
        s.push_str("  ],\n");
    }
    s.push_str(&format!(
        "  \"unsupported_mmio_truncated\": {unsupported_truncated},\n"
    ));

    if let Some(lcd) = lcd {
        s.push_str(&lcd.to_json());
    }
    if let Some(fb) = fb {
        s.push_str(&fb.to_json());
    }
    if let Some(psram) = psram {
        s.push_str(&psram.to_json());
    }
    if let Some(flash) = flash {
        s.push_str(&flash.to_json());
    }
    if let Some(sd) = sd {
        s.push_str(&sd.to_json());
    }
    if let Some(keyboard) = keyboard {
        s.push_str(&keyboard.to_json());
    }
    if let Some(pwm) = pwm {
        s.push_str(&pwm.to_json());
    }
    if let Some(audio_sink) = audio_sink {
        s.push_str(&audio_sink.to_json());
    }
    if let Some(pio) = pio {
        s.push_str(&pio.to_json());
    }
    if let Some((engine, file)) = scenario {
        s.push_str(&engine.to_json(&file, json_string));
    }

    s.push_str(&format!(
        "  \"uart\": {{\"bytes\": {}, \"sha256\": {}}}\n",
        outcome.uart_bytes.len(),
        json_string(uart_sha)
    ));
    s.push_str("}\n");
    s
}

#[cfg(feature = "behavior-trace")]
const BEHAVIOR_ARTIFACT_SCHEMA_VERSION: u32 = 1;

/// Build the path- and provenance-free, explicitly allow-listed value
/// whose canonical JSON bytes define `behavior_sha256`.
///
/// `serde_json` is built without `preserve_order`, so object keys are
/// stored in sorted maps. Serialising this value is therefore a stable
/// canonical encoding for this schema version; arrays retain event and
/// scenario order, which is behaviorally significant.
#[cfg(feature = "behavior-trace")]
fn behavior_projection(
    report: &str,
    scenario_sha256: Option<&str>,
    trace: &BehaviorTraceSnapshot,
) -> Result<serde_json::Value, String> {
    use serde_json::{Map, Value, json};

    let source: Value = serde_json::from_str(report)
        .map_err(|e| format!("parsing normal report for behavior projection: {e}"))?;
    let source = source
        .as_object()
        .ok_or_else(|| "normal report root is not an object".to_string())?;
    let mut out = Map::new();
    out.insert("projection_schema_version".into(), json!(1));
    for name in [
        "execution_model",
        "board",
        "lcd_variant",
        "boot",
        "step_quantum",
        "cycle_limit",
        "stop_pc",
        "stop_reason",
        "cycles",
        "elapsed_us",
        "pc",
        "exception",
        "error",
        "verdict",
        "verdict_reasons",
        "expectations",
        "unsupported_mmio",
        "unsupported_mmio_truncated",
        "lcd",
        "psram",
        "flash",
        "sd",
        "keyboard",
        "pwm",
        "pio",
        "uart",
    ] {
        if let Some(value) = source.get(name) {
            out.insert(name.into(), value.clone());
        }
    }
    if let Some(value) = source.get("firmware").and_then(Value::as_object) {
        out.insert("firmware".into(), json!({"sha256": value.get("sha256")}));
    }
    if let Some(value) = source.get("bootrom").and_then(Value::as_object) {
        out.insert(
            "bootrom".into(),
            json!({
                "sha256": value.get("sha256"),
                "executed": value.get("executed"),
            }),
        );
    }

    // Basenames and output PNG paths are provenance, not behavior.
    if let Some(value) = source.get("framebuffer").and_then(Value::as_object) {
        let mut framebuffer = Map::new();
        for name in ["width", "height", "rgb565_sha256", "non_black_pixels"] {
            if let Some(item) = value.get(name) {
                framebuffer.insert(name.into(), item.clone());
            }
        }
        out.insert("framebuffer".into(), Value::Object(framebuffer));
    }

    if let Some(value) = source.get("scenario").and_then(Value::as_object) {
        let mut scenario = Map::new();
        for name in [
            "name",
            "description",
            "status",
            "poll_ms",
            "steps_total",
            "error",
        ] {
            if let Some(item) = value.get(name) {
                scenario.insert(name.into(), item.clone());
            }
        }
        let steps = value
            .get("steps")
            .and_then(Value::as_array)
            .map(|items| {
                items
                    .iter()
                    .filter_map(Value::as_object)
                    .map(|item| {
                        let mut step = Map::new();
                        for name in [
                            "index",
                            "op",
                            "label",
                            "status",
                            "at_ms",
                            "at_cycles",
                            "detail",
                            "rgb565_sha256",
                        ] {
                            if let Some(value) = item.get(name) {
                                step.insert(name.into(), value.clone());
                            }
                        }
                        Value::Object(step)
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        scenario.insert("steps".into(), Value::Array(steps));
        scenario.insert("input_sha256".into(), json!(scenario_sha256));
        out.insert("scenario".into(), Value::Object(scenario));
    } else {
        out.insert("scenario_input_sha256".into(), json!(scenario_sha256));
    }

    out.insert("event_trace".into(), behavior_trace_json(trace));
    Ok(Value::Object(out))
}

#[cfg(feature = "behavior-trace")]
fn behavior_trace_json(trace: &BehaviorTraceSnapshot) -> serde_json::Value {
    use serde_json::json;
    json!({
        "schema_version": trace.schema_version,
        "canonical_encoding": "PICOEM-EVENT-v1",
        "streaming": true,
        "retains_event_array": false,
        "total_events": trace.total_events,
        "sha256": trace.sha256,
        "domains": trace.domains.iter().map(|value| json!({
            "name": value.domain.as_str(),
            "events": value.events,
            "sha256": value.sha256,
        })).collect::<Vec<_>>(),
    })
}

#[cfg(feature = "behavior-trace")]
fn build_behavior_artifact(
    report: &str,
    scenario_sha256: Option<&str>,
    trace: &BehaviorTraceSnapshot,
) -> Result<String, String> {
    use serde_json::json;

    let projection = behavior_projection(report, scenario_sha256, trace)?;
    let canonical = serde_json::to_vec(&projection)
        .map_err(|e| format!("serialising canonical behavior projection: {e}"))?;
    let behavior_sha256 = sha256_hex(&canonical);
    let artifact = json!({
        "schema_version": BEHAVIOR_ARTIFACT_SCHEMA_VERSION,
        "mode": "correctness_trace_on",
        "valid_for_wall_time": false,
        "backend_build": {
            "commit": BUILT_BACKEND_COMMIT,
            "dirty": built_backend_dirty(),
        },
        "normal_report_schema_version": SCHEMA_VERSION,
        "behavior_projection_encoding": "sorted-json-v1",
        "behavior_projection": projection,
        "behavior_sha256": behavior_sha256,
    });
    serde_json::to_string_pretty(&artifact)
        .map(|mut value| {
            value.push('\n');
            value
        })
        .map_err(|e| format!("serialising behavior artifact: {e}"))
}

/// Exit codes, matching `picocalc_emu`'s `tools/picocalc.py`: 0 pass,
/// 1 the run was judged and failed, 2 it could not be judged at all.
fn main() -> ExitCode {
    match run() {
        Ok(Verdict::Pass) => ExitCode::SUCCESS,
        Ok(Verdict::Fail) => ExitCode::from(1),
        Ok(Verdict::CannotJudge) => ExitCode::from(2),
        Err(e) => {
            eprintln!("picocalc-run: fatal: {e}");
            ExitCode::from(2)
        }
    }
}

/// Hash the framebuffer and, if asked, write the PNG. Only the PNG's
/// basename reaches the report — the JSON must carry no absolute paths.
fn build_framebuffer_report(
    fb: Option<&Framebuffer>,
    png_path: Option<&Path>,
) -> Result<Option<FramebufferReport>, String> {
    let Some(fb) = fb else { return Ok(None) };
    let png_basename = match png_path {
        Some(path) => {
            fb.write_png(path)
                .map_err(|e| format!("writing PNG {}: {e}", path.display()))?;
            Some(basename(path))
        }
        None => None,
    };
    Ok(Some(FramebufferReport {
        width: fb.width,
        height: fb.height,
        rgb565_sha256: fb.rgb565_sha256(),
        non_black_pixels: fb.non_black_pixels(),
        png_basename,
    }))
}

fn run() -> Result<Verdict, String> {
    let mut args = parse_args().inspect_err(|_| print_usage())?;

    if let Some(expected) = &args.expected_backend_commit {
        validate_backend_identity(expected, BUILT_BACKEND_COMMIT, built_backend_dirty())?;
    }

    let mut progress = match (args.run_id.clone(), args.progress_interval) {
        (Some(run_id), Some(interval)) => {
            let mut reporter = ProgressReporter::new(run_id, interval)?;
            reporter.start(args.cycles);
            Some(reporter)
        }
        (None, None) => None,
        _ => unreachable!("parse_args validates heartbeat option pairing"),
    };

    #[cfg(feature = "behavior-trace")]
    let scenario_sha256 = match &args.scenario {
        Some(path) => Some(sha256_hex(&std::fs::read(path).map_err(|e| {
            format!(
                "reading scenario {} for behavior identity: {e}",
                path.display()
            )
        })?)),
        None => None,
    };

    let scenario = match &args.scenario {
        Some(path) => Some(scenario::load(path)?),
        None => None,
    };
    if let Some(s) = &scenario {
        if s.needs_lcd() && args.board != Board::PicoCalc {
            return Err(format!(
                "scenario '{}' looks at the panel, which needs --board picocalc",
                s.name
            ));
        }
        // Same rule as --keys: asking for key events implies the
        // controller they arrive through.
        if s.needs_keyboard() {
            args.keyboard = true;
        }
    }

    let firmware = load_image(&args.bin, "firmware")?;
    let bootrom = load_image(&args.bootrom, "bootrom")?;
    let firmware_sha = sha256_hex(&firmware);
    let bootrom_sha = sha256_hex(&bootrom);

    // Explicit `--quantum` wins; then the `--stop-pc` single-step
    // requirement (correctness, not a preference); then `--psram`'s
    // stricter 1-cycle correctness requirement (see QUANTUM_PSRAM —
    // takes priority over the board default since it is the tighter
    // constraint whenever both are attached); then the board default;
    // then free-run.
    let step_quantum = match (args.quantum, args.stop_pc.is_some(), args.psram, args.board) {
        (Some(n), _, _, _) => n,
        (None, true, _, _) => QUANTUM_PC_WATCH,
        (None, false, true, _) => QUANTUM_PSRAM,
        (None, false, false, Board::PicoCalc) => QUANTUM_BOARD,
        (None, false, false, Board::None) => QUANTUM_FREE_RUN,
    };

    let keyboard = args.keyboard.then(|| {
        let kbd = Arc::new(Mutex::new(Keyboard::picocalc()));
        if let Some(keys) = args.keys.as_deref() {
            let mut guard = kbd.lock().expect("keyboard mutex");
            for ch in keys.chars() {
                // Only 8-bit codes cross the wire; the controller has no
                // encoding for anything wider.
                let code = u8::try_from(ch as u32).unwrap_or(b'?');
                guard.press_and_release(code);
            }
        }
        kbd
    });

    // The BSP mounts but does not format (FF_USE_MKFS=0), so the card
    // constructor supplies the selected pre-formatted volume. FAT32 is
    // the default; FAT16 is retained for compatibility targets.
    let sd_card = if args.sd {
        Some(Arc::new(Mutex::new(SdCard::new_with_format(
            picocalc_board::sdcard::DEFAULT_BLOCKS,
            args.sd_format,
        ))))
    } else if let Some(path) = args.sd_image.as_deref() {
        let card = SdCard::from_raw_file_with_format(path, args.sd_format)
            .map_err(|e| format!("opening SD RAW image {}: {e}", path.display()))?;
        Some(Arc::new(Mutex::new(card)))
    } else {
        None
    };

    let (mut emu, boot_mode, lcd) = boot(
        firmware,
        &bootrom,
        step_quantum,
        args.board,
        args.lcd_variant,
        args.psram,
        keyboard.clone(),
        sd_card.clone(),
    )?;
    emu.bus.unsupported_mmio_log_enabled = true;
    #[cfg(feature = "idle-profiler")]
    if args.idle_profile.is_some() {
        emu.enable_idle_profiler()
            .map_err(|e| format!("enabling idle profiler: {e}"))?;
    }
    #[cfg(feature = "behavior-trace")]
    if args.behavior_trace.is_some() {
        emu.enable_behavior_trace()
            .map_err(|e| format!("enabling behavior trace: {e}"))?;
        if args.board == Board::PicoCalc && args.lcd_variant == LcdVariant::B {
            emu.map_behavior_pio_domain(0, BehaviorEventDomain::Lcd);
        }
        if args.psram {
            emu.map_behavior_pio_domain(1, BehaviorEventDomain::Psram);
            emu.map_behavior_gpio_input_domain(BehaviorEventDomain::Psram);
        }
    }
    #[cfg(feature = "event-horizon-profiler")]
    if args.event_horizon_profile.is_some() && args.event_horizon_profile_after_uart.is_none() {
        emu.enable_running_event_profiler()
            .map_err(|e| format!("enabling running event-horizon profiler: {e}"))?;
    }
    if args.audio_wav.is_some() {
        emu.bus.enable_audio_pcm_capture();
    }

    let handles = BoardHandles {
        lcd: lcd.clone(),
        keyboard: keyboard.clone(),
        sd: sd_card.clone(),
    };
    let mut machine = MachineSession::new(emu, handles);
    #[cfg(feature = "event-horizon-profiler")]
    if let Some(marker) = args.event_horizon_profile_after_uart.clone() {
        machine.arm_event_profile_after_uart(marker);
    }
    if args.machine_api {
        run_machine_api(&mut machine, &args.snapshot_dir)?;
        return Ok(Verdict::Pass);
    }
    let mut engine = scenario.map(|s| scenario::Engine::new(s, args.snapshot_dir.clone()));

    let outcome = run_loop(
        &mut machine,
        args.cycles,
        args.stop_pc,
        engine.as_mut(),
        progress.as_mut(),
    );
    // Report generation still consumes the emulator after the shared
    // session ends. Moving it back out preserves the existing schema bytes.
    #[cfg(feature = "event-horizon-profiler")]
    let event_profile_start_cycle = machine.event_profile_start_cycle;
    let MachineSession { mut emu, .. } = machine;

    let flash_image = emu.bus.flash_image();
    if let Some(path) = args.flash_image_out.as_deref() {
        let input_canonical = std::fs::canonicalize(&args.bin).ok();
        let output_canonical = std::fs::canonicalize(path).ok();
        if input_canonical.is_some() && input_canonical == output_canonical {
            return Err("--flash-image-out must differ from --bin".to_string());
        }
        write_flash_image(path, &flash_image)?;
    }
    let flash_unknown_commands = emu.bus.flash_unknown_commands().to_vec();
    let flash_command_counts = emu.bus.flash_command_counts().to_vec();
    let flash_errors = emu.bus.flash_mutation_errors().to_vec();
    let flash_report = if args.flash_image_out.is_some()
        || emu.bus.flash_erase_count() != 0
        || emu.bus.flash_program_count() != 0
        || !flash_command_counts.is_empty()
        || !flash_unknown_commands.is_empty()
        || !flash_errors.is_empty()
    {
        Some(FlashReport {
            image_bytes: flash_image.len(),
            erase_count: emu.bus.flash_erase_count(),
            program_count: emu.bus.flash_program_count(),
            program_bytes: emu.bus.flash_program_bytes(),
            command_counts: flash_command_counts,
            unknown_commands: flash_unknown_commands,
            errors: flash_errors,
        })
    } else {
        None
    };

    #[cfg(feature = "idle-profiler")]
    if let Some(path) = &args.idle_profile {
        let snapshot = emu
            .idle_profile_snapshot()
            .expect("--idle-profile enabled the profiler before the run");
        let profile_report = build_idle_profile_report(
            BUILT_BACKEND_COMMIT,
            built_backend_dirty(),
            &basename(&args.bin),
            &firmware_sha,
            step_quantum,
            &outcome,
            &snapshot,
        );
        std::fs::write(path, profile_report.as_bytes())
            .map_err(|e| format!("writing idle profile {}: {e}", path.display()))?;
    }
    #[cfg(feature = "event-horizon-profiler")]
    if let Some(path) = &args.event_horizon_profile {
        let snapshot = emu.running_event_profile_snapshot().ok_or_else(|| {
            "--event-horizon-profile-after-uart marker was not observed before the run ended"
                .to_string()
        })?;
        let profile_report = build_running_event_profile_report_with_activation(
            BUILT_BACKEND_COMMIT,
            built_backend_dirty(),
            &basename(&args.bin),
            &firmware_sha,
            step_quantum,
            args.event_horizon_profile_after_uart.as_deref(),
            event_profile_start_cycle,
            &outcome,
            &snapshot,
        );
        std::fs::write(path, profile_report.as_bytes())
            .map_err(|e| format!("writing event-horizon profile {}: {e}", path.display()))?;
    }

    // A run that ended for its own reasons — cycle limit, HardFault —
    // leaves the scenario mid-step. Say so per step rather than letting
    // the report imply those steps passed.
    if let Some(e) = engine.as_mut() {
        e.finish_incomplete(&scenario::Observation {
            now_ns: outcome.elapsed_ns,
            cycles: outcome.cycles,
            lcd: None,
            keyboard: None,
            uart: &outcome.uart_bytes,
        });
    }

    // Snapshot the panel before anything else touches it.
    let (lcd_report, fb) = match &lcd {
        Some(lcd) => {
            let guard = lcd.lock().map_err(|_| "LCD model mutex poisoned")?;
            (Some(LcdReport::snapshot(&guard)), Some(guard.framebuffer()))
        }
        None => (None, None),
    };
    let fb_report = build_framebuffer_report(fb.as_ref(), args.fb_png.as_deref())?;

    #[cfg(feature = "behavior-trace")]
    if args.behavior_trace.is_some() {
        if let Some(engine) = engine.as_ref() {
            let mut payload = Vec::new();
            payload.extend_from_slice(&(engine.results().len() as u64).to_be_bytes());
            payload.extend_from_slice(engine.status().as_bytes());
            emu.record_behavior_event(BehaviorEventDomain::ScenarioInput, 2, &payload);
        }
        if let Some(lcd_report) = lcd_report.as_ref() {
            let mut payload = lcd_report.to_json().into_bytes();
            if let Some(framebuffer) = fb_report.as_ref() {
                payload.extend_from_slice(framebuffer.rgb565_sha256.as_bytes());
            }
            emu.record_behavior_event(BehaviorEventDomain::Lcd, 1, &payload);
        }
    }

    let psram_report = if args.psram {
        let psram = emu
            .bus
            .psram
            .as_ref()
            .expect("--psram implies bus.psram is Some");
        let verify = args
            .psram_verify_range
            .map(|(start, len)| verify_psram_range(&psram.buffer[..], start, len));
        Some(PsramReport {
            attached: true,
            tick_count: psram.tick_count,
            cs_falling_count: psram.cs_falling_count,
            bytes_written: psram.bytes_written,
            bytes_read: psram.bytes_read,
            cmd_write_count: psram.cmd_write_count,
            cmd_fast_read_count: psram.cmd_fast_read_count,
            cmd_reset_enable_count: psram.cmd_reset_enable_count,
            cmd_reset_count: psram.cmd_reset_count,
            cmd_unknown_count: psram.cmd_unknown_count,
            verify,
        })
    } else {
        None
    };

    let sd_report = sd_card.as_ref().map(|card| {
        let card = card.lock().expect("SD mutex");
        SdReport::snapshot(&card)
    });

    if let Some(path) = args.sd_image_out.as_deref() {
        let card = sd_card
            .as_ref()
            .ok_or_else(|| "--sd-image-out requires an attached RAW SD image".to_string())?;
        let mut card = card.lock().expect("SD mutex");
        card.export_raw(path)
            .map_err(|e| format!("exporting SD RAW image {}: {e}", path.display()))?;
    }

    let keyboard_report = keyboard.as_ref().map(|kbd| {
        let k = kbd.lock().expect("keyboard mutex");
        KeyboardReport {
            attached: true,
            addr: picocalc_board::keyboard::KEYBOARD_I2C_ADDR,
            reg_selects: k.reg_selects,
            key_events_delivered: k.key_events_delivered,
            key_events_remaining: k.queued(),
            key_events_dropped: k.key_events_dropped,
            key_events_overwritten: k.key_events_overwritten,
            battery_reads: k.battery_reads,
            backlight_writes: k.backlight_writes,
            backlight: k.backlight,
            lcd_backlight: k.lcd_backlight,
            config: k.config,
            interrupt_status: k.interrupt_status,
            caps_lock: k.caps_lock,
            num_lock: k.num_lock,
            unknown_reg_selects: k.unknown_reg_selects,
            unknown_reg_writes: k.unknown_reg_writes,
            last_unknown_reg: k.last_unknown_reg,
        }
    });

    // Always reported when a board model is attached: the official
    // sample configures PWM during init, and Gate 5 requires that to be
    // observable.
    let pwm_report = (args.board == Board::PicoCalc).then(|| PwmReport::collect(&emu.bus));

    // PWM/DMA audio observation is independent of the LCD board model. Keep
    // the report absent for ordinary board-less runs, but enable it whenever
    // an audio expectation or capture output was requested so an audio-only
    // run can omit the expensive PIO LCD observer. Screen assertions still
    // require `--board picocalc` in the scenario layer above.
    let audio_requested = args.expected_audio_sink_count.is_some()
        || args.audio_analysis.is_some()
        || args.audio_wav.is_some();
    let audio_sink_report = audio_requested.then(|| {
        AudioSinkReport::collect(
            &emu.bus,
            args.expected_audio_sink_count,
            args.expected_audio_sink_sha256.as_deref(),
        )
    });
    if let (Some(path), Some(audio_sink)) = (&args.audio_analysis, &audio_sink_report) {
        let analysis = audio_sink.analysis_json(
            BUILT_BACKEND_COMMIT,
            built_backend_dirty(),
            &basename(&args.bin),
            &firmware_sha,
        );
        std::fs::write(path, analysis.as_bytes())
            .map_err(|e| format!("writing audio analysis {}: {e}", path.display()))?;
    }
    if let Some(path) = &args.audio_wav {
        let samples = emu
            .bus
            .take_audio_pcm_capture()
            .expect("--audio-wav enabled PCM capture before the run");
        let sample_rate_hz = audio_sink_report
            .as_ref()
            .map_or(0, |report| report.snapshot.sample_rate_hz);
        write_audio_wav(path, &samples, sample_rate_hz)?;
    }

    let pio_report = (args.board == Board::PicoCalc).then(|| PioReport::collect(&mut emu.bus));

    let unsupported = emu.bus.unsupported_mmio_log();
    let unsupported_truncated = emu.bus.unsupported_mmio_log_truncated();
    let uart_sha = sha256_hex(&outcome.uart_bytes);

    if let Some(path) = &args.uart {
        std::fs::write(path, &outcome.uart_bytes)
            .map_err(|e| format!("writing UART log {}: {e}", path.display()))?;
    }

    let backend_commit = BUILT_BACKEND_COMMIT;

    let effective_expected_stop = args.expected_stop.or_else(|| {
        if args.stop_pc.is_some() {
            Some(StopReason::PcMatch)
        } else if engine.is_some() {
            Some(StopReason::ScenarioDone)
        } else {
            None
        }
    });
    let scenario_fault = engine.as_ref().is_some_and(|value| value.fault().is_some());
    let scenario_passed = engine.as_ref().map(|value| value.passed());
    let key_events_dropped = keyboard_report
        .as_ref()
        .map_or(0, |value| value.key_events_dropped);
    let keyboard_protocol_errors = keyboard_report.as_ref().map_or(0, |value| {
        value.unknown_reg_selects + value.unknown_reg_writes
    });
    let mut verdict = judge_run(
        &outcome,
        unsupported.len(),
        unsupported_truncated,
        key_events_dropped,
        keyboard_protocol_errors,
        scenario_passed,
        scenario_fault,
        effective_expected_stop,
        &args.expected_uart,
    );
    apply_audio_sink_expectation(&mut verdict, audio_sink_report.as_ref());

    let report = build_report(
        backend_commit,
        built_backend_dirty(),
        &basename(&args.bin),
        &firmware_sha,
        &basename(&args.bootrom),
        &bootrom_sha,
        boot_mode,
        step_quantum,
        args.cycles,
        args.stop_pc,
        &outcome,
        &unsupported,
        unsupported_truncated,
        &uart_sha,
        args.board,
        args.lcd_variant,
        lcd_report.as_ref(),
        fb_report.as_ref(),
        psram_report.as_ref(),
        flash_report.as_ref(),
        sd_report.as_ref(),
        keyboard_report.as_ref(),
        pwm_report.as_ref(),
        audio_sink_report.as_ref(),
        pio_report.as_ref(),
        engine.as_ref().map(|e| {
            (
                e,
                args.scenario.as_deref().map(basename).unwrap_or_default(),
            )
        }),
        &verdict,
    );

    #[cfg(feature = "behavior-trace")]
    if let Some(path) = &args.behavior_trace {
        let snapshot = emu
            .behavior_trace_snapshot()
            .expect("--behavior-trace enabled the tracer before the run");
        let artifact = build_behavior_artifact(&report, scenario_sha256.as_deref(), &snapshot)?;
        std::fs::write(path, artifact.as_bytes())
            .map_err(|e| format!("writing behavior trace {}: {e}", path.display()))?;
    }

    match &args.json {
        Some(path) => std::fs::write(path, report.as_bytes())
            .map_err(|e| format!("writing report {}: {e}", path.display()))?,
        None => print!("{report}"),
    }

    if let Some(engine) = engine {
        // The per-step lines go to stderr so a report on stdout stays
        // machine-readable when the two are piped apart.
        eprintln!("scenario '{}': {}", engine.name(), engine.status());
        for line in engine.summary_lines() {
            eprintln!("{line}");
        }
        if let Some(fault) = engine.fault() {
            eprintln!("scenario could not run: {fault}");
        }
    }
    // Dropped keys change what the firmware saw without changing what
    // the scenario said, so every step after the first drop is measuring
    // something other than the scripted input. Worth saying out loud
    // even when the run passed.
    if let Some(dropped) = keyboard_report
        .as_ref()
        .map(|k| k.key_events_dropped)
        .filter(|&n| n > 0)
    {
        eprintln!(
            "failure: the keyboard controller discarded {dropped} event(s) — input was \
             queued faster than the firmware drained it. Space the keys out with gap_ms; \
             the controller holds at most {} events.",
            picocalc_board::keyboard::MAX_QUEUED_EVENTS
        );
    }
    if let Some(reporter) = progress.as_mut() {
        reporter.finish(&outcome, verdict.status);
    }
    Ok(verdict.status)
}

#[cfg(test)]
mod tests {
    use super::{
        AudioSinkReport, BoardHandles, MachineApiState, MachineSession, RunOutcome, SdReport,
        StopReason, Verdict, apply_audio_sink_expectation, dispatch_machine_request,
        fatal_exception_name, json_escape, judge_run, run_loop, snapshot_machine,
        validate_backend_identity, validate_progress_interval, validate_run_id,
        validate_sd_selection, write_audio_wav,
    };
    use picocalc_board::{Keyboard, SdFormat, St7365p};
    use rp2040_emu::AudioSinkSnapshot;
    #[cfg(feature = "idle-profiler")]
    use rp2040_emu::IdleProfileSnapshot;
    use rp2040_emu::{Config, EmulatorBuilder};
    use std::path::Path;
    use std::sync::{Arc, Mutex};

    #[cfg(feature = "behavior-trace")]
    use super::behavior_projection;
    #[cfg(feature = "idle-profiler")]
    use super::build_idle_profile_report;
    #[cfg(feature = "event-horizon-profiler")]
    use super::build_running_event_profile_report;
    #[cfg(feature = "event-horizon-profiler")]
    use rp2040_emu::RunningEventProfileSnapshot;
    #[cfg(feature = "event-horizon-profiler")]
    use rp2040_emu::bus::UART0_BASE;
    #[cfg(feature = "behavior-trace")]
    use rp2040_emu::{BehaviorEventDomain, BehaviorTraceDomainSnapshot, BehaviorTraceSnapshot};

    #[cfg(feature = "behavior-trace")]
    fn test_trace() -> BehaviorTraceSnapshot {
        BehaviorTraceSnapshot {
            schema_version: 1,
            total_events: 1,
            sha256: "11".repeat(32),
            domains: vec![BehaviorTraceDomainSnapshot {
                domain: BehaviorEventDomain::Clock,
                events: 1,
                sha256: "22".repeat(32),
            }],
        }
    }

    #[cfg(feature = "behavior-trace")]
    #[test]
    fn behavior_projection_excludes_provenance_paths_and_backend() {
        let a = r#"{
            "backend_commit":"aaa",
            "backend_build":{"commit":"aaa","dirty":false},
            "firmware":{"basename":"a.bin","sha256":"f"},
            "bootrom":{"basename":"a.rom","sha256":"b","executed":false},
            "cycles":10,
            "framebuffer":{"width":1,"height":1,"rgb565_sha256":"c","non_black_pixels":1,"png":"a.png"},
            "scenario":{"file":"a.json","name":"s","status":"pass","steps":[{"index":0,"status":"pass","png_basename":"a.png"}]}
        }"#;
        let b = a
            .replace("\"aaa\"", "\"bbb\"")
            .replace("a.bin", "elsewhere.bin")
            .replace("a.rom", "elsewhere.rom")
            .replace("a.png", "elsewhere.png")
            .replace("a.json", "elsewhere.json");
        assert_eq!(
            behavior_projection(a, Some("input"), &test_trace()).unwrap(),
            behavior_projection(&b, Some("input"), &test_trace()).unwrap()
        );
    }

    #[cfg(feature = "behavior-trace")]
    #[test]
    fn behavior_projection_changes_with_behavior_and_scenario_identity() {
        let a = r#"{"firmware":{"sha256":"f"},"cycles":10}"#;
        let b = r#"{"firmware":{"sha256":"f"},"cycles":11}"#;
        assert_ne!(
            behavior_projection(a, Some("input-a"), &test_trace()).unwrap(),
            behavior_projection(b, Some("input-a"), &test_trace()).unwrap()
        );
        assert_ne!(
            behavior_projection(a, Some("input-a"), &test_trace()).unwrap(),
            behavior_projection(a, Some("input-b"), &test_trace()).unwrap()
        );
    }

    #[test]
    fn stop_reason_strings_are_stable() {
        assert_eq!(StopReason::CycleLimit.as_str(), "cycle_limit");
        assert_eq!(StopReason::PcMatch.as_str(), "pc_match");
        assert_eq!(StopReason::Exception.as_str(), "exception");
        assert_eq!(StopReason::Error.as_str(), "error");
    }

    #[test]
    fn run_id_validation_accepts_safe_identifiers() {
        assert_eq!(validate_run_id("mapper19-case.a_1:ok"), Ok(()));
        assert_eq!(validate_run_id(&"a".repeat(64)), Ok(()));
    }

    #[test]
    fn run_id_validation_rejects_ambiguous_or_injectable_identifiers() {
        let too_long = "a".repeat(65);
        for invalid in ["", "has space", "line\nfeed", "日本語"] {
            assert!(validate_run_id(invalid).is_err(), "accepted {invalid:?}");
        }
        assert!(validate_run_id(&too_long).is_err());
    }

    #[test]
    fn verdict_exit_codes_are_stable() {
        assert_eq!(Verdict::Pass.exit_code(), 0);
        assert_eq!(Verdict::Fail.exit_code(), 1);
        assert_eq!(Verdict::CannotJudge.exit_code(), 2);
    }

    #[test]
    fn progress_interval_validation_rejects_zero_and_clock_overflow() {
        assert!(validate_progress_interval(0).is_err());
        assert!(validate_progress_interval(1).is_ok());
        assert!(validate_progress_interval(u64::MAX).is_err());
    }

    fn outcome(stop_reason: StopReason, uart: &[u8]) -> RunOutcome {
        RunOutcome {
            stop_reason,
            cycles: 100,
            elapsed_ns: 1_000,
            pc: 0x1000_0100,
            exception: None,
            error: None,
            uart_bytes: uart.to_vec(),
        }
    }

    #[test]
    fn a_raw_cycle_limit_is_not_a_pass() {
        let result = judge_run(
            &outcome(StopReason::CycleLimit, b"ready"),
            0,
            false,
            0,
            0,
            None,
            false,
            None,
            &[],
        );
        assert!(result.status == Verdict::CannotJudge);
        assert_eq!(result.reasons, ["no_acceptance_criteria"]);
    }

    #[test]
    fn an_explicit_cycle_limit_and_present_markers_pass() {
        let result = judge_run(
            &outcome(StopReason::CycleLimit, b"boot lcd=pass ready"),
            0,
            false,
            0,
            0,
            None,
            false,
            Some(StopReason::CycleLimit),
            &["lcd=pass".to_string(), "ready".to_string()],
        );
        assert!(result.status == Verdict::Pass);
        assert!(result.reasons.is_empty());
        assert!(result.missing_uart_markers.is_empty());
    }

    #[test]
    fn a_marker_without_an_accepted_stop_cannot_pass() {
        let result = judge_run(
            &outcome(StopReason::CycleLimit, b"ready"),
            0,
            false,
            0,
            0,
            None,
            false,
            None,
            &["ready".to_string()],
        );
        assert!(result.status == Verdict::CannotJudge);
        assert_eq!(result.reasons, ["no_accepted_stop_reason"]);
    }

    #[test]
    fn missing_uart_marker_and_stop_mismatch_fail() {
        let result = judge_run(
            &outcome(StopReason::PcMatch, b"boot only"),
            0,
            false,
            0,
            0,
            None,
            false,
            Some(StopReason::CycleLimit),
            &["lcd=pass".to_string()],
        );
        assert!(result.status == Verdict::Fail);
        assert_eq!(
            result.reasons,
            ["stop_reason_mismatch", "missing_uart_markers"]
        );
    }

    #[test]
    fn unsafe_observations_always_fail() {
        let mut run = outcome(StopReason::Exception, b"ready");
        run.exception = Some("HardFault");
        let result = judge_run(
            &run,
            1,
            true,
            2,
            1,
            Some(true),
            false,
            Some(StopReason::Exception),
            &[],
        );
        assert!(result.status == Verdict::Fail);
        assert_eq!(
            result.reasons,
            [
                "exception",
                "unsupported_mmio",
                "unsupported_mmio_log_truncated",
                "keyboard_events_dropped",
                "keyboard_protocol_error"
            ]
        );
    }

    #[test]
    fn an_unrunnable_scenario_is_cannot_judge() {
        let result = judge_run(
            &outcome(StopReason::ScenarioDone, b""),
            0,
            false,
            0,
            0,
            Some(false),
            true,
            Some(StopReason::ScenarioDone),
            &["ready".to_string()],
        );
        assert!(result.status == Verdict::CannotJudge);
        assert_eq!(result.reasons, ["scenario_unrunnable"]);
    }

    #[test]
    fn a_failed_scenario_is_a_judged_failure() {
        let result = judge_run(
            &outcome(StopReason::CycleLimit, b""),
            0,
            false,
            0,
            0,
            Some(false),
            false,
            Some(StopReason::ScenarioDone),
            &[],
        );
        assert!(result.status == Verdict::Fail);
        assert_eq!(result.reasons, ["scenario_failed", "stop_reason_mismatch"]);
    }

    #[test]
    fn only_nmi_and_hardfault_are_fatal() {
        assert_eq!(fatal_exception_name(0, 0), None); // thread mode
        assert_eq!(fatal_exception_name(0, 2), Some("NMI"));
        assert_eq!(fatal_exception_name(0, 3), Some("HardFault"));
        assert_eq!(fatal_exception_name(1, 2), Some("core1 NMI"));
        assert_eq!(fatal_exception_name(1, 3), Some("core1 HardFault"));
        assert_eq!(fatal_exception_name(0, 11), None); // SVCall
        assert_eq!(fatal_exception_name(0, 15), None); // SysTick
        assert_eq!(fatal_exception_name(0, 16), None); // IRQ0
    }

    #[test]
    fn a_core1_hardfault_stops_the_run_fail_closed() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("serial emulator build");
        emu.cores[1].regs.xpsr = (emu.cores[1].regs.xpsr & !0x1ff) | 3;

        let mut machine = MachineSession::new(emu, BoardHandles::default());
        let run = run_loop(&mut machine, 100, None, None, None);

        assert!(run.stop_reason == StopReason::Exception);
        assert_eq!(run.exception, Some("core1 HardFault"));
        let verdict = judge_run(
            &run,
            0,
            false,
            0,
            0,
            None,
            false,
            Some(StopReason::CycleLimit),
            &[],
        );
        assert!(verdict.status == Verdict::Fail);
        assert_eq!(verdict.reasons, ["exception", "stop_reason_mismatch"]);
    }

    #[test]
    fn machine_api_rejects_unknown_fields_before_advancing() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("serial emulator build");
        let mut machine = MachineSession::new(emu, BoardHandles::default());
        let request = serde_json::json!({
            "schema": 1,
            "id": "r1",
            "op": "run",
            "max_cycles": 100,
            "max_cylces": 100
        });
        let header = super::machine_protocol::parse_request_header(&request).unwrap();
        let mut state = MachineApiState::default();
        let before = machine.cycles();
        let error =
            dispatch_machine_request(&mut machine, &mut state, &request, &header, Path::new("."))
                .expect_err("unknown field must fail closed");
        assert_eq!(
            error.code,
            super::machine_protocol::ErrorCode::InvalidRequest
        );
        assert_eq!(machine.cycles(), before);
    }

    #[test]
    fn machine_api_run_is_bounded_and_reports_actual_cycles() {
        let emu = EmulatorBuilder::new(Config::default())
            .step_quantum(1)
            .build()
            .expect("serial emulator build");
        let mut machine = MachineSession::new(emu, BoardHandles::default());
        let request = serde_json::json!({
            "schema": 1,
            "id": "bounded",
            "op": "run",
            "max_cycles": 1
        });
        let header = super::machine_protocol::parse_request_header(&request).unwrap();
        let (result, changed) = dispatch_machine_request(
            &mut machine,
            &mut MachineApiState::default(),
            &request,
            &header,
            Path::new("."),
        )
        .expect("bounded run must execute");
        assert!(changed);
        assert_eq!(result["reason"], "cycle_budget");
        assert!(result["advanced_cycles"].as_u64().unwrap() >= 1);
    }

    #[test]
    fn machine_api_input_makes_fifo_drop_explicit() {
        let keyboard = Arc::new(Mutex::new(Keyboard::picocalc()));
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("serial emulator build");
        let mut machine = MachineSession::new(
            emu,
            BoardHandles {
                keyboard: Some(keyboard),
                ..BoardHandles::default()
            },
        );
        let request = serde_json::json!({
            "schema": 1,
            "id": 2,
            "op": "input",
            "text": "abcdefghijklmnop"
        });
        let header = super::machine_protocol::parse_request_header(&request).unwrap();
        let (result, changed) = dispatch_machine_request(
            &mut machine,
            &mut MachineApiState::default(),
            &request,
            &header,
            Path::new("."),
        )
        .expect("valid input command");
        assert!(changed);
        assert_eq!(result["status"], "dropped");
        assert_eq!(result["dropped"], 1);
        assert_eq!(result["queued"], 31);
    }

    #[test]
    fn machine_api_snapshot_cannot_escape_its_output_directory() {
        let lcd = Arc::new(Mutex::new(St7365p::new()));
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("serial emulator build");
        let machine = MachineSession::new(
            emu,
            BoardHandles {
                lcd: Some(lcd),
                ..BoardHandles::default()
            },
        );
        for name in ["../escape.png", "/tmp/escape.png", "nested/escape.png"] {
            let request = serde_json::json!({"png": name});
            let error = snapshot_machine(&machine, &request, Path::new("safe"))
                .expect_err("path escape must be rejected");
            assert_eq!(
                error.code,
                super::machine_protocol::ErrorCode::InvalidRequest
            );
        }
    }

    #[test]
    fn json_escaping_covers_the_dangerous_bytes() {
        assert_eq!(json_escape(r#"a"b\c"#), r#"a\"b\\c"#);
        assert_eq!(json_escape("l1\nl2\ttab\r"), "l1\\nl2\\ttab\\r");
        assert_eq!(json_escape("\u{1}"), "\\u0001");
        assert_eq!(json_escape("plain/path-ok"), "plain/path-ok");
    }

    #[test]
    fn an_explicit_sd_format_requires_an_attached_card() {
        assert_eq!(validate_sd_selection(true, None, None, true), Ok(()));
        assert_eq!(validate_sd_selection(true, None, None, false), Ok(()));
        assert_eq!(validate_sd_selection(false, None, None, false), Ok(()));
        assert_eq!(
            validate_sd_selection(false, None, None, true),
            Err("--sd-format requires --sd".to_string())
        );
    }

    #[test]
    fn sd_report_names_the_selected_format() {
        let report = SdReport {
            format: SdFormat::Fat32,
            block_count: 131_072,
            commands_seen: 7,
            blocks_read: 2,
            blocks_written: 1,
            unknown_commands: Vec::new(),
            raw: None,
        }
        .to_json();
        assert!(report.contains("\"format\": \"fat32\""));
        assert!(report.contains("\"block_size\": 512"));
        assert!(report.contains("\"blocks_written\": 1"));
    }

    #[test]
    fn audio_sink_report_matches_only_on_exact_count_and_sha() {
        let mut snapshot = AudioSinkSnapshot {
            status: "pass",
            dma_write_count: 3,
            target_write_attempt_count: 3,
            other_pwm_cc_write_count: 0,
            wrong_width_count: 0,
            wrong_treq_count: 0,
            missing_due_cycle_count: 0,
            pcm_sha256: "aabbccdd".repeat(8),
            first_words: Vec::new(),
            last_words: Vec::new(),
            timer_index: 0,
            treq: 59,
            timer_fraction_x: 3,
            timer_fraction_y: 15625,
            timer_event_count: 0,
            timer_miss_count: 0,
            timer_miss_audio_not_busy: 0,
            timer_miss_other_dma_selected: 0,
            timer_miss_no_dma_selected: 0,
            timer_miss_multiple_due_in_window: 0,
            timer_due_cycle_sha256: String::new(),
            block_start_count: 1,
            block_frame_min: Some(3),
            block_frame_max: Some(3),
            malformed_block_count: 0,
            block_boundary_gap_count: 0,
            block_boundary_gap_min_cycles: None,
            block_boundary_gap_max_cycles: None,
            block_boundary_gap_sha256: String::new(),
            gap_5208_count: 0,
            gap_5209_count: 0,
            unexpected_gap_count: 0,
            service_latency_min_cycles: None,
            service_latency_max_cycles: None,
            service_latency_sha256: String::new(),
            analysis_frame_count: 3,
            sample_rate_hz: 48_000,
            channel_count: 2,
            reconstructed_pcm_format: "stereo_s16le_from_pwm8_duty",
            analysis_window_frames: 1024,
            active_abs_threshold: 512,
            peak_abs_left: 32_768,
            peak_abs_right: 32_767,
            stream_rms: 12_000,
            max_window_rms: 16_000,
            dc_offset_left: 0,
            dc_offset_right: 0,
            active_frame_count: 3,
            active_frame_ratio_ppm: 1_000_000,
            rail_sample_count: 1,
            rail_sample_ratio_ppm: 166_666,
            max_consecutive_rail_frames: 1,
            out_of_range_duty_sample_count: 0,
        };

        assert_eq!(
            AudioSinkReport::status_for_expected(
                &snapshot,
                Some(3),
                Some("aabbccdd".repeat(8).as_str())
            ),
            "pass"
        );

        snapshot.dma_write_count = 2;
        assert_eq!(
            AudioSinkReport::status_for_expected(
                &snapshot,
                Some(3),
                Some("aabbccdd".repeat(8).as_str())
            ),
            "fail"
        );

        snapshot.dma_write_count = 3;
        assert_eq!(
            AudioSinkReport::status_for_expected(
                &snapshot,
                Some(3),
                Some("00112233".repeat(8).as_str())
            ),
            "fail"
        );

        assert_eq!(
            AudioSinkReport::status_for_expected(&snapshot, None, None),
            "pass"
        );

        let report = AudioSinkReport {
            snapshot: snapshot.clone(),
            expected_count: None,
            expected_sha256: None,
            status: "pass",
        };
        let frozen_analysis =
            report.analysis_json("a".repeat(40).as_str(), false, "app.bin", &"b".repeat(64));
        assert!(frozen_analysis.contains("\"schema_version\": 1"));
        assert!(frozen_analysis.contains("\"sample_rate_hz\": 48000"));

        let mut parameterized = snapshot.clone();
        parameterized.sample_rate_hz = 22_050;
        let report = AudioSinkReport {
            snapshot: parameterized,
            expected_count: None,
            expected_sha256: None,
            status: "pass",
        };
        let parameterized_analysis =
            report.analysis_json("a".repeat(40).as_str(), false, "app.bin", &"b".repeat(64));
        assert!(parameterized_analysis.contains("\"schema_version\": 2"));
        assert!(parameterized_analysis.contains("\"sample_rate_hz\": 22050"));

        assert_eq!(
            AudioSinkReport::status_for_expected(&snapshot, Some(3), None),
            "fail"
        );
    }

    #[test]
    fn audio_wav_header_uses_the_observed_sample_rate() {
        let path = std::env::temp_dir().join(format!(
            "picocalc-audio-wav-rate-{}-{}.wav",
            std::process::id(),
            22_050
        ));
        write_audio_wav(&path, &[0, 0, 1, -1], 22_050).expect("WAV write");
        let bytes = std::fs::read(&path).expect("WAV read");
        assert_eq!(
            u32::from_le_bytes(bytes[24..28].try_into().unwrap()),
            22_050
        );
        assert_eq!(
            u32::from_le_bytes(bytes[28..32].try_into().unwrap()),
            22_050 * 4
        );
        std::fs::remove_file(path).expect("WAV cleanup");
    }

    #[test]
    fn audio_sink_expectation_failure_forces_verdict_fail() {
        let mut verdict = super::VerdictReport {
            status: Verdict::Pass,
            reasons: Vec::new(),
            expected_stop: Some(StopReason::CycleLimit),
            required_uart_markers: Vec::new(),
            missing_uart_markers: Vec::new(),
        };

        let report = AudioSinkReport {
            snapshot: AudioSinkSnapshot {
                status: "pass",
                dma_write_count: 3,
                target_write_attempt_count: 3,
                other_pwm_cc_write_count: 0,
                wrong_width_count: 0,
                wrong_treq_count: 0,
                missing_due_cycle_count: 0,
                pcm_sha256: "aabbccdd".repeat(8),
                first_words: Vec::new(),
                last_words: Vec::new(),
                timer_index: 0,
                treq: 59,
                timer_fraction_x: 3,
                timer_fraction_y: 15625,
                timer_event_count: 0,
                timer_miss_count: 0,
                timer_miss_audio_not_busy: 0,
                timer_miss_other_dma_selected: 0,
                timer_miss_no_dma_selected: 0,
                timer_miss_multiple_due_in_window: 0,
                timer_due_cycle_sha256: String::new(),
                block_start_count: 1,
                block_frame_min: Some(3),
                block_frame_max: Some(3),
                malformed_block_count: 0,
                block_boundary_gap_count: 0,
                block_boundary_gap_min_cycles: None,
                block_boundary_gap_max_cycles: None,
                block_boundary_gap_sha256: String::new(),
                gap_5208_count: 0,
                gap_5209_count: 0,
                unexpected_gap_count: 0,
                service_latency_min_cycles: None,
                service_latency_max_cycles: None,
                service_latency_sha256: String::new(),
                analysis_frame_count: 3,
                sample_rate_hz: 48_000,
                channel_count: 2,
                reconstructed_pcm_format: "stereo_s16le_from_pwm8_duty",
                analysis_window_frames: 1024,
                active_abs_threshold: 512,
                peak_abs_left: 32_768,
                peak_abs_right: 32_767,
                stream_rms: 12_000,
                max_window_rms: 16_000,
                dc_offset_left: 0,
                dc_offset_right: 0,
                active_frame_count: 3,
                active_frame_ratio_ppm: 1_000_000,
                rail_sample_count: 1,
                rail_sample_ratio_ppm: 166_666,
                max_consecutive_rail_frames: 1,
                out_of_range_duty_sample_count: 0,
            },
            expected_count: Some(3),
            expected_sha256: Some("aabbccdd".repeat(8)),
            status: "fail",
        };

        apply_audio_sink_expectation(&mut verdict, Some(&report));
        assert!(verdict.status == Verdict::Fail);
        assert_eq!(verdict.reasons, ["audio_sink_mismatch"]);

        let mut pass_verdict = verdict;
        let pass_report = AudioSinkReport {
            status: "pass",
            ..report
        };
        apply_audio_sink_expectation(&mut pass_verdict, Some(&pass_report));
        assert!(pass_verdict.status == Verdict::Fail);
        assert_eq!(pass_verdict.reasons, ["audio_sink_mismatch"]);
    }

    #[test]
    fn backend_identity_must_match_the_clean_compiled_source() {
        let expected = "0123456789012345678901234567890123456789";
        assert_eq!(validate_backend_identity(expected, expected, false), Ok(()));
        assert!(
            validate_backend_identity(expected, "wrong", false)
                .unwrap_err()
                .contains("was required")
        );
        assert!(
            validate_backend_identity(expected, expected, true)
                .unwrap_err()
                .contains("dirty")
        );
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_profile_report_is_deterministic_valid_json() {
        let mut profile = IdleProfileSnapshot {
            step_calls: 11,
            total_master_cycles: 100,
            both_blocked_cycles: 80,
            proven_safe_cycles: 64,
            ..IdleProfileSnapshot::default()
        };
        profile.blockers.pwm = 16;
        profile.blocker_episodes.pwm = 1;
        profile.stationary_sources.uart = 80;
        profile.stationary_source_episodes.uart = 2;
        profile.exact_bulk_sources.pwm = 64;
        profile.exact_bulk_source_episodes.pwm = 1;
        profile.blocked_lengths.episodes_ge[0] = 2;
        profile.blocked_lengths.cycle_mass_ge[0] = 80;
        profile.event_bounded_safe_lengths.episodes_ge[0] = 3;
        profile.event_bounded_safe_lengths.cycle_mass_ge[0] = 64;
        profile.horizon_boundary_events.pwm = 2;
        let run = outcome(StopReason::ScenarioDone, b"");
        let report = build_idle_profile_report(
            "0123456789012345678901234567890123456789",
            false,
            "firmware.bin",
            "abcdef",
            1,
            &run,
            &profile,
        );
        let parsed: serde_json::Value = serde_json::from_str(&report).unwrap();
        assert_eq!(parsed["kind"], "rp2040_serial_idle_profile");
        assert_eq!(parsed["instrumented"], true);
        assert_eq!(parsed["valid_for_wall_time"], false);
        assert_eq!(parsed["counters"]["both_blocked_cycles"], 80);
        assert_eq!(parsed["blocker_cycles"]["pwm"], 16);
        assert_eq!(parsed["blocker_episodes"]["pwm"], 1);
        assert_eq!(parsed["stationary_source_cycles"]["uart"], 80);
        assert_eq!(parsed["stationary_source_episodes"]["uart"], 2);
        assert_eq!(parsed["exact_bulk_source_cycles"]["pwm"], 64);
        assert_eq!(parsed["exact_bulk_source_episodes"]["pwm"], 1);
        assert_eq!(parsed["event_bounded_safe_lengths"]["cycle_mass_ge"][0], 64);
        assert_eq!(parsed["horizon_boundary_events"]["pwm"], 2);
        assert_eq!(
            parsed["histogram_thresholds_cycles"]
                .as_array()
                .unwrap()
                .len(),
            64
        );
        assert_eq!(
            report,
            build_idle_profile_report(
                "0123456789012345678901234567890123456789",
                false,
                "firmware.bin",
                "abcdef",
                1,
                &run,
                &profile,
            )
        );
    }

    #[cfg(feature = "event-horizon-profiler")]
    #[test]
    fn running_event_profile_report_is_deterministic_valid_json() {
        let mut profile = RunningEventProfileSnapshot::default();
        profile.boundary.running_steps = 7;
        profile.boundary.total_running_cycles = 12;
        profile.boundary.boundary_steps = 3;
        profile.boundary.candidate_dispatches = 5;
        profile.boundary.candidate_cycles = 9;
        profile.boundary.boundary_events.cpu_mmio = 2;
        profile.boundary.one_cycle_fallback_cycles.pio = 8;
        profile.boundary.one_cycle_fallback_signatures.steps[1] = 4;
        profile.boundary.one_cycle_fallback_signatures.cycle_mass[1] = 8;
        profile.boundary.observed_candidate_dispatches.episodes_ge[1] = 1;
        profile.boundary.observed_candidate_dispatches.cycle_mass_ge[1] = 5;
        profile.decode_by_core[0].cacheable_hits = 11;
        profile.decode_by_core[0].cacheable_misses = 2;
        profile.decode_by_core[0].noncacheable_fetches = 3;
        profile.decode_by_core[0].cacheable_hits_narrow = 4;
        profile.decode_by_core[0].cacheable_hits_wide = 5;
        profile.decode_by_core[0].lookup_hits_by_region.rom = 6;
        profile.decode_by_core[0]
            .lookup_hits_by_region
            .immutable_xip_flash_aliases = 7;
        profile.decode_by_core[0].lookup_hits_by_region.xip_sram = 8;
        profile.decode_by_core[0].lookup_hits_by_region.sram = 9;
        profile.decode_by_core[0].lookup_hits_by_region.other = 10;
        profile.decode_by_core[0].lookup_misses_by_region.rom = 1;
        profile.decode_by_core[0]
            .lookup_misses_by_region
            .immutable_xip_flash_aliases = 2;
        profile.decode_by_core[0].lookup_misses_by_region.xip_sram = 3;
        profile.decode_by_core[0].lookup_misses_by_region.sram = 4;
        profile.decode_by_core[0].lookup_misses_by_region.other = 5;
        profile.decode_by_core[0]
            .sequential_cache_hit_runs
            .episodes_ge[2] = 3;
        profile.decode_by_core[0].immutable_xip_hit_runs.episodes_ge[1] = 2;
        profile.decode_by_core[0]
            .immutable_xip_hit_run_termination_counters
            .post_execute_next_pc_redirect = 11;
        profile.decode_by_core[0]
            .immutable_xip_hit_run_termination_counters
            .xip_miss = 12;
        profile.decode_by_core[0]
            .immutable_xip_hit_run_termination_counters
            .region_exit = 13;
        profile.decode_by_core[0]
            .immutable_xip_hit_run_termination_counters
            .prefetch_exception = 14;
        profile.decode_by_core[0]
            .immutable_xip_hit_run_termination_counters
            .fault = 15;
        profile.decode_by_core[0]
            .decode_cache_invalidation_observations
            .entry_address_count = 16;
        profile.decode_by_core[0]
            .decode_cache_invalidation_observations
            .rom = 17;
        profile.decode_by_core[0]
            .decode_cache_invalidation_observations
            .xip = 18;
        profile.decode_by_core[0]
            .decode_cache_invalidation_observations
            .sram = 19;
        profile.decode_by_core[0]
            .decode_cache_invalidation_observations
            .bulk = 20;
        profile.decode_by_core[0]
            .decode_cache_invalidation_observations
            .all = 21;
        let run = outcome(StopReason::ScenarioDone, b"");
        let report = build_running_event_profile_report(
            "0123456789012345678901234567890123456789",
            false,
            "firmware.bin",
            "abcdef",
            1,
            &run,
            &profile,
        );
        let parsed: serde_json::Value = serde_json::from_str(&report).unwrap();
        assert_eq!(
            parsed["kind"],
            "rp2040_serial_running_event_horizon_profile"
        );
        assert_eq!(parsed["schema_version"], 3);
        assert_eq!(parsed["observed_gaps_are_safe_windows"], false);
        assert_eq!(parsed["fallback_occupancy_is_safe_window"], false);
        assert_eq!(parsed["decode_hit_runs_are_speedup_prediction"], false);
        assert_eq!(
            parsed["immutable_xip_hit_runs_are_speedup_prediction"],
            false
        );
        assert_eq!(parsed["counters"]["running_steps"], 7);
        assert_eq!(parsed["counters"]["candidate_dispatches"], 5);
        assert_eq!(parsed["counters"]["candidate_cycles"], 9);
        assert_eq!(parsed["boundary_events"]["cpu_mmio"], 2);
        assert_eq!(parsed["one_cycle_fallback_cycles"]["pio"], 8);
        assert_eq!(parsed["one_cycle_fallback_signatures"]["steps"][1], 4);
        assert_eq!(parsed["one_cycle_fallback_signatures"]["cycle_mass"][1], 8);
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["cacheable_hits"],
            11
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["cacheable_misses"],
            2
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["noncacheable_fetches"],
            3
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["cacheable_hits_narrow"],
            4
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["cacheable_hits_wide"],
            5
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_hits_by_region"]["rom"],
            6
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_hits_by_region"]["immutable_xip_flash_aliases"],
            7
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_hits_by_region"]["xip_sram"],
            8
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_hits_by_region"]["sram"],
            9
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_hits_by_region"]["other"],
            10
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_misses_by_region"]["rom"],
            1
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_misses_by_region"]["immutable_xip_flash_aliases"],
            2
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_misses_by_region"]["xip_sram"],
            3
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_misses_by_region"]["sram"],
            4
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["lookup_misses_by_region"]["other"],
            5
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["sequential_cache_hit_runs"]["episodes_ge"][2],
            3
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["immutable_xip_hit_runs"]["episodes_ge"][1],
            2
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["immutable_xip_hit_run_termination_counters"]["post_execute_next_pc_redirect"],
            11
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["immutable_xip_hit_run_termination_counters"]["xip_miss"],
            12
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["immutable_xip_hit_run_termination_counters"]["region_exit"],
            13
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["immutable_xip_hit_run_termination_counters"]["prefetch_exception"],
            14
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["immutable_xip_hit_run_termination_counters"]["fault"],
            15
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["decode_cache_invalidation_observations"]["entry_address_count"],
            16
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["decode_cache_invalidation_observations"]["rom"],
            17
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["decode_cache_invalidation_observations"]["xip"],
            18
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["decode_cache_invalidation_observations"]["sram"],
            19
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["decode_cache_invalidation_observations"]["bulk"],
            20
        );
        assert_eq!(
            parsed["decode_opportunity_by_core"][0]["decode_cache_invalidation_observations"]["all"],
            21
        );
        assert_eq!(
            parsed["observed_candidate_dispatches"]["cycle_mass_ge"][1],
            5
        );
        assert_eq!(
            report,
            build_running_event_profile_report(
                "0123456789012345678901234567890123456789",
                false,
                "firmware.bin",
                "abcdef",
                1,
                &run,
                &profile,
            )
        );
        let report2 = build_running_event_profile_report(
            "0123456789012345678901234567890123456789",
            false,
            "firmware.bin",
            "abcdef",
            1,
            &run,
            &profile,
        );
        assert_eq!(report.as_bytes(), report2.as_bytes());
    }

    #[cfg(feature = "event-horizon-profiler")]
    #[test]
    fn deferred_event_marker_is_recognised_across_uart_drains() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("serial emulator build");
        let mut machine = MachineSession::new(emu, BoardHandles::default());
        machine.arm_event_profile_after_uart("READY".to_string());
        // The UART log is drained by the same method used by the run loop.
        // Write the marker in two separate batches so no single drain sees
        // the complete marker; the persistent accumulation must still match.
        machine.emu.bus.write32(0x4000_f000, 1 << 22); // release UART0
        machine.emu.bus.write32(UART0_BASE + 0x30, 0x101); // UARTEN | TXE
        for byte in b"REA" {
            machine.emu.bus.write8(UART0_BASE, *byte);
        }
        machine.drain_uart();
        assert!(machine.event_profile_after_uart.is_some());
        assert!(machine.event_profile_start_cycle.is_none());

        for byte in b"DY" {
            machine.emu.bus.write8(UART0_BASE, *byte);
        }
        let drain_cycle = machine.cycles();
        machine.drain_uart();
        assert!(machine.event_profile_after_uart.is_none());
        assert_eq!(machine.event_profile_start_cycle, Some(drain_cycle));
        assert!(machine.emu.running_event_profile_snapshot().is_some());
    }

    #[cfg(feature = "event-horizon-profiler")]
    #[test]
    fn deferred_event_marker_remains_unactivated_at_run_boundary() {
        for stop_reason in [StopReason::ScenarioDone, StopReason::CycleLimit] {
            let emu = EmulatorBuilder::new(Config::default())
                .build()
                .expect("serial emulator build");
            let mut machine = MachineSession::new(emu, BoardHandles::default());
            machine.arm_event_profile_after_uart("NEVER".to_string());
            let _outcome = machine.finish(stop_reason, None, None);
            assert!(machine.event_profile_after_uart.is_some());
            assert!(machine.event_profile_start_cycle.is_none());
            assert!(machine.emu.running_event_profile_snapshot().is_none());
        }
    }
}
