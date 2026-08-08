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

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::{Arc, Mutex};

use picocalc_board::sha256::sha256_hex;
use picocalc_board::{
    Framebuffer, Keyboard, KeyboardWire, LcdPioWire, SdCard, SdCardWire, SdFormat, St7365p,
    St7365pWire, pins,
};
#[cfg(feature = "behavior-trace")]
use rp2040_emu::{BehaviorEventDomain, BehaviorTraceSnapshot};
use rp2040_emu::{Config, Emulator, EmulatorBuilder};
#[cfg(feature = "idle-profiler")]
use rp2040_emu::{
    CumulativeHistogramSnapshot, IDLE_HISTOGRAM_BUCKETS, IDLE_PROFILE_SCHEMA_VERSION,
    IdleBlockerCycles, IdleBlockerEpisodes, IdleHorizonEvents, IdleProfileSnapshot,
};

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
/// Batch only the harness dispatch loop; core/single-quantum execution
/// semantics remain unchanged (hardware quantum stays 1 for this path).
const EXACT_DISPATCH_BATCH_CYCLES: u64 = 64;

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
    expected_backend_commit: Option<String>,
    board: Board,
    lcd_variant: LcdVariant,
    fb_png: Option<PathBuf>,
    quantum: Option<u32>,
    psram: bool,
    psram_verify_range: Option<(u32, u32)>,
    keyboard: bool,
    sd: bool,
    sd_format: SdFormat,
    keys: Option<String>,
    scenario: Option<PathBuf>,
    snapshot_dir: PathBuf,
    expected_stop: Option<StopReason>,
    expected_uart: Vec<String>,
    #[cfg(feature = "idle-profiler")]
    idle_profile: Option<PathBuf>,
    #[cfg(feature = "behavior-trace")]
    behavior_trace: Option<PathBuf>,
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

fn validate_sd_selection(sd: bool, format_explicit: bool) -> Result<(), String> {
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
         --expect-stop <reason>   Required stop: cycle_limit, pc_match, or scenario_done.\n\
         --expect-uart <text>     Required UART substring. Repeat for each marker.\n\
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
}

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut bin: Option<PathBuf> = None;
    let mut bootrom: Option<PathBuf> = None;
    let mut cycles: Option<u64> = None;
    let mut stop_pc: Option<u32> = None;
    let mut json: Option<PathBuf> = None;
    let mut uart: Option<PathBuf> = None;
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
    let mut sd_format = SdFormat::default();
    let mut sd_format_explicit = false;
    let mut keys: Option<String> = None;
    let mut scenario: Option<PathBuf> = None;
    let mut snapshot_dir: Option<PathBuf> = None;
    let mut expected_stop: Option<StopReason> = None;
    let mut expected_uart: Vec<String> = Vec::new();
    #[cfg(feature = "idle-profiler")]
    let mut idle_profile: Option<PathBuf> = None;
    #[cfg(feature = "behavior-trace")]
    let mut behavior_trace: Option<PathBuf> = None;

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
            "--sd-format" => {
                let raw = value("--sd-format")?;
                sd_format = raw.parse::<SdFormat>()?;
                sd_format_explicit = true;
            }
            "--keys" => keys = Some(value("--keys")?),
            "--scenario" => scenario = Some(PathBuf::from(value("--scenario")?)),
            "--snapshot-dir" => snapshot_dir = Some(PathBuf::from(value("--snapshot-dir")?)),
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
            #[cfg(feature = "idle-profiler")]
            "--idle-profile" => idle_profile = Some(PathBuf::from(value("--idle-profile")?)),
            #[cfg(feature = "behavior-trace")]
            "--behavior-trace" => behavior_trace = Some(PathBuf::from(value("--behavior-trace")?)),
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
    validate_sd_selection(sd, sd_format_explicit)?;
    // Queueing keys implies the controller they arrive through.
    if keys.is_some() {
        keyboard = true;
    }
    if snapshot_dir.is_some() && scenario.is_none() {
        return Err("--snapshot-dir only means anything with --scenario".to_string());
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
    #[cfg(all(feature = "idle-profiler", feature = "behavior-trace"))]
    if idle_profile.is_some() && behavior_trace.is_some() {
        return Err(
            "--idle-profile and --behavior-trace are separate diagnostic modes".to_string(),
        );
    }

    Ok(Args {
        bin: bin.ok_or_else(|| "missing required --bin <path>".to_string())?,
        bootrom: bootrom.unwrap_or_else(|| PathBuf::from(DEFAULT_BOOTROM_PATH)),
        cycles: cycles.unwrap_or(DEFAULT_CYCLE_LIMIT),
        stop_pc,
        json,
        uart,
        expected_backend_commit,
        board,
        lcd_variant,
        fb_png,
        quantum,
        psram,
        psram_verify_range,
        keyboard,
        sd,
        sd_format,
        keys,
        scenario,
        snapshot_dir: snapshot_dir.unwrap_or_else(|| PathBuf::from(".")),
        expected_stop,
        expected_uart,
        #[cfg(feature = "idle-profiler")]
        idle_profile,
        #[cfg(feature = "behavior-trace")]
        behavior_trace,
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

/// ARMv6-M IPSR exception numbers that mean "the firmware has fallen
/// over". Ordinary IRQs (>= 16), SVCall, PendSV and SysTick are normal
/// operation and must not stop the run.
fn fatal_exception_name(ipsr: u32) -> Option<&'static str> {
    match ipsr {
        2 => Some("NMI"),
        3 => Some("HardFault"),
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
}

fn run_loop(
    emu: &mut Emulator,
    cycle_limit: u64,
    stop_pc: Option<u32>,
    mut engine: Option<&mut scenario::Engine>,
    board: &BoardHandles,
) -> RunOutcome {
    let mut uart_bytes: Vec<u8> = Vec::new();
    let mut steps: u64 = 0;

    let mut vclock = VirtualClock::new(emu.bus.clock_tree.sys_clk_hz);
    // Comparing cycles rather than converting to nanoseconds keeps the
    // per-step check to one integer compare; the division only happens
    // at a poll or a clock change.
    let mut next_poll_cycles = match engine.as_deref() {
        Some(e) => vclock.cycles_at(e.next_poll_ns()),
        None => u64::MAX,
    };

    let finish = |emu: &mut Emulator,
                  vclock: &VirtualClock,
                  uart_bytes: &mut Vec<u8>,
                  stop_reason: StopReason,
                  exception: Option<&'static str>,
                  error: Option<String>| {
        uart_bytes.extend_from_slice(&emu.drain_uart0_tx_log());
        RunOutcome {
            stop_reason,
            cycles: emu.clock.cycles,
            elapsed_ns: vclock.ns_at(emu.clock.cycles),
            pc: emu.cores[0].regs.pc(),
            exception,
            error,
            uart_bytes: std::mem::take(uart_bytes),
        }
    };

    loop {
        // Pre-step observations. Checking before the first step means a
        // `--stop-pc` equal to the reset vector matches immediately, and
        // that a scenario's first step sees the machine at reset.
        if let Some(e) = engine.as_deref_mut()
            && emu.clock.cycles >= next_poll_cycles
        {
            // The engine may test the UART stream, so it must see every
            // byte sent so far — not just those the periodic drain has
            // collected.
            uart_bytes.extend_from_slice(&emu.drain_uart0_tx_log());
            e.poll(&scenario::Observation {
                now_ns: vclock.ns_at(emu.clock.cycles),
                cycles: emu.clock.cycles,
                lcd: board.lcd.as_deref(),
                keyboard: board.keyboard.as_deref(),
                uart: &uart_bytes,
            });
            #[cfg(feature = "behavior-trace")]
            {
                // The scenario file digest identifies the complete input
                // program in the behavior projection. This event records
                // when that program was observed/applied in virtual time.
                let mut payload = Vec::with_capacity(25);
                payload.extend_from_slice(&(e.results().len() as u64).to_be_bytes());
                payload.push(u8::from(e.is_done()));
                payload.extend_from_slice(&e.next_poll_ns().to_be_bytes());
                payload.extend_from_slice(&(uart_bytes.len() as u64).to_be_bytes());
                emu.record_behavior_event(BehaviorEventDomain::ScenarioInput, 1, &payload);
            }
            if e.is_done() {
                return finish(
                    emu,
                    &vclock,
                    &mut uart_bytes,
                    StopReason::ScenarioDone,
                    None,
                    None,
                );
            }
            next_poll_cycles = vclock.cycles_at(e.next_poll_ns());
            // A poll that changed nothing would otherwise re-fire every
            // step until virtual time moved on.
            next_poll_cycles = next_poll_cycles.max(emu.clock.cycles + 1);
        }

        if let Some(target) = stop_pc
            && emu.cores[0].regs.pc() == target
        {
            return finish(
                emu,
                &vclock,
                &mut uart_bytes,
                StopReason::PcMatch,
                None,
                None,
            );
        }
        if let Some(name) = fatal_exception_name(emu.cores[0].regs.xpsr & 0x1FF) {
            return finish(
                emu,
                &vclock,
                &mut uart_bytes,
                StopReason::Exception,
                Some(name),
                None,
            );
        }
        if emu.clock.cycles >= cycle_limit {
            return finish(
                emu,
                &vclock,
                &mut uart_bytes,
                StopReason::CycleLimit,
                None,
                None,
            );
        }

        let external_event_cycle = next_poll_cycles.min(cycle_limit);
        let consumed = if stop_pc.is_some() {
            emu.step_until(external_event_cycle)
        } else {
            emu.step_until_batched(external_event_cycle, EXACT_DISPATCH_BATCH_CYCLES)
        };
        let consumed = match consumed {
            Ok(c) => c,
            Err(e) => {
                let msg = e.to_string();
                return finish(
                    emu,
                    &vclock,
                    &mut uart_bytes,
                    StopReason::Error,
                    None,
                    Some(msg),
                );
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
                park_state(emu, 0),
                park_state(emu, 1)
            );
            return finish(
                emu,
                &vclock,
                &mut uart_bytes,
                StopReason::Error,
                None,
                Some(detail),
            );
        }

        if steps.is_multiple_of(UART_DRAIN_INTERVAL) {
            uart_bytes.extend_from_slice(&emu.drain_uart0_tx_log());
        }

        // Firmware reprograms the clock tree during init; from here on,
        // virtual milliseconds have to mean what the firmware thinks they
        // mean. The pending poll deadline is expressed in nanoseconds, so
        // it moves with the rebase rather than being stranded at the old
        // rate.
        if vclock.rebase(emu.clock.cycles, emu.bus.clock_tree.sys_clk_hz)
            && let Some(e) = engine.as_deref()
        {
            next_poll_cycles = vclock.cycles_at(e.next_poll_ns()).max(emu.clock.cycles + 1);
        }
    }
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
    sd: Option<&SdReport>,
    keyboard: Option<&KeyboardReport>,
    pwm: Option<&PwmReport>,
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
    if let Some(sd) = sd {
        s.push_str(&sd.to_json());
    }
    if let Some(keyboard) = keyboard {
        s.push_str(&keyboard.to_json());
    }
    if let Some(pwm) = pwm {
        s.push_str(&pwm.to_json());
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
    let sd_card = args.sd.then(|| {
        Arc::new(Mutex::new(SdCard::new_with_format(
            picocalc_board::sdcard::DEFAULT_BLOCKS,
            args.sd_format,
        )))
    });

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

    let handles = BoardHandles {
        lcd: lcd.clone(),
        keyboard: keyboard.clone(),
    };
    let mut engine = scenario.map(|s| scenario::Engine::new(s, args.snapshot_dir.clone()));

    let outcome = run_loop(
        &mut emu,
        args.cycles,
        args.stop_pc,
        engine.as_mut(),
        &handles,
    );

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
    let verdict = judge_run(
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
        sd_report.as_ref(),
        keyboard_report.as_ref(),
        pwm_report.as_ref(),
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
    Ok(verdict.status)
}

#[cfg(test)]
mod tests {
    use super::{
        RunOutcome, SdReport, StopReason, Verdict, fatal_exception_name, json_escape, judge_run,
        validate_backend_identity, validate_sd_selection,
    };
    use picocalc_board::SdFormat;
    #[cfg(feature = "idle-profiler")]
    use rp2040_emu::IdleProfileSnapshot;

    #[cfg(feature = "behavior-trace")]
    use super::behavior_projection;
    #[cfg(feature = "idle-profiler")]
    use super::build_idle_profile_report;
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
        assert_eq!(fatal_exception_name(0), None); // thread mode
        assert_eq!(fatal_exception_name(2), Some("NMI"));
        assert_eq!(fatal_exception_name(3), Some("HardFault"));
        assert_eq!(fatal_exception_name(11), None); // SVCall
        assert_eq!(fatal_exception_name(15), None); // SysTick
        assert_eq!(fatal_exception_name(16), None); // IRQ0
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
        assert_eq!(validate_sd_selection(true, true), Ok(()));
        assert_eq!(validate_sd_selection(true, false), Ok(()));
        assert_eq!(validate_sd_selection(false, false), Ok(()));
        assert_eq!(
            validate_sd_selection(false, true),
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
        }
        .to_json();
        assert!(report.contains("\"format\": \"fat32\""));
        assert!(report.contains("\"block_size\": 512"));
        assert!(report.contains("\"blocks_written\": 1"));
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
}
