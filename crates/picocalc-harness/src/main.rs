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
//! Stop reasons: `cycle_limit` (budget exhausted — a normal stop),
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
    Framebuffer, Keyboard, KeyboardWire, LcdPioWire, St7365p, St7365pWire, pins,
};
use rp2040_emu::{Config, Emulator, EmulatorBuilder};

/// Report schema version. Bump on any breaking field change.
///
/// * 1 — Gate 1: boot / UART / unsupported-MMIO report.
/// * 2 — Gate 2: optional `lcd` + `framebuffer` sections, `board` field.
/// * 3 — Gate 3: optional `psram` section (`--psram` / `--psram-verify-range`).
const SCHEMA_VERSION: u32 = 4;

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
const BUILT_IN_BACKEND_COMMIT: Option<&str> = option_env!("PICOEM_BACKEND_COMMIT");

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
    backend_commit: Option<String>,
    board: Board,
    lcd_variant: LcdVariant,
    fb_png: Option<PathBuf>,
    quantum: Option<u32>,
    psram: bool,
    psram_verify_range: Option<(u32, u32)>,
    keyboard: bool,
    keys: Option<String>,
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

fn print_usage() {
    eprintln!(
        "Usage:\n  \
         picocalc-run --bin <firmware.bin> [options]\n\
         \n\
         --bin <path>             Required. Raw RP2040 flash image (.bin), loaded at\n\
                                  0x1000_0000 and direct-booted from offset 0x100.\n\
         --bootrom <path>         16 KB RP2040 bootrom image, loaded but never executed.\n\
                                  Default: {DEFAULT_BOOTROM_PATH}\n\
         --cycles <N>             Cycle budget. Exceeding it is a normal stop\n\
                                  (stop_reason=cycle_limit). Default: {DEFAULT_CYCLE_LIMIT}\n\
         --stop-pc <hex>          Stop with stop_reason=pc_match when core 0's PC equals\n\
                                  this address (e.g. 0x10000ca8). Forces a 1-cycle step\n\
                                  quantum so every instruction boundary is observed.\n\
         --json <path>            Write the JSON report here. Default: stdout.\n\
         --uart <path>            Write raw UART0 TX bytes here. Default: discarded\n\
                                  (byte count + sha256 still reported).\n\
         --backend-commit <str>   Value for the report's backend_commit field.\n\
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
         -h, --help               This message."
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
    let mut backend_commit: Option<String> = None;
    let mut board = Board::None;
    // Variant B is the Canonical BSP default; variant A is what the
    // official sample uses. Selected explicitly so a report always
    // records which transport produced it.
    let mut lcd_variant = LcdVariant::B;
    let mut fb_png: Option<PathBuf> = None;
    let mut quantum: Option<u32> = None;
    let mut psram = false;
    let mut psram_verify_range: Option<(u32, u32)> = None;
    let mut keyboard = false;
    let mut keys: Option<String> = None;

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
            "--backend-commit" => backend_commit = Some(value("--backend-commit")?),
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
            "--keys" => keys = Some(value("--keys")?),
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
    // Queueing keys implies the controller they arrive through.
    if keys.is_some() {
        keyboard = true;
    }

    Ok(Args {
        bin: bin.ok_or_else(|| "missing required --bin <path>".to_string())?,
        bootrom: bootrom.unwrap_or_else(|| PathBuf::from(DEFAULT_BOOTROM_PATH)),
        cycles: cycles.unwrap_or(DEFAULT_CYCLE_LIMIT),
        stop_pc,
        json,
        uart,
        backend_commit,
        board,
        lcd_variant,
        fb_png,
        quantum,
        psram,
        psram_verify_range,
        keyboard,
        keys,
    })
}

/// Why the run stopped. `as_str` values are part of the report schema.
#[derive(Clone, Copy, PartialEq, Eq)]
enum StopReason {
    CycleLimit,
    PcMatch,
    Exception,
    Error,
}

impl StopReason {
    fn as_str(self) -> &'static str {
        match self {
            StopReason::CycleLimit => "cycle_limit",
            StopReason::PcMatch => "pc_match",
            StopReason::Exception => "exception",
            StopReason::Error => "error",
        }
    }
}

/// Result of the run loop — everything the report needs that isn't
/// already known from the arguments.
struct RunOutcome {
    stop_reason: StopReason,
    cycles: u64,
    pc: u32,
    exception: Option<&'static str>,
    error: Option<String>,
    uart_bytes: Vec<u8>,
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
            .attach_i2c_device(pins::KEYBOARD_I2C_INSTANCE, Box::new(KeyboardWire::new(kbd)))
            .map_err(|i| format!("no I2C instance {i} on RP2040"))?;
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

fn run_loop(emu: &mut Emulator, cycle_limit: u64, stop_pc: Option<u32>) -> RunOutcome {
    let mut uart_bytes: Vec<u8> = Vec::new();
    let mut steps: u64 = 0;

    let finish = |emu: &mut Emulator,
                  uart_bytes: &mut Vec<u8>,
                  stop_reason: StopReason,
                  exception: Option<&'static str>,
                  error: Option<String>| {
        uart_bytes.extend_from_slice(&emu.drain_uart0_tx_log());
        RunOutcome {
            stop_reason,
            cycles: emu.clock.cycles,
            pc: emu.cores[0].regs.pc(),
            exception,
            error,
            uart_bytes: std::mem::take(uart_bytes),
        }
    };

    loop {
        // Pre-step observations. Checking before the first step means a
        // `--stop-pc` equal to the reset vector matches immediately.
        if let Some(target) = stop_pc
            && emu.cores[0].regs.pc() == target
        {
            return finish(emu, &mut uart_bytes, StopReason::PcMatch, None, None);
        }
        if let Some(name) = fatal_exception_name(emu.cores[0].regs.xpsr & 0x1FF) {
            return finish(
                emu,
                &mut uart_bytes,
                StopReason::Exception,
                Some(name),
                None,
            );
        }
        if emu.clock.cycles >= cycle_limit {
            return finish(emu, &mut uart_bytes, StopReason::CycleLimit, None, None);
        }

        let consumed = match emu.step() {
            Ok(c) => c,
            Err(e) => {
                let msg = e.to_string();
                return finish(emu, &mut uart_bytes, StopReason::Error, None, Some(msg));
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
            return finish(emu, &mut uart_bytes, StopReason::Error, None, Some(detail));
        }

        if steps.is_multiple_of(UART_DRAIN_INTERVAL) {
            uart_bytes.extend_from_slice(&emu.drain_uart0_tx_log());
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

/// The `keyboard` report section (Gate 4).
struct KeyboardReport {
    attached: bool,
    addr: u16,
    reg_selects: u64,
    key_events_delivered: u64,
    key_events_remaining: usize,
    battery_reads: u64,
    backlight_writes: u64,
    backlight: u8,
    unknown_reg_selects: u64,
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
            "    \"battery_reads\": {}, \"backlight_writes\": {}, \"backlight\": {},\n",
            self.battery_reads, self.backlight_writes, self.backlight
        ));
        match self.last_unknown_reg {
            Some(reg) => s.push_str(&format!(
                "    \"unknown_reg_selects\": {}, \"last_unknown_reg\": \"0x{:02x}\"\n",
                self.unknown_reg_selects, reg
            )),
            None => s.push_str(&format!(
                "    \"unknown_reg_selects\": {}, \"last_unknown_reg\": null\n",
                self.unknown_reg_selects
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
    lcd: Option<&LcdReport>,
    fb: Option<&FramebufferReport>,
    psram: Option<&PsramReport>,
    keyboard: Option<&KeyboardReport>,
    pwm: Option<&PwmReport>,
    pio: Option<&PioReport>,
) -> String {
    let mut s = String::new();
    s.push_str("{\n");
    s.push_str(&format!("  \"schema_version\": {SCHEMA_VERSION},\n"));
    s.push_str(&format!(
        "  \"backend_commit\": {},\n",
        json_string(backend_commit)
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
    if let Some(keyboard) = keyboard {
        s.push_str(&keyboard.to_json());
    }
    if let Some(pwm) = pwm {
        s.push_str(&pwm.to_json());
    }
    if let Some(pio) = pio {
        s.push_str(&pio.to_json());
    }

    s.push_str(&format!(
        "  \"uart\": {{\"bytes\": {}, \"sha256\": {}}}\n",
        outcome.uart_bytes.len(),
        json_string(uart_sha)
    ));
    s.push_str("}\n");
    s
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
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

fn run() -> Result<(), String> {
    let args = parse_args().inspect_err(|_| print_usage())?;

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

    let (mut emu, boot_mode, lcd) = boot(
        firmware,
        &bootrom,
        step_quantum,
        args.board,
        args.lcd_variant,
        args.psram,
        keyboard.clone(),
    )?;
    emu.bus.unsupported_mmio_log_enabled = true;

    let outcome = run_loop(&mut emu, args.cycles, args.stop_pc);

    // Snapshot the panel before anything else touches it.
    let (lcd_report, fb) = match &lcd {
        Some(lcd) => {
            let guard = lcd.lock().map_err(|_| "LCD model mutex poisoned")?;
            (Some(LcdReport::snapshot(&guard)), Some(guard.framebuffer()))
        }
        None => (None, None),
    };
    let fb_report = build_framebuffer_report(fb.as_ref(), args.fb_png.as_deref())?;

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

    let keyboard_report = keyboard.as_ref().map(|kbd| {
        let k = kbd.lock().expect("keyboard mutex");
        KeyboardReport {
            attached: true,
            addr: picocalc_board::keyboard::KEYBOARD_I2C_ADDR,
            reg_selects: k.reg_selects,
            key_events_delivered: k.key_events_delivered,
            key_events_remaining: k.queued(),
            battery_reads: k.battery_reads,
            backlight_writes: k.backlight_writes,
            backlight: k.backlight,
            unknown_reg_selects: k.unknown_reg_selects,
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

    let backend_commit = args
        .backend_commit
        .as_deref()
        .or(BUILT_IN_BACKEND_COMMIT)
        .unwrap_or("unknown");

    let report = build_report(
        backend_commit,
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
        lcd_report.as_ref(),
        fb_report.as_ref(),
        psram_report.as_ref(),
        keyboard_report.as_ref(),
        pwm_report.as_ref(),
        pio_report.as_ref(),
    );

    match &args.json {
        Some(path) => std::fs::write(path, report.as_bytes())
            .map_err(|e| format!("writing report {}: {e}", path.display()))?,
        None => print!("{report}"),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{StopReason, fatal_exception_name, json_escape};

    #[test]
    fn stop_reason_strings_are_stable() {
        assert_eq!(StopReason::CycleLimit.as_str(), "cycle_limit");
        assert_eq!(StopReason::PcMatch.as_str(), "pc_match");
        assert_eq!(StopReason::Exception.as_str(), "exception");
        assert_eq!(StopReason::Error.as_str(), "error");
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
}
