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

mod sha256;

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use rp2040_emu::{Config, Emulator, EmulatorBuilder};

use crate::sha256::sha256_hex;

/// Report schema version. Bump on any breaking field change.
const SCHEMA_VERSION: u32 = 1;

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

struct Args {
    bin: PathBuf,
    bootrom: PathBuf,
    cycles: u64,
    stop_pc: Option<u32>,
    json: Option<PathBuf>,
    uart: Option<PathBuf>,
    backend_commit: Option<String>,
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
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
        i += 1;
    }

    Ok(Args {
        bin: bin.ok_or_else(|| "missing required --bin <path>".to_string())?,
        bootrom: bootrom.unwrap_or_else(|| PathBuf::from(DEFAULT_BOOTROM_PATH)),
        cycles: cycles.unwrap_or(DEFAULT_CYCLE_LIMIT),
        stop_pc,
        json,
        uart,
        backend_commit,
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
fn boot(
    firmware: Vec<u8>,
    bootrom: &[u8],
    step_quantum: u32,
) -> Result<(Emulator, BootMode), String> {
    let mut emu = EmulatorBuilder::new(Config {
        sys_clk_hz: DEFAULT_SYS_CLK_HZ,
    })
    .step_quantum(step_quantum)
    .flash(firmware)
    .build()
    .expect("Serial build is infallible");

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
        return Ok((emu, BootMode::BootromResetVector));
    }
    emu.direct_boot_from_flash(SDK_VTOR_FLASH_OFFSET);
    Ok((emu, BootMode::DirectBootFromFlash))
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

fn run() -> Result<(), String> {
    let args = parse_args().inspect_err(|_| print_usage())?;

    let firmware = load_image(&args.bin, "firmware")?;
    let bootrom = load_image(&args.bootrom, "bootrom")?;
    let firmware_sha = sha256_hex(&firmware);
    let bootrom_sha = sha256_hex(&bootrom);

    let step_quantum = if args.stop_pc.is_some() {
        QUANTUM_PC_WATCH
    } else {
        QUANTUM_FREE_RUN
    };

    let (mut emu, boot_mode) = boot(firmware, &bootrom, step_quantum)?;
    emu.bus.unsupported_mmio_log_enabled = true;

    let outcome = run_loop(&mut emu, args.cycles, args.stop_pc);

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
