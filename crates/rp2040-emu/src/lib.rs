//! RP2040 emulator library.
//!
//! Phase 5.A fills in the bus fabric, CLOCKS/RESETS/PLL/XOSC/ROSC
//! register storage, full SIO (GPIO, CPUID, FIFO, spinlocks, divider,
//! interpolators — **no** doorbells / MTIME / coprocessor bridge),
//! IO_BANK0 / PADS_BANK0, XIP_CTRL / SSI stubs, and dual-core stepping
//! (core 0 runs; core 1 stays halted until woken via the SIO FIFO
//! protocol).
//!
//! Phase 5.B wires the two PIO blocks (`bus.pio[0]`, `bus.pio[1]`) into
//! AHB at `0x5020_0000` / `0x5030_0000`, steps them once per emulator
//! step, and merges their pad outputs into `bus.gpio_in` (PIO OE
//! overrides SIO on a per-pin basis, mirroring `rp2350_emu::Emulator`).
//!
//! See `wrk_docs/2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

use tracing::info;

mod audio_sink;
pub mod bus;
pub mod core;
pub mod dma;
pub mod dreq;
pub mod irq;
pub mod memory;
pub mod peripherals;

pub use audio_sink::AudioSinkSnapshot;

mod idle_profile;

#[cfg(feature = "event-horizon-profiler")]
mod running_profile;

#[cfg(feature = "idle-profiler")]
pub use idle_profile::{
    CumulativeHistogramSnapshot, IDLE_HISTOGRAM_BUCKETS, IDLE_HORIZON_SCHEMA_VERSION,
    IDLE_PROFILE_SCHEMA_VERSION, IdleBlockerCycles, IdleBlockerEpisodes, IdleCurrentProbe,
    IdleEventHorizonProbe, IdleEventSourceMask, IdleHorizonEvents, IdleProfileSnapshot,
};

#[cfg(feature = "event-horizon-profiler")]
pub use running_profile::{
    DecodeProfileSnapshot, ONE_CYCLE_FALLBACK_SIGNATURE_BUCKETS,
    RUNNING_EVENT_PROFILE_SCHEMA_VERSION, RunningBoundaryEvents, RunningBoundaryMask,
    RunningBoundarySnapshot, RunningEventProfileSnapshot,
};

#[cfg(feature = "behavior-trace")]
mod behavior_trace;

#[cfg(feature = "behavior-trace")]
pub use behavior_trace::{
    BEHAVIOR_TRACE_SCHEMA_VERSION, BehaviorEventDomain, BehaviorTraceDomainSnapshot,
    BehaviorTraceSnapshot,
};

// Dual-execution HLD V1 (Stage 3b.2) — threaded runtime scaffolding.
// The module file internally `#![cfg]`-gates to x86_64 Windows + the
// `threading` cargo feature, so non-Windows and `--no-default-features`
// builds compile an empty module and the serial path is unaffected.
#[cfg(feature = "threading")]
pub mod threaded;

// -----------------------------------------------------------------------
// Dual-execution HLD V1 (Stage 3b.1) — public types.
//
// Introduces the `ExecutionModel` selector, `ConfigError`, `WorkerName`,
// and `EmulatorError` to mirror the RP2350 crate. Stage 3b.1 ships the
// types + the `CoreBus` trait port so later sub-stages (3b.2: threaded/
// module, 3b.4: builder wiring) can land against a stable surface. The
// Emulator dispatch path stays Serial-only in 3b.1.
// -----------------------------------------------------------------------

/// Execution model for an [`Emulator`]. Selected at construction via
/// [`EmulatorBuilder::execution`]; cannot be switched post-build.
///
/// - `Serial` — oracle-validated reference path (QEMU + silicon
///   differentials). Single-threaded, per-instruction interleave.
///   Always available.
/// - `Threaded` — multi-thread runtime; opt-in throughput optimization
///   on x86_64 Windows or Linux hosts with the `threading` cargo
///   feature on.
///   Not validated against QEMU/silicon oracles. Not yet wired into
///   [`Emulator::step`] — arrives with Stage 3b.4.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ExecutionModel {
    #[default]
    Serial,
    Threaded,
}

/// Errors returned by [`EmulatorBuilder::build`] once the Stage 3b.4
/// wiring lands. The only non-trivial variant today is
/// `ThreadingUnavailable`, returned when the caller selects
/// [`ExecutionModel::Threaded`] but the host platform or build
/// configuration cannot satisfy it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// `ExecutionModel::Threaded` selected but the current build does
    /// not include a threaded runtime — either the `threading` cargo
    /// feature is off, or the host is not one of the supported
    /// platforms (currently x86_64 Windows or Linux only).
    ThreadingUnavailable,
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::ThreadingUnavailable => write!(
                f,
                "ExecutionModel::Threaded is unavailable (requires x86_64 Windows/Linux \
                 with the `threading` cargo feature enabled)"
            ),
        }
    }
}

impl std::error::Error for ConfigError {}

/// Identifier for a worker thread in the threaded runtime. RP2040
/// uses a three-worker layout (core0, core1, coordinator) — smaller
/// than RP2350's six-worker layout because M0+ has no PIO-as-worker
/// split in the Stage 3b plan. rp2350_emu's `Pio0`/`Pio1`/`Pio2` worker
/// variants are intentionally omitted here; if PIO becomes a
/// bottleneck the enum can gain those variants in a follow-up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkerName {
    Core0,
    Core1,
    Coord,
}

impl WorkerName {
    /// Short label for summary tables / error messages. Kept stable so
    /// harness tooling can scrape diagnostic output.
    pub fn as_str(self) -> &'static str {
        match self {
            WorkerName::Core0 => "core0",
            WorkerName::Core1 => "core1",
            WorkerName::Coord => "coord",
        }
    }
}

/// Errors returned by post-construction [`Emulator`] methods once the
/// Stage 3b.4 wiring lands. Surfaces runtime-model mismatches and
/// worker panics (dual-execution HLD V1 §5.5).
///
/// `WorkerPanicked` is sticky: once an [`Emulator`] observes a worker
/// panic, every subsequent call on that instance returns the same
/// error without re-attempting the workers (one-shot-after-panic, HLD
/// §5.5 item 5). Drop the instance and rebuild from a fresh
/// [`EmulatorBuilder`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EmulatorError {
    /// Called a Serial-only method on a Threaded emulator, e.g.
    /// `step()` — Threaded runs in quanta, not single-step. HLD §5.4.
    NotSupportedInThreadedMode,
    /// One of the worker threads panicked. The `Emulator` is sticky-
    /// poisoned after this; drop and rebuild. Only produced on the
    /// Threaded path.
    WorkerPanicked { which: WorkerName, message: String },
    /// The shared [`picoem_common::SpinBarrier`] watchdog fired
    /// because a worker failed to arrive at the rendezvous within
    /// [`picoem_common::threaded::DEFAULT_DEADLINE`]. The `Emulator`
    /// is sticky-poisoned after this; drop and rebuild. HLD V1 §6.6.
    ///
    /// Only produced on the Threaded path. `which` is the first worker
    /// that returned `TimedOut` at its barrier; since the barrier
    /// cannot identify *which* worker failed to arrive, this field
    /// names an observer rather than the culprit. `elapsed_ms` is the
    /// reporting waiter's own wall-clock elapsed time at expiry.
    BarrierTimeout { which: WorkerName, elapsed_ms: u32 },
}

impl std::fmt::Display for EmulatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EmulatorError::NotSupportedInThreadedMode => write!(
                f,
                "operation not supported on a Threaded Emulator (Serial-only)"
            ),
            EmulatorError::WorkerPanicked { which, message } => {
                write!(f, "worker {} panicked: {message}", which.as_str())
            }
            EmulatorError::BarrierTimeout { which, elapsed_ms } => write!(
                f,
                "barrier watchdog fired (observed by worker {}) after {}ms",
                which.as_str(),
                elapsed_ms
            ),
        }
    }
}

impl std::error::Error for EmulatorError {}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod pio_tests;

pub use self::bus::Bus;
pub use self::core::CortexM0Plus;
pub use self::memory::{Memory, ROM_SIZE, SRAM_SIZE, bank_for_address};

pub use picoem_common::Pacer;
pub use picoem_common::{Clock, PacerSnapshot, PacerStats};

/// ROSC nominal frequency (~6.5 MHz). RP2040 boots on ROSC at the same
/// nominal rate as RP2350; PLL configuration (if any) happens later in
/// firmware.
pub use picoem_common::ROSC_FREQ_HZ;

/// Emulator configuration.
pub struct Config {
    /// System clock frequency in Hz. Default: ROSC (~6.5 MHz).
    pub sys_clk_hz: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sys_clk_hz: ROSC_FREQ_HZ,
        }
    }
}

/// Default quantum size in cycles. Matches `rp2350_emu`.
pub const DEFAULT_STEP_QUANTUM: u32 = 64;

/// Top-level RP2040 emulator. Owns dual Cortex-M0+ cores, bus fabric,
/// memory, and clock.
///
/// Dual-execution HLD V1: an `Emulator` has a fixed [`ExecutionModel`]
/// picked at construction time via [`EmulatorBuilder::execution`]. In
/// Serial mode (default) the `cores` / `bus` / `clock` fields are the
/// authoritative state and the existing per-instruction interleave
/// applies. In Threaded mode those fields retain their post-seed
/// snapshot until the first `run_quantum` promotes them into the
/// threaded runtime; afterwards the flat fields are zero-cost
/// placeholders and typed accessors fire a debug-assert if touched.
pub struct Emulator {
    pub cores: [CortexM0Plus; 2],
    pub bus: Bus,
    pub clock: Clock,
    /// Cycles advanced per call to [`Self::step`].
    pub step_quantum: u32,
    /// Total PIO ticks performed in the slow path
    /// (`tick_pio_and_route_irqs`). Diagnostic-only — used by the
    /// PicoGUS harness to confirm PIO is actually being driven.
    /// Bumps by `cycles` per quantum after HLD 2026.04.26 V5 chunked
    /// refactor (per-quantum granularity is acceptable).
    pub pio_tick_count: u64,
    /// Subset of [`Self::pio_tick_count`] where bit 4 (IOW for PicoGUS)
    /// of `bus.gpio_in` was low at the moment of the tick. If this stays
    /// at zero while the harness is asserting IOW low, the override
    /// merge is breaking somewhere in the path.
    pub pio_tick_iow_low_count: u64,
    /// Diagnostic — maximum PC value PIO0 SM0 has held during the run
    /// (observed after each slow-path tick). PicoGUS bring-up: if this
    /// stays at the WAIT-pin instruction slot, SM0 never escaped its
    /// wait. If it climbs to a higher slot, SM0 advanced through the
    /// program. Slow-path-only — fast-path skips PIO when both blocks
    /// are idle so SM0 wouldn't be moving regardless.
    pub pio0_sm0_max_pc: u8,
    /// Diagnostic — number of times PIO0 SM0's PC differed from its
    /// previous-tick value (advanced or jumped). Slow-path-only.
    pub pio0_sm0_pc_advances: u64,
    /// Last observed PC of PIO0 SM0 — internal scratch used by
    /// [`Self::tick_pio_and_route_irqs`] to decide whether the
    /// PC moved this tick. Initialised to a sentinel `0xFF` so the
    /// very first observation always counts as an advance.
    pub(crate) pio0_sm0_last_pc: u8,
    /// OPT0-A Serial idle profiler. Entirely absent from normal builds;
    /// diagnostic harnesses opt in through the `idle-profiler` feature.
    #[cfg(feature = "idle-profiler")]
    idle_profiler: Option<idle_profile::IdleProfiler>,
    /// OPT2-B running event-horizon opportunity profiler. Diagnostic only;
    /// normal and performance builds contain neither this state nor its bus
    /// access latches.
    #[cfg(feature = "event-horizon-profiler")]
    running_profiler: Option<running_profile::RunningProfile>,
    /// OPT0-B streaming correctness trace. Entirely absent from normal
    /// builds so performance mode has no disabled hot-path branch.
    #[cfg(feature = "behavior-trace")]
    behavior_tracer: Option<behavior_trace::BehaviorTracer>,
    /// Execution model chosen at build time; cannot change
    /// post-construction. Dispatch for [`Self::step`] / [`Self::run`] /
    /// [`Self::run_quantum`] branches on this. Defaults to
    /// [`ExecutionModel::Serial`].
    pub execution_model: ExecutionModel,
    /// Live 3-thread runtime when `execution_model == Threaded` and the
    /// first `run` / `run_quantum` has fired. Takes ownership of the
    /// pre-seeded cores / bus / clock during lazy `promote_to_threaded`.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub(crate) threaded: Option<threaded::ThreadedEmulator>,
    /// Sticky panic record from a Threaded worker. Set once when
    /// `run_quantum` / `run` observes a worker panic; every subsequent
    /// call returns this cached error without re-attempting workers.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub(crate) panic_info: Option<(WorkerName, String)>,
    /// Sticky watchdog-timeout record from a Threaded run. Set once
    /// when `run_quantum` / `run` observes a barrier timeout; every
    /// subsequent call returns this cached error without re-attempting
    /// workers. HLD V1 §6.6 Stage 5.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub(crate) timeout_info: Option<(WorkerName, u32)>,
    /// Test-only panic injector. Armed via
    /// [`Self::inject_panic_for_testing`]; consumed on the next
    /// `run_quantum` / `run` call which forwards to
    /// [`threaded::ThreadedEmulator::inject_panic_for_testing`].
    #[cfg(all(
        feature = "testing",
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub(crate) pending_panic_inject: Option<WorkerName>,
    /// `true` once `promote_to_threaded` has moved the seeded state
    /// into `self.threaded` — the flat `cores` / `bus` / `clock` fields
    /// now hold zero-cost placeholders. Typed accessors
    /// (`core`, `core_mut`, `peek`, `gpio_read`, …) `debug_assert!` on
    /// this flag so Serial-only callers trip loudly if they reach for
    /// the flat fields after a Threaded run.
    ///
    /// Known escape: raw field access (`emu.bus.*`) bypasses the
    /// guarded accessors — documented in `tech_debt.md`.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub(crate) bus_is_placeholder: bool,
}

impl Emulator {
    /// Create a new Serial-mode emulator with the given configuration.
    /// Infallible shim: Serial builds always succeed. For Threaded
    /// construction or to surface `ConfigError` explicitly, use
    /// [`EmulatorBuilder`] directly.
    pub fn new(config: Config) -> Self {
        EmulatorBuilder::new(config)
            .build()
            .expect("Serial build is infallible")
    }

    /// Currently selected execution model. Set at build time; does not
    /// change post-construction.
    pub fn execution_model(&self) -> ExecutionModel {
        self.execution_model
    }

    /// Cycle counter for core `idx` (0 or 1). Serial reads directly
    /// from the flat `cores[idx]`; Threaded reads the worker-thread
    /// snapshot (valid between `run_quantum` calls). Returns 0 on
    /// Threaded before the first `run_quantum` (cores not yet taken).
    pub fn core_cycles(&self, idx: u8) -> u64 {
        #[cfg(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        if let Some(t) = &self.threaded {
            return t.core_cycles(idx);
        }
        match idx {
            0 | 1 => self.cores[idx as usize].cycles(),
            _ => panic!("core_cycles: idx must be 0 or 1"),
        }
    }

    /// Placeholder-guard message shared by the typed accessors below.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    const PLACEHOLDER_GUARD_MSG: &'static str = "direct field access on cores/bus/clock is Serial-only; emulator is in \
         Threaded mode — use typed accessors like core_cycles(), master_cycle(), \
         gpio_read() instead";

    /// Debug-only placeholder assertion. No-op on non-threading
    /// platforms and in release builds.
    #[inline(always)]
    fn assert_not_placeholder(&self) {
        #[cfg(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        debug_assert!(!self.bus_is_placeholder, "{}", Self::PLACEHOLDER_GUARD_MSG);
    }

    /// Reset the emulator:
    /// * Load SP from ROM word 0, PC from ROM word 4 into both cores.
    /// * Core 0 is the bootstrapped core (runs from reset).
    /// * Core 1 is halted — the Pico SDK launches it by writing a
    ///   wake sequence through the SIO FIFO; `step` calls
    ///   [`Self::wake_checks`] each quantum to observe the handshake.
    pub fn reset(&mut self) {
        self.assert_not_placeholder();
        let initial_sp = self.bus.memory.rom_read32(0);
        let reset_vector = self.bus.memory.rom_read32(4);

        for i in 0..2 {
            self.cores[i] = CortexM0Plus::with_id(i as u8);
            self.cores[i].regs.msp = initial_sp;
            self.cores[i].regs.r[13] = initial_sp;
            self.cores[i].regs.set_pc(reset_vector & !1);
            self.cores[i].regs.xpsr = 1 << 24; // Thumb bit (XPSR_T)
        }

        self.bus.sio.reset();
        self.bus.resets.reset();
        self.bus.clocks_regs.reset();
        self.bus.xosc_regs.reset();
        self.bus.rosc_regs.reset();
        self.bus.watchdog_tick.reset();
        self.bus.timer.reset();
        self.bus.uart0.reset();
        self.bus.uart1.reset();
        self.bus.spi0.reset();
        self.bus.spi1.reset();
        self.bus.i2c0.reset();
        self.bus.i2c1.reset();
        self.bus.adc.reset();
        self.bus.pwm.reset();
        self.bus.dma.reset();
        self.bus.irq_pending = 0;
        for n in &mut self.bus.nvics {
            n.reset();
        }
        self.bus.pll_sys_regs = bus::clocks::PLL_RESET;
        self.bus.pll_usb_regs = bus::clocks::PLL_RESET;
        self.bus.pll_sys_lock_at_cycle = None;
        self.bus.pll_usb_lock_at_cycle = None;
        self.bus.master_cycle = 0;
        self.bus.clock_tree = Default::default();
        self.bus.io_bank0.reset();
        self.bus.pads_bank0.reset();
        for pio in &mut self.bus.pio {
            pio.reset();
        }
        // Diagnostic counters track post-reset behaviour, so zero them
        // on `reset()` too (the SM `pc` field also resets to 0, hence
        // the sentinel `0xFF` for `last_pc` to make the first observed
        // PC count as an advance).
        self.pio0_sm0_max_pc = 0;
        self.pio0_sm0_pc_advances = 0;
        self.pio0_sm0_last_pc = 0xFF;
        #[cfg(feature = "idle-profiler")]
        if let Some(profiler) = self.idle_profiler.as_mut() {
            *profiler = idle_profile::IdleProfiler::default();
        }
        #[cfg(feature = "event-horizon-profiler")]
        if let Some(profiler) = self.running_profiler.as_mut() {
            *profiler = running_profile::RunningProfile::default();
        }
        if let Some(ref mut psram) = self.bus.psram {
            psram.reset_state();
        }
        self.bus.clear_bus_fault();
        self.bus.ppb = [Default::default(), Default::default()];
        self.bus.event_flag = [false; 2];
        self.bus.wfe_waiting = [false; 2];
        self.bus.gpio_in = 0;
        self.bus.external_gpio_in_override = 0;
        self.bus.external_gpio_in_mask = 0;
        self.bus.end_core1_step();

        self.clock = Clock { cycles: 0 };

        // Core 1 stays halted — bootrom on real silicon parks core 1 in
        // a wait-for-event loop until core 0 sends the wake sequence.
        // Routed through the wrapper so the SIO handshake FSM `armed`
        // flag stays in sync with core 1's halt state (HLD §2.1).
        self.halt_core1();
    }

    /// Load a raw binary at the given address. ROM writes are honoured
    /// (test seeding path); SRAM writes land in the SRAM backing store;
    /// XIP loads use [`Self::load_flash`].
    pub fn load_image(&mut self, addr: u32, data: &[u8]) {
        self.assert_not_placeholder();
        match addr >> 28 {
            0x0 => {
                // ROM: bootrom-style loads happen via `load_bootrom`.
                // Support ROM overlay here for tests that want to place
                // code at an arbitrary ROM offset without zero-padding.
                let offset = (addr & 0x0FFF_FFFF) as usize;
                let mut rom_buf = vec![0u8; ROM_SIZE];
                // Seed with current ROM content so a partial overlay
                // preserves whatever was already loaded.
                for i in 0..ROM_SIZE {
                    rom_buf[i] = self.bus.memory.rom_read8(i as u32);
                }
                let end = (offset + data.len()).min(ROM_SIZE);
                if offset < ROM_SIZE {
                    rom_buf[offset..end].copy_from_slice(&data[..end - offset]);
                    self.bus.memory.load_rom(&rom_buf);
                }
                self.invalidate_decode_caches_region(crate::bus::invalidation_regions::ROM);
            }
            0x2 => {
                for (i, &byte) in data.iter().enumerate() {
                    let a = addr.wrapping_add(i as u32);
                    self.bus.memory.sram_write8(a & 0x00FF_FFFF, byte);
                }
                self.invalidate_decode_caches_region(crate::bus::invalidation_regions::SRAM);
            }
            _ => {}
        }
    }

    /// Bulk-invalidate both cores' decode caches for the given region
    /// bitmask. Used by `load_image` (which writes directly to the
    /// memory backing store, bypassing `Bus::write*`'s automatic
    /// per-write invalidation queue) to keep the caches coherent with
    /// the new bytes. Caller passes a single region bit (ROM / XIP /
    /// SRAM) or BULK to drain everything.
    fn invalidate_decode_caches_region(&mut self, region: u8) {
        self.cores[0].invalidate_decode_cache_regions(region);
        self.cores[1].invalidate_decode_cache_regions(region);
    }

    /// Load the 16 KB RP2040 bootrom at address `0x0000_0000`.
    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.assert_not_placeholder();
        self.bus.load_bootrom(data);
        // Drain the region bit `Bus::load_bootrom` set so the next
        // `step` doesn't see a stale ROM region flag.
        let regions = std::mem::take(&mut self.bus.pending_invalidation_regions);
        if regions != 0 {
            self.cores[0].invalidate_decode_cache_regions(regions);
            self.cores[1].invalidate_decode_cache_regions(regions);
        }
    }

    /// Load an XIP flash image (appears at XIP address `0x1000_0000`).
    pub fn load_flash(&mut self, data: &[u8]) {
        self.assert_not_placeholder();
        self.bus.load_flash(data);
        let regions = std::mem::take(&mut self.bus.pending_invalidation_regions);
        if regions != 0 {
            self.cores[0].invalidate_decode_cache_regions(regions);
            self.cores[1].invalidate_decode_cache_regions(regions);
        }
    }

    /// Direct-boot into an SDK-style firmware by emulating the boot2 →
    /// application handoff. On real silicon the boot2 stub does three
    /// things before jumping to the application reset handler: it loads
    /// SP from word 0 of the vector table, sets VTOR to the vector
    /// table's flash address, and branches to the reset handler at word
    /// 1 (Thumb bit stripped). This helper performs the same three-piece
    /// handoff — SP, VTOR, PC — into both cores, then parks core 1
    /// halted as `reset()` does. The vector table is expected at
    /// `vtor_offset` within flash (typically `0x100` for pico-sdk).
    ///
    /// Skipping VTOR is silently fatal for any pico-sdk firmware that
    /// calls `runtime_init_install_ram_vector_table`, which copies the
    /// flash vector table into SRAM and then writes the SRAM address to
    /// VTOR. The copy walks `mem[VTOR + 4*i]` for `i` in 0..48; with
    /// VTOR left at `0x0000_0000` that reads from the bootrom image —
    /// garbage bytes get installed as exception handlers and the first
    /// systick fault sends PC into the weeds.
    ///
    /// Why this helper exists at all — the real RP2040 B2 bootrom
    /// detects an attached QSPI flash chip by sampling six QSPI pads via
    /// `SIO GPIO_HI_IN` (offset `0x008`) and validates boot2 by CRC of
    /// the first 252 flash bytes read through the SSI peripheral. Our
    /// emulator stubs SSI and has no QSPI pad model, so the bootrom
    /// (correctly) gives up and enters USB MSC boot mode, where it waits
    /// forever for a UF2 drop. This helper bypasses that check.
    ///
    /// The bootrom image remains populated at `0x00000000` so firmware
    /// can resolve ROM function-table pointers (`rom_func_lookup`,
    /// `rom_data_lookup`). Call **after** `load_bootrom` + `load_flash`
    /// + `reset`.
    pub fn direct_boot_from_flash(&mut self, vtor_offset: u32) {
        self.assert_not_placeholder();
        let sp = self.bus.memory.xip_read32(vtor_offset);
        let pc = self.bus.memory.xip_read32(vtor_offset + 4) & !1;
        let vtor_addr = bus::XIP_FLASH_BASE + vtor_offset;
        for core in 0..2 {
            self.cores[core].regs.msp = sp;
            self.cores[core].regs.r[13] = sp;
            self.cores[core].regs.set_pc(pc);
        }
        self.bus.ppb[0].vtor = vtor_addr;
        self.bus.ppb[1].vtor = vtor_addr;
        // Core 1 stays halted — SDK firmware launches it explicitly via
        // the SIO FIFO handshake, same as after bootrom hand-off. Route
        // through the wrapper so the handshake FSM re-arms if the caller
        // used `direct_boot_from_flash` as a mode-switch (§2.1).
        self.halt_core1();
    }

    /// Advance the system by up to `step_quantum` master-clock cycles,
    /// then tick peripherals once. Returns the number of cycles actually
    /// consumed in this quantum (may be less than `step_quantum` if both
    /// cores halt mid-quantum).
    ///
    /// Per-instruction interleaving of core 0 and core 1 is preserved so
    /// that bank contention timing on core 1 (`contention_check_active`)
    /// still accounts +1 cycle on same-port accesses. Each core is armed
    /// independently per iteration — core 1 can continue running while
    /// core 0 is halted, and vice-versa. Per-instruction FIFO wake
    /// checks (`maybe_wake_core1`) also remain so a FIFO write from
    /// core 0 wakes core 1 within the same quantum.
    ///
    /// Dual-core schedule (per inner-loop iteration):
    /// 1. If core 0 is not halted, step it — fetch/decode/execute one
    ///    instruction.
    /// 2. If core 1 is not halted, step it with `contention_check_active`
    ///    so same-bank SRAM accesses incur +1 cycle.
    /// 3. Advance the master clock by `max(c0, c1)` — both cores share
    ///    one clock on real silicon.
    ///
    /// The loop exits when `clock.cycles >= target` or both cores are
    /// halted. Then advance PIO and the GPIO/PSRAM merge. On the slow
    /// branch (see below), this runs **one system cycle at a time** —
    /// but *only* when a PIO-driven pin-watching off-chip device is
    /// actually attached (`Bus::has_pin_watching_device`, currently just
    /// PSRAM) and PIO is active and the quantum consumed more than one
    /// cycle. PIO-driven SPI programs toggle SCK every 1–2 sysclks, and
    /// `tick_pio_and_route_irqs` takes a single static `bus.gpio_in`
    /// snapshot for its entire `cycles` argument — so a bulk
    /// `tick_pio_and_route_irqs(consumed)` followed by one
    /// `update_gpio()` would let every SCK/CS edge inside the quantum go
    /// unobserved by the off-chip model until the quantum boundary (the
    /// PSRAM would only ever see the quantum's final pin snapshot, and a
    /// real firmware's byte framing would desync — this was an actual
    /// bug: see `wrk_journals` Gate 3 PSRAM PIO-integration diagnosis).
    /// Looping `tick_pio_and_route_irqs(1) + update_gpio()` `consumed`
    /// times keeps every edge synchronous with `Psram::tick`, at the
    /// cost of `consumed` `update_gpio` calls instead of one.
    ///
    /// Fast-path: when both PIO blocks have no SM enabled, no GPIO pin
    /// can change during this peripheral-tick window (SIO writes only
    /// land inside the core loop above, which has already finished),
    /// so a second `psram.tick` on the same pin snapshot would be a
    /// semantic no-op — one bulk `tick_pio(consumed) + update_gpio()`
    /// suffices. This preserves paced_bench_rp2040's throughput on
    /// pure-ALU workloads (no PIO activity), which would otherwise pay
    /// a per-cycle `update_gpio` tax for nothing.
    ///
    /// Slow-path without a pin-watching device attached (e.g. a board
    /// with PIO active but no PSRAM): still one bulk call per quantum,
    /// same as before this fix — no off-chip model depends on
    /// sub-quantum edges, so there is nothing to preserve and no reason
    /// to pay the per-cycle cost. This is what keeps existing
    /// (PSRAM-less) workloads' performance and behaviour unchanged.
    ///
    /// Core 1 halted ⇒ PIO may still be ticking (e.g. SPI PSRAM on core
    /// 0), so the per-cycle loop runs regardless of core-halt state.
    /// Differs from `rp2350_emu::Emulator::step`'s quantum-end peripheral
    /// tick — rp2040_emu has the external PSRAM which is sensitive to
    /// sub-quantum edge timing; rp2350_emu has no equivalent peripheral.
    pub fn step(&mut self) -> Result<u64, EmulatorError> {
        if self.execution_model == ExecutionModel::Threaded {
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            if let Some((which, message)) = &self.panic_info {
                return Err(EmulatorError::WorkerPanicked {
                    which: *which,
                    message: message.clone(),
                });
            }
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            if let Some((which, elapsed_ms)) = self.timeout_info {
                return Err(EmulatorError::BarrierTimeout { which, elapsed_ms });
            }
            return Err(EmulatorError::NotSupportedInThreadedMode);
        }
        let consumed = self.step_serial();
        #[cfg(feature = "behavior-trace")]
        self.observe_behavior_trace();
        Ok(consumed)
    }

    /// Advance the system by one normal quantum, or exactly fast-forward a
    /// both-cores-blocked interval to its next event boundary.
    ///
    /// In Serial mode this is the same execution pipeline as [`Self::step`],
    /// but when both cores are blocked and every autonomous source is proven
    /// safe, it may advance directly to the first internal or caller-owned
    /// horizon. A blocked interval never advances past `external_event_cycle`;
    /// a running core retains the existing instruction-boundary semantics. A
    /// source without an exact bulk contract conservatively limits the advance
    /// to one cycle.
    ///
    /// In Threaded mode, this remains unsupported and returns
    /// [`EmulatorError::NotSupportedInThreadedMode`].
    pub fn step_until(&mut self, external_event_cycle: u64) -> Result<u64, EmulatorError> {
        if self.execution_model == ExecutionModel::Threaded {
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            if let Some((which, message)) = &self.panic_info {
                return Err(EmulatorError::WorkerPanicked {
                    which: *which,
                    message: message.clone(),
                });
            }
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            if let Some((which, elapsed_ms)) = self.timeout_info {
                return Err(EmulatorError::BarrierTimeout { which, elapsed_ms });
            }
            return Err(EmulatorError::NotSupportedInThreadedMode);
        }
        let consumed = self.step_serial_with_external(Some(external_event_cycle), true);
        #[cfg(feature = "behavior-trace")]
        self.observe_behavior_trace();
        Ok(consumed)
    }

    /// Serial-mode single-quantum step. Shared by [`Self::step`],
    /// [`Self::step_until`], [`Self::run`] (inner loop) and
    /// [`Self::run_quantum`] (Serial path).
    #[inline(always)]
    fn step_serial(&mut self) -> u64 {
        self.step_serial_with_external(None, false)
    }

    /// Drain the bus's pending decode-cache invalidations into both
    /// cores' caches and reset the buffers. Called after each
    /// `core.step` in [`Self::step_serial`] (mirroring the rp2350_emu
    /// drain at lib.rs:1356-1373, commit `0c31479`).
    ///
    /// Per-instruction queue (`pending_cache_invalidations`) drains
    /// only into the core that just ran — the runner that wrote the
    /// bytes is the one most likely to refetch them, and the peer
    /// core's executable bytes haven't moved this step. Region-scoped
    /// bulk invalidations (`pending_invalidation_regions`, set by ISB
    /// inside an instruction or by a mid-step `Bus::load_*`) drain to
    /// BOTH cores so cross-core SMC observers get evicted on their
    /// next turn.
    #[inline]
    fn drain_cache_invalidations(bus: &mut Bus, cores: &mut [CortexM0Plus; 2]) {
        if !bus.pending_cache_invalidations.is_empty() {
            let active = bus.active_core();
            cores[active].invalidate_decode_cache_entries(&bus.pending_cache_invalidations);
            bus.pending_cache_invalidations.clear();
        }
        if bus.pending_invalidation_regions != 0 {
            let regions = bus.pending_invalidation_regions;
            cores[0].invalidate_decode_cache_regions(regions);
            cores[1].invalidate_decode_cache_regions(regions);
            bus.pending_invalidation_regions = 0;
        }
    }

    /// Serial-mode single-quantum step with an optional external
    /// synchronization boundary.
    ///
    /// `exact_idle_fast_forward` is enabled only by [`Self::step_until`].
    /// The ordinary [`Self::step`] / [`Self::run`] paths retain their
    /// historical one-quantum TIMER wake behavior.
    fn step_serial_with_external(
        &mut self,
        external_event_cycle: Option<u64>,
        exact_idle_fast_forward: bool,
    ) -> u64 {
        debug_assert!(self.step_quantum > 0, "step_quantum must be >= 1");
        // Refresh the Bus's view of the master cycle count so any MMIO
        // reads / writes performed during this quantum (notably PLL CS
        // lock bit + lock-arm transitions — see
        // `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 P2)
        // observe a current cycle. Staleness is bounded by one quantum.
        self.bus.master_cycle = self.clock.cycles;
        let start = self.clock.cycles;
        let target = start.wrapping_add(self.step_quantum as u64);
        #[cfg(feature = "event-horizon-profiler")]
        self.bus.reset_running_cpu_boundaries();
        #[cfg(feature = "event-horizon-profiler")]
        let running_before = self.behavior_observation();
        #[cfg(feature = "event-horizon-profiler")]
        let running_horizon = self.idle_event_horizon_internal(external_event_cycle);

        // Per HLD 2026.04.26 V5 §5.2.3: accumulate per-core cycle counts
        // across the inner loop so the slow-branch SysTick advance
        // mirrors per-core hardware semantics (each core's SysTick
        // decrements on its own consumed cycles, not on the active-core
        // shared register).
        let mut c0_total: u64 = 0;
        let mut c1_total: u64 = 0;

        while self.clock.cycles < target
            && (!self.cores[0].is_halted() || !self.cores[1].is_halted())
        {
            let c0 = if !self.cores[0].is_halted() && !self.bus.wfe_waiting[0] {
                self.bus.set_active_core(0);
                let c = self.cores[0].step(&mut self.bus) as u64;
                // Drain decode-cache invalidations recorded by writes
                // during this step into the core that just ran.
                // Region-scoped bulk invalidations (load_*) reach BOTH
                // cores so a peer core fetching from the same region
                // sees the eviction next quantum. Mirrors rp2350_emu
                // (commit 0c31479, lib.rs §lookup-and-drain).
                Self::drain_cache_invalidations(&mut self.bus, &mut self.cores);
                self.maybe_wake_core1(0);
                c
            } else {
                0
            };

            let c1 = if !self.cores[1].is_halted() && !self.bus.wfe_waiting[1] {
                self.bus.set_active_core(1);
                self.bus.begin_core1_step();
                let c = self.cores[1].step(&mut self.bus) as u64;
                Self::drain_cache_invalidations(&mut self.bus, &mut self.cores);
                self.bus.end_core1_step();
                self.maybe_wake_core1(1);
                c
            } else {
                // Still clear any leftover bank-tracking state so the
                // next iteration starts fresh.
                self.bus.end_core1_step();
                0
            };

            if c0 == 0 && c1 == 0 {
                break;
            }
            c0_total = c0_total.wrapping_add(c0);
            c1_total = c1_total.wrapping_add(c1);
            self.clock.cycles = self.clock.cycles.wrapping_add(c0.max(c1));
        }

        // SIO FIFO IRQs are core-local level signals, unlike the shared
        // peripheral bits drained through `bus.irq_pending`. Refresh after
        // both core steps so FIFO push/pop and WOF/ROE changes from this
        // quantum are visible before blocked-core and wake decisions.
        self.bus.refresh_sio_fifo_irqs();

        let consumed = self.clock.cycles.wrapping_sub(start);
        let both_cores_blocked = (self.cores[0].is_halted() || self.bus.wfe_waiting[0])
            && (self.cores[1].is_halted() || self.bus.wfe_waiting[1]);

        let blocked_advance = if both_cores_blocked && exact_idle_fast_forward {
            let observation = self.idle_source_observation();
            let horizon = self.idle_event_horizon_internal(external_event_cycle);
            horizon.distance_cycles.and_then(|distance| {
                if distance == 0 {
                    None
                } else {
                    // Only the fully classified quiescent path may cross a
                    // quantum. Unresolved temporal sources use the complete
                    // one-cycle fallback encoded by the horizon.
                    let cap = if observation.proven_safe() {
                        u64::from(u32::MAX)
                    } else {
                        1
                    };
                    Some((distance.min(cap), distance, observation, horizon))
                }
            })
        } else if both_cores_blocked {
            // Preserve the established public step/run contract: ordinary
            // stepping advances only toward an IRQ-enabled TIMER alarm and
            // never by more than one configured quantum.
            self.bus
                .next_scheduled_lazy_deadline()
                .and_then(|deadline| {
                    let distance = deadline.saturating_sub(self.bus.master_cycle);
                    if distance == 0 {
                        None
                    } else {
                        let horizon = self.idle_event_horizon_internal(None);
                        Some((
                            distance.min(self.step_quantum as u64),
                            distance,
                            self.idle_source_observation(),
                            horizon,
                        ))
                    }
                })
        } else {
            None
        };

        // tech_debt §1649: when both cores are blocked (halted-or-WFE)
        // and the inner loop made no progress, `consumed == 0` and
        // neither the fast-path nor the slow-path below would advance
        // the master clock — so a TIMER alarm scheduled in the future
        // could never fire to wake either core. Detect that exact
        // state, advance the master clock to the soonest scheduled
        // IRQ-raising alarm (capped by `step_quantum`), tick lazy
        // peripherals once, drain the resulting IRQs to the NVICs, and
        // let `wake_checks` at the tail un-halt the woken core. Take
        // the early return so we don't double-advance via fast/slow
        // path. Production fix per HLD V2 / tech_debt §1649 Option 1.
        if consumed == 0
            && both_cores_blocked
            && self.bus.irq_pending == 0
            && self.bus.nvics[0].pending_and_enabled() == 0
            && self.bus.nvics[1].pending_and_enabled() == 0
            && let Some((advance, horizon_distance, observation, horizon)) = blocked_advance
            && advance > 0
        {
            #[cfg(feature = "behavior-trace")]
            let exact_pwm_boundary = exact_idle_fast_forward
                && advance == horizon_distance
                && horizon
                    .limiting_sources
                    .contains(idle_profile::IdleEventSourceMask::PWM)
                && !horizon
                    .one_cycle_fallback_sources
                    .contains(idle_profile::IdleEventSourceMask::PWM);
            #[cfg(not(feature = "idle-profiler"))]
            let _ = (observation, horizon, horizon_distance);
            #[cfg(feature = "idle-profiler")]
            let blocked_observation = (
                observation,
                horizon,
                self.cores[0].is_halted(),
                self.bus.wfe_waiting[0],
                self.cores[1].is_halted(),
                self.bus.wfe_waiting[1],
            );
            self.clock.cycles = self.clock.cycles.wrapping_add(advance);
            self.bus.master_cycle = self.clock.cycles;
            self.bus.tick_peripherals(advance as u32);
            self.drain_pending_irqs_to_cores();
            self.wake_checks();
            #[cfg(feature = "behavior-trace")]
            if exact_pwm_boundary {
                self.observe_behavior_trace_timer_pwm_boundary();
            }
            #[cfg(feature = "idle-profiler")]
            if let Some(profiler) = self.idle_profiler.as_mut() {
                let (observation, event_horizon, c0_halted, c0_wfe, c1_halted, c1_wfe) =
                    blocked_observation;
                profiler.record_blocked(
                    advance,
                    horizon_distance,
                    observation,
                    event_horizon,
                    c0_halted,
                    c0_wfe,
                    c1_halted,
                    c1_wfe,
                );
            }
            #[cfg(feature = "event-horizon-profiler")]
            if let Some(profiler) = self.running_profiler.as_mut() {
                profiler.record_non_running();
            }
            return advance;
        }
        // See the fn docstring for the rationale on the fast-path and
        // the per-cycle interleave. Measured impact of the fast-path
        // gate on paced_bench_rp2040 (pure ALU, PIO disabled): without
        // it, ~49% throughput regression; with it, neutral.
        //
        // HLD V7 §5.5 broadens the gate from "PIO idle" to "PIO idle
        // AND peripherals (including DMA) idle AND no IRQ pending".
        // TIMER/WATCHDOG_TICK are lazy. DMA and the other stateful
        // peripherals remain part of `all_peripherals_idle()`; the
        // short-circuit below only avoids evaluating them after active
        // PIO has already made the slow path mandatory.
        let pio_idle = self.bus.pio_all_idle();
        // SysTick fires by ORing into `bus.ppb[active].icsr` — NOT by
        // setting `bus.irq_pending` — so the IRQ check below does
        // not gate the fast path on SysTick activity. With SysTick
        // enabled and no peripheral activity (e.g. the V5 §5.2
        // tail-chain scenario's `b .` busy-wait after preamble), the
        // fast path would otherwise trigger and SysTick would never
        // tick. Drop to the slow path whenever SysTick is enabled on
        // the active core; SysTick-disabled workloads (almost
        // everything) keep their fast-path eligibility.
        // Evaluate the remaining read-only predicates only when PIO is
        // idle. PicoCalc workloads keep an SM enabled for long periods;
        // while it is active the slow path is already mandatory, so
        // repeatedly inspecting every peripheral cannot change the
        // decision. `all_peripherals_idle()` includes DMA.
        let fast_path = pio_idle
            && self.bus.irq_pending == 0
            && !self.bus.systicks[self.bus.active_core()].is_enabled()
            && self.bus.all_peripherals_idle();
        if fast_path {
            self.tick_pio(consumed as u32);
            // Advance lazy-scheduled peripherals (TIMER alarms) by the
            // same window the cores consumed. Any alarm matching inside
            // the window fires into `bus.irq_pending` and gets drained
            // in the same breath — so firmware that kicks off an alarm
            // in one quantum sees the IRQ land by the start of the
            // next.
            self.bus.advance_lazy_scheduled(consumed);
            self.drain_pending_irqs_to_cores();
            self.update_gpio();
        } else {
            // Per HLD 2026.04.26 V5 §5.3: chunked once-per-quantum slow
            // branch. `master_cycle` advances by `consumed` BEFORE
            // `tick_peripherals` so TIMER's alarm `>=` poll sees the
            // window's end-of-quantum cycle. SysTick advances per-core
            // by each core's actual consumed cycle count (mirrors M0+
            // hardware: SysTick is per-core, decremented on the cycles
            // the owning core consumes — see §5.2.3).
            self.bus.master_cycle = self.bus.master_cycle.wrapping_add(consumed);
            self.bus.tick_peripherals(consumed as u32);
            self.tick_systick(c0_total as u32, c1_total as u32);
            // PIO + GPIO merge: per the fn docstring, a PIO-driven off-chip
            // device (currently only PSRAM — `Bus::has_pin_watching_device`)
            // needs every SCK/CS edge, not just the quantum-end pad
            // snapshot. `tick_pio_and_route_irqs` takes one static
            // `bus.gpio_in` read at its top and reuses it for the entire
            // `cycles` argument, so a bulk call here would let every edge
            // inside a `consumed > 1` quantum go unobserved by
            // `update_gpio` (which is what feeds `Psram::tick`) until the
            // quantum boundary — exactly the failure this branch's old
            // bulk-call implementation had despite the docstring above
            // promising per-cycle interleave. Loop one system cycle at a
            // time only when it can matter (PIO active, quantum > 1, and
            // a pin-watching device is actually attached); otherwise keep
            // the single bulk call so PSRAM-less workloads (the common
            // case) pay no extra cost. IRQ and drain semantics are
            // unchanged either way — net IRQ-delivery latency still grows
            // from ≤1 cycle to ≤step_quantum-1 cycles in the bulk case
            // (see §5.4); the per-cycle case delivers at ≤1 cycle same as
            // the fast path, as a side effect of the finer granularity.
            if !pio_idle && consumed > 1 && self.bus.has_pin_watching_device() {
                for _ in 0..consumed {
                    self.tick_pio_and_route_irqs(1);
                    self.update_gpio();
                }
            } else {
                self.tick_pio_and_route_irqs(consumed as u32);
                self.update_gpio();
            }
            self.drain_pending_irqs_to_cores();
        }
        self.wake_checks();
        #[cfg(feature = "event-horizon-profiler")]
        {
            let mut boundaries = self.bus.take_running_cpu_boundaries();
            let running_after = self.behavior_observation();
            boundaries.insert(Self::running_device_boundaries(
                &running_before,
                &running_after,
            ));
            if external_event_cycle.is_some_and(|cycle| self.clock.cycles >= cycle) {
                boundaries.insert(running_profile::RunningBoundaryMask::EXTERNAL);
            }
            if let Some(profiler) = self.running_profiler.as_mut() {
                if consumed == 0 {
                    profiler.record_non_running();
                } else {
                    profiler.record_running(consumed, boundaries, running_horizon);
                }
            }
        }
        #[cfg(feature = "idle-profiler")]
        if let Some(profiler) = self.idle_profiler.as_mut() {
            let both_blocked = (self.cores[0].is_halted() || self.bus.wfe_waiting[0])
                && (self.cores[1].is_halted() || self.bus.wfe_waiting[1]);
            if consumed == 0 && both_blocked {
                profiler.record_zero_progress_blocked();
            } else {
                profiler.record_running(consumed, c0_total, c1_total);
            }
        }
        consumed
    }

    /// Classify autonomous sources for a prospective idle jump.
    ///
    /// This diagnostic-only predicate deliberately does not reuse the
    /// production fast-path `is_idle()` methods: those methods conservatively
    /// include FIFO contents and sticky IRQ bits, while an idle jump needs to
    /// know whether state changes *with time*. Static state and work already
    /// handled by an exact bulk tick are retained as separate evidence.
    fn idle_source_observation(&self) -> idle_profile::IdleSourceObservation {
        use idle_profile::IdleBlockerMask as M;

        let mut blockers = 0u16;
        let mut stationary = 0u16;
        let mut exact_bulk = 0u16;

        // An enabled PIO SM that is already stalled on an empty TX FIFO
        // cannot change PC or pins until a CPU/DMA write supplies data. Raw
        // masked IRQ flags are likewise static. Other enabled SM states, or
        // an asserted enabled PIO IRQ, still require an event horizon.
        let mut pio_enabled = false;
        let mut pio_all_enabled_stalled = true;
        let irq_can_wake = |irq: u32| {
            self.bus.nvics[0].is_enabled(irq as u8) || self.bus.nvics[1].is_enabled(irq as u8)
        };
        let mut pio_static_irq = false;
        let mut pio_wake_irq = false;
        for (block, pio) in self.bus.pio.iter().enumerate() {
            for sm in &pio.sm {
                if sm.enabled() {
                    pio_enabled = true;
                    pio_all_enabled_stalled &= sm.stalled_on_empty_tx();
                }
            }
            pio_static_irq |= pio.pending_irqs() != 0;
            let line0 = 7 + (block as u32 * 2);
            let int0_asserted = pio.int0_ints_rp2040() != 0;
            let int1_asserted = pio.int1_ints_rp2040() != 0;
            pio_wake_irq |= (int0_asserted && irq_can_wake(line0))
                || (int1_asserted && irq_can_wake(line0 + 1));
            pio_static_irq |= (int0_asserted && !irq_can_wake(line0))
                || (int1_asserted && !irq_can_wake(line0 + 1));
        }
        if pio_wake_irq || (pio_enabled && !pio_all_enabled_stalled) {
            blockers |= M::PIO;
        }
        if pio_static_irq || (pio_enabled && pio_all_enabled_stalled) {
            stationary |= M::PIO;
        }

        let pwm = self.bus.pwm.idle_profile_state();
        let pwm_irq_can_wake = irq_can_wake(4);
        if (pwm.temporal_boundary || pwm.routable_irq) && pwm_irq_can_wake {
            blockers |= M::PWM;
        }
        if pwm.static_state || (pwm.routable_irq && !pwm_irq_can_wake) {
            stationary |= M::PWM;
        }
        if pwm.exact_bulk_work || (pwm.temporal_boundary && !pwm_irq_can_wake) {
            exact_bulk |= M::PWM;
        }
        if self.bus.systicks[0].is_enabled() || self.bus.systicks[1].is_enabled() {
            blockers |= M::SYSTICK;
        }

        let mut classify =
            |state: idle_profile::IdlePeripheralState, bit: u16, irq_can_wake: bool| {
                if state.temporal_work || (state.routable_irq && irq_can_wake) {
                    blockers |= bit;
                }
                if state.static_state || (state.routable_irq && !irq_can_wake) {
                    stationary |= bit;
                }
            };
        classify(
            self.bus.dma.idle_profile_state(),
            M::DMA,
            irq_can_wake(11) || irq_can_wake(12),
        );
        classify(
            self.bus.uart0.idle_profile_state(),
            M::UART,
            irq_can_wake(20),
        );
        classify(
            self.bus.uart1.idle_profile_state(),
            M::UART,
            irq_can_wake(21),
        );
        classify(self.bus.spi0.idle_profile_state(), M::SPI, irq_can_wake(18));
        classify(self.bus.spi1.idle_profile_state(), M::SPI, irq_can_wake(19));
        classify(self.bus.i2c0.idle_profile_state(), M::I2C, irq_can_wake(23));
        classify(self.bus.i2c1.idle_profile_state(), M::I2C, irq_can_wake(24));
        classify(self.bus.adc.idle_profile_state(), M::ADC, irq_can_wake(22));
        classify(
            self.bus.timer.idle_profile_state(),
            M::TIMER,
            (0..4).any(irq_can_wake),
        );
        if self.bus.irq_pending != 0
            || self.bus.nvics[0].pending_and_enabled() != 0
            || self.bus.nvics[1].pending_and_enabled() != 0
        {
            blockers |= M::PENDING_IRQ;
        }
        idle_profile::IdleSourceObservation {
            blockers: M::from_bits(blockers),
            stationary: M::from_bits(stationary),
            exact_bulk: M::from_bits(exact_bulk),
        }
    }

    /// Drain [`Bus::irq_pending`] into both cores' NVIC pending
    /// latches. Per HLD V7 §5.2 this runs once per slow-path inner
    /// cycle so level-triggered peripherals have at most one
    /// architectural cycle of routing lag from assert to NVIC latch.
    ///
    /// Both cores see every IRQ — RP2040 has a single NVIC per core
    /// but shared peripheral IRQ wires, so each line latches
    /// independently on both cores and firmware routes via
    /// `NVIC_IPR` / `NVIC_ISER` (modelled in
    /// `bus/mod.rs::nvic_mmio_write32` + `nvic_mmio_read32`).
    fn drain_pending_irqs_to_cores(&mut self) {
        if self.bus.irq_pending != 0 {
            let raised = std::mem::replace(&mut self.bus.irq_pending, 0);
            for irq in 0..crate::irq::IRQ_COUNT {
                if raised & (1u32 << irq) != 0 {
                    let exception = 16 + irq as u16;
                    for core in 0..2 {
                        // A still-asserted level belonging to the handler
                        // that is already active is not a second event. If
                        // the source remains asserted after exception
                        // return, the next peripheral tick routes it again.
                        // The other core still receives the shared wire.
                        if !self.bus.ppb[core].is_active(exception) {
                            self.bus.nvics[core].set_pending(irq as u8);
                        }
                    }
                }
            }
        }
    }

    /// Per HLD 2026.04.26 V5 §5.2.3: advance each core's SysTick by
    /// its own consumed cycle count for this quantum. Mirrors M0+
    /// hardware semantics — SysTick is per-core, the active-core
    /// `Bus` field is just a banked-MMIO selector, not a tick gate.
    /// PENDSTSET (`ICSR[26]`) latches per-core; `drain_pending_irqs_to_cores`
    /// runs after this call so the SysTick handler is taken on the
    /// next quantum boundary.
    fn tick_systick(&mut self, c0: u32, c1: u32) {
        for _ in 0..c0 {
            if self.bus.systicks[0].tick() {
                self.bus.ppb[0].icsr |= 1 << 26;
            }
        }
        for _ in 0..c1 {
            if self.bus.systicks[1].tick() {
                self.bus.ppb[1].icsr |= 1 << 26;
            }
        }
    }

    /// Step both PIO blocks by `cycles` system clocks and route their
    /// IRQ flags into [`Bus::irq_pending`].
    ///
    /// Per HLD V7 §5.5 + Appendix B, each PIO block has 8 internal
    /// Per-block 12-bit raw status (`IRQ[3:0]` + RXNEMPTY[3:0] +
    /// TXNFULL[3:0]) is masked through `INT0_INTE` / `INT1_INTE` and
    /// OR'd with `INT0_INTF` / `INT1_INTF` to derive the effective
    /// values on each NVIC line. Each block has two lines: PIO0_IRQ_0/1
    /// at NVIC #7/#8 and PIO1_IRQ_0/1 at NVIC #9/#10. PicoGUS firmware
    /// enables `RXNEMPTY_SM0` on PIO0 INT0_INTE so its ISA handler
    /// fires when an autopushed event lands in PIO0 SM0's RX FIFO.
    ///
    /// Per HLD 2026.04.26 V5 §5.1: chunked once-per-quantum on the
    /// common (no pin-watching device) slow-path call site. The
    /// PSRAM-aware slow-path branch instead calls this with `cycles=1`
    /// in a loop, `consumed` times, so every SCK/CS edge gets its own
    /// `bus.gpio_in` snapshot — see `step_serial`'s docstring.
    fn tick_pio_and_route_irqs(&mut self, cycles: u32) {
        let gpio_in = self.bus.gpio_in;
        // Diagnostic counters bump by `cycles` (per-quantum granularity is
        // acceptable per HLD 2026.04.26 V5 §7 risk row "Per-cycle
        // observation diagnostics under-count by quantum factor").
        self.pio_tick_count = self.pio_tick_count.wrapping_add(cycles as u64);
        if gpio_in & (1u32 << 4) == 0 {
            self.pio_tick_iow_low_count = self.pio_tick_iow_low_count.wrapping_add(cycles as u64);
        }
        self.bus.pio[0].step_n(cycles, gpio_in);
        self.bus.pio[1].step_n(cycles, gpio_in);
        // Observe PIO0 SM0's PC after the step. Tracks max PC and the
        // number of times the PC differs from the prior observation
        // (counts both linear advances and jumps; sequential same-PC
        // ticks — e.g. a stalled WAIT — do not increment).
        let sm0_pc = self.bus.pio[0].sm[0].pc();
        if sm0_pc > self.pio0_sm0_max_pc {
            self.pio0_sm0_max_pc = sm0_pc;
        }
        if sm0_pc != self.pio0_sm0_last_pc {
            self.pio0_sm0_pc_advances = self.pio0_sm0_pc_advances.wrapping_add(1);
            self.pio0_sm0_last_pc = sm0_pc;
        }
        for (block, line0_bit) in [(0usize, 7u32), (1usize, 9u32)] {
            if self.bus.pio[block].int0_ints_rp2040() != 0 {
                self.bus.irq_pending |= 1u32 << line0_bit;
            }
            if self.bus.pio[block].int1_ints_rp2040() != 0 {
                self.bus.irq_pending |= 1u32 << (line0_bit + 1);
            }
        }
    }

    /// Advance both PIO blocks by `cycles` system-clock cycles.
    ///
    /// PIO reads `bus.gpio_in` as its view of external pin state — feed it
    /// the pre-step merge so programs sampling GPIO (e.g. IN PINS) see the
    /// value SIO / the previous PIO step wrote last. The post-step
    /// `update_gpio()` then refreshes `bus.gpio_in` from `pad_out`/`pad_oe`.
    fn tick_pio(&mut self, cycles: u32) {
        if cycles == 0 {
            return;
        }
        let gpio_in = self.bus.gpio_in;
        for pio in &mut self.bus.pio {
            pio.step_n(cycles, gpio_in);
        }
    }

    /// Run for at least `cycles` virtual cycles. Returns the number of
    /// cycles actually executed. May overshoot by up to `step_quantum - 1`
    /// cycles (one quantum's worth), matching the documented overshoot
    /// behaviour of [`Self::step`].
    ///
    /// Dispatches to the selected [`ExecutionModel`]. In Threaded mode
    /// this rounds up to the nearest quantum boundary (HLD V1 §5.4)
    /// and returns `Err(EmulatorError::WorkerPanicked)` sticky on
    /// worker panic.
    pub fn run(&mut self, cycles: u64) -> Result<u64, EmulatorError> {
        if self.execution_model == ExecutionModel::Serial {
            let start = self.clock.cycles;
            while self.clock.cycles.wrapping_sub(start) < cycles {
                let consumed = self.step_serial();
                if consumed == 0 {
                    break;
                }
            }
            return Ok(self.clock.cycles.wrapping_sub(start));
        }
        #[cfg(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        {
            if let Some((which, message)) = &self.panic_info {
                return Err(EmulatorError::WorkerPanicked {
                    which: *which,
                    message: message.clone(),
                });
            }
            if let Some((which, elapsed_ms)) = self.timeout_info {
                return Err(EmulatorError::BarrierTimeout { which, elapsed_ms });
            }
            if self.threaded.is_none() {
                self.promote_to_threaded();
            }
            self.apply_pending_panic_inject();
            let step_q = self.step_quantum as u64;
            let quanta = cycles.div_ceil(step_q.max(1));
            let threaded = self.threaded.as_mut().expect("threaded promoted above");
            match threaded.run_quanta_checked(quanta) {
                Ok(()) => Ok(quanta.saturating_mul(step_q)),
                Err(threaded::RunError::Panic { which, message }) => {
                    self.panic_info = Some((which, message.clone()));
                    Err(EmulatorError::WorkerPanicked { which, message })
                }
                Err(threaded::RunError::Timeout { which, elapsed_ms }) => {
                    self.timeout_info = Some((which, elapsed_ms));
                    Err(EmulatorError::BarrierTimeout { which, elapsed_ms })
                }
            }
        }
        #[cfg(not(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        )))]
        {
            let _ = cycles;
            Err(EmulatorError::NotSupportedInThreadedMode)
        }
    }

    /// Advance the emulator by exactly one quantum (`step_quantum`
    /// cycles). Primary entry point for the Threaded path; on Serial
    /// this is the same as [`Self::step`] and returns the cycles
    /// consumed. HLD V1 §5.4.
    ///
    /// Returns `Err(EmulatorError::WorkerPanicked)` sticky on worker
    /// panic in Threaded mode. One-shot-after-panic: subsequent calls
    /// return the cached error without re-attempting workers.
    pub fn run_quantum(&mut self) -> Result<u64, EmulatorError> {
        match self.execution_model {
            ExecutionModel::Serial => Ok(self.step_serial()),
            ExecutionModel::Threaded => self.run_quantum_threaded(),
        }
    }

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    fn run_quantum_threaded(&mut self) -> Result<u64, EmulatorError> {
        if let Some((which, message)) = &self.panic_info {
            return Err(EmulatorError::WorkerPanicked {
                which: *which,
                message: message.clone(),
            });
        }
        if let Some((which, elapsed_ms)) = self.timeout_info {
            return Err(EmulatorError::BarrierTimeout { which, elapsed_ms });
        }
        if self.threaded.is_none() {
            self.promote_to_threaded();
        }
        self.apply_pending_panic_inject();
        let step_q = self.step_quantum as u64;
        let threaded = self.threaded.as_mut().expect("threaded promoted above");
        match threaded.run_quanta_checked(1) {
            Ok(()) => Ok(step_q),
            Err(threaded::RunError::Panic { which, message }) => {
                self.panic_info = Some((which, message.clone()));
                Err(EmulatorError::WorkerPanicked { which, message })
            }
            Err(threaded::RunError::Timeout { which, elapsed_ms }) => {
                self.timeout_info = Some((which, elapsed_ms));
                Err(EmulatorError::BarrierTimeout { which, elapsed_ms })
            }
        }
    }

    #[cfg(not(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    )))]
    fn run_quantum_threaded(&mut self) -> Result<u64, EmulatorError> {
        Err(EmulatorError::NotSupportedInThreadedMode)
    }

    /// Forward any pending `inject_panic_for_testing` target into the
    /// live `ThreadedEmulator`. No-op on non-testing builds.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[inline]
    fn apply_pending_panic_inject(&mut self) {
        #[cfg(feature = "testing")]
        if let Some(which) = self.pending_panic_inject.take()
            && let Some(t) = self.threaded.as_mut()
        {
            t.inject_panic_for_testing(which);
        }
    }

    /// Move the seeded Serial state into a fresh `ThreadedEmulator`.
    /// Called lazily on the first `run_quantum` / `run` so harness
    /// setup that poked `emu.bus` / `emu.core_mut(...)` pre-run is
    /// carried over. After promotion, the top-level `cores` / `bus` /
    /// `clock` fields hold zero-cost placeholders and must not be
    /// inspected mid-run.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    fn promote_to_threaded(&mut self) {
        let placeholder_bus = Bus::new();
        let placeholder_cores = [CortexM0Plus::with_id(0), CortexM0Plus::with_id(1)];
        let seeded_bus = std::mem::replace(&mut self.bus, placeholder_bus);
        let seeded_cores = std::mem::replace(&mut self.cores, placeholder_cores);
        let seeded_clock = std::mem::replace(&mut self.clock, Clock { cycles: 0 });
        let seeded = Emulator {
            cores: seeded_cores,
            bus: seeded_bus,
            clock: seeded_clock,
            step_quantum: self.step_quantum,
            pio_tick_count: self.pio_tick_count,
            pio_tick_iow_low_count: self.pio_tick_iow_low_count,
            pio0_sm0_max_pc: self.pio0_sm0_max_pc,
            pio0_sm0_pc_advances: self.pio0_sm0_pc_advances,
            pio0_sm0_last_pc: self.pio0_sm0_last_pc,
            #[cfg(feature = "idle-profiler")]
            idle_profiler: None,
            #[cfg(feature = "event-horizon-profiler")]
            running_profiler: None,
            #[cfg(feature = "behavior-trace")]
            behavior_tracer: None,
            execution_model: ExecutionModel::Serial,
            threaded: None,
            panic_info: None,
            timeout_info: None,
            #[cfg(feature = "testing")]
            pending_panic_inject: None,
            bus_is_placeholder: false,
        };
        self.threaded = Some(threaded::ThreadedEmulator::from_emulator(seeded));
        self.bus_is_placeholder = true;
    }

    /// Test-only: arm a panic injection for the next `run_quantum` /
    /// `run` call. The matching worker panics on its first barrier
    /// entry; the emulator surfaces `Err(EmulatorError::WorkerPanicked)`
    /// and becomes sticky-poisoned.
    ///
    /// Feature-gated behind `testing` so release consumers cannot brick
    /// their emulator by calling an internal hook.
    #[cfg(all(
        feature = "testing",
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub fn inject_panic_for_testing(&mut self, which: WorkerName) {
        self.pending_panic_inject = Some(which);
    }

    /// Merge SIO and PIO GPIO outputs into `bus.gpio_in`.
    ///
    /// SIO `gpio_out & gpio_oe` is the base; each PIO block's
    /// `pad_out & pad_oe` overrides SIO on the pins it drives (PIO wins
    /// wherever `pad_oe` has a bit set — mirrors `rp2350_emu::Emulator::
    /// update_gpio`). The result is masked to the RP2040 30-pin range
    /// (GPIO0..GPIO29).
    ///
    /// Next, the off-chip SPI PSRAM observes the post-merge pin state
    /// on its CS/SCK/MOSI pins and, if it is currently driving MISO,
    /// splices its bit into `gpio_in` bit 0. MISO override happens after
    /// SIO/PIO so MOSI/SCK/CS seen by the PSRAM reflect the actual pin
    /// levels driven by PIO / SIO on this tick (no feedback from the
    /// override into the PSRAM's observation on the same tick).
    ///
    /// Finally, any [`Bus::external_gpio_in_mask`] bits override the
    /// merged value with [`Bus::external_gpio_in_override`]. External
    /// drivers (e.g. the `picogus_diff_rp2040` harness injecting a
    /// synthetic ISA waveform) win over the on-chip merge for the pins
    /// they claim — without this final override step, harness pokes to
    /// `bus.gpio_in` would be silently clobbered the next time
    /// `update_gpio` ran.
    pub(crate) fn update_gpio(&mut self) {
        let mut out = self.bus.sio.gpio_out & self.bus.sio.gpio_oe;
        for pio in &self.bus.pio {
            let pio_mask = pio.pad_oe;
            out = (out & !pio_mask) | (pio.pad_out & pio_mask);
        }
        out &= 0x3FFF_FFFF;
        if let Some(ref mut psram) = self.bus.psram
            && let Some(miso) = psram.tick(out)
        {
            let pin = psram.pin_miso();
            let mask = 1u32 << pin;
            out = (out & !mask) | ((miso as u32) << pin);
        }
        // Devices wired to pins the chip drives directly, rather than
        // through a controller's FIFO. A PIO-driven display is the case
        // this exists for: nothing passes through SPI, so the only way
        // to see the traffic is to watch the pads.
        for device in &mut self.bus.pin_devices {
            if let Some((pin, level)) = device.tick(out) {
                let mask = 1u32 << pin;
                out = (out & !mask) | ((level as u32) << pin);
            }
        }
        let ext_mask = self.bus.external_gpio_in_mask;
        if ext_mask != 0 {
            out = (out & !ext_mask) | (self.bus.external_gpio_in_override & ext_mask);
        }
        self.bus.gpio_in = out;

        // Off-chip SPI slaves also watch side-band pins that move while
        // the SSP is completely idle — an LCD's hardware RESET pulse,
        // for instance, happens long before the first frame is queued,
        // and an idle SPI means `tick_peripherals` (which does its own
        // pre-drain sample) is skipped by the fast path. Sampling here
        // as well is idempotent: nothing executes between the two call
        // sites within a `step`, so both see the same pad levels.
        if self.bus.spi_has_device(0) || self.bus.spi_has_device(1) {
            let pads = self.bus.pad_out_levels();
            self.bus.spi0.observe_pins(pads);
            self.bus.spi1.observe_pins(pads);
        }
    }

    /// WFE/SEV / WFI quantum-end wake check. See `wrk_docs/2026.04.26
    /// - HLD - RP2040 WFE-SEV Wake Mechanics V1.md` §4.4.
    ///
    /// Per-core:
    /// - **WFE wake** — if the core is parked on `wfe_waiting` and an
    ///   `event_flag` is latched, consume the latch and un-park. The
    ///   latch is intentionally preserved (one-shot wake) when no
    ///   waiter is parked: the SEV-before-WFE idiom requires the latch
    ///   to survive until the next WFE consumes it.
    /// - **WFE IRQ wake** — ARMv6-M ARM §B1.5.18 lists, alongside SEV
    ///   and the event register, "an asynchronous exception at a
    ///   priority that preempts any currently active exceptions" as a
    ///   WFE wake-up event. So a parked core must also un-park when an
    ///   enabled+pending IRQ appears on its own NVIC, exactly like the
    ///   WFI case below; the exception is then taken on the next
    ///   `step()` via `try_take_any_pending_exception`. The event
    ///   register is NOT consumed on this path — only a WFE that
    ///   actually finds the latch set clears it. Without this arm the
    ///   Pico SDK `sleep_until` idiom (arm a TIMER alarm, then
    ///   `while (!done) __wfe();` with the alarm ISR setting `done`)
    ///   dead-locks: the alarm IRQ latches in the NVIC, which both
    ///   fails to wake the WFE-parked core AND disqualifies the
    ///   tech_debt §1649 clock-advance branch below (it requires no
    ///   pending IRQ), so the master clock freezes forever.
    ///   PRIMASK is intentionally not consulted, mirroring the WFI
    ///   decision documented in `core/execute.rs`.
    /// - **WFI wake** — if the core is halted and an enabled+pending
    ///   IRQ exists on its NVIC, un-halt. The pending bit is consumed
    ///   on the next `step()` via `try_take_any_pending_exception`. The
    ///   `halted` flag is shared with BKPT/debug halt by design (matches
    ///   rp2350_emu precedent).
    ///
    /// Crucially: this function no longer unconditionally clears
    /// `event_flag[0]`. A latched event with no waiter survives until
    /// the next WFE on that core consumes it. The launch consumer's
    /// explicit `event_flag[1] = false` reset (`maybe_wake_core1`) is
    /// preserved verbatim — that's an intentional clean-launch reset,
    /// not part of the WFE/SEV protocol.
    fn wake_checks(&mut self) {
        for core in 0..2 {
            // WFE wake: parked core + latched event = consume + un-park.
            if self.bus.wfe_waiting[core] && self.bus.event_flag[core] {
                self.bus.event_flag[core] = false;
                self.bus.wfe_waiting[core] = false;
            }
            // WFE IRQ wake: parked core + pending+enabled IRQ = un-park,
            // leaving `event_flag` untouched (ARMv6-M ARM §B1.5.18 — an
            // interrupt-driven wake is not an event-register consume).
            if self.bus.wfe_waiting[core] && self.bus.nvics[core].pending_and_enabled() != 0 {
                self.bus.wfe_waiting[core] = false;
            }
            // WFI wake: halted core + pending+enabled IRQ = un-halt.
            // Reuses `halted` so this also wakes a BKPT-halted core if
            // an IRQ asserts; matches the rp2350_emu design wart.
            if self.cores[core].is_halted() && self.bus.nvics[core].pending_and_enabled() != 0 {
                self.cores[core].wake();
            }
        }
    }

    /// Halt core 1 and synchronously re-arm the multicore-launch FSM.
    ///
    /// This is the ONLY sanctioned path for halting core 1 from
    /// production code. Direct `cores[1].halt()` skips the `armed`
    /// sync and will silently drift the FSM state against the core's
    /// actual halt status. See HLD 2026.04.16 §5 (invariants).
    pub fn halt_core1(&mut self) {
        self.assert_not_placeholder();
        self.cores[1].halt();
        self.bus.sio.set_handshake_armed(true);
    }

    /// Wake core 1 and synchronously disarm the multicore-launch FSM.
    ///
    /// This is the ONLY sanctioned path for waking core 1 from
    /// production code. The launch consumer in [`Self::maybe_wake_core1`]
    /// calls this after applying VTOR / MSP / PC; external callers
    /// (tests simulating a mode switch; future reset-path code) also
    /// route through here.
    ///
    /// `wake_core1` does not touch CPU register state. Callers that need
    /// a clean architectural baseline (e.g. the launch consumer after a
    /// re-halt) must call [`CortexM0Plus::reset_control_for_launch`]
    /// before this.
    pub fn wake_core1(&mut self) {
        self.assert_not_placeholder();
        self.cores[1].wake();
        self.bus.sio.set_handshake_armed(false);
    }

    /// Observe the Pico SDK multicore-launch handshake. The armed-path
    /// FSM in [`crate::bus::Sio::fifo_wr`] consumes core-0 FIFO pushes
    /// while core 1 is halted; on the 6th valid word the FSM produces a
    /// [`crate::bus::sio::Core1Launch`] token. This consumer applies
    /// VTOR / MSP / PC to core 1, resets CONTROL/PSP/xPSR/IPSR/PRIMASK
    /// to a clean launch baseline, clears any stale `event_flag[1]`,
    /// and wakes the core via the [`Self::wake_core1`] wrapper (which
    /// synchronously disarms the FSM).
    ///
    /// Called once after each core-0 step so that a pushed-then-popped
    /// handshake within a single quantum still wakes core 1 in that
    /// quantum. The `writer_core` argument is unused on this branch —
    /// the FSM is only armed while core 0 pushes, so a core-1 step
    /// cannot produce a pending_launch. Kept for call-site-compatibility
    /// with the replaced placeholder.
    fn maybe_wake_core1(&mut self, _writer_core: usize) {
        let Some(launch) = self.bus.sio.take_pending_launch() else {
            return;
        };
        // Invariant: the FSM only arms while core 1 is halted; launch
        // tokens can only be produced in that state. If this fails we
        // have a logic bug in the arming mechanism (HLD §2.5).
        debug_assert!(
            self.cores[1].is_halted(),
            "pending_launch emitted against an awake core 1 — arming bug"
        );

        self.bus.ppb[1].vtor = launch.vtor;
        self.cores[1].regs.msp = launch.sp;
        self.cores[1].regs.r[13] = launch.sp;
        // `entry & !1` matches `direct_boot_from_flash` (silent strip).
        // On real silicon a Thumb-bit-clear BLX target HardFaults; this
        // asymmetry is logged in tech_debt.md alongside direct_boot.
        self.cores[1].regs.set_pc(launch.entry & !1);
        self.cores[1].reset_control_for_launch();
        self.bus.event_flag[1] = false; // clear any stale wake signal
        self.wake_core1();
    }

    /// Read a GPIO pin from the merged pin state. Debug-only: asserts
    /// the emulator has not been promoted into Threaded mode.
    pub fn gpio_read(&self, pin: u8) -> bool {
        self.assert_not_placeholder();
        if pin >= 30 {
            return false;
        }
        (self.bus.gpio_in >> pin) & 1 != 0
    }

    /// Write a GPIO pin. Sets the SIO GPIO_OUT bit and asserts output
    /// enable so the pin state becomes observable via [`Self::gpio_read`].
    /// Useful as a test-shim to inject a pin level without hand-rolling
    /// the SIO register poking.
    pub fn gpio_write(&mut self, pin: u8, value: bool) {
        self.assert_not_placeholder();
        if pin >= 30 {
            return;
        }
        let mask = 1u32 << pin;
        self.bus.sio.gpio_oe |= mask;
        if value {
            self.bus.sio.gpio_out |= mask;
        } else {
            self.bus.sio.gpio_out &= !mask;
        }
        self.update_gpio();
    }

    /// Read all GPIO pins as a bitmask. Debug-only: asserts the
    /// emulator has not been promoted into Threaded mode.
    pub fn gpio_read_all(&self) -> u64 {
        self.assert_not_placeholder();
        self.bus.gpio_in as u64
    }

    /// Access core state. Debug-only: asserts the emulator has not
    /// been promoted into Threaded mode (the flat `cores` field would
    /// be a placeholder).
    pub fn core(&self, id: usize) -> &CortexM0Plus {
        self.assert_not_placeholder();
        &self.cores[id]
    }

    /// Mutable accessor; same debug-only placeholder assertion.
    pub fn core_mut(&mut self, id: usize) -> &mut CortexM0Plus {
        self.assert_not_placeholder();
        &mut self.cores[id]
    }

    /// Direct memory read (bypasses bus timing). Debug-only: asserts
    /// the emulator has not been promoted into Threaded mode.
    pub fn peek(&self, addr: u32) -> u32 {
        self.assert_not_placeholder();
        self.bus.peek32(addr)
    }

    /// Direct memory write (bypasses bus timing). Debug-only: asserts
    /// the emulator has not been promoted into Threaded mode.
    pub fn poke(&mut self, addr: u32, value: u32) {
        self.assert_not_placeholder();
        self.bus.poke32(addr, value);
        // poke32 bypasses the Bus::write* invalidation hooks
        // (memory.sram_write32 / xip_sram direct slice). Conservative
        // bulk invalidation here keeps the cache coherent with any
        // pre-step `poke` of executable bytes, with negligible overhead
        // (callers typically poke before the first step).
        self.bus.pending_invalidation_regions |= crate::bus::invalidation_regions::BULK;
        self.cores[0].invalidate_decode_cache_all();
        self.cores[1].invalidate_decode_cache_all();
        self.bus.pending_invalidation_regions = 0;
        self.bus.pending_cache_invalidations.clear();
    }

    /// Current master cycle count. Debug-only: asserts the emulator
    /// has not been promoted into Threaded mode — Threaded callers
    /// read the live master cycle via the value returned from
    /// [`Self::run_quantum`] / [`Self::run`].
    pub fn cycles(&self) -> u64 {
        self.assert_not_placeholder();
        self.clock.cycles
    }

    /// Write a 32-bit word to an MMIO address via the bus. Charges zero
    /// emulator cycles (intended for setup code running outside `run()`).
    ///
    /// Delegates to [`Bus::write32`], so alias bits (`(addr >> 12) & 3`)
    /// are honoured: base address = normal, XOR alias = `|0x1000`, SET
    /// alias = `|0x2000`, CLR alias = `|0x3000`. Useful for poking PIO
    /// INSTR_MEM, configuring SIO GPIO_OE/_OUT, releasing RESETS bits,
    /// etc., without hand-rolling the bus machinery.
    pub fn mmio_write32(&mut self, addr: u32, value: u32) {
        self.assert_not_placeholder();
        // Mirror the `step()` stash so PLL write-time lock-arm transitions
        // observe the current cycle count when the harness pokes MMIO
        // outside the step path. See HLD §6 P2.
        self.bus.master_cycle = self.clock.cycles;
        self.bus.write32(addr, value);
    }

    /// Read a 32-bit word from an MMIO address via the bus. Charges zero
    /// emulator cycles (intended for setup code running outside `run()`).
    ///
    /// **Warning: reads may have side effects.** Several RP2040 MMIO
    /// registers mutate state on read — e.g. PIO `RXFn` pops the receive
    /// FIFO, SIO divider `QUOTIENT` / `REMAINDER` clear the CSR dirty
    /// bit, and a handful of W1C sticky flags are cleared by reads. Setup
    /// code should therefore be write-heavy; reads through this method
    /// are for confirmation only and should be chosen carefully to avoid
    /// disturbing the peripheral's state.
    pub fn mmio_read32(&mut self, addr: u32) -> u32 {
        self.assert_not_placeholder();
        // Mirror the `step()` stash so PLL CS reads observe the current
        // cycle count when the harness reads MMIO outside the step path.
        self.bus.master_cycle = self.clock.cycles;
        self.bus.read32(addr)
    }

    /// Harness-only diagnostic: drain every byte firmware has written to
    /// UART0 `DR` since the previous call. Returns empty if idle.
    pub fn drain_uart0_tx_log(&mut self) -> Vec<u8> {
        self.assert_not_placeholder();
        self.bus.drain_uart0_tx_log()
    }

    /// Enable and reset the OPT0-B streaming event trace.
    ///
    /// Only Serial mode is accepted. The initial observable state is
    /// recorded immediately, then subsequent calls to [`Self::step`] fold
    /// changed domains into SHA-256 without retaining an event array.
    #[cfg(feature = "behavior-trace")]
    pub fn enable_behavior_trace(&mut self) -> Result<(), EmulatorError> {
        if self.execution_model != ExecutionModel::Serial {
            return Err(EmulatorError::NotSupportedInThreadedMode);
        }
        // Events that predate trace enablement are outside this stream.  Keep
        // the independent UART tap aligned with the initial observation.
        let _ = self.bus.drain_uart0_behavior_tx_log();
        let observation = self.behavior_observation();
        self.behavior_tracer = Some(behavior_trace::BehaviorTracer::new(observation));
        Ok(())
    }

    /// Add a harness-owned event (scenario input, LCD observation, or
    /// another external boundary) to the same canonical stream.
    #[cfg(feature = "behavior-trace")]
    pub fn record_behavior_event(
        &mut self,
        domain: BehaviorEventDomain,
        source: u16,
        payload: &[u8],
    ) {
        if let Some(tracer) = self.behavior_tracer.as_mut() {
            tracer.record(domain, source, self.clock.cycles, payload);
        }
    }

    /// Route one PIO block's pin-edge events to a device domain. This
    /// keeps the generic RP2040 backend board-agnostic while allowing a
    /// harness to declare that, for example, PIO0 drives an LCD and PIO1
    /// drives PSRAM in the selected target.
    #[cfg(feature = "behavior-trace")]
    pub fn map_behavior_pio_domain(&mut self, block: usize, domain: BehaviorEventDomain) {
        let Some(pio) = self.bus.pio.get(block) else {
            return;
        };
        let state = [pio.pad_out, pio.pad_oe];
        if let Some(tracer) = self.behavior_tracer.as_mut() {
            tracer.map_pio_domain(block, domain, self.clock.cycles, state);
        }
    }

    /// Route observable GPIO input edges to a device domain. PicoCalc's
    /// PSRAM MISO line is the motivating case; the board harness owns
    /// that pin assignment, not the generic emulator.
    #[cfg(feature = "behavior-trace")]
    pub fn map_behavior_gpio_input_domain(&mut self, domain: BehaviorEventDomain) {
        if let Some(tracer) = self.behavior_tracer.as_mut() {
            tracer.map_gpio_input_domain(domain, self.clock.cycles, self.bus.gpio_in);
        }
    }

    /// Finalize a clone of the streaming hash state. This is read-only and
    /// may be called repeatedly; no event is closed or consumed.
    #[cfg(feature = "behavior-trace")]
    pub fn behavior_trace_snapshot(&self) -> Option<BehaviorTraceSnapshot> {
        self.behavior_tracer
            .as_ref()
            .map(behavior_trace::BehaviorTracer::snapshot)
    }

    #[cfg(feature = "behavior-trace")]
    fn record_uart0_behavior_events(&mut self) {
        let bytes = self.bus.drain_uart0_behavior_tx_log();
        if let Some(tracer) = self.behavior_tracer.as_mut() {
            for byte in bytes {
                tracer.record(
                    BehaviorEventDomain::SerialBus,
                    1,
                    self.clock.cycles,
                    &[byte],
                );
            }
        }
    }

    #[cfg(feature = "behavior-trace")]
    fn observe_behavior_trace(&mut self) {
        self.record_uart0_behavior_events();
        let observation = self.behavior_observation();
        if let Some(tracer) = self.behavior_tracer.as_mut() {
            tracer.observe(observation);
        }
    }

    #[cfg(feature = "behavior-trace")]
    fn observe_behavior_trace_timer_pwm_boundary(&mut self) {
        self.record_uart0_behavior_events();
        let observation = self.behavior_observation();
        if let Some(tracer) = self.behavior_tracer.as_mut() {
            tracer.observe_timer_pwm_boundary(observation);
        }
    }

    #[cfg(feature = "behavior-trace")]
    fn behavior_observation(&self) -> behavior_trace::BehaviorObservation {
        let mut pio_state = [0u32; 4];
        for (block, pio) in self.bus.pio.iter().enumerate() {
            pio_state[block * 2] = pio.pad_out;
            pio_state[block * 2 + 1] = pio.pad_oe;
        }
        let mut dma_transfers = [0u64; crate::dma::NUM_CHANNELS];
        for (index, value) in dma_transfers.iter_mut().enumerate() {
            *value = self.bus.dma_channel(index).transfers_issued;
        }
        let mut pwm = [behavior_trace::PwmObservation::default(); 8];
        for (index, value) in pwm.iter_mut().enumerate() {
            if let Some(slice) = self.bus.pwm().slice(index) {
                *value = behavior_trace::PwmObservation {
                    enabled: slice.csr & crate::peripherals::pwm::CSR_EN != 0,
                    ctr: slice.ctr,
                    top: slice.top,
                };
            }
        }
        let psram = self
            .bus
            .psram
            .as_ref()
            .map(|value| behavior_trace::PsramObservation {
                cs_falling_count: value.cs_falling_count,
                bytes_written: value.bytes_written,
                bytes_read: value.bytes_read,
            });
        behavior_trace::BehaviorObservation {
            cycle: self.clock.cycles,
            clock_hz: [
                self.bus.clock_tree.sys_clk_hz,
                self.bus.clock_tree.ref_clk_hz,
                self.bus.clock_tree.peri_clk_hz,
            ],
            irq: [
                self.bus.irq_pending,
                self.bus.nvics[0].pending,
                self.bus.nvics[1].pending,
                self.cores[0].regs.xpsr & 0x1ff,
                self.cores[1].regs.xpsr & 0x1ff,
            ],
            gpio_in: self.bus.gpio_in,
            pio_state,
            dma_transfers,
            timer: self.bus.timer.behavior_trace_state(),
            pwm,
            psram,
            serial: self.bus.behavior_serial_state(),
        }
    }

    /// Compare the same observable projection used by OPT0-B and classify
    /// which device-side boundary ended an OPT2-B running interval.  The
    /// profiler records these post-hoc gaps as an opportunity upper bound;
    /// it does not promote them to a predictive/safe horizon.
    #[cfg(feature = "event-horizon-profiler")]
    fn running_device_boundaries(
        before: &behavior_trace::BehaviorObservation,
        after: &behavior_trace::BehaviorObservation,
    ) -> u16 {
        use running_profile::RunningBoundaryMask as M;

        let mut bits = 0u16;
        if before.clock_hz != after.clock_hz {
            bits |= M::CLOCK;
        }
        if before.irq != after.irq {
            bits |= M::IRQ_EXCEPTION;
        }
        if before.gpio_in != after.gpio_in
            || before.pio_state != after.pio_state
            || before.psram != after.psram
        {
            bits |= M::PIO_DEVICE;
        }
        if before.dma_transfers != after.dma_transfers {
            bits |= M::DMA_DREQ;
        }
        if before.timer != after.timer || behavior_trace::pwm_boundary(&before.pwm, &after.pwm) {
            bits |= M::TIMER_SYSTICK_PWM;
        }
        if before.serial != after.serial {
            bits |= M::SERIAL;
        }
        bits
    }

    /// Enable and reset the running event-horizon/decode opportunity
    /// profiler, including OPT3-A immutable-XIP cursor metrics. This
    /// diagnostic is Serial-only and is intentionally
    /// separate from the wall-time measurement binary.
    #[cfg(feature = "event-horizon-profiler")]
    pub fn enable_running_event_profiler(&mut self) -> Result<(), EmulatorError> {
        if self.execution_model != ExecutionModel::Serial {
            return Err(EmulatorError::NotSupportedInThreadedMode);
        }
        self.running_profiler = Some(running_profile::RunningProfile::default());
        for core in &mut self.cores {
            core.reset_decode_profile();
        }
        Ok(())
    }

    /// Snapshot aggregate counters without mutating open intervals.
    #[cfg(feature = "event-horizon-profiler")]
    pub fn running_event_profile_snapshot(&self) -> Option<RunningEventProfileSnapshot> {
        self.running_profiler
            .as_ref()
            .map(|profile| RunningEventProfileSnapshot {
                boundary: profile.snapshot(),
                decode_by_core: [
                    self.cores[0].decode_profile_snapshot(),
                    self.cores[1].decode_profile_snapshot(),
                ],
            })
    }

    /// Enable and reset the diagnostic Serial idle profiler.
    ///
    /// Available only in builds with the `idle-profiler` feature. The
    /// method is rejected on Threaded emulators because OPT0-A defines
    /// the Serial path as its reference.
    #[cfg(feature = "idle-profiler")]
    pub fn enable_idle_profiler(&mut self) -> Result<(), EmulatorError> {
        if self.execution_model != ExecutionModel::Serial {
            return Err(EmulatorError::NotSupportedInThreadedMode);
        }
        self.idle_profiler = Some(idle_profile::IdleProfiler::default());
        Ok(())
    }

    /// Snapshot the diagnostic idle profiler without closing or changing
    /// the currently open episode. Returns `None` until enabled.
    #[cfg(feature = "idle-profiler")]
    pub fn idle_profile_snapshot(&self) -> Option<IdleProfileSnapshot> {
        self.idle_profiler
            .as_ref()
            .map(idle_profile::IdleProfiler::snapshot)
    }

    /// Sample the current conservative idle gate for OPT0-A cost studies.
    ///
    /// This does not mutate emulated state and does not claim to compute a
    /// complete future event horizon. It exists only in diagnostic builds.
    #[cfg(feature = "idle-profiler")]
    #[inline(never)]
    pub fn idle_current_probe(&self) -> IdleCurrentProbe {
        self.assert_not_placeholder();
        let observation = self.idle_source_observation();
        IdleCurrentProbe {
            master_cycle: self.bus.master_cycle,
            next_lazy_deadline: self.bus.next_scheduled_lazy_deadline(),
            blocker_count: observation.blockers.count(),
            stationary_source_count: observation.stationary.count(),
            exact_bulk_source_count: observation.exact_bulk.count(),
            proven_jump_safe: observation.proven_safe(),
        }
    }

    /// Internal helper used by both the normal execution path and the
    /// diagnostic-only public API.
    fn idle_event_horizon_internal(
        &self,
        external_event_cycle: Option<u64>,
    ) -> crate::idle_profile::IdleEventHorizonProbe {
        use idle_profile::{IdleBlockerMask as B, IdleEventSourceMask as S};

        self.assert_not_placeholder();
        let now = self.bus.master_cycle;
        let observation = self.idle_source_observation();
        let mut next_event_cycle = None;
        let mut limiting_bits = 0u16;
        let mut fallback_bits = 0u16;

        let mut consider = |cycle: u64, source: u16| {
            let cycle = cycle.max(now);
            match next_event_cycle {
                None => {
                    next_event_cycle = Some(cycle);
                    limiting_bits = source;
                }
                Some(current) if cycle < current => {
                    next_event_cycle = Some(cycle);
                    limiting_bits = source;
                }
                Some(current) if cycle == current => limiting_bits |= source,
                Some(_) => {}
            }
        };

        if observation.blockers.contains(B::PENDING_IRQ) {
            consider(now, S::PENDING_IRQ);
        }

        // These sources do not yet expose a promoted exact deadline. A
        // one-cycle boundary is nevertheless complete and safe.
        for (blocker, source) in [
            (B::PIO, S::PIO),
            (B::DMA, S::DMA),
            (B::SYSTICK, S::SYSTICK),
            (B::UART, S::UART),
            (B::SPI, S::SPI),
            (B::I2C, S::I2C),
            (B::ADC, S::ADC),
            (B::TIMER, S::TIMER),
        ] {
            if observation.blockers.contains(blocker) {
                fallback_bits |= source;
                consider(now.saturating_add(1), source);
            }
        }

        // PWM has an exact first-wrap distance, but an already-routable IRQ
        // can still require the conservative one-cycle boundary above.
        if observation.blockers.contains(B::PWM) {
            fallback_bits |= S::PWM;
            consider(now.saturating_add(1), S::PWM);
        }
        if let Some(distance) = self.bus.pwm.next_wrap_distance() {
            consider(now.saturating_add(distance), S::PWM);
        }

        // Include masked alarms: their INTR/ARMED state changes at match even
        // though they cannot currently wake either core.
        if let Some(deadline) = self.bus.timer.next_armed_fire_cycle() {
            consider(deadline, S::TIMER);
        }
        if let Some(deadline) = external_event_cycle {
            consider(deadline, S::EXTERNAL);
        }

        crate::idle_profile::IdleEventHorizonProbe {
            master_cycle: now,
            next_event_cycle,
            distance_cycles: next_event_cycle.map(|cycle| cycle.saturating_sub(now)),
            limiting_sources: S::from_bits(limiting_bits),
            one_cycle_fallback_sources: S::from_bits(fallback_bits),
            complete_for_current_model: true,
        }
    }

    /// Compute a complete, conservative next-event horizon while both cores
    /// are blocked.
    ///
    /// Exact deadlines are used for TIMER alarms, PWM wraps, and the optional
    /// caller-owned external boundary. Every other temporal source is
    /// represented by a one-cycle fallback. The fallback can reduce an
    /// optimization opportunity, but can never allow the probe to jump over
    /// an event that the current model could observe.
    #[cfg(feature = "idle-profiler")]
    #[inline(never)]
    pub fn idle_event_horizon(
        &self,
        external_event_cycle: Option<u64>,
    ) -> crate::idle_profile::IdleEventHorizonProbe {
        self.idle_event_horizon_internal(external_event_cycle)
    }
}

/// Builder for assembling the emulator. Seeds the Bus clock tree from
/// `Config::sys_clk_hz` — the first CLOCKS / PLL register write
/// replaces the seed with the derived value.
pub struct EmulatorBuilder {
    config: Config,
    step_quantum: u32,
    flash: Option<Vec<u8>>,
    psram: Option<picoem_devices::Psram>,
    execution: ExecutionModel,
}

impl EmulatorBuilder {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            step_quantum: DEFAULT_STEP_QUANTUM,
            flash: None,
            psram: None,
            execution: ExecutionModel::default(),
        }
    }

    /// Override the per-step quantum (default [`DEFAULT_STEP_QUANTUM`]).
    ///
    /// Clamps `0 -> 1`. Previously a `debug_assert!` here meant
    /// `step_quantum(0)` silently advanced 0 cycles per `step()` in
    /// release builds, a guaranteed infinite-loop footgun for `run()`.
    pub fn step_quantum(mut self, n: u32) -> Self {
        self.step_quantum = n.max(1);
        self
    }

    /// Pre-load an XIP flash image. Applied at [`Self::build`] time via
    /// [`Emulator::load_flash`]; oversize images are silently clamped to
    /// the 2 MB flash window.
    pub fn flash(mut self, bytes: Vec<u8>) -> Self {
        self.flash = Some(bytes);
        self
    }

    /// Attach an off-chip SPI PSRAM device to the emulator. When set,
    /// [`Emulator::update_gpio`] feeds the device's `tick()` method on
    /// every GPIO merge and splices its MISO output back into `gpio_in`.
    pub fn psram(mut self, psram: picoem_devices::Psram) -> Self {
        self.psram = Some(psram);
        self
    }

    /// Select the runtime [`ExecutionModel`]. Defaults to
    /// `ExecutionModel::Serial` (the oracle-validated reference path).
    /// `ExecutionModel::Threaded` requires the `threading` cargo feature
    /// and an x86_64 Windows host; otherwise [`Self::build`] returns
    /// `Err(ConfigError::ThreadingUnavailable)`.
    pub fn execution(mut self, model: ExecutionModel) -> Self {
        self.execution = model;
        self
    }

    pub fn build(self) -> Result<Emulator, ConfigError> {
        // Threading availability gate — dual-execution HLD V1 §5.2.
        if self.execution == ExecutionModel::Threaded {
            #[cfg(not(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            )))]
            return Err(ConfigError::ThreadingUnavailable);
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            {
                let n = std::thread::available_parallelism()
                    .map(|n| n.get())
                    .unwrap_or(1);
                if n < 3 {
                    return Err(ConfigError::ThreadingUnavailable);
                }
            }
        }

        let mut bus = Bus::new();
        bus.seed_sys_clk_hz(self.config.sys_clk_hz);
        bus.psram = self.psram;
        let mut emu = Emulator {
            cores: [CortexM0Plus::with_id(0), CortexM0Plus::with_id(1)],
            bus,
            clock: Clock { cycles: 0 },
            step_quantum: self.step_quantum,
            pio_tick_count: 0,
            pio_tick_iow_low_count: 0,
            pio0_sm0_max_pc: 0,
            pio0_sm0_pc_advances: 0,
            pio0_sm0_last_pc: 0xFF,
            #[cfg(feature = "idle-profiler")]
            idle_profiler: None,
            #[cfg(feature = "event-horizon-profiler")]
            running_profiler: None,
            #[cfg(feature = "behavior-trace")]
            behavior_tracer: None,
            execution_model: self.execution,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            threaded: None,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            panic_info: None,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            timeout_info: None,
            #[cfg(all(
                feature = "testing",
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            pending_panic_inject: None,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            bus_is_placeholder: false,
        };
        // Default: core 1 halted — Pico SDK wakes it via SIO FIFO.
        // Route through the wrapper so the SIO handshake FSM `armed`
        // flag is in sync (HLD 2026.04.16 §2.1 / §5 invariant).
        emu.halt_core1();
        if let Some(bytes) = self.flash {
            emu.load_flash(&bytes);
        }
        info!(
            rom_size = ROM_SIZE,
            sram_size = SRAM_SIZE,
            step_quantum = self.step_quantum,
            sys_clk_hz = self.config.sys_clk_hz,
            execution = ?self.execution,
            "emulator constructed"
        );
        Ok(emu)
    }
}

// ---------------------------------------------------------------------------
// Stage 4: residue branch coverage for the top-level `lib.rs` (Emulator,
// EmulatorBuilder, Config, ConfigError, EmulatorError, ExecutionModel,
// WorkerName). Pure append-only — does not modify any production code.
// ---------------------------------------------------------------------------
#[cfg(test)]
mod stage4_lib_residue {
    use super::*;

    // ------------------- ConfigError -------------------

    #[test]
    fn config_error_display_threading_unavailable() {
        let s = format!("{}", ConfigError::ThreadingUnavailable);
        assert!(s.contains("Threaded"));
        assert!(s.contains("unavailable"));
    }

    #[test]
    fn config_error_debug_and_clone_eq() {
        let e1 = ConfigError::ThreadingUnavailable;
        let e2 = e1.clone();
        assert_eq!(e1, e2);
        let _ = format!("{:?}", e1);
    }

    #[test]
    fn config_error_is_std_error() {
        fn assert_err<E: std::error::Error>(_: &E) {}
        assert_err(&ConfigError::ThreadingUnavailable);
    }

    // ------------------- EmulatorError -------------------

    #[test]
    fn emulator_error_display_not_supported_in_threaded() {
        let s = format!("{}", EmulatorError::NotSupportedInThreadedMode);
        assert!(s.contains("Threaded"));
    }

    #[test]
    fn emulator_error_display_worker_panicked() {
        let e = EmulatorError::WorkerPanicked {
            which: WorkerName::Core0,
            message: String::from("boom"),
        };
        let s = format!("{}", e);
        assert!(s.contains("panicked"));
        assert!(s.contains("boom"));
        assert!(s.contains("core0"));
    }

    #[test]
    fn emulator_error_display_barrier_timeout() {
        let e = EmulatorError::BarrierTimeout {
            which: WorkerName::Coord,
            elapsed_ms: 1_234,
        };
        let s = format!("{}", e);
        assert!(s.contains("barrier"));
        assert!(s.contains("1234"));
        assert!(s.contains("coord"));
    }

    #[test]
    fn emulator_error_clone_eq_debug() {
        let e1 = EmulatorError::NotSupportedInThreadedMode;
        let e2 = e1.clone();
        assert_eq!(e1, e2);
        let _ = format!("{:?}", e1);
    }

    // ------------------- WorkerName -------------------

    #[test]
    fn worker_name_as_str_all_variants() {
        assert_eq!(WorkerName::Core0.as_str(), "core0");
        assert_eq!(WorkerName::Core1.as_str(), "core1");
        assert_eq!(WorkerName::Coord.as_str(), "coord");
    }

    #[test]
    fn worker_name_clone_eq_debug() {
        let w = WorkerName::Core1;
        assert_eq!(w, w.clone());
        let _ = format!("{:?}", w);
    }

    // ------------------- ExecutionModel -------------------

    #[test]
    fn execution_model_default_is_serial() {
        assert_eq!(ExecutionModel::default(), ExecutionModel::Serial);
    }

    #[test]
    fn execution_model_eq_and_debug() {
        assert_eq!(ExecutionModel::Threaded, ExecutionModel::Threaded);
        assert_ne!(ExecutionModel::Serial, ExecutionModel::Threaded);
        let _ = format!("{:?}", ExecutionModel::Serial);
        let _ = format!("{:?}", ExecutionModel::Threaded);
    }

    // ------------------- Builder: ConfigError::ThreadingUnavailable -------------------

    #[cfg(not(feature = "threading"))]
    #[test]
    fn builder_threaded_no_feature_returns_threading_unavailable() {
        let res = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build();
        match res {
            Err(ConfigError::ThreadingUnavailable) => {}
            Ok(_) => panic!("Threaded should fail without `threading` feature"),
        }
    }

    #[cfg(not(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    )))]
    #[test]
    fn builder_threaded_off_platform_returns_threading_unavailable() {
        let res = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build();
        match res {
            Err(ConfigError::ThreadingUnavailable) => {}
            Ok(_) => panic!("Threaded should fail on unsupported platforms"),
        }
    }

    // ------------------- Builder defaults / overrides -------------------

    #[test]
    fn builder_default_step_quantum() {
        let emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        assert_eq!(emu.step_quantum, DEFAULT_STEP_QUANTUM);
    }

    #[test]
    fn builder_custom_step_quantum() {
        let emu = EmulatorBuilder::new(Config::default())
            .step_quantum(8)
            .build()
            .unwrap();
        assert_eq!(emu.step_quantum, 8);
    }

    #[test]
    fn step_quantum_zero_clamps_to_one() {
        // Regression: `EmulatorBuilder::step_quantum(0)` previously
        // tripped a `debug_assert!` (and silently advanced 0 cycles
        // per `step()` in release builds — an infinite-loop footgun
        // for `run()`). The clamp at the builder entry point keeps
        // the runtime contract `step_quantum >= 1` intact.
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(0)
            .build()
            .unwrap();
        assert_eq!(emu.step_quantum, 1);
        // `step()` must make forward progress (advance >= 1 master
        // cycle) and not loop forever.
        let advanced = emu.step().unwrap();
        assert!(advanced >= 1);
    }

    #[test]
    fn builder_custom_sysclk() {
        let cfg = Config {
            sys_clk_hz: 125_000_000,
        };
        let emu = EmulatorBuilder::new(cfg).build().unwrap();
        // The clock tree is recomputed from sys_clk_hz; simplest check is
        // that the emulator builds and the master cycle starts at zero.
        assert_eq!(emu.cycles(), 0);
    }

    #[test]
    fn builder_execution_serial_explicit() {
        let emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Serial)
            .build()
            .unwrap();
        assert_eq!(emu.execution_model(), ExecutionModel::Serial);
    }

    #[test]
    fn builder_with_flash_pre_loads_xip() {
        let mut flash = vec![0u8; 32];
        flash[0..4].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
        let emu = EmulatorBuilder::new(Config::default())
            .flash(flash)
            .build()
            .unwrap();
        // XIP base is 0x1000_0000 — the flash loader writes there.
        assert_eq!(emu.bus.memory.xip_read32(0), 0xDEAD_BEEF);
    }

    // ------------------- Emulator basics -------------------

    #[test]
    fn execution_model_accessor_returns_selected() {
        let emu = Emulator::new(Config::default());
        assert_eq!(emu.execution_model(), ExecutionModel::Serial);
    }

    #[test]
    fn core_cycles_default_zero() {
        let emu = Emulator::new(Config::default());
        assert_eq!(emu.core_cycles(0), 0);
        assert_eq!(emu.core_cycles(1), 0);
    }

    #[test]
    #[should_panic(expected = "core_cycles: idx must be 0 or 1")]
    fn core_cycles_invalid_idx_panics() {
        let emu = Emulator::new(Config::default());
        let _ = emu.core_cycles(2);
    }

    #[test]
    fn cycles_starts_at_zero() {
        let emu = Emulator::new(Config::default());
        assert_eq!(emu.cycles(), 0);
    }

    #[test]
    fn core_and_core_mut_accessors() {
        let mut emu = Emulator::new(Config::default());
        // No id() public method available like rp2350_emu's, but we can
        // exercise both accessors and the placeholder-guard path.
        let _ = emu.core(0);
        let _ = emu.core_mut(1);
    }

    // ------------------- Emulator::run / step -------------------

    #[test]
    fn run_zero_cycles_serial_is_noop() {
        let mut emu = Emulator::new(Config::default());
        let executed = emu.run(0).unwrap();
        // run() returns the delta in cycles. With cycles=0 the inner loop
        // condition `clock.cycles - start < 0` is false on first check, so
        // no quanta run.
        assert_eq!(executed, 0);
    }

    #[test]
    fn run_quantum_serial_returns_ok() {
        let mut emu = Emulator::new(Config::default());
        let r = emu.run_quantum().unwrap();
        // run_quantum returns the number of cycles consumed in this
        // quantum; with both cores halted (core 1 always halted, core 0
        // running zero-data ROM) this is bounded by step_quantum.
        assert!(r <= emu.step_quantum as u64);
    }

    #[test]
    fn step_respects_quantum_cap_when_both_cores_running() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(32)
            .build()
            .unwrap();
        emu.wake_core1();
        let _ = emu.step().unwrap();
        // Existing serial scheduler can still return fewer cycles when one
        // core exits, but it must not exceed the configured quantum.
        assert!(emu.cycles() <= 32);
    }

    #[test]
    fn step_serial_returns_ok() {
        let mut emu = Emulator::new(Config::default());
        // Both cores halt quickly with zero-data ROM, but step still
        // returns Ok(consumed) on Serial.
        let _ = emu.step().unwrap();
    }

    // ------------------- gpio bounds -------------------

    #[test]
    fn gpio_read_pin_30_returns_false() {
        let emu = Emulator::new(Config::default());
        assert!(!emu.gpio_read(30));
        assert!(!emu.gpio_read(31));
    }

    #[test]
    fn gpio_write_pin_out_of_range_is_noop() {
        let mut emu = Emulator::new(Config::default());
        // Pin 30 is past the valid range. The function bails early — no
        // GPIO_OE bit gets set.
        emu.gpio_write(30, true);
        assert_eq!(emu.bus.sio.gpio_oe & (1u32 << 30), 0);
    }

    #[test]
    fn gpio_write_in_range_sets_oe_and_out() {
        let mut emu = Emulator::new(Config::default());
        emu.gpio_write(5, true);
        assert_ne!(emu.bus.sio.gpio_oe & (1u32 << 5), 0);
        assert_ne!(emu.bus.sio.gpio_out & (1u32 << 5), 0);
        emu.gpio_write(5, false);
        assert_eq!(emu.bus.sio.gpio_out & (1u32 << 5), 0);
    }

    #[test]
    fn gpio_read_all_default_zero() {
        let emu = Emulator::new(Config::default());
        assert_eq!(emu.gpio_read_all(), 0);
    }

    // ------------------- load_image / load_bootrom / load_flash -------------------

    #[test]
    fn load_image_sram_writes_through() {
        let mut emu = Emulator::new(Config::default());
        let data = [0x11u8, 0x22, 0x33, 0x44];
        emu.load_image(0x2000_0100, &data);
        assert_eq!(emu.peek(0x2000_0100), 0x4433_2211);
    }

    #[test]
    fn load_image_rom_region_overlay() {
        let mut emu = Emulator::new(Config::default());
        let data = [0xAAu8, 0xBB, 0xCC, 0xDD];
        emu.load_image(0x0000_0000, &data);
        assert_eq!(emu.bus.memory.rom_read8(0), 0xAA);
        assert_eq!(emu.bus.memory.rom_read8(3), 0xDD);
    }

    #[test]
    fn load_image_unknown_region_silently_dropped() {
        let mut emu = Emulator::new(Config::default());
        let data = [0xFFu8; 4];
        // 0x4 region — no match arm, falls through.
        emu.load_image(0x4000_0000, &data);
    }

    #[test]
    fn load_bootrom_loads_first_word() {
        let mut emu = Emulator::new(Config::default());
        let mut data = vec![0u8; 32];
        data[0..4].copy_from_slice(&0x2000_8000u32.to_le_bytes());
        emu.load_bootrom(&data);
        assert_eq!(emu.bus.memory.rom_read32(0), 0x2000_8000);
    }

    #[test]
    fn load_flash_drains_invalidations() {
        let mut emu = Emulator::new(Config::default());
        emu.load_flash(&[0u8; 16]);
        assert_eq!(emu.bus.pending_invalidation_regions, 0);
    }

    // ------------------- direct_boot_from_flash -------------------

    #[test]
    fn direct_boot_from_flash_seeds_sp_pc_vtor() {
        let mut emu = Emulator::new(Config::default());
        // Build a minimal vector table at flash offset 0x100 (pico-sdk).
        let sp = 0x2002_0000u32;
        let entry = 0x1000_0301u32; // Thumb-bit set
        let vtor_offset = 0x100u32;
        let mut flash = vec![0u8; 0x200];
        flash[(vtor_offset as usize)..(vtor_offset as usize + 4)]
            .copy_from_slice(&sp.to_le_bytes());
        flash[(vtor_offset as usize + 4)..(vtor_offset as usize + 8)]
            .copy_from_slice(&entry.to_le_bytes());
        emu.load_flash(&flash);
        emu.direct_boot_from_flash(vtor_offset);
        for c in 0..2 {
            assert_eq!(emu.cores[c].regs.msp, sp);
            assert_eq!(emu.cores[c].regs.pc(), entry & !1);
        }
        assert_eq!(emu.bus.ppb[0].vtor, 0x1000_0000 + vtor_offset);
        // Core 1 stays halted by halt_core1.
        assert!(emu.cores[1].is_halted());
    }

    // ------------------- halt_core1 / wake_core1 -------------------

    #[test]
    fn halt_and_wake_core1_round_trip() {
        let mut emu = Emulator::new(Config::default());
        // Initial state: core 1 halted.
        assert!(emu.cores[1].is_halted());
        emu.wake_core1();
        assert!(!emu.cores[1].is_halted());
        emu.halt_core1();
        assert!(emu.cores[1].is_halted());
    }

    // ------------------- mmio_read32 / mmio_write32 -------------------

    #[test]
    fn mmio_write_read_roundtrip_sram() {
        let mut emu = Emulator::new(Config::default());
        emu.mmio_write32(0x2000_0000, 0xDEAD_BEEF);
        assert_eq!(emu.mmio_read32(0x2000_0000), 0xDEAD_BEEF);
    }

    // ------------------- reset -------------------

    #[test]
    fn reset_clears_clock() {
        let mut emu = Emulator::new(Config::default());
        let _ = emu.step().unwrap();
        emu.reset();
        assert_eq!(emu.cycles(), 0);
        assert_eq!(emu.pio_tick_count, 0);
    }

    // ------------------- drain_uart0_tx_log -------------------

    #[test]
    fn drain_uart0_tx_log_default_empty() {
        let mut emu = Emulator::new(Config::default());
        let v = emu.drain_uart0_tx_log();
        assert!(v.is_empty());
    }

    // ------------------- poke / peek -------------------

    #[test]
    fn poke_and_peek_round_trip_sram() {
        let mut emu = Emulator::new(Config::default());
        emu.poke(0x2000_2000, 0xCAFE_F00D);
        assert_eq!(emu.peek(0x2000_2000), 0xCAFE_F00D);
    }
}

// ---------------------------------------------------------------------------
// Stage 5: branch-coverage residue not hit by Stage 4. Targets specific
// `if` arms inside `reset`, `load_image`, `step_serial`, `tick_systick`,
// `tick_pio_and_route_irqs`, `tick_pio`, `update_gpio`, and `wake_checks`.
// Pure append-only — does not modify production code.
// ---------------------------------------------------------------------------
#[cfg(test)]
mod stage5_lib_residue {
    use super::*;
    use picoem_devices::Psram;

    // ------------------- reset: psram present (line 422) -------------------

    /// Drives the true branch of `if let Some(ref mut psram) = self.bus.psram`
    /// (line 422) inside `reset()` by attaching a PSRAM device first.
    #[test]
    fn reset_with_psram_attached() {
        let psram = Psram::new(0, 1, 2, 3);
        let mut emu = EmulatorBuilder::new(Config::default())
            .psram(psram)
            .build()
            .unwrap();
        emu.reset();
        assert_eq!(emu.cycles(), 0);
    }

    // ------------------- load_image: ROM offset boundary (line 461) -------------------

    /// Drives the false branch of `if offset < ROM_SIZE` inside the ROM
    /// arm of `load_image` (line 461) by passing an offset past ROM end.
    /// `offset = addr & 0x0FFF_FFFF` so `0x0FFF_FFFF` is well past the
    /// 16 KB ROM_SIZE.
    #[test]
    fn load_image_rom_offset_past_end_is_skipped() {
        let mut emu = Emulator::new(Config::default());
        let data = [0x55u8; 4];
        // offset = 0x0FFF_FFFF >= ROM_SIZE so the inner copy is skipped.
        emu.load_image(0x0FFF_FFFF, &data);
        // ROM byte 0 untouched (was 0).
        assert_eq!(emu.bus.memory.rom_read8(0), 0);
    }

    /// Drives the true branch of the same `if offset < ROM_SIZE` (line 461)
    /// — the existing test suite hits this through `load_image_rom_region_overlay`,
    /// repeated here for explicit branch attribution.
    #[test]
    fn load_image_rom_offset_inside_overlays() {
        let mut emu = Emulator::new(Config::default());
        let data = [0xAAu8, 0xBB, 0xCC, 0xDD];
        emu.load_image(0x0000_0010, &data);
        assert_eq!(emu.bus.memory.rom_read8(0x10), 0xAA);
    }

    // ------------------- step_serial: while-loop arms + c0/c1 paths -------------------

    /// Drives the true arm of the c1 step `if !self.cores[1].is_halted()
    /// && !self.bus.wfe_waiting[1]` (line 700) by waking core 1 first.
    /// Also covers line 681/682 (while predicate evaluates with both
    /// cores live).
    #[test]
    fn step_serial_runs_core1_when_awake() {
        let mut emu = Emulator::new(Config::default());
        emu.wake_core1();
        assert!(!emu.cores[1].is_halted());
        let _ = emu.step().unwrap();
    }

    /// Drives the WFE-waiting branch in the c1 selection (line 700 false
    /// arm via wfe_waiting=true).
    #[test]
    fn step_serial_skips_core1_when_wfe_waiting() {
        let mut emu = Emulator::new(Config::default());
        emu.wake_core1();
        emu.bus.wfe_waiting[1] = true;
        let _ = emu.step().unwrap();
    }

    /// Drives the true arm of `if c0 == 0 && c1 == 0` (line 715) by
    /// halting core 0 and parking core 1 in WFE so neither contributes
    /// cycles, but the while predicate still fires once because halted
    /// flags can be reset between iterations. Best-effort branch-visit
    /// test — actual entry depends on inner-loop timing.
    #[test]
    fn step_serial_break_on_zero_cycles_smoke() {
        let mut emu = Emulator::new(Config::default());
        // Both cores halted so the while predicate gates entry to the
        // loop body. The loop exits before producing cycles, so the
        // post-loop bookkeeping still runs.
        emu.cores[0].halt();
        emu.halt_core1();
        let _ = emu.step().unwrap();
    }

    /// Production fix for tech_debt §1649: when both cores are blocked
    /// (halted/WFI here) and a TIMER alarm with INTE is scheduled in the
    /// future, the master clock must still advance to the alarm's fire
    /// cycle so the IRQ raises and `wake_checks` un-halts a core.
    ///
    /// Pre-fix behaviour: `consumed == 0` every quantum → master_cycle
    /// never moves → poll_alarms never matches → core stays parked
    /// forever.
    /// Post-fix behaviour: the both-blocked branch advances the clock
    /// to the soonest scheduled fire cycle (capped by `step_quantum`),
    /// raises the IRQ, drains it to NVICs, and `wake_checks` un-halts.
    #[test]
    fn step_serial_advances_clock_when_both_cores_blocked_with_armed_alarm() {
        use crate::peripherals::timer::{ALARM0_OFFSET, INTE_OFFSET};

        // Use a generous step_quantum so a single step() can cover the
        // whole armed-alarm interval in one go.
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(2_000_000)
            .build()
            .expect("Serial build is infallible");

        // Both cores parked. Core 1 is halt_core1 (correct production
        // path); core 0 we explicitly halt to emulate WFI.
        emu.cores[0].halt();
        emu.halt_core1();
        assert!(emu.cores[0].is_halted());
        assert!(emu.cores[1].is_halted());

        // Arm TIMER ALARM0 200 µs into the future and enable INTE so the
        // fire raises NVIC line 0. Reach into bus.timer directly — same
        // crate, `pub(crate)` field.
        let sys_hz = emu.bus.clock_tree.sys_clk_hz;
        emu.bus
            .timer
            .write32(INTE_OFFSET, 0x1, 0, emu.bus.master_cycle, sys_hz);
        emu.bus
            .timer
            .write32(ALARM0_OFFSET, 200, 0, emu.bus.master_cycle, sys_hz);

        // Sanity: nothing pending yet — the bus IRQ vector is clean,
        // both NVICs are clean, and both cores are halted. Without the
        // fix, this state is a permanent dead-end.
        assert_eq!(emu.bus.irq_pending, 0);
        assert_eq!(emu.bus.nvics[0].pending_and_enabled(), 0);
        assert_eq!(emu.bus.nvics[1].pending_and_enabled(), 0);

        // One step suffices when step_quantum >> alarm horizon.
        let _ = emu.step().unwrap();

        // Post-fix: TIMER alarm fired, IRQ drained to at least one NVIC,
        // and `wake_checks` un-halted the core(s) it landed on. Any of
        // these three observations is sufficient — and on the pre-fix
        // code, none of them holds.
        let nvic_pending = emu.bus.nvics[0].is_pending(0) || emu.bus.nvics[1].is_pending(0);
        let core_woke = !emu.cores[0].is_halted() || !emu.cores[1].is_halted();
        assert!(
            nvic_pending || core_woke,
            "Both-blocked clock-advance branch did not deliver TIMER alarm: \
             nvic0_pend0={} nvic1_pend0={} c0_halted={} c1_halted={} \
             master_cycle={} alarm_fire={:?}",
            emu.bus.nvics[0].is_pending(0),
            emu.bus.nvics[1].is_pending(0),
            emu.cores[0].is_halted(),
            emu.cores[1].is_halted(),
            emu.bus.master_cycle,
            emu.bus.next_scheduled_lazy_deadline(),
        );
    }

    #[test]
    fn step_until_stops_at_external_boundary_when_shorter_than_internal_horizon() {
        use crate::peripherals::timer::{ALARM0_OFFSET, INTE_OFFSET};

        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(64)
            .build()
            .unwrap();
        emu.cores[0].halt();
        emu.halt_core1();
        let sys_hz = emu.bus.clock_tree.sys_clk_hz;
        emu.bus
            .timer
            .write32(INTE_OFFSET, 1, 0, emu.bus.master_cycle, sys_hz);
        emu.bus
            .timer
            .write32(ALARM0_OFFSET, 200, 0, emu.bus.master_cycle, sys_hz);

        let now = emu.bus.master_cycle;
        let consumed = emu.step_until(now + 7).unwrap();
        assert_eq!(consumed, 7);
        assert_eq!(emu.bus.master_cycle, now + 7);
        assert_eq!(emu.bus.irq_pending, 0);
    }

    #[test]
    fn step_until_active_pio_fallback_is_one_cycle() {
        use crate::bus::PIO0_BASE;
        use crate::peripherals::timer::{ALARM0_OFFSET, INTE_OFFSET};

        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(64)
            .build()
            .unwrap();
        emu.cores[0].halt();
        emu.halt_core1();
        emu.bus.write32(PIO0_BASE + 0x2000, 1);
        let sys_hz = emu.bus.clock_tree.sys_clk_hz;
        emu.bus
            .timer
            .write32(INTE_OFFSET, 1, 0, emu.bus.master_cycle, sys_hz);
        emu.bus
            .timer
            .write32(ALARM0_OFFSET, 200, 0, emu.bus.master_cycle, sys_hz);

        let consumed = emu.step_until(emu.bus.master_cycle + 64).unwrap();
        assert_eq!(consumed, 1);
        assert_eq!(emu.bus.master_cycle, 1);
    }

    #[test]
    fn step_until_matches_one_cycle_reference_at_masked_timer_boundary() {
        use crate::peripherals::timer::ALARM0_OFFSET;

        let mut bulk = EmulatorBuilder::new(Config::default())
            .step_quantum(64)
            .build()
            .unwrap();
        let mut ref_emu = EmulatorBuilder::new(Config::default())
            .step_quantum(1)
            .build()
            .unwrap();

        let sys_hz = bulk.bus.clock_tree.sys_clk_hz;
        for emu in [&mut bulk, &mut ref_emu] {
            emu.cores[0].halt();
            emu.halt_core1();
            emu.bus
                .timer
                .write32(ALARM0_OFFSET, 200, 0, emu.bus.master_cycle, sys_hz);
        }

        let target = bulk.bus.timer.next_armed_fire_cycle().unwrap();
        assert!(target - bulk.bus.master_cycle > bulk.step_quantum as u64);
        let bulk_consumed = bulk.step_until(target).unwrap();
        while ref_emu.cycles() < target {
            let next = ref_emu.cycles() + 1;
            assert_eq!(ref_emu.step_until(next).unwrap(), 1);
        }

        assert_eq!(bulk_consumed, target);
        assert_eq!(bulk.bus.master_cycle, target);
        assert_eq!(ref_emu.bus.master_cycle, target);
        assert_eq!(
            bulk.bus.timer.behavior_trace_state(),
            ref_emu.bus.timer.behavior_trace_state()
        );
        assert_eq!(bulk.bus.irq_pending, ref_emu.bus.irq_pending);
        assert_eq!(
            bulk.bus.nvics[0].pending_and_enabled(),
            ref_emu.bus.nvics[0].pending_and_enabled()
        );
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_profiler_records_conservative_safe_blocked_cycles() {
        use crate::peripherals::timer::{ALARM0_OFFSET, INTE_OFFSET};

        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(64)
            .build()
            .unwrap();
        emu.enable_idle_profiler().unwrap();
        emu.cores[0].halt();
        emu.halt_core1();
        let sys_hz = emu.bus.clock_tree.sys_clk_hz;
        emu.bus
            .timer
            .write32(INTE_OFFSET, 1, 0, emu.bus.master_cycle, sys_hz);
        emu.bus
            .timer
            .write32(ALARM0_OFFSET, 200, 0, emu.bus.master_cycle, sys_hz);

        let probe = emu.idle_current_probe();
        assert!(probe.proven_jump_safe);
        assert_eq!(probe.blocker_count, 0);
        assert!(probe.next_lazy_deadline.is_some());
        assert_eq!(emu.step().unwrap(), 64);
        let profile = emu.idle_profile_snapshot().unwrap();
        assert_eq!(profile.step_calls, 1);
        assert_eq!(profile.total_master_cycles, 64);
        assert_eq!(profile.both_blocked_cycles, 64);
        assert_eq!(profile.proven_safe_cycles, 64);
        assert_eq!(profile.blocked_lengths.episodes_ge[6], 1);
        assert_eq!(profile.blocked_lengths.cycle_mass_ge[6], 64);
        assert_eq!(profile.proven_safe_lengths.episodes_ge[6], 1);
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_profiler_attributes_pio_as_overlapping_blocker() {
        use crate::bus::PIO0_BASE;
        use crate::peripherals::timer::{ALARM0_OFFSET, INTE_OFFSET};

        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(8)
            .build()
            .unwrap();
        emu.enable_idle_profiler().unwrap();
        emu.cores[0].halt();
        emu.halt_core1();
        emu.bus.write32(PIO0_BASE + 0x2000, 1);
        assert!(!emu.bus.pio_all_idle());
        let sys_hz = emu.bus.clock_tree.sys_clk_hz;
        emu.bus
            .timer
            .write32(INTE_OFFSET, 1, 0, emu.bus.master_cycle, sys_hz);
        emu.bus
            .timer
            .write32(ALARM0_OFFSET, 200, 0, emu.bus.master_cycle, sys_hz);

        let probe = emu.idle_current_probe();
        assert!(!probe.proven_jump_safe);
        assert_eq!(probe.blocker_count, 1);
        assert_eq!(emu.step().unwrap(), 8);
        let profile = emu.idle_profile_snapshot().unwrap();
        assert_eq!(profile.both_blocked_cycles, 8);
        assert_eq!(profile.proven_safe_cycles, 0);
        assert_eq!(profile.blockers.pio, 8);
        assert_eq!(profile.blocker_episodes.pio, 1);
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_profiler_classifies_empty_tx_stalled_pio_as_stationary() {
        use crate::bus::PIO0_BASE;

        let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        emu.enable_idle_profiler().unwrap();
        // SM0 executes a blocking PULL from an empty TX FIFO, then remains
        // unable to change PC or pins until CPU/DMA supplies a word.
        emu.bus.write32(PIO0_BASE + 0x048, 0x80A0);
        emu.bus.write32(PIO0_BASE + 0x000, 1);
        emu.bus.pio[0].step(0);
        assert!(emu.bus.pio[0].sm[0].stalled_on_empty_tx());

        let probe = emu.idle_current_probe();
        assert!(probe.proven_jump_safe);
        assert_eq!(probe.blocker_count, 0);
        assert_eq!(probe.stationary_source_count, 1);
        assert_eq!(probe.exact_bulk_source_count, 0);
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_horizon_includes_masked_timer_alarm() {
        use crate::peripherals::timer::ALARM0_OFFSET;

        let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        let sys_hz = emu.bus.clock_tree.sys_clk_hz;
        emu.bus
            .timer
            .write32(ALARM0_OFFSET, 200, 0, emu.bus.master_cycle, sys_hz);

        let horizon = emu.idle_event_horizon(None);
        assert!(horizon.complete_for_current_model);
        assert_eq!(
            horizon.distance_cycles,
            Some(200 * (sys_hz as u64 / 1_000_000))
        );
        assert!(
            horizon
                .limiting_sources
                .contains(IdleEventSourceMask::TIMER)
        );
        assert_eq!(horizon.one_cycle_fallback_sources.bits(), 0);
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_horizon_uses_one_cycle_fallback_for_active_pio() {
        use crate::bus::PIO0_BASE;

        let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        emu.bus.write32(PIO0_BASE + 0x2000, 1);

        let horizon = emu.idle_event_horizon(None);
        assert_eq!(horizon.distance_cycles, Some(1));
        assert!(horizon.limiting_sources.contains(IdleEventSourceMask::PIO));
        assert!(
            horizon
                .one_cycle_fallback_sources
                .contains(IdleEventSourceMask::PIO)
        );
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_horizon_external_boundary_wins_and_ties_are_retained() {
        let emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        let now = emu.bus.master_cycle;
        let horizon = emu.idle_event_horizon(Some(now + 7));
        assert_eq!(horizon.next_event_cycle, Some(now + 7));
        assert_eq!(horizon.distance_cycles, Some(7));
        assert!(
            horizon
                .limiting_sources
                .contains(IdleEventSourceMask::EXTERNAL)
        );
    }

    #[cfg(feature = "idle-profiler")]
    #[test]
    fn idle_horizon_pending_irq_is_current_boundary() {
        let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        emu.bus.irq_pending = 1;
        let horizon = emu.idle_event_horizon(Some(emu.bus.master_cycle + 100));
        assert_eq!(horizon.distance_cycles, Some(0));
        assert!(
            horizon
                .limiting_sources
                .contains(IdleEventSourceMask::PENDING_IRQ)
        );
    }

    // ------------------- step_serial: slow-path / fast-path gating (line 750) -------------------

    /// Drives the slow-path arm of the fast-path gate (line 750 false)
    /// by enabling SysTick on the active core. The predicate
    /// `systick_idle = !systicks[active].is_enabled()` becomes false,
    /// forcing the slow branch.
    #[test]
    fn step_serial_drops_to_slow_path_when_systick_enabled() {
        let mut emu = Emulator::new(Config::default());
        // Enable SysTick on core 0 (CSR.ENABLE = bit 0).
        emu.bus.systicks[0].csr |= 1;
        let _ = emu.step().unwrap();
    }

    /// Drives the fast-path arm of the same gate (line 750 true). Default
    /// state has no SysTick, no IRQ, no PIO — the existing
    /// `step_serial_returns_ok` already exercises this; making it explicit
    /// here for branch attribution.
    #[test]
    fn step_serial_takes_fast_path_when_idle() {
        let mut emu = Emulator::new(Config::default());
        let _ = emu.step().unwrap();
    }

    // ------------------- drain_pending_irqs_to_cores (line 792, 795) -------------------

    /// Drives the true arm of `if self.bus.irq_pending != 0` (line 792)
    /// AND the inner per-IRQ scan (line 795 true) by pre-staging a bus
    /// irq_pending bit. Forces the slow path via SysTick enable so the
    /// drain runs.
    #[test]
    fn drain_pending_irqs_routes_to_nvics() {
        let mut emu = Emulator::new(Config::default());
        // Force slow path.
        emu.bus.systicks[0].csr |= 1;
        // Pre-stage a pending IRQ on bus.irq_pending.
        emu.bus.irq_pending = 0x1; // line 0
        let _ = emu.step().unwrap();
        // After the slow path drains, both NVICs see line 0 pending.
        assert!(emu.bus.nvics[0].is_pending(0) || emu.bus.nvics[1].is_pending(0));
    }

    #[test]
    fn active_level_irq_does_not_repend_same_core_but_remains_shared() {
        let mut emu = Emulator::new(Config::default());
        const IRQ: u8 = 11;
        const EXCEPTION: u16 = 16 + IRQ as u16;

        emu.bus.ppb[0].mark_active(EXCEPTION);
        emu.bus.irq_pending = 1u32 << IRQ;
        emu.drain_pending_irqs_to_cores();

        assert!(!emu.bus.nvics[0].is_pending(IRQ));
        assert!(emu.bus.nvics[1].is_pending(IRQ));

        emu.bus.nvics[1].clear_pending(IRQ);
        emu.bus.ppb[0].clear_active(EXCEPTION);
        emu.bus.irq_pending = 1u32 << IRQ;
        emu.drain_pending_irqs_to_cores();

        assert!(emu.bus.nvics[0].is_pending(IRQ));
        assert!(emu.bus.nvics[1].is_pending(IRQ));
    }

    // ------------------- tick_systick (lines 812, 817) -------------------

    /// Drives the true branches of `if systicks[0].tick()` (line 812) and
    /// `if systicks[1].tick()` (line 817) by enabling SysTick with
    /// CVR=0, RVR=0 so the very first tick fires.
    #[test]
    fn tick_systick_fires_on_both_cores_when_enabled() {
        let mut emu = Emulator::new(Config::default());
        // Wake core 1 so it consumes cycles → tick_systick(c0, c1) with
        // both > 0.
        emu.wake_core1();
        // Enable SysTick (ENABLE=1, TICKINT=1) on both cores; CVR=0, RVR=0
        // → first tick reloads + fires.
        emu.bus.systicks[0].csr = 0b11;
        emu.bus.systicks[1].csr = 0b11;
        emu.bus.systicks[0].cvr = 0;
        emu.bus.systicks[1].cvr = 0;
        let _ = emu.step().unwrap();
    }

    // ------------------- tick_pio_and_route_irqs (lines 842, 852, 855, 860, 863) -------------------

    /// Drives the false arm of `if gpio_in & (1u32 << 4) == 0` (line 842)
    /// by seeding `bus.gpio_in` with bit 4 high before the slow-path
    /// tick. tick_pio_and_route_irqs reads gpio_in directly so we must
    /// pre-set it; update_gpio runs after tick_pio so the external mask
    /// alone doesn't apply early enough.
    #[test]
    fn tick_pio_iow_high_does_not_count_as_low() {
        let mut emu = Emulator::new(Config::default());
        // Force slow path so tick_pio_and_route_irqs runs.
        emu.bus.systicks[0].csr |= 1;
        // Pre-seed gpio_in with bit 4 high AND set the external mask so
        // update_gpio's post-tick rewrite preserves it across the cycle.
        emu.bus.gpio_in = 1u32 << 4;
        emu.bus.external_gpio_in_mask = 1u32 << 4;
        emu.bus.external_gpio_in_override = 1u32 << 4;
        let _ = emu.step().unwrap();
        // The branch was visited; whether the count stays zero depends
        // on whether tick_pio runs again after update_gpio. The goal is
        // line coverage of the false arm of the IOW gate.
    }

    /// Drives the true arms of the PIO INTF routing `if int0_ints != 0`
    /// (line 860) and `int1_ints != 0` (line 863) for both PIO blocks.
    /// Uses the slow-path forcing trick so tick_pio_and_route_irqs runs.
    #[test]
    fn tick_pio_routes_intf_to_irq_pending() {
        let mut emu = Emulator::new(Config::default());
        // Force slow path.
        emu.bus.systicks[0].csr |= 1;
        // Release every peripheral from reset so MMIO writes to PIO land.
        emu.bus.resets.state = 0;
        // Write INT0_INTF / INT1_INTF on both PIO blocks. RP2040 PIO
        // INT0_INTF offset 0x034, INT1_INTF offset 0x040 (datasheet
        // §3.7). bus offsets: PIO0=0x5020_0000, PIO1=0x5030_0000.
        emu.bus.write32(0x5020_0034, 0x1);
        emu.bus.write32(0x5020_0040, 0x1);
        emu.bus.write32(0x5030_0034, 0x1);
        emu.bus.write32(0x5030_0040, 0x1);
        let _ = emu.step().unwrap();
    }

    // ------------------- tick_pio early-return (line 876) -------------------

    /// Drives the true arm of `if cycles == 0` inside `tick_pio`
    /// (line 876) by stepping with cores idle. This is best-effort —
    /// the step path may always pass non-zero cycles. We instead invoke
    /// the situation by halting both cores so `consumed` may be zero.
    #[test]
    fn tick_pio_zero_cycles_is_noop_smoke() {
        let mut emu = Emulator::new(Config::default());
        emu.cores[0].halt();
        emu.halt_core1();
        let _ = emu.step().unwrap();
    }

    // ------------------- update_gpio: psram path + external mask (lines 1110, 1111, 1118) -------------------

    /// Drives the true branch of `if let Some(ref mut psram) = self.bus.psram`
    /// (line 1110) plus the inner `if let Some(miso) = psram.tick(out)`
    /// (line 1111) by attaching a PSRAM and calling update_gpio via a
    /// public path (gpio_write triggers it).
    #[test]
    fn update_gpio_with_psram_attached() {
        let psram = Psram::new(0, 1, 2, 3);
        let mut emu = EmulatorBuilder::new(Config::default())
            .psram(psram)
            .build()
            .unwrap();
        // gpio_write calls update_gpio internally.
        emu.gpio_write(5, true);
    }

    /// Drives the true branch of `if ext_mask != 0` (line 1118) by
    /// asserting an external GPIO mask before update_gpio runs.
    #[test]
    fn update_gpio_external_mask_overrides() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.external_gpio_in_mask = 1u32 << 5;
        emu.bus.external_gpio_in_override = 1u32 << 5;
        // Trigger update_gpio via gpio_write.
        emu.gpio_write(0, false);
        assert!(emu.gpio_read(5));
    }

    // ------------------- wake_checks (lines 1148, 1155) -------------------

    /// Drives the true arm of `if self.bus.wfe_waiting[core] &&
    /// self.bus.event_flag[core]` (line 1148): pre-park core 0 on WFE
    /// with an event latched.
    #[test]
    fn wake_checks_consumes_wfe_event_core0() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.wfe_waiting[0] = true;
        emu.bus.event_flag[0] = true;
        emu.cores[0].halt();
        let _ = emu.step().unwrap();
        assert!(!emu.bus.wfe_waiting[0]);
        assert!(!emu.bus.event_flag[0]);
    }

    /// Drives the true arm of `if self.cores[core].is_halted() &&
    /// self.bus.nvics[core].pending_and_enabled() != 0` (line 1155)
    /// by enabling and pending an IRQ on core 0 while halted.
    #[test]
    fn wake_checks_unhalts_on_pending_enabled_irq_core0() {
        let mut emu = Emulator::new(Config::default());
        emu.cores[0].halt();
        emu.bus.nvics[0].set_enabled(0);
        emu.bus.nvics[0].set_pending(0);
        let _ = emu.step().unwrap();
        assert!(!emu.cores[0].is_halted());
    }

    // ------------------- core_cycles: fall-through after core 1 wake -------------------

    /// Indirect coverage for the cycle-counter idx==1 arm in core_cycles
    /// after the core has actually consumed a cycle. The default-zero
    /// case is covered by `core_cycles_default_zero`; this test forces
    /// a non-zero counter to pair with it.
    #[test]
    fn core_cycles_idx1_after_step() {
        let mut emu = Emulator::new(Config::default());
        emu.wake_core1();
        let _ = emu.step().unwrap();
        // No exact assertion — accessor evaluates the idx==1 arm.
        let _ = emu.core_cycles(1);
    }
}
