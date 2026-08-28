//! RP2040 AHB-Lite bus fabric.
//!
//! Phase 5.A: full address decode + peripheral routing for the registers
//! firmware actually touches (CLOCKS, RESETS, PLL_SYS, PLL_USB, XOSC, ROSC,
//! SIO, IO_BANK0, PADS_BANK0, XIP_CTRL stub, SSI stub). SRAM bank routing
//! uses the RP2040 4+2 layout from [`crate::memory::bank_for_address`];
//! bank contention is modelled simply (+1 cycle on SRAM access when
//! the companion core has already touched SRAM this quantum).
//!
//! Phase 5.B: PIO0 / PIO1 wired into the AHB decode at `0x5020_0000` and
//! `0x5030_0000`. Register access goes through `PioBlock::read32` /
//! `write32` (mirrors `rp2350_emu::Bus`). Sub-word writes to PIO ranges are
//! ignored — several PIO registers have side-effects on read (RXF pop) or
//! write (TXF push, CTRL bit flags) that would behave incorrectly under a
//! synthetic read-modify-write. Sub-word reads still go through `read32`
//! (matching rp2350_emu) and so observe those same side-effects on the
//! enclosing word — firmware that only touches PIO with word-sized
//! accesses (the supported path in the datasheet) is unaffected.

pub mod clocks;
pub mod io_bank0;
pub mod pads_bank0;
pub mod peripheral_dispatch;
pub mod ppb;
pub mod resets;
pub mod sio;
pub mod ssi_flash;
pub mod systick;

use std::collections::HashMap;
use std::io::Write;

use tracing::debug;

use picoem_common::PioBlock;

#[cfg(all(
    feature = "compact-dispatch-key-prototype",
    feature = "decoded-op-8byte-prototype"
))]
compile_error!(
    "compact-dispatch-key-prototype is incompatible with decoded-op-8byte-prototype; enable one cache representation at a time"
);

use picoem_common::clocks::{pll_cs_read_with_lock, pll_should_arm_lock};

use crate::core::Nvic;
use crate::dma::Dma;
use crate::irq::{
    IRQ_ADC_IRQ_FIFO, IRQ_I2C0_IRQ, IRQ_I2C1_IRQ, IRQ_PWM_IRQ_WRAP, IRQ_SIO_IRQ_PROC0,
    IRQ_SIO_IRQ_PROC1, IRQ_SPI0_IRQ, IRQ_SPI1_IRQ, IRQ_UART0_IRQ, IRQ_UART1_IRQ,
};
use crate::memory::{FLASH_SIZE, Memory, ROM_SIZE, SRAM_SIZE, bank_for_address};
use crate::peripherals::adc::AdcRegs;
use crate::peripherals::i2c::I2cRegs;
use crate::peripherals::pwm::PwmRegs;
use crate::peripherals::spi::SpiRegs;
use crate::peripherals::timer::TimerRegs;
use crate::peripherals::uart::{UartRegs, UartRxResult};
use crate::peripherals::watchdog_tick::WatchdogTickRegs;
use crate::virtual_time::VirtualClock;
use clocks::{ClockTree, ClocksRegs, PLL_RESET, PllRegs, ROSC_FREQ_HZ, RoscRegs, XoscRegs};
use io_bank0::IoBank0;
use pads_bank0::PadsBank0;
use picoem_devices::Psram;
use ppb::Ppb;
use resets::Resets;
use sio::Sio;
use systick::SysTick;

/// Peripheral base addresses (see RP2040 datasheet §2.2).
pub const APB_BASE: u32 = 0x4000_0000;
pub const SIO_BASE: u32 = 0xD000_0000;

// APB peripheral base addresses (RP2040 datasheet §2.2).
pub const SYSINFO_BASE: u32 = 0x4000_0000;
pub const SYSCFG_BASE: u32 = 0x4000_4000;
pub const CLOCKS_BASE: u32 = 0x4000_8000;
pub const RESETS_BASE: u32 = 0x4000_C000;
pub const PSM_BASE: u32 = 0x4001_0000;
pub const IO_BANK0_BASE: u32 = 0x4001_4000;
pub const IO_QSPI_BASE: u32 = 0x4001_8000;
pub const PADS_BANK0_BASE: u32 = 0x4001_C000;
pub const PADS_QSPI_BASE: u32 = 0x4002_0000;
pub const XOSC_BASE: u32 = 0x4002_4000;
pub const PLL_SYS_BASE: u32 = 0x4002_8000;
pub const PLL_USB_BASE: u32 = 0x4002_C000;
pub const BUSCTRL_BASE: u32 = 0x4003_0000;
pub const ROSC_BASE: u32 = 0x4006_0000;
/// UART0 block base (RP2040 datasheet §4.2). Reset-gated on bit 22.
pub const UART0_BASE: u32 = 0x4003_4000;
/// UART1 block base (RP2040 datasheet §4.2). Reset-gated on bit 23.
pub const UART1_BASE: u32 = 0x4003_8000;
/// SPI0 block base (RP2040 datasheet §4.4). Reset-gated on bit 16.
pub const SPI0_BASE: u32 = 0x4003_C000;
/// SPI1 block base (RP2040 datasheet §4.4). Reset-gated on bit 17.
pub const SPI1_BASE: u32 = 0x4004_0000;
/// I2C0 block base (RP2040 datasheet §4.3). Reset-gated on bit 3.
pub const I2C0_BASE: u32 = 0x4004_4000;
/// I2C1 block base (RP2040 datasheet §4.3). Reset-gated on bit 4.
pub const I2C1_BASE: u32 = 0x4004_8000;
/// ADC block base (RP2040 datasheet §4.9). Reset-gated on bit 0.
pub const ADC_BASE: u32 = 0x4004_C000;
/// PWM block base (RP2040 datasheet §4.5). Reset-gated on bit 14.
pub const PWM_BASE: u32 = 0x4005_0000;
/// TIMER block base (RP2040 datasheet §4.6). Reset-gated on
/// [`peripheral_dispatch::RESET_TIMER`] (bit 21) — TIMER and WATCHDOG
/// have independent RESETS bits per datasheet §2.14 Table 26. The 1 µs
/// cadence coupling to WATCHDOG_TICK is runtime signalling, not a
/// reset-bit relationship.
pub const TIMER_BASE: u32 = 0x4005_4000;
/// WATCHDOG block base (RP2040 datasheet §4.7). Phase 1 models only
/// `WATCHDOG_TICK` (offset `0x2C`); remaining registers read as 0.
pub const WATCHDOG_BASE: u32 = 0x4005_8000;
pub const XIP_CTRL_BASE: u32 = 0x1400_0000;
pub const SSI_BASE: u32 = 0x1800_0000;
/// IO_QSPI GPIO1 (QSPI_SS_N) control register. The RP2040 ROM flash
/// helpers force this output low/high around command transfers instead of
/// toggling SSIENR, so it is also the flash transaction boundary.
const IO_QSPI_SS_CTRL: u32 = 0x0C;
const IO_QSPI_OUTOVER_SHIFT: u32 = 8;
const IO_QSPI_OUTOVER_MASK: u32 = 0x3 << IO_QSPI_OUTOVER_SHIFT;
const IO_QSPI_OUTOVER_LOW: u32 = 0x2 << IO_QSPI_OUTOVER_SHIFT;
const IO_QSPI_OUTOVER_HIGH: u32 = 0x3 << IO_QSPI_OUTOVER_SHIFT;
/// XIP_SSI register offsets touched by the boot-time flash helpers.
const SSI_SSIENR: u32 = 0x08;
const SSI_TXFLR: u32 = 0x20;
const SSI_RXFLR: u32 = 0x24;
const SSI_SR: u32 = 0x28;
const SSI_DR0: u32 = 0x60;
/// SSI_SR bits: transmit FIFO not full, transmit FIFO empty, receive
/// FIFO not empty.
const SSI_SR_TFNF: u32 = 1 << 1;
const SSI_SR_TFE: u32 = 1 << 2;
const SSI_SR_RFNE: u32 = 1 << 3;
pub const XIP_SRAM_BASE: u32 = 0x1500_0000;
pub const XIP_SRAM_END: u32 = 0x1500_4000; // 16 KB
/// XIP flash window base. Aliases at `+0x0100_0000`, `+0x0200_0000`,
/// `+0x0300_0000` mirror the same 2 MB flash buffer.
pub const XIP_FLASH_BASE: u32 = 0x1000_0000;

/// Read a PLL register image with CS[31] (LOCK) derived from the
/// current arm state and master cycle count. Non-CS offsets return the
/// raw stored value via `clocks::pll_read`.
///
/// See `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §4.
#[inline]
fn pll_read_with_lock(regs: &PllRegs, offset: u32, lock_at: Option<u64>, now: u64) -> u32 {
    if offset == 0x00 {
        pll_cs_read_with_lock(regs, lock_at, now)
    } else {
        clocks::pll_read(regs, offset)
    }
}

/// Returns `Some(offset)` if `addr` falls inside one of the four 2 MB
/// XIP flash alias windows (`0x10`, `0x11`, `0x12`, `0x13` at bits
/// [27:24]). The returned offset is the byte offset into the flash
/// buffer (in `0..FLASH_SIZE`).
#[inline]
pub(crate) fn xip_flash_offset(addr: u32) -> Option<u32> {
    // Region selector (bits [31:28]) must be 0x1 for XIP. Alias bits
    // [27:24] (values 0..3 for the four 2 MB aliases) are validated
    // below.
    if (addr & 0xF000_0000) != XIP_FLASH_BASE {
        return None;
    }
    // Alias select bits [27:24]: 0x0, 0x1, 0x2, 0x3.
    let alias = (addr >> 24) & 0xF;
    if alias > 3 {
        return None;
    }
    // Offset inside the 2 MB alias window.
    let offset = addr & 0x00FF_FFFF;
    if (offset as usize) < FLASH_SIZE {
        Some(offset)
    } else {
        None
    }
}

// PIO AHB windows (RP2040 datasheet §3 — two PIO blocks).
pub const PIO0_BASE: u32 = 0x5020_0000;
pub const PIO1_BASE: u32 = 0x5030_0000;

/// Translate an RP2040 PIO register offset to the internal PioBlock offset.
///
/// `PioBlock` uses RP2350-style offsets (INTR at 0x16C, IRQ0_INTE at 0x170,
/// …, IRQ1_INTS at 0x184) because that is the authoritative silicon layout.
/// The RP2040 has no RXFn_PUTGET / GPIOBASE registers, so its INT block
/// starts immediately after the per-SM block at 0x128 — 0x44 bytes earlier.
/// This helper compensates for that shift when the RP2040 bus dispatches
/// into PioBlock.
#[inline(always)]
const fn pio_rp2040_to_internal(offset: u32) -> u32 {
    if offset >= 0x128 && offset <= 0x140 {
        offset + 0x44
    } else {
        offset
    }
}

/// RP2040 PIO register read dispatch. Handles INTR and INTn_INTS using the
/// RP2040-specific bit layout (IRQ flags at [3:0], RXNEMPTY at [7:4],
/// TXNFULL at [11:8]); all other registers are forwarded through the
/// standard offset translator `pio_rp2040_to_internal`.
///
/// Only the INTR, INT0_INTS, and INT1_INTS reads differ between chips —
/// the INTE/INTF registers expose the same raw storage value regardless
/// of which bit layout is in use.
fn pio_read_rp2040(pio: &mut PioBlock, offset: u32) -> u32 {
    match offset {
        // INTR (RP2040 offset 0x128): raw status in RP2040 12-bit layout.
        0x128 => pio.raw_intr_rp2040(),
        // INT0_INTS (RP2040 offset 0x134): (INTR_rp2040 & INTE) | INTF.
        0x134 => pio.int0_ints_rp2040(),
        // INT1_INTS (RP2040 offset 0x140): (INTR_rp2040 & INTE) | INTF.
        0x140 => pio.int1_ints_rp2040(),
        // All other offsets: translate to RP2350-internal and dispatch normally.
        other => pio.read32(pio_rp2040_to_internal(other)),
    }
}

/// DMA peripheral base (RP2040 datasheet §2.5 — single 4 KB window
/// covering 12 channels + global registers + debug aliases). Phase 4
/// (HLD V7 §5.6) enables the full DMA model.
pub const DMA_BASE: u32 = 0x5000_0000;

/// XIP SRAM size (16 KB on RP2040 — the cache RAM exposed as scratch).
pub const XIP_SRAM_SIZE: usize = 16 * 1024;

/// Number of entries in the per-core PC-keyed decoded-op cache.
/// Direct-mapped, indexed by `(pc >> 1) & (DECODE_CACHE_SIZE - 1)`.
/// 8192 entries × 12 B = 96 KB per core (8 B in the
/// `decoded-op-8byte-prototype` experiment). Modelled on the RP2350 cache
/// (rp2350_emu commit 0c31479) but sized down: RP2040 hot loops are well
/// under 1 KB and total executable space (16 KB ROM + 16 KB XIP-SRAM +
/// 264 KB SRAM + 2 MB XIP flash) hashes to 8K slots without meaningful
/// conflict pressure for the workloads we measure.
pub(crate) const DECODE_CACHE_SIZE: usize = 8192;

/// One decoded ARMv6-M instruction, `Copy`.
///
/// Populated lazily on a cache miss by
/// [`crate::core::CortexM0Plus::populate_decode_cache`]. In the default
/// representation an entry with `tag == u32::MAX` is empty; the packed
/// representation uses its valid bit instead.
///
/// The `decoded-op-8byte-prototype` representation stores the upper 18 PC
/// bits and the wide flag in one `u32`; the direct-mapped slot supplies the
/// lower PC bits. This preserves the full tag while reducing the entry from
/// 12 to 8 bytes. The default representation remains unchanged.
///
/// Differs from the rp2350_emu [`crate::bus::DecodedOp`] equivalent by
/// dropping `fetch_wait` (RP2040 has no `extra_wait_states` accumulator
/// — `Bus::read16` writes `last_access_cycles` but the core path does
/// not consume it) and `is_thumb16_flag_only` (ARMv6-M has no IT
/// blocks).
#[cfg(not(feature = "decoded-op-8byte-prototype"))]
#[derive(Clone, Copy, Debug)]
pub(crate) struct DecodedOp {
    /// PC this entry is valid for. Full tag (no shift). `u32::MAX` =
    /// empty.
    pub tag: u32,
    /// First halfword (the one at PC).
    pub hw0: u16,
    /// Second halfword (at PC+2). Zero for narrow instructions.
    pub hw1: u16,
    /// Packed flags.
    ///   bit 0 — `is_wide`
    ///   bits 1..7 — reserved
    pub flags: u8,
}

#[cfg(feature = "decoded-op-8byte-prototype")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct DecodedOp {
    /// Bits [17:0] contain `pc >> 14`; bit 18 is `is_wide`; bit 19 is the
    /// valid bit. A zero valid bit is the empty/fault sentinel, allowing a
    /// fault result to retain its wide classification without becoming a
    /// cache hit.
    tag_flags: u32,
    /// First halfword (the one at PC).
    pub hw0: u16,
    /// Second halfword (at PC+2). Zero for narrow instructions.
    pub hw1: u16,
}

impl DecodedOp {
    #[cfg(not(feature = "decoded-op-8byte-prototype"))]
    pub(crate) const FLAG_WIDE: u8 = 0b0000_0001;

    #[cfg(all(
        feature = "compact-dispatch-key-prototype",
        not(feature = "decoded-op-8byte-prototype")
    ))]
    pub(crate) const FLAG_DISPATCH_KEY_MASK: u8 = 0b0111_1110;
    #[cfg(all(
        feature = "compact-dispatch-key-prototype",
        not(feature = "decoded-op-8byte-prototype")
    ))]
    pub(crate) const FLAG_DISPATCH_KEY_SHIFT: u8 = 1;

    #[cfg(feature = "decoded-op-8byte-prototype")]
    const TAG_MASK: u32 = (1 << 18) - 1;
    #[cfg(feature = "decoded-op-8byte-prototype")]
    const WIDE_MASK: u32 = 1 << 18;
    #[cfg(feature = "decoded-op-8byte-prototype")]
    const VALID_MASK: u32 = 1 << 19;

    #[inline(always)]
    pub(crate) fn empty() -> Self {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            return Self {
                tag_flags: 0,
                hw0: 0,
                hw1: 0,
            };
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        Self {
            tag: u32::MAX,
            hw0: 0,
            hw1: 0,
            flags: 0,
        }
    }

    #[inline(always)]
    pub(crate) fn is_wide(&self) -> bool {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            return self.tag_flags & Self::WIDE_MASK != 0;
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            self.flags & Self::FLAG_WIDE != 0
        }
    }

    /// Compact handler class stored in the otherwise-unused default flags.
    /// The OPT4-C packed entry is explicitly incompatible with this feature.
    #[cfg(all(
        feature = "compact-dispatch-key-prototype",
        not(feature = "decoded-op-8byte-prototype")
    ))]
    #[inline(always)]
    pub(crate) fn dispatch_key(&self) -> u8 {
        (self.flags & Self::FLAG_DISPATCH_KEY_MASK) >> Self::FLAG_DISPATCH_KEY_SHIFT
    }

    #[cfg(all(
        feature = "compact-dispatch-key-prototype",
        not(feature = "decoded-op-8byte-prototype")
    ))]
    #[inline(always)]
    pub(crate) fn with_dispatch_key(mut self, wide: bool, key: u8) -> Self {
        debug_assert!(key <= (Self::FLAG_DISPATCH_KEY_MASK >> Self::FLAG_DISPATCH_KEY_SHIFT));
        self.flags = (self.flags & !(Self::FLAG_WIDE | Self::FLAG_DISPATCH_KEY_MASK))
            | (if wide { Self::FLAG_WIDE } else { 0 })
            | ((key << Self::FLAG_DISPATCH_KEY_SHIFT) & Self::FLAG_DISPATCH_KEY_MASK);
        self
    }

    /// Build a valid decoded entry for a halfword-aligned PC.
    #[inline(always)]
    pub(crate) fn from_parts(pc: u32, hw0: u16, hw1: u16, wide: bool) -> Self {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            let mut tag_flags = Self::VALID_MASK | ((pc >> 14) & Self::TAG_MASK);
            if wide {
                tag_flags |= Self::WIDE_MASK;
            }
            Self {
                tag_flags,
                hw0,
                hw1,
            }
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            Self {
                tag: pc,
                hw0,
                hw1,
                flags: if wide { Self::FLAG_WIDE } else { 0 },
            }
        }
    }

    /// Build an uncached fetch result while retaining its decoded halfwords.
    #[inline(always)]
    pub(crate) fn fault_result(hw0: u16, hw1: u16, wide: bool) -> Self {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            let mut result = Self::empty();
            result.hw0 = hw0;
            result.hw1 = hw1;
            if wide {
                result.tag_flags = Self::WIDE_MASK;
            }
            result
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            Self {
                tag: u32::MAX,
                hw0,
                hw1,
                flags: if wide { Self::FLAG_WIDE } else { 0 },
            }
        }
    }

    /// Does this entry represent `pc` in the supplied direct-mapped slot?
    #[inline(always)]
    pub(crate) fn matches_pc(&self, pc: u32, _slot: usize) -> bool {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            !self.is_empty()
                && (self.tag_flags & Self::TAG_MASK) == ((pc >> 14) & Self::TAG_MASK)
                && (_slot as u32 & ((DECODE_CACHE_SIZE as u32) - 1))
                    == ((pc >> 1) & ((DECODE_CACHE_SIZE as u32) - 1))
                && pc & 1 == 0
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            !self.is_empty() && self.tag == pc
        }
    }

    #[inline(always)]
    #[allow(dead_code)]
    pub(crate) fn is_empty(&self) -> bool {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            self.tag_flags & Self::VALID_MASK == 0
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            self.tag == u32::MAX
        }
    }

    /// Reconstruct the full PC for diagnostics/tests from this entry and its
    /// direct-mapped slot. Returns `u32::MAX` for an empty entry.
    #[allow(dead_code)]
    #[inline(always)]
    pub(crate) fn tag_for_slot(&self, slot: usize) -> u32 {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            if self.is_empty() {
                u32::MAX
            } else {
                ((self.tag_flags & Self::TAG_MASK) << 14)
                    | (((slot as u32) & ((DECODE_CACHE_SIZE as u32) - 1)) << 1)
            }
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            let _ = slot;
            self.tag
        }
    }

    /// Set the full tag for a test/diagnostic cache fixture, retaining the
    /// current wide bit in the experimental packed representation.
    #[allow(dead_code)]
    #[inline(always)]
    pub(crate) fn set_tag_for_slot(&mut self, slot: usize, pc: u32) {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            let _ = slot;
            let wide = self.tag_flags & Self::WIDE_MASK;
            self.tag_flags = Self::VALID_MASK | ((pc >> 14) & Self::TAG_MASK) | wide;
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            let _ = slot;
            self.tag = pc;
        }
    }

    /// Original address-region nibble used by region-scoped invalidation.
    #[inline(always)]
    pub(crate) fn region_nibble(&self) -> u8 {
        #[cfg(feature = "decoded-op-8byte-prototype")]
        {
            if self.is_empty() {
                0xF
            } else {
                ((self.tag_flags >> 14) & 0xF) as u8
            }
        }
        #[cfg(not(feature = "decoded-op-8byte-prototype"))]
        {
            (self.tag >> 28) as u8
        }
    }
}

#[cfg(feature = "decoded-op-8byte-prototype")]
const _: () = assert!(core::mem::size_of::<DecodedOp>() == 8);
#[cfg(not(feature = "decoded-op-8byte-prototype"))]
const _: () = assert!(core::mem::size_of::<DecodedOp>() == 12);

/// True if `pc` lies in an executable region the cache may index.
/// Only ROM (`0x0`), XIP / XIP-SRAM (`0x1`), and SRAM (`0x2`) qualify.
/// Everything else (peripherals, SIO, PPB) either cannot legitimately
/// contain code or is dynamic and not worth caching.
#[inline(always)]
pub(crate) fn is_cacheable_pc(pc: u32) -> bool {
    matches!(pc >> 28, 0x0..=0x2)
}

/// Region bits for [`Bus::pending_invalidation_regions`] /
/// [`crate::core::CortexM0Plus::invalidate_decode_cache_regions`]. The
/// meaningful regions are the three cacheable ones (see
/// [`is_cacheable_pc`]). `BULK` is the universal escape hatch used when
/// the caller doesn't know (or doesn't care) which region changed.
pub mod invalidation_regions {
    /// Region `0x0` — boot ROM (16 KB).
    pub const ROM: u8 = 1 << 0;
    /// Region `0x1` — XIP / XIP-SRAM (flash window + cache scratch).
    pub const XIP: u8 = 1 << 1;
    /// Region `0x2` — on-chip SRAM (264 KB across 4 striped + 2 scratch
    /// banks).
    pub const SRAM: u8 = 1 << 2;
    /// Bulk bit — drain every slot regardless of tag region. Used by
    /// `ISB` and any path that can't attribute the change to a specific
    /// region (e.g. `Emulator::poke`, legacy bypass writes).
    pub const BULK: u8 = 1 << 7;
}

/// Maximum number of distinct `(addr, pc)` pairs retained by the
/// opt-in unsupported-MMIO accumulator (`Bus::unsupported_mmio_log`).
/// Bounds harness memory for firmware that faults across a wide
/// address window; occurrence counts for already-recorded pairs keep
/// accumulating past the cap.
pub const UNSUPPORTED_MMIO_LOG_CAP: usize = 4096;

/// RP2040 AHB-Lite bus fabric.
/// A device wired straight to the chip's pads.
///
/// Controllers like SPI or I2C hand a device whole words through their
/// FIFOs, but a PIO program drives pins directly, so a device on the far
/// end of one can only be observed by watching the pads change. `tick`
/// is called once per system cycle with the merged pad output; return
/// `Some((pin, level))` to drive an input pin back.
pub trait PinWatchingDevice: Send {
    fn tick(&mut self, gpio_out: u32) -> Option<(u8, bool)>;
}

pub struct Bus {
    pub memory: Memory,
    /// GPIO input state after merging SIO output with PIO outputs
    /// (Phase 5.A: SIO only). Read by firmware via SIO_GPIO_IN.
    pub gpio_in: u32,
    /// Per-core PPB (VTOR, SHPR, ICSR, active bitmap).
    pub ppb: [Ppb; 2],
    /// Single-cycle IO block.
    pub sio: Sio,
    /// RESETS peripheral.
    pub resets: Resets,
    /// CLOCKS register storage.
    pub clocks_regs: ClocksRegs,
    /// XOSC register storage.
    pub xosc_regs: XoscRegs,
    /// ROSC register storage.
    pub rosc_regs: RoscRegs,
    /// PLL_SYS register image (`[CS, PWR, FBDIV_INT, PRIM]`).
    pub pll_sys_regs: PllRegs,
    /// PLL_USB register image.
    pub pll_usb_regs: PllRegs,
    /// Master cycle count at the start of the current step. Populated by
    /// `Emulator::step` / `Emulator::run` before any core dispatch so that
    /// PLL CS reads and write-time lock-arm transitions observe a fresh
    /// cycle. See `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 P2.
    pub(crate) master_cycle: u64,
    /// Master cycle at which PLL_SYS's lock-detect counter expires. `None`
    /// means the PLL is not currently armed. Managed by the peripheral
    /// write dispatch via `picoem_common::clocks::pll_should_arm_lock`.
    pub(crate) pll_sys_lock_at_cycle: Option<u64>,
    /// Master cycle at which PLL_USB's lock-detect counter expires. Same
    /// semantics as `pll_sys_lock_at_cycle`.
    pub(crate) pll_usb_lock_at_cycle: Option<u64>,
    /// Derived clock tree frequencies (recomputed on any CLOCKS/PLL write).
    pub clock_tree: ClockTree,
    /// Single deterministic cycle-to-nanosecond snapshot shared by all
    /// optional external I2C devices and the harness observation API.
    virtual_time: VirtualClock,
    /// IO_BANK0 per-pin function select.
    pub io_bank0: IoBank0,
    /// PADS_BANK0 per-pin pad control.
    pub pads_bank0: PadsBank0,
    /// XIP SRAM (16 KB cache memory usable as SRAM, 0x1500_0000..0x1500_4000).
    xip_sram: Box<[u8; XIP_SRAM_SIZE]>,
    /// XIP_CTRL register backing store (stub — firmware round-trips only).
    xip_ctrl_regs: HashMap<u32, u32>,
    /// SSI register backing store (stub).
    ssi_regs: HashMap<u32, u32>,
    /// SPI flash as seen through XIP_SSI, for the boot-time commands
    /// firmware asks the chip rather than reading from the XIP window.
    pub ssi_flash: ssi_flash::SsiFlash,
    /// Catch-all APB peripheral register backing store for blocks we
    /// don't model in detail (PSM / BUSCTRL / SYSCFG / IO_QSPI / PADS_QSPI /
    /// UART / SPI / I2C / ADC / PWM / TIMER / WATCHDOG / RTC / VREG / TBMAN).
    /// Keyed by canonical word address (alias bits stripped).
    peripheral_regs: HashMap<u32, u32>,
    /// PIO0 / PIO1. Wired into the AHB decode at `0x5020_0000` /
    /// `0x5030_0000` (see [`PIO0_BASE`] / [`PIO1_BASE`]); output pins are
    /// merged into [`Self::gpio_in`] by [`crate::Emulator::update_gpio`].
    pub pio: [PioBlock; 2],
    /// Off-chip devices watching the pads (see [`PinWatchingDevice`]).
    pub pin_devices: Vec<Box<dyn PinWatchingDevice>>,
    /// WATCHDOG_TICK register model (Phase 1 scope — HLD V7 §5.5). Only
    /// the `TICK` register at offset `0x2C` is modelled today; the rest
    /// of the WATCHDOG block reads as 0.
    pub watchdog_tick: WatchdogTickRegs,
    /// Sticky request raised by WATCHDOG CTRL.TRIGGER.  The CPU scheduler
    /// consumes it only after the current instruction retires, so the core
    /// executing the trigger never executes a following instruction.
    pub(crate) watchdog_reset_requested: bool,
    /// TIMER register model (Phase 1 Wave 2 — HLD V7 §5.3). Lazy
    /// microsecond counter + four alarms; `advance_lazy_scheduled`
    /// polls `poll_alarms` on every step tail to surface alarm-match
    /// IRQs into `irq_pending`.
    pub(crate) timer: TimerRegs,
    /// UART0 — PL011-derived (Phase 2 — HLD V7 §5.3).
    pub(crate) uart0: UartRegs,
    /// UART1 — PL011-derived.
    pub(crate) uart1: UartRegs,
    /// SPI0 — PL022-derived.
    pub(crate) spi0: SpiRegs,
    /// SPI1 — PL022-derived.
    pub(crate) spi1: SpiRegs,
    /// I2C0 — DW_apb_i2c.
    pub(crate) i2c0: I2cRegs,
    /// I2C1 — DW_apb_i2c.
    pub(crate) i2c1: I2cRegs,
    /// ADC — single instance at 0x4004_C000 (Phase 3 — HLD V7 §5.3).
    /// Runs on `clk_adc` (48 MHz nominal) via a fixed-point accumulator
    /// scaling from `clk_sys`. Reset-gated on RESETS bit 0.
    pub(crate) adc: AdcRegs,
    /// PWM — 8-slice block at 0x4005_0000 (Phase 3 — HLD V7 §5.3).
    /// Phase 3 cadence: CTR += 1 per sys_clk on enabled slices (DIV
    /// ignored); wrap latches `INTR[slice]` and routes `PWM_IRQ_WRAP`.
    /// Reset-gated on RESETS bit 14.
    pub(crate) pwm: PwmRegs,
    /// DMA controller — Phase 1 stub (always idle). Phase 4 replaces
    /// this with the 12-channel model. Consulted by the fast-path gate
    /// in [`crate::Emulator::step`] via [`Dma::is_idle`].
    ///
    /// `pub(crate)` because external readers (diagnostic harnesses
    /// such as `picogus_diff_rp2040` that need per-channel observation
    /// counters on `DmaChannel`) go through [`Self::dma_channel`]
    /// instead. Control still flows through `bus.write32`.
    pub(crate) dma: Dma,
    /// Pending external IRQ bitmap (bit N = IRQ #N asserted this
    /// cycle). Peripherals OR into this field when their state raises
    /// a line; [`crate::Emulator::drain_pending_irqs_to_cores`] drains
    /// the bitmap into both cores' NVIC pending latches per inner-loop
    /// iteration (HLD V7 §5.2).
    pub(crate) irq_pending: u32,
    /// Per-core NVIC — one pending/enabled/priority set per CPU. The
    /// System Control Space NVIC registers (`0xE000_E100..0xE000_E41F`)
    /// are banked per-core on M0+; `nvics[active_core]` is the one the
    /// currently-executing CPU sees. Drained from [`Self::irq_pending`]
    /// by [`crate::Emulator::drain_pending_irqs_to_cores`], polled by
    /// [`crate::core::CortexM0Plus::step`] for dispatch.
    pub nvics: [Nvic; 2],
    /// Per-core SysTick (24-bit countdown timer mapped at
    /// `0xE000_E010..0xE000_E01F`). HLD V5 §5.2: ticked once per master
    /// cycle for the active core only; on a TICKINT-arm underflow the
    /// caller ORs `ICSR.PENDSTSET` (bit 26) onto the active-core PPB.
    pub(crate) systicks: [SysTick; 2],
    /// Off-chip 8 MB SPI PSRAM (PicoGUS v2 hardware). `None` when no
    /// PSRAM is attached (e.g. non-PicoGUS boards). Observed via
    /// [`crate::Emulator::update_gpio`] on the device's pin assignments
    /// and drives MISO back into [`Self::gpio_in`].
    pub psram: Option<Psram>,
    /// Externally-driven GPIO input override values. Bits set in
    /// [`Self::external_gpio_in_mask`] take their value from this field
    /// instead of whatever SIO / PIO / PSRAM would have produced; bits
    /// not set in the mask are unaffected. Used by the
    /// `picogus_diff_rp2040` harness to inject synthetic ISA bus
    /// waveforms (IOW#, IOR#, AD0..AD9) without those pokes being
    /// clobbered by [`crate::Emulator::update_gpio`]. The override is
    /// applied last in the merge — after SIO, PIO, and PSRAM — so an
    /// external driver always wins on the pins it claims.
    pub external_gpio_in_override: u32,
    /// Mask of GPIO input bits driven externally (see
    /// [`Self::external_gpio_in_override`]). The harness sets this to
    /// cover the ISA pins it injects (IOW#, IOR#, AD0..AD9) but **not**
    /// PSRAM pins (GPIO0..3) — those still belong to the on-chip merge.
    pub external_gpio_in_mask: u32,
    /// Per-core event flag for WFE/SEV / FIFO event protocol.
    pub event_flag: [bool; 2],
    /// Per-core WFE-park flag. `true` means the core is sleeping on
    /// WFE and will not execute until [`Self::event_flag`] for that
    /// core is consumed at a quantum boundary by
    /// `Emulator::wake_checks`. See `wrk_docs/2026.04.26 - HLD - RP2040
    /// WFE-SEV Wake Mechanics V1.md` §4.1.
    pub wfe_waiting: [bool; 2],
    /// Which core is currently executing on the bus.
    active_core: usize,
    /// Cycle cost of the most recent bus access.
    last_access_cycles: u32,
    /// OPT2-B diagnostic latch for CPU-visible synchronization accesses
    /// performed during the current Serial dispatch.  It is compiled out of
    /// production builds and drained before peripheral/DMA advancement, so
    /// bus traffic generated by a DMA tick cannot be mistaken for CPU MMIO.
    #[cfg(feature = "event-horizon-profiler")]
    running_cpu_boundary_mask: crate::running_profile::RunningBoundaryMask,
    /// Bus fault sticky flags.
    bus_fault: bool,
    bus_fault_addr: u32,
    /// Per-quantum bank-touched bitmap — bit N = bank N was accessed by
    /// the core 0 step. Read by core 1 to compute +1 cycle contention.
    core0_bank_touched: u8,
    /// True while the currently-running core is core 1 and it is looking
    /// up contention against `core0_bank_touched`. Set/cleared by the
    /// dual-core scheduler via `begin_core1_step` / `end_core1_step`.
    contention_check_active: bool,
    /// MMIO trace toggle (see `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral
    /// Coverage V7.md` §4.3). When `true`, each byte/half/word bus access
    /// and each peripheral/SIO/PPB dispatch emits a line to
    /// [`Self::mmio_trace_sink`] (defaults to stdout when `None`). Zero overhead
    /// when `false` — the hot path short-circuits before any formatting.
    pub mmio_trace_enabled: bool,
    /// Unsupported-MMIO observation toggle (headless firmware runner,
    /// `picocalc-harness`). When `true`, every [`Self::set_bus_fault`]
    /// — the single funnel for accesses that hit no decoded region —
    /// records `(addr, active_pc)` into [`Self::unsupported_mmio_log`]
    /// with an occurrence count. Purely observational: the emulated
    /// program sees exactly the same bus behaviour either way (the
    /// fault is still sticky, still escalates to HardFault in
    /// `CortexM0Plus::step`). Zero cost when `false`.
    ///
    /// Coverage note: this records accesses to *undecoded* addresses.
    /// A peripheral that is decoded but only partially modelled does
    /// not fault and therefore does not appear here — use
    /// `mmio_trace_enabled` for the full access log.
    pub unsupported_mmio_log_enabled: bool,
    /// `(addr, pc) -> count` accumulator filled while
    /// [`Self::unsupported_mmio_log_enabled`] is set. Read back with
    /// [`Self::unsupported_mmio_log`] (sorted, deterministic).
    unsupported_mmio_log: HashMap<(u32, u32), u64>,
    /// Per-core, per-instruction PC snapshot. Indexed by `active_core`
    /// so that a scheduler switch (`set_active_core(0→1→0)`) does not
    /// alias one core's decode PC onto the other. Set by the core's
    /// decode path (`CortexM0Plus::decode_execute`) immediately before
    /// instruction fetch, so every read/write during that instruction
    /// carries the correct architectural PC. Also set to sentinel values
    /// by `enter_exception` / `exit_exception` when hardware stacks or
    /// unstacks the 8-word frame (see `core::exceptions`).
    /// Default `[0, 0]`; only meaningful while a core is executing.
    pub(crate) active_pc: [u32; 2],
    /// Optional override sink for MMIO trace output. `None` routes to stdout
    /// via `println!`. Unit tests inject a `Vec<u8>`-backed sink to
    /// capture lines without wrestling with fd 1 redirection.
    pub(crate) mmio_trace_sink: Option<Box<dyn Write>>,
    /// Dirty-range log for the per-core decode caches. Every SRAM /
    /// XIP-SRAM write pushes the target halfword address(es) here; the
    /// driver (`Emulator::step_serial`) drains this into the core that
    /// just ran, evicting stale entries. Mirrors the rp2350_emu
    /// `Bus::pending_cache_invalidations` mechanism (commit 0c31479).
    pub pending_cache_invalidations: Vec<u32>,
    /// Region-scoped bulk-invalidation bitmask. Set by
    /// [`Self::load_bootrom`] (bit [`invalidation_regions::ROM`]),
    /// [`Self::load_flash`] (bit [`invalidation_regions::XIP`]), and ISB
    /// execution (bit [`invalidation_regions::BULK`]) when a write has
    /// replaced executable bytes wholesale. The driver drains the mask
    /// by calling
    /// [`crate::core::CortexM0Plus::invalidate_decode_cache_regions`] on
    /// each core and then resets it to `0`.
    pub pending_invalidation_regions: u8,
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: Memory::with_flash(ROM_SIZE, SRAM_SIZE, FLASH_SIZE),
            gpio_in: 0,
            ppb: [Ppb::new(), Ppb::new()],
            sio: Sio::new(),
            resets: Resets::new(),
            clocks_regs: ClocksRegs::new(),
            xosc_regs: XoscRegs::new(),
            rosc_regs: RoscRegs::new(),
            pll_sys_regs: PLL_RESET,
            pll_usb_regs: PLL_RESET,
            master_cycle: 0,
            pll_sys_lock_at_cycle: None,
            pll_usb_lock_at_cycle: None,
            clock_tree: ClockTree::default(),
            virtual_time: VirtualClock::new(ClockTree::default().sys_clk_hz),
            io_bank0: IoBank0::new(),
            pads_bank0: PadsBank0::new(),
            xip_sram: Box::new([0u8; XIP_SRAM_SIZE]),
            xip_ctrl_regs: HashMap::new(),
            ssi_regs: HashMap::new(),
            ssi_flash: ssi_flash::SsiFlash::new(),
            peripheral_regs: HashMap::new(),
            pio: [PioBlock::new(), PioBlock::new()],
            pin_devices: Vec::new(),
            watchdog_tick: WatchdogTickRegs::new(),
            watchdog_reset_requested: false,
            timer: TimerRegs::new(),
            uart0: UartRegs::new(IRQ_UART0_IRQ),
            uart1: UartRegs::new(IRQ_UART1_IRQ),
            spi0: SpiRegs::new(IRQ_SPI0_IRQ),
            spi1: SpiRegs::new(IRQ_SPI1_IRQ),
            i2c0: I2cRegs::new(IRQ_I2C0_IRQ),
            i2c1: I2cRegs::new(IRQ_I2C1_IRQ),
            adc: AdcRegs::new(IRQ_ADC_IRQ_FIFO),
            pwm: PwmRegs::new(IRQ_PWM_IRQ_WRAP),
            dma: Dma::new(),
            irq_pending: 0,
            nvics: [Nvic::new(), Nvic::new()],
            systicks: [SysTick::new(), SysTick::new()],
            psram: None,
            external_gpio_in_override: 0,
            external_gpio_in_mask: 0,
            event_flag: [false; 2],
            wfe_waiting: [false; 2],
            active_core: 0,
            last_access_cycles: 0,
            #[cfg(feature = "event-horizon-profiler")]
            running_cpu_boundary_mask: Default::default(),
            bus_fault: false,
            bus_fault_addr: 0,
            core0_bank_touched: 0,
            contention_check_active: false,
            mmio_trace_enabled: false,
            unsupported_mmio_log_enabled: false,
            unsupported_mmio_log: HashMap::new(),
            active_pc: [0; 2],
            mmio_trace_sink: None,
            // 16 entries up front — STM tops out at 13 registers; 16
            // covers a worst-case STM PC-rewrite without reallocation.
            pending_cache_invalidations: Vec::with_capacity(16),
            pending_invalidation_regions: 0,
        }
    }

    // --- Active-core / scheduler plumbing ---------------------------------

    #[inline]
    pub fn active_core(&self) -> usize {
        self.active_core
    }

    #[inline]
    pub fn set_active_core(&mut self, core: usize) {
        debug_assert!(core < 2);
        self.active_core = core;
    }

    /// PC attributed to the instruction currently executing on the active
    /// core.  Used only for deterministic watchdog reset provenance.
    #[inline]
    pub(crate) fn active_pc_for_event(&self) -> u32 {
        self.active_pc[self.active_core]
    }

    #[inline]
    pub(crate) fn take_watchdog_reset_request(&mut self) -> bool {
        std::mem::take(&mut self.watchdog_reset_requested)
    }

    #[cfg(feature = "event-horizon-profiler")]
    pub(crate) fn reset_running_cpu_boundaries(&mut self) {
        self.running_cpu_boundary_mask = Default::default();
    }

    #[cfg(feature = "event-horizon-profiler")]
    pub(crate) fn take_running_cpu_boundaries(
        &mut self,
    ) -> crate::running_profile::RunningBoundaryMask {
        std::mem::take(&mut self.running_cpu_boundary_mask)
    }

    /// Classify CPU-visible synchronization accesses for the OPT2-B
    /// opportunity profile. Categories deliberately overlap: GPIO_IN and
    /// FIFO/DREQ reads are also ordinary MMIO boundaries.
    #[cfg(feature = "event-horizon-profiler")]
    #[inline]
    fn note_running_cpu_access(&mut self, addr: u32, read: bool) {
        use crate::running_profile::RunningBoundaryMask as M;

        let region = addr >> 28;
        if !matches!(region, 0x4 | 0x5 | 0xD | 0xE) {
            return;
        }
        self.running_cpu_boundary_mask.insert(M::CPU_MMIO);

        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        if read && base == SIO_BASE && (offset & !3) == 0x004 {
            self.running_cpu_boundary_mask.insert(M::GPIO_IN);
        }

        let fifo_or_dreq = (base == SIO_BASE && (0x050..=0x058).contains(&(offset & !3)))
            || ((base == PIO0_BASE || base == PIO1_BASE)
                && (0x010..=0x02c).contains(&(offset & !3)))
            || matches!(
                base,
                DMA_BASE
                    | UART0_BASE
                    | UART1_BASE
                    | SPI0_BASE
                    | SPI1_BASE
                    | I2C0_BASE
                    | I2C1_BASE
                    | ADC_BASE
            );
        if fifo_or_dreq {
            self.running_cpu_boundary_mask.insert(M::FIFO_DREQ);
        }
    }

    /// Stash the instruction PC of the currently-executing instruction
    /// for the *currently-active core*. Called by
    /// [`crate::core::CortexM0Plus::decode_execute`] before instruction
    /// fetch so the MMIO trace can report a meaningful PC for every
    /// access that instruction performs. Also called by exception
    /// entry / exit with sentinel values (`0xFFFF_FFFE`,
    /// `0xFFFF_FFFD`) so stacking / unstacking lines are distinguishable
    /// from ordinary instruction-driven access. See HLD V7 §4.3.
    #[inline]
    pub fn set_active_pc(&mut self, pc: u32) {
        self.active_pc[self.active_core] = pc;
    }

    /// Emit a single MMIO trace line. `rw` is `'R'` or `'W'`, `size` is 1/2/4
    /// bytes, `val` is the value read or written. Zero overhead when
    /// [`Self::mmio_trace_enabled`] is `false`; the caller is expected to gate
    /// with `if self.mmio_trace_enabled` so the formatting cost is paid only
    /// when tracing.
    ///
    /// Routes to [`Self::mmio_trace_sink`] if set, else `println!` (stdout).
    /// No buffering — each line flushes at the `writeln!` boundary.
    ///
    /// Coverage note (see HLD V7 §4.3). The trace is emitted only from the
    /// six outer access methods ([`Self::read8`] … [`Self::write32`]). The
    /// internal peripheral/SIO/PPB dispatch helpers (`peripheral_read32`,
    /// `peripheral_write32`, `sio_read32`, `sio_write32`,
    /// `ppb[].read32/write32`) are **only reachable** from those six
    /// methods — they have no other callers in the crate (verified by
    /// grep) and are not `pub`. So outer-only tracing covers 100% of the
    /// MMIO surface firmware can touch, at one line per architectural
    /// access. Hooking the inner helpers as well would double-emit on
    /// word-sized peripheral access (outer calls inner directly) and
    /// surface the byte/half RMW-through-word32 artefact on narrow
    /// peripheral access — neither of which helps the "what does firmware
    /// touch next?" workflow the oracle is meant to unblock.
    #[inline(never)]
    fn emit_mmio_trace(&mut self, rw: char, size: u32, addr: u32, val: u32) {
        let line = format!(
            "TRACE {} {} 0x{:08X} val=0x{:08X} core={} pc=0x{:08X}",
            rw, size, addr, val, self.active_core, self.active_pc[self.active_core]
        );
        if let Some(sink) = self.mmio_trace_sink.as_mut() {
            let _ = writeln!(sink, "{}", line);
        } else {
            println!("{}", line);
        }
    }

    /// Install a captured MMIO trace sink (used by unit tests). `None` routes
    /// back to stdout. This is `pub(crate)` to keep it off the public
    /// surface — the binary toggles `mmio_trace_enabled` only.
    #[cfg(test)]
    pub(crate) fn set_mmio_trace_sink(&mut self, sink: Option<Box<dyn Write>>) {
        self.mmio_trace_sink = sink;
    }

    /// Called before core 1 steps each quantum — enables the contention
    /// check that adds +1 cycle when core 1 touches an SRAM bank already
    /// touched by core 0.
    #[inline]
    pub fn begin_core1_step(&mut self) {
        self.contention_check_active = true;
    }

    /// Called after core 1 has finished its slice. Clears the
    /// contention window and wipes the core-0 bank map for the next
    /// quantum.
    #[inline]
    pub fn end_core1_step(&mut self) {
        self.contention_check_active = false;
        self.core0_bank_touched = 0;
    }

    // --- Clock-tree accessors --------------------------------------------

    #[inline]
    pub fn sys_clk_hz(&self) -> u32 {
        self.clock_tree.sys_clk_hz
    }

    #[inline]
    pub fn ref_clk_hz(&self) -> u32 {
        self.clock_tree.ref_clk_hz
    }

    /// Seed the derived clock tree with an initial frequency. First
    /// write to CLOCKS / PLL replaces the seed with the derived value.
    pub fn seed_sys_clk_hz(&mut self, hz: u32) {
        self.clock_tree.sys_clk_hz = hz;
        self.clock_tree.ref_clk_hz = hz;
        self.virtual_time.rebase(self.master_cycle, hz);
    }

    /// Current deterministic virtual time in nanoseconds.
    #[inline]
    pub fn virtual_time_ns(&self) -> u64 {
        self.virtual_time.ns_at(self.master_cycle)
    }

    /// Convert a virtual nanosecond deadline to the corresponding absolute
    /// master cycle using the shared snapshot.
    #[inline]
    pub fn virtual_time_cycles_at(&self, ns: u64) -> u64 {
        self.virtual_time.cycles_at(ns)
    }

    /// Reset the shared virtual-time epoch for a cold emulator reset.
    pub(crate) fn reset_virtual_time(&mut self) {
        self.virtual_time.reset(self.clock_tree.sys_clk_hz);
    }

    /// Restore virtual time after a watchdog warm reset. The external
    /// module state remains attached, so elapsed time must not jump back to
    /// zero merely because the MCU reset domain did.
    pub(crate) fn restore_virtual_time_after_reset(&mut self, ns: u64) {
        self.virtual_time
            .restore_after_reset(self.master_cycle, ns, self.clock_tree.sys_clk_hz);
    }

    /// Advance the shared snapshot once and deliver the resulting delta to
    /// both I2C controllers. This is called exactly once for each peripheral
    /// window, regardless of whether the scheduler used normal ticking or
    /// lazy fast-forward.
    #[inline]
    fn advance_external_virtual_time(&mut self) {
        let nanoseconds = self
            .virtual_time
            .advance_to(self.master_cycle, self.clock_tree.sys_clk_hz);
        let delta = crate::peripherals::i2c::I2cVirtualTimeDelta { nanoseconds };
        self.i2c0.advance_virtual_time(delta);
        self.i2c1.advance_virtual_time(delta);
    }

    fn recompute_clock_tree(&mut self) {
        clocks::recompute(
            &self.clocks_regs,
            &self.pll_sys_regs,
            &self.pll_usb_regs,
            &mut self.clock_tree,
        );
        debug!(
            sys_clk_hz = self.clock_tree.sys_clk_hz,
            ref_clk_hz = self.clock_tree.ref_clk_hz,
            peri_clk_hz = self.clock_tree.peri_clk_hz,
            "clock tree recomputed"
        );
    }

    // --- Flash / XIP management ------------------------------------------

    /// Copy `data` into the 2 MB XIP flash window at offset 0. Oversized
    /// images are clamped by [`Memory::load_flash`]; the mapped window
    /// is always 2 MB so reads past the image length return 0.
    pub fn load_flash(&mut self, data: &[u8]) {
        self.memory.load_flash(data);
        // XIP / XIP-SRAM share region nibble 0x1 in the cache region
        // mask, so a flash reload invalidates any cached entry that
        // tagged into either window.
        self.pending_invalidation_regions |= invalidation_regions::XIP;
    }

    /// Snapshot the current XIP flash image, including SSI erase/program
    /// mutations performed by firmware during the run.
    pub fn flash_image(&self) -> Vec<u8> {
        self.memory.xip_image()
    }

    /// SSI flash protocol/range errors accumulated during the run.
    pub fn flash_mutation_errors(&self) -> &[String] {
        &self.ssi_flash.errors
    }

    /// Unknown SSI opcodes observed during the run.
    pub fn flash_unknown_commands(&self) -> &[(u8, u32)] {
        &self.ssi_flash.unknown_commands
    }

    /// Every SSI flash opcode observed, including commands the model does
    /// not yet implement. Counts are bounded by opcode, not transaction.
    pub fn flash_command_counts(&self) -> &[(u8, u32)] {
        &self.ssi_flash.command_counts
    }

    pub fn flash_erase_count(&self) -> u64 {
        self.ssi_flash.erase_count
    }

    pub fn flash_program_count(&self) -> u64 {
        self.ssi_flash.program_count
    }

    pub fn flash_program_bytes(&self) -> u64 {
        self.ssi_flash.program_bytes
    }

    // --- Bus-fault plumbing -----------------------------------------------

    pub fn bus_fault(&self) -> bool {
        self.bus_fault
    }

    pub fn bus_fault_addr(&self) -> u32 {
        self.bus_fault_addr
    }

    pub fn clear_bus_fault(&mut self) {
        self.bus_fault = false;
    }

    /// Set the sticky bus-fault flag and record the faulting address.
    /// Emits a tracing event on the cold path so unmapped accesses are
    /// observable without throwaway `eprintln!`.
    #[inline]
    fn set_bus_fault(&mut self, addr: u32) {
        debug!(addr = format_args!("{:#010x}", addr), "unmapped bus access");
        if self.unsupported_mmio_log_enabled {
            self.record_unsupported_mmio(addr);
        }
        self.bus_fault = true;
        self.bus_fault_addr = addr;
    }

    /// Cold-path accumulator for [`Self::unsupported_mmio_log`]. Split
    /// out of `set_bus_fault` so the disabled path stays a single
    /// predictable branch. Capped at [`UNSUPPORTED_MMIO_LOG_CAP`]
    /// distinct `(addr, pc)` pairs — a firmware faulting in a tight
    /// loop over a large address window must not grow the map without
    /// bound. Counts for already-known pairs keep accumulating after
    /// the cap is hit.
    #[inline(never)]
    #[cold]
    fn record_unsupported_mmio(&mut self, addr: u32) {
        let pc = self.active_pc[self.active_core];
        let key = (addr, pc);
        if let Some(count) = self.unsupported_mmio_log.get_mut(&key) {
            *count = count.saturating_add(1);
        } else if self.unsupported_mmio_log.len() < UNSUPPORTED_MMIO_LOG_CAP {
            self.unsupported_mmio_log.insert(key, 1);
        }
    }

    /// Snapshot of the unsupported-MMIO accumulator as
    /// `(addr, pc, count)`, sorted by `(addr, pc)` so repeated runs of
    /// the same firmware produce byte-identical reports. Empty unless
    /// [`Self::unsupported_mmio_log_enabled`] was set during the run.
    pub fn unsupported_mmio_log(&self) -> Vec<(u32, u32, u64)> {
        let mut out: Vec<(u32, u32, u64)> = self
            .unsupported_mmio_log
            .iter()
            .map(|(&(addr, pc), &count)| (addr, pc, count))
            .collect();
        out.sort_unstable_by_key(|&(addr, pc, _)| (addr, pc));
        out
    }

    /// `true` if the accumulator hit [`UNSUPPORTED_MMIO_LOG_CAP`] and
    /// may therefore be missing distinct `(addr, pc)` pairs.
    pub fn unsupported_mmio_log_truncated(&self) -> bool {
        self.unsupported_mmio_log.len() >= UNSUPPORTED_MMIO_LOG_CAP
    }

    // --- Direct peek/poke (bypasses decode, still routes through regions)

    pub fn peek32(&self, addr: u32) -> u32 {
        if (addr >> 28) == 0x2 {
            // SRAM
            self.memory.sram_read32(addr & 0x00FF_FFFF)
        } else if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&addr) {
            let off = (addr - XIP_SRAM_BASE) as usize;
            u32::from_le_bytes([
                self.xip_sram[off],
                self.xip_sram[off + 1],
                self.xip_sram[off + 2],
                self.xip_sram[off + 3],
            ])
        } else {
            self.memory.peek32(addr)
        }
    }

    pub fn poke32(&mut self, addr: u32, value: u32) {
        if (addr >> 28) == 0x2 {
            self.memory.sram_write32(addr & 0x00FF_FFFF, value);
        } else if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&addr) {
            let off = (addr - XIP_SRAM_BASE) as usize;
            let bytes = value.to_le_bytes();
            self.xip_sram[off..off + 4].copy_from_slice(&bytes);
        } else {
            self.memory.poke32(addr, value);
        }
    }

    // --- Raw ROM loader (used by `Emulator::load_bootrom`) ----------------

    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.memory.load_rom(data);
        self.pending_invalidation_regions |= invalidation_regions::ROM;
    }

    /// Queue cache invalidations covering `[addr, addr+len)` on the
    /// per-core decode caches. `len` is 1, 2, or 4 bytes. Pushes into
    /// [`Self::pending_cache_invalidations`]; the driver
    /// (`Emulator::step_serial`) drains it into the core that just ran.
    /// Mirrors the rp2350_emu pattern (commit 0c31479).
    ///
    /// The drainer
    /// ([`crate::core::CortexM0Plus::invalidate_decode_cache_entries`])
    /// evicts both the slot for `addr` and the slot for `addr - 2`
    /// (covering a wide instruction whose `hw1` landed at `addr`). For
    /// a 4-byte write we push `{addr, addr+2}` so the combined
    /// per-slot coverage is `{addr-2, addr, addr+2}`.
    #[inline]
    fn invalidate_pc_range(&mut self, addr: u32, len: u8) {
        debug_assert!(len == 1 || len == 2 || len == 4);
        if matches!(addr >> 28, 0x0..=0x2) {
            self.pending_cache_invalidations.push(addr);
            if len == 4 {
                self.pending_cache_invalidations.push(addr.wrapping_add(2));
            }
        }
    }

    // --- Latency helpers --------------------------------------------------

    #[inline]
    pub fn last_access_cycles(&self) -> u32 {
        self.last_access_cycles
    }

    /// Harness-only diagnostic: drain every byte firmware has written to
    /// `UART0.DR` since the previous call. See `UartRegs::drain_tx_log`.
    pub fn drain_uart0_tx_log(&mut self) -> Vec<u8> {
        self.uart0.drain_tx_log()
    }

    /// Drain UART0 TX writes with their exact virtual bus cycles. The
    /// byte-only accessor remains the compatibility path for reports; this
    /// richer tap is used by the realtime preview transport.
    pub fn drain_uart0_tx_log_with_cycles(&mut self) -> Vec<(u64, u8)> {
        self.uart0.drain_tx_log_with_cycles()
    }

    /// Enable virtual-cycle metadata on the UART0 TX diagnostic tap. This is
    /// preview-only and leaves the authoritative byte-only runner path
    /// unchanged.
    pub fn enable_uart0_tx_cycle_tap(&mut self) {
        self.uart0.enable_wire_cycle_tap();
    }

    /// Inject one byte on the external UART0 RX wire.  This is a harness /
    /// preview operation; ordinary guest MMIO and authoritative batch runs
    /// remain unchanged.  The UART model applies its enable, FIFO-capacity,
    /// IRQ and overrun semantics before returning the result.
    pub fn inject_uart0_rx(&mut self, byte: u8) -> UartRxResult {
        self.uart0.inject_rx(byte, &mut self.irq_pending)
    }

    /// Return the number of bytes waiting in the UART0 guest RX FIFO.
    pub fn uart0_rx_fifo_len(&self) -> usize {
        self.uart0.rx_fifo_len()
    }

    /// Return UART0's raw interrupt status for preview diagnostics.
    pub fn uart0_raw_interrupt_status(&self) -> u32 {
        self.uart0.raw_interrupt_status()
    }

    #[cfg(feature = "behavior-trace")]
    pub(crate) fn drain_uart0_behavior_tx_log(&mut self) -> Vec<u8> {
        self.uart0.drain_behavior_tx_log()
    }

    #[cfg(feature = "behavior-trace")]
    pub(crate) fn behavior_serial_state(&self) -> [u64; 34] {
        let mut state = [0u64; 34];
        state[0..4].copy_from_slice(&self.uart0.behavior_trace_state());
        state[4..8].copy_from_slice(&self.uart1.behavior_trace_state());
        state[8..14].copy_from_slice(&self.spi0.behavior_trace_state());
        state[14..20].copy_from_slice(&self.spi1.behavior_trace_state());
        state[20..27].copy_from_slice(&self.i2c0.behavior_trace_state());
        state[27..34].copy_from_slice(&self.i2c1.behavior_trace_state());
        state
    }

    /// Borrow a single DMA channel's observation state.
    ///
    /// External diagnostic harnesses (e.g. `picogus_diff_rp2040`) read
    /// per-channel counters on `DmaChannel` to verdict scenarios that
    /// MMIO can't surface without write side-effects. The accessor
    /// keeps `dma` itself `pub(crate)` while preserving the read-only
    /// observability the harness needs.
    ///
    /// Panics if `i >= crate::dma::NUM_CHANNELS` (matches the
    /// underlying `Dma::channel` slice index).
    #[inline]
    pub fn dma_channel(&self, i: usize) -> &crate::dma::DmaChannel {
        self.dma.channel(i)
    }

    /// Snapshot DMA-origin writes to the PicoCalc PWM audio sink.
    pub fn audio_sink_snapshot(&self) -> crate::AudioSinkSnapshot {
        self.dma
            .audio_sink_snapshot_at_clock(self.clock_tree.sys_clk_hz)
    }

    /// Snapshot DMA timer pacing and digital audio observation state.
    pub fn dma_scheduler_snapshot(&self) -> crate::DmaSchedulerSnapshot {
        self.dma
            .scheduler_snapshot_at_clock(self.clock_tree.sys_clk_hz)
    }

    /// Enable optional PCM retention for a later diagnostic WAV export.
    pub fn enable_audio_pcm_capture(&mut self) {
        self.dma.enable_audio_pcm_capture();
    }

    /// Take the optional interleaved stereo PCM retained by the audio sink.
    pub fn take_audio_pcm_capture(&mut self) -> Option<Vec<i16>> {
        self.dma.take_audio_pcm_capture()
    }

    /// Base read latency for an address region (cycles).
    #[inline]
    fn read_latency(region: u32) -> u32 {
        match region {
            0x0 => 1, // ROM
            0x1 => 1, // XIP / XIP_CTRL / SSI
            0x2 => 1, // SRAM
            0x4 => 3, // APB peripherals
            0x5 => 1, // AHB peripherals
            0xD => 1, // SIO
            0xE => 1, // PPB
            _ => 1,
        }
    }

    #[inline]
    fn write_latency(region: u32) -> u32 {
        match region {
            0x4 => 4, // APB writes
            _ => 1,
        }
    }

    /// Record an SRAM bank touch for the active core and return any
    /// contention wait states (simple +1 model).
    #[inline]
    fn note_sram_access(&mut self, addr: u32) -> u32 {
        if let Some(bank) = bank_for_address(addr) {
            let bit = 1u8 << (bank & 7);
            let wait = if self.contention_check_active && self.core0_bank_touched & bit != 0 {
                1
            } else {
                0
            };
            if self.active_core == 0 {
                self.core0_bank_touched |= bit;
            }
            wait
        } else {
            0
        }
    }

    // --- XIP SRAM scratch helpers ----------------------------------------

    fn xip_sram_read(&self, addr: u32, width: usize) -> u32 {
        let off = (addr - XIP_SRAM_BASE) as usize;
        let end = off + width;
        if end <= self.xip_sram.len() {
            match width {
                1 => self.xip_sram[off] as u32,
                2 => u16::from_le_bytes([self.xip_sram[off], self.xip_sram[off + 1]]) as u32,
                4 => u32::from_le_bytes([
                    self.xip_sram[off],
                    self.xip_sram[off + 1],
                    self.xip_sram[off + 2],
                    self.xip_sram[off + 3],
                ]),
                _ => 0,
            }
        } else {
            0
        }
    }

    fn xip_sram_write(&mut self, addr: u32, val: u32, width: usize) {
        let off = (addr - XIP_SRAM_BASE) as usize;
        let end = off + width;
        if end <= self.xip_sram.len() {
            let bytes = val.to_le_bytes();
            for i in 0..width {
                self.xip_sram[off + i] = bytes[i];
            }
        }
    }

    // --- Peripheral read dispatch ----------------------------------------

    fn peripheral_read32(&mut self, addr: u32) -> u32 {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        // RESETS Bus-level guard (HLD V7 §5.3). Reset-gated peripherals
        // return 0 without the peripheral module ever seeing the read.
        if peripheral_dispatch::is_held_in_reset(self, base) {
            return 0;
        }
        match base {
            SYSINFO_BASE => self.sysinfo_read(offset),
            CLOCKS_BASE => self.clocks_regs.read32(offset),
            RESETS_BASE => self.resets.read32(offset),
            XOSC_BASE => self.xosc_regs.read32(offset),
            PLL_SYS_BASE => pll_read_with_lock(
                &self.pll_sys_regs,
                offset,
                self.pll_sys_lock_at_cycle,
                self.master_cycle,
            ),
            PLL_USB_BASE => pll_read_with_lock(
                &self.pll_usb_regs,
                offset,
                self.pll_usb_lock_at_cycle,
                self.master_cycle,
            ),
            ROSC_BASE => self.rosc_regs.read32(offset),
            IO_BANK0_BASE => self.io_bank0.read32(offset),
            PADS_BANK0_BASE => self.pads_bank0.read32(offset),
            PIO0_BASE => pio_read_rp2040(&mut self.pio[0], offset),
            PIO1_BASE => pio_read_rp2040(&mut self.pio[1], offset),
            DMA_BASE => self.dma.read32(offset),
            TIMER_BASE => self
                .timer
                .read32(offset, self.master_cycle, self.clock_tree.sys_clk_hz),
            WATCHDOG_BASE => self.watchdog_tick.read32(offset),
            UART0_BASE => self.uart0.read32(offset),
            UART1_BASE => self.uart1.read32(offset),
            SPI0_BASE => self.spi0.read32(offset),
            SPI1_BASE => self.spi1.read32(offset),
            I2C0_BASE => self.i2c0.read32(offset),
            I2C1_BASE => self.i2c1.read32(offset),
            ADC_BASE => self.adc.read32(offset),
            PWM_BASE => self.pwm.read32(offset),
            _ => *self.peripheral_regs.get(&canonical).unwrap_or(&0),
        }
    }

    fn peripheral_write32(&mut self, addr: u32, val: u32, alias: u32) {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        // RESETS Bus-level guard (HLD V7 §5.3). Reset-gated peripherals
        // drop the write without the peripheral module ever seeing it.
        if peripheral_dispatch::is_held_in_reset(self, base) {
            return;
        }
        match base {
            SYSINFO_BASE => {} // read-only
            CLOCKS_BASE => {
                if self.clocks_regs.write32(offset, val, alias) {
                    self.recompute_clock_tree();
                }
            }
            RESETS_BASE => self.resets.write32(offset, val, alias),
            XOSC_BASE => self.xosc_regs.write32(offset, val, alias),
            PLL_SYS_BASE => {
                let old_regs = self.pll_sys_regs;
                if clocks::pll_write(&mut self.pll_sys_regs, offset, val, alias) {
                    self.pll_sys_lock_at_cycle = pll_should_arm_lock(
                        &old_regs,
                        &self.pll_sys_regs,
                        self.pll_sys_lock_at_cycle,
                        self.master_cycle,
                    );
                    self.recompute_clock_tree();
                }
            }
            PLL_USB_BASE => {
                let old_regs = self.pll_usb_regs;
                if clocks::pll_write(&mut self.pll_usb_regs, offset, val, alias) {
                    self.pll_usb_lock_at_cycle = pll_should_arm_lock(
                        &old_regs,
                        &self.pll_usb_regs,
                        self.pll_usb_lock_at_cycle,
                        self.master_cycle,
                    );
                    self.recompute_clock_tree();
                }
            }
            ROSC_BASE => self.rosc_regs.write32(offset, val, alias),
            IO_BANK0_BASE => self.io_bank0.write32(offset, val, alias),
            IO_QSPI_BASE => {
                let old = *self.peripheral_regs.get(&canonical).unwrap_or(&0);
                let new = match alias {
                    0 => val,
                    1 => old ^ val,
                    2 => old | val,
                    3 => old & !val,
                    _ => val,
                };
                if offset == IO_QSPI_SS_CTRL {
                    let old_out = old & IO_QSPI_OUTOVER_MASK;
                    let new_out = new & IO_QSPI_OUTOVER_MASK;
                    // ROM flash_cs_force() drives active-low CS low to
                    // begin a command and high to commit it. End the
                    // parser transaction on the rising edge; relying only
                    // on SSIENR misses the SDK/bootrom path entirely.
                    if old_out == IO_QSPI_OUTOVER_LOW && new_out == IO_QSPI_OUTOVER_HIGH {
                        self.ssi_flash.end_transaction();
                        self.apply_ssi_flash_mutations();
                    }
                }
                self.peripheral_regs.insert(canonical, new);
            }
            PADS_BANK0_BASE => self.pads_bank0.write32(offset, val, alias),
            PADS_QSPI_BASE => {
                let old = *self.peripheral_regs.get(&canonical).unwrap_or(&0);
                let new = match alias {
                    0 => val,
                    1 => old ^ val,
                    2 => old | val,
                    3 => old & !val,
                    _ => val,
                };
                self.peripheral_regs.insert(canonical, new);
            }
            PIO0_BASE => self.pio[0].write32(pio_rp2040_to_internal(offset), val, alias),
            PIO1_BASE => self.pio[1].write32(pio_rp2040_to_internal(offset), val, alias),
            DMA_BASE => self.dma.write32(offset, val, alias),
            TIMER_BASE => {
                let sys_hz = self.clock_tree.sys_clk_hz;
                let mc = self.master_cycle;
                self.timer.write32(offset, val, alias, mc, sys_hz);
            }
            WATCHDOG_BASE => {
                if self.watchdog_tick.write32(offset, val, alias) {
                    self.watchdog_reset_requested = true;
                }
            }
            UART0_BASE => {
                self.uart0.set_wire_cycle(self.master_cycle);
                self.uart0
                    .write32(offset, val, alias, &mut self.irq_pending);
            }
            UART1_BASE => {
                self.uart1.set_wire_cycle(self.master_cycle);
                self.uart1
                    .write32(offset, val, alias, &mut self.irq_pending);
            }
            SPI0_BASE => self.spi0.write32(offset, val, alias, &mut self.irq_pending),
            SPI1_BASE => self.spi1.write32(offset, val, alias, &mut self.irq_pending),
            I2C0_BASE => self.i2c0.write32(offset, val, alias, &mut self.irq_pending),
            I2C1_BASE => self.i2c1.write32(offset, val, alias, &mut self.irq_pending),
            ADC_BASE => self.adc.write32(offset, val, alias, &mut self.irq_pending),
            PWM_BASE => self.pwm.write32(offset, val, alias, &mut self.irq_pending),
            _ => {
                // Catch-all: store with alias semantics so firmware round-trips.
                let old = *self.peripheral_regs.get(&canonical).unwrap_or(&0);
                let new = match alias {
                    0 => val,
                    1 => old ^ val,
                    2 => old | val,
                    3 => old & !val,
                    _ => val,
                };
                self.peripheral_regs.insert(canonical, new);
            }
        }
    }

    fn sysinfo_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => 0x0000_0001, // CHIP_ID: RP2040 manufacturer (placeholder)
            0x004 => 0x0000_0000, // PLATFORM
            _ => 0,
        }
    }

    // --- XIP_CTRL + SSI stubs --------------------------------------------

    fn xip_ctrl_read(&self, offset: u32) -> u32 {
        // XIP_CTRL_CTRL (offset 0x00) reports EN=1 so the bootrom's check
        // for "XIP cache enabled" succeeds immediately.
        match offset {
            0x00 => *self.xip_ctrl_regs.get(&0).unwrap_or(&1),
            _ => *self.xip_ctrl_regs.get(&offset).unwrap_or(&0),
        }
    }

    fn xip_ctrl_write(&mut self, offset: u32, val: u32) {
        self.xip_ctrl_regs.insert(offset, val);
    }

    fn ssi_read(&mut self, offset: u32) -> u32 {
        match offset {
            // SR: transfers complete as soon as they are issued, so the
            // TX side always reports empty and not-full, and BUSY stays
            // clear. RFNE follows the modelled flash's reply queue —
            // without it, `flash_do_cmd` waits for a byte that never
            // arrives.
            SSI_SR => {
                let mut status = SSI_SR_TFNF | SSI_SR_TFE;
                if self.ssi_flash.has_rx() {
                    status |= SSI_SR_RFNE;
                }
                status
            }
            // FIFO levels. The bootrom transfer loop reads these rather
            // than the status flags to decide how much it may push and
            // how much has come back. Transmit completes as it is
            // issued, so the transmit level is always zero.
            SSI_TXFLR => 0,
            SSI_RXFLR => self.ssi_flash.rx_len(),
            SSI_DR0 => self.ssi_flash.pop_rx() as u32,
            _ => *self.ssi_regs.get(&offset).unwrap_or(&0),
        }
    }

    fn ssi_write(&mut self, offset: u32, val: u32) {
        match offset {
            // Disabling the controller ends whatever transaction was in
            // flight. The boot helpers bracket every command with an
            // SSIENR toggle, which is what delimits one command from the
            // next here.
            SSI_SSIENR => {
                if val & 1 == 0 {
                    self.ssi_flash.end_transaction();
                    self.apply_ssi_flash_mutations();
                }
                self.ssi_regs.insert(offset, val);
            }
            SSI_DR0 => {
                self.ssi_flash.push_tx(val as u8);
                // A transaction normally commits at SSIENR=0.  Applying
                // here as well keeps the model correct for firmware that
                // leaves the controller enabled between commands.
                self.apply_ssi_flash_mutations();
            }
            _ => {
                self.ssi_regs.insert(offset, val);
            }
        }
    }

    /// Apply completed SSI NOR operations to the executable XIP image.
    /// The SSI parser deliberately emits operations instead of borrowing
    /// `Memory`, so the bus can update XIP and invalidate decode caches at
    /// one well-defined boundary.
    fn apply_ssi_flash_mutations(&mut self) {
        use ssi_flash::FlashMutation;

        let mutations = self.ssi_flash.take_mutations();
        for mutation in mutations {
            match mutation {
                FlashMutation::Erase { offset, len } => {
                    let start = offset as usize;
                    let erase_len = if len == 0 {
                        self.memory.flash_size().saturating_sub(start)
                    } else {
                        len as usize
                    };
                    if !self.memory.xip_erase(start, erase_len) {
                        self.ssi_flash.errors.push(format!(
                            "erase_out_of_range:offset=0x{offset:08x}:len=0x{len:08x}"
                        ));
                    } else {
                        self.pending_invalidation_regions |= invalidation_regions::XIP;
                    }
                }
                FlashMutation::Program { offset, data } => {
                    let start = offset as usize;
                    let Some(end) = start.checked_add(data.len()) else {
                        self.ssi_flash.errors.push("program_range_overflow".into());
                        continue;
                    };
                    if end > self.memory.flash_size() {
                        self.ssi_flash.errors.push(format!(
                            "program_out_of_range:offset=0x{offset:08x}:len=0x{:x}",
                            data.len()
                        ));
                        continue;
                    }
                    for (index, requested) in data.into_iter().enumerate() {
                        let address = start + index;
                        let current = self.memory.xip_byte(address).unwrap_or(0);
                        // NOR programming cannot change a zero back to one.
                        // Do not silently accept a firmware bug: retain the
                        // physical AND result but record a fail-closed error.
                        if (requested & !current) != 0 {
                            self.ssi_flash.errors.push(format!(
                                "program_attempted_0_to_1:offset=0x{address:08x}:old=0x{current:02x}:requested=0x{requested:02x}"
                            ));
                        }
                        let _ = self.memory.xip_program_byte(address, requested);
                    }
                    self.pending_invalidation_regions |= invalidation_regions::XIP;
                }
            }
        }
    }

    // ----------------------------------------------------------------
    // Narrow-access dispatch for byte/halfword-significant peripheral
    // registers (UART_DR, SSPDR, IC_DATA_CMD). Returns `Some(val)` /
    // `true` when handled, `None` / `false` to signal the caller it
    // should fall back to the word-RMW path. This keeps UART/SPI/I2C
    // side-effect registers from suffering spurious FIFO pops on a
    // byte write that would otherwise execute `read32` → splice →
    // `write32` and double-side-effect the register.
    // ----------------------------------------------------------------

    /// Per HLD V7 §5.4, only peripherals with side-effect narrow
    /// registers need narrow dispatch. Returns `true` if the peripheral
    /// at `base` has one (UART DR, SPI DR, I2C DATA_CMD); callers
    /// dispatch to `narrow_peripheral_read*` / `narrow_peripheral_write*`
    /// when this is set, otherwise RMW-through-word is used.
    #[inline]
    fn peripheral_has_narrow_register(base: u32, offset: u32) -> bool {
        match base {
            UART0_BASE | UART1_BASE => offset == crate::peripherals::uart::UARTDR,
            SPI0_BASE | SPI1_BASE => offset == crate::peripherals::spi::SSPDR,
            I2C0_BASE | I2C1_BASE => offset == crate::peripherals::i2c::IC_DATA_CMD,
            // ADC FIFO pops a sample on any-width read; a byte / halfword
            // read must hit the narrow path so a spurious word32 read-
            // modify-write doesn't double-pop the FIFO. Datasheet §4.9.6
            // notes firmware may configure FCS.SHIFT to right-justify the
            // 12-bit sample to an 8-bit halfword read.
            ADC_BASE => offset == crate::peripherals::adc::FIFO,
            _ => false,
        }
    }

    /// Byte-read a peripheral register with side-effect semantics —
    /// UART_DR / SSPDR / IC_DATA_CMD / ADC FIFO. Caller guarantees
    /// `base + offset` has been checked by
    /// `peripheral_has_narrow_register`.
    fn narrow_peripheral_read8(&mut self, base: u32, offset: u32) -> u8 {
        // RESETS is Bus-level — held peripherals return 0.
        if peripheral_dispatch::is_held_in_reset(self, base) {
            return 0;
        }
        match base {
            UART0_BASE => self.uart0.read8(offset),
            UART1_BASE => self.uart1.read8(offset),
            SPI0_BASE => self.spi0.read8(offset),
            SPI1_BASE => self.spi1.read8(offset),
            I2C0_BASE => self.i2c0.read8(offset),
            I2C1_BASE => self.i2c1.read8(offset),
            ADC_BASE => self.adc.read16(offset) as u8,
            _ => 0,
        }
    }

    /// Halfword-read. Same constraints as `narrow_peripheral_read8`.
    fn narrow_peripheral_read16(&mut self, base: u32, offset: u32) -> u16 {
        if peripheral_dispatch::is_held_in_reset(self, base) {
            return 0;
        }
        match base {
            SPI0_BASE => self.spi0.read16(offset),
            SPI1_BASE => self.spi1.read16(offset),
            // UART DR and I2C DATA_CMD don't carry halfword semantics
            // — a 16-bit read collapses to the low byte with zero in
            // the high byte. Use the narrow byte dispatch and
            // zero-extend.
            UART0_BASE => self.uart0.read8(offset) as u16,
            UART1_BASE => self.uart1.read8(offset) as u16,
            I2C0_BASE => self.i2c0.read32(offset) as u16,
            I2C1_BASE => self.i2c1.read32(offset) as u16,
            ADC_BASE => self.adc.read16(offset),
            _ => 0,
        }
    }

    /// Byte-write.
    fn narrow_peripheral_write8(&mut self, base: u32, offset: u32, val: u8) {
        if peripheral_dispatch::is_held_in_reset(self, base) {
            return;
        }
        match base {
            UART0_BASE => {
                self.uart0.set_wire_cycle(self.master_cycle);
                self.uart0.write8(offset, val, &mut self.irq_pending);
            }
            UART1_BASE => {
                self.uart1.set_wire_cycle(self.master_cycle);
                self.uart1.write8(offset, val, &mut self.irq_pending);
            }
            SPI0_BASE => self.spi0.write8(offset, val, &mut self.irq_pending),
            SPI1_BASE => self.spi1.write8(offset, val, &mut self.irq_pending),
            I2C0_BASE => self.i2c0.write8(offset, val, &mut self.irq_pending),
            I2C1_BASE => self.i2c1.write8(offset, val, &mut self.irq_pending),
            // ADC FIFO is read-only — narrow writes swallowed.
            ADC_BASE => {}
            _ => {}
        }
    }

    /// Halfword-write.
    fn narrow_peripheral_write16(&mut self, base: u32, offset: u32, val: u16) {
        if peripheral_dispatch::is_held_in_reset(self, base) {
            return;
        }
        match base {
            SPI0_BASE => self.spi0.write16(offset, val, &mut self.irq_pending),
            SPI1_BASE => self.spi1.write16(offset, val, &mut self.irq_pending),
            UART0_BASE => {
                self.uart0.set_wire_cycle(self.master_cycle);
                self.uart0.write8(offset, val as u8, &mut self.irq_pending);
            }
            UART1_BASE => {
                self.uart1.set_wire_cycle(self.master_cycle);
                self.uart1.write8(offset, val as u8, &mut self.irq_pending);
            }
            I2C0_BASE => self
                .i2c0
                .write32(offset, val as u32, 0, &mut self.irq_pending),
            I2C1_BASE => self
                .i2c1
                .write32(offset, val as u32, 0, &mut self.irq_pending),
            // ADC FIFO is read-only — narrow writes swallowed.
            ADC_BASE => {}
            _ => {}
        }
    }

    // ======================================================================
    // Read / write entry points
    // ======================================================================

    pub fn read8(&mut self, addr: u32) -> u8 {
        #[cfg(feature = "event-horizon-profiler")]
        self.note_running_cpu_access(addr, true);
        let region = addr >> 28;
        self.last_access_cycles = Self::read_latency(region);
        let val = match region {
            0x0 if (addr & 0x0FFF_FFFF) < ROM_SIZE as u32 => {
                self.memory.rom_read8(addr & 0x0FFF_FFFF)
            }
            0x1 => self.region1_read(addr, 1) as u8,
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if (off as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    self.memory.sram_read8(off)
                } else {
                    self.set_bus_fault(addr);
                    0
                }
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let offset = canonical & 0x0000_0FFF;
                if Self::peripheral_has_narrow_register(base, offset & !3) {
                    self.narrow_peripheral_read8(base, offset & !3)
                } else {
                    let word = self.peripheral_read32(addr & !3);
                    word.to_le_bytes()[(addr & 3) as usize]
                }
            }
            0xD => {
                let word = self.sio_read32(addr & !3);
                word.to_le_bytes()[(addr & 3) as usize]
            }
            0xE => {
                let w32 = addr & !3;
                let word = if let Some(v) = self.nvic_mmio_read32(w32) {
                    v
                } else if let Some(v) = self.systick_mmio_read32(w32) {
                    v
                } else {
                    self.ppb[self.active_core].read32(w32)
                };
                word.to_le_bytes()[(addr & 3) as usize]
            }
            _ => {
                self.set_bus_fault(addr);
                0
            }
        };
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('R', 1, addr, val as u32);
        }
        val
    }

    pub fn read16(&mut self, addr: u32) -> u16 {
        #[cfg(feature = "event-horizon-profiler")]
        self.note_running_cpu_access(addr, true);
        let region = addr >> 28;
        self.last_access_cycles = Self::read_latency(region);
        let val = match region {
            0x0 if (addr & 0x0FFF_FFFF) + 1 < ROM_SIZE as u32 => {
                self.memory.rom_read16(addr & 0x0FFF_FFFF)
            }
            0x1 => self.region1_read(addr, 2) as u16,
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 1) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    self.memory.sram_read16(off)
                } else {
                    self.set_bus_fault(addr);
                    0
                }
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let offset = canonical & 0x0000_0FFF;
                if Self::peripheral_has_narrow_register(base, offset & !3) {
                    self.narrow_peripheral_read16(base, offset & !3)
                } else {
                    let word = self.peripheral_read32(addr & !3);
                    let half = ((addr >> 1) & 1) as usize;
                    [word as u16, (word >> 16) as u16][half]
                }
            }
            0xD => {
                let word = self.sio_read32(addr & !3);
                let half = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half]
            }
            0xE => {
                let w32 = addr & !3;
                let word = if let Some(v) = self.nvic_mmio_read32(w32) {
                    v
                } else if let Some(v) = self.systick_mmio_read32(w32) {
                    v
                } else {
                    self.ppb[self.active_core].read32(w32)
                };
                let half = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half]
            }
            _ => {
                self.set_bus_fault(addr);
                0
            }
        };
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('R', 2, addr, val as u32);
        }
        val
    }

    pub fn read32(&mut self, addr: u32) -> u32 {
        #[cfg(feature = "event-horizon-profiler")]
        self.note_running_cpu_access(addr, true);
        let region = addr >> 28;
        self.last_access_cycles = Self::read_latency(region);
        let val = match region {
            0x0 if (addr & 0x0FFF_FFFF) + 3 < ROM_SIZE as u32 => {
                self.memory.rom_read32(addr & 0x0FFF_FFFF)
            }
            0x1 => self.region1_read(addr, 4),
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 3) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    self.memory.sram_read32(off)
                } else {
                    self.set_bus_fault(addr);
                    0
                }
            }
            0x4 | 0x5 => self.peripheral_read32(addr),
            0xD => self.sio_read32(addr),
            0xE => {
                if let Some(w) = self.nvic_mmio_read32(addr) {
                    w
                } else if let Some(w) = self.systick_mmio_read32(addr) {
                    w
                } else {
                    self.ppb[self.active_core].read32(addr)
                }
            }
            _ => {
                self.set_bus_fault(addr);
                0
            }
        };
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('R', 4, addr, val);
        }
        val
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        #[cfg(feature = "event-horizon-profiler")]
        self.note_running_cpu_access(addr, false);
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('W', 1, addr, val as u32);
        }
        let region = addr >> 28;
        self.last_access_cycles = Self::write_latency(region);
        match region {
            0x1 if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&addr) => {
                self.xip_sram_write(addr, val as u32, 1);
                self.invalidate_pc_range(addr, 1);
            }
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if (off as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    // All four SRAM alias windows (0x20/0x21/0x22/0x23)
                    // map to the same backing storage — RP2040 datasheet
                    // §2.1.2 calls out alias bits [25:24] as bank-striping
                    // flavours for DMA, not peripheral XOR/SET/CLR.
                    self.memory.sram_write8(off, val);
                    self.invalidate_pc_range(addr, 1);
                } else {
                    self.set_bus_fault(addr);
                }
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                if base == PIO0_BASE || base == PIO1_BASE {
                    let pio_offset = canonical & 0x0000_0FFF;
                    // TXF0-3: byte-replicate (ARM AHB-Lite HSIZE=0
                    // replication) and push to the TX FIFO. All other
                    // PIO registers: 32-bit access only — byte writes
                    // would trigger spurious RXF pops via the RMW read,
                    // so silently ignore.
                    if (0x010..=0x01C).contains(&pio_offset) {
                        let replicated = (val as u32) * 0x0101_0101;
                        let idx = if base == PIO0_BASE { 0 } else { 1 };
                        self.pio[idx].write32(pio_offset, replicated, 0);
                    }
                    return;
                }
                let offset = canonical & 0x0000_0FFF;
                let word_offset = offset & !3;
                if Self::peripheral_has_narrow_register(base, word_offset) {
                    self.narrow_peripheral_write8(base, word_offset, val);
                } else {
                    let alias = (addr >> 12) & 3;
                    // Byte-level RMW into the word, preserving alias semantics.
                    let word_addr = canonical & !3;
                    let byte_idx = (canonical & 3) as usize;
                    let old = self.peripheral_read32(word_addr);
                    let mut bytes = old.to_le_bytes();
                    bytes[byte_idx] = val;
                    let new_word = u32::from_le_bytes(bytes);
                    // For an alias access, convert the byte to a positioned
                    // word and defer alias math to the peripheral layer.
                    if alias == 0 {
                        self.peripheral_write32(word_addr, new_word, 0);
                    } else {
                        let shifted = (val as u32) << (byte_idx * 8);
                        self.peripheral_write32(word_addr, shifted, alias);
                    }
                }
            }
            0xD => {
                let word_addr = addr & !3;
                let byte_idx = (addr & 3) as usize;
                let old = self.sio_read32(word_addr);
                let mut bytes = old.to_le_bytes();
                bytes[byte_idx] = val;
                self.sio_write32(word_addr, u32::from_le_bytes(bytes));
            }
            0xE => {
                let word_addr = addr & !3;
                let byte_idx = (addr & 3) as usize;
                let old = if let Some(v) = self.nvic_mmio_read32(word_addr) {
                    v
                } else if let Some(v) = self.systick_mmio_read32(word_addr) {
                    v
                } else {
                    self.ppb[self.active_core].read32(word_addr)
                };
                let mut bytes = old.to_le_bytes();
                bytes[byte_idx] = val;
                let new_word = u32::from_le_bytes(bytes);
                // NVIC and SysTick both consume the write fully when they
                // claim it; PPB takes the rest.
                if !self.nvic_mmio_write32(word_addr, new_word)
                    && !self.systick_mmio_write32(word_addr, new_word)
                {
                    self.ppb[self.active_core].write32(word_addr, new_word);
                }
            }
            0x0 | 0x1 => {} // ROM / XIP flash — silently ignored at any width
            _ => {
                // Unmapped at any width sets the sticky bus-fault flag so
                // step() can escalate to HardFault.
                self.set_bus_fault(addr);
            }
        }
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        #[cfg(feature = "event-horizon-profiler")]
        self.note_running_cpu_access(addr, false);
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('W', 2, addr, val as u32);
        }
        let region = addr >> 28;
        self.last_access_cycles = Self::write_latency(region);
        match region {
            0x1 if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&addr) => {
                self.xip_sram_write(addr, val as u32, 2);
                self.invalidate_pc_range(addr, 2);
            }
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 1) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    // All four SRAM alias windows map to the same storage
                    // (RP2040 datasheet §2.1.2).
                    self.memory.sram_write16(off, val);
                    self.invalidate_pc_range(addr, 2);
                } else {
                    self.set_bus_fault(addr);
                }
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                if base == PIO0_BASE || base == PIO1_BASE {
                    let pio_offset = canonical & 0x0000_0FFF;
                    // TXF0-3: halfword-replicate (ARM AHB-Lite HSIZE=1
                    // replication) and push to the TX FIFO. All other
                    // PIO registers: 32-bit access only.
                    if (0x010..=0x01C).contains(&pio_offset) {
                        let replicated = (val as u32) | ((val as u32) << 16);
                        let idx = if base == PIO0_BASE { 0 } else { 1 };
                        self.pio[idx].write32(pio_offset, replicated, 0);
                    }
                    return;
                }
                let offset = canonical & 0x0000_0FFF;
                let word_offset = offset & !3;
                if Self::peripheral_has_narrow_register(base, word_offset) {
                    self.narrow_peripheral_write16(base, word_offset, val);
                } else {
                    let alias = (addr >> 12) & 3;
                    let word_addr = canonical & !3;
                    let half_idx = ((canonical >> 1) & 1) as usize;
                    let old = self.peripheral_read32(word_addr);
                    let mut halves: [u16; 2] = [old as u16, (old >> 16) as u16];
                    halves[half_idx] = val;
                    let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                    if alias == 0 {
                        self.peripheral_write32(word_addr, new_word, 0);
                    } else {
                        let shifted = (val as u32) << (half_idx * 16);
                        self.peripheral_write32(word_addr, shifted, alias);
                    }
                }
            }
            0xD => {
                let word_addr = addr & !3;
                let half_idx = ((addr >> 1) & 1) as usize;
                let old = self.sio_read32(word_addr);
                let mut halves: [u16; 2] = [old as u16, (old >> 16) as u16];
                halves[half_idx] = val;
                let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                self.sio_write32(word_addr, new_word);
            }
            0xE => {
                let word_addr = addr & !3;
                let half_idx = ((addr >> 1) & 1) as usize;
                let old = if let Some(v) = self.nvic_mmio_read32(word_addr) {
                    v
                } else if let Some(v) = self.systick_mmio_read32(word_addr) {
                    v
                } else {
                    self.ppb[self.active_core].read32(word_addr)
                };
                let mut halves: [u16; 2] = [old as u16, (old >> 16) as u16];
                halves[half_idx] = val;
                let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                // NVIC and SysTick both consume the write fully when they
                // claim it; PPB takes the rest.
                if !self.nvic_mmio_write32(word_addr, new_word)
                    && !self.systick_mmio_write32(word_addr, new_word)
                {
                    self.ppb[self.active_core].write32(word_addr, new_word);
                }
            }
            0x0 | 0x1 => {} // ROM / XIP flash — silently ignored at any width
            _ => {
                self.set_bus_fault(addr);
            }
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        #[cfg(feature = "event-horizon-profiler")]
        self.note_running_cpu_access(addr, false);
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('W', 4, addr, val);
        }
        let region = addr >> 28;
        self.last_access_cycles = Self::write_latency(region);
        match region {
            0x1 if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&addr) => {
                self.xip_sram_write(addr, val, 4);
                self.invalidate_pc_range(addr, 4);
            }
            0x1 => {
                // Region 0x1 at XIP_CTRL (0x1400_0000) or SSI (0x1800_0000).
                let base = addr & 0xFFFF_F000;
                let offset = addr & 0x0FFF;
                if base == XIP_CTRL_BASE {
                    self.xip_ctrl_write(offset, val);
                } else if base == SSI_BASE {
                    self.ssi_write(offset, val);
                }
            }
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 3) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    // All four SRAM alias windows map to the same storage
                    // (RP2040 datasheet §2.1.2).
                    self.memory.sram_write32(off, val);
                    self.invalidate_pc_range(addr, 4);
                } else {
                    self.set_bus_fault(addr);
                }
            }
            0x4 | 0x5 => {
                let alias = (addr >> 12) & 3;
                self.peripheral_write32(addr, val, alias);
            }
            0xD => self.sio_write32(addr, val),
            0xE => {
                // NVIC and SysTick both consume the write fully when they
                // claim it; PPB takes the rest.
                if !self.nvic_mmio_write32(addr, val) && !self.systick_mmio_write32(addr, val) {
                    self.ppb[self.active_core].write32(addr, val);
                }
            }
            0x0 => {} // ROM — silently ignored at any width
            _ => {
                self.set_bus_fault(addr);
            }
        }
    }

    // --- Region 0x1 read dispatch (XIP flash / XIP SRAM / XIP_CTRL / SSI)

    fn region1_read(&mut self, addr: u32, width: usize) -> u32 {
        if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&addr) {
            return self.xip_sram_read(addr, width);
        }
        let base = addr & 0xFFFF_F000;
        let offset = addr & 0x0FFF;
        if base == XIP_CTRL_BASE {
            return self.xip_ctrl_read(offset);
        }
        if base == SSI_BASE {
            return self.ssi_read(offset);
        }
        // XIP flash window (0x10/0x11/0x12/0x13, each a 2 MB mirror).
        // PicoGUS Integration HLD (Stage 1): flash is a plain mapped
        // window — no wait states, no cache, no fault before load.
        if let Some(flash_off) = xip_flash_offset(addr) {
            return match width {
                1 => self.memory.xip_read8(flash_off) as u32,
                2 => self.memory.xip_read16(flash_off) as u32,
                4 => self.memory.xip_read32(flash_off),
                _ => 0,
            };
        }
        0
    }

    // --- PPB + NVIC read/write dispatch ----------------------------------
    //
    // The NVIC lives in the System Control Space at `0xE000_E100..=
    // 0xE000_E41F`. These registers are banked per-core on M0+, so
    // `nvics[active_core]` is the one the currently-executing CPU sees.
    // Other 0xE000_Exxx addresses fall through to the PPB.

    /// Intercept NVIC MMIO before the PPB sees it. Returns `Some(word)`
    /// when `addr` lies inside the NVIC ISER0 / ICER0 / ISPR0 / ICPR0 /
    /// IPR0..7 range, `None` otherwise so the caller can fall through
    /// to the PPB dispatch.
    fn nvic_mmio_read32(&self, addr: u32) -> Option<u32> {
        let low = addr & 0xFFFF;
        let n = &self.nvics[self.active_core];
        match low {
            // NVIC_ISER0 (0xE100) and NVIC_ICER0 (0xE180) both READ the
            // enable mask (ARMv6-M ARM §B3.4.4 / §B3.4.5).
            0xE100 | 0xE180 => Some(n.enabled),
            // NVIC_ISPR0 (0xE200) / NVIC_ICPR0 (0xE280) both READ the
            // pending mask.
            0xE200 | 0xE280 => Some(n.pending),
            // NVIC_IPR0..7 at 0xE400 + 4N. Each word holds 4 × 8-bit
            // priority bytes for IRQs [N*4..N*4+4].
            0xE400..=0xE41F => {
                let word_idx = ((low - 0xE400) >> 2) as usize;
                let base_irq = word_idx * 4;
                let mut w = 0u32;
                for lane in 0..4 {
                    let irq = base_irq + lane;
                    if irq < 32 {
                        w |= (n.priority[irq] as u32) << (lane * 8);
                    }
                }
                Some(w)
            }
            _ => None,
        }
    }

    /// Intercept NVIC MMIO writes. Returns `true` when handled. All
    /// four register families are per-core.
    fn nvic_mmio_write32(&mut self, addr: u32, val: u32) -> bool {
        let low = addr & 0xFFFF;
        let n = &mut self.nvics[self.active_core];
        match low {
            // NVIC_ISER0: write-1-to-SET the enable bit.
            0xE100 => {
                n.enabled |= val & crate::irq::IRQ_LINE_MASK;
                true
            }
            // NVIC_ICER0: write-1-to-CLEAR the enable bit.
            0xE180 => {
                n.enabled &= !(val & crate::irq::IRQ_LINE_MASK);
                true
            }
            // NVIC_ISPR0: write-1-to-SET the pending bit.
            0xE200 => {
                n.pending |= val & crate::irq::IRQ_LINE_MASK;
                true
            }
            // NVIC_ICPR0: write-1-to-CLEAR the pending bit.
            0xE280 => {
                n.pending &= !(val & crate::irq::IRQ_LINE_MASK);
                true
            }
            // NVIC_IPR0..7: 4×u8 priority bytes, each masked to the
            // implemented bits [7:6] (M0+ supports 4 priority levels).
            0xE400..=0xE41F => {
                let word_idx = ((low - 0xE400) >> 2) as usize;
                let base_irq = word_idx * 4;
                for lane in 0..4 {
                    let irq = base_irq + lane;
                    if irq < 32 {
                        let byte = ((val >> (lane * 8)) & 0xFF) as u8;
                        n.priority[irq] = byte & crate::core::nvic::PRIORITY_MASK;
                    }
                }
                true
            }
            _ => false,
        }
    }

    // --- SysTick MMIO dispatch -------------------------------------------
    //
    // HLD V5 §5.2: SysTick lives at `0xE000_E010..0xE000_E01F`. Per-core
    // banked register set; reads/writes target `systicks[active_core]`.
    // Note `systick_mmio_read32` is `&mut self` — `SYST_CSR` reads clear
    // `COUNTFLAG` per ARMv6-M ARM §B3.3.2.

    /// Intercept SysTick MMIO before the PPB sees it. Returns
    /// `Some(word)` when `addr` lies in the SysTick range, `None`
    /// otherwise so the caller can fall through.
    fn systick_mmio_read32(&mut self, addr: u32) -> Option<u32> {
        match addr & 0xFFFF {
            0xE010..=0xE01F => Some(self.systicks[self.active_core].read32(addr)),
            _ => None,
        }
    }

    /// Intercept SysTick MMIO writes. Returns `true` when handled.
    fn systick_mmio_write32(&mut self, addr: u32, val: u32) -> bool {
        match addr & 0xFFFF {
            0xE010..=0xE01F => {
                self.systicks[self.active_core].write32(addr, val);
                true
            }
            _ => false,
        }
    }

    // --- SIO read/write dispatch -----------------------------------------
    //
    // GPIO_IN (0x004) is owned by Bus so the SIO crate has no direct
    // dependency on PIO (Phase 5.B lifts this out). All other offsets
    // delegate to `Sio`.

    fn sio_read32(&mut self, addr: u32) -> u32 {
        let offset = addr & 0xFFF;
        let value = match offset {
            0x004 => self.gpio_in,
            _ => {
                let core = self.active_core;
                self.sio.read32(offset, core)
            }
        };
        self.refresh_sio_fifo_irqs();
        value
    }

    fn sio_write32(&mut self, addr: u32, val: u32) {
        let offset = addr & 0xFFF;
        let core = self.active_core;
        self.sio.write32(offset, val, core);
        if let Some(receiver) = self.sio.pending_fifo_event.take() {
            self.event_flag[receiver] = true;
        }
        self.refresh_sio_fifo_irqs();
    }

    /// Project the two core-local, level-sensitive SIO FIFO interrupt lines
    /// into their matching NVICs. These lines must not use `irq_pending`,
    /// because that shared-peripheral path broadcasts every bit to both
    /// cores; `SIO_IRQ_PROC0` belongs only to core 0 and `SIO_IRQ_PROC1`
    /// only to core 1.
    pub(crate) fn refresh_sio_fifo_irqs(&mut self) {
        if self.sio.fifo_irq_asserted(0) {
            self.nvics[0].set_pending(IRQ_SIO_IRQ_PROC0 as u8);
        }
        if self.sio.fifo_irq_asserted(1) {
            self.nvics[1].set_pending(IRQ_SIO_IRQ_PROC1 as u8);
        }
    }

    // --- Back-compat accessors for Phase 3 / 4 tests ---------------------
    //
    // The previous Phase 3 stub exposed a `gpio_in` field; keep that
    // interface stable so tests don't need updating.
    #[inline]
    pub fn gpio_in(&self) -> u32 {
        self.gpio_in
    }

    /// Merged SIO + PIO pad-output levels (bit *n* = GPIO *n*), masked
    /// to the RP2040's 30-pin range.
    ///
    /// Same merge `Emulator::update_gpio` performs, minus the off-chip
    /// PSRAM MISO splice and the harness `external_gpio_in_*` override —
    /// i.e. strictly "what the chip is driving onto the pads right now",
    /// which is what an attached slave sees on its side-band pins.
    /// Available on `Bus` (not just `Emulator`) so peripheral ticking can
    /// consult it *before* draining a FIFO.
    #[inline]
    pub fn pad_out_levels(&self) -> u32 {
        let mut out = self.sio.gpio_out & self.sio.gpio_oe;
        for pio in &self.pio {
            let pio_mask = pio.pad_oe;
            out = (out & !pio_mask) | (pio.pad_out & pio_mask);
        }
        out & 0x3FFF_FFFF
    }

    /// Attach an off-chip SPI slave to instance `instance` (0 = SPI0,
    /// 1 = SPI1). Returns the previously attached device, if any, or an
    /// error for an out-of-range instance.
    ///
    /// Board-level crates own the device model; this crate only routes
    /// words and the pad snapshot to it (see
    /// [`crate::peripherals::spi::SpiExternalDevice`]).
    pub fn attach_spi_device(
        &mut self,
        instance: usize,
        device: Box<dyn crate::peripherals::spi::SpiExternalDevice>,
    ) -> Result<Option<Box<dyn crate::peripherals::spi::SpiExternalDevice>>, usize> {
        match instance {
            0 => Ok(self.spi0.attach_device(device)),
            1 => Ok(self.spi1.attach_device(device)),
            other => Err(other),
        }
    }

    /// Read-only view of the PWM block, for reporting which slices
    /// firmware configured.
    /// Attach a device that watches the pads directly.
    pub fn attach_pin_device(&mut self, device: Box<dyn PinWatchingDevice>) {
        self.pin_devices.push(device);
    }

    pub fn pwm(&self) -> &crate::peripherals::pwm::PwmRegs {
        &self.pwm
    }

    /// Attach an off-chip I2C slave to controller `instance` (0 or 1),
    /// returning whatever was attached before. `Err(instance)` for an
    /// out-of-range controller.
    pub fn attach_i2c_device(
        &mut self,
        instance: usize,
        device: Box<dyn crate::peripherals::i2c::I2cExternalDevice>,
    ) -> Result<Option<Box<dyn crate::peripherals::i2c::I2cExternalDevice>>, usize> {
        match instance {
            0 => Ok(self.i2c0.attach_device(device)),
            1 => Ok(self.i2c1.attach_device(device)),
            other => Err(other),
        }
    }

    /// Attach an explicitly configured I2C profile and disable the legacy
    /// synthetic ACK fallback for that controller. Profile builders use
    /// this entry point so unclaimed addresses remain NACKed.
    pub fn attach_i2c_device_exclusive(
        &mut self,
        instance: usize,
        device: Box<dyn crate::peripherals::i2c::I2cExternalDevice>,
    ) -> Result<Option<Box<dyn crate::peripherals::i2c::I2cExternalDevice>>, usize> {
        match instance {
            0 => Ok(self.i2c0.attach_device_exclusive(device)),
            1 => Ok(self.i2c1.attach_device_exclusive(device)),
            other => Err(other),
        }
    }

    /// Mutably borrow the slave attached to I2C `instance`, e.g. to
    /// inject input from a scenario.
    pub fn i2c_device_mut(
        &mut self,
        instance: usize,
    ) -> Option<&mut (dyn crate::peripherals::i2c::I2cExternalDevice + 'static)> {
        match instance {
            0 => self.i2c0.device_mut(),
            1 => self.i2c1.device_mut(),
            _ => None,
        }
    }

    /// True iff instance `instance` has an off-chip slave attached.
    pub fn spi_has_device(&self, instance: usize) -> bool {
        match instance {
            0 => self.spi0.has_device(),
            1 => self.spi1.has_device(),
            _ => false,
        }
    }

    /// Signal SEV to both cores.
    pub fn signal_sev(&mut self) {
        self.event_flag[0] = true;
        self.event_flag[1] = true;
    }

    // --- Fast-path gate helpers (HLD V7 §5.5) ----------------------------

    /// True iff every stateful peripheral is idle — i.e. nothing that
    /// could observably change on a per-cycle tick is currently active.
    ///
    /// **MUST be updated when a new stateful peripheral is added to
    /// `Bus`.** This function is the fast-path gate in
    /// [`crate::Emulator::step`]; forgetting to AND a new peripheral's
    /// `is_idle()` into the result silently keeps the fast path taken
    /// while the peripheral is actually running, losing per-cycle
    /// observability (interrupts, FIFO edges, DMA transfers).
    ///
    /// Phase 1 peripherals with state (`TIMER`, `WATCHDOG_TICK`) are
    /// lazy and therefore always idle at the fast-path check — their
    /// internal effects fire on alarm-match rather than per-cycle. DMA
    /// is a bus master rather than a per-cycle-ticked peripheral, but
    /// is included here so this method is the complete non-PIO
    /// fast-path gate. Phase 3 adds ADC (idle iff no
    /// conversion armed) and PWM (idle iff `EN == 0`).
    #[inline]
    pub fn all_peripherals_idle(&self) -> bool {
        // Explicit acknowledgement of every stateful peripheral field.
        // When a new peripheral field is added to `Bus`, add a
        // reference here; Rust will flag any field rename / removal at
        // compile time. Phase 1 Wave 2: TIMER is a lazy peripheral —
        // alarm firing happens inside `advance_lazy_scheduled`, which
        // itself runs in the fast path. A latched INTR without INTE
        // set (no NVIC routing) still counts as idle because nothing
        // observable happens per-cycle. WATCHDOG_TICK has no tick.
        // Phase 2: UART/SPI/I2C are per-cycle-tickable — their
        // `is_idle()` getters gate the fast path.
        // Phase 3: ADC + PWM added — ADC mid-conversion must keep the
        // fast path closed so the clk_adc accumulator advances at all;
        // PWM with any enabled slice similarly.
        #[cfg(debug_assertions)]
        {
            let _ = (
                &self.watchdog_tick,
                &self.dma,
                &self.timer,
                &self.uart0,
                &self.uart1,
                &self.spi0,
                &self.spi1,
                &self.i2c0,
                &self.i2c1,
                &self.adc,
                &self.pwm,
            );
        }
        self.timer.is_idle()
            && self.uart0.is_idle()
            && self.uart1.is_idle()
            && self.spi0.is_idle()
            && self.spi1.is_idle()
            && self.i2c0.is_idle()
            && self.i2c1.is_idle()
            && self.adc.is_idle()
            && self.pwm.is_idle()
            && self.dma.is_idle()
    }

    /// True iff at least one PIO SM is enabled in either block, or any
    /// IRQ flag is still asserted. Either condition means a per-cycle
    /// PIO step could mutate pin state or the IRQ-flags register — so
    /// the fast-path is not safe (HLD V7 §5.5, "`pio_all_idle`"
    /// semantics).
    #[inline]
    pub fn pio_all_idle(&self) -> bool {
        !self.pio[0].any_sm_enabled()
            && !self.pio[1].any_sm_enabled()
            && self.pio[0].pending_irqs() == 0
            && self.pio[1].pending_irqs() == 0
    }

    /// True iff some off-chip device wired to `update_gpio`'s merge
    /// needs to observe *every* SCK/CS/etc. pad edge, not just the
    /// quantum-end snapshot.
    ///
    /// `Emulator::step_serial`'s slow path uses this to decide whether
    /// the PIO+GPIO merge for the current quantum must run one system
    /// cycle at a time (see that function's docs). Currently PSRAM is
    /// the only such device, but the predicate is named generically so
    /// a future PIO-driven off-chip model (e.g. a second SPI PSRAM, an
    /// I2S codec) can opt in without another call-site change.
    #[inline]
    pub fn has_pin_watching_device(&self) -> bool {
        self.psram.is_some() || !self.pin_devices.is_empty()
    }

    /// Collect the current DREQ (data-request) bitmap for all 64 TREQ
    /// sources + the FORCE sentinel at bit 63.
    ///
    /// Layout (RP2040 datasheet §2.5.3.1 Table 120, pinned in HLD V7
    /// Appendix C):
    /// * bits 0..3   PIO0 TX0..3
    /// * bits 4..7   PIO0 RX0..3
    /// * bits 8..11  PIO1 TX0..3
    /// * bits 12..15 PIO1 RX0..3
    /// * bit  16     SPI0 TX
    /// * bit  17     SPI0 RX
    /// * bit  18     SPI1 TX
    /// * bit  19     SPI1 RX
    /// * bit  20     UART0 TX
    /// * bit  21     UART0 RX
    /// * bit  22     UART1 TX
    /// * bit  23     UART1 RX
    /// * bits 24..31 PWM WRAP0..7 (deferred — one-shot per wrap, V1 zero)
    /// * bit  32     I2C0 TX
    /// * bit  33     I2C0 RX
    /// * bit  34     I2C1 TX
    /// * bit  35     I2C1 RX
    /// * bit  36     ADC FIFO
    /// * bits 37..39 XIP (not modelled)
    /// * bit  63     FORCE (always true — `TREQ_SEL == 0x3F` bypass)
    ///
    /// Consumed by [`crate::dma::Dma::tick`] — snapshot taken before any
    /// bus access so peripheral state changes driven by the current
    /// transfer don't feed back into same-cycle DREQ arbitration.
    pub fn collect_dreqs(&self) -> u64 {
        let mut bits = 0u64;
        // PIO0 / PIO1 — four SM × (TX | RX) per block.
        for sm in 0..4 {
            if self.pio[0].tx_dreq(sm) {
                bits |= 1u64 << sm;
            }
            if self.pio[0].rx_dreq(sm) {
                bits |= 1u64 << (4 + sm);
            }
            if self.pio[1].tx_dreq(sm) {
                bits |= 1u64 << (8 + sm);
            }
            if self.pio[1].rx_dreq(sm) {
                bits |= 1u64 << (12 + sm);
            }
        }
        if self.spi0.tx_dreq() {
            bits |= 1u64 << 16;
        }
        if self.spi0.rx_dreq() {
            bits |= 1u64 << 17;
        }
        if self.spi1.tx_dreq() {
            bits |= 1u64 << 18;
        }
        if self.spi1.rx_dreq() {
            bits |= 1u64 << 19;
        }
        if self.uart0.tx_dreq() {
            bits |= 1u64 << 20;
        }
        if self.uart0.rx_dreq() {
            bits |= 1u64 << 21;
        }
        if self.uart1.tx_dreq() {
            bits |= 1u64 << 22;
        }
        if self.uart1.rx_dreq() {
            bits |= 1u64 << 23;
        }
        // PWM wrap DREQs (bits 24..31) are one-shot-per-wrap and deferred
        // to a later phase; `audio_i2s` uses PIO DREQ, not PWM wrap.
        if self.i2c0.tx_dreq() {
            bits |= 1u64 << 32;
        }
        if self.i2c0.rx_dreq() {
            bits |= 1u64 << 33;
        }
        if self.i2c1.tx_dreq() {
            bits |= 1u64 << 34;
        }
        if self.i2c1.rx_dreq() {
            bits |= 1u64 << 35;
        }
        if self.adc.dreq() {
            bits |= 1u64 << 36;
        }
        // XIP DREQs (bits 37..39) not modelled.
        // FORCE is always 1 — no peripheral produces it.
        bits |= 1u64 << 63;
        bits
    }

    /// Drive the DMA by one cycle. Swaps the DMA out of `self` to avoid
    /// cross-borrows while it issues transfers through the bus, then
    /// restores it and routes any pending IRQs through `irq_pending`.
    ///
    /// Per HLD V7 §5.6 ordering contract: peripherals tick first (to
    /// produce DREQ), then `tick_dma` consumes the snapshot. Call at the
    /// tail of [`Self::tick_peripherals`].
    pub fn tick_dma(&mut self) {
        let mut dma = std::mem::take(&mut self.dma);
        dma.tick(self, 1);
        dma.route_irqs(&mut self.irq_pending);
        self.dma = dma;
    }

    /// Drive DMA for an arbitrary number of sysclks.
    pub fn tick_dma_with_cycles(&mut self, cycles: u32) {
        let mut dma = std::mem::take(&mut self.dma);
        dma.tick(self, cycles);
        dma.route_irqs(&mut self.irq_pending);
        self.dma = dma;
    }

    /// Advance every stateful peripheral by `cycles` system-clock cycles.
    ///
    /// Called from the slow-path branch in [`crate::Emulator::step`]
    /// whenever the fast-path gate opens (PIO active, DMA live, or an
    /// IRQ already pending). TIMER alarms are polled here for lazy-
    /// fire at match. UART/SPI/I2C advance their TX shift registers
    /// by `cycles` sysclks via `tick(cycles, clock_tree, irqs)` and OR
    /// any level-driven IRQs into `irq_pending`.
    ///
    /// Per HLD 2026.04.26 V5 §5.1: chunked once-per-quantum advance
    /// replaces the previous per-cycle interleave. `tick_dma` still
    /// runs once per quantum at the tail to preserve the "peripherals
    /// produce DREQ, then DMA consumes the snapshot" ordering.
    #[inline]
    pub fn tick_peripherals(&mut self, cycles: u32) {
        // TIMER alarms are lazy-fire at match: `poll_alarms` is cheap
        // (four armed-bit checks) and we run it here so firmware
        // stepping in the slow path observes alarm-match IRQs on the
        // same inner cycle as the source condition.
        let nvic_bits = self
            .timer
            .poll_alarms(self.master_cycle, self.clock_tree.sys_clk_hz);
        // TIMER IRQs occupy NVIC lines 0..3.
        self.irq_pending |= nvic_bits & 0xF;
        // UART / SPI / I2C: chunked TX drain + IRQ route.
        self.uart0
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        self.uart1
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        // Off-chip SPI slaves sample their side-band pins (chip-select,
        // command/data, reset, …) *before* the FIFO drain below. The
        // CPU cannot have moved those lines past the queued word yet:
        // pico-sdk's `spi_write_blocking` spins on `SSPSR.BSY`, which
        // stays set while the TX FIFO is non-empty, so control-line
        // changes always land on the far side of a drain. Sampling
        // after the drain would frame each word with the *next*
        // transaction's control lines. Skipped entirely — including the
        // pad merge — when nothing is attached.
        if self.spi0.has_device() || self.spi1.has_device() {
            let pads = self.pad_out_levels();
            self.spi0.observe_pins(pads);
            self.spi1.observe_pins(pads);
        }
        self.spi0
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        self.spi1
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        self.i2c0
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        self.i2c1
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        // External I2C devices consume the same shared virtual-time
        // snapshot as the harness. This is deliberately one call per
        // peripheral window; lazy scheduled advance below is the other,
        // mutually exclusive path.
        self.advance_external_virtual_time();
        // ADC: fixed-point clk_adc accumulator advances via tick.
        self.adc
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        // PWM: per-slice counter advance + wrap-IRQ latch.
        self.pwm
            .tick(cycles, &self.clock_tree, &mut self.irq_pending);
        // DMA ticks LAST per HLD V7 §5.6 ordering contract — peripherals
        // produce DREQ on this cycle, DMA snapshots + consumes. Stays
        // once per quantum; mirrors RP2350.
        self.tick_dma_with_cycles(cycles);
    }

    /// Fast-path lazy-schedule advance (HLD V7 §5.5).
    ///
    /// Called from [`crate::Emulator::step`]'s fast-path branch after
    /// the core(s) have stepped. Bumps the bus's master-cycle cache by
    /// `consumed` and polls all lazy peripherals (TIMER alarms) to
    /// surface IRQs for any events that fell inside the window
    /// `[old_master_cycle, old_master_cycle + consumed]`.
    ///
    /// TIMER is currently the only lazy peripheral; later phases may
    /// add RTC alarm + PWM wrap-on-overflow.
    pub fn advance_lazy_scheduled(&mut self, consumed: u64) {
        self.master_cycle = self.master_cycle.wrapping_add(consumed);
        let nvic_bits = self
            .timer
            .poll_alarms(self.master_cycle, self.clock_tree.sys_clk_hz);
        self.irq_pending |= nvic_bits & 0xF;
        self.advance_external_virtual_time();
    }

    /// Soonest scheduled lazy IRQ deadline (master-cycle space) across
    /// peripherals modelled today. TIMER is the only lazy IRQ source in
    /// V1 (PWM/ADC/etc. tick on `consumed`, not on absolute deadlines),
    /// so this currently returns `timer.next_armed_inte_fire_cycle()`.
    /// Used by the both-cores-blocked clock-advance branch in
    /// `Emulator::step_serial` to pick the next event horizon —
    /// closes tech_debt §1649 for RP2040.
    pub fn next_scheduled_lazy_deadline(&self) -> Option<u64> {
        self.timer.next_armed_inte_fire_cycle()
    }

    /// Read the current pending-IRQ bitmap. Mostly for tests.
    #[cfg(test)]
    #[inline]
    pub(crate) fn irq_pending(&self) -> u32 {
        self.irq_pending
    }

    /// ROSC nominal frequency re-export.
    pub const ROSC_FREQ_HZ_CONST: u32 = ROSC_FREQ_HZ;
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

// ===================================================================
// `CoreBus` implementation — dual-execution HLD V1 Stage 3b.1.
//
// Every method forwards directly to an inherent `Bus` method or field.
// Monomorphization against this impl reproduces the pre-refactor
// codegen on the Serial hot path. Stage 3b.2 will add a
// `impl CoreBus for WorkerBus` in the `threaded/` module once that
// module lands.
// ===================================================================

use crate::core::CoreBus;

impl CoreBus for Bus {
    #[inline(always)]
    fn read8(&mut self, addr: u32) -> u8 {
        Bus::read8(self, addr)
    }
    #[inline(always)]
    fn read16(&mut self, addr: u32) -> u16 {
        Bus::read16(self, addr)
    }
    #[inline(always)]
    fn read32(&mut self, addr: u32) -> u32 {
        Bus::read32(self, addr)
    }

    #[inline(always)]
    fn write8(&mut self, addr: u32, val: u8) {
        Bus::write8(self, addr, val)
    }
    #[inline(always)]
    fn write16(&mut self, addr: u32, val: u16) {
        Bus::write16(self, addr, val)
    }
    #[inline(always)]
    fn write32(&mut self, addr: u32, val: u32) {
        Bus::write32(self, addr, val)
    }

    #[inline(always)]
    fn set_active_pc(&mut self, pc: u32) {
        Bus::set_active_pc(self, pc)
    }

    #[inline(always)]
    fn set_active_pc_for_instruction(&mut self, pc: u32) {
        #[cfg(feature = "diagnostic-pc-compile-out-prototype")]
        {
            let _ = pc;
        }
        #[cfg(not(feature = "diagnostic-pc-compile-out-prototype"))]
        {
            Bus::set_active_pc(self, pc);
        }
    }

    #[inline(always)]
    fn bus_fault(&self) -> bool {
        Bus::bus_fault(self)
    }
    #[inline(always)]
    fn bus_fault_addr(&self) -> u32 {
        Bus::bus_fault_addr(self)
    }
    #[inline(always)]
    fn clear_bus_fault(&mut self) {
        Bus::clear_bus_fault(self)
    }

    #[inline(always)]
    fn ppb(&self, core: usize) -> &ppb::Ppb {
        &self.ppb[core]
    }
    #[inline(always)]
    fn ppb_mut(&mut self, core: usize) -> &mut ppb::Ppb {
        &mut self.ppb[core]
    }

    #[inline(always)]
    fn nvic(&self, core: usize) -> &Nvic {
        &self.nvics[core]
    }
    #[inline(always)]
    fn nvic_mut(&mut self, core: usize) -> &mut Nvic {
        &mut self.nvics[core]
    }

    #[inline(always)]
    fn active_core(&self) -> usize {
        Bus::active_core(self)
    }

    // --- WFE / SEV wake protocol --------------------------------------

    #[inline(always)]
    fn event_flag(&self, core: usize) -> bool {
        self.event_flag[core]
    }

    #[inline(always)]
    fn consume_event_flag(&mut self, core: usize) -> bool {
        let prior = self.event_flag[core];
        self.event_flag[core] = false;
        prior
    }

    #[inline(always)]
    fn set_wfe_waiting(&mut self, core: usize, val: bool) {
        self.wfe_waiting[core] = val;
    }

    #[inline(always)]
    fn signal_sev(&mut self) {
        Bus::signal_sev(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    struct TimeProbe {
        deltas: Arc<Mutex<Vec<u64>>>,
    }

    impl crate::peripherals::i2c::I2cExternalDevice for TimeProbe {
        fn responds_to(&self, addr: u16) -> bool {
            addr == 0x68
        }

        fn write_byte(&mut self, _byte: u8) -> bool {
            true
        }

        fn read_byte(&mut self) -> u8 {
            0
        }

        fn transaction_end(&mut self) {}

        fn advance_virtual_time(&mut self, delta: crate::peripherals::i2c::I2cVirtualTimeDelta) {
            self.deltas
                .lock()
                .expect("time probe lock")
                .push(delta.nanoseconds);
        }
    }

    #[cfg(feature = "event-horizon-profiler")]
    #[test]
    fn running_profiler_classifies_cpu_visible_accesses_without_memory_noise() {
        use crate::running_profile::RunningBoundaryMask as M;

        let mut bus = Bus::new();
        bus.reset_running_cpu_boundaries();
        let _ = bus.read32(0x2000_0000);
        assert_eq!(bus.take_running_cpu_boundaries().bits(), 0);

        bus.reset_running_cpu_boundaries();
        let _ = bus.read32(SIO_BASE + 0x004);
        let gpio = bus.take_running_cpu_boundaries();
        assert!(gpio.contains(M::CPU_MMIO));
        assert!(gpio.contains(M::GPIO_IN));

        bus.reset_running_cpu_boundaries();
        bus.write32(DMA_BASE, 0);
        let dma = bus.take_running_cpu_boundaries();
        assert!(dma.contains(M::CPU_MMIO));
        assert!(dma.contains(M::FIFO_DREQ));
    }

    #[test]
    fn new_bus_all_peripherals_in_reset() {
        let bus = Bus::new();
        assert_eq!(bus.resets.state, resets::RESET_MASK);
    }

    #[test]
    fn external_i2c_devices_receive_one_shared_delta_per_window() {
        let mut bus = Bus::new();
        bus.seed_sys_clk_hz(100_000_000);
        let deltas = Arc::new(Mutex::new(Vec::new()));
        bus.attach_i2c_device_exclusive(
            1,
            Box::new(TimeProbe {
                deltas: Arc::clone(&deltas),
            }),
        )
        .expect("I2C1 exists");

        bus.master_cycle = 100;
        bus.tick_peripherals(100);
        // Calling the private helper a second time for the same absolute
        // cycle must not double-advance the child.
        bus.advance_external_virtual_time();
        bus.advance_lazy_scheduled(100);

        assert_eq!(
            *deltas.lock().expect("time probe lock"),
            vec![1_000, 1_000],
            "100 cycles at 100 MHz must be delivered once per window"
        );
    }

    #[test]
    fn sram_write_read_roundtrip() {
        let mut bus = Bus::new();
        bus.write32(0x2000_0100, 0xDEAD_BEEF);
        assert_eq!(bus.read32(0x2000_0100), 0xDEAD_BEEF);
    }

    #[test]
    fn sram_aliases_mirror_same_storage() {
        // RP2040 datasheet §2.1.2: all four SRAM alias windows
        // (0x20/0x21/0x22/0x23) address the same backing bytes. Aliases
        // are bank-striping flavours for DMA, not peripheral XOR/SET/CLR.
        let mut bus = Bus::new();
        bus.write32(0x2000_0100, 0xF0F0_F0F0);
        // A write via 0x21xxxxxx overwrites the same bytes, not XORs.
        bus.write32(0x2100_0100, 0x0F0F_0F0F);
        assert_eq!(bus.read32(0x2000_0100), 0x0F0F_0F0F);
        // Reads through every alias observe the identical word.
        assert_eq!(bus.read32(0x2100_0100), 0x0F0F_0F0F);
        assert_eq!(bus.read32(0x2200_0100), 0x0F0F_0F0F);
        assert_eq!(bus.read32(0x2300_0100), 0x0F0F_0F0F);
        // Writing through 0x22 / 0x23 also just overwrites.
        bus.write32(0x2200_0200, 0xAAAA_AAAA);
        bus.write32(0x2300_0200, 0x5555_5555);
        assert_eq!(bus.read32(0x2000_0200), 0x5555_5555);
    }

    #[test]
    fn resets_clr_deasserts() {
        let mut bus = Bus::new();
        // CLR alias at RESETS base 0x4000_C000 + alias 3 → offset 0x3000.
        bus.write32(0x4000_F000, 0x0000_0001);
        assert_eq!(bus.read32(0x4000_C000) & 1, 0);
        assert_eq!(bus.read32(0x4000_C008) & 1, 1);
    }

    #[test]
    fn clocks_ref_mux_switch_to_xosc() {
        let mut bus = Bus::new();
        // CLK_REF_CTRL at 0x4000_8030, write SRC=2 (XOSC).
        bus.write32(0x4000_8030, 2);
        assert_eq!(bus.clock_tree.ref_clk_hz, clocks::XOSC_FREQ_HZ);
    }

    #[test]
    fn clocks_sys_div_write_at_0x40_recomputes_tree() {
        // RP2040 datasheet §2.15.7: CLK_SYS_DIV is at CLOCKS_BASE + 0x40.
        // A write to 0x4000_8040 must land on `clk_sys_div` and feed
        // through `recompute_clock_tree()` — confirming the constants
        // aren't swapped with CLK_SYS_SELECTED (0x44, mux indicator).
        let mut bus = Bus::new();
        bus.seed_sys_clk_hz(clocks::ROSC_FREQ_HZ);
        // DIV integer field lives in bits [31:16]; write /4.
        bus.write32(0x4000_8040, 4 << 16);
        assert_eq!(bus.clocks_regs.clk_sys_div, 4 << 16);
        assert_eq!(bus.clock_tree.sys_clk_hz, clocks::ROSC_FREQ_HZ / 4);
    }

    #[test]
    fn pll_lock_bit_forced_high() {
        // Post-`2026.04.15 HLD - PLL LOCK Modelling` fix: at Bus::new()
        // the PLL is powered down (PWR=0x2D), FBDIV=0, so CS[31] must
        // read 0. The test name is historical — the old pre-fix
        // behaviour forced the bit; the new modelled behaviour derives
        // it from power state + FBDIV + arm timing.
        let mut bus = Bus::new();
        let cs = bus.read32(PLL_SYS_BASE);
        assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 at reset (PLL unpowered)");
    }

    #[test]
    fn sio_gpio_out_roundtrip() {
        let mut bus = Bus::new();
        bus.write32(SIO_BASE + 0x010, 0x5A);
        assert_eq!(bus.read32(SIO_BASE + 0x010), 0x5A);
    }

    #[test]
    fn sio_cpuid_reflects_active_core() {
        let mut bus = Bus::new();
        bus.set_active_core(1);
        assert_eq!(bus.read32(SIO_BASE), 1);
        bus.set_active_core(0);
        assert_eq!(bus.read32(SIO_BASE), 0);
    }

    #[test]
    fn sio_fifo_irq_routes_only_to_receiving_core_and_reasserts() {
        let mut bus = Bus::new();
        bus.sio.set_handshake_armed(false);

        bus.set_active_core(0);
        bus.write32(SIO_BASE + 0x054, 0xCAFE_BABE);
        assert!(!bus.nvics[0].is_pending(IRQ_SIO_IRQ_PROC0 as u8));
        assert!(!bus.nvics[0].is_pending(IRQ_SIO_IRQ_PROC1 as u8));
        assert!(bus.nvics[1].is_pending(IRQ_SIO_IRQ_PROC1 as u8));

        bus.nvics[1].clear_pending(IRQ_SIO_IRQ_PROC1 as u8);
        bus.refresh_sio_fifo_irqs();
        assert!(
            bus.nvics[1].is_pending(IRQ_SIO_IRQ_PROC1 as u8),
            "a still-readable FIFO must reassert its level IRQ"
        );

        bus.set_active_core(1);
        assert_eq!(bus.read32(SIO_BASE + 0x058), 0xCAFE_BABE);
        bus.nvics[1].clear_pending(IRQ_SIO_IRQ_PROC1 as u8);
        bus.refresh_sio_fifo_irqs();
        assert!(!bus.nvics[1].is_pending(IRQ_SIO_IRQ_PROC1 as u8));
    }

    #[test]
    fn gpio_in_is_owned_by_bus() {
        let mut bus = Bus::new();
        bus.gpio_in = 0x42;
        assert_eq!(bus.read32(SIO_BASE + 0x004), 0x42);
    }

    #[test]
    fn xip_fresh_bus_reads_zero_without_fault() {
        // PicoGUS integration (Stage 1 HLD): flash is a plain mapped
        // window. Reads before `load_flash` must return 0 without
        // setting bus_fault so a firmware that probes XIP during boot
        // doesn't take a spurious HardFault.
        let mut bus = Bus::new();
        assert_eq!(bus.read32(0x1000_0000), 0);
        assert_eq!(bus.read8(0x1000_0001), 0);
        assert_eq!(bus.read16(0x1000_0002), 0);
        assert!(!bus.bus_fault());
    }

    #[test]
    fn xip_read_after_flash_load() {
        let mut bus = Bus::new();
        bus.load_flash(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(bus.read32(0x1000_0000), 0xDDCCBBAA);
        assert_eq!(bus.read8(0x1000_0000), 0xAA);
        assert_eq!(bus.read8(0x1000_0003), 0xDD);
        assert_eq!(bus.read16(0x1000_0002), 0xDDCC);
    }

    #[test]
    fn ssi_erase_program_mutates_xip_and_records_zero_to_one() {
        let mut bus = Bus::new();
        bus.load_flash(&[0x00; 0x2000]);
        let tx = |bus: &mut Bus, bytes: &[u8]| {
            bus.write32(SSI_BASE + SSI_SSIENR, 1);
            for &byte in bytes {
                bus.write32(SSI_BASE + SSI_DR0, u32::from(byte));
                let _ = bus.read32(SSI_BASE + SSI_DR0);
            }
            bus.write32(SSI_BASE + SSI_SSIENR, 0);
        };

        tx(&mut bus, &[0x06]); // WREN
        tx(&mut bus, &[0x20, 0x00, 0x00, 0x00]); // sector erase
        assert_eq!(bus.read8(0x1000_0000), 0xFF);
        assert!(
            bus.flash_mutation_errors().is_empty(),
            "errors: {:?}",
            bus.flash_mutation_errors()
        );

        tx(&mut bus, &[0x06]);
        tx(&mut bus, &[0x02, 0x00, 0x00, 0x00, 0xA5]);
        assert_eq!(bus.read8(0x1000_0000), 0xA5);
        assert!(
            bus.flash_mutation_errors().is_empty(),
            "errors: {:?}",
            bus.flash_mutation_errors()
        );

        tx(&mut bus, &[0x06]);
        tx(&mut bus, &[0x02, 0x00, 0x00, 0x00, 0xFF]);
        assert_eq!(bus.read8(0x1000_0000), 0xA5);
        assert!(
            bus.flash_mutation_errors()
                .iter()
                .any(|error| error.starts_with("program_attempted_0_to_1"))
        );
    }

    #[test]
    fn xip_aliases_mirror_flash_base() {
        // RP2040 XIP has three read-only aliases at 0x11/0x12/0x13 that
        // map to the same 2 MB flash window. All four addresses must
        // observe identical bytes after `load_flash`.
        let mut bus = Bus::new();
        bus.load_flash(&[0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE]);
        let words_at = |bus: &mut Bus, base: u32| {
            (
                bus.read32(base),
                bus.read32(base + 4),
                bus.read8(base + 1),
                bus.read16(base + 6),
            )
        };
        let canonical = words_at(&mut bus, 0x1000_0000);
        assert_eq!(words_at(&mut bus, 0x1100_0000), canonical);
        assert_eq!(words_at(&mut bus, 0x1200_0000), canonical);
        assert_eq!(words_at(&mut bus, 0x1300_0000), canonical);
        assert_eq!(canonical.0, 0xEFBEADDE);
    }

    #[test]
    fn xip_read_past_loaded_length_returns_zero() {
        // Within the mapped 2 MB window, addresses past the loaded
        // image length must read 0 (pre-allocated zero bytes in the
        // backing buffer). No bus fault.
        let mut bus = Bus::new();
        bus.load_flash(&[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(bus.read32(0x1000_0004), 0);
        assert_eq!(bus.read32(0x1010_0000), 0); // 1 MB in
        assert_eq!(bus.read32(0x101F_FFFC), 0); // last word of window
        assert_eq!(bus.read32(0x1110_0000), 0); // alias 0x11, mid-window
        assert!(!bus.bus_fault());
    }

    #[test]
    fn xip_writes_silently_ignored_at_every_width() {
        // Real flash needs erase/program via XIP_SSI; at the AHB layer
        // writes to the flash window must not fault and must not alter
        // the loaded bytes.
        let mut bus = Bus::new();
        bus.load_flash(&[0x55, 0x66, 0x77, 0x88]);
        bus.write8(0x1000_0000, 0xAA);
        bus.write16(0x1000_0002, 0xBBBB);
        bus.write32(0x1000_0000, 0xDEAD_BEEF);
        // Aliases must also swallow writes.
        bus.write8(0x1100_0000, 0xAA);
        bus.write16(0x1200_0002, 0xBBBB);
        bus.write32(0x1300_0000, 0xDEAD_BEEF);
        assert!(!bus.bus_fault(), "flash writes must not raise bus_fault");
        assert_eq!(bus.read32(0x1000_0000), 0x88776655);
        assert_eq!(bus.read32(0x1100_0000), 0x88776655);
    }

    #[test]
    fn xip_sram_scratch() {
        let mut bus = Bus::new();
        bus.write32(XIP_SRAM_BASE, 0xCAFE_BABE);
        assert_eq!(bus.read32(XIP_SRAM_BASE), 0xCAFE_BABE);
    }

    #[test]
    fn xip_ctrl_roundtrip() {
        let mut bus = Bus::new();
        bus.write32(XIP_CTRL_BASE + 0x4, 0x1234);
        assert_eq!(bus.read32(XIP_CTRL_BASE + 0x4), 0x1234);
    }

    #[test]
    fn unmapped_region_faults() {
        let mut bus = Bus::new();
        bus.read32(0x7000_0000);
        assert!(bus.bus_fault());
    }

    #[test]
    fn unmapped_writes_fault_at_every_width() {
        // Consistent policy (see write8/write16/write32): unmapped writes
        // at any width set the sticky bus-fault flag.
        let mut bus = Bus::new();
        bus.write8(0x7000_0000, 0xAA);
        assert!(bus.bus_fault(), "write8 to unmapped region must fault");
        bus.clear_bus_fault();

        bus.write16(0x7000_0000, 0xAABB);
        assert!(bus.bus_fault(), "write16 to unmapped region must fault");
        bus.clear_bus_fault();

        bus.write32(0x7000_0000, 0xAABB_CCDD);
        assert!(bus.bus_fault(), "write32 to unmapped region must fault");
    }

    #[test]
    fn rom_writes_silently_ignored_at_every_width() {
        // ROM is read-only — writes at any width must NOT raise bus_fault.
        let mut bus = Bus::new();
        bus.write8(0x0000_0100, 0xAA);
        assert!(!bus.bus_fault(), "write8 to ROM is silent");
        bus.write16(0x0000_0100, 0xAABB);
        assert!(!bus.bus_fault(), "write16 to ROM is silent");
        bus.write32(0x0000_0100, 0xAABB_CCDD);
        assert!(!bus.bus_fault(), "write32 to ROM is silent");
    }

    #[test]
    fn sram_bank_contention_plus_one_cycle() {
        let mut bus = Bus::new();
        // Core 0 touches bank 0.
        bus.set_active_core(0);
        let _ = bus.read32(0x2000_0000);
        // Core 1 touches the same bank — expect +1 cycle latency.
        bus.set_active_core(1);
        bus.begin_core1_step();
        let _ = bus.read32(0x2000_0000);
        assert_eq!(bus.last_access_cycles, 2);
        bus.end_core1_step();
    }

    #[test]
    fn sram_bank_no_contention_different_banks() {
        let mut bus = Bus::new();
        bus.set_active_core(0);
        let _ = bus.read32(0x2000_0000); // bank 0
        bus.set_active_core(1);
        bus.begin_core1_step();
        let _ = bus.read32(0x2000_0004); // bank 1
        assert_eq!(bus.last_access_cycles, 1);
        bus.end_core1_step();
    }

    /// A thread-safe `Vec<u8>` sink so we can capture the MMIO trace output
    /// without wrestling with stdout redirection. Wraps `Vec<u8>` behind
    /// an `Arc<Mutex<...>>` so the test can drain the buffer after the
    /// bus has written through the sink. (`Bus::mmio_trace_sink` requires
    /// `Write + Send`.)
    #[derive(Clone)]
    struct CaptureSink(std::sync::Arc<std::sync::Mutex<Vec<u8>>>);

    impl std::io::Write for CaptureSink {
        fn write(&mut self, data: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(data);
            Ok(data.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn mmio_trace_enabled_emits_write32_line() {
        // HLD V7 §4.3: `write32(addr, val)` with `mmio_trace_enabled = true`
        // emits one line in the prescribed format. We inject a captured
        // `CaptureSink` so the test doesn't depend on fd 1 redirection.
        let capture = CaptureSink(std::sync::Arc::new(std::sync::Mutex::new(Vec::new())));
        let mut bus = Bus::new();
        bus.set_active_core(0);
        bus.set_active_pc(0x1000_0100);
        bus.mmio_trace_enabled = true;
        bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));

        // SRAM word write — exercises the hot path and one of the six
        // access methods required by the spec.
        bus.write32(0x2000_0200, 0xDEAD_BEEF);

        let captured = capture.0.lock().unwrap();
        let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
        // Exactly one line, with the expected fields.
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines.len(),
            1,
            "expected one trace line, got {}: {:?}",
            lines.len(),
            lines
        );
        let line = lines[0];
        assert!(
            line.starts_with("TRACE W 4 0x20000200"),
            "line = {:?}",
            line
        );
        assert!(line.contains("val=0xDEADBEEF"), "line = {:?}", line);
        assert!(line.contains("core=0"), "line = {:?}", line);
        assert!(line.contains("pc=0x10000100"), "line = {:?}", line);
    }

    #[test]
    fn trace_disabled_emits_nothing() {
        // Zero-overhead path — `mmio_trace_enabled = false` must not route any
        // bytes to the sink. Guards the hot path (non-trace runs must not
        // pay a formatting cost).
        let capture = CaptureSink(std::sync::Arc::new(std::sync::Mutex::new(Vec::new())));
        let mut bus = Bus::new();
        bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));
        // mmio_trace_enabled is false by default.
        bus.write32(0x2000_0200, 0xCAFE_F00D);
        let _ = bus.read32(0x2000_0200);
        assert!(capture.0.lock().unwrap().is_empty());
    }

    #[test]
    fn instruction_pc_publication_respects_opt4d_boundary() {
        use crate::core::CoreBus;

        let mut bus = Bus::new();
        bus.set_active_core(0);
        bus.set_active_pc(0x1111_0000);
        CoreBus::set_active_pc_for_instruction(&mut bus, 0x2222_0000);

        #[cfg(not(feature = "diagnostic-pc-compile-out-prototype"))]
        assert_eq!(bus.active_pc[0], 0x2222_0000);
        #[cfg(feature = "diagnostic-pc-compile-out-prototype")]
        assert_eq!(bus.active_pc[0], 0x1111_0000);
    }

    #[test]
    fn trace_active_pc_is_per_core() {
        // Regression guard for the dual-core `active_pc` staleness bug
        // (Wave 2 review SHOULD-FIX 1). The scheduler alternates
        // `set_active_core(0)` / `set_active_core(1)` each quantum; a
        // bus access on core 1 that doesn't go through `decode_execute`
        // (e.g. exception stacking) must NOT observe core 0's last decode
        // PC, and vice versa. Simulate the pattern: decode PC=0x1000 on
        // core 0, switch to core 1, decode PC=0x2000, switch back to
        // core 0 and issue an access *without* re-decoding — the trace
        // line must still carry PC=0x1000 for core 0.
        let capture = CaptureSink(std::sync::Arc::new(std::sync::Mutex::new(Vec::new())));
        let mut bus = Bus::new();
        bus.mmio_trace_enabled = true;
        bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));

        // Core 0 "decodes" at 0x1000 and writes.
        bus.set_active_core(0);
        bus.set_active_pc(0x0000_1000);
        bus.write32(0x2000_0100, 0xAAAA_AAAA);

        // Scheduler switches to core 1, which "decodes" at 0x2000 and
        // writes.
        bus.set_active_core(1);
        bus.set_active_pc(0x0000_2000);
        bus.write32(0x2000_0104, 0xBBBB_BBBB);

        // Scheduler switches back to core 0 WITHOUT a re-decode (mimics
        // hardware-triggered access like exception stacking before the
        // handler's first `decode_execute`). The stored per-core PC
        // must still be 0x1000 for core 0 — not 0x2000 from core 1's
        // quantum.
        bus.set_active_core(0);
        bus.write32(0x2000_0108, 0xCCCC_CCCC);

        let captured = capture.0.lock().unwrap();
        let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines.len(),
            3,
            "expected three trace lines, got {:?}",
            lines
        );
        assert!(
            lines[0].contains("core=0") && lines[0].contains("pc=0x00001000"),
            "line 0 = {:?}",
            lines[0]
        );
        assert!(
            lines[1].contains("core=1") && lines[1].contains("pc=0x00002000"),
            "line 1 = {:?}",
            lines[1]
        );
        assert!(
            lines[2].contains("core=0") && lines[2].contains("pc=0x00001000"),
            "line 2 = {:?} (core 0 PC must survive the core-1 excursion)",
            lines[2]
        );
    }

    // ----- NVIC MMIO dispatch (Phase 1 Wave 2, HLD V7 §5.2) --------------
    //
    // Exercise the bus-level interception of the NVIC register window
    // `0xE000_E100..=0xE000_E41F`. Reads go through the Bus's normal
    // `read32`/`read16`/`read8` API so alias + byte/halfword access is
    // covered; writes mirror.

    #[test]
    fn nvic_iser0_write_sets_enable_read_back() {
        let mut bus = Bus::new();
        // Enable IRQ 5 + IRQ 12.
        bus.write32(0xE000_E100, (1u32 << 5) | (1u32 << 12));
        // ISER0 read returns the enabled mask.
        assert_eq!(bus.read32(0xE000_E100), (1u32 << 5) | (1u32 << 12));
        // ICER0 read aliases the same mask.
        assert_eq!(bus.read32(0xE000_E180), (1u32 << 5) | (1u32 << 12));
        // Writing ICER0 clears the specified bits.
        bus.write32(0xE000_E180, 1u32 << 5);
        assert_eq!(bus.read32(0xE000_E100), 1u32 << 12);
        // ISER0 write is write-1-set (does not clear unchanged bits).
        bus.write32(0xE000_E100, 1u32 << 3);
        assert_eq!(bus.read32(0xE000_E100), (1u32 << 3) | (1u32 << 12));
    }

    #[test]
    fn nvic_ispr0_write_sets_pending_read_back() {
        let mut bus = Bus::new();
        // Set pending IRQ 0 via ISPR0.
        bus.write32(0xE000_E200, 1u32 << 0);
        assert_eq!(bus.read32(0xE000_E200), 1u32 << 0);
        assert_eq!(bus.read32(0xE000_E280), 1u32 << 0); // ICPR0 shows pending too
        // W1C via ICPR0.
        bus.write32(0xE000_E280, 1u32 << 0);
        assert_eq!(bus.read32(0xE000_E200), 0);
    }

    #[test]
    fn nvic_ipr_word_encodes_four_priorities() {
        let mut bus = Bus::new();
        // Write IPR1 (covers IRQs 4..=7). Priority 0xC0 on IRQ 4 (lane 0),
        // 0x80 on IRQ 5 (lane 1), 0x40 on IRQ 6 (lane 2), 0x00 on IRQ 7
        // (lane 3).
        let word = 0xC0u32 | (0x80u32 << 8) | (0x40u32 << 16);
        bus.write32(0xE000_E404, word);
        assert_eq!(bus.read32(0xE000_E404), word);
        // Non-implemented bits of a priority byte must be masked — write
        // 0x3F (all low bits) to IRQ 8 on IPR2 lane 0; readback is 0.
        bus.write32(0xE000_E408, 0x3F);
        assert_eq!(bus.read32(0xE000_E408), 0);
    }

    #[test]
    fn nvic_iser_then_icer_clears() {
        // V5 §5.1 audit shape: write 0xFFFF to ISER0, then 0x00FF to
        // ICER0; ISER0 readback must be 0xFF00 (high half preserved,
        // low half cleared by W1C).
        let mut bus = Bus::new();
        bus.write32(0xE000_E100, 0x0000_FFFF);
        bus.write32(0xE000_E180, 0x0000_00FF);
        assert_eq!(bus.read32(0xE000_E100), 0x0000_FF00);
        // ICPR mirrors ISPR for pending — analogous shape covered by
        // `nvic_ispr0_write_sets_pending_read_back` above.
    }

    #[test]
    fn nvic_ipr_top_two_bits_significant() {
        // V5 §5.1 audit shape: writing 0xC5 to a priority byte must
        // mask to 0xC0 (only bits [7:6] are implemented on M0+).
        let mut bus = Bus::new();
        // IPR0 lane 0 covers IRQ 0.
        bus.write32(0xE000_E400, 0xC5);
        assert_eq!(bus.read32(0xE000_E400), 0xC0);
    }

    #[test]
    fn nvic_is_per_core_banked() {
        // ARMv6-M banks the SCS per-core. Active core = 0: writes land
        // on nvics[0] only; active core = 1 sees independent state.
        let mut bus = Bus::new();
        bus.set_active_core(0);
        bus.write32(0xE000_E100, 1u32 << 4);
        bus.set_active_core(1);
        bus.write32(0xE000_E100, 1u32 << 19);
        // Core 0 must NOT see IRQ 19 enabled; core 1 must NOT see IRQ 4.
        bus.set_active_core(0);
        assert_eq!(bus.read32(0xE000_E100), 1u32 << 4);
        bus.set_active_core(1);
        assert_eq!(bus.read32(0xE000_E100), 1u32 << 19);
    }

    #[test]
    fn nvic_mmio_reset_gate_does_not_apply() {
        // NVIC is part of SCS (0xE...), not an APB peripheral — no RESETS
        // bit gates it. Freshly-constructed bus with everything held in
        // reset still honours NVIC reads/writes.
        let mut bus = Bus::new();
        bus.write32(0xE000_E100, 1u32 << 0);
        assert_eq!(bus.read32(0xE000_E100), 1u32 << 0);
    }

    #[test]
    fn nvic_high_bits_razwi() {
        // RP2040 has 26 IRQ lines (bits 0..=25). Writes to ISER0/ICER0/
        // ISPR0/ICPR0 with bits 26..31 set must be silently dropped on
        // real silicon (RAZ/WI). Reads naturally RAZ since the field
        // never has high bits set.
        let mut bus = Bus::new();
        // ISER0: write all-ones, expect only the 26 implemented bits.
        bus.write32(0xE000_E100, 0xFFFF_FFFF);
        assert_eq!(bus.read32(0xE000_E100), 0x03FF_FFFF);
        // ICER0: with all 26 enable bits set, an all-ones write must
        // clear them all (no high-bit residue gating the AND-NOT).
        bus.write32(0xE000_E180, 0xFFFF_FFFF);
        assert_eq!(bus.read32(0xE000_E100), 0);
        // ISPR0: same shape as ISER0 on the SET side.
        bus.write32(0xE000_E200, 0xFFFF_FFFF);
        assert_eq!(bus.read32(0xE000_E200), 0x03FF_FFFF);
        // ICPR0: same shape as ICER0 on the CLEAR side.
        bus.write32(0xE000_E280, 0xFFFF_FFFF);
        assert_eq!(bus.read32(0xE000_E200), 0);

        // Force high bits into the field directly to exercise the
        // clear-arm mask: without `val & IRQ_LINE_MASK` on the ICER0/
        // ICPR0 arms, an all-ones write would clear the high bits too.
        bus.nvics[0].enabled = 0xFFFF_FFFF;
        bus.write32(0xE000_E180, 0xFFFF_FFFF); // ICER0 — should clear only bits 0..25
        assert_eq!(
            bus.read32(0xE000_E100),
            0xFC00_0000,
            "ICER0 must mask val before clearing — high bits preserved"
        );

        bus.nvics[0].pending = 0xFFFF_FFFF;
        bus.write32(0xE000_E280, 0xFFFF_FFFF); // ICPR0 — should clear only bits 0..25
        assert_eq!(
            bus.read32(0xE000_E200),
            0xFC00_0000,
            "ICPR0 must mask val before clearing — high bits preserved"
        );
    }

    // ----- ADC + PWM bus integration (Phase 3) --------------------------

    #[test]
    fn adc_held_in_reset_returns_zero() {
        // Default Bus holds RESET_ADC (bit 0) asserted — reads must
        // return 0 and writes must not route.
        let mut bus = Bus::new();
        bus.write32(ADC_BASE + crate::peripherals::adc::CS, 0x1);
        assert_eq!(
            bus.read32(ADC_BASE + crate::peripherals::adc::CS),
            0,
            "ADC held in reset must RAZ"
        );
    }

    #[test]
    fn adc_unreset_roundtrips_cs() {
        let mut bus = Bus::new();
        // Clear RESET_ADC (bit 0) via CLR alias at RESETS+0x3000.
        bus.write32(0x4000_F000, 0x1);
        bus.write32(
            ADC_BASE + crate::peripherals::adc::CS,
            crate::peripherals::adc::CS_EN,
        );
        assert_eq!(
            bus.read32(ADC_BASE + crate::peripherals::adc::CS) & crate::peripherals::adc::CS_EN,
            crate::peripherals::adc::CS_EN
        );
    }

    #[test]
    fn pwm_held_in_reset_returns_zero() {
        let mut bus = Bus::new();
        bus.write32(PWM_BASE + crate::peripherals::pwm::EN, 0xFF);
        assert_eq!(bus.read32(PWM_BASE + crate::peripherals::pwm::EN), 0);
    }

    #[test]
    fn pwm_unreset_roundtrips_slice_registers() {
        let mut bus = Bus::new();
        // Clear RESET_PWM (bit 14).
        bus.write32(0x4000_F000, 1u32 << 14);
        // Slice 0 TOP = 100 via canonical address.
        let slice0_top = PWM_BASE + 0x10;
        bus.write32(slice0_top, 100);
        assert_eq!(bus.read32(slice0_top), 100);
    }

    // ----- PIO TXF narrow-write dispatch (Phase D) -------------------------
    //
    // Regression tests for the PIO byte/halfword TXF write fix.
    // DMA_SIZE_8 writes to TXF were silently dropped by the blanket PIO
    // guard in `write8`/`write16`, breaking rp2040-psram transfers.

    #[test]
    fn pio_txf_byte_write_pushes_replicated_word() {
        let mut bus = Bus::new();
        // Release PIO1 from reset (RESETS bit 11).
        bus.write32(0x4000_F000, 1u32 << 11);
        // Enable PIO1 SM0 via CTRL.SM_ENABLE bit 0.
        bus.write32(PIO1_BASE, 0x1);
        // Byte write to PIO1 TXF0.
        bus.write8(PIO1_BASE + 0x010, 0x42);
        // FSTAT: TXEMPTY bit 24 for SM0 must be cleared.
        let fstat = bus.read32(PIO1_BASE + 0x004);
        assert_eq!(
            fstat & (1 << 24),
            0,
            "TX FIFO must not be empty after byte write"
        );
        // Pop from TX FIFO and verify byte-replicated value.
        let val = bus.pio[1].pop_tx(0).expect("TX FIFO should have one entry");
        assert_eq!(
            val, 0x42424242,
            "byte 0x42 must be replicated to all four lanes"
        );
    }

    #[test]
    fn pio_txf_halfword_write_pushes_replicated_word() {
        let mut bus = Bus::new();
        // Release PIO1 from reset.
        bus.write32(0x4000_F000, 1u32 << 11);
        // Enable PIO1 SM0.
        bus.write32(PIO1_BASE, 0x1);
        // Halfword write to PIO1 TXF0.
        bus.write16(PIO1_BASE + 0x010, 0x1234);
        // FSTAT: TXEMPTY bit 24 for SM0 must be cleared.
        let fstat = bus.read32(PIO1_BASE + 0x004);
        assert_eq!(
            fstat & (1 << 24),
            0,
            "TX FIFO must not be empty after halfword write"
        );
        // Pop from TX FIFO and verify halfword-replicated value.
        let val = bus.pio[1].pop_tx(0).expect("TX FIFO should have one entry");
        assert_eq!(
            val, 0x12341234,
            "halfword 0x1234 must be replicated to both lanes"
        );
    }

    #[test]
    fn pio_non_txf_byte_write_still_dropped() {
        let mut bus = Bus::new();
        // Release PIO1 from reset.
        bus.write32(0x4000_F000, 1u32 << 11);
        // Write CTRL to enable SM0 (known baseline).
        bus.write32(PIO1_BASE, 0x1);
        let ctrl_before = bus.read32(PIO1_BASE);
        // Byte write to PIO1 CTRL (offset 0x000) — must be silently dropped.
        bus.write8(PIO1_BASE, 0xFF);
        let ctrl_after = bus.read32(PIO1_BASE);
        assert_eq!(
            ctrl_before, ctrl_after,
            "byte write to non-TXF PIO register must be dropped"
        );
    }

    #[test]
    fn adc_fifo_narrow_read_does_not_double_pop() {
        // Drive one sample into the ADC FIFO and read it back byte-wise
        // via `read8` — the narrow dispatch must pop exactly one entry,
        // not trigger an RMW that pops twice.
        let mut bus = Bus::new();
        // Release ADC from reset.
        bus.write32(0x4000_F000, 0x1);
        // Prime the internal FIFO with one known sample by driving
        // through the Phase 3 path: enable + FCS + start_once + tick.
        bus.adc.reset();
        let mut irqs = 0u32;
        // Channel 3 so the sample is non-zero.
        bus.adc.write32(
            crate::peripherals::adc::FCS,
            crate::peripherals::adc::FCS_EN,
            0,
            &mut irqs,
        );
        bus.adc.write32(
            crate::peripherals::adc::CS,
            crate::peripherals::adc::CS_EN | crate::peripherals::adc::CS_START_ONCE | (3 << 12),
            0,
            &mut irqs,
        );
        // Tick enough sys cycles to complete the conversion
        // (96 adc_clk / 48 MHz ≈ 2 µs = 250 sys_clk at 125 MHz).
        bus.seed_sys_clk_hz(125_000_000);
        bus.adc.tick(400, &bus.clock_tree, &mut irqs);
        assert_eq!(bus.adc.fifo_len(), 1, "one sample must be queued");
        // Byte read must not over-pop — still one sample, count drops
        // to zero only once.
        let _ = bus.read8(ADC_BASE + crate::peripherals::adc::FIFO);
        assert_eq!(bus.adc.fifo_len(), 0, "byte read pops exactly once");
    }

    /// HLD V5 §5.2: a SysTick underflow must set `ICSR.PENDSTSET` (bit
    /// 26) only on the active core's PPB. Tick the active-core SysTick
    /// directly here — the per-cycle tick wired into `Emulator::step`
    /// is exercised end-to-end by §6.2's integration test, but the
    /// active-core-only invariant is testable purely at the bus level.
    #[test]
    fn tick_underflow_sets_icsr_pendst_on_active_core_only() {
        let mut bus = Bus::new();
        // Run from core 0's perspective so MMIO writes target
        // `systicks[0]`.
        bus.set_active_core(0);
        // Enable SysTick with TICKINT and CLKSOURCE=processor; CVR=0
        // so the very first tick fires.
        bus.write32(0xE000_E018, 0); // CVR
        bus.write32(0xE000_E014, 0); // RVR (period 1)
        bus.write32(0xE000_E010, 0b111); // CSR: ENABLE | TICKINT | CLKSOURCE
        let fired = bus.systicks[0].tick();
        assert!(fired, "TICKINT-armed first tick must fire");
        if fired {
            bus.ppb[0].icsr |= 1 << 26;
        }
        assert_ne!(
            bus.ppb[0].icsr & (1 << 26),
            0,
            "PENDSTSET must latch on active-core PPB"
        );
        assert_eq!(
            bus.ppb[1].icsr & (1 << 26),
            0,
            "PENDSTSET must NOT latch on the other core"
        );
    }
}
