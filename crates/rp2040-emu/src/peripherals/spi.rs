//! RP2040 SPI peripheral (PL022-derived; datasheet §4.4).
//!
//! Phase 2 of the RP2040 peripheral coverage plan (HLD V7 §5.3 / §6).
//! Two instances live at `0x4003_C000` (SPI0) and `0x4004_0000` (SPI1).
//! Observed-register subset only — pico-sdk's `spi_master` loopback
//! exercises `SSPCR0`, `SSPCR1`, `SSPCPSR`, `SSPDR`, `SSPSR`, and the
//! interrupt registers, which is what this module models. Everything
//! else (`SSPCR1.SOD`, slave-only modes) is storage-round-trip.
//!
//! # Register map (offsets relative to `SSPn_BASE`)
//!
//! | Offset  | Name       | Access | Notes                                |
//! |---------|------------|--------|--------------------------------------|
//! | `0x000` | `SSPCR0`   | R/W    | Frame format, clock rate             |
//! | `0x004` | `SSPCR1`   | R/W    | Enable, LBM, MS, SOD                 |
//! | `0x008` | `SSPDR`    | R/W    | Data (FIFO push/pop)                 |
//! | `0x00C` | `SSPSR`    | RO     | TFE/TNF/RNE/RFF/BSY status           |
//! | `0x010` | `SSPCPSR`  | R/W    | Clock prescale divisor               |
//! | `0x014` | `SSPIMSC`  | R/W    | Interrupt mask                       |
//! | `0x018` | `SSPRIS`   | RO     | Raw interrupt status                 |
//! | `0x01C` | `SSPMIS`   | RO     | Masked interrupt status              |
//! | `0x020` | `SSPICR`   | W1C    | Interrupt clear (RTIC / RORIC only)  |
//! | `0x024` | `SSPDMACR` | R/W    | DMA control                          |
//! | `0xFE0..0xFEC` | `SSPPERIPHID0..3` | RO | PrimeCell peripheral ID     |
//! | `0xFF0..0xFFC` | `SSPPCELLID0..3`  | RO | PrimeCell ID                 |
//!
//! # Loopback model (`SSPCR1.LBM`)
//!
//! When firmware sets `SSPCR1.LBM=1`, every write to `SSPDR` pushes the
//! word into the RX FIFO directly — simulating the PL022's internal TX
//! → RX tie. This is exactly what the `spi_master` corpus binary
//! expects: write 0xA5, read 0xA5 back. When LBM=0 the TX FIFO drains
//! off-chip with no RX response (a full PIO-driven external slave is
//! out of scope for Phase 2).
//!
//! # Baud-rate cadence (non-loopback)
//!
//! Even in non-loopback mode the TX FIFO must eventually drain so
//! `SSPSR.BSY` can fall back to 0. [`SpiRegs::tick`] models the PL022
//! clock rate as `clk_peri / (SCR + 1) / SSPCPSR` and pops one TX
//! FIFO entry every `sysclks_per_word`. When `LBM=1` that drain
//! replays into the RX FIFO (already queued at write time to keep
//! `spi_master`'s poll-then-read rhythm deterministic).
//!
//! # IRQ sources
//!
//! The PL022 surfaces four interrupts ORed onto the peripheral's
//! single NVIC line (SPI0=18, SPI1=19):
//! * `SSPRIS.ROR` — RX overrun (not modelled; never raised).
//! * `SSPRIS.RT`  — RX timeout.
//! * `SSPRIS.RX`  — RX FIFO ≥ 1/2 full.
//! * `SSPRIS.TX`  — TX FIFO ≤ 1/2 full.

use std::collections::VecDeque;

use picoem_common::clocks::ClockTree;

/// An off-chip SPI slave attached to one of the RP2040 SSP instances.
///
/// Generic hook: `rp2040-emu` knows nothing about what the device is or
/// which GPIOs frame its transactions. Board-level crates implement this
/// trait and decide, from the pin snapshot handed to
/// [`SpiExternalDevice::observe_pins`], whether a given word is addressed
/// to them.
///
/// Lifecycle inside one [`SpiRegs::tick`]:
///
/// 1. The bus calls [`SpiExternalDevice::observe_pins`] with the merged
///    SIO/PIO pad-output levels *before* any TX FIFO drain, so the device
///    sees the chip-select / command-data levels the CPU had established
///    by the end of the instruction burst that queued the word. Sampling
///    them after the drain (or one `step()` later, from a runner) would
///    frame the word with the *next* transaction's control lines.
/// 2. Every word popped off the TX FIFO is handed to
///    [`SpiExternalDevice::transfer`]; the returned word is pushed into
///    the RX FIFO — non-loopback only, because in loopback the PL022 ties
///    TX to RX inside the chip and the off-chip pins are bypassed.
pub trait SpiExternalDevice: Send {
    /// Shift one frame out to the device and return the frame shifted
    /// back in on MISO. `bits` is the configured frame width taken from
    /// `SSPCR0.DSS` (4..=16).
    fn transfer(&mut self, word: u16, bits: u8) -> u16;

    /// Observe the current GPIO pad-output levels (bit *n* = GPIO *n*).
    /// Called once per [`SpiRegs::tick`], before the FIFO drain. Devices
    /// with no side-band pins can ignore it.
    fn observe_pins(&mut self, _gpio_out_levels: u32) {}
}

/// Offset: `SSPCR0` — frame format / clock rate.
pub const SSPCR0: u32 = 0x000;
/// Offset: `SSPCR1` — enable / LBM / MS / SOD.
pub const SSPCR1: u32 = 0x004;
/// Offset: `SSPDR` — data (byte/halfword side-effect).
pub const SSPDR: u32 = 0x008;
/// Offset: `SSPSR` — status (read-only).
pub const SSPSR: u32 = 0x00C;
/// Offset: `SSPCPSR` — clock prescale.
pub const SSPCPSR: u32 = 0x010;
/// Offset: `SSPIMSC` — interrupt mask.
pub const SSPIMSC: u32 = 0x014;
/// Offset: `SSPRIS` — raw interrupt status.
pub const SSPRIS: u32 = 0x018;
/// Offset: `SSPMIS` — masked interrupt status.
pub const SSPMIS: u32 = 0x01C;
/// Offset: `SSPICR` — W1C interrupt clear (only RTIC + RORIC are valid bits).
pub const SSPICR: u32 = 0x020;
/// Offset: `SSPDMACR` — DMA control.
pub const SSPDMACR: u32 = 0x024;

pub const SSPPERIPHID0: u32 = 0xFE0;
pub const SSPPERIPHID1: u32 = 0xFE4;
pub const SSPPERIPHID2: u32 = 0xFE8;
pub const SSPPERIPHID3: u32 = 0xFEC;
pub const SSPPCELLID0: u32 = 0xFF0;
pub const SSPPCELLID1: u32 = 0xFF4;
pub const SSPPCELLID2: u32 = 0xFF8;
pub const SSPPCELLID3: u32 = 0xFFC;

// --- SSPCR1 bits ------------------------------------------------------
const SSPCR1_LBM: u32 = 1 << 0;
const SSPCR1_SSE: u32 = 1 << 1;

// --- SSPSR bits -------------------------------------------------------
const SSPSR_TFE: u32 = 1 << 0; // TX FIFO empty
const SSPSR_TNF: u32 = 1 << 1; // TX FIFO not full
const SSPSR_RNE: u32 = 1 << 2; // RX FIFO not empty
const SSPSR_RFF: u32 = 1 << 3; // RX FIFO full
const SSPSR_BSY: u32 = 1 << 4; // busy

// --- Interrupt bits (shared across IMSC / RIS / MIS / ICR) ------------
pub const SSP_INT_ROR: u32 = 1 << 0;
pub const SSP_INT_RT: u32 = 1 << 1;
pub const SSP_INT_RX: u32 = 1 << 2;
pub const SSP_INT_TX: u32 = 1 << 3;
const SSP_INT_MASK: u32 = SSP_INT_ROR | SSP_INT_RT | SSP_INT_RX | SSP_INT_TX;

/// PL022 FIFO depth.
pub const SSP_FIFO_DEPTH: usize = 8;

/// PL022 peripheral ID (r1p3). TRM Table 2-18 canonical values.
const PERIPH_ID: [u32; 4] = [0x22, 0x10, 0x34, 0x00];
const PCELL_ID: [u32; 4] = [0x0D, 0xF0, 0x05, 0xB1];

/// PL022-derived SPI (RP2040 §4.4).
pub struct SpiRegs {
    cr0: u32,
    cr1: u32,
    cpsr: u32,
    imsc: u32,
    ris: u32,
    dmacr: u32,
    tx_fifo: VecDeque<u32>,
    rx_fifo: VecDeque<u32>,
    tx_cycle_accum: u64,
    nvic_irq: u32,
    /// Optional off-chip slave. `None` — the default — reproduces the
    /// pre-hook behaviour exactly: drained words are discarded and
    /// nothing is ever pushed into the RX FIFO.
    device: Option<Box<dyn SpiExternalDevice>>,
}

impl SpiRegs {
    /// Construct a fresh SPI at power-on default state. `nvic_irq` is
    /// the NVIC line (18 for SPI0, 19 for SPI1 on RP2040).
    pub fn new(nvic_irq: u32) -> Self {
        Self {
            cr0: 0,
            cr1: 0,
            cpsr: 0,
            imsc: 0,
            ris: 0,
            dmacr: 0,
            tx_fifo: VecDeque::with_capacity(SSP_FIFO_DEPTH),
            rx_fifo: VecDeque::with_capacity(SSP_FIFO_DEPTH),
            tx_cycle_accum: 0,
            nvic_irq,
            device: None,
        }
    }

    pub fn reset(&mut self) {
        let irq = self.nvic_irq;
        // The attached device models a *board*, not a chip register: an
        // MCU-side peripheral reset does not unsolder it. Carry it
        // across so `Emulator::reset` cannot silently detach it.
        let device = self.device.take();
        *self = Self::new(irq);
        self.device = device;
    }

    /// Attach (or replace) the off-chip slave on this instance.
    /// Returns whatever was attached before, if anything.
    pub fn attach_device(
        &mut self,
        device: Box<dyn SpiExternalDevice>,
    ) -> Option<Box<dyn SpiExternalDevice>> {
        self.device.replace(device)
    }

    /// True iff an off-chip slave is attached.
    #[inline]
    pub fn has_device(&self) -> bool {
        self.device.is_some()
    }

    /// Forward the merged GPIO pad-output snapshot to the attached
    /// device. No-op when nothing is attached.
    #[inline]
    pub fn observe_pins(&mut self, gpio_out_levels: u32) {
        if let Some(dev) = self.device.as_mut() {
            dev.observe_pins(gpio_out_levels);
        }
    }

    /// Frame width in bits per `SSPCR0.DSS`, clamped to the PL022's
    /// legal range exactly the way [`Self::frame_data_mask`] does.
    #[inline]
    fn frame_bits(&self) -> u8 {
        ((self.cr0 & 0xF).max(3) + 1) as u8
    }

    /// True iff no outstanding work — TX and RX FIFOs empty.
    pub fn is_idle(&self) -> bool {
        self.tx_fifo.is_empty() && self.rx_fifo.is_empty() && self.ris == 0
    }

    /// OPT0 diagnostic classification: distinguish an actively shifting
    /// transmitter from static FIFO/interrupt state.
    #[cfg(feature = "idle-profiler")]
    pub(crate) fn idle_profile_state(&self) -> crate::idle_profile::IdlePeripheralState {
        crate::idle_profile::IdlePeripheralState {
            temporal_work: self.is_enabled() && !self.tx_fifo.is_empty(),
            routable_irq: (self.ris & self.imsc) != 0,
            static_state: !self.tx_fifo.is_empty()
                || !self.rx_fifo.is_empty()
                || self.ris != 0
                || self.tx_cycle_accum != 0,
        }
    }

    /// DREQ: TX FIFO has room and the peripheral is enabled. Phase 4
    /// DMA TREQ matrix consults this for `SPI0_TX` / `SPI1_TX`.
    #[inline]
    pub fn tx_dreq(&self) -> bool {
        self.is_enabled() && self.tx_fifo.len() < SSP_FIFO_DEPTH
    }

    /// DREQ: RX FIFO has data to drain.
    #[inline]
    pub fn rx_dreq(&self) -> bool {
        self.is_enabled() && !self.rx_fifo.is_empty()
    }

    #[inline]
    fn is_enabled(&self) -> bool {
        (self.cr1 & SSPCR1_SSE) != 0
    }

    #[inline]
    fn is_loopback(&self) -> bool {
        (self.cr1 & SSPCR1_LBM) != 0
    }

    /// Frame data width in bits, per `SSPCR0.DSS` ([3:0]). 4 → 5-bit
    /// frame, ..., 15 → 16-bit frame. For masking purposes we need the
    /// low-N-bits value so loopback round-trips every written bit.
    fn frame_data_mask(&self) -> u32 {
        let dss = self.cr0 & 0xF;
        // DSS encoding: 3 = 4-bit, ..., 15 = 16-bit.
        let bits = dss.max(3) + 1;
        if bits >= 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        }
    }

    fn sr_read(&self) -> u32 {
        let mut sr = 0u32;
        if self.tx_fifo.is_empty() {
            sr |= SSPSR_TFE;
        } else {
            sr |= SSPSR_BSY;
        }
        if self.tx_fifo.len() < SSP_FIFO_DEPTH {
            sr |= SSPSR_TNF;
        }
        if !self.rx_fifo.is_empty() {
            sr |= SSPSR_RNE;
        }
        if self.rx_fifo.len() >= SSP_FIFO_DEPTH {
            sr |= SSPSR_RFF;
        }
        sr
    }

    fn route_irq(&self, irqs: &mut u32) {
        if (self.ris & self.imsc) != 0 {
            *irqs |= 1u32 << self.nvic_irq;
        }
    }

    fn refresh_tx_rx_interrupts(&mut self) {
        // PL022 TX latches when TX FIFO ≤ 1/2 (4 of 8 entries).
        if self.tx_fifo.len() <= SSP_FIFO_DEPTH / 2 {
            self.ris |= SSP_INT_TX;
        }
        // RX latches when RX FIFO ≥ 1/2 full.
        if self.rx_fifo.len() >= SSP_FIFO_DEPTH / 2 {
            self.ris |= SSP_INT_RX;
        } else {
            // Level-fall: once RX drains below threshold, drop the bit
            // so firmware can re-arm without a spurious re-trigger.
            self.ris &= !SSP_INT_RX;
        }
    }

    /// Push a word into the TX FIFO; loopback mirrors into RX.
    fn push_dr(&mut self, value: u32, irqs: &mut u32) {
        if !self.is_enabled() {
            return;
        }
        let mask = self.frame_data_mask();
        let word = value & mask;
        if self.tx_fifo.len() < SSP_FIFO_DEPTH {
            self.tx_fifo.push_back(word);
            if self.is_loopback() && self.rx_fifo.len() < SSP_FIFO_DEPTH {
                self.rx_fifo.push_back(word);
            }
        } else {
            // Overrun latched when RX FIFO can't accept a loopback copy.
            if self.is_loopback() {
                self.ris |= SSP_INT_ROR;
            }
        }
        self.refresh_tx_rx_interrupts();
        self.route_irq(irqs);
    }

    /// Pop a word from the RX FIFO (DR read side-effect).
    fn pop_dr(&mut self) -> u32 {
        self.rx_fifo.pop_front().unwrap_or(0)
    }

    fn sysclks_per_word(&self, clock_tree: &ClockTree) -> u64 {
        // PL022: bit rate = peri_hz / (CPSDVSR * (1 + SCR)). Per frame
        // width = (DSS+1) bits. Collapse into one clamp.
        let cpsdvsr = (self.cpsr & 0xFE).max(2) as u64; // must be even ≥ 2
        let scr = ((self.cr0 >> 8) & 0xFF) as u64;
        let peri = clock_tree.peri_hz().max(1);
        let bits_per_frame = (((self.cr0 & 0xF).max(3)) + 1) as u64;
        let denom = cpsdvsr.saturating_mul(1 + scr);
        if denom == 0 {
            return 1;
        }
        let bits_per_sec = peri / denom;
        if bits_per_sec == 0 {
            return 1;
        }
        let sys = clock_tree.sys_clk_hz.max(1) as u64;
        (sys.saturating_mul(bits_per_frame) / bits_per_sec).max(1)
    }

    // -------------------------------------------------------------------
    // Register dispatch
    // -------------------------------------------------------------------

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            SSPCR0 => self.cr0,
            SSPCR1 => self.cr1,
            SSPDR => self.pop_dr(),
            SSPSR => self.sr_read(),
            SSPCPSR => self.cpsr,
            SSPIMSC => self.imsc,
            SSPRIS => self.ris,
            SSPMIS => self.ris & self.imsc,
            SSPICR => 0,
            SSPDMACR => self.dmacr,
            SSPPERIPHID0 => PERIPH_ID[0],
            SSPPERIPHID1 => PERIPH_ID[1],
            SSPPERIPHID2 => PERIPH_ID[2],
            SSPPERIPHID3 => PERIPH_ID[3],
            SSPPCELLID0 => PCELL_ID[0],
            SSPPCELLID1 => PCELL_ID[1],
            SSPPCELLID2 => PCELL_ID[2],
            SSPPCELLID3 => PCELL_ID[3],
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u32) {
        match offset {
            SSPCR0 => {
                let mut stored = self.cr0;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.cr0 = stored & 0xFFFF;
            }
            SSPCR1 => {
                let mut stored = self.cr1;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.cr1 = stored & 0xF;
                // Disabling collapses FIFOs to empty per PL022 reset
                // semantics (real silicon holds state but firmware
                // observes post-disable reads as 0).
                if !self.is_enabled() {
                    self.tx_cycle_accum = 0;
                }
            }
            SSPDR => self.push_dr(value, irqs),
            SSPCPSR => {
                let mut stored = self.cpsr;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.cpsr = stored & 0xFE;
            }
            SSPIMSC => {
                let mut stored = self.imsc;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.imsc = stored & SSP_INT_MASK;
                self.route_irq(irqs);
            }
            SSPICR => {
                // Only RTIC + RORIC are valid ICR bits (TX/RX are level
                // and clear on drain/fill). We still honour W1C for
                // whatever bits firmware sets on ROR/RT.
                let mut clr = self.ris;
                super::apply_alias_rmw(&mut clr, value, alias);
                let mask = clr & (SSP_INT_ROR | SSP_INT_RT);
                self.ris &= !mask;
                self.route_irq(irqs);
            }
            SSPDMACR => {
                let mut stored = self.dmacr;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.dmacr = stored & 0x3;
            }
            // SR / RIS / MIS are read-only.
            _ => {}
        }
    }

    pub fn read8(&mut self, offset: u32) -> u8 {
        if offset == SSPDR {
            self.pop_dr() as u8
        } else {
            self.read32(offset) as u8
        }
    }

    pub fn read16(&mut self, offset: u32) -> u16 {
        if offset == SSPDR {
            self.pop_dr() as u16
        } else {
            self.read32(offset) as u16
        }
    }

    pub fn write8(&mut self, offset: u32, value: u8, irqs: &mut u32) {
        if offset == SSPDR {
            self.push_dr(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    pub fn write16(&mut self, offset: u32, value: u16, irqs: &mut u32) {
        if offset == SSPDR {
            self.push_dr(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    pub fn tick(&mut self, cycles: u32, clock_tree: &ClockTree, irqs: &mut u32) {
        if cycles == 0 || !self.is_enabled() || self.tx_fifo.is_empty() {
            return;
        }
        let spw = self.sysclks_per_word(clock_tree);
        let mask = self.frame_data_mask();
        let bits = self.frame_bits();
        let loopback = self.is_loopback();
        self.tx_cycle_accum = self.tx_cycle_accum.saturating_add(cycles as u64);
        while self.tx_cycle_accum >= spw && !self.tx_fifo.is_empty() {
            self.tx_cycle_accum -= spw;
            // Drain one word out of the TX FIFO. In loopback mode the
            // RX copy was pushed at `push_dr` time so no extra work
            // here.
            let word = self.tx_fifo.pop_front().unwrap_or(0);
            // Off-chip slave, if any. Loopback bypasses it electrically
            // (TX is tied to RX inside the PL022), so the device is
            // neither driven nor sampled and the RX copy is not
            // duplicated.
            if !loopback && let Some(dev) = self.device.as_mut() {
                let rx = (dev.transfer(word as u16, bits) as u32) & mask;
                if self.rx_fifo.len() < SSP_FIFO_DEPTH {
                    self.rx_fifo.push_back(rx);
                } else {
                    // PL022: a push-on-full latches the sticky overrun
                    // flag and the incoming frame is lost. Write-only
                    // streaming firmware (pico-sdk `spi_write_fast`)
                    // depends on exactly this, then clears RORIC.
                    self.ris |= SSP_INT_ROR;
                }
            }
        }
        self.refresh_tx_rx_interrupts();
        self.route_irq(irqs);
    }
}

impl Default for SpiRegs {
    fn default() -> Self {
        Self::new(18)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPI0_IRQ: u32 = 18;
    const SYS_HZ: u32 = 125_000_000;

    fn tree() -> ClockTree {
        ClockTree {
            sys_clk_hz: SYS_HZ,
            peri_clk_hz: SYS_HZ,
            ref_clk_hz: SYS_HZ,
        }
    }

    // --- reset / defaults ---------------------------------------------

    #[test]
    fn reset_defaults_all_zero() {
        let s = SpiRegs::new(SPI0_IRQ);
        assert_eq!(s.cr0, 0);
        assert_eq!(s.cr1, 0);
        assert_eq!(s.cpsr, 0);
        assert_eq!(s.imsc, 0);
    }

    #[test]
    fn sr_reports_tfe_at_reset() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let sr = s.read32(SSPSR);
        assert!(sr & SSPSR_TFE != 0);
        assert!(sr & SSPSR_TNF != 0);
        assert!(sr & SSPSR_RNE == 0);
        assert!(sr & SSPSR_BSY == 0);
    }

    // --- loopback -----------------------------------------------------

    #[test]
    fn loopback_roundtrips_byte_value() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // Enable + LBM; DSS = 7 (8-bit frames).
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write32(SSPDR, 0xA5, 0, &mut irqs);
        // RX FIFO should carry the loopback copy immediately.
        assert!(
            s.read32(SSPSR) & SSPSR_RNE != 0,
            "RX non-empty after LBM push"
        );
        let rx = s.read32(SSPDR);
        assert_eq!(rx, 0xA5);
    }

    #[test]
    fn loopback_masks_to_frame_width() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // DSS=3 → 4-bit frames: values clamp to 4 LSBs.
        s.write32(SSPCR0, 0x03, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write32(SSPDR, 0xFF, 0, &mut irqs);
        assert_eq!(s.read32(SSPDR), 0x0F);
    }

    #[test]
    fn dr_write_before_enable_is_dropped() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPDR, 0xAA, 0, &mut irqs);
        assert!(s.tx_fifo.is_empty());
        assert!(s.rx_fifo.is_empty());
    }

    // --- FIFO + SR flags ---------------------------------------------

    #[test]
    fn tx_fifo_saturates_at_eight() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        // Enable but no loopback — bytes stay queued until tick drains.
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        for i in 0..12 {
            s.write32(SSPDR, i as u32, 0, &mut irqs);
        }
        assert_eq!(s.tx_fifo.len(), SSP_FIFO_DEPTH);
        // SR: TNF clear when full.
        assert!(s.read32(SSPSR) & SSPSR_TNF == 0);
        assert!(s.read32(SSPSR) & SSPSR_BSY != 0);
    }

    // --- tick drains ---------------------------------------------------

    #[test]
    fn tick_drains_tx_at_configured_rate() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // 1 MHz bit rate: CPSDVSR=50, SCR=1 → 125MHz / (50 * 2) = 1.25 MHz.
        s.write32(SSPCR0, (1 << 8) | 0x07, 0, &mut irqs);
        s.write32(SSPCPSR, 50, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        for i in 0..4 {
            s.write32(SSPDR, i as u32, 0, &mut irqs);
        }
        assert_eq!(s.tx_fifo.len(), 4);
        let t = tree();
        s.tick(10_000, &t, &mut irqs); // 80 µs worth of cycles
        // 8 bits at ~1.25 MHz = 6.4 µs/word; 4 words ≈ 26 µs → fully
        // drained inside 80 µs.
        assert!(s.tx_fifo.is_empty());
    }

    // --- IRQ routing --------------------------------------------------

    #[test]
    fn tx_irq_latches_when_fifo_under_half() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write32(SSPIMSC, SSP_INT_TX, 0, &mut irqs);
        // Fill past half then drain via tick.
        for _ in 0..6 {
            s.write32(SSPDR, 0x11, 0, &mut irqs);
        }
        // After 6 entries, TX FIFO is above the 1/2 (4) threshold.
        // Actually, "TX IRQ" fires when level <= 1/2 = 4. 6 > 4, so
        // TXIS should currently NOT be set from refresh. Re-check
        // after drain.
        let _ = s.ris; // forces the refresh during push_dr
        s.write32(SSPCPSR, 2, 0, &mut irqs);
        // SCR=0, CPSDVSR=2 → 125MHz / 2 / 1 = 62.5 MHz → tiny sysclks/word.
        let t = tree();
        // Drain a few words.
        s.tick(1_000, &t, &mut irqs);
        assert!(s.ris & SSP_INT_TX != 0);
        assert!(irqs & (1u32 << SPI0_IRQ) != 0);
    }

    #[test]
    fn rx_irq_latches_when_fifo_half_full() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write32(SSPIMSC, SSP_INT_RX, 0, &mut irqs);
        // Push 4 words — hits RX threshold in loopback mode.
        for i in 0..4 {
            s.write32(SSPDR, i as u32, 0, &mut irqs);
        }
        assert!(s.ris & SSP_INT_RX != 0);
        assert!(irqs & (1u32 << SPI0_IRQ) != 0);
    }

    #[test]
    fn ror_ric_clears_ror() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.ris = SSP_INT_ROR | SSP_INT_RT;
        s.write32(SSPICR, SSP_INT_ROR, 0, &mut irqs);
        assert_eq!(s.ris & SSP_INT_ROR, 0);
        assert_eq!(s.ris & SSP_INT_RT, SSP_INT_RT);
    }

    // --- is_idle ------------------------------------------------------

    #[test]
    fn is_idle_true_at_reset() {
        let s = SpiRegs::new(SPI0_IRQ);
        assert!(s.is_idle());
    }

    #[test]
    fn is_idle_false_with_pending_tx() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write32(SSPDR, 0x11, 0, &mut irqs);
        assert!(!s.is_idle());
    }

    // --- Byte/halfword DR narrow dispatch ----------------------------

    #[test]
    fn byte_write_to_dr_pushes_into_tx_fifo() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write8(SSPDR, 0x73, &mut irqs);
        assert_eq!(s.rx_fifo.front().copied(), Some(0x73));
    }

    #[test]
    fn halfword_loopback_16_bit_frame() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // DSS=15 → 16-bit frames.
        s.write32(SSPCR0, 0x0F, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write16(SSPDR, 0xBEEF, &mut irqs);
        assert_eq!(s.read16(SSPDR), 0xBEEF);
    }

    // --- PrimeCell ID ------------------------------------------------

    #[test]
    fn peripheral_and_pcell_id_match_pl022() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        assert_eq!(s.read32(SSPPERIPHID0), 0x22);
        assert_eq!(s.read32(SSPPCELLID0), 0x0D);
        assert_eq!(s.read32(SSPPCELLID3), 0xB1);
    }

    // --- external device hook ----------------------------------------

    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct Probe {
        seen: Vec<(u16, u8)>,
        pins: Vec<u32>,
        reply_base: u16,
    }

    struct DummyDevice(Arc<Mutex<Probe>>);

    impl SpiExternalDevice for DummyDevice {
        fn transfer(&mut self, word: u16, bits: u8) -> u16 {
            let mut p = self.0.lock().unwrap();
            p.seen.push((word, bits));
            p.reply_base.wrapping_add(word)
        }
        fn observe_pins(&mut self, levels: u32) {
            self.0.lock().unwrap().pins.push(levels);
        }
    }

    /// Enabled, 8-bit frames, fast baud so one `tick` drains everything.
    fn armed(loopback: bool) -> (SpiRegs, u32) {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCPSR, 2, 0, &mut irqs);
        let cr1 = if loopback {
            SSPCR1_SSE | SSPCR1_LBM
        } else {
            SSPCR1_SSE
        };
        s.write32(SSPCR1, cr1, 0, &mut irqs);
        (s, irqs)
    }

    #[test]
    fn attached_device_receives_tx_words_and_drives_rx() {
        let probe = Arc::new(Mutex::new(Probe {
            reply_base: 0x10,
            ..Default::default()
        }));
        let (mut s, mut irqs) = armed(false);
        s.attach_device(Box::new(DummyDevice(probe.clone())));
        assert!(s.has_device());
        s.write32(SSPDR, 0x5A, 0, &mut irqs);
        s.write32(SSPDR, 0x01, 0, &mut irqs);
        // Nothing reaches the device until the FIFO drains.
        assert!(probe.lock().unwrap().seen.is_empty());
        s.tick(10_000, &tree(), &mut irqs);
        let seen = probe.lock().unwrap().seen.clone();
        assert_eq!(seen, vec![(0x5A, 8), (0x01, 8)]);
        // Returned words land in the RX FIFO, in order.
        assert_eq!(s.read32(SSPDR), 0x6A);
        assert_eq!(s.read32(SSPDR), 0x11);
    }

    #[test]
    fn device_frame_width_follows_cr0_dss() {
        let probe = Arc::new(Mutex::new(Probe::default()));
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // DSS = 15 → 16-bit frames.
        s.write32(SSPCR0, 0x0F, 0, &mut irqs);
        s.write32(SSPCPSR, 2, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.attach_device(Box::new(DummyDevice(probe.clone())));
        s.write16(SSPDR, 0xBEEF, &mut irqs);
        s.tick(10_000, &tree(), &mut irqs);
        assert_eq!(probe.lock().unwrap().seen, vec![(0xBEEF, 16)]);
    }

    #[test]
    fn observe_pins_reaches_the_device_and_is_a_noop_when_detached() {
        let probe = Arc::new(Mutex::new(Probe::default()));
        let (mut s, _) = armed(false);
        // Detached: must not panic, must not record anything.
        s.observe_pins(0xDEAD_BEEF);
        assert!(probe.lock().unwrap().pins.is_empty());
        s.attach_device(Box::new(DummyDevice(probe.clone())));
        s.observe_pins(0x0000_E000);
        assert_eq!(probe.lock().unwrap().pins, vec![0x0000_E000]);
    }

    #[test]
    fn loopback_bypasses_the_device_and_does_not_double_push_rx() {
        let probe = Arc::new(Mutex::new(Probe {
            reply_base: 0x10,
            ..Default::default()
        }));
        let (mut s, mut irqs) = armed(true);
        s.attach_device(Box::new(DummyDevice(probe.clone())));
        s.write32(SSPDR, 0x5A, 0, &mut irqs);
        s.tick(10_000, &tree(), &mut irqs);
        // Device never driven; RX holds exactly one word, the LBM copy.
        assert!(probe.lock().unwrap().seen.is_empty());
        assert_eq!(s.rx_fifo.len(), 1);
        assert_eq!(s.read32(SSPDR), 0x5A);
    }

    #[test]
    fn detached_tick_still_discards_tx_and_leaves_rx_empty() {
        let (mut s, mut irqs) = armed(false);
        for i in 0..4u32 {
            s.write32(SSPDR, i, 0, &mut irqs);
        }
        s.tick(10_000, &tree(), &mut irqs);
        assert!(s.tx_fifo.is_empty());
        assert!(s.rx_fifo.is_empty(), "no device ⇒ no RX response");
        assert_eq!(s.ris & SSP_INT_ROR, 0);
    }

    #[test]
    fn device_rx_overrun_latches_ror_and_drops_the_frame() {
        let probe = Arc::new(Mutex::new(Probe::default()));
        let (mut s, mut irqs) = armed(false);
        s.attach_device(Box::new(DummyDevice(probe.clone())));
        // 8-deep TX FIFO → 8 responses into an 8-deep RX FIFO fills it;
        // a second burst overruns.
        for i in 0..8u32 {
            s.write32(SSPDR, i, 0, &mut irqs);
        }
        s.tick(10_000, &tree(), &mut irqs);
        assert_eq!(s.rx_fifo.len(), SSP_FIFO_DEPTH);
        assert_eq!(s.ris & SSP_INT_ROR, 0);
        s.write32(SSPDR, 0xFF, 0, &mut irqs);
        s.tick(10_000, &tree(), &mut irqs);
        assert_eq!(s.rx_fifo.len(), SSP_FIFO_DEPTH);
        assert_ne!(s.ris & SSP_INT_ROR, 0, "push-on-full latches ROR");
    }

    #[test]
    fn reset_keeps_the_soldered_device_attached() {
        let probe = Arc::new(Mutex::new(Probe::default()));
        let (mut s, mut irqs) = armed(false);
        s.attach_device(Box::new(DummyDevice(probe.clone())));
        s.reset();
        assert!(s.has_device());
        // And it still works after re-arming.
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCPSR, 2, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write32(SSPDR, 0x22, 0, &mut irqs);
        s.tick(10_000, &tree(), &mut irqs);
        assert_eq!(probe.lock().unwrap().seen, vec![(0x22, 8)]);
    }
}
