//! Variant B display wiring: the panel driven from PIO0 rather than SPI1.
//!
//! The Canonical BSP's default display path does not use the SSP at all.
//! `bsp/vendor/lcd_spi_min.pio` is a two-instruction shift engine —
//!
//! ```text
//! .wrap_target
//!     out pins, 1    side 0   ; SCK low, present one MOSI bit
//!     nop            side 1   ; SCK high
//! .wrap
//! ```
//!
//! — so bytes never pass through a controller FIFO the emulator could
//! tap. The only place the traffic is visible is the pads, which is what
//! [`PinWatchingDevice`] exists for.
//!
//! Wire contract, from `bsp/vendor/lcd_rgb565_pio.cpp` and the PIO
//! program's own header:
//!
//! | Signal | Pin | Driven by |
//! |--------|-----|-----------|
//! | SCK    | 10  | PIO0 side-set |
//! | MOSI   | 11  | PIO0 `out pins` |
//! | CS     | 13  | CPU (SIO) |
//! | DC     | 14  | CPU (SIO) |
//! | RESET  | 15  | CPU (SIO) |
//!
//! Mode 0, MSB first: the master presents a bit while SCK is low and the
//! panel samples it on the rising edge. Bytes are eight such edges. This
//! is the same panel and the same command set as variant A — only the
//! transport differs — so the decoded bytes go to the same [`St7365p`]
//! model, and the transfer paths are kept apart exactly as the plan
//! requires.
//!
//! Pixel format differs from A: `COLMOD=0x65`, two bytes per pixel in
//! RGB565, against A's three-byte RGB666 container. That difference
//! lives in the display model, selected by the `COLMOD` the firmware
//! writes, not here.

use std::sync::{Arc, Mutex};

use rp2040_emu::bus::PinWatchingDevice;

use crate::pins;
use crate::st7365p::St7365p;

/// Shifts pad-level traffic back into bytes and feeds the panel model.
pub struct LcdPioWire {
    panel: Arc<Mutex<St7365p>>,
    /// Previous pad snapshot, for edge detection.
    last: u32,
    /// True once the first snapshot has been seen.
    primed: bool,
    /// Bits accumulated for the byte in flight, MSB first.
    shift: u8,
    /// How many bits of `shift` are valid.
    bit_count: u8,
    /// Byte the panel handed back, being shifted out on MISO.
    miso_shift: u8,
    /// Level currently presented on MISO.
    miso_level: bool,

    // --- observation counters ---
    /// Rising SCK edges seen while CS was asserted.
    pub sck_edges: u64,
    /// Bytes handed to the panel.
    pub bytes: u64,
    /// Bits dropped because CS rose mid-byte.
    pub partial_bytes: u64,
}

impl LcdPioWire {
    pub fn new(panel: Arc<Mutex<St7365p>>) -> Self {
        // This transport is bit-level full duplex, so a reply is
        // already one byte behind the request; the panel must not also
        // count a dummy byte. See .
        panel.lock().expect("panel mutex").set_ramrd_dummy(false);
        Self {
            panel,
            last: 0,
            primed: false,
            shift: 0,
            bit_count: 0,
            miso_shift: 0,
            miso_level: false,
            sck_edges: 0,
            bytes: 0,
            partial_bytes: 0,
        }
    }

    fn level(pads: u32, pin: u8) -> bool {
        pads & (1 << pin) != 0
    }
}

impl PinWatchingDevice for LcdPioWire {
    fn tick(&mut self, pads: u32) -> Option<(u8, bool)> {
        let previous = self.last;
        self.last = pads;
        if !self.primed {
            self.primed = true;
            return None;
        }

        // Control lines are CPU-driven and move between transfers. The
        // panel model takes raw pin levels and applies the active-low
        // sense itself, the same way variant A's wire delivers them.
        let cs_high = Self::level(pads, pins::PIN_CS);
        {
            let mut panel = self.panel.lock().expect("panel mutex");
            panel.set_control_lines(
                cs_high,
                Self::level(pads, pins::PIN_DC),
                Self::level(pads, pins::PIN_RESET),
            );
        }

        // CS high ends any partial byte: the bits that were in flight
        // are not a byte the panel ever saw.
        if cs_high {
            if self.bit_count != 0 {
                self.partial_bytes += 1;
                self.bit_count = 0;
                self.shift = 0;
            }
            return None;
        }

        let sck_was = Self::level(previous, pins::PIN_SCK);
        let sck_now = Self::level(pads, pins::PIN_SCK);

        // Falling edge: present the next MISO bit, so it is stable by
        // the time the master samples on the rising edge. Reads happen
        // when firmware parks the PIO program and bit-bangs these same
        // pins from SIO, which is how `scroll_lcd_spi` and the GRAM
        // readback in the BSP smoke test work.
        if sck_was && !sck_now {
            self.miso_level = self.miso_shift & 0x80 != 0;
            self.miso_shift <<= 1;
        }

        if !(!sck_was && sck_now) {
            return Some((pins::PIN_MISO, self.miso_level));
        }

        // Rising edge with CS asserted: the panel latches MOSI.
        self.sck_edges += 1;
        let bit = Self::level(pads, pins::PIN_MOSI);
        self.shift = (self.shift << 1) | bit as u8;
        self.bit_count += 1;
        if self.bit_count == 8 {
            let byte = self.shift;
            self.shift = 0;
            self.bit_count = 0;
            self.bytes += 1;
            let mut panel = self.panel.lock().expect("panel mutex");
            // Full duplex: what the panel returns for this byte goes out
            // on MISO during the next one.
            self.miso_shift = panel.transfer_byte(byte);
        }
        Some((pins::PIN_MISO, self.miso_level))
    }

    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn supports_constant_pin_bulk(&self) -> bool {
        true
    }

    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn tick_constant_pins(&mut self, gpio_out: u32, repetitions: u32) -> Option<(u8, bool)> {
        if repetitions == 0 {
            return None;
        }
        self.tick(gpio_out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SCK: u32 = 1 << pins::PIN_SCK;
    const MOSI: u32 = 1 << pins::PIN_MOSI;
    const CS: u32 = 1 << pins::PIN_CS;
    const DC: u32 = 1 << pins::PIN_DC;
    const RESET: u32 = 1 << pins::PIN_RESET;

    /// Idle pads: CS high (deselected), RESET high (not in reset).
    const IDLE: u32 = CS | RESET;

    fn wire() -> (LcdPioWire, Arc<Mutex<St7365p>>) {
        let panel = Arc::new(Mutex::new(St7365p::new()));
        let mut w = LcdPioWire::new(Arc::clone(&panel));
        // Prime, then take the panel out of reset.
        w.tick(IDLE);
        w.tick(IDLE);
        (w, panel)
    }

    /// Clock one byte in MSB first, the way the PIO program does: bit
    /// presented with SCK low, latched on the rising edge.
    fn send(wire: &mut LcdPioWire, base: u32, byte: u8) {
        for i in (0..8).rev() {
            let bit = if byte & (1 << i) != 0 { MOSI } else { 0 };
            wire.tick(base | bit);
            wire.tick(base | bit | SCK);
        }
    }

    #[test]
    fn a_command_byte_reaches_the_panel() {
        let (mut w, panel) = wire();
        // CS low, DC low = command.
        send(&mut w, RESET, crate::st7365p::CMD_SLPOUT);
        assert_eq!(w.bytes, 1);
        assert_eq!(panel.lock().unwrap().slpout_count, 1);
    }

    #[test]
    fn bits_arrive_most_significant_first() {
        let (mut w, panel) = wire();
        send(&mut w, RESET, crate::st7365p::CMD_COLMOD);
        // DC high = parameter.
        send(&mut w, RESET | DC, 0x65);
        assert_eq!(panel.lock().unwrap().colmod_reg, 0x65);
    }

    #[test]
    fn nothing_is_latched_while_cs_is_high() {
        let (mut w, _panel) = wire();
        // Same waveform, but deselected.
        send(&mut w, IDLE, 0xFF);
        assert_eq!(w.sck_edges, 0);
        assert_eq!(w.bytes, 0);
    }

    #[test]
    fn falling_edges_do_not_latch() {
        let (mut w, _panel) = wire();
        // Drive SCK high then low repeatedly without the rising pattern
        // starting from low: only genuine rising edges count.
        w.tick(RESET | SCK);
        w.tick(RESET);
        w.tick(RESET | SCK);
        assert_eq!(w.sck_edges, 2, "one edge per low-to-high transition");
    }

    #[test]
    fn a_partial_byte_is_discarded_when_cs_rises() {
        let (mut w, panel) = wire();
        // Four bits, then deselect.
        for _ in 0..4 {
            w.tick(RESET | MOSI);
            w.tick(RESET | MOSI | SCK);
        }
        w.tick(IDLE);
        assert_eq!(w.partial_bytes, 1);
        assert_eq!(w.bytes, 0);
        // The next full byte still decodes correctly.
        send(&mut w, RESET, crate::st7365p::CMD_DISPON);
        assert_eq!(panel.lock().unwrap().dispon_count, 1);
    }

    #[test]
    fn reset_is_passed_through_active_low() {
        let (mut w, panel) = wire();
        // RESET pin low means the panel is held in reset.
        w.tick(CS);
        assert!(panel.lock().unwrap().in_reset());
        w.tick(IDLE);
        assert!(!panel.lock().unwrap().in_reset());
    }

    #[test]
    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn tick_constant_ticks_once_after_priming() {
        let (mut w_ref, panel_ref) = wire();
        let (mut w_bulk, panel_bulk) = wire();

        let bulk = &mut w_bulk as &mut dyn PinWatchingDevice;
        assert!(bulk.supports_constant_pin_bulk());
        assert_eq!(w_ref.tick(IDLE), bulk.tick_constant_pins(IDLE, 1));
        send(&mut w_ref, RESET, crate::st7365p::CMD_DISPON);
        send(&mut w_bulk, RESET, crate::st7365p::CMD_DISPON);
        assert_eq!(
            panel_ref.lock().unwrap().dispon_count,
            panel_bulk.lock().unwrap().dispon_count
        );
    }

    #[test]
    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn tick_constant_matches_reference_for_cs_rising_after_partial_byte() {
        let (mut w_ref, panel_ref) = wire();
        let (mut w_bulk, panel_bulk) = wire();

        // Four bits into a partial byte with CS asserted.
        for _ in 0..4 {
            w_ref.tick(RESET | MOSI);
            w_ref.tick(RESET | MOSI | SCK);

            w_bulk.tick(RESET | MOSI);
            w_bulk.tick(RESET | MOSI | SCK);
        }

        let mut ref_out = None;
        for _ in 0..3 {
            ref_out = w_ref.tick(IDLE);
        }
        let bulk = &mut w_bulk as &mut dyn PinWatchingDevice;
        let bulk_out = bulk.tick_constant_pins(IDLE, 3);

        assert_eq!(bulk_out, ref_out);
        assert_eq!(w_ref.partial_bytes, w_bulk.partial_bytes);
        assert_eq!(w_ref.bytes, w_bulk.bytes);
        assert_eq!(
            panel_ref.lock().unwrap().dispon_count,
            panel_bulk.lock().unwrap().dispon_count
        );
    }

    #[test]
    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn tick_constant_falling_then_repeated_rising_sck_sample_matches_reference() {
        let (mut w_ref, _panel_ref) = wire();
        let (mut w_bulk, _panel_bulk) = wire();

        let pre = RESET | SCK;
        w_ref.tick(pre);
        w_bulk.tick(pre);

        let mut ref_out = None;
        for _ in 0..3 {
            ref_out = w_ref.tick(RESET);
        }
        let bulk = &mut w_bulk as &mut dyn PinWatchingDevice;
        let bulk_out = bulk.tick_constant_pins(RESET, 3);
        assert_eq!(bulk_out, ref_out);
    }
}
