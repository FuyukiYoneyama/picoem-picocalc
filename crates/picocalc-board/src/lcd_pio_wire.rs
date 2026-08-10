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
    /// True after an actual pad-level SCK edge selected this transport.
    /// Variant A also has a frame-level SPI wire, so CS alone is not
    /// sufficient to decide which reply timing is active.
    bit_level_active: bool,

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
        Self {
            panel,
            last: 0,
            primed: false,
            shift: 0,
            bit_count: 0,
            miso_shift: 0,
            miso_level: false,
            bit_level_active: false,
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
            if self.bit_level_active {
                // Frame-level SPI replies are returned in the same
                // transfer and therefore need the model's explicit
                // RAMRD dummy again after SIO/PIO bit traffic ends.
                self.panel
                    .lock()
                    .expect("panel mutex")
                    .set_ramrd_dummy(true);
                self.bit_level_active = false;
            }
            if self.bit_count != 0 {
                self.partial_bytes += 1;
                self.bit_count = 0;
                self.shift = 0;
            }
            return None;
        }

        let sck_was = Self::level(previous, pins::PIN_SCK);
        let sck_now = Self::level(pads, pins::PIN_SCK);

        if sck_was != sck_now && !self.bit_level_active {
            // A bit-level full-duplex reply is inherently one byte
            // behind the request, so it already supplies the physical
            // RAMRD dummy byte.  Activate this only after a real pad
            // edge: variant A's normal SPI1 frames share CS/DC with this
            // observer but do not synthesize pad-level SCK edges.
            self.panel
                .lock()
                .expect("panel mutex")
                .set_ramrd_dummy(false);
            self.bit_level_active = true;
        }

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

    /// Transfer one byte with the historical SIO observer timing: sample
    /// MISO while SCK is low, latch MOSI on the rising edge, and finish low.
    fn exchange(wire: &mut LcdPioWire, base: u32, byte: u8) -> u8 {
        let mut reply = 0;
        for i in (0..8).rev() {
            let bit = if byte & (1 << i) != 0 { MOSI } else { 0 };
            let sample = wire
                .tick(base | bit)
                .map(|(_, level)| level)
                .unwrap_or(false);
            reply = (reply << 1) | u8::from(sample);
            wire.tick(base | bit | SCK);
        }
        wire.tick(base);
        reply
    }

    fn paint_one_rgb666_pixel(panel: &Arc<Mutex<St7365p>>, rgb: [u8; 3]) {
        let mut panel = panel.lock().unwrap();
        for (command, data) in [
            (crate::st7365p::CMD_CASET, &[0, 0, 0, 0][..]),
            (crate::st7365p::CMD_RASET, &[0, 0, 0, 0][..]),
        ] {
            panel.set_control_lines(false, false, true);
            panel.transfer_byte(command);
            panel.set_control_lines(false, true, true);
            for byte in data {
                panel.transfer_byte(*byte);
            }
        }
        panel.set_control_lines(false, false, true);
        panel.transfer_byte(crate::st7365p::CMD_RAMWR);
        panel.set_control_lines(false, true, true);
        for byte in rgb {
            panel.transfer_byte(byte);
        }
        panel.set_control_lines(true, true, true);
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
    fn sio_ramrd_returns_rgb666_in_rgb_order_after_one_wire_dummy() {
        let (mut wire, panel) = wire();
        paint_one_rgb666_pixel(&panel, [0xF8, 0x40, 0x08]);

        let command_reply = exchange(&mut wire, RESET, crate::st7365p::CMD_RAMRD);
        let dummy = exchange(&mut wire, RESET | DC, 0);
        let red = exchange(&mut wire, RESET | DC, 0);
        let green = exchange(&mut wire, RESET | DC, 0);
        let blue = exchange(&mut wire, RESET | DC, 0);

        assert_eq!(command_reply, 0);
        assert_eq!(dummy, 0);
        assert_eq!([red, green, blue], [0xF8, 0x40, 0x08]);
    }

    #[test]
    fn deselect_restores_same_transfer_spi_dummy_timing() {
        let (mut wire, panel) = wire();
        paint_one_rgb666_pixel(&panel, [0xF8, 0x40, 0x08]);

        let _ = exchange(&mut wire, RESET, crate::st7365p::CMD_RAMRD);
        let _ = exchange(&mut wire, RESET | DC, 0);
        wire.tick(IDLE);

        let mut panel = panel.lock().unwrap();
        panel.set_control_lines(false, false, true);
        panel.transfer_byte(crate::st7365p::CMD_RAMRD);
        panel.set_control_lines(false, true, true);
        assert_eq!(panel.transfer_byte(0), 0, "SPI path keeps explicit dummy");
        assert_eq!(panel.transfer_byte(0), 0xF8, "pixel follows the dummy");
    }
}
