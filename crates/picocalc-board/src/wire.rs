//! Glue between `rp2040-emu`'s generic SPI hook and [`St7365p`].
//!
//! The model is shared through an `Arc<Mutex<_>>` because the emulator
//! takes ownership of the boxed device, but the runner still needs to
//! read the framebuffer and the observation counters out of it while the
//! run is in flight.

use std::sync::{Arc, Mutex};

use rp2040_emu::peripherals::spi::SpiExternalDevice;

use crate::pins::{PIN_CS, PIN_DC, PIN_RESET, level};
use crate::st7365p::St7365p;

/// Wire-level adapter: maps GPIO pad levels onto the panel's CS / DC /
/// RESET inputs and forwards SPI frames as bytes.
pub struct St7365pWire {
    lcd: Arc<Mutex<St7365p>>,
}

impl St7365pWire {
    pub fn new(lcd: Arc<Mutex<St7365p>>) -> Self {
        Self { lcd }
    }

    /// Shared handle to the model behind this wire.
    pub fn model(&self) -> Arc<Mutex<St7365p>> {
        self.lcd.clone()
    }
}

impl SpiExternalDevice for St7365pWire {
    fn transfer(&mut self, word: u16, _bits: u8) -> u16 {
        // The firmware runs the panel with 8-bit frames throughout
        // (`spi_init` leaves `SSPCR0.DSS` at 7 and every transfer goes
        // through byte-wide `spi_write_blocking` / `spi_write_fast`), so
        // the low byte is the whole frame. `bits` is ignored rather than
        // asserted on: a wider frame would be a firmware change, not an
        // emulator fault, and panicking mid-run would destroy the report.
        self.lcd
            .lock()
            .expect("LCD mutex")
            .transfer_byte(word as u8) as u16
    }

    fn observe_pins(&mut self, gpio_out_levels: u32) {
        let cs = level(gpio_out_levels, PIN_CS);
        let dc = level(gpio_out_levels, PIN_DC);
        let reset = level(gpio_out_levels, PIN_RESET);
        self.lcd
            .lock()
            .expect("LCD mutex")
            .set_control_lines(cs, dc, reset);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::st7365p::{CMD_DISPON, CMD_MADCTL};

    fn levels(cs: bool, dc: bool, reset: bool) -> u32 {
        ((cs as u32) << PIN_CS) | ((dc as u32) << PIN_DC) | ((reset as u32) << PIN_RESET)
    }

    #[test]
    fn pad_levels_map_onto_cs_dc_reset() {
        let lcd = Arc::new(Mutex::new(St7365p::new()));
        let mut wire = St7365pWire::new(lcd.clone());
        // Release reset, select the panel, command mode.
        wire.observe_pins(levels(true, false, false));
        wire.observe_pins(levels(false, false, true));
        assert_eq!(lcd.lock().unwrap().reset_pulses, 1);
        wire.transfer(CMD_DISPON as u16, 8);
        assert!(lcd.lock().unwrap().display_on);

        // Data mode carries the parameter.
        wire.transfer(CMD_MADCTL as u16, 8);
        wire.observe_pins(levels(false, true, true));
        wire.transfer(0x48, 8);
        assert_eq!(lcd.lock().unwrap().madctl, 0x48);
    }

    #[test]
    fn deselected_frames_are_dropped() {
        let lcd = Arc::new(Mutex::new(St7365p::new()));
        let mut wire = St7365pWire::new(lcd.clone());
        wire.observe_pins(levels(true, false, false));
        wire.observe_pins(levels(true, false, true)); // CS high
        wire.transfer(CMD_DISPON as u16, 8);
        assert!(!lcd.lock().unwrap().display_on);
    }

    #[test]
    fn only_the_low_byte_of_a_frame_reaches_the_panel() {
        let lcd = Arc::new(Mutex::new(St7365p::new()));
        let mut wire = St7365pWire::new(lcd.clone());
        wire.observe_pins(levels(true, false, false));
        wire.observe_pins(levels(false, false, true));
        assert_eq!(wire.transfer(0xFF00 | CMD_DISPON as u16, 16), 0);
        assert!(lcd.lock().unwrap().display_on);
    }
}
