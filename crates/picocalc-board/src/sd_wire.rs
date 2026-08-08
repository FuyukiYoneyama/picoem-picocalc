//! Glue between `rp2040-emu`'s generic SPI hook and [`SdCard`].
//!
//! The card sits on SPI0 with CS on GP17, driven by the CPU rather than
//! by the controller's own slave-select. Releasing CS ends whatever
//! exchange was in flight, which is how the driver recovers between
//! commands, so the wire watches that pin and tells the card.
//!
//! Card detect (GP22) is an input to the chip rather than an output, so
//! it is driven from the runner rather than here; see
//! `Bus::external_gpio_in_override`.

use std::sync::{Arc, Mutex};

use rp2040_emu::peripherals::spi::SpiExternalDevice;

use crate::pins::{SD_PIN_CS, level};
use crate::sdcard::SdCard;

/// Wire-level adapter for the SD card.
pub struct SdCardWire {
    card: Arc<Mutex<SdCard>>,
    /// Last observed CS level, for edge detection.
    cs_high: bool,
}

impl SdCardWire {
    pub fn new(card: Arc<Mutex<SdCard>>) -> Self {
        Self {
            card,
            // Idle state is deselected.
            cs_high: true,
        }
    }

    /// Shared handle to the model behind this wire.
    pub fn model(&self) -> Arc<Mutex<SdCard>> {
        self.card.clone()
    }
}

impl SpiExternalDevice for SdCardWire {
    fn transfer(&mut self, word: u16, _bits: u8) -> u16 {
        // The driver sets 8-bit frames and never changes them. Same
        // reasoning as the panel wire: a wider frame would be a firmware
        // change, not an emulator fault.
        self.card.lock().expect("SD mutex").transfer(word as u8) as u16
    }

    fn observe_pins(&mut self, gpio_out_levels: u32) {
        let cs = level(gpio_out_levels, SD_PIN_CS);
        // Only the rising edge matters: that is what ends a command.
        if cs && !self.cs_high {
            self.card.lock().expect("SD mutex").deselect();
        }
        self.cs_high = cs;
    }

    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn supports_constant_pin_bulk(&self) -> bool {
        true
    }

    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn observe_constant_pins(&mut self, gpio_out_levels: u32, repetitions: u32) {
        if repetitions == 0 {
            return;
        }
        self.observe_pins(gpio_out_levels);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn pin_level(cs: bool) -> u32 {
        (cs as u32) << SD_PIN_CS
    }

    fn wire() -> (SdCardWire, Arc<Mutex<SdCard>>) {
        // Wire-protocol tests need only a few blocks; use the compact
        // FAT16 profile because a valid FAT32 volume needs >=65,525 clusters.
        let card = Arc::new(Mutex::new(SdCard::new_with_format(
            64,
            crate::sdcard::SdFormat::Fat16,
        )));
        (SdCardWire::new(Arc::clone(&card)), card)
    }

    /// Send a command frame and collect the reply bytes the driver would
    /// see while polling.
    fn command(wire: &mut SdCardWire, index: u8, arg: u32) -> Vec<u8> {
        wire.transfer(0x40 | index as u16, 8);
        for shift in [24, 16, 8, 0] {
            wire.transfer(((arg >> shift) & 0xFF) as u16, 8);
        }
        wire.transfer(0x95, 8); // CRC, ignored in SPI mode
        (0..8).map(|_| wire.transfer(0xFF, 8) as u8).collect()
    }

    /// First byte with bit 7 clear is the R1 response.
    fn r1(reply: &[u8]) -> u8 {
        reply
            .iter()
            .copied()
            .find(|b| b & 0x80 == 0)
            .unwrap_or(0xFF)
    }

    #[test]
    fn go_idle_state_reports_idle() {
        let (mut w, _card) = wire();
        assert_eq!(r1(&command(&mut w, 0, 0)), 0x01);
    }

    #[test]
    fn send_if_cond_echoes_the_check_pattern() {
        let (mut w, _card) = wire();
        // The driver sends 0x1AA and requires the last two response
        // bytes to come back as 0x01, 0xAA.
        let reply = command(&mut w, 8, 0x0000_01AA);
        let start = reply.iter().position(|b| b & 0x80 == 0).unwrap();
        assert_eq!(reply[start], 0x01, "R1 idle");
        assert_eq!(reply[start + 3], 0x01, "voltage nibble");
        assert_eq!(reply[start + 4], 0xAA, "check pattern");
    }

    #[test]
    fn acmd41_reports_busy_then_ready() {
        let (mut w, _card) = wire();
        // Each ACMD41 is preceded by CMD55.
        let mut answers = Vec::new();
        for _ in 0..4 {
            let _ = command(&mut w, 55, 0);
            answers.push(r1(&command(&mut w, 41, 0x4000_0000)));
        }
        assert_eq!(answers[0], 0x01, "busy at first");
        assert!(
            answers.contains(&0x00),
            "must become ready, got {answers:?}"
        );
    }

    #[test]
    fn read_ocr_marks_the_card_high_capacity() {
        let (mut w, _card) = wire();
        let reply = command(&mut w, 58, 0);
        let start = reply.iter().position(|b| b & 0x80 == 0).unwrap();
        assert_eq!(reply[start], 0x00, "R1 ready");
        // The driver tests bit 6 of the first OCR byte for SDHC.
        assert_ne!(reply[start + 1] & 0x40, 0, "CCS must be set");
    }

    #[test]
    fn a_written_block_reads_back_unchanged() {
        let (mut w, card) = wire();
        // CMD24: write block 3.
        let _ = command(&mut w, 24, 3);
        w.transfer(0xFE, 8); // start token
        for i in 0..512u32 {
            w.transfer((i & 0xFF) as u16, 8);
        }
        // Two CRC bytes, then the response — the order write_data_block
        // uses.
        w.transfer(0xFF, 8);
        w.transfer(0xFF, 8);
        let accepted = w.transfer(0xFF, 8) as u8;
        assert_eq!(accepted & 0x1F, 0x05, "data accepted token");
        assert_eq!(card.lock().unwrap().blocks_written, 1);

        // CS pulse between commands, as the driver does.
        w.observe_pins(1 << SD_PIN_CS);
        w.observe_pins(0);

        // CMD17: read it back.
        w.transfer(0x40 | 17, 8);
        for shift in [24, 16, 8, 0] {
            w.transfer(((3u32 >> shift) & 0xFF) as u16, 8);
        }
        w.transfer(0x95, 8);
        // Poll past the R1 byte to the start token, then take the block.
        let mut seen_token = false;
        let mut data = Vec::new();
        for _ in 0..600 {
            let b = w.transfer(0xFF, 8) as u8;
            if !seen_token {
                if b == 0xFE {
                    seen_token = true;
                }
                continue;
            }
            data.push(b);
            if data.len() == 512 {
                break;
            }
        }
        assert!(seen_token, "card must send a start token");
        assert_eq!(data.len(), 512);
        for (i, b) in data.iter().enumerate() {
            assert_eq!(*b, (i & 0xFF) as u8, "byte {i}");
        }
    }

    #[test]
    fn releasing_cs_abandons_a_partial_command() {
        let (mut w, _card) = wire();
        w.transfer(0x40, 8); // CMD0, then interrupted
        w.observe_pins(1 << SD_PIN_CS);
        w.observe_pins(0);
        // A fresh command still decodes.
        assert_eq!(r1(&command(&mut w, 0, 0)), 0x01);
    }

    #[test]
    fn unknown_commands_are_counted() {
        let (mut w, card) = wire();
        let _ = command(&mut w, 62, 0);
        assert_eq!(card.lock().unwrap().unknown_commands, vec![(62, 1)]);
    }

    #[test]
    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn observe_constant_pins_is_one_observe_when_supported() {
        let (mut w, card) = wire();
        w.observe_pins(pin_level(false)); // select
        {
            let device = &mut w as &mut dyn SpiExternalDevice;
            assert!(device.supports_constant_pin_bulk());
            device.observe_constant_pins(pin_level(true), 3);
        }
        assert_eq!(card.lock().unwrap().commands_seen, 0);
        {
            let device = &mut w as &mut dyn SpiExternalDevice;
            device.observe_constant_pins(pin_level(false), 0);
        }
        assert_eq!(card.lock().unwrap().commands_seen, 0);
    }

    #[test]
    #[cfg(feature = "stationary-pin-bulk-prototype")]
    fn observe_constant_observe_control_edge_matches_reference() {
        let (mut wire_ref, card_ref) = wire();
        let (mut wire_bulk, card_bulk) = wire();

        wire_ref.observe_pins(pin_level(false));
        wire_bulk.observe_pins(pin_level(false));

        // Start a command in progress, then raise CS while keeping it
        // raised for subsequent repeated observe calls.
        wire_ref.transfer(0x40, 8);
        wire_bulk.transfer(0x40, 8);

        wire_ref.observe_pins(pin_level(true));
        wire_ref.observe_pins(pin_level(true));
        wire_ref.observe_pins(pin_level(true));

        {
            let bulk = &mut wire_bulk as &mut dyn SpiExternalDevice;
            bulk.observe_constant_pins(pin_level(true), 3);
        }

        let ref_r1 = r1(&command(&mut wire_ref, 0, 0));
        let bulk_r1 = r1(&command(&mut wire_bulk, 0, 0));
        assert_eq!(ref_r1, bulk_r1);
        assert_eq!(
            card_ref.lock().unwrap().commands_seen,
            card_bulk.lock().unwrap().commands_seen
        );
    }
}
