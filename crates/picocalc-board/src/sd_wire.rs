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
        if !cs && self.cs_high {
            self.card.lock().expect("SD mutex").trace_select();
        }
        // Only the rising edge matters: that is what ends a command.
        if cs && !self.cs_high {
            self.card.lock().expect("SD mutex").deselect();
        }
        self.cs_high = cs;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn command_with_crc(wire: &mut SdCardWire, index: u8, arg: u32, crc: u8) -> Vec<u8> {
        wire.transfer(0x40 | index as u16, 8);
        for shift in [24, 16, 8, 0] {
            wire.transfer(((arg >> shift) & 0xFF) as u16, 8);
        }
        wire.transfer(crc as u16, 8);
        (0..8).map(|_| wire.transfer(0xFF, 8) as u8).collect()
    }

    fn command(wire: &mut SdCardWire, index: u8, arg: u32) -> Vec<u8> {
        let crc = match index {
            0 => 0x95,
            8 => 0x87,
            _ => 0x01, // dummy end-bit CRC while general CRC is disabled
        };
        command_with_crc(wire, index, arg, crc)
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
    fn send_if_cond_rejects_bad_crc_without_r7() {
        let (mut w, _card) = wire();
        let reply = command_with_crc(&mut w, 8, 0x0000_01AA, 0x85);
        let start = reply.iter().position(|b| b & 0x80 == 0).unwrap();
        assert_eq!(reply[start], 0x09, "idle plus COM_CRC_ERROR");
        assert!(
            reply[start + 1..].iter().all(|byte| *byte == 0xFF),
            "CRC error must not carry an R7 payload: {reply:02x?}"
        );
    }

    #[test]
    fn go_idle_state_rejects_bad_crc_and_accepts_a_retry() {
        let (mut w, _card) = wire();
        assert_eq!(
            r1(&command_with_crc(&mut w, 0, 0, 0x97)),
            0x09,
            "idle plus COM_CRC_ERROR"
        );
        assert_eq!(r1(&command(&mut w, 0, 0)), 0x01);
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
    fn single_block_read_returns_r1_before_data_token() {
        let (mut w, _card) = wire();
        let reply = command(&mut w, 17, 0);

        // One idle byte may precede the response, but the command response
        // must be visible before the 0xFE data token and sector payload.
        assert_eq!(&reply[..3], &[0xFF, 0x00, 0xFE]);
        // The compact FAT16 fixture's boot sector begins with the standard
        // short jump over its BPB.
        assert_eq!(reply[3], 0xEB);
    }

    #[test]
    fn diagnostic_trace_is_opt_in_and_records_command_boundaries() {
        let (mut w, card) = wire();
        assert!(card.lock().unwrap().trace_snapshot().is_none());
        card.lock().unwrap().enable_trace();

        let _ = command(&mut w, 17, 3);
        w.observe_pins(0);
        w.observe_pins(1 << SD_PIN_CS);

        let snapshot = card.lock().unwrap().trace_snapshot().unwrap();
        assert_eq!(
            snapshot.schema_version,
            crate::sdcard::SD_TRACE_SCHEMA_VERSION
        );
        assert!(snapshot.event_count >= 2, "command plus deselect");
        assert!(!snapshot.digest_sha256.is_empty());
        assert!(snapshot.preview.iter().any(|event| matches!(
            event,
            crate::sdcard::SdTraceEvent::Command {
                index: 17,
                argument: 3,
                data: Some(crate::sdcard::SdTraceData {
                    direction: crate::sdcard::SdTraceDirection::Read,
                    block: 3,
                    length: crate::sdcard::BLOCK_SIZE,
                    ..
                }),
                ..
            }
        )));
        assert!(
            snapshot
                .preview
                .iter()
                .any(|event| matches!(event, crate::sdcard::SdTraceEvent::Deselect { .. }))
        );
    }

    #[test]
    fn diagnostic_trace_does_not_change_card_replies_or_counters() {
        let (mut traced, traced_card) = wire();
        let (mut plain, plain_card) = wire();
        traced_card.lock().unwrap().enable_trace();

        for (index, argument) in [(0, 0), (8, 0x1AA), (17, 2), (24, 3)] {
            let traced_reply = command(&mut traced, index, argument);
            let plain_reply = command(&mut plain, index, argument);
            assert_eq!(traced_reply, plain_reply, "CMD{index} reply");
        }
        let traced = traced_card.lock().unwrap();
        let plain = plain_card.lock().unwrap();
        assert_eq!(traced.commands_seen, plain.commands_seen);
        assert_eq!(traced.blocks_read, plain.blocks_read);
        assert_eq!(traced.blocks_written, plain.blocks_written);
        assert_eq!(traced.unknown_commands, plain.unknown_commands);
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
        w.observe_pins(0); // select the card before starting the frame
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

    #[cfg(feature = "sd-gen1-multiblock")]
    fn send_frame(wire: &mut SdCardWire, index: u8, arg: u32) {
        wire.transfer(0x40 | index as u16, 8);
        for shift in [24, 16, 8, 0] {
            wire.transfer(((arg >> shift) & 0xFF) as u16, 8);
        }
        wire.transfer(0x01, 8);
    }

    #[cfg(feature = "sd-gen1-multiblock")]
    fn drain_block(wire: &mut SdCardWire) -> Vec<u8> {
        let token_seen = (0..32).find_map(|_| {
            let byte = wire.transfer(0xFF, 8) as u8;
            (byte == 0xFE).then_some(byte)
        });
        assert_eq!(token_seen, Some(0xFE), "expected a data token");
        (0..crate::sdcard::BLOCK_SIZE + 2)
            .map(|_| wire.transfer(0xFF, 8) as u8)
            .collect()
    }

    #[cfg(feature = "sd-gen1-multiblock")]
    fn write_multi_block(wire: &mut SdCardWire, block: u32, value: u8) {
        assert_eq!(wire.transfer(0xFC, 8) as u8, 0xFF);
        for _ in 0..crate::sdcard::BLOCK_SIZE {
            wire.transfer(value as u16, 8);
        }
        wire.transfer(0xFF, 8);
        wire.transfer(0xFF, 8);
        assert_eq!(wire.transfer(0xFF, 8) as u8, 0x05, "block {block} accepted");
        assert_eq!(wire.transfer(0xFF, 8) as u8, 0x00, "block {block} busy");
    }

    #[cfg(feature = "sd-gen1-multiblock")]
    #[test]
    fn synthetic_cmd18_streams_two_blocks_and_cmd12_stops() {
        let (mut w, card) = wire();
        send_frame(&mut w, 18, 3);
        let first = drain_block(&mut w);
        assert_eq!(first.len(), crate::sdcard::BLOCK_SIZE + 2);

        // The host clocks one idle byte to request the next block while CS
        // remains low; the card starts the next token on the following byte.
        assert_eq!(w.transfer(0xFF, 8) as u8, 0xFF);
        let second = drain_block(&mut w);
        assert_eq!(second.len(), crate::sdcard::BLOCK_SIZE + 2);

        // CMD12 is framed without a CS pulse after the final block.
        send_frame(&mut w, 12, 0);
        let stop_reply: Vec<_> = (0..8).map(|_| w.transfer(0xFF, 8) as u8).collect();
        assert_eq!(r1(&stop_reply), 0x00);
        assert!(card.lock().unwrap().protocol_errors.is_empty());
    }

    #[cfg(feature = "sd-gen1-multiblock")]
    #[test]
    fn synthetic_multi_read_trace_replays_command_and_block_boundaries() {
        let (mut w, card) = wire();
        card.lock().unwrap().enable_trace();
        w.observe_pins(0);
        send_frame(&mut w, 18, 3);
        let _ = drain_block(&mut w);
        let _ = w.transfer(0xFF, 8);
        let _ = drain_block(&mut w);
        send_frame(&mut w, 12, 0);
        let _ = (0..8).map(|_| w.transfer(0xFF, 8) as u8).collect::<Vec<_>>();
        w.observe_pins(1 << SD_PIN_CS);

        let snapshot = card.lock().unwrap().trace_snapshot().unwrap();
        let command_indices = snapshot
            .preview
            .iter()
            .filter_map(|event| match event {
                crate::sdcard::SdTraceEvent::Command { index, .. } => Some(*index),
                _ => None,
            })
            .collect::<Vec<_>>();
        let block_events = snapshot
            .preview
            .iter()
            .filter(|event| matches!(event, crate::sdcard::SdTraceEvent::BlockData { .. }))
            .count();
        assert_eq!(command_indices, vec![18, 12]);
        assert_eq!(block_events, 1, "first block is attached to CMD18");
        assert_eq!(snapshot.event_count, 4, "CMD18 + block + CMD12 + deselect");
    }

    #[cfg(feature = "sd-gen1-multiblock")]
    #[test]
    fn synthetic_cmd23_cmd25_writes_two_blocks_with_busy_between_them() {
        let (mut w, card) = wire();
        send_frame(&mut w, 23, 2);
        let _ = (0..8).map(|_| w.transfer(0xFF, 8) as u8).collect::<Vec<_>>();
        send_frame(&mut w, 25, 3);
        let _ = (0..8).map(|_| w.transfer(0xFF, 8) as u8).collect::<Vec<_>>();

        write_multi_block(&mut w, 3, 0xA5);
        write_multi_block(&mut w, 4, 0x5A);
        assert_eq!(w.transfer(0xFD, 8) as u8, 0xFF, "stop token is consumed");
        assert_eq!(card.lock().unwrap().blocks_written, 2);

        // Read one written block back through the existing single-block
        // path; this also proves that the multi-block commit used the same
        // backing/COW boundary as CMD24.
        send_frame(&mut w, 17, 3);
        let data = drain_block(&mut w);
        assert!(data[..crate::sdcard::BLOCK_SIZE]
            .iter()
            .all(|byte| *byte == 0xA5));
    }

    #[cfg(feature = "sd-gen1-multiblock")]
    #[test]
    fn multi_block_mutations_are_visible_and_fail_closed() {
        let (mut w, card) = wire();

        // Out-of-range CMD18 returns no token and records the reason.
        send_frame(&mut w, 18, 64);
        let reply: Vec<_> = (0..16).map(|_| w.transfer(0xFF, 8) as u8).collect();
        assert!(!reply.contains(&0xFE));
        assert!(card
            .lock()
            .unwrap()
            .protocol_errors
            .iter()
            .any(|error| error == "multi_read_block_out_of_range_64"));

        // A single-block token is not silently accepted for CMD25.
        send_frame(&mut w, 25, 3);
        let _ = (0..8).map(|_| w.transfer(0xFF, 8) as u8).collect::<Vec<_>>();
        assert_eq!(w.transfer(0xFE, 8) as u8, 0xFF);
        assert!(card
            .lock()
            .unwrap()
            .protocol_errors
            .iter()
            .any(|error| error == "multi_write_expected_fc_or_fd_got_fe"));
    }

    #[cfg(feature = "sd-gen1-multiblock")]
    #[test]
    fn cs_abort_discards_a_multi_block_transfer() {
        let (mut w, card) = wire();
        send_frame(&mut w, 18, 3);
        let _ = drain_block(&mut w);
        w.observe_pins(1 << SD_PIN_CS);
        w.observe_pins(0);

        send_frame(&mut w, 17, 0);
        let data = drain_block(&mut w);
        assert_eq!(data.len(), crate::sdcard::BLOCK_SIZE + 2);
        assert!(card.lock().unwrap().protocol_errors.is_empty());
    }
}
