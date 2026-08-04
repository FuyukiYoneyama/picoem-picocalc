//! Minimal SPI flash behind XIP_SSI, for commands the boot path issues.
//!
//! The emulator maps the flash image directly into the XIP window and
//! does not model QSPI pads, so `Bus` normally treats XIP_SSI as a set
//! of scratch registers. That is enough right up until firmware asks
//! the flash chip a question rather than reading from it.
//!
//! `pico_unique_id` does exactly that at startup: its constructor runs
//! `flash_get_unique_id`, which drops out of XIP, clocks a Read Unique
//! ID (`0x4B`) command through SSI, and reads eight bytes back. With
//! the register stub the receive FIFO never fills, so the bootrom
//! helper waits forever — firmware built from the standard template
//! hung before reaching `main`, while the official sample (which does
//! not link `pico_unique_id`) ran fine.
//!
//! What is modelled is the command/response shape, not the wire. Each
//! byte written to `DR0` produces one response byte, matching the
//! full-duplex SSI the boot helpers drive with `SSIENR` toggles around
//! each transaction.
//!
//! | Command | Response |
//! |---------|----------|
//! | `0x4B` Read Unique ID | 4 dummy bytes, then 8 ID bytes |
//! | `0x9F` JEDEC ID | 3 ID bytes |
//! | `0x05` Read Status 1 | `0x00` — never busy, nothing to wait for |
//! | `0x35` Read Status 2 | `0x00` |
//! | anything else | zeros, and the opcode is counted |
//!
//! Erase and program are deliberately absent: nothing in the
//! conformance track writes flash, and a half-modelled program path
//! would corrupt the XIP image rather than fail visibly.

use std::collections::VecDeque;

/// Bytes of unique ID a `0x4B` command returns.
pub const UNIQUE_ID_LEN: usize = 8;

const CMD_READ_UNIQUE_ID: u8 = 0x4B;
const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_READ_STATUS1: u8 = 0x05;
const CMD_READ_STATUS2: u8 = 0x35;

/// Dummy bytes between the `0x4B` opcode and the ID itself.
const UNIQUE_ID_DUMMY: usize = 4;

/// SPI flash chip as seen through XIP_SSI.
pub struct SsiFlash {
    /// Reported by `0x4B`. Fixed so runs stay reproducible; the real
    /// value is per-chip and no conformance target depends on it.
    pub unique_id: [u8; UNIQUE_ID_LEN],
    /// Reported by `0x9F`: manufacturer, type, capacity.
    pub jedec_id: [u8; 3],
    rx: VecDeque<u8>,
    /// Opcode of the transaction in flight, if any.
    command: Option<u8>,
    /// Bytes seen since the opcode.
    index: usize,
    /// Opcodes this model does not answer, with their counts. Kept so a
    /// hang caused by an unmodelled command is diagnosable rather than
    /// silent.
    pub unknown_commands: Vec<(u8, u32)>,
}

impl Default for SsiFlash {
    fn default() -> Self {
        Self::new()
    }
}

impl SsiFlash {
    pub fn new() -> Self {
        Self {
            // Deterministic stand-in, distinguishable from all-zero or
            // all-ones so firmware that prints it shows something real.
            unique_id: [0xE6, 0x60, 0x58, 0x38, 0x93, 0x0D, 0x2A, 0x11],
            // W25Q16JV (Winbond, 2 MB) — the part the Pico ships with.
            jedec_id: [0xEF, 0x40, 0x15],
            rx: VecDeque::new(),
            command: None,
            index: 0,
            unknown_commands: Vec::new(),
        }
    }

    /// Chip select released, or the controller disabled: whatever was in
    /// flight is abandoned.
    pub fn end_transaction(&mut self) {
        self.command = None;
        self.index = 0;
        self.rx.clear();
    }

    /// True while a response byte is waiting to be read.
    pub fn has_rx(&self) -> bool {
        !self.rx.is_empty()
    }

    /// Bytes waiting in the receive FIFO, for `RXFLR`. The bootrom's
    /// transfer loop reads the FIFO level registers rather than the
    /// status flags, so this has to track the queue.
    pub fn rx_len(&self) -> u32 {
        self.rx.len() as u32
    }

    /// Read one response byte; zero if the firmware over-reads.
    pub fn pop_rx(&mut self) -> u8 {
        self.rx.pop_front().unwrap_or(0)
    }

    /// One byte clocked out by the controller. The reply is queued for
    /// the matching read, as full-duplex SPI would deliver it.
    pub fn push_tx(&mut self, byte: u8) {
        let reply = match self.command {
            None => {
                self.command = Some(byte);
                self.index = 0;
                match byte {
                    CMD_READ_UNIQUE_ID | CMD_JEDEC_ID | CMD_READ_STATUS1 | CMD_READ_STATUS2 => {}
                    other => self.note_unknown(other),
                }
                // The opcode byte itself reads back as zero.
                0
            }
            Some(command) => {
                let position = self.index;
                self.index += 1;
                match command {
                    CMD_READ_UNIQUE_ID => {
                        if position < UNIQUE_ID_DUMMY {
                            0
                        } else {
                            let i = position - UNIQUE_ID_DUMMY;
                            self.unique_id.get(i).copied().unwrap_or(0)
                        }
                    }
                    CMD_JEDEC_ID => self.jedec_id.get(position).copied().unwrap_or(0),
                    // Status registers: not busy, not write-enabled.
                    CMD_READ_STATUS1 | CMD_READ_STATUS2 => 0,
                    _ => 0,
                }
            }
        };
        self.rx.push_back(reply);
    }

    fn note_unknown(&mut self, code: u8) {
        if let Some(entry) = self
            .unknown_commands
            .iter_mut()
            .find(|(existing, _)| *existing == code)
        {
            entry.1 = entry.1.saturating_add(1);
        } else {
            self.unknown_commands.push((code, 1));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Clock `bytes` through and collect the replies, the way
    /// `flash_do_cmd` drives a full-duplex transfer.
    fn transfer(flash: &mut SsiFlash, bytes: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        for &b in bytes {
            flash.push_tx(b);
            out.push(flash.pop_rx());
        }
        out
    }

    #[test]
    fn read_unique_id_matches_the_sdk_frame() {
        let mut flash = SsiFlash::new();
        // pico_unique_id sends 1 opcode + 4 dummy + 8 data bytes.
        let reply = transfer(&mut flash, &[CMD_READ_UNIQUE_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(reply.len(), 13);
        assert_eq!(&reply[5..13], &flash.unique_id[..]);
    }

    #[test]
    fn dummy_bytes_before_the_id_read_as_zero() {
        let mut flash = SsiFlash::new();
        let reply = transfer(&mut flash, &[CMD_READ_UNIQUE_ID, 0, 0, 0, 0]);
        assert_eq!(&reply[..5], &[0, 0, 0, 0, 0]);
    }

    #[test]
    fn jedec_id_reports_the_pinned_part() {
        let mut flash = SsiFlash::new();
        let reply = transfer(&mut flash, &[CMD_JEDEC_ID, 0, 0, 0]);
        assert_eq!(&reply[1..4], &[0xEF, 0x40, 0x15]);
    }

    #[test]
    fn status_reads_are_never_busy() {
        let mut flash = SsiFlash::new();
        let reply = transfer(&mut flash, &[CMD_READ_STATUS1, 0]);
        assert_eq!(reply[1] & 0x01, 0, "BUSY must stay clear or firmware spins");
    }

    #[test]
    fn a_new_transaction_starts_a_new_command() {
        let mut flash = SsiFlash::new();
        let _ = transfer(&mut flash, &[CMD_READ_UNIQUE_ID, 0, 0]);
        flash.end_transaction();
        let reply = transfer(&mut flash, &[CMD_JEDEC_ID, 0]);
        assert_eq!(reply[1], 0xEF, "second command must not continue the first");
    }

    #[test]
    fn unknown_opcodes_are_counted_not_swallowed() {
        let mut flash = SsiFlash::new();
        let _ = transfer(&mut flash, &[0x77, 0, 0]);
        assert_eq!(flash.unknown_commands, vec![(0x77, 1)]);
    }

    #[test]
    fn over_reading_past_the_id_yields_zero() {
        let mut flash = SsiFlash::new();
        let reply = transfer(&mut flash, &[CMD_READ_UNIQUE_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(&reply[13..], &[0, 0]);
    }
}
