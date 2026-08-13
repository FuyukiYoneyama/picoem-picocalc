//! Deterministic SPI-NOR flash behind XIP_SSI.
//!
//! The XIP window is backed by [`Memory`](crate::memory::Memory).  This
//! device models the small command subset used by the RP2040 SDK's flash
//! helpers and emits mutations for `Bus` to apply to that XIP window.  The
//! separation is intentional: the flash command parser owns wire protocol
//! state while the bus owns the executable image and cache invalidation.

use std::collections::VecDeque;

/// Bytes of unique ID a `0x4B` command returns.
pub const UNIQUE_ID_LEN: usize = 8;

const CMD_READ: u8 = 0x03;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_READ_UNIQUE_ID: u8 = 0x4B;
const CMD_JEDEC_ID: u8 = 0x9F;
const CMD_READ_STATUS1: u8 = 0x05;
const CMD_READ_STATUS2: u8 = 0x35;
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_WRITE_DISABLE: u8 = 0x04;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_QUAD_IO_READ: u8 = 0xEB;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_BLOCK_ERASE: u8 = 0xD8;
const CMD_CHIP_ERASE: u8 = 0xC7;
const CMD_CHIP_ERASE_ALT: u8 = 0x60;

const UNIQUE_ID_DUMMY: usize = 4;

/// A completed flash operation waiting for the bus to apply it to XIP.
#[derive(Debug, PartialEq, Eq)]
pub enum FlashMutation {
    Erase { offset: u32, len: u32 },
    Program { offset: u32, data: Vec<u8> },
}

/// SPI flash chip as seen through XIP_SSI.
pub struct SsiFlash {
    /// Reported by `0x4B`. Fixed so runs stay reproducible.
    pub unique_id: [u8; UNIQUE_ID_LEN],
    /// Reported by `0x9F`: manufacturer, type, capacity.
    pub jedec_id: [u8; 3],
    rx: VecDeque<u8>,
    command: Option<u8>,
    /// Bytes seen since the opcode.
    index: usize,
    address: u32,
    program_data: Vec<u8>,
    write_enable_latch: bool,
    write_in_progress: bool,
    mutations: VecDeque<FlashMutation>,
    /// Protocol violations are kept as deterministic diagnostics.  The
    /// runner can fail a mutation gate when this list is non-empty.
    pub errors: Vec<String>,
    pub erase_count: u64,
    pub program_count: u64,
    pub program_bytes: u64,
    /// Opcodes this model does not answer, with their counts.
    pub unknown_commands: Vec<(u8, u32)>,
    /// All command opcodes observed, with counts. This is intentionally
    /// bounded to opcode counts rather than retaining raw transactions, so
    /// a long firmware run cannot turn diagnostics into an unbounded trace.
    pub command_counts: Vec<(u8, u32)>,
}

impl Default for SsiFlash {
    fn default() -> Self {
        Self::new()
    }
}

impl SsiFlash {
    pub fn new() -> Self {
        Self {
            unique_id: [0xE6, 0x60, 0x58, 0x38, 0x93, 0x0D, 0x2A, 0x11],
            jedec_id: [0xEF, 0x40, 0x15],
            rx: VecDeque::new(),
            command: None,
            index: 0,
            address: 0,
            program_data: Vec::new(),
            write_enable_latch: false,
            write_in_progress: false,
            mutations: VecDeque::new(),
            errors: Vec::new(),
            erase_count: 0,
            program_count: 0,
            program_bytes: 0,
            unknown_commands: Vec::new(),
            command_counts: Vec::new(),
        }
    }

    /// Chip select released, or the controller disabled.  Mutating
    /// commands commit at CS release, matching SPI-NOR command framing.
    pub fn end_transaction(&mut self) {
        self.finish_mutating_command();
        self.command = None;
        self.index = 0;
        self.address = 0;
        self.program_data.clear();
        self.rx.clear();
    }

    pub fn has_rx(&self) -> bool {
        !self.rx.is_empty()
    }
    pub fn rx_len(&self) -> u32 {
        self.rx.len() as u32
    }
    pub fn pop_rx(&mut self) -> u8 {
        self.rx.pop_front().unwrap_or(0)
    }

    /// Drain completed erase/program operations.
    pub fn take_mutations(&mut self) -> Vec<FlashMutation> {
        self.mutations.drain(..).collect()
    }

    /// One byte clocked out by the controller.  The reply is queued for the
    /// matching read, as full-duplex SPI would deliver it.
    pub fn push_tx(&mut self, byte: u8) {
        let reply = match self.command {
            None => {
                self.command = Some(byte);
                self.note_command(byte);
                self.index = 0;
                self.address = 0;
                self.program_data.clear();
                match byte {
                    0x00 | 0xFF | CMD_READ | CMD_FAST_READ | CMD_READ_UNIQUE_ID | CMD_JEDEC_ID
                    | CMD_READ_STATUS1 | CMD_READ_STATUS2 | CMD_WRITE_ENABLE
                    | CMD_WRITE_DISABLE | CMD_PAGE_PROGRAM | CMD_QUAD_IO_READ
                    | CMD_SECTOR_ERASE | CMD_BLOCK_ERASE | CMD_CHIP_ERASE | CMD_CHIP_ERASE_ALT => {}
                    other => self.note_unknown(other),
                }
                0
            }
            Some(command) => {
                let position = self.index;
                self.index = self.index.saturating_add(1);
                match command {
                    CMD_READ_UNIQUE_ID => {
                        if position < UNIQUE_ID_DUMMY {
                            0
                        } else {
                            self.unique_id
                                .get(position - UNIQUE_ID_DUMMY)
                                .copied()
                                .unwrap_or(0)
                        }
                    }
                    CMD_JEDEC_ID => self.jedec_id.get(position).copied().unwrap_or(0),
                    CMD_READ_STATUS1 => {
                        (self.write_in_progress as u8) | ((self.write_enable_latch as u8) << 1)
                    }
                    CMD_READ_STATUS2 => 0,
                    CMD_READ | CMD_FAST_READ => {
                        if position < 3 {
                            self.address = (self.address << 8) | u32::from(byte);
                            0
                        } else if command == CMD_FAST_READ && position == 3 {
                            0
                        } else {
                            // XIP reads are normally used for execution; the
                            // SSI read path is provided for boot helpers and
                            // returns a deterministic placeholder here.  A
                            // future bus-backed read hook can replace it.
                            0
                        }
                    }
                    CMD_QUAD_IO_READ => {
                        // RP2040 boot2 re-enters XIP with 0xEB quad-I/O
                        // reads. The executable XIP window is already the
                        // authoritative data source, but the command still
                        // needs correct address/mode/dummy framing so the
                        // CS transaction is consumed without a false
                        // unsupported-command diagnostic.
                        if position < 3 {
                            self.address = (self.address << 8) | u32::from(byte);
                        }
                        0
                    }
                    CMD_PAGE_PROGRAM => {
                        if position < 3 {
                            self.address = (self.address << 8) | u32::from(byte);
                        } else {
                            self.program_data.push(byte);
                        }
                        0
                    }
                    CMD_SECTOR_ERASE | CMD_BLOCK_ERASE => {
                        if position < 3 {
                            self.address = (self.address << 8) | u32::from(byte);
                        }
                        0
                    }
                    CMD_CHIP_ERASE | CMD_WRITE_ENABLE | CMD_WRITE_DISABLE => 0,
                    _ => 0,
                }
            }
        };
        self.rx.push_back(reply);
    }

    fn finish_mutating_command(&mut self) {
        let Some(command) = self.command else { return };
        match command {
            CMD_WRITE_ENABLE => self.write_enable_latch = true,
            CMD_WRITE_DISABLE => self.write_enable_latch = false,
            CMD_PAGE_PROGRAM => {
                if !self.write_enable_latch {
                    self.errors.push("page_program_without_wren".into());
                } else if self.program_data.is_empty() {
                    self.errors.push("page_program_without_data".into());
                } else if self.program_data.len() > 256
                    || (self.address & 0xFF) + self.program_data.len() as u32 > 256
                {
                    self.errors
                        .push("page_program_crosses_256_byte_page".into());
                } else {
                    self.write_in_progress = true;
                    let data = std::mem::take(&mut self.program_data);
                    self.program_count += 1;
                    self.program_bytes += data.len() as u64;
                    self.mutations.push_back(FlashMutation::Program {
                        offset: self.address,
                        data,
                    });
                    self.write_in_progress = false;
                    self.write_enable_latch = false;
                }
            }
            CMD_SECTOR_ERASE => self.finish_erase(0x1000),
            CMD_BLOCK_ERASE => self.finish_erase(0x1_0000),
            CMD_CHIP_ERASE | CMD_CHIP_ERASE_ALT => {
                if !self.write_enable_latch {
                    self.errors.push("chip_erase_without_wren".into());
                } else {
                    self.write_in_progress = true;
                    self.mutations
                        .push_back(FlashMutation::Erase { offset: 0, len: 0 });
                    self.erase_count += 1;
                    self.write_in_progress = false;
                    self.write_enable_latch = false;
                }
            }
            _ => {}
        }
    }

    fn finish_erase(&mut self, len: u32) {
        if !self.write_enable_latch {
            self.errors.push("erase_without_wren".into());
            return;
        }
        if self.address % len != 0 {
            self.errors.push("erase_address_not_aligned".into());
            return;
        }
        self.write_in_progress = true;
        self.mutations.push_back(FlashMutation::Erase {
            offset: self.address,
            len,
        });
        self.erase_count += 1;
        self.write_in_progress = false;
        self.write_enable_latch = false;
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

    fn note_command(&mut self, code: u8) {
        if let Some(entry) = self
            .command_counts
            .iter_mut()
            .find(|(existing, _)| *existing == code)
        {
            entry.1 = entry.1.saturating_add(1);
        } else {
            self.command_counts.push((code, 1));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn transfer(flash: &mut SsiFlash, bytes: &[u8]) -> Vec<u8> {
        bytes
            .iter()
            .map(|&b| {
                flash.push_tx(b);
                flash.pop_rx()
            })
            .collect()
    }

    #[test]
    fn read_unique_id_matches_sdk_frame() {
        let mut flash = SsiFlash::new();
        let reply = transfer(
            &mut flash,
            &[CMD_READ_UNIQUE_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        );
        assert_eq!(&reply[5..13], &flash.unique_id[..]);
    }

    #[test]
    fn jedec_and_status_are_deterministic() {
        let mut flash = SsiFlash::new();
        let reply = transfer(&mut flash, &[CMD_JEDEC_ID, 0, 0, 0]);
        assert_eq!(&reply[1..4], &[0xEF, 0x40, 0x15]);
        flash.end_transaction();
        let _ = transfer(&mut flash, &[CMD_WRITE_ENABLE]);
        flash.end_transaction();
        let status = transfer(&mut flash, &[CMD_READ_STATUS1, 0]);
        assert_eq!(status[1] & 0x02, 0x02);
    }

    #[test]
    fn wren_program_emits_page_mutation() {
        let mut flash = SsiFlash::new();
        let _ = transfer(&mut flash, &[CMD_WRITE_ENABLE]);
        flash.end_transaction();
        let mut frame = vec![CMD_PAGE_PROGRAM, 0, 1, 0];
        frame.extend_from_slice(&[0xAA, 0x55]);
        let _ = transfer(&mut flash, &frame);
        flash.end_transaction();
        assert_eq!(
            flash.take_mutations(),
            vec![FlashMutation::Program {
                offset: 0x100,
                data: vec![0xAA, 0x55]
            }]
        );
        assert!(flash.errors.is_empty());
    }

    #[test]
    fn erase_requires_wren_and_alignment() {
        let mut flash = SsiFlash::new();
        let _ = transfer(&mut flash, &[CMD_SECTOR_ERASE, 0, 0, 1]);
        flash.end_transaction();
        assert_eq!(flash.errors, vec!["erase_without_wren"]);
        flash.errors.clear();
        let _ = transfer(&mut flash, &[CMD_WRITE_ENABLE]);
        flash.end_transaction();
        let _ = transfer(&mut flash, &[CMD_SECTOR_ERASE, 0, 0, 1]);
        flash.end_transaction();
        assert_eq!(flash.errors, vec!["erase_address_not_aligned"]);
    }

    #[test]
    fn boot2_quad_read_and_dummy_frames_are_known_commands() {
        let mut flash = SsiFlash::new();
        let _ = transfer(&mut flash, &[CMD_QUAD_IO_READ, 0, 1, 2, 0, 0, 0, 0, 0]);
        flash.end_transaction();
        let _ = transfer(&mut flash, &[0x00, 0xFF]);
        flash.end_transaction();
        assert!(flash.errors.is_empty());
        assert!(flash.unknown_commands.is_empty());
        assert_eq!(
            flash
                .command_counts
                .iter()
                .find(|(command, _)| *command == CMD_QUAD_IO_READ)
                .map(|(_, count)| *count),
            Some(1)
        );
    }
}
