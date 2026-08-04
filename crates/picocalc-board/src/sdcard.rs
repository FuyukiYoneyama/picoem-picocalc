//! SD card in SPI mode, as the PicoCalc's slot presents it.
//!
//! The Canonical BSP talks to the card over SPI0 (GP16 MISO, GP17 CS,
//! GP18 SCK, GP19 MOSI), with GP22 as card-detect. It runs the standard
//! SPI-mode bring-up — CMD0, CMD8, CMD55+ACMD41 until ready, CMD58 to
//! read OCR — then switches from 400 kHz to 12 MHz and does block I/O
//! with CMD17 and CMD24.
//!
//! # What is modelled
//!
//! A high-capacity (SDHC) card whose blocks live in memory. That is
//! enough for the BSP's smoke test, which mounts a FAT volume, writes a
//! file, syncs, reads it back, compares and deletes it. Storage starts
//! zeroed; FatFs formats it on first mount if there is no filesystem, so
//! nothing needs a prepared image.
//!
//! Command framing follows the driver's own shape: six bytes (`0x40 |
//! index`, four argument bytes, CRC), then the card holds MISO high for
//! a few bytes before answering. R1 is one byte; R3 and R7 add four
//! trailing bytes. Data blocks arrive after a `0xFE` token and end with
//! two CRC bytes the model accepts and ignores, because SPI mode leaves
//! CRC off by default and the driver never turns it on.
//!
//! # What is not modelled
//!
//! No card-removal or write-protect behaviour, no CSD/CID registers
//! beyond what bring-up reads, no multi-block transfer (CMD18/CMD25),
//! and no busy timing: writes complete immediately rather than holding
//! MISO low for a programming delay. Nothing in the conformance track
//! exercises those, and a half-modelled busy state would be harder to
//! reason about than its absence.

/// Bytes per block. SDHC addresses in blocks, not bytes.
pub const BLOCK_SIZE: usize = 512;

/// Default capacity: 64 MiB. Large enough for a FAT volume with room to
/// work in, small enough to allocate without thought.
pub const DEFAULT_BLOCKS: usize = (64 << 20) / BLOCK_SIZE;

/// Idle byte. The card leaves MISO high when it has nothing to say.
const IDLE: u8 = 0xFF;
/// R1 bit 0: the card is in idle state (still initialising).
const R1_IDLE: u8 = 0x01;
/// R1 with no bits set: ready.
const R1_READY: u8 = 0x00;
/// Start token for a single block, both directions.
const TOKEN_START: u8 = 0xFE;
/// Data-response token meaning the write was accepted.
const DATA_ACCEPTED: u8 = 0x05;

/// Where the card is in a command exchange.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Phase {
    /// Waiting for the first byte of a command frame.
    Idle,
    /// Collecting the five bytes that follow the command index.
    Command { index: u8, taken: u8 },
    /// Streaming a queued reply out.
    Reply,
    /// Waiting for the start token of a block the host is writing.
    AwaitWriteToken,
    /// Taking the 512 data bytes plus two CRC bytes.
    WriteData { taken: usize },
    /// Data taken; the acceptance token goes out on the next transfer.
    DataResponse,
}

/// SD card in SPI mode.
pub struct SdCard {
    blocks: Vec<u8>,
    phase: Phase,
    /// Argument bytes of the command being assembled.
    arg: [u8; 4],
    /// Bytes queued to go out on MISO, in order.
    reply: std::collections::VecDeque<u8>,
    /// True once CMD55 has been seen, so the next command is an ACMD.
    app_cmd_pending: bool,
    /// True once ACMD41 has reported ready.
    initialised: bool,
    /// How many ACMD41 polls to answer "still busy" before reporting
    /// ready. Real cards need a few; answering instantly would leave the
    /// driver's retry loop untested.
    acmd41_busy_left: u8,
    /// Block address of the write in flight.
    write_block: u32,
    /// True between a WRITE_BLOCK reply and its data phase.
    write_pending: bool,
    /// Buffer for the block being written.
    write_buf: Vec<u8>,

    // --- observation counters ---
    pub commands_seen: u64,
    pub blocks_read: u64,
    pub blocks_written: u64,
    /// Commands this model does not implement, with their counts.
    pub unknown_commands: Vec<(u8, u32)>,
}

impl Default for SdCard {
    fn default() -> Self {
        Self::new(DEFAULT_BLOCKS)
    }
}

impl SdCard {
    pub fn new(block_count: usize) -> Self {
        let mut card = Self {
            blocks: vec![0u8; block_count * BLOCK_SIZE],
            phase: Phase::Idle,
            arg: [0; 4],
            reply: std::collections::VecDeque::new(),
            app_cmd_pending: false,
            initialised: false,
            // Two busy answers, so the driver's poll loop runs at least
            // one extra iteration.
            acmd41_busy_left: 2,
            write_block: 0,
            write_pending: false,
            write_buf: Vec::with_capacity(BLOCK_SIZE),
            commands_seen: 0,
            blocks_read: 0,
            blocks_written: 0,
            unknown_commands: Vec::new(),
        };
        card.format_fat16();
        card
    }

    /// Lay down an empty FAT16 volume.
    ///
    /// The BSP mounts the card and expects a filesystem to be there; it
    /// never formats. A real card ships formatted, so the model does the
    /// same rather than making every test carry a prepared image.
    ///
    /// FAT16 rather than FAT32 because the geometry is simpler to get
    /// right and FatFs mounts either. The layout is the textbook one:
    /// one reserved sector holding the boot record, two copies of the
    /// allocation table, a fixed-size root directory, then data.
    fn format_fat16(&mut self) {
        const BYTES_PER_SECTOR: usize = 512;
        const SECTORS_PER_CLUSTER: u8 = 4;
        const RESERVED_SECTORS: u16 = 1;
        const NUM_FATS: u8 = 2;
        const ROOT_ENTRIES: u16 = 512;

        let total_sectors = self.block_count();
        let root_sectors = (ROOT_ENTRIES as usize * 32) / BYTES_PER_SECTOR;

        // Size the allocation table from the data area it has to
        // describe. One pass is enough here: the table is small relative
        // to the volume, so folding its own size back in does not push
        // the cluster count across a boundary.
        let usable = total_sectors - RESERVED_SECTORS as usize - root_sectors;
        let approx_clusters = usable / SECTORS_PER_CLUSTER as usize;
        let fat_sectors =
            ((approx_clusters + 2) * 2).div_ceil(BYTES_PER_SECTOR);

        let boot = &mut self.blocks[0..BYTES_PER_SECTOR];
        boot.fill(0);
        // Jump over the BPB, as a bootable volume would.
        boot[0..3].copy_from_slice(&[0xEB, 0x3C, 0x90]);
        boot[3..11].copy_from_slice(b"MSWIN4.1");
        boot[11..13].copy_from_slice(&(BYTES_PER_SECTOR as u16).to_le_bytes());
        boot[13] = SECTORS_PER_CLUSTER;
        boot[14..16].copy_from_slice(&RESERVED_SECTORS.to_le_bytes());
        boot[16] = NUM_FATS;
        boot[17..19].copy_from_slice(&ROOT_ENTRIES.to_le_bytes());
        // Sector count goes in the 32-bit field when it does not fit in
        // sixteen bits, which is the case for any card worth modelling.
        if total_sectors < 0x1_0000 {
            boot[19..21].copy_from_slice(&(total_sectors as u16).to_le_bytes());
        } else {
            boot[32..36].copy_from_slice(&(total_sectors as u32).to_le_bytes());
        }
        boot[21] = 0xF8; // fixed disk
        boot[22..24].copy_from_slice(&(fat_sectors as u16).to_le_bytes());
        boot[24..26].copy_from_slice(&63u16.to_le_bytes()); // sectors per track
        boot[26..28].copy_from_slice(&255u16.to_le_bytes()); // heads
        boot[36] = 0x80; // drive number
        boot[38] = 0x29; // extended boot signature
        boot[39..43].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        boot[43..54].copy_from_slice(b"PICOCALC   ");
        boot[54..62].copy_from_slice(b"FAT16   ");
        boot[510] = 0x55;
        boot[511] = 0xAA;

        // Both allocation tables start with the media descriptor in
        // entry 0 and an end-of-chain marker in entry 1; every data
        // cluster is free.
        for copy in 0..NUM_FATS as usize {
            let start = (RESERVED_SECTORS as usize + copy * fat_sectors) * BYTES_PER_SECTOR;
            let end = start + fat_sectors * BYTES_PER_SECTOR;
            self.blocks[start..end].fill(0);
            self.blocks[start..start + 2].copy_from_slice(&0xFFF8u16.to_le_bytes());
            self.blocks[start + 2..start + 4].copy_from_slice(&0xFFFFu16.to_le_bytes());
        }

        // Empty root directory.
        let root_start = (RESERVED_SECTORS as usize
            + NUM_FATS as usize * fat_sectors)
            * BYTES_PER_SECTOR;
        let root_end = root_start + root_sectors * BYTES_PER_SECTOR;
        self.blocks[root_start..root_end].fill(0);
    }

    /// Capacity in blocks.
    pub fn block_count(&self) -> usize {
        self.blocks.len() / BLOCK_SIZE
    }

    /// Chip select released: abandon whatever was in flight.
    pub fn deselect(&mut self) {
        self.phase = Phase::Idle;
        self.reply.clear();
        self.write_buf.clear();
    }

    /// One byte exchanged. Returns what the card puts on MISO.
    pub fn transfer(&mut self, byte: u8) -> u8 {
        match self.phase {
            Phase::Idle => {
                // A command frame starts with 0b01xxxxxx.
                if byte & 0xC0 == 0x40 {
                    self.phase = Phase::Command {
                        index: byte & 0x3F,
                        taken: 0,
                    };
                }
                IDLE
            }
            Phase::Command { index, taken } => {
                if taken < 4 {
                    self.arg[taken as usize] = byte;
                    self.phase = Phase::Command {
                        index,
                        taken: taken + 1,
                    };
                } else {
                    // Fifth byte is the CRC; SPI mode ignores it unless
                    // CRC checking was turned on, which this driver
                    // never does.
                    self.begin_reply(index);
                }
                IDLE
            }
            Phase::Reply => match self.reply.pop_front() {
                Some(b) => {
                    if self.reply.is_empty() {
                        // A write command hands over to the data phase
                        // rather than going idle.
                        self.phase = match self.pending_write() {
                            true => Phase::AwaitWriteToken,
                            false => Phase::Idle,
                        };
                    }
                    b
                }
                None => {
                    self.phase = Phase::Idle;
                    IDLE
                }
            },
            Phase::DataResponse => {
                self.phase = Phase::Idle;
                DATA_ACCEPTED
            }
            Phase::AwaitWriteToken => {
                if byte == TOKEN_START {
                    self.phase = Phase::WriteData { taken: 0 };
                    self.write_buf.clear();
                }
                IDLE
            }
            Phase::WriteData { taken } => {
                if taken < BLOCK_SIZE {
                    self.write_buf.push(byte);
                    self.phase = Phase::WriteData { taken: taken + 1 };
                    IDLE
                } else if taken < BLOCK_SIZE + 1 {
                    // First CRC byte.
                    self.phase = Phase::WriteData { taken: taken + 1 };
                    IDLE
                } else {
                    // Second CRC byte. The driver sends both CRC bytes
                    // discarding what comes back, then reads the
                    // response on the *next* transfer -- so commit here
                    // and answer one byte later.
                    self.commit_write();
                    self.phase = Phase::DataResponse;
                    IDLE
                }
            }
        }
    }

    /// True when the reply just sent was for a block-write command.
    fn pending_write(&self) -> bool {
        self.write_pending
    }

    fn commit_write(&mut self) {
        let offset = self.write_block as usize * BLOCK_SIZE;
        if offset + BLOCK_SIZE <= self.blocks.len() {
            self.blocks[offset..offset + BLOCK_SIZE].copy_from_slice(&self.write_buf);
            self.blocks_written += 1;
        }
        self.write_buf.clear();
        self.write_pending = false;
    }

    fn arg_value(&self) -> u32 {
        u32::from_be_bytes(self.arg)
    }

    fn begin_reply(&mut self, index: u8) {
        self.commands_seen += 1;
        let is_app = std::mem::take(&mut self.app_cmd_pending);
        self.reply.clear();
        // The card takes a byte or two to answer; the driver polls for
        // the first byte with bit 7 clear, so one idle byte in front is
        // both realistic and harmless.
        self.reply.push_back(IDLE);

        if is_app {
            match index {
                41 => {
                    // ACMD41: report busy a few times, then ready.
                    if self.acmd41_busy_left > 0 {
                        self.acmd41_busy_left -= 1;
                        self.reply.push_back(R1_IDLE);
                    } else {
                        self.initialised = true;
                        self.reply.push_back(R1_READY);
                    }
                }
                other => {
                    self.note_unknown(other);
                    self.reply.push_back(R1_READY);
                }
            }
            self.phase = Phase::Reply;
            return;
        }

        match index {
            // GO_IDLE_STATE: enter SPI mode, report idle.
            0 => self.reply.push_back(R1_IDLE),
            // SEND_IF_COND: R7 echoes the voltage nibble and check byte.
            8 => {
                self.reply.push_back(R1_IDLE);
                self.reply.push_back(0x00);
                self.reply.push_back(0x00);
                self.reply.push_back(0x01);
                self.reply.push_back((self.arg_value() & 0xFF) as u8);
            }
            // SET_BLOCKLEN: SDHC is fixed at 512, so just accept it.
            16 => self.reply.push_back(R1_READY),
            // READ_SINGLE_BLOCK.
            17 => {
                self.reply.push_back(R1_READY);
                self.queue_block_read(self.arg_value());
            }
            // WRITE_BLOCK: acknowledge, then take the data phase.
            24 => {
                self.reply.push_back(R1_READY);
                self.write_block = self.arg_value();
                self.write_pending = true;
            }
            // APP_CMD: the next command is an ACMD.
            55 => {
                self.app_cmd_pending = true;
                self.reply.push_back(if self.initialised { R1_READY } else { R1_IDLE });
            }
            // READ_OCR: R3. Bit 30 of the first byte marks high capacity,
            // which is what makes the driver address in blocks.
            58 => {
                self.reply.push_back(R1_READY);
                self.reply.push_back(0xC0);
                self.reply.push_back(0xFF);
                self.reply.push_back(0x80);
                self.reply.push_back(0x00);
            }
            other => {
                self.note_unknown(other);
                self.reply.push_back(R1_READY);
            }
        }
        self.phase = Phase::Reply;
    }

    fn queue_block_read(&mut self, block: u32) {
        let offset = block as usize * BLOCK_SIZE;
        if offset + BLOCK_SIZE > self.blocks.len() {
            // Out of range: leave the host polling for a token that
            // never comes rather than inventing data.
            return;
        }
        self.reply.push_back(TOKEN_START);
        for i in 0..BLOCK_SIZE {
            self.reply.push_back(self.blocks[offset + i]);
        }
        // Two CRC bytes the driver reads and discards.
        self.reply.push_back(0xFF);
        self.reply.push_back(0xFF);
        self.blocks_read += 1;
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
