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
//! file, syncs, reads it back, compares and deletes it. The BSP's FatFs
//! configuration mounts but does not format (`FF_USE_MKFS=0`), so this
//! model lays down an empty volume during construction. The model capacity
//! is 64 MiB, while FAT32 is the default to match the filesystem choice of
//! the 32 GB card supplied with PicoCalc; FAT16 remains available as an
//! explicit compatibility profile.
//!
//! Command framing follows the driver's own shape: six bytes (`0x40 |
//! index`, four argument bytes, CRC), then the card holds MISO high for
//! a few bytes before answering. R1 is one byte; R3 and R7 add four
//! trailing bytes. CMD0 and CMD8 command CRCs are checked even while
//! general SPI-mode CRC is disabled, as required by the SD protocol.
//! Data blocks arrive after a `0xFE` token and end with two CRC bytes the
//! model accepts and ignores, because the driver never enables general
//! CRC checking.
//!
//! # What is not modelled
//!
//! No card-removal or write-protect behaviour, no CSD/CID registers
//! beyond what bring-up reads, no multi-block transfer (CMD18/CMD25),
//! and no busy timing: writes complete immediately rather than holding
//! MISO low for a programming delay. Nothing in the conformance track
//! exercises those, and a half-modelled busy state would be harder to
//! reason about than its absence.

use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use crate::sha256::StreamingSha256;

/// Bytes per block. SDHC addresses in blocks, not bytes.
pub const BLOCK_SIZE: usize = 512;

/// Default capacity: 64 MiB. Large enough for a FAT volume with room to
/// work in, small enough to allocate without thought.
pub const DEFAULT_BLOCKS: usize = (64 << 20) / BLOCK_SIZE;

/// Filesystem layout provisioned into a newly-created card.
///
/// This affects only the initial block contents. The SPI/SDHC protocol
/// remains a filesystem-independent block transport.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SdFormat {
    /// Compatibility profile retained for older targets and diagnostics.
    Fat16,
    /// Default filesystem profile, matching PicoCalc's bundled 32 GB card.
    /// The model capacity remains [`DEFAULT_BLOCKS`] (64 MiB).
    #[default]
    Fat32,
}

impl SdFormat {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Fat16 => "fat16",
            Self::Fat32 => "fat32",
        }
    }
}

impl std::str::FromStr for SdFormat {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value.to_ascii_lowercase().as_str() {
            "fat16" => Ok(Self::Fat16),
            "fat32" => Ok(Self::Fat32),
            _ => Err(format!(
                "unknown SD format '{value}' (expected fat16|fat32)"
            )),
        }
    }
}

/// Stable, path-free metadata for a RAW-backed card. The source path is
/// intentionally omitted so callers can report provenance without leaking
/// host-specific absolute paths.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawMetadata {
    pub bytes: u64,
    pub blocks: usize,
    pub dirty_blocks: usize,
    pub source_sha256: String,
}

/// Idle byte. The card leaves MISO high when it has nothing to say.
const IDLE: u8 = 0xFF;
/// R1 bit 0: the card is in idle state (still initialising).
const R1_IDLE: u8 = 0x01;
/// R1 bit 3: the received command frame failed its mandatory CRC check.
const R1_COM_CRC_ERROR: u8 = 0x08;
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

/// Schema version for the diagnostic SD protocol trace.  This is separate
/// from the runner report schema because a trace is an optional observation
/// artifact, not an acceptance input.
pub const SD_TRACE_SCHEMA_VERSION: u32 = 1;
const SD_TRACE_PREVIEW_LIMIT: usize = 4096;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SdTraceDirection {
    Read,
    Write,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SdTraceData {
    pub direction: SdTraceDirection,
    pub block: u32,
    pub token: u8,
    pub length: usize,
    pub crc: [u8; 2],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SdTraceEvent {
    Command {
        sequence: u64,
        cs_epoch: u64,
        transfers: u64,
        index: u8,
        argument: u32,
        crc: u8,
        crc_valid: bool,
        response: Vec<u8>,
        data: Option<SdTraceData>,
    },
    BlockData {
        sequence: u64,
        cs_epoch: u64,
        transfers: u64,
        data: SdTraceData,
    },
    Deselect {
        sequence: u64,
        cs_epoch: u64,
        transfers: u64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SdTraceSnapshot {
    pub schema_version: u32,
    pub event_count: u64,
    pub digest_sha256: String,
    pub preview_truncated: bool,
    pub preview: Vec<SdTraceEvent>,
}

#[derive(Clone)]
struct SdTraceState {
    digest: StreamingSha256,
    event_count: u64,
    preview: Vec<SdTraceEvent>,
    preview_truncated: bool,
    cs_epoch: u64,
    cs_active: bool,
    transfers: u64,
}

impl SdTraceState {
    fn new() -> Self {
        Self {
            digest: StreamingSha256::new(),
            event_count: 0,
            preview: Vec::new(),
            preview_truncated: false,
            cs_epoch: 0,
            cs_active: false,
            transfers: 0,
        }
    }

    fn select(&mut self) {
        if !self.cs_active {
            self.cs_epoch = self.cs_epoch.saturating_add(1);
            self.cs_active = true;
            self.transfers = 0;
        }
    }

    fn transfer(&mut self) {
        // Unit tests can exercise the card without the wire's pin callbacks.
        // Treat their first byte as an implicit CS-low edge so the trace is
        // still self-contained and deterministic.
        self.select();
        self.transfers = self.transfers.saturating_add(1);
    }

    fn record(&mut self, event: SdTraceEvent) {
        let mut canonical = Vec::with_capacity(96);
        match &event {
            SdTraceEvent::Command {
                sequence,
                cs_epoch,
                transfers,
                index,
                argument,
                crc,
                crc_valid,
                response,
                data,
            } => {
                canonical.push(1);
                canonical.extend_from_slice(&sequence.to_be_bytes());
                canonical.extend_from_slice(&cs_epoch.to_be_bytes());
                canonical.extend_from_slice(&transfers.to_be_bytes());
                canonical.push(*index);
                canonical.extend_from_slice(&argument.to_be_bytes());
                canonical.push(*crc);
                canonical.push(u8::from(*crc_valid));
                canonical.extend_from_slice(&(response.len() as u32).to_be_bytes());
                canonical.extend_from_slice(response);
                match data {
                    Some(data) => {
                        canonical.push(1);
                        canonical.push(match data.direction {
                            SdTraceDirection::Read => 0,
                            SdTraceDirection::Write => 1,
                        });
                        canonical.extend_from_slice(&data.block.to_be_bytes());
                        canonical.push(data.token);
                        canonical.extend_from_slice(&(data.length as u64).to_be_bytes());
                        canonical.extend_from_slice(&data.crc);
                    }
                    None => canonical.push(0),
                }
            }
            SdTraceEvent::Deselect {
                sequence,
                cs_epoch,
                transfers,
            } => {
                canonical.push(2);
                canonical.extend_from_slice(&sequence.to_be_bytes());
                canonical.extend_from_slice(&cs_epoch.to_be_bytes());
                canonical.extend_from_slice(&transfers.to_be_bytes());
            }
            SdTraceEvent::BlockData {
                sequence,
                cs_epoch,
                transfers,
                data,
            } => {
                canonical.push(3);
                canonical.extend_from_slice(&sequence.to_be_bytes());
                canonical.extend_from_slice(&cs_epoch.to_be_bytes());
                canonical.extend_from_slice(&transfers.to_be_bytes());
                canonical.push(match data.direction {
                    SdTraceDirection::Read => 0,
                    SdTraceDirection::Write => 1,
                });
                canonical.extend_from_slice(&data.block.to_be_bytes());
                canonical.push(data.token);
                canonical.extend_from_slice(&(data.length as u64).to_be_bytes());
                canonical.extend_from_slice(&data.crc);
            }
        }
        self.digest.update(&canonical);
        self.event_count = self.event_count.saturating_add(1);
        if self.preview.len() < SD_TRACE_PREVIEW_LIMIT {
            self.preview.push(event);
        } else {
            self.preview_truncated = true;
        }
    }

    fn command(
        &mut self,
        index: u8,
        argument: u32,
        crc: u8,
        crc_valid: bool,
        response: Vec<u8>,
        data: Option<SdTraceData>,
    ) {
        self.select();
        let sequence = self.event_count;
        self.record(SdTraceEvent::Command {
            sequence,
            cs_epoch: self.cs_epoch,
            transfers: self.transfers,
            index,
            argument,
            crc,
            crc_valid,
            response,
            data,
        });
    }

    fn deselect(&mut self) {
        if !self.cs_active {
            return;
        }
        let sequence = self.event_count;
        self.record(SdTraceEvent::Deselect {
            sequence,
            cs_epoch: self.cs_epoch,
            transfers: self.transfers,
        });
        self.cs_active = false;
        self.transfers = 0;
    }

    fn snapshot(&self) -> SdTraceSnapshot {
        SdTraceSnapshot {
            schema_version: SD_TRACE_SCHEMA_VERSION,
            event_count: self.event_count,
            digest_sha256: self.digest.finalize_hex(),
            preview_truncated: self.preview_truncated,
            preview: self.preview.clone(),
        }
    }
}

struct RawBacking {
    file: File,
    source_path: PathBuf,
    source_sha256: String,
    block_count: usize,
    /// Only sectors written by the emulated card are kept here. The input
    /// file remains read-only and is never changed by a run.
    overlay: HashMap<usize, Box<[u8; BLOCK_SIZE]>>,
}

impl RawBacking {
    fn open(path: &Path) -> io::Result<Self> {
        let mut file = File::open(path)?;
        let metadata = file.metadata()?;
        if !metadata.is_file() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SD RAW backing must be a regular file",
            ));
        }
        let length = metadata.len();
        if length == 0 || length % BLOCK_SIZE as u64 != 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SD RAW image must be non-empty and a multiple of 512 bytes",
            ));
        }
        let block_count = usize::try_from(length / BLOCK_SIZE as u64).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "SD RAW image is too large for this host",
            )
        })?;
        let source_sha256 = crate::sha256::sha256_reader_hex(&mut file)?;
        file.seek(SeekFrom::Start(0))?;
        Ok(Self {
            file,
            source_path: std::fs::canonicalize(path)?,
            block_count,
            source_sha256,
            overlay: HashMap::new(),
        })
    }

    fn read_sector(&mut self, block: usize) -> io::Result<[u8; BLOCK_SIZE]> {
        if let Some(sector) = self.overlay.get(&block) {
            return Ok(**sector);
        }
        let offset = (block as u64)
            .checked_mul(BLOCK_SIZE as u64)
            .ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidInput, "SD block offset overflow")
            })?;
        self.file.seek(SeekFrom::Start(offset))?;
        let mut sector = [0u8; BLOCK_SIZE];
        self.file.read_exact(&mut sector)?;
        Ok(sector)
    }

    fn write_sector(&mut self, block: usize, data: &[u8]) -> io::Result<()> {
        if block >= self.block_count {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SD block is outside the RAW image",
            ));
        }
        if data.len() != BLOCK_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SD sector writes must be exactly 512 bytes",
            ));
        }
        let mut sector = [0u8; BLOCK_SIZE];
        sector.copy_from_slice(data);
        self.overlay.insert(block, Box::new(sector));
        Ok(())
    }

    fn export_raw(&mut self, output: &Path) -> io::Result<()> {
        // `output` is normally a new path, so canonicalize its existing
        // parent as well as an already-existing file.  Comparing only
        // `canonicalize(output)` misses alternate spellings when the final
        // component does not exist yet, and could let a same-file export
        // bypass the policy check.  Reject symlink output paths explicitly;
        // the export is an atomic rename, never a write through a link.
        if let Ok(metadata) = std::fs::symlink_metadata(output) {
            if metadata.file_type().is_symlink() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "SD RAW output must not be a symlink",
                ));
            }
        }
        let output_canonical = if output.exists() {
            std::fs::canonicalize(output)
        } else {
            let parent = output.parent().unwrap_or_else(|| Path::new("."));
            let name = output.file_name().ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "SD RAW output must name a file",
                )
            })?;
            std::fs::canonicalize(parent).map(|parent| parent.join(name))
        };
        if matches!(
            output_canonical.as_deref(),
            Ok(path) if path == self.source_path.as_path()
        ) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "SD RAW input and output must be different files",
            ));
        }
        let temporary =
            output.with_extension(format!("tmp-{}-{}", std::process::id(), self.overlay.len()));
        let mut sink = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(&temporary)?;
        for block in 0..self.block_count {
            let sector = self.read_sector(block)?;
            sink.write_all(&sector)?;
        }
        sink.flush()?;
        sink.sync_all()?;
        std::fs::rename(&temporary, output)?;
        Ok(())
    }
}

/// SD card in SPI mode.
pub struct SdCard {
    blocks: Vec<u8>,
    raw: Option<RawBacking>,
    format: SdFormat,
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
    /// CRC bytes supplied after a block-write payload.  General CRC is not
    /// validated by this model, but the trace records the wire values.
    write_crc: [u8; 2],

    // --- observation counters ---
    pub commands_seen: u64,
    pub blocks_read: u64,
    pub blocks_written: u64,
    /// Commands this model does not implement, with their counts.
    pub unknown_commands: Vec<(u8, u32)>,
    trace: Option<SdTraceState>,
}

impl Default for SdCard {
    fn default() -> Self {
        Self::new(DEFAULT_BLOCKS)
    }
}

impl SdCard {
    /// Create a card using the default FAT32 profile.
    pub fn new(block_count: usize) -> Self {
        Self::new_with_format(block_count, SdFormat::default())
    }

    /// Create a card with an explicitly selected initial volume format.
    pub fn new_with_format(block_count: usize, format: SdFormat) -> Self {
        let mut card = Self {
            blocks: vec![0u8; block_count * BLOCK_SIZE],
            raw: None,
            format,
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
            write_crc: [0; 2],
            commands_seen: 0,
            blocks_read: 0,
            blocks_written: 0,
            unknown_commands: Vec::new(),
            trace: None,
        };
        match format {
            SdFormat::Fat16 => card.format_fat16(),
            SdFormat::Fat32 => card.format_fat32(),
        }
        card
    }

    /// Open a non-empty, 512-byte-aligned RAW image as a read-only card
    /// backing. Writes are kept in a sector-sized copy-on-write overlay.
    /// The default format label is FAT32 for compatibility with the normal
    /// in-memory card; the RAW bytes themselves are not reformatted.
    pub fn from_raw_file(path: impl AsRef<Path>) -> io::Result<Self> {
        Self::from_raw_file_with_format(path, SdFormat::default())
    }

    /// Open a RAW image and retain an explicit format label for reports.
    /// The selected format does not modify or validate the image contents.
    pub fn from_raw_file_with_format(path: impl AsRef<Path>, format: SdFormat) -> io::Result<Self> {
        let raw = RawBacking::open(path.as_ref())?;
        Ok(Self {
            blocks: Vec::new(),
            raw: Some(raw),
            format,
            phase: Phase::Idle,
            arg: [0; 4],
            reply: std::collections::VecDeque::new(),
            app_cmd_pending: false,
            initialised: false,
            acmd41_busy_left: 2,
            write_block: 0,
            write_pending: false,
            write_buf: Vec::with_capacity(BLOCK_SIZE),
            write_crc: [0; 2],
            commands_seen: 0,
            blocks_read: 0,
            blocks_written: 0,
            unknown_commands: Vec::new(),
            trace: None,
        })
    }

    /// Export the complete RAW image, applying any dirty overlay sectors.
    /// Memory-backed cards do not have an input image to export.
    pub fn export_raw(&mut self, path: impl AsRef<Path>) -> io::Result<()> {
        self.raw
            .as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::Unsupported, "card has no RAW backing"))?
            .export_raw(path.as_ref())
    }

    /// Return path-free RAW metadata, or `None` for the legacy in-memory
    /// backing. Dirty blocks are the sectors currently held by the COW
    /// overlay and have not been written to the input file.
    pub fn raw_metadata(&self) -> Option<RawMetadata> {
        self.raw.as_ref().map(|raw| RawMetadata {
            bytes: (raw.block_count as u64) * BLOCK_SIZE as u64,
            blocks: raw.block_count,
            dirty_blocks: raw.overlay.len(),
            source_sha256: raw.source_sha256.clone(),
        })
    }

    /// Initial filesystem profile selected for this card.
    pub const fn format(&self) -> SdFormat {
        self.format
    }

    /// Enable the bounded, structured SPI protocol trace used by the U4
    /// loader investigation.  It is deliberately opt-in and does not alter
    /// card replies, counters, or backing-store behaviour.
    pub fn enable_trace(&mut self) {
        self.trace = Some(SdTraceState::new());
    }

    /// Return the trace snapshot, if diagnostic tracing was enabled.
    pub fn trace_snapshot(&self) -> Option<SdTraceSnapshot> {
        self.trace.as_ref().map(SdTraceState::snapshot)
    }

    /// Notify the card that SPI chip-select went low.  The wire calls this
    /// on the falling edge; direct card tests may omit it because the trace
    /// state also infers a first selection from the first transfer.
    pub fn trace_select(&mut self) {
        if let Some(trace) = self.trace.as_mut() {
            trace.select();
        }
    }

    /// Lay down an empty FAT16 volume.
    ///
    /// The BSP mounts the card and expects a filesystem to be there; it
    /// never formats. A real card ships formatted, so the model does the
    /// same rather than making every test carry a prepared image.
    ///
    /// This is the explicit compatibility profile; the default formatter
    /// is FAT32. The FAT16 layout is the textbook one: one reserved sector
    /// holding the boot record, two copies of the allocation table, a
    /// fixed-size root directory, then data.
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
        let fat_sectors = ((approx_clusters + 2) * 2).div_ceil(BYTES_PER_SECTOR);

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
        let root_start =
            (RESERVED_SECTORS as usize + NUM_FATS as usize * fat_sectors) * BYTES_PER_SECTOR;
        let root_end = root_start + root_sectors * BYTES_PER_SECTOR;
        self.blocks[root_start..root_end].fill(0);
    }

    /// Lay down an empty FAT32 volume.
    ///
    /// A one-sector cluster keeps a 64 MiB test card above the FAT32
    /// minimum cluster count. The volume is a superfloppy, like the
    /// existing FAT16 profile: sector zero is the VBR, not an MBR.
    fn format_fat32(&mut self) {
        const BYTES_PER_SECTOR: usize = BLOCK_SIZE;
        const SECTORS_PER_CLUSTER: u8 = 1;
        const RESERVED_SECTORS: u16 = 32;
        const NUM_FATS: u8 = 2;
        const ROOT_CLUSTER: u32 = 2;
        const FSINFO_SECTOR: u16 = 1;
        const BACKUP_BOOT_SECTOR: u16 = 6;
        const MIN_FAT32_CLUSTERS: usize = 65_525;

        let total_sectors = self.block_count();
        assert!(
            total_sectors <= u32::MAX as usize,
            "FAT32 profile exceeds the 32-bit BPB sector count"
        );

        // FAT size depends on the number of data clusters, which in turn
        // depends on FAT size. Iterate to the small fixed point.
        let mut fat_sectors = 1usize;
        let cluster_count = loop {
            let overhead = RESERVED_SECTORS as usize + NUM_FATS as usize * fat_sectors;
            assert!(
                total_sectors > overhead,
                "FAT32 profile needs more than {overhead} sectors"
            );
            let clusters = (total_sectors - overhead) / SECTORS_PER_CLUSTER as usize;
            let required = ((clusters + 2) * 4).div_ceil(BYTES_PER_SECTOR);
            // The exact self-consistent values can straddle a rounding
            // boundary and alternate by one sector. A table that is at
            // least as large as required is valid, so stop there.
            if required <= fat_sectors {
                break clusters;
            }
            fat_sectors = required;
        };
        assert!(
            cluster_count >= MIN_FAT32_CLUSTERS,
            "FAT32 profile needs at least {MIN_FAT32_CLUSTERS} clusters, got {cluster_count}"
        );
        assert!(fat_sectors <= u32::MAX as usize);

        self.blocks.fill(0);
        let boot = &mut self.blocks[..BYTES_PER_SECTOR];
        boot[0..3].copy_from_slice(&[0xEB, 0x58, 0x90]);
        boot[3..11].copy_from_slice(b"MSWIN4.1");
        boot[11..13].copy_from_slice(&(BYTES_PER_SECTOR as u16).to_le_bytes());
        boot[13] = SECTORS_PER_CLUSTER;
        boot[14..16].copy_from_slice(&RESERVED_SECTORS.to_le_bytes());
        boot[16] = NUM_FATS;
        // FAT32 has no fixed root directory and uses only TotSec32/FATSz32.
        boot[17..19].copy_from_slice(&0u16.to_le_bytes());
        boot[19..21].copy_from_slice(&0u16.to_le_bytes());
        boot[21] = 0xF8;
        boot[22..24].copy_from_slice(&0u16.to_le_bytes());
        boot[24..26].copy_from_slice(&63u16.to_le_bytes());
        boot[26..28].copy_from_slice(&255u16.to_le_bytes());
        boot[28..32].copy_from_slice(&0u32.to_le_bytes());
        boot[32..36].copy_from_slice(&(total_sectors as u32).to_le_bytes());
        boot[36..40].copy_from_slice(&(fat_sectors as u32).to_le_bytes());
        boot[40..42].copy_from_slice(&0u16.to_le_bytes());
        boot[42..44].copy_from_slice(&0u16.to_le_bytes());
        boot[44..48].copy_from_slice(&ROOT_CLUSTER.to_le_bytes());
        boot[48..50].copy_from_slice(&FSINFO_SECTOR.to_le_bytes());
        boot[50..52].copy_from_slice(&BACKUP_BOOT_SECTOR.to_le_bytes());
        boot[64] = 0x80;
        boot[66] = 0x29;
        boot[67..71].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        boot[71..82].copy_from_slice(b"PICOCALC   ");
        boot[82..90].copy_from_slice(b"FAT32   ");
        boot[510..512].copy_from_slice(&[0x55, 0xAA]);

        let fsinfo_offset = FSINFO_SECTOR as usize * BYTES_PER_SECTOR;
        let fsinfo = &mut self.blocks[fsinfo_offset..fsinfo_offset + BYTES_PER_SECTOR];
        fsinfo[0..4].copy_from_slice(&0x4161_5252u32.to_le_bytes());
        fsinfo[484..488].copy_from_slice(&0x6141_7272u32.to_le_bytes());
        fsinfo[488..492].copy_from_slice(&((cluster_count - 1) as u32).to_le_bytes());
        fsinfo[492..496].copy_from_slice(&3u32.to_le_bytes());
        fsinfo[508..512].copy_from_slice(&0xAA55_0000u32.to_le_bytes());

        let backup_offset = BACKUP_BOOT_SECTOR as usize * BYTES_PER_SECTOR;
        self.blocks.copy_within(0..BYTES_PER_SECTOR, backup_offset);
        let backup_fsinfo_offset =
            (BACKUP_BOOT_SECTOR as usize + FSINFO_SECTOR as usize) * BYTES_PER_SECTOR;
        self.blocks.copy_within(
            fsinfo_offset..fsinfo_offset + BYTES_PER_SECTOR,
            backup_fsinfo_offset,
        );

        // Reserved entries plus the allocated root-directory cluster.
        for copy in 0..NUM_FATS as usize {
            let start = (RESERVED_SECTORS as usize + copy * fat_sectors) * BYTES_PER_SECTOR;
            self.blocks[start..start + 4].copy_from_slice(&0x0FFF_FFF8u32.to_le_bytes());
            self.blocks[start + 4..start + 8].copy_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
            self.blocks[start + 8..start + 12].copy_from_slice(&0x0FFF_FFFFu32.to_le_bytes());
        }

        // Cluster 2 is the root directory. The backing store is already
        // zeroed, but spell out the address whose geometry the tests use.
        let root_sector = RESERVED_SECTORS as usize + NUM_FATS as usize * fat_sectors;
        let root_offset = root_sector * BYTES_PER_SECTOR;
        self.blocks[root_offset..root_offset + BYTES_PER_SECTOR].fill(0);
    }

    /// Capacity in blocks.
    pub fn block_count(&self) -> usize {
        self.raw
            .as_ref()
            .map_or(self.blocks.len() / BLOCK_SIZE, |raw| raw.block_count)
    }

    fn read_sector(&mut self, block: usize) -> io::Result<[u8; BLOCK_SIZE]> {
        if let Some(raw) = self.raw.as_mut() {
            if block >= raw.block_count {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "SD block is outside the RAW image",
                ));
            }
            return raw.read_sector(block);
        }
        let offset = block.checked_mul(BLOCK_SIZE).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "SD block offset overflow")
        })?;
        let end = offset.checked_add(BLOCK_SIZE).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "SD block offset overflow")
        })?;
        if end > self.blocks.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "SD block is outside the card",
            ));
        }
        let mut sector = [0u8; BLOCK_SIZE];
        sector.copy_from_slice(&self.blocks[offset..end]);
        Ok(sector)
    }

    /// Chip select released: abandon whatever was in flight.
    pub fn deselect(&mut self) {
        if let Some(trace) = self.trace.as_mut() {
            trace.deselect();
        }
        self.phase = Phase::Idle;
        self.reply.clear();
        self.write_buf.clear();
        self.write_crc = [0; 2];
    }

    /// One byte exchanged. Returns what the card puts on MISO.
    pub fn transfer(&mut self, byte: u8) -> u8 {
        if let Some(trace) = self.trace.as_mut() {
            trace.transfer();
        }
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
                    // CMD0 and CMD8 retain mandatory command-CRC checking
                    // even while general SPI-mode CRC is disabled.
                    self.begin_reply(index, byte);
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
                    self.write_crc = [0; 2];
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
                    self.write_crc[0] = byte;
                    self.phase = Phase::WriteData { taken: taken + 1 };
                    IDLE
                } else {
                    // Second CRC byte. The driver sends both CRC bytes
                    // discarding what comes back, then reads the
                    // response on the *next* transfer -- so commit here
                    // and answer one byte later.
                    self.write_crc[1] = byte;
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
        let block = self.write_block as usize;
        let committed = if let Some(raw) = self.raw.as_mut() {
            block < raw.block_count && raw.write_sector(block, &self.write_buf).is_ok()
        } else {
            let offset = block * BLOCK_SIZE;
            if offset + BLOCK_SIZE <= self.blocks.len() {
                self.blocks[offset..offset + BLOCK_SIZE].copy_from_slice(&self.write_buf);
                true
            } else {
                false
            }
        };
        if committed {
            self.blocks_written += 1;
            if let Some(trace) = self.trace.as_mut() {
                trace.record(SdTraceEvent::BlockData {
                    sequence: trace.event_count,
                    cs_epoch: trace.cs_epoch,
                    transfers: trace.transfers,
                    data: SdTraceData {
                        direction: SdTraceDirection::Write,
                        block: block as u32,
                        token: TOKEN_START,
                        length: self.write_buf.len(),
                        crc: self.write_crc,
                    },
                });
            }
        }
        self.write_buf.clear();
        self.write_pending = false;
    }

    fn arg_value(&self) -> u32 {
        u32::from_be_bytes(self.arg)
    }

    fn begin_reply(&mut self, index: u8, received_crc: u8) {
        self.commands_seen += 1;
        self.reply.clear();
        let argument = self.arg_value();
        let crc_valid = !matches!(index, 0 | 8) || received_crc == self.command_crc(index);
        // The card takes a byte or two to answer; the driver polls for
        // the first byte with bit 7 clear, so one idle byte in front is
        // both realistic and harmless.
        self.reply.push_back(IDLE);

        if matches!(index, 0 | 8) && !crc_valid {
            // A rejected command has no R3/R7 extension and must not
            // otherwise change card state. In particular, do not consume
            // a pending APP_CMD prefix for a frame that was never accepted.
            let state = if self.initialised { R1_READY } else { R1_IDLE };
            let response = state | R1_COM_CRC_ERROR;
            self.reply.push_back(response);
            self.trace_command(
                index,
                argument,
                received_crc,
                crc_valid,
                vec![response],
                None,
            );
            self.phase = Phase::Reply;
            return;
        }

        let is_app = std::mem::take(&mut self.app_cmd_pending);

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
            let response = self.reply.back().copied().unwrap_or(R1_READY);
            self.trace_command(
                index,
                argument,
                received_crc,
                crc_valid,
                vec![response],
                None,
            );
            self.phase = Phase::Reply;
            return;
        }

        let mut response = Vec::new();
        let mut data = None;
        match index {
            // GO_IDLE_STATE: enter SPI mode, report idle.
            0 => response.push(R1_IDLE),
            // SEND_IF_COND: R7 echoes the voltage nibble and check byte.
            8 => {
                response.extend_from_slice(&[R1_IDLE, 0x00, 0x00, 0x01, (argument & 0xFF) as u8]);
            }
            // SET_BLOCKLEN: SDHC is fixed at 512, so just accept it.
            16 => response.push(R1_READY),
            // READ_SINGLE_BLOCK.
            17 => {
                response.push(R1_READY);
                data = self.queue_block_read(argument);
            }
            // WRITE_BLOCK: acknowledge, then take the data phase.
            24 => {
                response.push(R1_READY);
                self.write_block = argument;
                self.write_pending = true;
            }
            // APP_CMD: the next command is an ACMD.
            55 => {
                self.app_cmd_pending = true;
                response.push(if self.initialised { R1_READY } else { R1_IDLE });
            }
            // READ_OCR: R3. Bit 30 of the first byte marks high capacity,
            // which is what makes the driver address in blocks.
            58 => {
                response.extend_from_slice(&[R1_READY, 0xC0, 0xFF, 0x80, 0x00]);
            }
            other => {
                self.note_unknown(other);
                response.push(R1_READY);
            }
        }
        // A block read queues its token and payload while the command is
        // decoded.  The SPI protocol still returns the command response
        // first (R1, then the data token/payload); keep the queued data
        // behind that response.  Without this ordering, a real Petit
        // FatFs host can mistake the first payload byte for R1 and retry
        // the mount forever.
        if data.is_some() {
            let queued_data = self.reply.split_off(1);
            self.reply.extend(response.iter().copied());
            self.reply.extend(queued_data);
        } else {
            self.reply.extend(response.iter().copied());
        }
        self.trace_command(index, argument, received_crc, crc_valid, response, data);
        self.phase = Phase::Reply;
    }

    fn trace_command(
        &mut self,
        index: u8,
        argument: u32,
        crc: u8,
        crc_valid: bool,
        response: Vec<u8>,
        data: Option<SdTraceData>,
    ) {
        if let Some(trace) = self.trace.as_mut() {
            trace.command(index, argument, crc, crc_valid, response, data);
        }
    }

    /// CRC byte for a command frame: CRC7 over command+argument, shifted
    /// left with the required end bit set.
    fn command_crc(&self, index: u8) -> u8 {
        let mut crc = 0u8;
        for mut byte in [
            0x40 | index,
            self.arg[0],
            self.arg[1],
            self.arg[2],
            self.arg[3],
        ] {
            for _ in 0..8 {
                crc <<= 1;
                if (byte ^ crc) & 0x80 != 0 {
                    crc ^= 0x09;
                }
                byte <<= 1;
            }
        }
        (crc << 1) | 1
    }

    fn queue_block_read(&mut self, block: u32) -> Option<SdTraceData> {
        let block = block as usize;
        let sector = match self.read_sector(block) {
            Ok(sector) => sector,
            Err(_) => {
                // Out of range or unreadable: leave the host polling for a
                // token that never comes rather than inventing data.
                return None;
            }
        };
        self.reply.push_back(TOKEN_START);
        self.reply.extend(sector);
        // Two CRC bytes the driver reads and discards.
        self.reply.push_back(0xFF);
        self.reply.push_back(0xFF);
        self.blocks_read += 1;
        Some(SdTraceData {
            direction: SdTraceDirection::Read,
            block: block as u32,
            token: TOKEN_START,
            length: BLOCK_SIZE,
            crc: [0xFF, 0xFF],
        })
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
mod format_tests {
    use super::{BLOCK_SIZE, DEFAULT_BLOCKS, SdCard, SdFormat};
    use std::path::PathBuf;

    fn temp_path(label: &str) -> PathBuf {
        std::env::temp_dir().join(format!("picocalc-sdcard-{label}-{}", std::process::id()))
    }

    fn u16_at(bytes: &[u8], offset: usize) -> u16 {
        u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap())
    }

    #[test]
    fn mandatory_command_crc_known_vectors_match_sd_protocol() {
        let mut card = SdCard::new_with_format(64, SdFormat::Fat16);
        card.arg = [0x00, 0x00, 0x00, 0x00];
        assert_eq!(card.command_crc(0), 0x95);
        card.arg = [0x00, 0x00, 0x01, 0xAA];
        assert_eq!(card.command_crc(8), 0x87);
    }

    fn u32_at(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    #[test]
    fn default_card_is_fat32() {
        let card = SdCard::default();
        assert_eq!(card.format(), SdFormat::Fat32);
        assert_eq!(&card.blocks[82..90], b"FAT32   ");
    }

    #[test]
    fn fat16_remains_an_explicit_compatibility_profile() {
        let card = SdCard::new_with_format(DEFAULT_BLOCKS, SdFormat::Fat16);
        assert_eq!(card.format(), SdFormat::Fat16);
        assert_eq!(&card.blocks[54..62], b"FAT16   ");
        assert_eq!(&card.blocks[510..512], &[0x55, 0xAA]);
    }

    #[test]
    fn fat32_bpb_has_mountable_geometry() {
        let card = SdCard::default();
        let boot = &card.blocks[..BLOCK_SIZE];
        let reserved = u16_at(boot, 14) as usize;
        let fats = boot[16] as usize;
        let fat_sectors = u32_at(boot, 36) as usize;
        let sectors_per_cluster = boot[13] as usize;
        let total = u32_at(boot, 32) as usize;
        let clusters = (total - reserved - fats * fat_sectors) / sectors_per_cluster;

        assert_eq!(u16_at(boot, 11), BLOCK_SIZE as u16);
        assert_eq!(u16_at(boot, 17), 0);
        assert_eq!(u16_at(boot, 22), 0);
        assert_eq!(u32_at(boot, 44), 2);
        assert_eq!(u16_at(boot, 48), 1);
        assert_eq!(u16_at(boot, 50), 6);
        assert!(clusters >= 65_525, "FAT32 cluster count was {clusters}");
        assert_eq!(&boot[510..512], &[0x55, 0xAA]);
    }

    #[test]
    fn fat32_writes_fsinfo_backup_and_both_fats() {
        let card = SdCard::default();
        let boot = &card.blocks[..BLOCK_SIZE];
        let reserved = u16_at(boot, 14) as usize;
        let fat_sectors = u32_at(boot, 36) as usize;
        let fsinfo = &card.blocks[BLOCK_SIZE..2 * BLOCK_SIZE];
        let backup = &card.blocks[6 * BLOCK_SIZE..7 * BLOCK_SIZE];

        assert_eq!(u32_at(fsinfo, 0), 0x4161_5252);
        assert_eq!(u32_at(fsinfo, 484), 0x6141_7272);
        assert_eq!(u32_at(fsinfo, 492), 3);
        assert_eq!(u32_at(fsinfo, 508), 0xAA55_0000);
        assert_eq!(backup, boot);

        for start_sector in [reserved, reserved + fat_sectors] {
            let fat = &card.blocks[start_sector * BLOCK_SIZE..];
            assert_eq!(u32_at(fat, 0), 0x0FFF_FFF8);
            assert_eq!(u32_at(fat, 4), 0xFFFF_FFFF);
            assert_eq!(u32_at(fat, 8), 0x0FFF_FFFF);
        }
    }

    #[test]
    fn format_names_parse_and_reject_unknown_values() {
        assert_eq!("fat16".parse(), Ok(SdFormat::Fat16));
        assert_eq!("FAT32".parse(), Ok(SdFormat::Fat32));
        assert!("exfat".parse::<SdFormat>().is_err());
    }

    #[test]
    fn raw_backing_reads_through_and_exports_cow_sectors() {
        let input = temp_path("raw-input");
        let output = temp_path("raw-output");
        let original: Vec<u8> = (0..(2 * BLOCK_SIZE)).map(|value| value as u8).collect();
        std::fs::write(&input, &original).unwrap();

        let mut card = SdCard::from_raw_file(&input).unwrap();
        assert_eq!(card.block_count(), 2);
        assert_eq!(
            card.read_sector(0).unwrap().as_slice(),
            &original[..BLOCK_SIZE]
        );

        let replacement = [0xA5u8; BLOCK_SIZE];
        card.raw
            .as_mut()
            .unwrap()
            .write_sector(1, &replacement)
            .unwrap();
        assert_eq!(card.read_sector(1).unwrap(), replacement);
        card.export_raw(&output).unwrap();

        assert_eq!(std::fs::read(&input).unwrap(), original);
        let mut expected = original;
        expected[BLOCK_SIZE..].copy_from_slice(&replacement);
        assert_eq!(std::fs::read(&output).unwrap(), expected);

        let _ = std::fs::remove_file(input);
        let _ = std::fs::remove_file(output);
    }

    #[test]
    fn raw_backing_rejects_empty_and_unaligned_inputs() {
        let empty = temp_path("raw-empty");
        let unaligned = temp_path("raw-unaligned");
        std::fs::write(&empty, []).unwrap();
        std::fs::write(&unaligned, [0u8; BLOCK_SIZE - 1]).unwrap();

        assert!(SdCard::from_raw_file(&empty).is_err());
        assert!(SdCard::from_raw_file(&unaligned).is_err());

        let _ = std::fs::remove_file(empty);
        let _ = std::fs::remove_file(unaligned);
    }

    #[test]
    fn raw_export_rejects_the_input_path_and_memory_cards() {
        let input = temp_path("raw-same-path");
        std::fs::write(&input, [0u8; BLOCK_SIZE]).unwrap();
        let mut raw = SdCard::from_raw_file(&input).unwrap();
        assert!(raw.export_raw(&input).is_err());
        let dotted = input
            .parent()
            .unwrap()
            .join(".")
            .join(input.file_name().unwrap());
        assert!(raw.export_raw(&dotted).is_err());
        #[cfg(unix)]
        {
            let alias_dir = temp_path("raw-same-path-alias-dir");
            std::fs::create_dir(&alias_dir).unwrap();
            let alias = alias_dir.join(input.file_name().unwrap());
            std::fs::remove_dir(&alias_dir).unwrap();
            std::os::unix::fs::symlink(input.parent().unwrap(), &alias_dir).unwrap();
            assert!(raw.export_raw(&alias).is_err());
            std::fs::remove_file(&alias_dir).unwrap();
        }
        assert!(
            SdCard::new_with_format(1024, SdFormat::Fat16)
                .export_raw(temp_path("memory-output"))
                .is_err()
        );
        let _ = std::fs::remove_file(input);
    }
}
