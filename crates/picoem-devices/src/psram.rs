//! apmemory APS6404L-3SQR-SN external SPI PSRAM model (8 MB).
//!
//! Board-level SPI PSRAM device — pin assignments are parameterised so
//! the same model serves PicoGUS v2 (GPIO0..3) and future boards with
//! different wiring.
//!
//! Command subset (single-SPI mode only — the firmware never sends the
//! QPI-enter opcode `0x38`, so QPI state is deliberately not modelled):
//!
//! | Opcode | Mnemonic | Frame |
//! |--------|----------|-------|
//! | `0x66` | Reset Enable | 1 cmd byte |
//! | `0x99` | Reset        | 1 cmd byte, must follow `0x66` |
//! | `0x02` | Write        | 1 cmd + 3 addr (BE) + N data |
//! | `0x0B` | Fast Read    | 1 cmd + 3 addr + 8 dummy cycles + N data out |
//!
//! Any other opcode is a silent NOP (buffer unchanged, no MISO drive) —
//! protocol errors should leave subsequent commands working.
//!
//! Real-chip wall-clock delays (50/100 us reset waits, tRC, tCPH) are
//! NOT modelled — we honour the command sequence and nothing else.
//!
//! # Protocol framing
//!
//! * CS# falling edge starts a new frame: command byte shift register cleared.
//! * CS# rising edge ends the current frame: any partial byte is discarded;
//!   the buffer write done so far is preserved.
//! * Bits are clocked MSB-first on SCK rising edge (master-driven).
//! * The PSRAM drives MISO on SCK falling edge; we update the MISO latch
//!   on falling edges so it's stable for the master to sample on the next
//!   rising edge.
//!
//! # Fast Read output-start delay (`read_output_delay_sck`)
//!
//! The real APS6404L needs one extra SCK cycle after the dummy phase
//! before its output driver has settled — `rp2040-psram`'s
//! `psram_spi.pio` documents this on the chip vendor's own authority:
//! the `spi_psram_fudge` program's comment on its extra `nop` states
//! plainly "the PSRAM needs 1 extra \[clock\] for output to start
//! appearing". Firmware that selects the *non-fudge* `spi_psram`
//! program (or drives Fast Read by hand without that settling
//! allowance) does not need this — hence it is a per-instance,
//! opt-in delay (`with_read_output_delay`), not a change to the
//! default protocol timing. Default `0` reproduces the original
//! "ideal chip" behaviour byte-for-byte (see the `psram::tests`
//! module — every pre-existing test keeps passing unmodified).
//!
//! With a non-zero delay, the model still latches the read byte off
//! `buffer` immediately when the dummy phase ends (there is no
//! externally observable effect from *when* we peek the buffer, only
//! from when we start driving MISO), but withholds `driving_miso`
//! for that many SCK falling edges, presenting a deterministic `0`
//! bit while it does — matching the existing dummy-phase convention
//! ("don't-care in the real chip, but deterministic here") rather
//! than leaking stale state from a previous frame.

/// PSRAM size: 8 MiB, as on PicoGUS v2 hardware.
pub const PSRAM_SIZE: usize = 8 << 20;

const CMD_RESET_ENABLE: u8 = 0x66;
const CMD_RESET: u8 = 0x99;
const CMD_WRITE: u8 = 0x02;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_READ_ID: u8 = 0x9F;

/// SPI frame-phase state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// CS is high — no frame in progress.
    Idle,
    /// CS is low; clocking in command byte bits.
    Cmd,
    /// Inside a write: clocking in three address bytes.
    WriteAddr,
    /// Inside a write: streaming data bytes to `buffer[addr..]`.
    WriteData,
    /// Inside a fast-read: clocking in three address bytes.
    ReadAddr,
    /// Inside a fast-read: 8 dummy cycles (1 byte) after address.
    ReadDummy,
    /// Inside a fast-read: clocking data bytes out on MISO.
    ReadData,
    /// Inside Read ID: clocking in the three address bytes the
    /// command carries but does not use.
    IdAddr,
    /// Inside Read ID: clocking the eight identity bytes out.
    IdData,
    /// Unrecognised command — silent NOP for the rest of the frame.
    SilentNop,
}

/// apmemory APS6404L 8 MB SPI PSRAM model.
pub struct Psram {
    /// Backing storage — fixed-size, zero-alloc hot path.
    pub buffer: Box<[u8; PSRAM_SIZE]>,

    /// GPIO pin number for MISO (PSRAM drives this when CS is low during reads).
    pin_miso: u8,
    /// GPIO pin number for CS# (active low).
    pin_cs: u8,
    /// GPIO pin number for SCK (bit clock, driven by master).
    pin_sck: u8,
    /// GPIO pin number for MOSI (data from master).
    pin_mosi: u8,

    phase: Phase,

    /// Shift register for bits clocked in on MOSI. MSB-first; bit count
    /// drops back to zero once a full byte is consumed.
    shift_in: u8,
    shift_in_bits: u8,

    /// Shift register for bits clocked out on MISO. MSB-first; top bit
    /// is the one the master will sample on the next rising edge.
    shift_out: u8,
    shift_out_bits: u8,

    /// Accumulator for the 3 big-endian address bytes at the start of a
    /// read/write frame.
    addr_bytes_seen: u8,
    addr: u32,

    /// True iff the last completed command was `0x66` (Reset Enable);
    /// enables the next `0x99` to actually reset.
    reset_armed: bool,

    /// Previous SCK / CS observations — edge detection lives here.
    prev_sck: bool,
    prev_cs: bool,
    /// Latched MOSI sample for the most recent SCK rising edge.
    latched_mosi: bool,
    /// Latest MISO bit we want to assert (only meaningful while driving).
    miso_bit: bool,
    /// True while we are actively driving MISO (i.e. inside ReadData /
    /// ReadDummy — MISO is don't-care during dummy cycles in the real
    /// chip's output, but we leave it at 0 so the pin is deterministic).
    driving_miso: bool,

    /// Byte counters — used by write buffer overflow detection. Not
    /// strictly required by the firmware but handy for debugging.
    pub bytes_written: u64,
    pub bytes_read: u64,

    /// Number of times [`Psram::tick`] has been invoked. Useful for
    /// chain-of-life diagnostics in the harness when PSRAM appears
    /// unused — if `tick_count == 0` the bus integration never wired
    /// the tick into `update_gpio`; if non-zero but `cs_falling_count`
    /// is 0, the master never asserted CS#.
    pub tick_count: u64,
    /// Number of CS# falling edges observed (start of an SPI frame).
    /// A non-zero value means the master attempted at least one frame.
    pub cs_falling_count: u64,

    /// Command-byte decode counters — observation-only, do not affect
    /// protocol behaviour. Added to diagnose real-PIO integrations
    /// (as opposed to the hand-driven unit tests below, which always
    /// present a clean bit stream): a real master's frame can open and
    /// close `cs_falling_count` times without ever landing a byte in
    /// `bytes_written` if every command byte is being decoded as
    /// something other than `0x02`/`0x0B` — these counters make that
    /// distinguishable from "frames never open" or "buffer writes are
    /// silently dropped".
    pub cmd_write_count: u64,
    pub cmd_fast_read_count: u64,
    pub cmd_reset_enable_count: u64,
    pub cmd_reset_count: u64,
    /// Read ID (`0x9F`) commands seen.
    pub cmd_read_id_count: u64,
    /// Any command byte other than `0x66`/`0x99`/`0x02`/`0x0B`/`0x9F`.
    pub cmd_unknown_count: u64,

    /// The eight identity bytes `0x9F` returns: manufacturer, known-good
    /// die, then a six-byte EID.
    ///
    /// The default is the value read from the PicoCalc's own chip during
    /// the 2026-08-05 hardware correlation run, so firmware that prints
    /// the ID shows what the device shows. Nothing in the conformance
    /// track depends on the EID bytes; the first two are the meaningful
    /// ones.
    pub identity: [u8; 8],
    /// How far through `identity` the current Read ID frame has got.
    id_index: usize,

    /// Number of SCK falling edges to withhold `driving_miso` for
    /// after the Fast Read dummy phase ends, before the real first
    /// data bit is presented. `0` (default) is the original
    /// zero-delay "ideal chip" behaviour. See the module-level
    /// "Fast Read output-start delay" docs. Set via
    /// [`Self::with_read_output_delay`].
    read_output_delay_sck: u8,
    /// Countdown of remaining withheld falling edges for the *current*
    /// Fast Read data phase. Set from `read_output_delay_sck` when
    /// `ReadDummy` completes; reaching 0 flips `driving_miso` on but
    /// does not itself pop a bit — the pop still needs its own
    /// falling edge, which is exactly the "1 extra SCK" the real chip
    /// needs. Only ever primed once per frame (at the dummy→data
    /// transition), never re-armed between subsequent bytes of a
    /// multi-byte burst read — the real chip's output driver, once
    /// settled, keeps driving continuously.
    read_delay_remaining: u8,
}

impl Psram {
    pub fn new(pin_miso: u8, pin_cs: u8, pin_sck: u8, pin_mosi: u8) -> Self {
        // Allocate the 8 MB buffer directly on the heap — `Box::new([0u8;
        // PSRAM_SIZE])` would materialise the 8 MB array on the stack
        // before moving into a Box, which blows the default 1 MB stack
        // on Windows debug builds. Go through a Vec to force heap alloc
        // and use into_boxed_slice + try_into for the sized-Box.
        let vec = vec![0u8; PSRAM_SIZE].into_boxed_slice();
        let buffer: Box<[u8; PSRAM_SIZE]> = vec
            .try_into()
            .expect("vec of exactly PSRAM_SIZE bytes fits a sized Box");
        Self {
            buffer,
            pin_miso,
            pin_cs,
            pin_sck,
            pin_mosi,
            phase: Phase::Idle,
            shift_in: 0,
            shift_in_bits: 0,
            shift_out: 0,
            shift_out_bits: 0,
            addr_bytes_seen: 0,
            addr: 0,
            reset_armed: false,
            prev_sck: false,
            prev_cs: true,
            latched_mosi: false,
            miso_bit: false,
            driving_miso: false,
            bytes_written: 0,
            bytes_read: 0,
            tick_count: 0,
            cs_falling_count: 0,
            cmd_write_count: 0,
            cmd_fast_read_count: 0,
            cmd_reset_enable_count: 0,
            cmd_reset_count: 0,
            cmd_read_id_count: 0,
            cmd_unknown_count: 0,
            // apmemory APS6404L as fitted to the PicoCalc: 0x0D
            // manufacturer, 0x5D known-good die, then the EID.
            identity: [0x0D, 0x5D, 0x53, 0x32, 0xC6, 0x81, 0x79, 0x46],
            id_index: 0,
            read_output_delay_sck: 0,
            read_delay_remaining: 0,
        }
    }

    /// Convenience constructor for PicoGUS v2 pin assignment:
    /// MISO=GPIO0, CS=GPIO1, SCK=GPIO2, MOSI=GPIO3.
    pub fn picogus() -> Self {
        Self::new(0, 1, 2, 3)
    }

    /// Builder-style setter: withhold Fast Read's `driving_miso` for
    /// `sck` extra SCK falling edges after the dummy phase, modelling
    /// the real chip's output-settling delay (see the module-level
    /// "Fast Read output-start delay" docs). `0` (the default from
    /// [`Self::new`]) is a no-op — every existing caller and test is
    /// unaffected unless it opts in.
    pub fn with_read_output_delay(mut self, sck: u8) -> Self {
        self.read_output_delay_sck = sck;
        self
    }

    /// GPIO pin number for MISO.
    pub fn pin_miso(&self) -> u8 {
        self.pin_miso
    }

    /// GPIO pin number for CS#.
    pub fn pin_cs(&self) -> u8 {
        self.pin_cs
    }

    /// GPIO pin number for SCK.
    pub fn pin_sck(&self) -> u8 {
        self.pin_sck
    }

    /// GPIO pin number for MOSI.
    pub fn pin_mosi(&self) -> u8 {
        self.pin_mosi
    }

    /// Reset the protocol state machine (buffer preserved). Mirrors the
    /// behaviour of the 0x66+0x99 sequence on the real chip.
    pub fn reset_state(&mut self) {
        self.phase = Phase::Idle;
        self.shift_in = 0;
        self.shift_in_bits = 0;
        self.shift_out = 0;
        self.shift_out_bits = 0;
        self.addr_bytes_seen = 0;
        self.addr = 0;
        self.reset_armed = false;
        self.latched_mosi = false;
        self.miso_bit = false;
        self.driving_miso = false;
        self.read_delay_remaining = 0;
        self.id_index = 0;
    }

    /// Observe the current GPIO pin state. Call on every emulator tick
    /// after the SIO + PIO merge has settled. `pins` is a bitmask where
    /// bit `n` is the level of GPIO`n`.
    ///
    /// Returns `Some(miso_bit)` if the PSRAM is driving MISO this tick,
    /// or `None` if MISO should keep whatever level the bus merge set.
    /// The caller is responsible for splicing the returned bit into
    /// `gpio_in` bit `pin_miso`.
    pub fn tick(&mut self, pins: u32) -> Option<bool> {
        self.tick_count = self.tick_count.wrapping_add(1);

        let cs = ((pins >> self.pin_cs) & 1) != 0;
        let sck = ((pins >> self.pin_sck) & 1) != 0;
        let mosi = ((pins >> self.pin_mosi) & 1) != 0;

        // CS edge detection has to happen before clock-edge work so a
        // simultaneous CS-rise-and-clock (unusual on real hardware, but
        // possible in a single-tick emulator) ends the frame first.
        let cs_fell = !cs && self.prev_cs;
        let cs_rose = cs && !self.prev_cs;

        if cs_rose {
            self.end_frame();
        }
        if cs_fell {
            self.cs_falling_count = self.cs_falling_count.wrapping_add(1);
            self.begin_frame();
        }

        if !cs {
            // Rising edge: master drives MOSI; we latch it.
            let rising = sck && !self.prev_sck;
            let falling = !sck && self.prev_sck;
            if rising {
                self.latched_mosi = mosi;
                self.on_sck_rising();
            } else if falling {
                self.on_sck_falling();
            }
        }

        self.prev_cs = cs;
        self.prev_sck = sck;

        if self.driving_miso {
            Some(self.miso_bit)
        } else {
            None
        }
    }

    // --- Frame-boundary handlers ---------------------------------------------

    fn begin_frame(&mut self) {
        // New frame — clear shift registers and drop to command phase.
        self.phase = Phase::Cmd;
        self.shift_in = 0;
        self.shift_in_bits = 0;
        self.shift_out = 0;
        self.shift_out_bits = 0;
        self.addr_bytes_seen = 0;
        self.addr = 0;
        self.driving_miso = false;
        self.miso_bit = false;
        self.read_delay_remaining = 0;
        self.id_index = 0;
    }

    fn end_frame(&mut self) {
        // Frame ended — partial byte/in-progress command discarded. The
        // reset_armed flag survives so a `0x66` / CS-cycle / `0x99`
        // sequence still resets on the next frame. The buffer state and
        // any data written so far are preserved.
        self.phase = Phase::Idle;
        self.shift_in = 0;
        self.shift_in_bits = 0;
        self.shift_out = 0;
        self.shift_out_bits = 0;
        self.driving_miso = false;
        self.miso_bit = false;
        self.read_delay_remaining = 0;
        self.id_index = 0;
    }

    // --- Clock-edge handlers -------------------------------------------------

    fn on_sck_rising(&mut self) {
        // Clock in one bit on rising edge.
        self.shift_in = (self.shift_in << 1) | (self.latched_mosi as u8);
        self.shift_in_bits += 1;
        if self.shift_in_bits == 8 {
            let byte = self.shift_in;
            self.shift_in = 0;
            self.shift_in_bits = 0;
            self.consume_byte(byte);
        }
    }

    fn on_sck_falling(&mut self) {
        // Fast Read output-start delay (see module docs): withhold the
        // real pop for `read_delay_remaining` falling edges after the
        // dummy phase ends. Reaching 0 turns `driving_miso` on — the
        // chip's output has "started appearing" — but does not itself
        // pop a bit; the first real bit still needs its own falling
        // edge, same as every other bit. `miso_bit` is forced to a
        // deterministic `0` for the transition edge rather than left
        // as whatever it last held, so this window reads the same as
        // the pre-existing "don't-care but deterministic" dummy-phase
        // convention, not leftover state from a prior byte/frame.
        if self.read_delay_remaining > 0 {
            self.read_delay_remaining -= 1;
            if self.read_delay_remaining == 0 {
                self.driving_miso = true;
                self.miso_bit = false;
            }
            return;
        }

        // On falling edge, the PSRAM latches out the next MISO bit.
        // This happens *after* the master has sampled the previous bit
        // on the last rising edge.
        if self.shift_out_bits > 0 {
            self.miso_bit = (self.shift_out & 0x80) != 0;
            self.shift_out <<= 1;
            self.shift_out_bits -= 1;
            if self.shift_out_bits == 0 {
                // Byte fully shifted out — queue the next one from
                // whichever source this frame is reading.
                if self.phase == Phase::IdData {
                    self.advance_id_byte();
                } else {
                    self.advance_read_byte();
                }
            }
        }
    }

    // --- Per-byte state transitions ------------------------------------------

    fn consume_byte(&mut self, byte: u8) {
        match self.phase {
            Phase::Cmd => self.handle_command(byte),
            Phase::WriteAddr => self.handle_addr_byte(byte, /*is_read=*/ false),
            Phase::WriteData => {
                let off = (self.addr as usize) & (PSRAM_SIZE - 1);
                self.buffer[off] = byte;
                self.addr = self.addr.wrapping_add(1);
                self.bytes_written += 1;
                // Stay in WriteData — further bytes continue to flow.
            }
            Phase::ReadAddr => self.handle_addr_byte(byte, /*is_read=*/ true),
            Phase::ReadDummy => {
                // One byte of dummy cycles — accept and advance. We don't
                // care what the MOSI bits are. Loading the byte to shift
                // out happens immediately regardless of
                // `read_output_delay_sck` (there's no externally
                // observable effect from *when* we peek `buffer`, only
                // from when `driving_miso` turns on) — see
                // `on_sck_falling` for the actual delay mechanics.
                self.phase = Phase::ReadData;
                self.read_delay_remaining = self.read_output_delay_sck;
                self.driving_miso = self.read_delay_remaining == 0;
                self.advance_read_byte();
            }
            Phase::ReadData => {
                // Master can keep clocking to read further bytes; the
                // input bits are don't-care. Nothing to do here — the
                // falling-edge handler drives MISO.
            }
            Phase::IdAddr => {
                // Read ID carries three address bytes that the chip
                // ignores; only their count matters.
                self.addr_bytes_seen += 1;
                if self.addr_bytes_seen == 3 {
                    self.phase = Phase::IdData;
                    // Same output-settling delay as Fast Read: it is a
                    // property of the chip's output driver, not of which
                    // command asked for the data.
                    self.read_delay_remaining = self.read_output_delay_sck;
                    self.driving_miso = self.read_delay_remaining == 0;
                    self.advance_id_byte();
                }
            }
            Phase::IdData => {
                // Further clocking walks the identity bytes; inputs are
                // don't-care.
            }
            Phase::Idle | Phase::SilentNop => {
                // Silent — accept bits, produce nothing.
            }
        }
    }

    fn handle_command(&mut self, byte: u8) {
        match byte {
            CMD_RESET_ENABLE => {
                self.cmd_reset_enable_count += 1;
                self.reset_armed = true;
                // Command complete; frame continues until CS rises. Any
                // further bytes inside this frame are ignored (treat as
                // silent nop), but CS-rise handling in end_frame() keeps
                // reset_armed so the next frame's 0x99 is effective.
                self.phase = Phase::SilentNop;
            }
            CMD_RESET => {
                self.cmd_reset_count += 1;
                if self.reset_armed {
                    // Reset the state machine — clears the in-progress
                    // phase but preserves buffer. `reset_state()` also
                    // clears `reset_armed`, which matches real chip
                    // semantics (reset is a one-shot).
                    self.reset_state();
                } else {
                    // 0x99 without prior 0x66 is a nop per the datasheet.
                    self.phase = Phase::SilentNop;
                }
            }
            CMD_WRITE => {
                self.cmd_write_count += 1;
                self.reset_armed = false;
                self.phase = Phase::WriteAddr;
                self.addr_bytes_seen = 0;
                self.addr = 0;
            }
            CMD_FAST_READ => {
                self.cmd_fast_read_count += 1;
                self.reset_armed = false;
                self.phase = Phase::ReadAddr;
                self.addr_bytes_seen = 0;
                self.addr = 0;
            }
            CMD_READ_ID => {
                self.cmd_read_id_count += 1;
                self.reset_armed = false;
                self.phase = Phase::IdAddr;
                self.addr_bytes_seen = 0;
                self.id_index = 0;
            }
            _ => {
                // Unknown command — silent nop for the rest of the frame.
                self.cmd_unknown_count += 1;
                self.reset_armed = false;
                self.phase = Phase::SilentNop;
            }
        }
    }

    fn handle_addr_byte(&mut self, byte: u8, is_read: bool) {
        self.addr = (self.addr << 8) | (byte as u32);
        self.addr_bytes_seen += 1;
        if self.addr_bytes_seen == 3 {
            // 24-bit address wraps at 8 MB (0x80_0000) — APS6404 wraps
            // addresses within the chip's address space naturally.
            self.addr &= (PSRAM_SIZE as u32) - 1;
            if is_read {
                self.phase = Phase::ReadDummy;
                // dummy phase consumes exactly one byte before data flows
            } else {
                self.phase = Phase::WriteData;
            }
        }
    }

    /// Load the next read byte into `shift_out` so the falling-edge
    /// handler can clock it out bit-by-bit.
    /// Load the next identity byte for shifting out. Past the end of
    /// the eight-byte identity the chip repeats nothing meaningful, so
    /// zeros are deterministic and obvious.
    fn advance_id_byte(&mut self) {
        self.shift_out = self.identity.get(self.id_index).copied().unwrap_or(0);
        self.shift_out_bits = 8;
        self.id_index += 1;
        self.bytes_read += 1;
    }

    fn advance_read_byte(&mut self) {
        let off = (self.addr as usize) & (PSRAM_SIZE - 1);
        self.shift_out = self.buffer[off];
        self.shift_out_bits = 8;
        self.addr = self.addr.wrapping_add(1);
        self.bytes_read += 1;
    }

    // --- Inspection helpers ----------------------------------------------------

    /// Returns `true` when no SPI frame is in progress (CS high).
    /// Exposed unconditionally so cross-crate integration tests in
    /// `rp2040_emu` can assert on PSRAM state after `Emulator::reset()`.
    pub fn phase_is_idle(&self) -> bool {
        matches!(self.phase, Phase::Idle)
    }

    #[cfg(test)]
    pub fn reset_armed(&self) -> bool {
        self.reset_armed
    }

    #[cfg(test)]
    pub fn bytes_written(&self) -> u64 {
        self.bytes_written
    }
}

// =============================================================================
// Unit tests — PSRAM protocol state machine in isolation (no bus, no PIO).
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const PIN_CS: u8 = 1;
    const PIN_SCK: u8 = 2;
    const PIN_MOSI: u8 = 3;

    /// Clock one 8-bit byte out on MOSI with CS low. Returns the 8 MISO
    /// bits captured on each SCK rising edge (MSB first) — during the
    /// master's "read" phase the master samples on rising, so that's what
    /// we record for the test oracle.
    fn clock_byte(psram: &mut Psram, pins: &mut u32, byte: u8) -> u8 {
        let mut out: u8 = 0;
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1;
            // Set MOSI before the rising edge.
            *pins = (*pins & !(1 << PIN_MOSI)) | ((bit as u32) << PIN_MOSI);
            // Keep SCK low first — gives PSRAM a falling-edge slot to
            // load the next MISO bit (matches real chip: PSRAM drives on
            // falling edge, master samples on rising).
            *pins &= !(1 << PIN_SCK);
            let _ = psram.tick(*pins);
            // Rise SCK — master samples MISO, PSRAM latches MOSI.
            *pins |= 1 << PIN_SCK;
            let miso = psram.tick(*pins).unwrap_or(false);
            out = (out << 1) | (miso as u8);
        }
        // Drop SCK to leave the bus in a clean state.
        *pins &= !(1 << PIN_SCK);
        let _ = psram.tick(*pins);
        out
    }

    /// Clock one bit continuously — no trailing "drop SCK" reset
    /// between calls, unlike [`clock_byte`]. Needed to test
    /// `read_output_delay_sck`: a delay-sensitive assertion must land
    /// on real per-bit falling-edge boundaries, and `clock_byte`'s
    /// trailing low tick after every byte would itself consume one
    /// falling edge — silently absorbing the very delay the test is
    /// trying to observe if it were used across the dummy→data
    /// boundary.
    fn clock_bit(psram: &mut Psram, pins: &mut u32, mosi_bit: bool) -> bool {
        *pins = (*pins & !(1 << PIN_MOSI)) | ((mosi_bit as u32) << PIN_MOSI);
        *pins &= !(1 << PIN_SCK);
        let _ = psram.tick(*pins);
        *pins |= 1 << PIN_SCK;
        psram.tick(*pins).unwrap_or(false)
    }

    /// One MSB-first byte via [`clock_bit`], continuously (no gap
    /// before/after relative to neighbouring calls).
    fn clock_byte_continuous(psram: &mut Psram, pins: &mut u32, byte: u8) -> u8 {
        let mut out = 0u8;
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1 != 0;
            let sample = clock_bit(psram, pins, bit);
            out = (out << 1) | (sample as u8);
        }
        out
    }

    /// Drive CS low to open a frame.
    fn cs_fall(psram: &mut Psram, pins: &mut u32) {
        *pins &= !(1 << PIN_CS);
        *pins &= !(1 << PIN_SCK);
        let _ = psram.tick(*pins);
    }

    /// Drive CS high to close a frame.
    fn cs_rise(psram: &mut Psram, pins: &mut u32) {
        *pins |= 1 << PIN_CS;
        *pins &= !(1 << PIN_SCK);
        let _ = psram.tick(*pins);
    }

    fn fresh() -> (Psram, u32) {
        // Default idle: CS high, SCK low, MOSI low.
        let psram = Psram::picogus();
        let pins = 1u32 << PIN_CS;
        (psram, pins)
    }

    #[test]
    fn reset_enable_then_reset_clears_state() {
        let (mut psram, mut pins) = fresh();
        // Start a write but don't complete it, so we have in-progress state.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02); // WRITE
        clock_byte(&mut psram, &mut pins, 0x00); // addr byte 1
        cs_rise(&mut psram, &mut pins);
        // in-progress state was WriteAddr; CS rise drops us to Idle but
        // reset_armed is still false.
        assert!(!psram.reset_armed());

        // Frame 1: Reset Enable (0x66).
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x66);
        cs_rise(&mut psram, &mut pins);
        assert!(psram.reset_armed(), "0x66 must arm reset");

        // Frame 2: Reset (0x99).
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x99);
        cs_rise(&mut psram, &mut pins);
        assert!(
            !psram.reset_armed(),
            "0x99 after 0x66 must clear reset_armed"
        );
        assert!(psram.phase_is_idle());
    }

    #[test]
    fn reset_alone_without_enable_is_nop() {
        let (mut psram, mut pins) = fresh();
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x99); // Reset without prior 0x66.
        cs_rise(&mut psram, &mut pins);
        assert!(!psram.reset_armed());
        assert!(psram.phase_is_idle());
    }

    #[test]
    fn write_round_trip() {
        let (mut psram, mut pins) = fresh();
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02); // WRITE
        clock_byte(&mut psram, &mut pins, 0x00); // addr[23:16]
        clock_byte(&mut psram, &mut pins, 0x00); // addr[15:8]
        clock_byte(&mut psram, &mut pins, 0x10); // addr[7:0]
        clock_byte(&mut psram, &mut pins, 0xDE);
        clock_byte(&mut psram, &mut pins, 0xAD);
        clock_byte(&mut psram, &mut pins, 0xBE);
        clock_byte(&mut psram, &mut pins, 0xEF);
        cs_rise(&mut psram, &mut pins);
        assert_eq!(&psram.buffer[0x10..0x14], &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(psram.bytes_written(), 4);
        assert_eq!(psram.cmd_write_count, 1);
        assert_eq!(psram.cmd_fast_read_count, 0);
        assert_eq!(psram.cmd_unknown_count, 0);
    }

    /// Observation-only command-decode counters (added for Gate 3 PSRAM
    /// PIO-integration diagnostics) must tally each command byte
    /// exactly once per frame, and must not perturb existing protocol
    /// behaviour (buffer contents / `bytes_written` unaffected).
    /// The Canonical BSP's `read_id` sends the opcode plus three address
    /// bytes it does not use, then clocks eight bytes back. Hardware
    /// answered 0d5d5332c6817946 on 2026-08-05; the model reports the
    /// same so firmware that prints the ID shows what the device shows.
    #[test]
    fn read_id_returns_the_identity_the_hardware_reports() {
        let mut pins = 0u32;
        let mut psram = Psram::new(0, 1, 2, 3);
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x9F);
        for _ in 0..3 {
            clock_byte(&mut psram, &mut pins, 0x00);
        }
        let mut got = [0u8; 8];
        for slot in got.iter_mut() {
            *slot = clock_byte(&mut psram, &mut pins, 0x00);
        }
        cs_rise(&mut psram, &mut pins);
        assert_eq!(got, [0x0D, 0x5D, 0x53, 0x32, 0xC6, 0x81, 0x79, 0x46]);
        assert_eq!(psram.cmd_read_id_count, 1);
        assert_eq!(psram.cmd_unknown_count, 0, "0x9F must not be unknown");
    }

    /// Reading past the identity yields zeros rather than stale bytes.
    #[test]
    fn read_id_past_the_end_is_zero() {
        let mut pins = 0u32;
        let mut psram = Psram::new(0, 1, 2, 3);
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x9F);
        for _ in 0..3 {
            clock_byte(&mut psram, &mut pins, 0x00);
        }
        for _ in 0..8 {
            let _ = clock_byte(&mut psram, &mut pins, 0x00);
        }
        assert_eq!(clock_byte(&mut psram, &mut pins, 0x00), 0);
    }

    #[test]
    fn cmd_decode_counters_tally_each_command_byte() {
        let (mut psram, mut pins) = fresh();

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x66); // Reset Enable
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.cmd_reset_enable_count, 1);

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x99); // Reset
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.cmd_reset_count, 1);

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02); // WRITE
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0xAB);
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.cmd_write_count, 1);
        assert_eq!(psram.buffer[0x00], 0xAB, "counters must not change behaviour");

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x0B); // Fast Read
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00); // dummy
        let _ = clock_byte(&mut psram, &mut pins, 0x00);
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.cmd_fast_read_count, 1);

        // Read ID is a real command: hardware answers it, and the
        // Canonical BSP driver prints what comes back.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x9F);
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.cmd_read_id_count, 1);

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x77); // genuinely unrecognised
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.cmd_unknown_count, 1);

        // Every counter reflects exactly one occurrence — none double
        // counted, none cross-contaminated.
        assert_eq!(psram.cmd_reset_enable_count, 1);
        assert_eq!(psram.cmd_reset_count, 1);
        assert_eq!(psram.cmd_write_count, 1);
        assert_eq!(psram.cmd_fast_read_count, 1);
        assert_eq!(psram.cmd_unknown_count, 1);
    }

    #[test]
    fn fast_read_returns_written_bytes() {
        let (mut psram, mut pins) = fresh();
        // Prime the buffer.
        psram.buffer[0x10] = 0xDE;
        psram.buffer[0x11] = 0xAD;
        psram.buffer[0x12] = 0xBE;
        psram.buffer[0x13] = 0xEF;

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x0B); // Fast Read
        clock_byte(&mut psram, &mut pins, 0x00); // addr[23:16]
        clock_byte(&mut psram, &mut pins, 0x00); // addr[15:8]
        clock_byte(&mut psram, &mut pins, 0x10); // addr[7:0]
        clock_byte(&mut psram, &mut pins, 0x00); // 8 dummy cycles (one byte)
        let b0 = clock_byte(&mut psram, &mut pins, 0x00);
        let b1 = clock_byte(&mut psram, &mut pins, 0x00);
        let b2 = clock_byte(&mut psram, &mut pins, 0x00);
        let b3 = clock_byte(&mut psram, &mut pins, 0x00);
        cs_rise(&mut psram, &mut pins);

        assert_eq!([b0, b1, b2, b3], [0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn fast_read_dummy_cycles_are_ignored() {
        let (mut psram, mut pins) = fresh();
        psram.buffer[0x00] = 0x5A;
        psram.buffer[0x01] = 0xA5;

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x0B);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        // Send a non-zero dummy byte — output should be unaffected.
        clock_byte(&mut psram, &mut pins, 0xFF);
        let b0 = clock_byte(&mut psram, &mut pins, 0x12);
        let b1 = clock_byte(&mut psram, &mut pins, 0x34);
        cs_rise(&mut psram, &mut pins);

        assert_eq!([b0, b1], [0x5A, 0xA5]);
    }

    #[test]
    fn cs_rise_mid_command_discards_state() {
        let (mut psram, mut pins) = fresh();
        // Begin a write, send cmd + 2 (of 3) addr bytes, then yank CS up.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        cs_rise(&mut psram, &mut pins);
        assert!(psram.phase_is_idle());

        // Start a fresh write to a different address; expected to land
        // cleanly at the new address, unaffected by the aborted frame.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x20);
        clock_byte(&mut psram, &mut pins, 0x77);
        cs_rise(&mut psram, &mut pins);

        assert_eq!(psram.buffer[0x20], 0x77);
        // Nothing was written to the first few bytes of the buffer.
        assert_eq!(psram.buffer[0x00], 0);
        assert_eq!(psram.buffer[0x10], 0);
    }

    #[test]
    fn unknown_command_is_silent_nop() {
        let (mut psram, mut pins) = fresh();
        // 0x9F is READ-ID (per datasheet), which we don't model.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x9F);
        // Clock out a few bytes — we shouldn't be driving MISO.
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        cs_rise(&mut psram, &mut pins);

        // Buffer unchanged.
        assert!(psram.buffer[..].iter().all(|&b| b == 0));

        // Subsequent commands work normally.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0xAB);
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.buffer[0x00], 0xAB);
    }

    #[test]
    fn address_wraps_at_8mb() {
        // APS6404 wraps addresses inside the chip's address space. We
        // replicate this: a write to address 0x80_0001 lands at 0x01.
        let (mut psram, mut pins) = fresh();
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x80); // addr[23:16] = 0x80 -> 8 MB
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x01);
        clock_byte(&mut psram, &mut pins, 0xC3);
        cs_rise(&mut psram, &mut pins);

        assert_eq!(psram.buffer[0x01], 0xC3);
    }

    #[test]
    fn write_then_read_spanning_multiple_bytes() {
        // More thorough round-trip: 16 bytes, arbitrary address.
        let (mut psram, mut pins) = fresh();
        let base_addr: u32 = 0x12_3450;
        let data: [u8; 16] = [
            0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0,
            0xF0, 0x00,
        ];

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, (base_addr >> 16) as u8);
        clock_byte(&mut psram, &mut pins, (base_addr >> 8) as u8);
        clock_byte(&mut psram, &mut pins, base_addr as u8);
        for b in &data {
            clock_byte(&mut psram, &mut pins, *b);
        }
        cs_rise(&mut psram, &mut pins);

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x0B);
        clock_byte(&mut psram, &mut pins, (base_addr >> 16) as u8);
        clock_byte(&mut psram, &mut pins, (base_addr >> 8) as u8);
        clock_byte(&mut psram, &mut pins, base_addr as u8);
        clock_byte(&mut psram, &mut pins, 0x00); // dummy
        let mut got = [0u8; 16];
        for (i, slot) in got.iter_mut().enumerate() {
            *slot = clock_byte(&mut psram, &mut pins, i as u8);
        }
        cs_rise(&mut psram, &mut pins);

        assert_eq!(&got, &data);
    }

    #[test]
    fn tick_idle_without_cs_activity_stays_idle() {
        // Degenerate input: CS stays high, SCK and MOSI toggle randomly.
        // Must not affect state.
        let (mut psram, mut pins) = fresh();
        for _ in 0..16 {
            pins ^= 1 << PIN_SCK;
            pins ^= 1 << PIN_MOSI;
            let drive = psram.tick(pins);
            assert!(drive.is_none());
        }
        assert!(psram.phase_is_idle());
    }

    /// `read_output_delay_sck` (Sol's approved fix for Gate 3's read-path
    /// bug): `0` (default) must reproduce the original zero-delay
    /// stream exactly; `1` must shift the *entire* Fast Read output
    /// stream later by exactly one bit — a deterministic `0` inserted
    /// at the front, with the original stream's last bit falling off
    /// the end of however many bits are sampled. This is the
    /// "1 SCK for output to start appearing" the `spi_psram_fudge` PIO
    /// program's own comment documents, modelled as a pure output-timing
    /// shift rather than any change to command/address decoding.
    #[test]
    fn read_output_delay_shifts_fast_read_stream_by_one_bit() {
        let addr: u32 = 0x30;
        let byte0 = 0xABu8;
        let byte1 = 0xCDu8;

        let mut psram0 = Psram::picogus(); // delay = 0 (default)
        psram0.buffer[addr as usize] = byte0;
        psram0.buffer[addr as usize + 1] = byte1;

        let mut psram1 = Psram::picogus().with_read_output_delay(1);
        psram1.buffer[addr as usize] = byte0;
        psram1.buffer[addr as usize + 1] = byte1;

        // Drive an identical Fast Read frame into both instances,
        // continuously clocked (no gap between bytes — see
        // `clock_bit`'s doc comment for why that matters here), then
        // read 2 data bytes back.
        fn drive_fast_read(psram: &mut Psram, addr: u32) -> (u8, u8) {
            let mut pins = 1u32 << PIN_CS;
            cs_fall(psram, &mut pins);
            clock_byte_continuous(psram, &mut pins, 0x0B); // Fast Read
            clock_byte_continuous(psram, &mut pins, (addr >> 16) as u8);
            clock_byte_continuous(psram, &mut pins, (addr >> 8) as u8);
            clock_byte_continuous(psram, &mut pins, addr as u8);
            clock_byte_continuous(psram, &mut pins, 0x00); // dummy
            let b0 = clock_byte_continuous(psram, &mut pins, 0x00);
            let b1 = clock_byte_continuous(psram, &mut pins, 0x00);
            cs_rise(psram, &mut pins);
            (b0, b1)
        }

        let delay0 = drive_fast_read(&mut psram0, addr);
        assert_eq!(
            delay0,
            (byte0, byte1),
            "delay=0 must reproduce the original, unshifted behaviour — \
             regression guard for the (unchanged) default"
        );

        let delay1 = drive_fast_read(&mut psram1, addr);
        let expected_stream: u16 = ((byte0 as u16) << 8) | (byte1 as u16);
        let shifted: u16 = expected_stream >> 1; // 0-fill MSB, drop trailing LSB
        let expected = ((shifted >> 8) as u8, shifted as u8);
        assert_eq!(
            delay1, expected,
            "delay=1 must shift the entire output stream later by \
             exactly one bit"
        );
        assert_ne!(
            delay1,
            (byte0, byte1),
            "sanity: the delayed read must actually differ from the \
             undelayed one for this data"
        );
    }

    /// `with_read_output_delay` must not perturb command/address
    /// decoding or the write path — only Fast Read's output timing.
    #[test]
    fn read_output_delay_does_not_affect_write_path() {
        let mut psram = Psram::picogus().with_read_output_delay(1);
        let mut pins = 1u32 << PIN_CS;
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02); // WRITE
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x10);
        clock_byte(&mut psram, &mut pins, 0x77);
        cs_rise(&mut psram, &mut pins);

        assert_eq!(psram.buffer[0x10], 0x77);
        assert_eq!(psram.bytes_written(), 1);
        assert_eq!(psram.cmd_write_count, 1);
    }
}
