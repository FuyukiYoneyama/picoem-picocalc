//! ST7365P display-controller model (driven as an "ILI9488" by the
//! official firmware).
//!
//! # Scope
//!
//! Enough of the command set to reconstruct what the panel would be
//! showing. Frame memory is 320×480 RGB565; the PicoCalc's visible
//! window is the top-left 320×320 of it (see [`crate::pins`]).
//!
//! Decoded commands:
//!
//! | Code   | Name     | Effect                                        |
//! |--------|----------|-----------------------------------------------|
//! | `0x01` | SWRESET  | Back to the power-on state, GRAM cleared.     |
//! | `0x10` | SLPIN    | `sleeping = true`.                            |
//! | `0x11` | SLPOUT   | `sleeping = false`.                           |
//! | `0x20` | INVOFF   | `inverted = false` (recorded, not rendered).  |
//! | `0x21` | INVON    | `inverted = true` (recorded, not rendered).   |
//! | `0x28` | DISPOFF  | `display_on = false`.                         |
//! | `0x29` | DISPON   | `display_on = true`.                          |
//! | `0x2A` | CASET    | Column window, 4 parameter bytes.             |
//! | `0x2B` | RASET    | Row window, 4 parameter bytes.                |
//! | `0x2C` | RAMWR    | Pixel stream into the window.                 |
//! | `0x2E` | RAMRD    | Pixel stream out of the window (see below).   |
//! | `0x36` | MADCTL   | Recorded (see below).                         |
//! | `0x3A` | COLMOD   | Pixel format; selects bytes-per-pixel.        |
//!
//! Every other command is a no-op but is **counted by opcode** in
//! [`St7365p::unknown_commands`] — the whole ILI9488 power/gamma/frame
//! init block lands there, and silently swallowing it would hide a
//! decoder that had lost frame sync.
//!
//! # Deliberate non-modelling, with rationale
//!
//! * **MADCTL geometry (`0x36`)** — the firmware writes `0x48`
//!   (`MX | BGR`, `ILI9341_Portrait` in `lcdspi.h`). `MX` mirrors the
//!   column counter against the *physical* glass so the image comes out
//!   upright on the PicoCalc's panel orientation. The framebuffer this
//!   model produces is indexed by the **logical** CASET/RASET addresses
//!   the firmware wrote, so applying `MX` here would mirror the text
//!   horizontally relative to what a person sees on the device. The
//!   value is recorded for the report and left unapplied.
//! * **MADCTL BGR bit** — likewise a panel-wiring compensation: it is
//!   set precisely so that an R-first byte stream renders red as red.
//!   Wire bytes are therefore decoded as (R, G, B) in stream order.
//! * **INVON (`0x21`)** — the official init enables inversion because
//!   the PicoCalc glass needs it to show `BLACK` as black. Applying an
//!   inversion to the decoded stream would produce a white background,
//!   the opposite of the device. Recorded, not applied.
//! # RAMRD byte order
//!
//! `read_buffer_spi` clocks one dummy byte after `0x2E`, then three
//! bytes per pixel, and finally swaps each triple's first and third
//! byte before `draw_buffer_spi` writes it straight back with RAMWR.
//! `scroll_lcd_spi` moves a line through exactly that read-swap-write
//! round trip, so for a scrolled line to keep its colour the panel has
//! to answer in the reverse of the RAMWR order: blue first, red last.
//! That is what this model does, and
//! `ramrd_round_trip_through_the_drivers_byte_swap_preserves_colour`
//! pins it down.
//!
//! # Framing
//!
//! Bytes are accepted only while CS is asserted (low). `DC = 0` starts a
//! new command; `DC = 1` appends a parameter to the command in flight.
//! Command state deliberately survives CS rising: the official
//! `spi_write_command` / `spi_write_data` pair pulses CS around *every
//! single byte*, so a controller that reset on CS-high could never
//! receive a parameter at all.

use crate::pins::{GRAM_HEIGHT, GRAM_WIDTH};

// --- command opcodes ---------------------------------------------------

pub const CMD_SWRESET: u8 = 0x01;
pub const CMD_SLPIN: u8 = 0x10;
pub const CMD_SLPOUT: u8 = 0x11;
pub const CMD_INVOFF: u8 = 0x20;
pub const CMD_INVON: u8 = 0x21;
pub const CMD_DISPOFF: u8 = 0x28;
pub const CMD_DISPON: u8 = 0x29;
pub const CMD_CASET: u8 = 0x2A;
pub const CMD_RASET: u8 = 0x2B;
pub const CMD_RAMWR: u8 = 0x2C;
pub const CMD_RAMRD: u8 = 0x2E;
pub const CMD_MADCTL: u8 = 0x36;
pub const CMD_COLMOD: u8 = 0x3A;

/// `COLMOD` pixel formats this model can unpack.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Colmod {
    /// `0x55` — 16 bits/pixel, 2 bytes on the wire.
    Rgb565,
    /// `0x66` — 18 bits/pixel, 3 bytes on the wire, 6 significant bits
    /// in the top of each byte. This is what the PicoCalc firmware uses.
    Rgb666,
}

impl Colmod {
    /// Wire bytes per pixel.
    #[inline]
    pub const fn bytes_per_pixel(self) -> usize {
        match self {
            Colmod::Rgb565 => 2,
            Colmod::Rgb666 => 3,
        }
    }

    /// Decode a `COLMOD` register value. Only the pixel-format field
    /// (bits [2:0] of the DPI/DBI nibbles) is meaningful here; anything
    /// this model cannot unpack is reported as `None` and leaves the
    /// previous format in force.
    pub const fn from_reg(value: u8) -> Option<Colmod> {
        match value {
            0x55 => Some(Colmod::Rgb565),
            0x66 => Some(Colmod::Rgb666),
            _ => None,
        }
    }
}

/// Normalise one RGB666 wire triple to RGB565.
///
/// Each wire byte carries its channel in the **top 6 bits**
/// (`ST7365P_SPEC_V1.0.pdf`, 18-bit serial pixel format), so the 5-bit
/// red/blue fields are `byte >> 3` and the 6-bit green field is
/// `byte >> 2`. `0xFC` and `0xFF` therefore both saturate — the low two
/// bits are don't-cares on the wire.
#[inline]
pub const fn rgb666_to_rgb565(r: u8, g: u8, b: u8) -> u16 {
    let r5 = (r >> 3) as u16;
    let g6 = (g >> 2) as u16;
    let b5 = (b >> 3) as u16;
    (r5 << 11) | (g6 << 5) | b5
}

/// One decoded-but-unmodelled command opcode and how often it arrived.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct UnknownCommand {
    pub code: u8,
    pub count: u32,
}

/// ST7365P frame memory + command decoder.
pub struct St7365p {
    gram: Vec<u16>,

    // --- control lines (as last observed) ---
    cs_asserted: bool,
    dc_data: bool,
    reset_asserted: bool,

    // --- decoder state ---
    command: u8,
    param_index: usize,
    params: [u8; 4],
    pixel_bytes: [u8; 3],
    pixel_len: usize,

    // --- addressable state ---
    col_start: u16,
    col_end: u16,
    row_start: u16,
    row_end: u16,
    x: u16,
    y: u16,

    // --- observable state ---
    pub sleeping: bool,
    pub display_on: bool,
    pub inverted: bool,
    pub madctl: u8,
    pub colmod_reg: u8,
    pub colmod: Colmod,

    // --- observation counters ---
    pub reset_pulses: u32,
    pub swreset_count: u32,
    pub slpin_count: u32,
    pub slpout_count: u32,
    pub dispon_count: u32,
    pub dispoff_count: u32,
    pub caset_count: u32,
    pub raset_count: u32,
    pub ramwr_count: u32,
    pub ramrd_count: u32,
    /// True until the master has clocked the dummy byte that follows
    /// RAMRD.
    ramrd_dummy_pending: bool,
    /// Which of the three bytes of the current pixel comes next.
    ramrd_phase: u8,
    /// Pixel currently being shifted out, expanded to 6-bit channels.
    ramrd_pixel: [u8; 3],
    pub madctl_count: u32,
    pub colmod_count: u32,
    pub pixels_written: u64,
    /// Pixel writes whose target address fell outside the GRAM.
    pub pixels_dropped: u64,
    /// Data bytes that arrived with no command in flight.
    pub orphan_data_bytes: u64,
    unknown: Vec<UnknownCommand>,
}

impl Default for St7365p {
    fn default() -> Self {
        Self::new()
    }
}

impl St7365p {
    /// Power-on state: asleep, display off, GRAM cleared to black.
    pub fn new() -> Self {
        Self {
            gram: vec![0u16; GRAM_WIDTH * GRAM_HEIGHT],
            // Undriven pads read low, and low is *asserted* for both
            // CS and RESET. That matches silicon at power-up, and the
            // firmware raises both in `lcd_spi_init` before it talks.
            cs_asserted: true,
            dc_data: false,
            reset_asserted: true,
            command: 0,
            param_index: 0,
            params: [0; 4],
            pixel_bytes: [0; 3],
            pixel_len: 0,
            col_start: 0,
            col_end: (GRAM_WIDTH - 1) as u16,
            row_start: 0,
            row_end: (GRAM_HEIGHT - 1) as u16,
            x: 0,
            y: 0,
            sleeping: true,
            display_on: false,
            inverted: false,
            madctl: 0,
            // ST7365P / ILI9488 reset default is 18 bits/pixel.
            colmod_reg: 0x66,
            colmod: Colmod::Rgb666,
            reset_pulses: 0,
            swreset_count: 0,
            slpin_count: 0,
            slpout_count: 0,
            dispon_count: 0,
            dispoff_count: 0,
            caset_count: 0,
            raset_count: 0,
            ramwr_count: 0,
            ramrd_count: 0,
            ramrd_dummy_pending: false,
            ramrd_phase: 0,
            ramrd_pixel: [0; 3],
            madctl_count: 0,
            colmod_count: 0,
            pixels_written: 0,
            pixels_dropped: 0,
            orphan_data_bytes: 0,
            unknown: Vec::new(),
        }
    }

    /// Unmodelled opcodes seen so far, ascending by opcode.
    pub fn unknown_commands(&self) -> &[UnknownCommand] {
        &self.unknown
    }

    /// Update the side-band control lines. `cs`, `dc` and `reset` are
    /// **pad levels**, not logical assertions: CS and RESET are active
    /// low, DC is 0 for command / 1 for data.
    ///
    /// A RESET low→high edge re-initialises the controller, counting one
    /// [`Self::reset_pulses`].
    pub fn set_control_lines(&mut self, cs: bool, dc: bool, reset: bool) {
        self.cs_asserted = !cs;
        self.dc_data = dc;
        let was_asserted = self.reset_asserted;
        self.reset_asserted = !reset;
        if was_asserted && !self.reset_asserted {
            self.reset_pulses += 1;
            self.power_on_reset();
        }
    }

    /// Shift one byte in. Returns the byte shifted back out on MISO
    /// (always 0 — see the RAMRD note in the module docs).
    pub fn transfer_byte(&mut self, byte: u8) -> u8 {
        // Held in reset, or not selected: the panel is not listening.
        if self.reset_asserted || !self.cs_asserted {
            return 0;
        }
        if self.dc_data {
            // A read in progress takes priority: the master keeps DC
            // high and clocks dummy bytes out to receive pixel data.
            if self.command == CMD_RAMRD {
                return self.ramrd_byte();
            }
            self.data_byte(byte);
        } else {
            self.command_byte(byte);
        }
        0
    }

    /// Produce the next byte of a RAMRD stream.
    ///
    /// The panel answers with three bytes per pixel, one 6-bit channel
    /// each in the top bits, and the driver in `read_buffer_spi` swaps
    /// the first and third before writing them back with RAMWR. For a
    /// read-modify-write scroll to preserve colour, the order on the
    /// wire must therefore be the reverse of what RAMWR consumes — blue
    /// first, red last.
    fn ramrd_byte(&mut self) -> u8 {
        if self.ramrd_dummy_pending {
            self.ramrd_dummy_pending = false;
            return 0;
        }
        if self.ramrd_phase == 0 {
            let colour = self.peek_pixel();
            let r = (((colour >> 11) & 0x1F) as u8) << 3;
            let g = (((colour >> 5) & 0x3F) as u8) << 2;
            let b = ((colour & 0x1F) as u8) << 3;
            self.ramrd_pixel = [b, g, r];
            self.advance_pointer();
        }
        let byte = self.ramrd_pixel[self.ramrd_phase as usize];
        self.ramrd_phase = (self.ramrd_phase + 1) % 3;
        byte
    }

    /// Frame-memory contents at the read pointer, without advancing it.
    fn peek_pixel(&self) -> u16 {
        let (x, y) = (self.x as usize, self.y as usize);
        if x < GRAM_WIDTH && y < GRAM_HEIGHT {
            self.gram[y * GRAM_WIDTH + x]
        } else {
            0
        }
    }

    // --- decoder ------------------------------------------------------

    fn command_byte(&mut self, code: u8) {
        self.command = code;
        self.param_index = 0;
        self.pixel_len = 0;
        match code {
            CMD_SWRESET => {
                self.swreset_count += 1;
                self.power_on_reset();
                // `power_on_reset` clears `command`; keep the opcode so
                // any (spec-illegal) trailing parameter is still bound
                // to SWRESET rather than to whatever ran before.
                self.command = CMD_SWRESET;
            }
            CMD_SLPIN => {
                self.slpin_count += 1;
                self.sleeping = true;
            }
            CMD_SLPOUT => {
                self.slpout_count += 1;
                self.sleeping = false;
            }
            CMD_INVOFF => self.inverted = false,
            CMD_INVON => self.inverted = true,
            CMD_DISPOFF => {
                self.dispoff_count += 1;
                self.display_on = false;
            }
            CMD_DISPON => {
                self.dispon_count += 1;
                self.display_on = true;
            }
            CMD_CASET => self.caset_count += 1,
            CMD_RASET => self.raset_count += 1,
            CMD_RAMWR => {
                self.ramwr_count += 1;
                // Frame-memory pointer resets to the window origin on
                // every RAMWR, which is what `define_region_spi` relies
                // on for each primitive it draws.
                self.x = self.col_start;
                self.y = self.row_start;
            }
            CMD_RAMRD => {
                self.ramrd_count += 1;
                // Same pointer reset as RAMWR: the driver sets a window
                // with CASET/RASET and then streams pixels out of it.
                self.x = self.col_start;
                self.y = self.row_start;
                // The first byte the master clocks after RAMRD is a
                // dummy; pixel data starts on the one after it.
                self.ramrd_dummy_pending = true;
                self.ramrd_phase = 0;
            }
            CMD_MADCTL => self.madctl_count += 1,
            CMD_COLMOD => self.colmod_count += 1,
            other => self.note_unknown(other),
        }
    }

    fn data_byte(&mut self, byte: u8) {
        match self.command {
            CMD_CASET | CMD_RASET => {
                if self.param_index < 4 {
                    self.params[self.param_index] = byte;
                }
                self.param_index += 1;
                if self.param_index == 4 {
                    let start = u16::from_be_bytes([self.params[0], self.params[1]]);
                    let end = u16::from_be_bytes([self.params[2], self.params[3]]);
                    // The controller ignores a reversed window; the
                    // official firmware always normalises before it gets
                    // here, so clamp rather than swap and let the
                    // pointer walk handle degenerate cases.
                    if self.command == CMD_CASET {
                        self.col_start = start;
                        self.col_end = end;
                        self.x = start;
                    } else {
                        self.row_start = start;
                        self.row_end = end;
                        self.y = start;
                    }
                    self.param_index = 0;
                }
            }
            CMD_MADCTL => {
                self.madctl = byte;
                self.param_index += 1;
            }
            CMD_COLMOD => {
                self.colmod_reg = byte;
                if let Some(fmt) = Colmod::from_reg(byte) {
                    self.colmod = fmt;
                }
                self.pixel_len = 0;
                self.param_index += 1;
            }
            CMD_RAMWR => self.pixel_byte(byte),
            0 => self.orphan_data_bytes += 1,
            _ => {
                // Parameter of an unmodelled command (gamma tables,
                // power control, …). Already counted at its opcode.
                self.param_index += 1;
            }
        }
    }

    fn pixel_byte(&mut self, byte: u8) {
        let need = self.colmod.bytes_per_pixel();
        self.pixel_bytes[self.pixel_len] = byte;
        self.pixel_len += 1;
        if self.pixel_len < need {
            return;
        }
        self.pixel_len = 0;
        let colour = match self.colmod {
            Colmod::Rgb666 => rgb666_to_rgb565(
                self.pixel_bytes[0],
                self.pixel_bytes[1],
                self.pixel_bytes[2],
            ),
            Colmod::Rgb565 => u16::from_be_bytes([self.pixel_bytes[0], self.pixel_bytes[1]]),
        };
        self.store_pixel(colour);
    }

    fn store_pixel(&mut self, colour: u16) {
        let (x, y) = (self.x as usize, self.y as usize);
        if x < GRAM_WIDTH && y < GRAM_HEIGHT {
            self.gram[y * GRAM_WIDTH + x] = colour;
            self.pixels_written += 1;
        } else {
            self.pixels_dropped += 1;
        }
        self.advance_pointer();
    }

    /// Column-then-row raster walk inside the CASET/RASET window,
    /// wrapping back to the window origin at the bottom (ST7365P frame-
    /// memory pointer behaviour with `MADCTL.MV = 0`).
    fn advance_pointer(&mut self) {
        if self.x >= self.col_end {
            self.x = self.col_start;
            if self.y >= self.row_end {
                self.y = self.row_start;
            } else {
                self.y += 1;
            }
        } else {
            self.x += 1;
        }
    }

    fn note_unknown(&mut self, code: u8) {
        match self.unknown.binary_search_by_key(&code, |u| u.code) {
            Ok(i) => self.unknown[i].count += 1,
            Err(i) => self.unknown.insert(i, UnknownCommand { code, count: 1 }),
        }
    }

    /// Hardware-reset / SWRESET state. Counters and the unknown-opcode
    /// tally are *observations of the run*, not chip state, so they
    /// survive; everything the panel itself would forget is cleared.
    fn power_on_reset(&mut self) {
        self.gram.fill(0);
        self.command = 0;
        self.param_index = 0;
        self.pixel_len = 0;
        self.col_start = 0;
        self.col_end = (GRAM_WIDTH - 1) as u16;
        self.row_start = 0;
        self.row_end = (GRAM_HEIGHT - 1) as u16;
        self.x = 0;
        self.y = 0;
        self.sleeping = true;
        self.display_on = false;
        self.inverted = false;
        self.madctl = 0;
        self.colmod_reg = 0x66;
        self.colmod = Colmod::Rgb666;
    }

    // --- readout ------------------------------------------------------

    /// Raw GRAM pixel at `(x, y)`, or `None` outside the frame memory.
    pub fn gram_pixel(&self, x: usize, y: usize) -> Option<u16> {
        if x < GRAM_WIDTH && y < GRAM_HEIGHT {
            Some(self.gram[y * GRAM_WIDTH + x])
        } else {
            None
        }
    }

    /// Current CASET/RASET window as `(col_start, col_end, row_start,
    /// row_end)`.
    pub fn window(&self) -> (u16, u16, u16, u16) {
        (self.col_start, self.col_end, self.row_start, self.row_end)
    }

    /// Current frame-memory write pointer.
    pub fn write_pointer(&self) -> (u16, u16) {
        (self.x, self.y)
    }

    /// The visible viewport as a [`crate::Framebuffer`].
    pub fn framebuffer(&self) -> crate::Framebuffer {
        crate::Framebuffer::from_gram(&self.gram, GRAM_WIDTH)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pins::{VIEWPORT_HEIGHT, VIEWPORT_WIDTH};

    /// Selected, out of reset, command mode.
    fn ready() -> St7365p {
        let mut d = St7365p::new();
        // RESET low→high, then CS low (selected).
        d.set_control_lines(true, false, false);
        d.set_control_lines(true, false, true);
        d.set_control_lines(false, false, true);
        d
    }

    /// Set a CASET/RASET window from inclusive pixel coordinates.
    fn set_window(d: &mut St7365p, x0: u16, y0: u16, x1: u16, y1: u16) {
        cmd(d, CMD_CASET);
        data(d, &[(x0 >> 8) as u8, x0 as u8, (x1 >> 8) as u8, x1 as u8]);
        cmd(d, CMD_RASET);
        data(d, &[(y0 >> 8) as u8, y0 as u8, (y1 >> 8) as u8, y1 as u8]);
    }

    fn cmd(d: &mut St7365p, code: u8) {
        d.set_control_lines(false, false, true);
        d.transfer_byte(code);
    }

    fn data(d: &mut St7365p, bytes: &[u8]) {
        d.set_control_lines(false, true, true);
        for &b in bytes {
            d.transfer_byte(b);
        }
    }

    // --- RGB666 → RGB565 ---------------------------------------------

    #[test]
    fn rgb666_boundary_values() {
        assert_eq!(rgb666_to_rgb565(0x00, 0x00, 0x00), 0x0000);
        // 0xFC and 0xFF both saturate: the low 2 bits are don't-cares.
        assert_eq!(rgb666_to_rgb565(0xFC, 0xFC, 0xFC), 0xFFFF);
        assert_eq!(rgb666_to_rgb565(0xFF, 0xFF, 0xFF), 0xFFFF);
        // Pure green, the colour `lcd_print_string` draws with.
        assert_eq!(rgb666_to_rgb565(0x00, 0xFF, 0x00), 0x07E0);
        assert_eq!(rgb666_to_rgb565(0xFF, 0x00, 0x00), 0xF800);
        assert_eq!(rgb666_to_rgb565(0x00, 0x00, 0xFF), 0x001F);
        // Mid-scale: 0x80 >> 3 = 16, 0x80 >> 2 = 32.
        assert_eq!(
            rgb666_to_rgb565(0x80, 0x80, 0x80),
            (16 << 11) | (32 << 5) | 16
        );
    }

    #[test]
    fn colmod_reg_decode() {
        assert_eq!(Colmod::from_reg(0x66), Some(Colmod::Rgb666));
        assert_eq!(Colmod::from_reg(0x55), Some(Colmod::Rgb565));
        assert_eq!(Colmod::from_reg(0x03), None);
        assert_eq!(Colmod::Rgb666.bytes_per_pixel(), 3);
        assert_eq!(Colmod::Rgb565.bytes_per_pixel(), 2);
    }

    // --- framing -------------------------------------------------------

    #[test]
    fn bytes_are_ignored_while_deselected_or_in_reset() {
        let mut d = ready();
        // CS high → not listening.
        d.set_control_lines(true, false, true);
        d.transfer_byte(CMD_DISPON);
        assert_eq!(d.dispon_count, 0);
        // RESET low → not listening.
        d.set_control_lines(false, false, false);
        d.transfer_byte(CMD_DISPON);
        assert_eq!(d.dispon_count, 0);
    }

    #[test]
    fn command_state_survives_cs_pulsing_between_bytes() {
        // `spi_write_command` + `spi_write_data` raise CS after every
        // single byte; parameters must still bind to the command.
        let mut d = ready();
        cmd(&mut d, CMD_MADCTL);
        d.set_control_lines(true, false, true); // CS high between bytes
        data(&mut d, &[0x48]);
        assert_eq!(d.madctl, 0x48);
    }

    #[test]
    fn reset_edge_reinitialises_and_counts() {
        let mut d = ready();
        cmd(&mut d, CMD_SLPOUT);
        cmd(&mut d, CMD_DISPON);
        assert!(!d.sleeping && d.display_on);
        assert_eq!(d.reset_pulses, 1);
        d.set_control_lines(false, false, false);
        d.set_control_lines(false, false, true);
        assert_eq!(d.reset_pulses, 2);
        assert!(d.sleeping && !d.display_on);
        // Observations survive the reset.
        assert_eq!(d.dispon_count, 1);
    }

    #[test]
    fn unknown_commands_are_counted_by_opcode_and_sorted() {
        let mut d = ready();
        cmd(&mut d, 0xE1);
        cmd(&mut d, 0xC0);
        data(&mut d, &[0x17, 0x15]); // parameters of the unknown 0xC0
        cmd(&mut d, 0xC0);
        assert_eq!(
            d.unknown_commands(),
            &[
                UnknownCommand {
                    code: 0xC0,
                    count: 2
                },
                UnknownCommand {
                    code: 0xE1,
                    count: 1
                },
            ]
        );
    }

    #[test]
    fn power_state_commands_track() {
        let mut d = ready();
        cmd(&mut d, CMD_SLPOUT);
        assert!(!d.sleeping);
        cmd(&mut d, CMD_SLPIN);
        assert!(d.sleeping);
        cmd(&mut d, CMD_DISPON);
        assert!(d.display_on);
        cmd(&mut d, CMD_DISPOFF);
        assert!(!d.display_on);
        cmd(&mut d, CMD_INVON);
        assert!(d.inverted);
        cmd(&mut d, CMD_INVOFF);
        assert!(!d.inverted);
        assert_eq!((d.slpout_count, d.slpin_count), (1, 1));
        assert_eq!((d.dispon_count, d.dispoff_count), (1, 1));
    }

    #[test]
    fn swreset_clears_gram_but_keeps_counters() {
        let mut d = ready();
        cmd(&mut d, CMD_CASET);
        data(&mut d, &[0, 0, 0, 0]);
        cmd(&mut d, CMD_RASET);
        data(&mut d, &[0, 0, 0, 0]);
        cmd(&mut d, CMD_RAMWR);
        data(&mut d, &[0xFF, 0xFF, 0xFF]);
        assert_eq!(d.gram_pixel(0, 0), Some(0xFFFF));
        cmd(&mut d, CMD_SWRESET);
        assert_eq!(d.gram_pixel(0, 0), Some(0x0000));
        assert_eq!(d.swreset_count, 1);
        assert_eq!(d.ramwr_count, 1);
        assert_eq!(d.window(), (0, 319, 0, 479));
    }

    // --- windowing / pointer -------------------------------------------

    #[test]
    fn caset_raset_set_the_window_and_origin() {
        let mut d = ready();
        cmd(&mut d, CMD_CASET);
        data(&mut d, &[0x00, 0x10, 0x01, 0x2F]); // 16 .. 303
        cmd(&mut d, CMD_RASET);
        data(&mut d, &[0x00, 0x05, 0x00, 0x09]); // 5 .. 9
        assert_eq!(d.window(), (16, 303, 5, 9));
        cmd(&mut d, CMD_RAMWR);
        assert_eq!(d.write_pointer(), (16, 5));
    }

    #[test]
    fn pointer_walks_columns_then_rows_and_wraps_at_the_window_end() {
        let mut d = ready();
        cmd(&mut d, CMD_CASET);
        data(&mut d, &[0, 2, 0, 3]); // columns 2..3
        cmd(&mut d, CMD_RASET);
        data(&mut d, &[0, 7, 0, 8]); // rows 7..8
        cmd(&mut d, CMD_RAMWR);
        // 5 pixels into a 2x2 window: 4 fill it, the 5th wraps to origin.
        let px: [[u8; 3]; 5] = [
            [0xFC, 0x00, 0x00],
            [0x00, 0xFC, 0x00],
            [0x00, 0x00, 0xFC],
            [0xFC, 0xFC, 0x00],
            [0xFC, 0xFC, 0xFC],
        ];
        for p in px {
            data(&mut d, &p);
        }
        assert_eq!(d.gram_pixel(2, 7), Some(0xFFFF)); // overwritten by #5
        assert_eq!(d.gram_pixel(3, 7), Some(0x07E0));
        assert_eq!(d.gram_pixel(2, 8), Some(0x001F));
        assert_eq!(d.gram_pixel(3, 8), Some(0xFFE0));
        assert_eq!(d.pixels_written, 5);
        assert_eq!(d.write_pointer(), (3, 7));
    }

    #[test]
    fn single_pixel_window_stays_put() {
        let mut d = ready();
        cmd(&mut d, CMD_CASET);
        data(&mut d, &[0, 4, 0, 4]);
        cmd(&mut d, CMD_RASET);
        data(&mut d, &[0, 4, 0, 4]);
        cmd(&mut d, CMD_RAMWR);
        data(&mut d, &[0xFC, 0x00, 0x00]);
        data(&mut d, &[0x00, 0x00, 0xFC]);
        assert_eq!(d.gram_pixel(4, 4), Some(0x001F));
        assert_eq!(d.write_pointer(), (4, 4));
    }

    #[test]
    fn writes_outside_the_gram_are_dropped_not_panicking() {
        let mut d = ready();
        cmd(&mut d, CMD_CASET);
        data(&mut d, &[0x01, 0x40, 0x01, 0x40]); // column 320 — off-panel
        cmd(&mut d, CMD_RASET);
        data(&mut d, &[0x00, 0x00, 0x00, 0x00]);
        cmd(&mut d, CMD_RAMWR);
        data(&mut d, &[0xFC, 0xFC, 0xFC]);
        assert_eq!(d.pixels_written, 0);
        assert_eq!(d.pixels_dropped, 1);
    }

    #[test]
    fn rgb565_colmod_consumes_two_bytes_per_pixel() {
        let mut d = ready();
        cmd(&mut d, CMD_COLMOD);
        data(&mut d, &[0x55]);
        assert_eq!(d.colmod, Colmod::Rgb565);
        cmd(&mut d, CMD_CASET);
        data(&mut d, &[0, 0, 0, 1]);
        cmd(&mut d, CMD_RASET);
        data(&mut d, &[0, 0, 0, 0]);
        cmd(&mut d, CMD_RAMWR);
        data(&mut d, &[0x07, 0xE0, 0xF8, 0x00]);
        assert_eq!(d.gram_pixel(0, 0), Some(0x07E0));
        assert_eq!(d.gram_pixel(1, 0), Some(0xF800));
    }

    #[test]
    fn unsupported_colmod_leaves_the_previous_format_in_force() {
        let mut d = ready();
        cmd(&mut d, CMD_COLMOD);
        data(&mut d, &[0x03]);
        assert_eq!(d.colmod_reg, 0x03);
        assert_eq!(d.colmod, Colmod::Rgb666);
    }

    #[test]
    fn ramrd_is_counted_and_starts_with_a_dummy_byte() {
        let mut d = ready();
        cmd(&mut d, CMD_RAMRD);
        d.set_control_lines(false, true, true);
        assert_eq!(d.transfer_byte(0xFF), 0, "first byte after RAMRD is dummy");
        assert_eq!(d.ramrd_count, 1);
    }

    /// `read_buffer_spi` reads three bytes per pixel, swaps the first
    /// and third, and `draw_buffer_spi` writes the result straight back.
    /// That round trip is how `scroll_lcd_spi` moves a line, so it has
    /// to return the pixel unchanged.
    #[test]
    fn ramrd_round_trip_through_the_drivers_byte_swap_preserves_colour() {
        let mut d = ready();
        // Paint one known pixel at the origin.
        set_window(&mut d, 0, 0, 0, 0);
        cmd(&mut d, CMD_RAMWR);
        d.set_control_lines(false, true, true);
        // RGB666 wire order for RAMWR is R, G, B.
        for b in [0xF8u8, 0x40, 0x08] {
            d.transfer_byte(b);
        }
        let stored = d.gram[0];

        // Read it back the way the driver does.
        set_window(&mut d, 0, 0, 0, 0);
        cmd(&mut d, CMD_RAMRD);
        d.set_control_lines(false, true, true);
        let _dummy = d.transfer_byte(0xFF);
        let mut got = [0u8; 3];
        for slot in got.iter_mut() {
            *slot = d.transfer_byte(0xFF);
        }
        // The driver's swap: p[0] and p[2] exchange places.
        got.swap(0, 2);

        // Write the swapped buffer back to a different pixel.
        set_window(&mut d, 5, 5, 5, 5);
        cmd(&mut d, CMD_RAMWR);
        d.set_control_lines(false, true, true);
        for b in got {
            d.transfer_byte(b);
        }
        assert_eq!(
            d.gram[5 * GRAM_WIDTH + 5],
            stored,
            "a scrolled line must keep its colour"
        );
    }

    #[test]
    fn ramrd_walks_the_window_pixel_by_pixel() {
        let mut d = ready();
        set_window(&mut d, 0, 0, 1, 0);
        cmd(&mut d, CMD_RAMWR);
        d.set_control_lines(false, true, true);
        for b in [0xF8u8, 0x00, 0x00, 0x00, 0x00, 0xF8] {
            d.transfer_byte(b);
        }

        set_window(&mut d, 0, 0, 1, 0);
        cmd(&mut d, CMD_RAMRD);
        d.set_control_lines(false, true, true);
        let _dummy = d.transfer_byte(0xFF);
        let first: Vec<u8> = (0..3).map(|_| d.transfer_byte(0xFF)).collect();
        let second: Vec<u8> = (0..3).map(|_| d.transfer_byte(0xFF)).collect();
        // Wire order is B, G, R: a red pixel leads with zero blue and a
        // blue pixel leads with full blue.
        assert_eq!(first[0], 0x00, "first pixel is red, so blue is zero");
        assert_ne!(first[2], 0x00, "first pixel is red, so red is set");
        assert_ne!(second[0], 0x00, "second pixel is blue");
        assert_eq!(second[2], 0x00, "second pixel has no red");
    }

    #[test]
    fn viewport_crops_the_gram_to_the_visible_square() {
        let mut d = ready();
        cmd(&mut d, CMD_CASET);
        data(&mut d, &[0x00, 0x00, 0x01, 0x3F]); // 0..319
        cmd(&mut d, CMD_RASET);
        data(&mut d, &[0x01, 0x40, 0x01, 0x40]); // row 320 — below the viewport
        cmd(&mut d, CMD_RAMWR);
        data(&mut d, &[0xFC, 0xFC, 0xFC]);
        assert_eq!(d.gram_pixel(0, 320), Some(0xFFFF));
        let fb = d.framebuffer();
        assert_eq!((fb.width, fb.height), (VIEWPORT_WIDTH, VIEWPORT_HEIGHT));
        // Row 320 is outside the viewport, so the framebuffer is black.
        assert!(fb.pixels.iter().all(|&p| p == 0));
    }
}
