//! PicoCalc LCD wiring.
//!
//! Transcribed from ClockworkPi's official
//! `picocalc_helloworld/lcdspi/lcdspi.h`:
//!
//! ```text
//! #define Pico_LCD_SCK 10
//! #define Pico_LCD_TX  11 // MOSI
//! #define Pico_LCD_RX  12 // MISO
//! #define Pico_LCD_CS  13
//! #define Pico_LCD_DC  14
//! #define Pico_LCD_RST 15
//! #define Pico_LCD_SPI_MOD spi1
//! #define LCD_SPI_SPEED 25000000
//! #define LCD_WIDTH  320
//! #define LCD_HEIGHT 320
//! ```
//!
//! GP10..GP12 are switched to `GPIO_FUNC_SPI` by `lcd_spi_init`; CS, DC
//! and RESET stay on SIO and are driven with plain `gpio_put`, which is
//! why the device model observes them as pad levels rather than as part
//! of the SPI frame.

/// SSP instance driving the panel (`spi1`).
pub const LCD_SPI_INSTANCE: usize = 1;

/// Serial clock (`GPIO_FUNC_SPI`).
pub const PIN_SCK: u8 = 10;
/// Controller → panel data (`GPIO_FUNC_SPI`).
pub const PIN_MOSI: u8 = 11;
/// Panel → controller data (`GPIO_FUNC_SPI`).
pub const PIN_MISO: u8 = 12;
/// Chip select, active low (SIO).
pub const PIN_CS: u8 = 13;
/// Data/command select: 0 = command, 1 = parameter/pixel data (SIO).
pub const PIN_DC: u8 = 14;
/// Panel hardware reset, active low (SIO).
pub const PIN_RESET: u8 = 15;

/// Nominal SPI bit rate the firmware requests (`LCD_SPI_SPEED`).
pub const LCD_SPI_SPEED_HZ: u32 = 25_000_000;

/// Panel GRAM width in pixels. The ST7365P carries a 320×480 frame
/// memory; the PicoCalc only exposes a square window of it.
pub const GRAM_WIDTH: usize = 320;
/// Panel GRAM height in pixels.
pub const GRAM_HEIGHT: usize = 480;

/// Visible viewport width (`LCD_WIDTH`).
pub const VIEWPORT_WIDTH: usize = 320;
/// Visible viewport height (`LCD_HEIGHT`). The firmware sets
/// `hres = vres = 320` in `pico_lcd_init` and clips every primitive to
/// it, so rows 320..479 of the GRAM are never addressed.
pub const VIEWPORT_HEIGHT: usize = 320;

/// Extract the level of `pin` from a GPIO pad-level word.
#[inline]
pub const fn level(levels: u32, pin: u8) -> bool {
    (levels >> pin) & 1 != 0
}

/// PicoCalc off-chip SPI PSRAM (APS6404L, 8 MiB) wiring.
///
/// Transcribed from ClockworkPi's official
/// `picocalc_helloworld/CMakeLists.txt`:
///
/// ```text
/// target_compile_definitions(picocalc_helloworld PRIVATE
///     PSRAM_MUTEX=1
///     PSRAM_ASYNC=0
///     PSRAM_PIN_CS=20
///     PSRAM_PIN_SCK=21
///     PSRAM_PIN_MOSI=2
///     PSRAM_PIN_MISO=3
/// )
/// ```
///
/// `main.c` drives the PSRAM over `pio1` via
/// `psram_spi_init_clkdiv(pio1, -1, 1.0f, true)` — the `true` fourth
/// argument selects `rp2040-psram`'s `spi_psram_fudge` PIO program (not
/// the plain `spi_psram` program). See the `picoem-devices::psram`
/// module docs for how the emulator-side model's falling/rising edge
/// convention interacts with this choice, and [`psram_picocalc`]'s doc
/// comment for why that program requires a 1-SCK Fast Read output
/// delay on this board.
/// Chip select, active low. `PSRAM_PIN_CS`.
pub const PSRAM_PIN_CS: u8 = 20;
/// Serial clock, driven by `pio1`. `PSRAM_PIN_SCK`.
pub const PSRAM_PIN_SCK: u8 = 21;
/// Controller → chip data (MOSI). `PSRAM_PIN_MOSI`.
pub const PSRAM_PIN_MOSI: u8 = 2;
/// Chip → controller data (MISO). `PSRAM_PIN_MISO`.
pub const PSRAM_PIN_MISO: u8 = 3;

/// Build a [`picoem_devices::Psram`] wired the way PicoCalc solders it
/// (CS=GP20, SCK=GP21, MOSI=GP2, MISO=GP3). `Psram::new`'s parameter
/// order is `(miso, cs, sck, mosi)` — this helper exists so call sites
/// never have to remember that ordering or repeat the raw pin numbers.
///
/// Configured with `read_output_delay_sck = 1`. `main.c` selects the
/// `spi_psram_fudge` PIO program (see [`PSRAM_PIN_CS`]'s doc comment),
/// and `rp2040-psram`'s `psram_spi.pio` documents — on the chip
/// vendor's own authority — exactly why that program needs it, in the
/// comment on its extra `nop` before the read loop:
///
/// ```text
/// nop  side 0b10  ; Fudge factor of extra clock cycle; the PSRAM
///                 ; needs 1 extra for output to start appearing
/// ```
///
/// This is PicoCalc-specific only in the sense that PicoCalc's
/// firmware happens to select the fudge program; the delay itself is
/// a property of the real APS6404L chip's output settling time, not
/// of this board's wiring. A future board whose firmware selects the
/// plain (non-fudge) `spi_psram` program should build its `Psram` with
/// the default (`read_output_delay_sck = 0`) instead.
pub fn psram_picocalc() -> picoem_devices::Psram {
    picoem_devices::Psram::new(PSRAM_PIN_MISO, PSRAM_PIN_CS, PSRAM_PIN_SCK, PSRAM_PIN_MOSI)
        .with_read_output_delay(1)
}

// ---------------------------------------------------------------------
// Keyboard / power controller (STM32) on I2C1
// ---------------------------------------------------------------------
//
// From `picocalc_helloworld/i2ckbd/i2ckbd.h`:
//
//     #define I2C_KBD_MOD   i2c1
//     #define I2C_KBD_SDA   6
//     #define I2C_KBD_SCL   7
//     #define I2C_KBD_SPEED 10000   // 10 kHz when both I2C blocks are in use
//     #define I2C_KBD_ADDR  0x1F
//
// The Canonical BSP drives the same controller at 400 kHz with a
// repeated START; the bus speed does not change the byte sequence, so
// one model serves both.

/// I2C controller index the keyboard hangs off (`i2c1`).
pub const KEYBOARD_I2C_INSTANCE: usize = 1;
/// SDA pin. `I2C_KBD_SDA`.
pub const KEYBOARD_PIN_SDA: u8 = 6;
/// SCL pin. `I2C_KBD_SCL`.
pub const KEYBOARD_PIN_SCL: u8 = 7;
