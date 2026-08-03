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
