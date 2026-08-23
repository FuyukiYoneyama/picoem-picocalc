//! Board-level device models for the ClockworkPi PicoCalc.
//!
//! `rp2040-emu` is deliberately board-agnostic: it exposes a generic
//! [`rp2040_emu::peripherals::spi::SpiExternalDevice`] hook and knows
//! nothing about what is soldered to it. Every PicoCalc-specific
//! fact — which SPI instance drives the display, which GPIOs carry
//! CS/DC/RESET, what the panel does with a `0x2C` — lives in this crate.
//!
//! # Primary source
//!
//! Everything here is derived by reading (not copying) ClockworkPi's
//! official `picocalc_helloworld/lcdspi` firmware, cross-checked against
//! `ST7365P_SPEC_V1.0.pdf`. See [`pins`] and [`st7365p`] for the
//! per-decision citations.
//!
//! # Wiring it up
//!
//! ```no_run
//! use std::sync::{Arc, Mutex};
//! use picocalc_board::{pins, St7365p, St7365pWire};
//!
//! let lcd = Arc::new(Mutex::new(St7365p::new()));
//! # let mut emu: rp2040_emu::Emulator = unimplemented!();
//! emu.bus
//!     .attach_spi_device(pins::LCD_SPI_INSTANCE, Box::new(St7365pWire::new(lcd.clone())))
//!     .expect("SPI1 exists");
//! ```

pub mod framebuffer;
pub mod i2c_devices;
pub mod i2c_external;
pub mod keyboard;
pub mod lcd_pio_wire;
pub mod pins;
pub mod sd_wire;
pub mod sdcard;
pub mod sha256;
pub mod st7365p;
pub mod wire;

pub use framebuffer::{Framebuffer, PngError};
pub use i2c_devices::{
    AHT20_ADDRESS, AT24C32_ADDRESS, AT24C32_SIZE, Aht20, At24c32, BMP280_ADDRESS, Bmp280,
    DS3231_ADDRESS, Ds3231, RtcDateTime,
};
pub use i2c_external::{
    I2cBusMux, I2cBusMuxError, I2cBusObservation, I2cChildObservation, I2cExternalDeviceStub,
};
pub use keyboard::{KeyEvent, KeyState, Keyboard, KeyboardWire};
pub use lcd_pio_wire::LcdPioWire;
pub use sd_wire::SdCardWire;
pub use sdcard::{
    RawMetadata, SD_TRACE_SCHEMA_VERSION, SdCard, SdFormat, SdTraceData, SdTraceDirection,
    SdTraceEvent, SdTraceSnapshot,
};
pub use st7365p::{Colmod, St7365p};
pub use wire::St7365pWire;
