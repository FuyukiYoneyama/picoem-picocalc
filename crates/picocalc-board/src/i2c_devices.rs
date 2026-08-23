//! Deterministic models for the first private I2C-EXT devices.
//!
//! These models are intentionally independent of the harness CLI. A profile
//! builder may compose them with [`crate::I2cBusMux`], while downstream users
//! can copy the same child boundary for another private module without
//! changing the RP2040 controller model.

use rp2040_emu::peripherals::i2c::{I2cExternalDevice, I2cVirtualTimeDelta};

pub const DS3231_ADDRESS: u16 = 0x68;
pub const AT24C32_ADDRESS: u16 = 0x57;
pub const AHT20_ADDRESS: u16 = 0x38;
pub const BMP280_ADDRESS: u16 = 0x77;
const DS3231_REG_STATUS: usize = 0x0f;
pub const AT24C32_SIZE: usize = 4096;
const AT24C32_PAGE_SIZE: u16 = 32;
const EEPROM_WRITE_CYCLE_NS: u64 = 5_000_000;
const AHT20_MEASUREMENT_NS: u64 = 90_000_000;
const BMP280_MEASUREMENT_NS: u64 = 40_000_000;

fn bcd(value: u8) -> u8 {
    ((value / 10) << 4) | (value % 10)
}

fn unbcd(value: u8, max: u8) -> Option<u8> {
    if value & 0x0f > 9 || value >> 4 > 9 {
        return None;
    }
    let decoded = (value >> 4) * 10 + (value & 0x0f);
    (decoded <= max).then_some(decoded)
}

fn leap(year: u16) -> bool {
    year.is_multiple_of(400) || (year.is_multiple_of(4) && !year.is_multiple_of(100))
}

fn days_in_month(year: u16, month: u8) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap(year) => 29,
        2 => 28,
        _ => 0,
    }
}

/// A validated 24-hour DS3231 calendar value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RtcDateTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub day_of_week: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl RtcDateTime {
    pub fn new(
        year: u16,
        month: u8,
        day: u8,
        day_of_week: u8,
        hour: u8,
        minute: u8,
        second: u8,
    ) -> Option<Self> {
        let value = Self {
            year,
            month,
            day,
            day_of_week,
            hour,
            minute,
            second,
        };
        value.is_valid().then_some(value)
    }

    pub fn is_valid(self) -> bool {
        (2000..=2099).contains(&self.year)
            && (1..=12).contains(&self.month)
            && (1..=days_in_month(self.year, self.month)).contains(&self.day)
            && self.day_of_week > 0
            && self.day_of_week <= 7
            && self.hour < 24
            && self.minute < 60
            && self.second < 60
    }

    fn advance_one_second(&mut self) {
        self.second += 1;
        if self.second < 60 {
            return;
        }
        self.second = 0;
        self.minute += 1;
        if self.minute < 60 {
            return;
        }
        self.minute = 0;
        self.hour += 1;
        if self.hour < 24 {
            return;
        }
        self.hour = 0;
        self.day += 1;
        self.day_of_week = self.day_of_week % 7 + 1;
        if self.day <= days_in_month(self.year, self.month) {
            return;
        }
        self.day = 1;
        self.month += 1;
        if self.month <= 12 {
            return;
        }
        self.month = 1;
        self.year = if self.year == 2099 {
            2000
        } else {
            self.year + 1
        };
    }
}

/// Bounded DS3231 model used by the `picocalc-rtc-v1` profile.
pub struct Ds3231 {
    datetime: RtcDateTime,
    status: u8,
    pointer: u8,
    pointer_expected: bool,
    wrote_register: bool,
    write_start_pointer: u8,
    time_write_values: [u8; 7],
    time_write_mask: u8,
    fractional_ns: u64,
    protocol_errors: u64,
}

impl Ds3231 {
    pub fn new(datetime: RtcDateTime, osf: bool) -> Self {
        Self {
            datetime,
            status: if osf { 0x80 } else { 0 },
            pointer: 0,
            pointer_expected: true,
            wrote_register: false,
            write_start_pointer: 0,
            time_write_values: [0; 7],
            time_write_mask: 0,
            fractional_ns: 0,
            protocol_errors: 0,
        }
    }

    pub fn datetime(&self) -> RtcDateTime {
        self.datetime
    }

    pub fn status(&self) -> u8 {
        self.status
    }

    pub fn protocol_errors(&self) -> u64 {
        self.protocol_errors
    }

    fn register(&self, index: u8) -> u8 {
        match index {
            0 => bcd(self.datetime.second),
            1 => bcd(self.datetime.minute),
            2 => bcd(self.datetime.hour),
            3 => bcd(self.datetime.day_of_week),
            4 => bcd(self.datetime.day),
            5 => bcd(self.datetime.month),
            6 => bcd((self.datetime.year - 2000) as u8),
            0x0f => self.status,
            _ => 0,
        }
    }

    fn commit_time_registers(&mut self, values: [u8; 7]) {
        let Some(second) = unbcd(values[0] & 0x7f, 59) else {
            self.protocol_errors += 1;
            return;
        };
        let Some(minute) = unbcd(values[1] & 0x7f, 59) else {
            self.protocol_errors += 1;
            return;
        };
        let Some(hour) = unbcd(values[2] & 0x3f, 23) else {
            self.protocol_errors += 1;
            return;
        };
        let Some(day_of_week) = unbcd(values[3] & 0x07, 7) else {
            self.protocol_errors += 1;
            return;
        };
        let Some(day) = unbcd(values[4] & 0x3f, 31) else {
            self.protocol_errors += 1;
            return;
        };
        let Some(month) = unbcd(values[5] & 0x1f, 12) else {
            self.protocol_errors += 1;
            return;
        };
        let Some(year) = unbcd(values[6], 99) else {
            self.protocol_errors += 1;
            return;
        };
        let Some(datetime) = RtcDateTime::new(
            2000 + u16::from(year),
            month,
            day,
            day_of_week,
            hour,
            minute,
            second,
        ) else {
            self.protocol_errors += 1;
            return;
        };
        self.datetime = datetime;
        self.fractional_ns = 0;
    }
}

impl I2cExternalDevice for Ds3231 {
    fn model_name(&self) -> &'static str {
        "ds3231"
    }

    fn protocol_error_count(&self) -> u64 {
        self.protocol_errors
    }

    fn state_summary(&self) -> String {
        format!(
            "{{\"datetime\":\"{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z\",\"day_of_week\":{},\"status\":{},\"pointer\":{}}}",
            self.datetime.year,
            self.datetime.month,
            self.datetime.day,
            self.datetime.hour,
            self.datetime.minute,
            self.datetime.second,
            self.datetime.day_of_week,
            self.status,
            self.pointer,
        )
    }

    fn responds_to(&self, addr: u16) -> bool {
        addr == DS3231_ADDRESS
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        if !self.responds_to(addr) {
            return false;
        }
        self.pointer_expected = true;
        self.wrote_register = false;
        true
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        if self.pointer_expected {
            if byte > 0x12 {
                self.protocol_errors += 1;
                return false;
            }
            self.pointer = byte;
            self.write_start_pointer = byte;
            self.pointer_expected = false;
            return true;
        }
        if self.pointer > 0x12 {
            self.protocol_errors += 1;
            return false;
        }
        self.wrote_register = true;
        if self.pointer <= 6 {
            self.time_write_values[self.pointer as usize] = byte;
            self.time_write_mask |= 1 << self.pointer;
        } else if self.pointer == DS3231_REG_STATUS as u8 {
            self.status = byte & 0x88;
        }
        self.pointer = (self.pointer + 1) & 0x1f;
        true
    }

    fn read_byte(&mut self) -> u8 {
        let value = self.register(self.pointer);
        self.pointer = (self.pointer + 1) & 0x1f;
        value
    }

    fn transaction_end(&mut self) {
        if self.wrote_register && self.time_write_mask != 0 {
            if self.write_start_pointer == 0 && self.time_write_mask == 0x7f {
                self.commit_time_registers(self.time_write_values);
            } else {
                self.protocol_errors += 1;
            }
        }
        self.pointer_expected = true;
        self.wrote_register = false;
        self.time_write_mask = 0;
    }

    fn advance_virtual_time(&mut self, delta: I2cVirtualTimeDelta) {
        self.fractional_ns = self.fractional_ns.saturating_add(delta.nanoseconds);
        let seconds = self.fractional_ns / 1_000_000_000;
        self.fractional_ns %= 1_000_000_000;
        for _ in 0..seconds {
            self.datetime.advance_one_second();
        }
    }
}

/// AT24C32 (4 KiB, 32-byte pages) with deterministic write-cycle busy time.
pub struct At24c32 {
    memory: [u8; AT24C32_SIZE],
    pointer: u16,
    pointer_bytes: u8,
    page_start: u16,
    page_offset: u16,
    pending_write: Vec<u8>,
    busy_ns: u64,
    protocol_errors: u64,
}

impl At24c32 {
    pub fn new(initial: [u8; AT24C32_SIZE]) -> Self {
        Self {
            memory: initial,
            pointer: 0,
            pointer_bytes: 0,
            page_start: 0,
            page_offset: 0,
            pending_write: Vec::new(),
            busy_ns: 0,
            protocol_errors: 0,
        }
    }

    pub fn image(&self) -> &[u8; AT24C32_SIZE] {
        &self.memory
    }

    pub fn is_busy(&self) -> bool {
        self.busy_ns != 0
    }

    pub fn protocol_errors(&self) -> u64 {
        self.protocol_errors
    }
}

impl I2cExternalDevice for At24c32 {
    fn model_name(&self) -> &'static str {
        "at24c32"
    }

    fn protocol_error_count(&self) -> u64 {
        self.protocol_errors
    }

    fn state_summary(&self) -> String {
        format!(
            "{{\"image_sha256\":\"{}\",\"pointer\":{},\"busy\":{},\"busy_ns\":{}}}",
            crate::sha256::sha256_hex(&self.memory),
            self.pointer,
            self.is_busy(),
            self.busy_ns,
        )
    }

    fn responds_to(&self, addr: u16) -> bool {
        addr == AT24C32_ADDRESS
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        self.responds_to(addr) && !self.is_busy()
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        if self.pointer_bytes < 2 {
            if self.pointer_bytes == 0 {
                self.pointer = u16::from(byte) << 8;
            } else {
                self.pointer |= u16::from(byte);
                self.page_start = self.pointer & !(AT24C32_PAGE_SIZE - 1);
                self.page_offset = self.pointer & (AT24C32_PAGE_SIZE - 1);
            }
            self.pointer_bytes += 1;
            return true;
        }
        if self.pending_write.len() >= usize::from(AT24C32_PAGE_SIZE) {
            self.protocol_errors += 1;
            return false;
        }
        self.pending_write.push(byte);
        self.pointer = self.page_start
            + ((self.page_offset + self.pending_write.len() as u16) & (AT24C32_PAGE_SIZE - 1));
        true
    }

    fn read_byte(&mut self) -> u8 {
        let value = self.memory[usize::from(self.pointer) % AT24C32_SIZE];
        self.pointer = (self.pointer + 1) % AT24C32_SIZE as u16;
        value
    }

    fn transaction_end(&mut self) {
        if !self.pending_write.is_empty() {
            for (offset, byte) in self.pending_write.drain(..).enumerate() {
                let address = self.page_start
                    + ((self.page_offset + offset as u16) & (AT24C32_PAGE_SIZE - 1));
                self.memory[usize::from(address) % AT24C32_SIZE] = byte;
            }
            self.busy_ns = EEPROM_WRITE_CYCLE_NS;
        }
        self.pointer_bytes = 0;
    }

    fn advance_virtual_time(&mut self, delta: I2cVirtualTimeDelta) {
        self.busy_ns = self.busy_ns.saturating_sub(delta.nanoseconds);
    }
}

fn aht20_crc8(bytes: &[u8; 6]) -> u8 {
    let mut crc = 0xff;
    for byte in bytes {
        crc ^= *byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Deterministic AHT20 model for the optional `picocalc-rtc-env-v1` profile.
///
/// The model follows the command sequence used by the PicoCalc reference
/// firmware: optional calibration (`BE 08 00`), trigger (`AC 33 00`), a
/// deterministic 90 ms conversion window, then a seven-byte measurement.
/// Fixture bytes contain the sensor status/raw fields/CRC exactly as they
/// appear on the wire; host wall time is never consulted.
pub struct Aht20 {
    measurement: [u8; 7],
    status: u8,
    command: Vec<u8>,
    read_index: usize,
    conversion_ns: u64,
    protocol_errors: u64,
}

impl Aht20 {
    pub fn new(measurement: [u8; 7]) -> Result<Self, String> {
        let payload = [
            measurement[0],
            measurement[1],
            measurement[2],
            measurement[3],
            measurement[4],
            measurement[5],
        ];
        if measurement[0] & 0x80 != 0 {
            return Err("AHT20 fixture measurement must not be marked busy".to_string());
        }
        if aht20_crc8(&payload) != measurement[6] {
            return Err(format!(
                "AHT20 measurement CRC mismatch: expected 0x{:02x}, got 0x{:02x}",
                aht20_crc8(&payload),
                measurement[6]
            ));
        }
        Ok(Self {
            measurement,
            status: measurement[0] & 0x7f,
            command: Vec::with_capacity(3),
            read_index: 0,
            conversion_ns: 0,
            protocol_errors: 0,
        })
    }

    pub fn measurement(&self) -> &[u8; 7] {
        &self.measurement
    }

    pub fn is_busy(&self) -> bool {
        self.conversion_ns != 0
    }

    pub fn protocol_errors(&self) -> u64 {
        self.protocol_errors
    }

    fn current_status(&self) -> u8 {
        self.status | if self.is_busy() { 0x80 } else { 0 }
    }

    fn apply_command(&mut self) {
        match self.command.as_slice() {
            [0xbe, 0x08, 0x00] => self.status |= 0x08,
            [0xac, 0x33, 0x00] if !self.is_busy() => {
                self.conversion_ns = AHT20_MEASUREMENT_NS;
            }
            [0xac, 0x33, 0x00] => self.protocol_errors += 1,
            [] => {}
            _ => self.protocol_errors += 1,
        }
        self.command.clear();
    }
}

impl I2cExternalDevice for Aht20 {
    fn model_name(&self) -> &'static str {
        "aht20"
    }

    fn protocol_error_count(&self) -> u64 {
        self.protocol_errors
    }

    fn state_summary(&self) -> String {
        format!(
            "{{\"status\":{},\"busy\":{},\"conversion_ns\":{},\"measurement_sha256\":\"{}\"}}",
            self.current_status(),
            self.is_busy(),
            self.conversion_ns,
            crate::sha256::sha256_hex(&self.measurement),
        )
    }

    fn responds_to(&self, addr: u16) -> bool {
        addr == AHT20_ADDRESS
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        if !self.responds_to(addr) {
            return false;
        }
        self.command.clear();
        self.read_index = 0;
        true
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        if self.command.len() >= 3 {
            self.protocol_errors += 1;
            return false;
        }
        let valid = match self.command.as_slice() {
            [] => matches!(byte, 0xbe | 0xac),
            [0xbe] => byte == 0x08,
            [0xac] => byte == 0x33,
            [0xbe, 0x08] | [0xac, 0x33] => byte == 0x00,
            _ => false,
        };
        if !valid {
            self.protocol_errors += 1;
            return false;
        }
        self.command.push(byte);
        true
    }

    fn read_byte(&mut self) -> u8 {
        let value = if self.read_index == 0 {
            self.current_status()
        } else if self.read_index < self.measurement.len() {
            self.measurement[self.read_index]
        } else {
            self.protocol_errors += 1;
            0xff
        };
        self.read_index += 1;
        value
    }

    fn transaction_end(&mut self) {
        self.apply_command();
        self.read_index = 0;
    }

    fn advance_virtual_time(&mut self, delta: I2cVirtualTimeDelta) {
        self.conversion_ns = self.conversion_ns.saturating_sub(delta.nanoseconds);
    }
}

/// Deterministic BMP280 model for the optional `picocalc-rtc-env-v1` profile.
///
/// Only the register windows exercised by the PicoCalc reference firmware are
/// exposed: chip ID, calibration, measurement, and the two configuration
/// writes that start a forced conversion. Conversion readiness is driven by
/// shared virtual nanoseconds, not host time.
pub struct Bmp280 {
    calibration: [u8; 24],
    measurement: [u8; 6],
    pointer: u8,
    pending_write: Option<(u8, u8)>,
    read_index: usize,
    conversion_ns: u64,
    protocol_errors: u64,
}

impl Bmp280 {
    pub fn new(calibration: [u8; 24], measurement: [u8; 6]) -> Result<Self, String> {
        let dig_p1 = u16::from_le_bytes([calibration[6], calibration[7]]);
        if dig_p1 == 0 {
            return Err("BMP280 calibration dig_p1 must be non-zero".to_string());
        }
        Ok(Self {
            calibration,
            measurement,
            pointer: 0,
            pending_write: None,
            read_index: 0,
            conversion_ns: 0,
            protocol_errors: 0,
        })
    }

    pub fn calibration(&self) -> &[u8; 24] {
        &self.calibration
    }

    pub fn measurement(&self) -> &[u8; 6] {
        &self.measurement
    }

    pub fn is_busy(&self) -> bool {
        self.conversion_ns != 0
    }

    pub fn protocol_errors(&self) -> u64 {
        self.protocol_errors
    }

    fn register_value(&mut self, register: u8) -> u8 {
        match register {
            0xd0 => 0x58,
            0x88..=0x9f => self.calibration[usize::from(register - 0x88)],
            0xf7..=0xfc => {
                if self.is_busy() {
                    self.protocol_errors += 1;
                    // BMP280's unavailable-data sentinel is 0x80000 for
                    // both ADC values, encoded as 80 00 00 per field.
                    [0x80, 0x00, 0x00, 0x80, 0x00, 0x00][usize::from(register - 0xf7)]
                } else {
                    self.measurement[usize::from(register - 0xf7)]
                }
            }
            _ => {
                self.protocol_errors += 1;
                0xff
            }
        }
    }

    fn apply_pending_write(&mut self) {
        let Some((register, value)) = self.pending_write.take() else {
            return;
        };
        match (register, value) {
            (0xf5, 0x00) => {}
            (0xf4, 0x25) => self.conversion_ns = BMP280_MEASUREMENT_NS,
            _ => self.protocol_errors += 1,
        }
    }
}

impl I2cExternalDevice for Bmp280 {
    fn model_name(&self) -> &'static str {
        "bmp280"
    }

    fn protocol_error_count(&self) -> u64 {
        self.protocol_errors
    }

    fn state_summary(&self) -> String {
        format!(
            "{{\"chip_id\":88,\"pointer\":{},\"busy\":{},\"conversion_ns\":{},\"calibration_sha256\":\"{}\",\"measurement_sha256\":\"{}\"}}",
            self.pointer,
            self.is_busy(),
            self.conversion_ns,
            crate::sha256::sha256_hex(&self.calibration),
            crate::sha256::sha256_hex(&self.measurement),
        )
    }

    fn responds_to(&self, addr: u16) -> bool {
        addr == BMP280_ADDRESS
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        if !self.responds_to(addr) {
            return false;
        }
        self.pending_write = None;
        self.read_index = 0;
        true
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        if self.pending_write.is_some() {
            self.protocol_errors += 1;
            return false;
        }
        if self.read_index == 0 {
            self.pointer = byte;
            self.read_index = 1;
            return true;
        }
        if self.pointer != 0xf4 && self.pointer != 0xf5 {
            self.protocol_errors += 1;
            return false;
        }
        self.pending_write = Some((self.pointer, byte));
        true
    }

    fn read_byte(&mut self) -> u8 {
        let value = self.register_value(self.pointer);
        self.pointer = self.pointer.wrapping_add(1);
        self.read_index = self.read_index.saturating_add(1);
        value
    }

    fn transaction_end(&mut self) {
        self.apply_pending_write();
        self.read_index = 0;
    }

    fn advance_virtual_time(&mut self, delta: I2cVirtualTimeDelta) {
        self.conversion_ns = self.conversion_ns.saturating_sub(delta.nanoseconds);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn datetime() -> RtcDateTime {
        RtcDateTime::new(2024, 2, 28, 3, 23, 59, 58).unwrap()
    }

    #[test]
    fn rtc_rolls_over_leap_day_and_year() {
        let mut clock = Ds3231::new(datetime(), false);
        clock.advance_virtual_time(I2cVirtualTimeDelta {
            nanoseconds: 2_000_000_000,
        });
        assert_eq!(clock.datetime().day, 29);
        clock.advance_virtual_time(I2cVirtualTimeDelta {
            nanoseconds: 86_400_000_000_000,
        });
        assert_eq!(clock.datetime().month, 3);
        assert_eq!(clock.datetime().day, 1);
    }

    #[test]
    fn rtc_rejects_unknown_register_and_preserves_address_contract() {
        let mut clock = Ds3231::new(datetime(), true);
        assert!(clock.address_phase(DS3231_ADDRESS));
        assert!(!clock.write_byte(0x13));
        assert_eq!(clock.protocol_errors(), 1);
    }

    #[test]
    fn rtc_accepts_reference_seven_byte_bcd_time_write() {
        let mut clock = Ds3231::new(datetime(), true);
        assert!(clock.address_phase(DS3231_ADDRESS));
        for byte in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x24] {
            assert!(clock.write_byte(byte));
        }
        clock.transaction_end();
        assert_eq!(
            clock.datetime(),
            RtcDateTime::new(2024, 6, 5, 4, 3, 2, 1).unwrap()
        );
        assert_eq!(clock.protocol_errors(), 0);
    }

    #[test]
    fn eeprom_page_write_wraps_and_busy_nacks_until_ready() {
        let mut eeprom = At24c32::new([0; AT24C32_SIZE]);
        assert!(eeprom.address_phase(AT24C32_ADDRESS));
        assert!(eeprom.write_byte(0x00));
        assert!(eeprom.write_byte(0x1F));
        assert!(eeprom.write_byte(0xAA));
        assert!(eeprom.write_byte(0x55));
        eeprom.transaction_end();
        assert_eq!(eeprom.image()[0x1F], 0xAA);
        assert_eq!(eeprom.image()[0x00], 0x55);
        assert!(!eeprom.address_phase(AT24C32_ADDRESS));
        eeprom.advance_virtual_time(I2cVirtualTimeDelta {
            nanoseconds: EEPROM_WRITE_CYCLE_NS,
        });
        assert!(eeprom.address_phase(AT24C32_ADDRESS));
    }

    #[test]
    fn eeprom_pointer_write_without_data_does_not_enter_busy_cycle() {
        let mut eeprom = At24c32::new([0xA5; AT24C32_SIZE]);
        assert!(eeprom.address_phase(AT24C32_ADDRESS));
        assert!(eeprom.write_byte(0x00));
        assert!(eeprom.write_byte(0x10));
        eeprom.transaction_end();
        assert!(!eeprom.is_busy());
        assert_eq!(eeprom.read_byte(), 0xA5);
    }

    #[test]
    fn aht20_validates_crc_and_exposes_reference_command_sequence() {
        let measurement = [0x18, 0x80, 0x00, 0x06, 0x00, 0x00, 0x23];
        assert!(
            Aht20::new({
                let mut invalid = measurement;
                invalid[6] ^= 1;
                invalid
            })
            .is_err()
        );
        assert!(
            Aht20::new({
                let mut invalid = measurement;
                invalid[0] |= 0x80;
                invalid[6] = aht20_crc8(&[
                    invalid[0], invalid[1], invalid[2], invalid[3], invalid[4], invalid[5],
                ]);
                invalid
            })
            .is_err()
        );
        let mut sensor = Aht20::new(measurement).unwrap();
        assert!(sensor.address_phase(AHT20_ADDRESS));
        assert_eq!(sensor.read_byte(), 0x18);
        sensor.transaction_end();

        assert!(sensor.address_phase(AHT20_ADDRESS));
        for byte in [0xbe, 0x08, 0x00] {
            assert!(sensor.write_byte(byte));
        }
        sensor.transaction_end();
        assert!(sensor.address_phase(AHT20_ADDRESS));
        for byte in [0xac, 0x33, 0x00] {
            assert!(sensor.write_byte(byte));
        }
        sensor.transaction_end();
        assert!(sensor.is_busy());

        assert!(sensor.address_phase(AHT20_ADDRESS));
        assert_eq!(sensor.read_byte(), 0x98);
        sensor.transaction_end();
        sensor.advance_virtual_time(I2cVirtualTimeDelta {
            nanoseconds: AHT20_MEASUREMENT_NS,
        });
        assert!(!sensor.is_busy());
        assert!(sensor.address_phase(AHT20_ADDRESS));
        let mut actual = [0u8; 7];
        for byte in &mut actual {
            *byte = sensor.read_byte();
        }
        sensor.transaction_end();
        assert_eq!(actual, measurement);
        assert_eq!(sensor.protocol_errors(), 0);
    }

    #[test]
    fn bmp280_exposes_id_calibration_and_forced_measurement() {
        let calibration = [
            0x70, 0x6b, 0x43, 0x67, 0x18, 0xfc, 0x7d, 0x8e, 0x43, 0xd6, 0xd0, 0x0b, 0x27, 0x0b,
            0x8c, 0x00, 0xf9, 0xff, 0x8c, 0x3c, 0xf8, 0xc6, 0x70, 0x17,
        ];
        let measurement = [0x65, 0x5a, 0xc0, 0x7e, 0xed, 0x00];
        let mut invalid_calibration = calibration;
        invalid_calibration[6] = 0;
        invalid_calibration[7] = 0;
        assert!(Bmp280::new(invalid_calibration, measurement).is_err());
        let mut sensor = Bmp280::new(calibration, measurement).unwrap();

        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert!(sensor.write_byte(0xd0));
        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert_eq!(sensor.read_byte(), 0x58);
        sensor.transaction_end();

        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert!(sensor.write_byte(0x88));
        assert!(sensor.address_phase(BMP280_ADDRESS));
        let mut actual_calibration = [0u8; 24];
        for byte in &mut actual_calibration {
            *byte = sensor.read_byte();
        }
        sensor.transaction_end();
        assert_eq!(actual_calibration, calibration);

        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert!(sensor.write_byte(0xf5));
        assert!(sensor.write_byte(0x00));
        sensor.transaction_end();
        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert!(sensor.write_byte(0xf4));
        assert!(sensor.write_byte(0x25));
        sensor.transaction_end();
        assert!(sensor.is_busy());

        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert!(sensor.write_byte(0xf7));
        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert_eq!(sensor.read_byte(), 0x80);
        sensor.transaction_end();
        assert_eq!(sensor.protocol_errors(), 1);
        sensor.advance_virtual_time(I2cVirtualTimeDelta {
            nanoseconds: BMP280_MEASUREMENT_NS,
        });
        assert!(sensor.address_phase(BMP280_ADDRESS));
        assert!(sensor.write_byte(0xf7));
        assert!(sensor.address_phase(BMP280_ADDRESS));
        let mut actual_measurement = [0u8; 6];
        for byte in &mut actual_measurement {
            *byte = sensor.read_byte();
        }
        sensor.transaction_end();
        assert_eq!(actual_measurement, measurement);
        assert_eq!(sensor.protocol_errors(), 1);
    }
}
