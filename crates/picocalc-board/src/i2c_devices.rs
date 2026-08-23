//! Deterministic models for the first private I2C-EXT devices.
//!
//! These models are intentionally independent of the harness CLI. A profile
//! builder may compose them with [`crate::I2cBusMux`], while downstream users
//! can copy the same child boundary for another private module without
//! changing the RP2040 controller model.

use rp2040_emu::peripherals::i2c::{I2cExternalDevice, I2cVirtualTimeDelta};

pub const DS3231_ADDRESS: u16 = 0x68;
pub const AT24C32_ADDRESS: u16 = 0x57;
const DS3231_REG_STATUS: usize = 0x0f;
pub const AT24C32_SIZE: usize = 4096;
const AT24C32_PAGE_SIZE: u16 = 32;
const EEPROM_WRITE_CYCLE_NS: u64 = 5_000_000;

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
}
