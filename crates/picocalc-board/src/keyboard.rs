//! PicoCalc keyboard/power controller (STM32) as seen over I2C1.
//!
//! The PicoCalc mainboard puts an STM32 between the QWERTY matrix and the
//! Pico. It answers at 7-bit address `0x1F` on I2C1 (GP6 SDA / GP7 SCL).
//! The register set modelled here is the one the two known firmware
//! families actually use:
//!
//! | Register | Direction | Meaning |
//! |----------|-----------|---------|
//! | `0x04`   | read      | Key-FIFO count (Canonical BSP polls this first) |
//! | `0x09`   | read      | Pop one key event: `[state, keycode]` |
//! | `0x0B`   | read      | Battery: `[reg, charge]`, charge = pct \| charging<<7 |
//! | `0x0A`   | write     | Backlight level (sent as `0x8A`, bit 7 = write) |
//!
//! # Transaction shape
//!
//! The official `picocalc_helloworld` driver writes the register byte in
//! one transfer terminated by STOP, waits ~16 ms, then reads two bytes in
//! a separate transfer:
//!
//! ```text
//! START 0x1F+W  0x09  STOP        ... 16 ms ...  START 0x1F+R  b0 b1  STOP
//! ```
//!
//! The Canonical BSP instead uses a repeated START (no STOP between the
//! register write and the read). Both work here because the selected
//! register is remembered until the next register write — a STOP does not
//! clear it. That mirrors the real controller, which has to tolerate the
//! same two access patterns from different firmware.
//!
//! # Key event encoding
//!
//! The firmware reads two bytes into a `uint16_t` on a little-endian
//! core, so byte 0 lands in the low half: `state = buff & 0xFF` and
//! `keycode = buff >> 8`. State `1` is a press, `3` a release, and `2`
//! the hold state used by the modifier reports. An empty FIFO reads back
//! as `0x0000`, which the firmware treats as "no key".

/// 7-bit I2C address of the keyboard controller.
pub const KEYBOARD_I2C_ADDR: u16 = 0x1F;

/// Register: number of queued key events.
pub const REG_KEY_COUNT: u8 = 0x04;
/// Register: pop one key event.
pub const REG_KEY_FIFO: u8 = 0x09;
/// Register: backlight level (written with bit 7 set).
pub const REG_BACKLIGHT: u8 = 0x0A;
/// Register: battery state.
pub const REG_BATTERY: u8 = 0x0B;

/// Write-direction marker the firmware ORs into the register number.
const REG_WRITE_FLAG: u8 = 0x80;

/// Key event state byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyState {
    Pressed = 1,
    Held = 2,
    Released = 3,
}

/// One queued key event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KeyEvent {
    pub state: KeyState,
    pub code: u8,
}

impl KeyEvent {
    pub fn pressed(code: u8) -> Self {
        Self {
            state: KeyState::Pressed,
            code,
        }
    }

    pub fn released(code: u8) -> Self {
        Self {
            state: KeyState::Released,
            code,
        }
    }
}

/// Which register the master selected, and how far into the reply we are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReadState {
    /// No register selected yet — reads return zero.
    Idle,
    /// Serving `reply`, `pos` bytes already handed out.
    Serving { reply: [u8; 2], pos: u8 },
}

/// STM32 keyboard/power controller model.
pub struct Keyboard {
    addr: u16,
    fifo: std::collections::VecDeque<KeyEvent>,
    read_state: ReadState,
    selected: Option<u8>,
    /// Pending register write waiting for its value byte (backlight).
    pending_write_reg: Option<u8>,

    /// Backlight level last written by firmware.
    pub backlight: u8,
    /// Battery percentage reported to firmware.
    pub battery_percent: u8,
    /// Battery flag byte reported to firmware.
    pub battery_flags: u8,

    // --- observation counters (diagnostics only) ---
    pub reg_selects: u64,
    pub key_events_delivered: u64,
    pub battery_reads: u64,
    pub backlight_writes: u64,
    pub unknown_reg_selects: u64,
    pub last_unknown_reg: Option<u8>,
}

impl Keyboard {
    /// Controller at the PicoCalc's address with a plausible battery.
    pub fn picocalc() -> Self {
        Self::at_address(KEYBOARD_I2C_ADDR)
    }

    pub fn at_address(addr: u16) -> Self {
        Self {
            addr,
            fifo: std::collections::VecDeque::new(),
            read_state: ReadState::Idle,
            selected: None,
            pending_write_reg: None,
            backlight: 0,
            // 100% on battery power, no charger flags. The firmware only
            // requires a non-zero word to consider the read successful.
            battery_percent: 100,
            battery_flags: 0,
            reg_selects: 0,
            key_events_delivered: 0,
            battery_reads: 0,
            backlight_writes: 0,
            unknown_reg_selects: 0,
            last_unknown_reg: None,
        }
    }

    /// Queue a press/release pair for `code`, as a real keypress would.
    pub fn press_and_release(&mut self, code: u8) {
        self.fifo.push_back(KeyEvent::pressed(code));
        self.fifo.push_back(KeyEvent::released(code));
    }

    /// Queue one raw event.
    pub fn push_event(&mut self, event: KeyEvent) {
        self.fifo.push_back(event);
    }

    /// Events still waiting to be read.
    pub fn queued(&self) -> usize {
        self.fifo.len()
    }

    /// Build the two-byte reply for the currently selected register.
    fn reply_for(&mut self, reg: u8) -> [u8; 2] {
        match reg {
            REG_KEY_COUNT => [self.fifo.len().min(255) as u8, 0],
            REG_KEY_FIFO => match self.fifo.pop_front() {
                Some(ev) => {
                    self.key_events_delivered += 1;
                    [ev.state as u8, ev.code]
                }
                // Empty FIFO reads as zero; firmware treats it as "no key".
                None => [0, 0],
            },
            REG_BATTERY => {
                self.battery_reads += 1;
                // `test_battery` takes the *high* byte: it does
                // `read_battery() >> 8`, reads bit 7 as the charging
                // flag and the low seven bits as the percentage. The
                // two bytes land in a little-endian u16, so the charge
                // state has to be the second byte on the wire. The
                // first byte is not inspected by any firmware we model;
                // echoing the register keeps the word non-zero.
                let charge = (self.battery_percent & 0x7F) | (self.battery_flags & 0x80);
                [REG_BATTERY, charge]
            }
            // `set_kbd_backlight` treats a zero word as failure and
            // returns -1, so the register echo in the first byte also
            // serves to keep a zero backlight level distinguishable
            // from a dead controller.
            REG_BACKLIGHT => [REG_BACKLIGHT, self.backlight],
            _ => [0, 0],
        }
    }
}

impl rp2040_emu::peripherals::i2c::I2cExternalDevice for Keyboard {
    fn responds_to(&self, addr: u16) -> bool {
        addr == self.addr
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        // Second byte of a two-byte register write (backlight level).
        if let Some(reg) = self.pending_write_reg.take() {
            if reg == REG_BACKLIGHT {
                self.backlight = byte;
                self.backlight_writes += 1;
            }
            // The firmware reads two bytes back after every write and
            // treats a zero word as an error, so leave the register's
            // current value ready to be clocked out.
            let reply = self.reply_for(reg);
            self.read_state = ReadState::Serving { reply, pos: 0 };
            return true;
        }

        if byte & REG_WRITE_FLAG != 0 {
            // Register write: the value byte follows.
            self.pending_write_reg = Some(byte & !REG_WRITE_FLAG);
            return true;
        }

        // Register select for a subsequent read.
        self.reg_selects += 1;
        match byte {
            REG_KEY_COUNT | REG_KEY_FIFO | REG_BATTERY | REG_BACKLIGHT => {}
            other => {
                self.unknown_reg_selects += 1;
                self.last_unknown_reg = Some(other);
            }
        }
        self.selected = Some(byte);
        // Latch the reply now so a repeated START and a STOP-then-START
        // both see the same snapshot.
        let reply = self.reply_for(byte);
        self.read_state = ReadState::Serving { reply, pos: 0 };
        true
    }

    fn read_byte(&mut self) -> u8 {
        match &mut self.read_state {
            ReadState::Idle => 0,
            ReadState::Serving { reply, pos } => {
                let idx = *pos as usize;
                if idx < reply.len() {
                    let byte = reply[idx];
                    *pos += 1;
                    byte
                } else {
                    // Master clocked past the two-byte reply.
                    0
                }
            }
        }
    }

    fn transaction_end(&mut self) {
        // A STOP ends the transfer but not the register selection: the
        // official driver writes the register, stops, waits, then reads.
        self.pending_write_reg = None;
    }
}

/// Shares one [`Keyboard`] between the I2C bus and the harness, so a
/// scenario can queue keys and read counters while the firmware runs.
pub struct KeyboardWire {
    inner: std::sync::Arc<std::sync::Mutex<Keyboard>>,
}

impl KeyboardWire {
    pub fn new(inner: std::sync::Arc<std::sync::Mutex<Keyboard>>) -> Self {
        Self { inner }
    }
}

impl rp2040_emu::peripherals::i2c::I2cExternalDevice for KeyboardWire {
    fn responds_to(&self, addr: u16) -> bool {
        self.inner.lock().expect("keyboard mutex").responds_to(addr)
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        self.inner.lock().expect("keyboard mutex").write_byte(byte)
    }

    fn read_byte(&mut self) -> u8 {
        self.inner.lock().expect("keyboard mutex").read_byte()
    }

    fn transaction_end(&mut self) {
        self.inner
            .lock()
            .expect("keyboard mutex")
            .transaction_end();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rp2040_emu::peripherals::i2c::I2cExternalDevice;

    fn read_word(kbd: &mut Keyboard) -> u16 {
        let lo = kbd.read_byte() as u16;
        let hi = kbd.read_byte() as u16;
        lo | (hi << 8)
    }

    #[test]
    fn answers_only_its_own_address() {
        let kbd = Keyboard::picocalc();
        assert!(kbd.responds_to(0x1F));
        assert!(!kbd.responds_to(0x1E));
        assert!(!kbd.responds_to(0x3C));
    }

    #[test]
    fn empty_fifo_reads_as_zero() {
        let mut kbd = Keyboard::picocalc();
        kbd.write_byte(REG_KEY_FIFO);
        assert_eq!(read_word(&mut kbd), 0);
    }

    #[test]
    fn press_event_decodes_the_way_firmware_reads_it() {
        let mut kbd = Keyboard::picocalc();
        kbd.press_and_release(b'A');

        kbd.write_byte(REG_KEY_FIFO);
        let buff = read_word(&mut kbd);
        // Firmware: state = buff & 0xff, keycode = buff >> 8.
        assert_eq!(buff & 0xFF, KeyState::Pressed as u16);
        assert_eq!(buff >> 8, b'A' as u16);

        kbd.transaction_end();
        kbd.write_byte(REG_KEY_FIFO);
        let buff = read_word(&mut kbd);
        assert_eq!(buff & 0xFF, KeyState::Released as u16);
        assert_eq!(buff >> 8, b'A' as u16);
        assert_eq!(kbd.key_events_delivered, 2);
    }

    #[test]
    fn fifo_drains_in_order_then_returns_zero() {
        let mut kbd = Keyboard::picocalc();
        kbd.push_event(KeyEvent::pressed(b'X'));
        kbd.push_event(KeyEvent::pressed(b'Y'));
        assert_eq!(kbd.queued(), 2);

        kbd.write_byte(REG_KEY_FIFO);
        assert_eq!(read_word(&mut kbd) >> 8, b'X' as u16);
        kbd.write_byte(REG_KEY_FIFO);
        assert_eq!(read_word(&mut kbd) >> 8, b'Y' as u16);
        kbd.write_byte(REG_KEY_FIFO);
        assert_eq!(read_word(&mut kbd), 0);
    }

    #[test]
    fn key_count_register_reports_queue_depth() {
        let mut kbd = Keyboard::picocalc();
        kbd.press_and_release(b'Z');
        kbd.write_byte(REG_KEY_COUNT);
        assert_eq!(read_word(&mut kbd) & 0xFF, 2);
        // Reading the count must not consume events.
        assert_eq!(kbd.queued(), 2);
    }

    #[test]
    fn battery_percentage_lands_where_firmware_looks_for_it() {
        let mut kbd = Keyboard::picocalc();
        kbd.write_byte(REG_BATTERY);
        let buff = read_word(&mut kbd);
        assert_ne!(buff, 0, "firmware discards a zero battery word");
        // test_battery(): bat_pcnt = read_battery() >> 8, bit 7 is the
        // charging flag, the low seven bits are the percentage.
        let high = (buff >> 8) as u8;
        assert_eq!(high & 0x80, 0, "not charging by default");
        assert_eq!(high & 0x7F, 100);
        assert_eq!(kbd.battery_reads, 1);
    }

    #[test]
    fn charging_flag_rides_the_top_bit_of_the_high_byte() {
        let mut kbd = Keyboard::picocalc();
        kbd.battery_percent = 42;
        kbd.battery_flags = 0x80;
        kbd.write_byte(REG_BATTERY);
        let high = (read_word(&mut kbd) >> 8) as u8;
        assert_ne!(high & 0x80, 0, "charging");
        assert_eq!(high & 0x7F, 42);
    }

    #[test]
    fn write_then_read_back_never_reports_a_dead_controller() {
        let mut kbd = Keyboard::picocalc();
        // set_kbd_backlight(0) still has to read back non-zero, or the
        // firmware prints -1 and treats the controller as failed.
        kbd.write_byte(REG_BACKLIGHT | REG_WRITE_FLAG);
        kbd.write_byte(0);
        assert_eq!(kbd.backlight, 0);
        assert_ne!(read_word(&mut kbd), 0);
    }

    #[test]
    fn backlight_write_stores_the_level() {
        let mut kbd = Keyboard::picocalc();
        // Firmware sends 0x0A with bit 7 set, then the value.
        kbd.write_byte(REG_BACKLIGHT | REG_WRITE_FLAG);
        kbd.write_byte(0x50);
        assert_eq!(kbd.backlight, 0x50);
        assert_eq!(kbd.backlight_writes, 1);

        // A follow-up read of the same register reports it back in the
        // high byte, behind the register echo.
        kbd.transaction_end();
        kbd.write_byte(REG_BACKLIGHT);
        assert_eq!(read_word(&mut kbd) >> 8, 0x50);
    }

    #[test]
    fn stop_between_select_and_read_keeps_the_reply() {
        let mut kbd = Keyboard::picocalc();
        kbd.press_and_release(b'Q');
        kbd.write_byte(REG_KEY_FIFO);
        // Official driver: STOP, sleep 16 ms, then a fresh read transfer.
        kbd.transaction_end();
        let buff = read_word(&mut kbd);
        assert_eq!(buff >> 8, b'Q' as u16);
    }

    #[test]
    fn unknown_register_is_counted_not_silently_ignored() {
        let mut kbd = Keyboard::picocalc();
        kbd.write_byte(0x77);
        assert_eq!(kbd.unknown_reg_selects, 1);
        assert_eq!(kbd.last_unknown_reg, Some(0x77));
        assert_eq!(read_word(&mut kbd), 0);
    }

    #[test]
    fn reading_past_the_reply_returns_zero() {
        let mut kbd = Keyboard::picocalc();
        kbd.press_and_release(b'W');
        kbd.write_byte(REG_KEY_FIFO);
        let _ = read_word(&mut kbd);
        assert_eq!(kbd.read_byte(), 0);
    }
}
