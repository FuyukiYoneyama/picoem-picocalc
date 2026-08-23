//! PicoCalc keyboard/power controller (STM32) as seen over I2C1.
//!
//! The PicoCalc mainboard puts an STM32 between the QWERTY matrix and the
//! Pico. It answers at 7-bit address `0x1F` on I2C1 (GP6 SDA / GP7 SCL).
//! The primary behavioral reference is ClockworkPi's official
//! `PicoCalc/Code/picocalc_keyboard` STM32F103R8T6 firmware. RP2040
//! applications are consumer-side evidence. The consumer-visible register,
//! FIFO, modifier, repeat and overflow behaviour is pinned to that source:
//!
//! | Register | Direction | Meaning |
//! |----------|-----------|---------|
//! | `0x01`   | read      | Firmware/BIOS version (`0x16`) |
//! | `0x04`   | read      | Key-FIFO count plus caps/num-lock flags |
//! | `0x05`   | read/write| LCD backlight level |
//! | `0x09`   | read      | Pop one key event: `[state, keycode]` |
//! | `0x0B`   | read      | Battery: `[reg, charge]`, charge = pct \| charging<<7 |
//! | `0x0A`   | read/write| Keyboard backlight level |
//! | `0x0C`   | read      | C64 matrix snapshot (`reg` + 9 bytes) |
//! | `0x0D`   | read      | C64 joystick bits |
//! | `0x0E`   | write     | Delayed power-off request |
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

pub const REG_VERSION: u8 = 0x01;
pub const REG_CONFIG: u8 = 0x02;
pub const REG_INTERRUPT: u8 = 0x03;

/// Register: number of queued key events.
pub const REG_KEY_COUNT: u8 = 0x04;
pub const REG_LCD_BACKLIGHT: u8 = 0x05;
pub const REG_DEBOUNCE: u8 = 0x06;
pub const REG_POLL_FREQUENCY: u8 = 0x07;
pub const REG_RESET: u8 = 0x08;

/// Deepest the key FIFO may get.
///
/// The official controller firmware defines `FIFO_SIZE` as 31 and
/// `KEY_COUNT_MASK` as `0x1F`. The Canonical BSP correspondingly reads the
/// count as `key_info[0] & 0x1f`.
///
/// Leaving the model's queue unbounded made it reachable. A scenario
/// that queued key bursts faster than the firmware consumed them drove
/// the backlog to exactly 224, `224 & 0x1f == 0`, and the driver went
/// permanently blind on an emulator-only state. Found by the PicoTetris
/// line-clear scenario; see `picocalc_emu/docs/SCENARIO_RUNNER.md`.
///
/// The official default configuration leaves `CFG_OVERFLOW_ON` clear, so a
/// full FIFO drops the arriving event. If software enables that bit, the
/// controller overwrites the oldest event while keeping the count at 31.
pub const MAX_QUEUED_EVENTS: usize = 31;
/// Register: pop one key event.
pub const REG_KEY_FIFO: u8 = 0x09;
/// Register: backlight level (written with bit 7 set).
pub const REG_KEYBOARD_BACKLIGHT: u8 = 0x0A;
/// Compatibility name retained for existing consumers.
pub const REG_BACKLIGHT: u8 = REG_KEYBOARD_BACKLIGHT;
/// Register: battery state.
pub const REG_BATTERY: u8 = 0x0B;
pub const REG_C64_MATRIX: u8 = 0x0C;
pub const REG_C64_JOYSTICK: u8 = 0x0D;
pub const REG_POWER_OFF: u8 = 0x0E;

pub const FIRMWARE_VERSION: u8 = 0x16;
pub const CFG_OVERFLOW_ON: u8 = 1 << 0;
pub const CFG_OVERFLOW_INT: u8 = 1 << 1;
pub const CFG_CAPSLOCK_INT: u8 = 1 << 2;
pub const CFG_NUMLOCK_INT: u8 = 1 << 3;
pub const CFG_KEY_INT: u8 = 1 << 4;
pub const CFG_REPORT_MODS: u8 = 1 << 6;
pub const CFG_USE_MODS: u8 = 1 << 7;
pub const DEFAULT_CONFIG: u8 = CFG_OVERFLOW_INT | CFG_KEY_INT | CFG_REPORT_MODS | CFG_USE_MODS;

pub const INT_OVERFLOW: u8 = 1 << 0;
pub const INT_CAPSLOCK: u8 = 1 << 1;
pub const INT_NUMLOCK: u8 = 1 << 2;
pub const INT_KEY: u8 = 1 << 3;

pub const KEY_CAPSLOCK_FLAG: u8 = 1 << 5;
pub const KEY_NUMLOCK_FLAG: u8 = 1 << 6;
pub const KEY_COUNT_MASK: u8 = 0x1F;

pub const KEY_HOLD_TIME_MS: u64 = 300;
pub const KEY_REPEAT_TIME_MS: u64 = 100;

pub const KEY_MOD_ALT: u8 = 0xA1;
pub const KEY_MOD_LEFT_SHIFT: u8 = 0xA2;
pub const KEY_MOD_RIGHT_SHIFT: u8 = 0xA3;
pub const KEY_MOD_SYMBOL: u8 = 0xA4;
pub const KEY_MOD_CONTROL: u8 = 0xA5;
pub const KEY_CAPS_LOCK: u8 = 0xC1;
pub const KEY_INSERT: u8 = 0xD1;

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

    pub fn held(code: u8) -> Self {
        Self {
            state: KeyState::Held,
            code,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modifier {
    Symbol,
    Alt,
    LeftShift,
    RightShift,
    Control,
}

impl Modifier {
    fn code(self) -> u8 {
        match self {
            Modifier::Symbol => KEY_MOD_SYMBOL,
            Modifier::Alt => KEY_MOD_ALT,
            Modifier::LeftShift => KEY_MOD_LEFT_SHIFT,
            Modifier::RightShift => KEY_MOD_RIGHT_SHIFT,
            Modifier::Control => KEY_MOD_CONTROL,
        }
    }

    fn index(self) -> usize {
        match self {
            Modifier::Symbol => 0,
            Modifier::Alt => 1,
            Modifier::LeftShift => 2,
            Modifier::RightShift => 3,
            Modifier::Control => 4,
        }
    }
}

#[derive(Clone, Copy)]
struct KeyMapEntry {
    primary: u8,
    symbol: u8,
    modifier: Option<Modifier>,
}

const fn key(primary: u8, symbol: u8) -> KeyMapEntry {
    KeyMapEntry {
        primary,
        symbol,
        modifier: None,
    }
}

const fn modifier(modifier: Modifier) -> KeyMapEntry {
    KeyMapEntry {
        primary: 0,
        symbol: 0,
        modifier: Some(modifier),
    }
}

/// The official 7x8 GPIO matrix, in source row/column order.
const MATRIX_KEYMAP: [[KeyMapEntry; 8]; 7] = [
    [
        key(0x85, 0x90),
        key(0x84, 0x89),
        key(0x83, 0x88),
        key(0x82, 0x87),
        key(0x81, 0x86),
        key(b'`', b'~'),
        key(b'3', b'#'),
        key(b'2', b'@'),
    ],
    [
        key(0x08, 0),
        key(0xD4, 0xD5),
        key(KEY_CAPS_LOCK, 0),
        key(0x09, 0xD2),
        key(0xB1, 0xD0),
        key(b'4', b'$'),
        key(b'E', 0),
        key(b'W', 0),
    ],
    [
        key(b'P', 0),
        key(b'=', b'+'),
        key(b'-', b'_'),
        key(b'\\', b'|'),
        key(b'/', b'?'),
        key(b'R', 0),
        key(b'S', 0),
        key(b'1', b'!'),
    ],
    [
        key(0x0A, KEY_INSERT),
        key(b'8', b'*'),
        key(b'7', b'&'),
        key(b'6', b'^'),
        key(b'5', b'%'),
        key(b'F', 0),
        key(b'X', 0),
        key(b'Q', 0),
    ],
    [
        key(b'.', b'>'),
        key(b'I', 0),
        key(b'U', 0),
        key(b'Y', 0),
        key(b'T', 0),
        key(b'V', 0),
        key(b';', b':'),
        key(b'A', 0),
    ],
    [
        key(b'L', 0),
        key(b'K', 0),
        key(b'J', 0),
        key(b'H', 0),
        key(b'G', 0),
        key(b'C', 0),
        key(b'\'', b'"'),
        key(b'Z', 0),
    ],
    [
        key(b'O', 0),
        key(b',', b'<'),
        key(b'M', 0),
        key(b'N', 0),
        key(b'B', 0),
        key(b'D', 0),
        key(b' ', 0),
        key(0, 0),
    ],
];

/// The official twelve direct buttons, in source index order.
const BUTTON_KEYMAP: [KeyMapEntry; 12] = [
    modifier(Modifier::Alt),
    modifier(Modifier::Control),
    modifier(Modifier::LeftShift),
    modifier(Modifier::RightShift),
    key(b'0', b')'),
    key(b'9', b'('),
    key(b']', b'}'),
    key(b'[', b'{'),
    key(0xB7, 0),
    key(0xB5, 0xD6),
    key(0xB6, 0xD7),
    key(0xB4, 0),
];

/// Which register the master selected, and how far into the reply we are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReadState {
    /// No register selected yet — reads return zero.
    Idle,
    /// Serving `reply`, `pos` bytes already handed out.
    Serving { reply: [u8; 10], len: u8, pos: u8 },
}

#[derive(Clone, Copy)]
struct RegisterReply {
    bytes: [u8; 10],
    len: u8,
}

impl RegisterReply {
    fn two(first: u8, second: u8) -> Self {
        let mut bytes = [0; 10];
        bytes[0] = first;
        bytes[1] = second;
        Self { bytes, len: 2 }
    }
}

/// STM32 keyboard/power controller model.
pub struct Keyboard {
    addr: u16,
    fifo: std::collections::VecDeque<KeyEvent>,
    read_state: ReadState,
    selected: Option<u8>,
    /// Pending register write waiting for its value byte (backlight).
    pending_write_reg: Option<u8>,

    /// Keyboard backlight level last written by firmware.
    pub backlight: u8,
    /// LCD backlight level last written by firmware.
    pub lcd_backlight: u8,
    /// Internal default configuration from `reg_init()`. The official I2C
    /// switch has no CFG case, so consumers cannot rewrite this at present.
    pub config: u8,
    /// Internal interrupt latch. The official I2C switch likewise has no
    /// readable INT case, but the state controls conformance diagnostics.
    pub interrupt_status: u8,
    pub caps_lock: bool,
    pub num_lock: bool,
    active_modifiers: [bool; 5],
    /// Battery percentage reported to firmware.
    pub battery_percent: u8,
    /// Battery flag byte reported to firmware.
    pub battery_flags: u8,
    pub c64_matrix: [u8; 9],
    pub c64_joystick: u8,
    pub power_off_delay_s: Option<u8>,
    pub reset_requests: u64,

    // --- observation counters (diagnostics only) ---
    pub reg_selects: u64,
    pub key_events_delivered: u64,
    /// Events discarded because the controller was already full. Non-zero
    /// means input was queued faster than the firmware drained it, and
    /// whatever the run then observed is not what the queued keys meant.
    pub key_events_dropped: u64,
    pub key_events_overwritten: u64,
    pub battery_reads: u64,
    pub backlight_writes: u64,
    pub unknown_reg_selects: u64,
    pub unknown_reg_writes: u64,
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
            lcd_backlight: 32,
            config: DEFAULT_CONFIG,
            interrupt_status: 0,
            caps_lock: false,
            num_lock: false,
            active_modifiers: [false; 5],
            // 100% on battery power, no charger flags. The firmware only
            // requires a non-zero word to consider the read successful.
            battery_percent: 100,
            battery_flags: 0,
            c64_matrix: [0; 9],
            c64_joystick: 0xFF,
            power_off_delay_s: None,
            reset_requests: 0,
            reg_selects: 0,
            key_events_delivered: 0,
            key_events_dropped: 0,
            key_events_overwritten: 0,
            battery_reads: 0,
            backlight_writes: 0,
            unknown_reg_selects: 0,
            unknown_reg_writes: 0,
            last_unknown_reg: None,
        }
    }

    /// Queue a press/release pair for `code`, as a real keypress would.
    ///
    /// The two events are queued independently, so a controller with one
    /// slot left keeps the press and loses the release — the same way a
    /// real FIFO would fill mid-keystroke.
    pub fn press_and_release(&mut self, code: u8) {
        self.push_event(KeyEvent::pressed(code));
        self.push_event(KeyEvent::released(code));
    }

    /// Set the controller's internal configuration for conformance tests.
    /// The official source has no I2C CFG case, so this is deliberately not
    /// exposed as a register write.
    pub fn set_internal_config(&mut self, config: u8) {
        self.config = config;
    }

    pub fn set_num_lock(&mut self, enabled: bool) {
        self.num_lock = enabled;
    }

    pub fn set_caps_lock(&mut self, enabled: bool) {
        self.caps_lock = enabled;
    }

    /// Emit a physical modifier transition using the official modifier
    /// keycodes. Modifier reporting is controlled by CFG_REPORT_MODS.
    pub fn modifier_event(&mut self, modifier: Modifier, state: KeyState) {
        if state == KeyState::Pressed {
            self.active_modifiers[modifier.index()] = true;
        }
        if self.config & CFG_REPORT_MODS != 0 {
            self.push_event(KeyEvent {
                state,
                code: modifier.code(),
            });
        }
        if state == KeyState::Released {
            self.active_modifiers[modifier.index()] = false;
        }
    }

    /// Inject a transition by the official 7x8 matrix coordinates.
    /// Returns false for a coordinate outside the firmware table.
    pub fn physical_matrix_event(&mut self, row: usize, column: usize, state: KeyState) -> bool {
        let Some(entry) = MATRIX_KEYMAP
            .get(row)
            .and_then(|entries| entries.get(column))
            .copied()
        else {
            return false;
        };
        self.dispatch_physical_entry(entry, state);
        true
    }

    /// Inject a transition by the official direct-button index.
    pub fn physical_button_event(&mut self, index: usize, state: KeyState) -> bool {
        let Some(entry) = BUTTON_KEYMAP.get(index).copied() else {
            return false;
        };
        self.dispatch_physical_entry(entry, state);
        true
    }

    fn dispatch_physical_entry(&mut self, entry: KeyMapEntry, state: KeyState) {
        if let Some(modifier) = entry.modifier {
            self.modifier_event(modifier, state);
        } else {
            self.mapped_key_event(entry.primary, entry.symbol, state);
        }
    }

    /// Translate one physical key transition. `primary` and `symbol` are
    /// the two columns in the official keyboard matrix table.
    pub fn mapped_key_event(&mut self, primary: u8, symbol: u8, state: KeyState) {
        let mut code = primary;
        let mut output = true;

        if primary == KEY_CAPS_LOCK && state == KeyState::Pressed {
            self.caps_lock = !self.caps_lock;
        }

        if self.config & CFG_USE_MODS != 0 {
            let shift = self.active_modifiers[Modifier::LeftShift.index()]
                || self.active_modifiers[Modifier::RightShift.index()];
            let alt = self.active_modifiers[Modifier::Alt.index()] || self.num_lock;
            if shift && !primary.is_ascii_uppercase() {
                code = symbol;
            } else if self.caps_lock && primary.is_ascii_uppercase() {
                // Caps lock leaves the matrix's uppercase primary value.
            } else if alt {
                // The official shortcut branches only inspect Pressed and
                // Released. Held falls through to the ordinary repeat path.
                if state != KeyState::Held {
                    match primary {
                        b',' => {
                            output = false;
                            if state == KeyState::Released {
                                self.lcd_backlight = self.lcd_backlight.saturating_sub(16).max(16);
                            }
                        }
                        b'.' => {
                            output = false;
                            if state == KeyState::Released {
                                self.lcd_backlight = self.lcd_backlight.saturating_add(16).min(240);
                            }
                        }
                        b' ' => {
                            output = false;
                            if state == KeyState::Released {
                                let next = self.backlight as u16 + 32;
                                self.backlight = if next > 240 { 0 } else { next as u8 };
                            }
                        }
                        b'B' => output = false,
                        b'I' => code = KEY_INSERT,
                        _ => {}
                    }
                }
            } else if !shift && primary.is_ascii_uppercase() {
                code = primary.to_ascii_lowercase();
            }
        }

        if output && code != 0 {
            let output_state = if state == KeyState::Held && is_repeatable(code) {
                KeyState::Pressed
            } else {
                state
            };
            self.push_event(KeyEvent {
                state: output_state,
                code,
            });
        }
    }

    /// Queue one raw event, discarding it if the controller is full.
    ///
    /// See [`MAX_QUEUED_EVENTS`]: silently growing past the depth the
    /// count register can express would let firmware reach a state real
    /// hardware cannot produce.
    pub fn push_event(&mut self, event: KeyEvent) {
        if self.config & CFG_KEY_INT != 0 {
            self.interrupt_status |= INT_KEY;
        }
        if self.fifo.len() >= MAX_QUEUED_EVENTS {
            self.key_events_dropped += 1;
            if self.config & CFG_OVERFLOW_INT != 0 {
                self.interrupt_status |= INT_OVERFLOW;
            }
            if self.config & CFG_OVERFLOW_ON != 0 {
                self.fifo.pop_front();
                self.fifo.push_back(event);
                self.key_events_overwritten += 1;
            }
            return;
        }
        self.fifo.push_back(event);
    }

    /// Events still waiting to be read.
    pub fn queued(&self) -> usize {
        self.fifo.len()
    }

    /// Whether the official scanner would emit another `Held` transition.
    /// Its comparisons are strictly greater-than, not greater-than-or-equal.
    pub fn repeat_due(held_ms: u64, since_last_repeat_ms: u64) -> bool {
        held_ms > KEY_HOLD_TIME_MS && since_last_repeat_ms > KEY_REPEAT_TIME_MS
    }

    fn reset_controller_state(&mut self) {
        self.fifo.clear();
        self.read_state = ReadState::Idle;
        self.selected = None;
        self.pending_write_reg = None;
        self.backlight = 0;
        self.lcd_backlight = 32;
        self.config = DEFAULT_CONFIG;
        self.interrupt_status = 0;
        self.caps_lock = false;
        self.num_lock = false;
        self.active_modifiers = [false; 5];
        self.power_off_delay_s = None;
    }

    /// Build the official reply buffer for the selected register.
    fn reply_for(&mut self, reg: u8) -> RegisterReply {
        match reg {
            REG_VERSION => RegisterReply::two(0, FIRMWARE_VERSION),
            REG_KEY_COUNT => {
                let status = (self.fifo.len() as u8 & KEY_COUNT_MASK)
                    | if self.caps_lock { KEY_CAPSLOCK_FLAG } else { 0 }
                    | if self.num_lock { KEY_NUMLOCK_FLAG } else { 0 };
                RegisterReply::two(status, 0)
            }
            REG_KEY_FIFO => match self.fifo.pop_front() {
                Some(ev) => {
                    self.key_events_delivered += 1;
                    RegisterReply::two(ev.state as u8, ev.code)
                }
                // Empty FIFO reads as zero; firmware treats it as "no key".
                None => RegisterReply::two(0, 0),
            },
            REG_LCD_BACKLIGHT => RegisterReply::two(REG_LCD_BACKLIGHT, self.lcd_backlight),
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
                RegisterReply::two(REG_BATTERY, charge)
            }
            // `set_kbd_backlight` treats a zero word as failure and
            // returns -1, so the register echo in the first byte also
            // serves to keep a zero backlight level distinguishable
            // from a dead controller.
            REG_KEYBOARD_BACKLIGHT => RegisterReply::two(REG_KEYBOARD_BACKLIGHT, self.backlight),
            REG_C64_MATRIX => {
                let mut bytes = [0; 10];
                bytes[0] = REG_C64_MATRIX;
                bytes[1..].copy_from_slice(&self.c64_matrix);
                RegisterReply { bytes, len: 10 }
            }
            REG_C64_JOYSTICK => RegisterReply::two(REG_C64_JOYSTICK, self.c64_joystick),
            REG_POWER_OFF => RegisterReply::two(REG_POWER_OFF, 1),
            REG_RESET => {
                self.reset_requests += 1;
                self.reset_controller_state();
                RegisterReply::two(0, 0)
            }
            // CFG, INT, DEB and FRQ exist internally, but the official
            // receiveEvent switch has no case and returns two zero bytes.
            REG_CONFIG | REG_INTERRUPT | REG_DEBOUNCE | REG_POLL_FREQUENCY => {
                RegisterReply::two(0, 0)
            }
            _ => RegisterReply::two(0, 0),
        }
    }
}

fn is_repeatable(code: u8) -> bool {
    (32..=127).contains(&code)
        || matches!(code, 0x0A | 0x09 | 0xD4 | 0x08 | 0xB5 | 0xB6 | 0xB7 | 0xB4)
}

fn is_known_register(reg: u8) -> bool {
    matches!(reg, REG_VERSION..=REG_POWER_OFF)
}

fn normalize_lcd_backlight(value: u8) -> u8 {
    (value / 16 * 16).clamp(16, 240)
}

fn normalize_keyboard_backlight(value: u8) -> u8 {
    let quantized = value / 32 * 32;
    if !(32..=240).contains(&quantized) {
        0
    } else {
        quantized
    }
}

impl rp2040_emu::peripherals::i2c::I2cExternalDevice for Keyboard {
    fn responds_to(&self, addr: u16) -> bool {
        addr == self.addr
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        self.responds_to(addr)
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        // Second byte of a two-byte register write.
        if let Some(reg) = self.pending_write_reg.take() {
            match reg {
                REG_LCD_BACKLIGHT => {
                    self.lcd_backlight = normalize_lcd_backlight(byte);
                    self.backlight_writes += 1;
                }
                REG_KEYBOARD_BACKLIGHT => {
                    self.backlight = normalize_keyboard_backlight(byte);
                    self.backlight_writes += 1;
                }
                REG_POWER_OFF => self.power_off_delay_s = Some(byte.max(6)),
                // The selected register's reply builder performs reset.
                REG_RESET => {}
                // These official cases ignore the write flag but still
                // prepare their ordinary reply (FIFO therefore still pops).
                REG_VERSION | REG_KEY_COUNT | REG_KEY_FIFO | REG_BATTERY | REG_C64_MATRIX
                | REG_C64_JOYSTICK => {}
                _ => {
                    self.unknown_reg_writes += 1;
                    self.last_unknown_reg = Some(reg);
                }
            }
            // The firmware reads two bytes back after every write and
            // treats a zero word as an error, so leave the register's
            // current value ready to be clocked out.
            let reply = self.reply_for(reg);
            self.read_state = ReadState::Serving {
                reply: reply.bytes,
                len: reply.len,
                pos: 0,
            };
            return true;
        }

        if byte & REG_WRITE_FLAG != 0 {
            // Register write: the value byte follows.
            self.pending_write_reg = Some(byte & !REG_WRITE_FLAG);
            return true;
        }

        // Register select for a subsequent read.
        self.reg_selects += 1;
        if !is_known_register(byte) {
            self.unknown_reg_selects += 1;
            self.last_unknown_reg = Some(byte);
        }
        self.selected = Some(byte);
        // Latch the reply now so a repeated START and a STOP-then-START
        // both see the same snapshot.
        let reply = self.reply_for(byte);
        self.read_state = ReadState::Serving {
            reply: reply.bytes,
            len: reply.len,
            pos: 0,
        };
        true
    }

    fn read_byte(&mut self) -> u8 {
        match &mut self.read_state {
            ReadState::Idle => 0,
            ReadState::Serving { reply, len, pos } => {
                let idx = *pos as usize;
                if idx < *len as usize {
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
        if let ReadState::Serving { pos, .. } = &mut self.read_state {
            *pos = 0;
        }
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
    fn model_name(&self) -> &'static str {
        "picocalc-keyboard"
    }

    fn protocol_error_count(&self) -> u64 {
        let keyboard = self.inner.lock().expect("keyboard mutex");
        keyboard.unknown_reg_selects + keyboard.unknown_reg_writes
    }

    fn state_summary(&self) -> String {
        let keyboard = self.inner.lock().expect("keyboard mutex");
        format!(
            "{{\"queued\":{},\"delivered\":{},\"dropped\":{},\"unknown_reg_selects\":{},\"unknown_reg_writes\":{}}}",
            keyboard.queued(),
            keyboard.key_events_delivered,
            keyboard.key_events_dropped,
            keyboard.unknown_reg_selects,
            keyboard.unknown_reg_writes,
        )
    }

    fn responds_to(&self, addr: u16) -> bool {
        self.inner.lock().expect("keyboard mutex").responds_to(addr)
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        self.inner
            .lock()
            .expect("keyboard mutex")
            .address_phase(addr)
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        self.inner.lock().expect("keyboard mutex").write_byte(byte)
    }

    fn read_byte(&mut self) -> u8 {
        self.inner.lock().expect("keyboard mutex").read_byte()
    }

    fn transaction_end(&mut self) {
        self.inner.lock().expect("keyboard mutex").transaction_end();
    }
}

#[cfg(test)]
mod overflow_tests {
    use super::*;
    use rp2040_emu::peripherals::i2c::I2cExternalDevice;

    /// Read the count register the way the Canonical BSP does: select
    /// `0x04`, take two bytes, mask the first with `0x1f`.
    fn bsp_key_count(kbd: &mut Keyboard) -> u8 {
        kbd.write_byte(REG_KEY_COUNT);
        let low = kbd.read_byte();
        let _high = kbd.read_byte();
        low & 0x1f
    }

    #[test]
    fn the_queue_never_exceeds_what_the_count_register_can_express() {
        let mut kbd = Keyboard::picocalc();
        for i in 0..200u32 {
            kbd.press_and_release(b'a' + (i % 26) as u8);
        }
        assert_eq!(kbd.queued(), MAX_QUEUED_EVENTS);
        assert_eq!(kbd.key_events_dropped, 400 - MAX_QUEUED_EVENTS as u64);
    }

    /// The regression this bound exists for. An unbounded queue could
    /// reach a multiple of 32, whereupon the BSP's `& 0x1f` reads zero
    /// and the driver stops draining for good — on a controller that is
    /// in fact full.
    #[test]
    fn a_full_controller_never_reports_itself_empty() {
        let mut kbd = Keyboard::picocalc();
        for i in 0..500u32 {
            kbd.push_event(KeyEvent::pressed(b'a' + (i % 26) as u8));
            assert_ne!(
                bsp_key_count(&mut kbd),
                0,
                "after {} events the BSP would see an empty controller",
                i + 1
            );
        }
    }

    #[test]
    fn a_drained_controller_still_reports_empty() {
        let mut kbd = Keyboard::picocalc();
        assert_eq!(bsp_key_count(&mut kbd), 0);
        kbd.press_and_release(b'x');
        assert_eq!(bsp_key_count(&mut kbd), 2);
        for _ in 0..2 {
            kbd.write_byte(REG_KEY_FIFO);
            kbd.read_byte();
            kbd.read_byte();
        }
        assert_eq!(bsp_key_count(&mut kbd), 0);
        assert_eq!(kbd.key_events_dropped, 0);
    }

    #[test]
    fn a_controller_with_one_slot_left_keeps_the_press_and_loses_the_release() {
        let mut kbd = Keyboard::picocalc();
        for _ in 0..MAX_QUEUED_EVENTS - 1 {
            kbd.push_event(KeyEvent::pressed(b'z'));
        }
        kbd.press_and_release(b'q');
        assert_eq!(kbd.queued(), MAX_QUEUED_EVENTS);
        assert_eq!(kbd.key_events_dropped, 1);
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

    fn read_event(kbd: &mut Keyboard) -> KeyEvent {
        kbd.write_byte(REG_KEY_FIFO);
        let word = read_word(kbd);
        let state = match word as u8 {
            1 => KeyState::Pressed,
            2 => KeyState::Held,
            3 => KeyState::Released,
            other => panic!("unexpected key state {other}"),
        };
        KeyEvent {
            state,
            code: (word >> 8) as u8,
        }
    }

    fn write_register(kbd: &mut Keyboard, reg: u8, value: u8) {
        kbd.write_byte(reg | REG_WRITE_FLAG);
        kbd.write_byte(value);
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
        assert_eq!(kbd.backlight, 0x40);
        assert_eq!(kbd.backlight_writes, 1);

        // A follow-up read of the same register reports it back in the
        // high byte, behind the register echo.
        kbd.transaction_end();
        kbd.write_byte(REG_BACKLIGHT);
        assert_eq!(read_word(&mut kbd) >> 8, 0x40);
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

    #[test]
    fn official_version_and_internal_only_registers_match_source() {
        let mut kbd = Keyboard::picocalc();
        kbd.write_byte(REG_VERSION);
        assert_eq!(read_word(&mut kbd), (FIRMWARE_VERSION as u16) << 8);
        assert_eq!(kbd.config, 0xD2);

        for reg in [REG_CONFIG, REG_INTERRUPT, REG_DEBOUNCE, REG_POLL_FREQUENCY] {
            kbd.write_byte(reg);
            assert_eq!(read_word(&mut kbd), 0, "register {reg:#04x}");
        }

        write_register(&mut kbd, REG_CONFIG, 0xFF);
        assert_eq!(kbd.config, DEFAULT_CONFIG);
        assert_eq!(kbd.unknown_reg_writes, 1);
        assert_eq!(kbd.last_unknown_reg, Some(REG_CONFIG));
    }

    #[test]
    fn count_register_includes_caps_and_num_lock_flags() {
        let mut kbd = Keyboard::picocalc();
        kbd.push_event(KeyEvent::pressed(b'a'));
        kbd.set_caps_lock(true);
        kbd.set_num_lock(true);
        kbd.write_byte(REG_KEY_COUNT);
        assert_eq!(
            read_word(&mut kbd) as u8,
            1 | KEY_CAPSLOCK_FLAG | KEY_NUMLOCK_FLAG
        );
    }

    #[test]
    fn both_backlights_use_the_official_quantisation() {
        let mut kbd = Keyboard::picocalc();
        write_register(&mut kbd, REG_LCD_BACKLIGHT, 0);
        assert_eq!(kbd.lcd_backlight, 16);
        write_register(&mut kbd, REG_LCD_BACKLIGHT, 0xFF);
        assert_eq!(kbd.lcd_backlight, 240);

        write_register(&mut kbd, REG_KEYBOARD_BACKLIGHT, 31);
        assert_eq!(kbd.backlight, 0);
        write_register(&mut kbd, REG_KEYBOARD_BACKLIGHT, 0x50);
        assert_eq!(kbd.backlight, 64);
        write_register(&mut kbd, REG_KEYBOARD_BACKLIGHT, 0xFF);
        assert_eq!(kbd.backlight, 224);
    }

    #[test]
    fn c64_power_and_reset_registers_follow_the_wire_contract() {
        let mut kbd = Keyboard::picocalc();
        kbd.c64_matrix = [1, 2, 3, 4, 5, 6, 7, 8, 9];
        kbd.c64_joystick = 0xA5;

        kbd.write_byte(REG_C64_MATRIX);
        let mut matrix_reply = [0; 10];
        for byte in &mut matrix_reply {
            *byte = kbd.read_byte();
        }
        assert_eq!(matrix_reply, [REG_C64_MATRIX, 1, 2, 3, 4, 5, 6, 7, 8, 9]);

        kbd.write_byte(REG_C64_JOYSTICK);
        assert_eq!(read_word(&mut kbd), 0xA50D);
        write_register(&mut kbd, REG_POWER_OFF, 2);
        assert_eq!(kbd.power_off_delay_s, Some(6));
        assert_eq!(read_word(&mut kbd), 0x010E);

        kbd.push_event(KeyEvent::pressed(b'x'));
        kbd.set_caps_lock(true);
        kbd.write_byte(REG_RESET);
        assert_eq!(kbd.reset_requests, 1);
        assert_eq!(kbd.queued(), 0);
        assert!(!kbd.caps_lock);
        assert_eq!(kbd.config, DEFAULT_CONFIG);
    }

    #[test]
    fn a_new_read_request_resends_the_latched_reply_from_byte_zero() {
        let mut kbd = Keyboard::picocalc();
        kbd.write_byte(REG_VERSION);
        assert_eq!(read_word(&mut kbd), 0x1600);
        kbd.transaction_end();
        assert_eq!(read_word(&mut kbd), 0x1600);
    }

    #[test]
    fn official_matrix_mapping_modifiers_and_caps_are_preserved() {
        let mut kbd = Keyboard::picocalc();
        // Matrix row 4/column 7 is A. Unshifted letters are lowercase.
        assert!(kbd.physical_matrix_event(4, 7, KeyState::Pressed));
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(b'a'));

        // Direct button 2 is left shift and is reported as 0xA2.
        assert!(kbd.physical_button_event(2, KeyState::Pressed));
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(KEY_MOD_LEFT_SHIFT));
        assert!(kbd.physical_matrix_event(0, 6, KeyState::Pressed));
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(b'#'));

        let mut caps = Keyboard::picocalc();
        assert!(caps.physical_matrix_event(1, 2, KeyState::Pressed));
        assert!(caps.caps_lock);
        assert_eq!(read_event(&mut caps), KeyEvent::pressed(KEY_CAPS_LOCK));
        assert!(caps.physical_matrix_event(4, 7, KeyState::Pressed));
        assert_eq!(read_event(&mut caps), KeyEvent::pressed(b'A'));
        assert!(!caps.physical_matrix_event(7, 0, KeyState::Pressed));
    }

    #[test]
    fn official_modifier_report_codes_are_exact() {
        let mut kbd = Keyboard::picocalc();
        for (modifier, code) in [
            (Modifier::Alt, KEY_MOD_ALT),
            (Modifier::LeftShift, KEY_MOD_LEFT_SHIFT),
            (Modifier::RightShift, KEY_MOD_RIGHT_SHIFT),
            (Modifier::Symbol, KEY_MOD_SYMBOL),
            (Modifier::Control, KEY_MOD_CONTROL),
        ] {
            kbd.modifier_event(modifier, KeyState::Pressed);
            assert_eq!(read_event(&mut kbd), KeyEvent::pressed(code));
            kbd.modifier_event(modifier, KeyState::Released);
            assert_eq!(read_event(&mut kbd), KeyEvent::released(code));
        }
    }

    #[test]
    fn alt_shortcuts_and_hold_repeat_match_the_official_transition() {
        let mut kbd = Keyboard::picocalc();
        kbd.set_internal_config(CFG_USE_MODS);
        kbd.modifier_event(Modifier::Alt, KeyState::Pressed);
        kbd.mapped_key_event(b'I', 0, KeyState::Pressed);
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(KEY_INSERT));

        kbd.mapped_key_event(b' ', 0, KeyState::Pressed);
        assert_eq!(kbd.queued(), 0);
        kbd.mapped_key_event(b' ', 0, KeyState::Released);
        assert_eq!(kbd.backlight, 32);
        assert_eq!(kbd.queued(), 0);

        // The source only applies Alt shortcuts on Pressed/Released;
        // Held follows the ordinary repeatable-key path.
        kbd.mapped_key_event(b' ', 0, KeyState::Held);
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(b' '));
        kbd.mapped_key_event(b'I', 0, KeyState::Held);
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(b'I'));

        kbd.modifier_event(Modifier::Alt, KeyState::Released);
        kbd.mapped_key_event(b'A', 0, KeyState::Held);
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(b'a'));
        kbd.mapped_key_event(0x81, 0x86, KeyState::Held);
        assert_eq!(read_event(&mut kbd), KeyEvent::held(0x81));

        assert!(!Keyboard::repeat_due(300, 101));
        assert!(!Keyboard::repeat_due(301, 100));
        assert!(Keyboard::repeat_due(301, 101));
    }

    #[test]
    fn configured_overflow_overwrites_oldest_and_latches_interrupt() {
        let mut kbd = Keyboard::picocalc();
        kbd.set_internal_config(DEFAULT_CONFIG | CFG_OVERFLOW_ON);
        for code in 0..MAX_QUEUED_EVENTS as u8 {
            kbd.push_event(KeyEvent::pressed(code));
        }
        kbd.push_event(KeyEvent::pressed(99));
        assert_eq!(kbd.queued(), MAX_QUEUED_EVENTS);
        assert_eq!(kbd.key_events_dropped, 1);
        assert_eq!(kbd.key_events_overwritten, 1);
        assert_ne!(kbd.interrupt_status & INT_OVERFLOW, 0);
        assert_eq!(read_event(&mut kbd), KeyEvent::pressed(1));
    }

    #[test]
    fn unknown_register_writes_are_observable() {
        let mut kbd = Keyboard::picocalc();
        write_register(&mut kbd, 0x77, 0x12);
        assert_eq!(kbd.unknown_reg_writes, 1);
        assert_eq!(kbd.last_unknown_reg, Some(0x77));
    }

    #[test]
    fn write_flag_on_an_official_read_case_is_not_a_protocol_error() {
        let mut kbd = Keyboard::picocalc();
        write_register(&mut kbd, REG_VERSION, 0xAA);
        assert_eq!(read_word(&mut kbd), 0x1600);
        assert_eq!(kbd.unknown_reg_writes, 0);

        kbd.push_event(KeyEvent::pressed(b'q'));
        write_register(&mut kbd, REG_KEY_FIFO, 0);
        assert_eq!(read_word(&mut kbd), 0x7101);
        assert_eq!(kbd.unknown_reg_writes, 0);
    }
}
