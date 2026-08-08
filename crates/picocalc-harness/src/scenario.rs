//! Scenario engine: timed input and mechanical screen checks.
//!
//! # Why this exists
//!
//! `--keys` queues a fixed string into the keyboard FIFO before the
//! firmware starts, and nothing after that can decide an input from what
//! the program is doing. Writing a game against the emulator
//! (`picocalc_emu/docs/DOGFOODING_20260805.md`) showed what that costs:
//! the line-clearing path could not be made to fire at all, because
//! "wait until the piece lands, then move left three and drop" is not
//! expressible. The screen side had the same shape of gap — a PNG came
//! out and a human decided whether it looked right.
//!
//! A scenario closes both. It is a list of steps evaluated *inside* the
//! run loop, so a step can look at the panel and the UART stream before
//! choosing to inject the next key.
//!
//! # Time
//!
//! Milliseconds here are **virtual**: they come from the emulated cycle
//! count divided by the system clock the firmware has currently
//! programmed. The conversion is re-based whenever `clk_sys` changes, so
//! a scenario that starts before the PLL locks still measures real
//! firmware time rather than boot-ROSC time. No wall clock is read
//! anywhere — the same scenario against the same image must produce the
//! same report.
//!
//! # Cost
//!
//! Conditions are evaluated on a cadence (`poll_ms`, default
//! [`DEFAULT_POLL_MS`]), not every cycle. Hashing a region is linear in
//! its area, so a scenario that watches a 140x280 well at 5 ms costs
//! about 8 million pixel reads per emulated second. Timed waits are
//! exact regardless: the engine asks to be polled at the deadline
//! itself, not at the next cadence tick.

use std::path::{Path, PathBuf};
use std::sync::Mutex;

use picocalc_board::sha256::sha256_hex;
use picocalc_board::{KeyEvent, KeyState, Keyboard, St7365p, pins};

use serde_json::Value;

/// Scenario file schema version. Bump on any breaking change.
pub const SCENARIO_SCHEMA: u64 = 1;

/// Default condition-evaluation cadence, in virtual milliseconds.
///
/// 5 ms is a third of a 60 Hz frame: fast enough that a scenario
/// watching for a redraw sees it in the frame it happened, slow enough
/// that region hashing stays a rounding error against the emulation
/// itself.
pub const DEFAULT_POLL_MS: u64 = 5;

/// Upper bound on a single `wait`/`wait_until`, in virtual
/// milliseconds. A scenario that asks for longer has almost certainly
/// mistyped a unit, and the cycle budget would expire first anyway.
const MAX_WAIT_MS: u64 = 10 * 60 * 1000;

// ---------------------------------------------------------------------
// Scenario model
// ---------------------------------------------------------------------

/// A rectangle in viewport coordinates. Validated against the 320x320
/// visible window at parse time, so evaluation never has to bounds-check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rect {
    pub x: usize,
    pub y: usize,
    pub w: usize,
    pub h: usize,
}

impl Rect {
    fn describe(&self) -> String {
        format!("({},{}) {}x{}", self.x, self.y, self.w, self.h)
    }
}

/// Something that can be true or false about the machine's observable
/// state at a moment in time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Condition {
    /// One viewport pixel holds (or does not hold) an RGB565 value.
    Pixel {
        x: usize,
        y: usize,
        equals: Option<u16>,
        not_equals: Option<u16>,
    },
    /// The count of non-black pixels in a region falls in a range.
    RegionNonBlack {
        rect: Rect,
        min: Option<usize>,
        max: Option<usize>,
    },
    /// A region hashes to a pinned value.
    RegionHash { rect: Rect, equals: String },
    /// A region has not changed for `for_ms`. Stateful: only valid
    /// inside `wait_until`.
    RegionStable { rect: Rect, for_ms: u64 },
    /// A region differs from what it was when the step began. Stateful:
    /// only valid inside `wait_until`.
    RegionChanged { rect: Rect },
    /// The UART transmit stream contains a byte sequence.
    UartContains { text: String },
}

impl Condition {
    fn kind(&self) -> &'static str {
        match self {
            Condition::Pixel { .. } => "pixel",
            Condition::RegionNonBlack { .. } => "region_non_black",
            Condition::RegionHash { .. } => "region_hash",
            Condition::RegionStable { .. } => "region_stable",
            Condition::RegionChanged { .. } => "region_changed",
            Condition::UartContains { .. } => "uart_contains",
        }
    }

    /// True for conditions that only mean something across time, and so
    /// cannot be asserted at a single instant.
    fn is_stateful(&self) -> bool {
        matches!(
            self,
            Condition::RegionStable { .. } | Condition::RegionChanged { .. }
        )
    }

    /// True when the condition needs the panel model attached.
    fn needs_lcd(&self) -> bool {
        !matches!(self, Condition::UartContains { .. })
    }
}

/// One scenario step.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Op {
    /// Let virtual time pass.
    Wait { ms: u64 },
    /// Let emulated cycles pass. Useful when the thing being waited on
    /// is a cycle budget rather than firmware time.
    WaitCycles { cycles: u64 },
    /// Advance until a condition holds, or fail at the timeout.
    WaitUntil {
        condition: Condition,
        timeout_ms: u64,
    },
    /// Queue key presses. Each character becomes a press followed by a
    /// release, exactly as `--keys` does.
    Key {
        text: String,
        repeat: u32,
        gap_ms: u64,
    },
    /// Queue explicit raw keyboard events with fixed `state`/`code`.
    KeyEvents { events: Vec<KeyEvent>, gap_ms: u64 },
    /// Record the framebuffer hash, and optionally write a PNG.
    Snapshot { png: Option<PathBuf> },
    /// Check a condition now. A failure is recorded and the scenario
    /// carries on, so one run reports every broken expectation rather
    /// than only the first.
    Assert { condition: Condition },
}

impl Op {
    fn name(&self) -> &'static str {
        match self {
            Op::Wait { .. } => "wait",
            Op::WaitCycles { .. } => "wait_cycles",
            Op::WaitUntil { .. } => "wait_until",
            Op::Key { .. } => "key",
            Op::KeyEvents { .. } => "key_events",
            Op::Snapshot { .. } => "snapshot",
            Op::Assert { .. } => "assert",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Step {
    pub label: Option<String>,
    pub op: Op,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Scenario {
    pub name: String,
    pub description: Option<String>,
    pub poll_ms: u64,
    pub steps: Vec<Step>,
}

impl Scenario {
    /// True if any step sends keys, so the caller can attach the
    /// controller rather than letting the run fail at the first `key`.
    pub fn needs_keyboard(&self) -> bool {
        self.steps
            .iter()
            .any(|s| matches!(s.op, Op::Key { .. } | Op::KeyEvents { .. }))
    }

    /// True if any step looks at the panel. Unlike the keyboard, the
    /// caller cannot quietly satisfy this: attaching the board changes
    /// the step quantum and the GPIO observation strategy, so it has to
    /// be the operator's explicit choice.
    pub fn needs_lcd(&self) -> bool {
        self.steps.iter().any(|s| match &s.op {
            Op::Snapshot { .. } => true,
            Op::WaitUntil { condition, .. } | Op::Assert { condition } => condition.needs_lcd(),
            _ => false,
        })
    }
}

// ---------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------

/// Read and parse a scenario file.
pub fn load(path: &Path) -> Result<Scenario, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("reading scenario {}: {e}", path.display()))?;
    let value: Value = serde_json::from_str(&text)
        .map_err(|e| format!("parsing scenario {}: {e}", path.display()))?;
    parse(&value).map_err(|e| format!("{}: {e}", path.display()))
}

/// Every parse error names the exact JSON path that is wrong. Scenario
/// files are written by hand; "expected an integer" without a location
/// is the difference between a two-second fix and a puzzled half hour.
fn parse(root: &Value) -> Result<Scenario, String> {
    let obj = root
        .as_object()
        .ok_or_else(|| "top level must be an object".to_string())?;

    let schema = obj
        .get("schema")
        .and_then(Value::as_u64)
        .ok_or_else(|| "schema: required, must be an integer".to_string())?;
    if schema != SCENARIO_SCHEMA {
        return Err(format!(
            "schema: this build understands version {SCENARIO_SCHEMA}, file says {schema}"
        ));
    }

    let name = obj
        .get("name")
        .and_then(Value::as_str)
        .ok_or_else(|| "name: required, must be a string".to_string())?
        .to_string();
    let description = obj
        .get("description")
        .and_then(Value::as_str)
        .map(str::to_string);
    let poll_ms = match obj.get("poll_ms") {
        None => DEFAULT_POLL_MS,
        Some(v) => {
            let n = v
                .as_u64()
                .ok_or_else(|| "poll_ms: must be an integer".to_string())?;
            if n == 0 {
                return Err("poll_ms: must be at least 1".to_string());
            }
            n
        }
    };

    let raw_steps = obj
        .get("steps")
        .and_then(Value::as_array)
        .ok_or_else(|| "steps: required, must be an array".to_string())?;
    if raw_steps.is_empty() {
        return Err("steps: must contain at least one step".to_string());
    }

    let mut steps = Vec::with_capacity(raw_steps.len());
    for (i, raw) in raw_steps.iter().enumerate() {
        steps.push(parse_step(raw).map_err(|e| format!("steps[{i}].{e}"))?);
    }

    Ok(Scenario {
        name,
        description,
        poll_ms,
        steps,
    })
}

fn parse_step(raw: &Value) -> Result<Step, String> {
    let obj = raw
        .as_object()
        .ok_or_else(|| ": must be an object".to_string())?;
    let label = obj.get("label").and_then(Value::as_str).map(str::to_string);
    let op_name = obj
        .get("op")
        .and_then(Value::as_str)
        .ok_or_else(|| "op: required, must be a string".to_string())?;

    let op = match op_name {
        "wait" => Op::Wait {
            ms: duration_field(obj, "ms")?,
        },
        "wait_cycles" => {
            let cycles = obj
                .get("cycles")
                .and_then(Value::as_u64)
                .ok_or_else(|| "cycles: required, must be an integer".to_string())?;
            if cycles == 0 {
                return Err("cycles: must be at least 1".to_string());
            }
            Op::WaitCycles { cycles }
        }
        "wait_until" => {
            let condition = parse_condition(
                obj.get("condition")
                    .ok_or_else(|| "condition: required".to_string())?,
            )
            .map_err(|e| format!("condition.{e}"))?;
            Op::WaitUntil {
                condition,
                timeout_ms: duration_field(obj, "timeout_ms")?,
            }
        }
        "key" => {
            let text = obj
                .get("text")
                .and_then(Value::as_str)
                .ok_or_else(|| "text: required, must be a string".to_string())?;
            if text.is_empty() {
                return Err("text: must not be empty".to_string());
            }
            // Only 8-bit codes cross the I2C wire; the controller has no
            // encoding for anything wider, and silently substituting
            // would make a scenario lie about what it pressed.
            for ch in text.chars() {
                if u8::try_from(ch as u32).is_err() {
                    return Err(format!(
                        "text: {ch:?} is not an 8-bit key code — the keyboard \
                         controller cannot carry it"
                    ));
                }
            }
            let repeat = match obj.get("repeat") {
                None => 1,
                Some(v) => {
                    let n = v
                        .as_u64()
                        .ok_or_else(|| "repeat: must be an integer".to_string())?;
                    if n == 0 {
                        return Err("repeat: must be at least 1".to_string());
                    }
                    u32::try_from(n).map_err(|_| "repeat: too large".to_string())?
                }
            };
            let gap_ms = match obj.get("gap_ms") {
                None => 0,
                Some(v) => {
                    let n = v
                        .as_u64()
                        .ok_or_else(|| "gap_ms: must be an integer".to_string())?;
                    if n > MAX_WAIT_MS {
                        return Err(format!("gap_ms: {n} exceeds the {MAX_WAIT_MS} ms limit"));
                    }
                    n
                }
            };
            Op::Key {
                text: text.to_string(),
                repeat,
                gap_ms,
            }
        }
        "key_events" => {
            let raw_events = obj
                .get("events")
                .and_then(Value::as_array)
                .ok_or_else(|| "events: required, must be an array".to_string())?;
            if raw_events.is_empty() {
                return Err("events: must not be empty".to_string());
            }
            let mut events = Vec::with_capacity(raw_events.len());
            for (i, raw_event) in raw_events.iter().enumerate() {
                let raw_event = raw_event
                    .as_object()
                    .ok_or_else(|| format!("events[{i}]: must be an object"))?;
                let state = raw_event
                    .get("state")
                    .and_then(Value::as_str)
                    .ok_or_else(|| format!("events[{i}].state: required, must be a string"))?;
                let state = match state {
                    "pressed" => KeyState::Pressed,
                    "held" => KeyState::Held,
                    "released" => KeyState::Released,
                    _ => {
                        return Err(format!(
                            "events[{i}].state: unknown state '{state}' (expected \
                             pressed, held or released)"
                        ));
                    }
                };
                let code = raw_event
                    .get("code")
                    .and_then(Value::as_u64)
                    .ok_or_else(|| format!("events[{i}].code: required, must be an integer"))?;
                let code = u8::try_from(code)
                    .map_err(|_| format!("events[{i}].code: {code} does not fit in u8"))?;
                events.push(KeyEvent { state, code });
            }
            let gap_ms = match obj.get("gap_ms") {
                None => 0,
                Some(v) => {
                    let n = v
                        .as_u64()
                        .ok_or_else(|| "gap_ms: must be an integer".to_string())?;
                    if n > MAX_WAIT_MS {
                        return Err(format!("gap_ms: {n} exceeds the {MAX_WAIT_MS} ms limit"));
                    }
                    n
                }
            };
            Op::KeyEvents { events, gap_ms }
        }
        "snapshot" => Op::Snapshot {
            png: obj
                .get("png")
                .map(|v| {
                    v.as_str()
                        .map(PathBuf::from)
                        .ok_or_else(|| "png: must be a string".to_string())
                })
                .transpose()?,
        },
        "assert" => {
            let condition = parse_condition(
                obj.get("condition")
                    .ok_or_else(|| "condition: required".to_string())?,
            )
            .map_err(|e| format!("condition.{e}"))?;
            if condition.is_stateful() {
                return Err(format!(
                    "condition.kind: '{}' compares against an earlier moment, so it \
                     only means something inside wait_until — assert checks one instant",
                    condition.kind()
                ));
            }
            Op::Assert { condition }
        }
        other => {
            return Err(format!(
                "op: unknown operation '{other}' (expected wait, wait_cycles, \
                 wait_until, key, key_events, snapshot or assert)"
            ));
        }
    };

    Ok(Step { label, op })
}

fn duration_field(obj: &serde_json::Map<String, Value>, field: &str) -> Result<u64, String> {
    let ms = obj
        .get(field)
        .and_then(Value::as_u64)
        .ok_or_else(|| format!("{field}: required, must be an integer"))?;
    if ms > MAX_WAIT_MS {
        return Err(format!("{field}: {ms} exceeds the {MAX_WAIT_MS} ms limit"));
    }
    Ok(ms)
}

fn parse_condition(raw: &Value) -> Result<Condition, String> {
    let obj = raw
        .as_object()
        .ok_or_else(|| ": must be an object".to_string())?;
    let kind = obj
        .get("kind")
        .and_then(Value::as_str)
        .ok_or_else(|| "kind: required, must be a string".to_string())?;

    match kind {
        "pixel" => {
            let x = coord(obj, "x", pins::VIEWPORT_WIDTH)?;
            let y = coord(obj, "y", pins::VIEWPORT_HEIGHT)?;
            let equals = colour(obj, "equals")?;
            let not_equals = colour(obj, "not_equals")?;
            if equals.is_none() && not_equals.is_none() {
                return Err("equals or not_equals: one is required".to_string());
            }
            if equals.is_some() && not_equals.is_some() {
                return Err(
                    "equals and not_equals: give one, not both — together they are \
                     either a contradiction or a tautology"
                        .to_string(),
                );
            }
            Ok(Condition::Pixel {
                x,
                y,
                equals,
                not_equals,
            })
        }
        "region_non_black" => {
            let rect = parse_rect(obj)?;
            let min = optional_usize(obj, "min")?;
            let max = optional_usize(obj, "max")?;
            if min.is_none() && max.is_none() {
                return Err("min or max: at least one is required".to_string());
            }
            if let (Some(lo), Some(hi)) = (min, max)
                && lo > hi
            {
                return Err(format!("min ({lo}) is above max ({hi})"));
            }
            let area = rect.w * rect.h;
            for (field, bound) in [("min", min), ("max", max)] {
                if let Some(n) = bound
                    && n > area
                {
                    return Err(format!(
                        "{field}: {n} exceeds the {area} pixels in {}",
                        rect.describe()
                    ));
                }
            }
            Ok(Condition::RegionNonBlack { rect, min, max })
        }
        "region_hash" => {
            let rect = parse_rect(obj)?;
            let equals = obj
                .get("equals")
                .and_then(Value::as_str)
                .ok_or_else(|| "equals: required, must be a 64-character hex digest".to_string())?;
            if equals.len() != 64 || !equals.bytes().all(|b| b.is_ascii_hexdigit()) {
                return Err(format!(
                    "equals: '{equals}' is not a 64-character hex SHA-256 digest"
                ));
            }
            Ok(Condition::RegionHash {
                rect,
                equals: equals.to_ascii_lowercase(),
            })
        }
        "region_stable" => Ok(Condition::RegionStable {
            rect: parse_rect(obj)?,
            for_ms: duration_field(obj, "for_ms")?,
        }),
        "region_changed" => Ok(Condition::RegionChanged {
            rect: parse_rect(obj)?,
        }),
        "uart_contains" => {
            let text = obj
                .get("text")
                .and_then(Value::as_str)
                .ok_or_else(|| "text: required, must be a string".to_string())?;
            if text.is_empty() {
                return Err("text: must not be empty".to_string());
            }
            Ok(Condition::UartContains {
                text: text.to_string(),
            })
        }
        other => Err(format!(
            "kind: unknown condition '{other}' (expected pixel, region_non_black, \
             region_hash, region_stable, region_changed or uart_contains)"
        )),
    }
}

fn parse_rect(obj: &serde_json::Map<String, Value>) -> Result<Rect, String> {
    let x = coord(obj, "x", pins::VIEWPORT_WIDTH)?;
    let y = coord(obj, "y", pins::VIEWPORT_HEIGHT)?;
    let w = extent(obj, "w")?;
    let h = extent(obj, "h")?;
    if x + w > pins::VIEWPORT_WIDTH {
        return Err(format!(
            "x + w: {} runs past the {}-pixel viewport width",
            x + w,
            pins::VIEWPORT_WIDTH
        ));
    }
    if y + h > pins::VIEWPORT_HEIGHT {
        return Err(format!(
            "y + h: {} runs past the {}-pixel viewport height",
            y + h,
            pins::VIEWPORT_HEIGHT
        ));
    }
    Ok(Rect { x, y, w, h })
}

fn coord(obj: &serde_json::Map<String, Value>, field: &str, limit: usize) -> Result<usize, String> {
    let n = obj
        .get(field)
        .and_then(Value::as_u64)
        .ok_or_else(|| format!("{field}: required, must be a non-negative integer"))?;
    let n = usize::try_from(n).map_err(|_| format!("{field}: too large"))?;
    if n >= limit {
        return Err(format!("{field}: {n} is outside the 0..{limit} viewport"));
    }
    Ok(n)
}

fn extent(obj: &serde_json::Map<String, Value>, field: &str) -> Result<usize, String> {
    let n = obj
        .get(field)
        .and_then(Value::as_u64)
        .ok_or_else(|| format!("{field}: required, must be a positive integer"))?;
    let n = usize::try_from(n).map_err(|_| format!("{field}: too large"))?;
    if n == 0 {
        return Err(format!("{field}: must be at least 1"));
    }
    Ok(n)
}

fn optional_usize(
    obj: &serde_json::Map<String, Value>,
    field: &str,
) -> Result<Option<usize>, String> {
    match obj.get(field) {
        None => Ok(None),
        Some(v) => {
            let n = v
                .as_u64()
                .ok_or_else(|| format!("{field}: must be a non-negative integer"))?;
            Ok(Some(
                usize::try_from(n).map_err(|_| format!("{field}: too large"))?,
            ))
        }
    }
}

/// RGB565 colours may be given as a number or as a `"0x…"` string —
/// hand-written scenarios reach for hex, and JSON has no hex literal.
fn colour(obj: &serde_json::Map<String, Value>, field: &str) -> Result<Option<u16>, String> {
    let Some(v) = obj.get(field) else {
        return Ok(None);
    };
    let raw = match v {
        Value::Number(n) => n
            .as_u64()
            .ok_or_else(|| format!("{field}: must be a non-negative integer"))?,
        Value::String(s) => {
            let body = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X"));
            let body =
                body.ok_or_else(|| format!("{field}: '{s}' must start with 0x, or be a number"))?;
            u64::from_str_radix(body, 16)
                .map_err(|_| format!("{field}: '{s}' is not hexadecimal"))?
        }
        _ => return Err(format!("{field}: must be a number or a 0x string")),
    };
    u16::try_from(raw)
        .map(Some)
        .map_err(|_| format!("{field}: {raw} does not fit in a 16-bit RGB565 value"))
}

// ---------------------------------------------------------------------
// Evaluation
// ---------------------------------------------------------------------

/// What the engine can see at a poll.
pub struct Observation<'a> {
    pub now_ns: u64,
    pub cycles: u64,
    pub lcd: Option<&'a Mutex<St7365p>>,
    pub keyboard: Option<&'a Mutex<Keyboard>>,
    /// Every UART byte the firmware has transmitted so far.
    pub uart: &'a [u8],
}

impl Observation<'_> {
    fn now_ms(&self) -> u64 {
        self.now_ns / 1_000_000
    }
}

/// Read a region out of the panel's GRAM as RGB565 little-endian bytes.
///
/// Viewport coordinates map one-to-one onto the top-left of the GRAM
/// (see `Framebuffer::from_gram`), so no translation is needed.
fn region_bytes(lcd: &St7365p, rect: Rect) -> Vec<u8> {
    let mut out = Vec::with_capacity(rect.w * rect.h * 2);
    for y in rect.y..rect.y + rect.h {
        for x in rect.x..rect.x + rect.w {
            let px = lcd.gram_pixel(x, y).unwrap_or(0);
            out.extend_from_slice(&px.to_le_bytes());
        }
    }
    out
}

fn region_non_black(lcd: &St7365p, rect: Rect) -> usize {
    let mut n = 0;
    for y in rect.y..rect.y + rect.h {
        for x in rect.x..rect.x + rect.w {
            if lcd.gram_pixel(x, y).unwrap_or(0) != 0 {
                n += 1;
            }
        }
    }
    n
}

/// The outcome of evaluating one condition: whether it holds, and a
/// human-readable account of what was actually seen.
struct Verdict {
    holds: bool,
    detail: String,
}

/// Per-step memory for conditions that compare against an earlier
/// moment.
#[derive(Default)]
struct History {
    last_hash: Option<String>,
    /// Virtual time at which `last_hash` was first observed.
    stable_since_ns: u64,
    /// Region contents when the step began.
    baseline: Option<String>,
}

fn evaluate(
    condition: &Condition,
    obs: &Observation<'_>,
    history: &mut History,
) -> Result<Verdict, String> {
    if condition.needs_lcd() && obs.lcd.is_none() {
        return Err(format!(
            "condition '{}' needs the panel, but no board model is attached \
             (pass --board picocalc)",
            condition.kind()
        ));
    }

    match condition {
        Condition::Pixel {
            x,
            y,
            equals,
            not_equals,
        } => {
            let lcd = obs.lcd.expect("checked above");
            let px = lcd
                .lock()
                .map_err(|_| "LCD model mutex poisoned".to_string())?
                .gram_pixel(*x, *y)
                .unwrap_or(0);
            let holds = match (equals, not_equals) {
                (Some(want), _) => px == *want,
                (_, Some(avoid)) => px != *avoid,
                _ => unreachable!("parse rejects a pixel condition with neither bound"),
            };
            Ok(Verdict {
                holds,
                detail: format!("pixel ({x},{y}) = {px:#06x}"),
            })
        }
        Condition::RegionNonBlack { rect, min, max } => {
            let lcd = obs.lcd.expect("checked above");
            let count = {
                let guard = lcd
                    .lock()
                    .map_err(|_| "LCD model mutex poisoned".to_string())?;
                region_non_black(&guard, *rect)
            };
            let holds = min.is_none_or(|lo| count >= lo) && max.is_none_or(|hi| count <= hi);
            Ok(Verdict {
                holds,
                detail: format!(
                    "{} non-black pixels in {} (want {}..{})",
                    count,
                    rect.describe(),
                    min.map(|n| n.to_string()).unwrap_or_default(),
                    max.map(|n| n.to_string()).unwrap_or_default()
                ),
            })
        }
        Condition::RegionHash { rect, equals } => {
            let lcd = obs.lcd.expect("checked above");
            let got = {
                let guard = lcd
                    .lock()
                    .map_err(|_| "LCD model mutex poisoned".to_string())?;
                sha256_hex(&region_bytes(&guard, *rect))
            };
            Ok(Verdict {
                holds: got == *equals,
                detail: format!("{} hashes to {got}", rect.describe()),
            })
        }
        Condition::RegionStable { rect, for_ms } => {
            let lcd = obs.lcd.expect("checked above");
            let got = {
                let guard = lcd
                    .lock()
                    .map_err(|_| "LCD model mutex poisoned".to_string())?;
                sha256_hex(&region_bytes(&guard, *rect))
            };
            if history.last_hash.as_deref() != Some(got.as_str()) {
                history.last_hash = Some(got);
                history.stable_since_ns = obs.now_ns;
            }
            let held_ns = obs.now_ns.saturating_sub(history.stable_since_ns);
            let held_ms = held_ns / 1_000_000;
            Ok(Verdict {
                holds: held_ms >= *for_ms,
                detail: format!(
                    "{} unchanged for {held_ms} ms (want {for_ms})",
                    rect.describe()
                ),
            })
        }
        Condition::RegionChanged { rect } => {
            let lcd = obs.lcd.expect("checked above");
            let got = {
                let guard = lcd
                    .lock()
                    .map_err(|_| "LCD model mutex poisoned".to_string())?;
                sha256_hex(&region_bytes(&guard, *rect))
            };
            // The first poll defines "before"; it can never satisfy the
            // condition, which is the point.
            let baseline = history.baseline.get_or_insert_with(|| got.clone());
            Ok(Verdict {
                holds: got != *baseline,
                detail: format!(
                    "{} is {} the baseline",
                    rect.describe(),
                    if got == *baseline {
                        "still at"
                    } else {
                        "away from"
                    }
                ),
            })
        }
        Condition::UartContains { text } => {
            let holds = obs
                .uart
                .windows(text.len().max(1))
                .any(|w| w == text.as_bytes());
            Ok(Verdict {
                holds,
                detail: format!("{} UART bytes seen", obs.uart.len()),
            })
        }
    }
}

// ---------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------

/// What a step ended up doing.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StepStatus {
    Pass,
    Fail,
    /// The run stopped before this step finished.
    Incomplete,
}

impl StepStatus {
    fn as_str(self) -> &'static str {
        match self {
            StepStatus::Pass => "pass",
            StepStatus::Fail => "fail",
            StepStatus::Incomplete => "incomplete",
        }
    }
}

pub struct StepResult {
    pub index: usize,
    pub op: &'static str,
    pub label: Option<String>,
    pub status: StepStatus,
    /// Virtual time at which the step finished.
    pub at_ms: u64,
    pub at_cycles: u64,
    pub detail: Option<String>,
    /// Framebuffer digest, for `snapshot` steps.
    pub snapshot_sha256: Option<String>,
    /// PNG basename, for `snapshot` steps that wrote one.
    pub png_basename: Option<String>,
}

/// Where the engine is inside the step it is currently working on.
enum Pending {
    /// Nothing started yet.
    Fresh,
    Wait {
        until_ns: u64,
    },
    WaitCycles {
        until_cycles: u64,
    },
    WaitUntil {
        deadline_ns: u64,
        history: History,
    },
    Key {
        /// Characters still to send, most recent last.
        queue: Vec<u8>,
        next_at_ns: u64,
    },
    KeyEvents {
        queue: Vec<KeyEvent>,
        next_at_ns: u64,
        gap_ms: u64,
    },
}

/// Drives a [`Scenario`] against a running emulator.
pub struct Engine {
    scenario: Scenario,
    /// Where snapshot PNGs are written.
    snapshot_dir: PathBuf,
    index: usize,
    pending: Pending,
    results: Vec<StepResult>,
    failed: bool,
    done: bool,
    /// Set when the run stopped with steps still outstanding. Kept
    /// apart from `failed`: "the firmware never got there" and "the
    /// expectation was wrong" call for different next moves.
    truncated: bool,
    /// Virtual time at which the engine next wants to be polled.
    next_poll_ns: u64,
    /// Set when something made the scenario unrunnable (a missing board
    /// model, an unwritable PNG path). Distinct from a failed assertion.
    fault: Option<String>,
}

impl Engine {
    pub fn new(scenario: Scenario, snapshot_dir: PathBuf) -> Self {
        Self {
            scenario,
            snapshot_dir,
            index: 0,
            pending: Pending::Fresh,
            results: Vec::new(),
            failed: false,
            done: false,
            truncated: false,
            next_poll_ns: 0,
            fault: None,
        }
    }

    pub fn name(&self) -> &str {
        &self.scenario.name
    }

    pub fn description(&self) -> Option<&str> {
        self.scenario.description.as_deref()
    }

    pub fn poll_ms(&self) -> u64 {
        self.scenario.poll_ms
    }

    pub fn steps_total(&self) -> usize {
        self.scenario.steps.len()
    }

    pub fn results(&self) -> &[StepResult] {
        &self.results
    }

    pub fn is_done(&self) -> bool {
        self.done
    }

    pub fn fault(&self) -> Option<&str> {
        self.fault.as_deref()
    }

    /// Virtual time at which [`Self::poll`] should next be called.
    pub fn next_poll_ns(&self) -> u64 {
        self.next_poll_ns
    }

    /// Whether every step ran and passed.
    pub fn passed(&self) -> bool {
        self.done && !self.failed && !self.truncated && self.fault.is_none()
    }

    /// `error` — the scenario could not be run (a model was missing, a
    /// file could not be written). `incomplete` — the run ended with
    /// steps outstanding. `fail` — a step ran and its expectation did
    /// not hold. All three exit non-zero; they are separated because
    /// only `fail` says anything about the firmware.
    pub fn status(&self) -> &'static str {
        if self.fault.is_some() {
            "error"
        } else if self.truncated {
            "incomplete"
        } else if self.failed {
            "fail"
        } else if self.done {
            "pass"
        } else {
            "incomplete"
        }
    }

    /// Mark whatever was still in flight when the run stopped.
    pub fn finish_incomplete(&mut self, obs: &Observation<'_>) {
        if self.done {
            return;
        }
        // Only the step that had actually started gets an explanation.
        // The ones behind it were never reached, and saying "stopped
        // before this finished" of a step that never began would put the
        // reader's attention in the wrong place.
        let in_flight = (!matches!(self.pending, Pending::Fresh)).then_some(self.index);
        while self.index < self.scenario.steps.len() {
            let step = &self.scenario.steps[self.index];
            let detail = (in_flight == Some(self.index))
                .then(|| "the run stopped before this step finished".to_string());
            self.results.push(StepResult {
                index: self.index,
                op: step.op.name(),
                label: step.label.clone(),
                status: StepStatus::Incomplete,
                at_ms: obs.now_ms(),
                at_cycles: obs.cycles,
                detail,
                snapshot_sha256: None,
                png_basename: None,
            });
            self.index += 1;
        }
        self.truncated = true;
        self.done = true;
    }

    /// Advance the scenario as far as the current observation allows.
    ///
    /// Instantaneous steps (`key`, `snapshot`, `assert`) chain within a
    /// single call; the loop only returns once a step needs time to
    /// pass, or the scenario ends.
    pub fn poll(&mut self, obs: &Observation<'_>) {
        while !self.done {
            let Some(step) = self.scenario.steps.get(self.index) else {
                self.done = true;
                return;
            };
            // Cloning the op keeps the borrow of `self.scenario` from
            // outliving the mutations below. Steps are small.
            let op = step.op.clone();
            let label = step.label.clone();

            match self.advance(&op, obs) {
                Progress::Blocked => return,
                Progress::Finished { status, result } => {
                    self.record(status, &op, label, obs, result);
                    if status == StepStatus::Fail {
                        self.failed = true;
                    }
                    self.index += 1;
                    self.pending = Pending::Fresh;
                }
                Progress::Faulted(message) => {
                    self.record(
                        StepStatus::Fail,
                        &op,
                        label,
                        obs,
                        StepOutput::detail(message.clone()),
                    );
                    self.failed = true;
                    self.fault = Some(message);
                    self.done = true;
                    return;
                }
            }
        }
    }

    fn record(
        &mut self,
        status: StepStatus,
        op: &Op,
        label: Option<String>,
        obs: &Observation<'_>,
        out: StepOutput,
    ) {
        self.results.push(StepResult {
            index: self.index,
            op: op.name(),
            label,
            status,
            at_ms: obs.now_ms(),
            at_cycles: obs.cycles,
            detail: out.detail,
            snapshot_sha256: out.snapshot_sha256,
            png_basename: out.png_basename,
        });
    }

    fn advance(&mut self, op: &Op, obs: &Observation<'_>) -> Progress {
        match op {
            Op::Wait { ms } => {
                if let Pending::Wait { until_ns } = self.pending {
                    if obs.now_ns >= until_ns {
                        return Progress::pass(format!("waited {ms} ms"));
                    }
                    self.next_poll_ns = until_ns;
                    return Progress::Blocked;
                }
                let until_ns = obs.now_ns + ms * 1_000_000;
                self.pending = Pending::Wait { until_ns };
                self.next_poll_ns = until_ns;
                // A zero-length wait is legal and completes at once.
                if obs.now_ns >= until_ns {
                    return Progress::pass(format!("waited {ms} ms"));
                }
                Progress::Blocked
            }
            Op::WaitCycles { cycles } => {
                if let Pending::WaitCycles { until_cycles } = self.pending {
                    if obs.cycles >= until_cycles {
                        return Progress::pass(format!("waited {cycles} cycles"));
                    }
                    // Cycle deadlines cannot be converted to a virtual
                    // time without assuming a clock, so fall back to the
                    // cadence and re-check.
                    self.next_poll_ns = obs.now_ns + self.scenario.poll_ms * 1_000_000;
                    return Progress::Blocked;
                }
                self.pending = Pending::WaitCycles {
                    until_cycles: obs.cycles + cycles,
                };
                self.next_poll_ns = obs.now_ns + self.scenario.poll_ms * 1_000_000;
                Progress::Blocked
            }
            Op::WaitUntil {
                condition,
                timeout_ms,
            } => {
                if !matches!(self.pending, Pending::WaitUntil { .. }) {
                    self.pending = Pending::WaitUntil {
                        deadline_ns: obs.now_ns + timeout_ms * 1_000_000,
                        history: History::default(),
                    };
                }
                let Pending::WaitUntil {
                    deadline_ns,
                    history,
                } = &mut self.pending
                else {
                    unreachable!("just installed")
                };
                let deadline_ns = *deadline_ns;
                let verdict = match evaluate(condition, obs, history) {
                    Ok(v) => v,
                    Err(e) => return Progress::Faulted(e),
                };
                if verdict.holds {
                    return Progress::pass(verdict.detail);
                }
                if obs.now_ns >= deadline_ns {
                    return Progress::fail(format!(
                        "timed out after {timeout_ms} ms — {}",
                        verdict.detail
                    ));
                }
                // Never overshoot the deadline: a timeout must be
                // reported at the time it happened, not at the next tick.
                self.next_poll_ns =
                    (obs.now_ns + self.scenario.poll_ms * 1_000_000).min(deadline_ns);
                Progress::Blocked
            }
            Op::Key {
                text,
                repeat,
                gap_ms,
            } => {
                let Some(kbd) = obs.keyboard else {
                    return Progress::Faulted(
                        "key steps need the keyboard model (pass --keyboard)".to_string(),
                    );
                };
                if !matches!(self.pending, Pending::Key { .. }) {
                    let mut queue = Vec::with_capacity(text.len() * *repeat as usize);
                    for _ in 0..*repeat {
                        for ch in text.chars() {
                            queue.push(ch as u8);
                        }
                    }
                    // Sent in order, so pop from the back of a reversed list.
                    queue.reverse();
                    self.pending = Pending::Key {
                        queue,
                        next_at_ns: obs.now_ns,
                    };
                }
                let Pending::Key { queue, next_at_ns } = &mut self.pending else {
                    unreachable!("just installed")
                };
                while let Some(&code) = queue.last() {
                    if obs.now_ns < *next_at_ns {
                        self.next_poll_ns = *next_at_ns;
                        return Progress::Blocked;
                    }
                    queue.pop();
                    match kbd.lock() {
                        Ok(mut guard) => guard.press_and_release(code),
                        Err(_) => {
                            return Progress::Faulted("keyboard model mutex poisoned".to_string());
                        }
                    }
                    *next_at_ns = obs.now_ns + gap_ms * 1_000_000;
                    // With no gap the whole burst goes in at once,
                    // matching what --keys does.
                    if *gap_ms > 0 && !queue.is_empty() {
                        self.next_poll_ns = *next_at_ns;
                        return Progress::Blocked;
                    }
                }
                Progress::pass(format!(
                    "sent {} key event(s)",
                    text.chars().count() * *repeat as usize
                ))
            }
            Op::KeyEvents { events, gap_ms } => {
                let Some(kbd) = obs.keyboard else {
                    return Progress::Faulted(
                        "key steps need the keyboard model (pass --keyboard)".to_string(),
                    );
                };
                if !matches!(self.pending, Pending::KeyEvents { .. }) {
                    self.pending = Pending::KeyEvents {
                        queue: events.iter().copied().rev().collect(),
                        next_at_ns: obs.now_ns,
                        gap_ms: *gap_ms,
                    };
                }
                let Pending::KeyEvents {
                    queue,
                    next_at_ns,
                    gap_ms,
                } = &mut self.pending
                else {
                    unreachable!("just installed")
                };
                while let Some(event) = queue.pop() {
                    if obs.now_ns < *next_at_ns {
                        self.next_poll_ns = *next_at_ns;
                        return Progress::Blocked;
                    }
                    match kbd.lock() {
                        Ok(mut guard) => guard.push_event(event),
                        Err(_) => {
                            return Progress::Faulted("keyboard model mutex poisoned".to_string());
                        }
                    }
                    *next_at_ns = obs.now_ns + *gap_ms * 1_000_000;
                    if *gap_ms > 0 && !queue.is_empty() {
                        self.next_poll_ns = *next_at_ns;
                        return Progress::Blocked;
                    }
                }
                Progress::pass(format!("sent {} raw key event(s)", events.len()))
            }
            Op::Snapshot { png } => {
                let Some(lcd) = obs.lcd else {
                    return Progress::Faulted(
                        "snapshot steps need the panel (pass --board picocalc)".to_string(),
                    );
                };
                let fb = match lcd.lock() {
                    Ok(guard) => guard.framebuffer(),
                    Err(_) => {
                        return Progress::Faulted("LCD model mutex poisoned".to_string());
                    }
                };
                let sha = fb.rgb565_sha256();
                let mut png_basename = None;
                if let Some(name) = png {
                    let path = self.snapshot_dir.join(name);
                    if let Some(parent) = path.parent()
                        && !parent.as_os_str().is_empty()
                        && let Err(e) = std::fs::create_dir_all(parent)
                    {
                        return Progress::Faulted(format!(
                            "creating snapshot directory {}: {e}",
                            parent.display()
                        ));
                    }
                    if let Err(e) = fb.write_png(&path) {
                        return Progress::Faulted(format!(
                            "writing snapshot {}: {e}",
                            path.display()
                        ));
                    }
                    png_basename = path.file_name().map(|n| n.to_string_lossy().into_owned());
                }
                Progress::Finished {
                    status: StepStatus::Pass,
                    result: StepOutput {
                        detail: Some(format!("{} non-black pixels", fb.non_black_pixels())),
                        snapshot_sha256: Some(sha),
                        png_basename,
                    },
                }
            }
            Op::Assert { condition } => {
                let mut history = History::default();
                match evaluate(condition, obs, &mut history) {
                    Ok(v) if v.holds => Progress::pass(v.detail),
                    Ok(v) => Progress::fail(v.detail),
                    Err(e) => Progress::Faulted(e),
                }
            }
        }
    }
}

struct StepOutput {
    detail: Option<String>,
    snapshot_sha256: Option<String>,
    png_basename: Option<String>,
}

impl StepOutput {
    fn detail(text: String) -> Self {
        Self {
            detail: Some(text),
            snapshot_sha256: None,
            png_basename: None,
        }
    }
}

enum Progress {
    Blocked,
    Finished {
        status: StepStatus,
        result: StepOutput,
    },
    /// The scenario cannot continue — a missing model, an I/O failure.
    Faulted(String),
}

impl Progress {
    fn pass(detail: String) -> Self {
        Progress::Finished {
            status: StepStatus::Pass,
            result: StepOutput::detail(detail),
        }
    }

    fn fail(detail: String) -> Self {
        Progress::Finished {
            status: StepStatus::Fail,
            result: StepOutput::detail(detail),
        }
    }
}

// ---------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------

impl Engine {
    /// The `"scenario"` section of the run report, indented to sit
    /// alongside the others. Field order is fixed so two runs of the
    /// same scenario produce byte-identical JSON.
    pub fn to_json(&self, scenario_basename: &str, json_string: impl Fn(&str) -> String) -> String {
        let mut s = String::new();
        s.push_str("  \"scenario\": {\n");
        s.push_str(&format!(
            "    \"file\": {},\n",
            json_string(scenario_basename)
        ));
        s.push_str(&format!("    \"name\": {},\n", json_string(self.name())));
        s.push_str(&format!(
            "    \"description\": {},\n",
            match self.description() {
                Some(d) => json_string(d),
                None => "null".to_string(),
            }
        ));
        s.push_str(&format!(
            "    \"status\": {},\n",
            json_string(self.status())
        ));
        s.push_str(&format!("    \"poll_ms\": {},\n", self.poll_ms()));
        s.push_str(&format!("    \"steps_total\": {},\n", self.steps_total()));
        s.push_str(&format!(
            "    \"error\": {},\n",
            match self.fault() {
                Some(f) => json_string(f),
                None => "null".to_string(),
            }
        ));
        s.push_str("    \"steps\": [");
        if self.results().is_empty() {
            s.push_str("]\n");
        } else {
            s.push('\n');
            for (i, r) in self.results().iter().enumerate() {
                s.push_str("      {");
                s.push_str(&format!("\"index\": {}, ", r.index));
                s.push_str(&format!("\"op\": {}, ", json_string(r.op)));
                s.push_str(&format!(
                    "\"label\": {}, ",
                    match &r.label {
                        Some(l) => json_string(l),
                        None => "null".to_string(),
                    }
                ));
                s.push_str(&format!("\"status\": {}, ", json_string(r.status.as_str())));
                s.push_str(&format!("\"at_ms\": {}, ", r.at_ms));
                s.push_str(&format!("\"at_cycles\": {}, ", r.at_cycles));
                s.push_str(&format!(
                    "\"detail\": {}",
                    match &r.detail {
                        Some(d) => json_string(d),
                        None => "null".to_string(),
                    }
                ));
                if let Some(sha) = &r.snapshot_sha256 {
                    s.push_str(&format!(", \"rgb565_sha256\": {}", json_string(sha)));
                }
                if let Some(png) = &r.png_basename {
                    s.push_str(&format!(", \"png_basename\": {}", json_string(png)));
                }
                s.push('}');
                if i + 1 < self.results().len() {
                    s.push(',');
                }
                s.push('\n');
            }
            s.push_str("    ]\n");
        }
        s.push_str("  },\n");
        s
    }

    /// One line per step, for the human watching the run.
    pub fn summary_lines(&self) -> Vec<String> {
        self.results()
            .iter()
            .map(|r| {
                let label = r.label.clone().unwrap_or_else(|| r.op.to_string());
                format!(
                    "  [{}] {:>8} ms  {}{}",
                    r.status.as_str(),
                    r.at_ms,
                    label,
                    match &r.detail {
                        Some(d) => format!(" — {d}"),
                        None => String::new(),
                    }
                )
            })
            .collect()
    }
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use picocalc_board::keyboard::REG_KEY_FIFO;
    use picocalc_board::{KeyEvent, KeyState, Keyboard};
    use rp2040_emu::peripherals::i2c::I2cExternalDevice;
    use serde_json::json;
    use std::sync::Mutex;

    fn scenario_json(steps: &str) -> String {
        format!(r#"{{"schema": 1, "name": "t", "steps": {steps}}}"#)
    }

    fn parse_str(text: &str) -> Result<Scenario, String> {
        parse(&serde_json::from_str::<Value>(text).expect("valid JSON"))
    }

    #[test]
    fn a_minimal_scenario_parses() {
        let s = parse_str(&scenario_json(r#"[{"op": "wait", "ms": 10}]"#)).unwrap();
        assert_eq!(s.name, "t");
        assert_eq!(s.poll_ms, DEFAULT_POLL_MS);
        assert_eq!(s.steps.len(), 1);
        assert_eq!(s.steps[0].op, Op::Wait { ms: 10 });
    }

    #[test]
    fn a_wrong_schema_version_is_named_in_the_error() {
        let e = parse_str(r#"{"schema": 99, "name": "t", "steps": []}"#).unwrap_err();
        assert!(e.contains("version 1"), "{e}");
        assert!(e.contains("99"), "{e}");
    }

    #[test]
    fn errors_point_at_the_step_and_field() {
        let e = parse_str(&scenario_json(
            r#"[{"op": "wait", "ms": 1}, {"op": "wait"}]"#,
        ))
        .unwrap_err();
        assert!(e.starts_with("steps[1].ms:"), "{e}");
    }

    #[test]
    fn errors_point_inside_a_condition() {
        let e = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition": {"kind": "pixel", "x": 1}}]"#,
        ))
        .unwrap_err();
        assert!(e.starts_with("steps[0].condition.y:"), "{e}");
    }

    #[test]
    fn an_unknown_operation_lists_the_known_ones() {
        let e = parse_str(&scenario_json(r#"[{"op": "sing"}]"#)).unwrap_err();
        assert!(e.contains("wait_until"), "{e}");
        assert!(e.contains("snapshot"), "{e}");
    }

    #[test]
    fn rectangles_must_fit_the_viewport() {
        let e = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition":
                 {"kind": "region_hash", "x": 300, "y": 0, "w": 40, "h": 8,
                  "equals": "00000000000000000000000000000000000000000000000000000000000000ab"}}]"#,
        ))
        .unwrap_err();
        assert!(e.contains("viewport width"), "{e}");
    }

    #[test]
    fn colours_take_hex_strings_or_numbers() {
        let s = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition":
                 {"kind": "pixel", "x": 0, "y": 0, "equals": "0xF81F"}},
                {"op": "assert", "condition":
                 {"kind": "pixel", "x": 0, "y": 0, "equals": 63519}}]"#,
        ))
        .unwrap();
        let want = Op::Assert {
            condition: Condition::Pixel {
                x: 0,
                y: 0,
                equals: Some(0xF81F),
                not_equals: None,
            },
        };
        assert_eq!(s.steps[0].op, want);
        assert_eq!(s.steps[1].op, want);
    }

    #[test]
    fn a_pixel_condition_needs_exactly_one_bound() {
        let neither = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition": {"kind": "pixel", "x": 0, "y": 0}}]"#,
        ))
        .unwrap_err();
        assert!(neither.contains("one is required"), "{neither}");

        let both = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition":
                 {"kind": "pixel", "x": 0, "y": 0, "equals": 1, "not_equals": 2}}]"#,
        ))
        .unwrap_err();
        assert!(both.contains("not both"), "{both}");
    }

    #[test]
    fn stateful_conditions_are_rejected_in_assert() {
        let e = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition":
                 {"kind": "region_stable", "x": 0, "y": 0, "w": 4, "h": 4, "for_ms": 10}}]"#,
        ))
        .unwrap_err();
        assert!(e.contains("wait_until"), "{e}");
    }

    #[test]
    fn stateful_conditions_are_accepted_in_wait_until() {
        let s = parse_str(&scenario_json(
            r#"[{"op": "wait_until", "timeout_ms": 100, "condition":
                 {"kind": "region_stable", "x": 0, "y": 0, "w": 4, "h": 4, "for_ms": 10}}]"#,
        ))
        .unwrap();
        assert!(matches!(
            s.steps[0].op,
            Op::WaitUntil {
                condition: Condition::RegionStable { for_ms: 10, .. },
                timeout_ms: 100
            }
        ));
    }

    #[test]
    fn a_non_black_bound_above_the_region_area_is_rejected() {
        let e = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition":
                 {"kind": "region_non_black", "x": 0, "y": 0, "w": 2, "h": 2, "min": 5}}]"#,
        ))
        .unwrap_err();
        assert!(e.contains("4 pixels"), "{e}");
    }

    #[test]
    fn wide_characters_are_refused_rather_than_substituted() {
        let e = parse_str(&scenario_json(r#"[{"op": "key", "text": "あ"}]"#)).unwrap_err();
        assert!(e.contains("8-bit"), "{e}");
    }

    #[test]
    fn key_events_must_not_be_empty() {
        let e = parse_str(&scenario_json(
            r#"[{"op": "key_events", "events": [], "gap_ms": 1}]"#,
        ))
        .unwrap_err();
        assert!(e.contains("events: must not be empty"), "{e}");
    }

    #[test]
    fn key_events_validates_state_code_and_gap() {
        let e = parse_str(&scenario_json(
            r#"[{"op":"key_events","events":[{"state":"n/a","code":1}]}]"#,
        ))
        .unwrap_err();
        assert!(e.contains("events[0].state"), "{e}");

        let e = parse_str(&scenario_json(
            r#"[{"op":"key_events","events":[{"state":"pressed","code":999}]}]"#,
        ))
        .unwrap_err();
        assert!(e.contains("events[0].code"), "{e}");

        let e = parse(&json!({
            "schema": 1,
            "name": "t",
            "steps": [
                {
                    "op": "key_events",
                    "events": [{ "state": "pressed", "code": 1 }],
                    "gap_ms": "1",
                }
            ]
        }))
        .unwrap_err();
        assert!(e.contains("gap_ms"), "{e}");
    }

    #[test]
    fn a_hash_must_look_like_a_digest() {
        let e = parse_str(&scenario_json(
            r#"[{"op": "assert", "condition":
                 {"kind": "region_hash", "x": 0, "y": 0, "w": 4, "h": 4, "equals": "cafe"}}]"#,
        ))
        .unwrap_err();
        assert!(e.contains("64-character"), "{e}");
    }

    // --- engine behaviour, with no emulator attached ------------------

    fn engine(steps: Vec<Step>) -> Engine {
        Engine::new(
            Scenario {
                name: "t".to_string(),
                description: None,
                poll_ms: 5,
                steps,
            },
            PathBuf::from("."),
        )
    }

    fn step(op: Op) -> Step {
        Step { label: None, op }
    }

    fn obs(now_ns: u64, uart: &[u8]) -> Observation<'_> {
        Observation {
            now_ns,
            cycles: now_ns / 8,
            lcd: None,
            keyboard: None,
            uart,
        }
    }

    #[test]
    fn a_wait_finishes_at_its_deadline_and_not_before() {
        let mut e = engine(vec![step(Op::Wait { ms: 100 })]);
        e.poll(&obs(0, b""));
        assert!(!e.is_done());
        assert_eq!(e.next_poll_ns(), 100_000_000);

        e.poll(&obs(99_999_999, b""));
        assert!(!e.is_done());

        e.poll(&obs(100_000_000, b""));
        assert!(e.is_done());
        assert!(e.passed());
        assert_eq!(e.results()[0].at_ms, 100);
    }

    #[test]
    fn instantaneous_steps_chain_in_one_poll() {
        let mut e = engine(vec![
            step(Op::Assert {
                condition: Condition::UartContains {
                    text: "ok".to_string(),
                },
            }),
            step(Op::Assert {
                condition: Condition::UartContains {
                    text: "ready".to_string(),
                },
            }),
        ]);
        e.poll(&obs(0, b"system ok and ready"));
        assert!(e.is_done());
        assert!(e.passed());
        assert_eq!(e.results().len(), 2);
    }

    #[test]
    fn a_failed_assertion_does_not_stop_the_rest() {
        let mut e = engine(vec![
            step(Op::Assert {
                condition: Condition::UartContains {
                    text: "absent".to_string(),
                },
            }),
            step(Op::Assert {
                condition: Condition::UartContains {
                    text: "present".to_string(),
                },
            }),
        ]);
        e.poll(&obs(0, b"present"));
        assert!(e.is_done());
        assert!(!e.passed());
        assert_eq!(e.results()[0].status, StepStatus::Fail);
        assert_eq!(e.results()[1].status, StepStatus::Pass);
        assert_eq!(e.status(), "fail");
    }

    #[test]
    fn wait_until_reports_the_timeout_at_the_moment_it_expires() {
        let mut e = engine(vec![step(Op::WaitUntil {
            condition: Condition::UartContains {
                text: "never".to_string(),
            },
            timeout_ms: 20,
        })]);
        e.poll(&obs(0, b""));
        // The cadence is 5 ms, so the engine must not schedule past the
        // 20 ms deadline.
        assert_eq!(e.next_poll_ns(), 5_000_000);
        for t in [5, 10, 15] {
            e.poll(&obs(t * 1_000_000, b""));
            assert!(!e.is_done(), "finished early at {t} ms");
        }
        e.poll(&obs(20_000_000, b""));
        assert!(e.is_done());
        assert_eq!(e.results()[0].status, StepStatus::Fail);
        assert_eq!(e.results()[0].at_ms, 20);
        assert!(
            e.results()[0]
                .detail
                .as_ref()
                .unwrap()
                .contains("timed out")
        );
    }

    #[test]
    fn wait_until_passes_as_soon_as_the_condition_holds() {
        let mut e = engine(vec![step(Op::WaitUntil {
            condition: Condition::UartContains {
                text: "boot".to_string(),
            },
            timeout_ms: 1000,
        })]);
        e.poll(&obs(0, b""));
        assert!(!e.is_done());
        e.poll(&obs(5_000_000, b"boot"));
        assert!(e.is_done());
        assert!(e.passed());
        assert_eq!(e.results()[0].at_ms, 5);
    }

    #[test]
    fn a_key_step_without_a_keyboard_is_an_error_not_a_silent_pass() {
        let mut e = engine(vec![step(Op::Key {
            text: "a".to_string(),
            repeat: 1,
            gap_ms: 0,
        })]);
        e.poll(&obs(0, b""));
        assert!(e.is_done());
        assert!(!e.passed());
        assert_eq!(e.status(), "error");
        assert!(e.fault().unwrap().contains("--keyboard"));
    }

    #[test]
    fn key_events_step_without_a_keyboard_is_an_error_not_a_silent_pass() {
        let mut e = engine(vec![step(Op::KeyEvents {
            events: vec![KeyEvent {
                state: KeyState::Pressed,
                code: b'a',
            }],
            gap_ms: 0,
        })]);
        e.poll(&obs(0, b""));
        assert!(e.is_done());
        assert!(!e.passed());
        assert_eq!(e.status(), "error");
        assert!(e.fault().unwrap().contains("--keyboard"));
    }

    #[test]
    fn key_events_sequence_is_delivered_in_order_and_holds_remain_held() {
        let keyboard = Mutex::new(Keyboard::picocalc());

        let mut e = engine(vec![step(Op::KeyEvents {
            events: vec![
                KeyEvent {
                    state: KeyState::Pressed,
                    code: 11,
                },
                KeyEvent {
                    state: KeyState::Held,
                    code: 22,
                },
                KeyEvent {
                    state: KeyState::Released,
                    code: 11,
                },
            ],
            gap_ms: 0,
        })]);

        let obs = Observation {
            now_ns: 0,
            cycles: 0,
            lcd: None,
            keyboard: Some(&keyboard),
            uart: b"",
        };
        e.poll(&obs);
        assert!(e.is_done());
        assert!(e.passed());
        assert_eq!(e.results()[0].status, StepStatus::Pass);

        let mut guard = keyboard.lock().expect("keyboard mutex");
        guard.write_byte(REG_KEY_FIFO);
        let first = {
            let lo = guard.read_byte() as u16;
            let hi = guard.read_byte() as u16;
            lo | (hi << 8)
        };
        guard.transaction_end();
        guard.write_byte(REG_KEY_FIFO);
        let second = {
            let lo = guard.read_byte() as u16;
            let hi = guard.read_byte() as u16;
            lo | (hi << 8)
        };
        guard.transaction_end();
        guard.write_byte(REG_KEY_FIFO);
        let third = {
            let lo = guard.read_byte() as u16;
            let hi = guard.read_byte() as u16;
            lo | (hi << 8)
        };
        assert_eq!(first, (11u16 << 8) | KeyState::Pressed as u16);
        assert_eq!(second, (22u16 << 8) | KeyState::Held as u16);
        assert_eq!(third, (11u16 << 8) | KeyState::Released as u16);
    }

    #[test]
    fn an_unfinished_run_marks_every_remaining_step_incomplete() {
        let mut e = engine(vec![step(Op::Wait { ms: 100 }), step(Op::Wait { ms: 100 })]);
        e.poll(&obs(0, b""));
        e.finish_incomplete(&obs(50_000_000, b""));

        assert_eq!(e.status(), "incomplete");
        assert!(!e.passed());
        let statuses: Vec<_> = e.results().iter().map(|r| r.status).collect();
        assert_eq!(statuses, vec![StepStatus::Incomplete; 2]);
        // Only the step that was actually in flight explains itself; the
        // ones behind it were simply never reached.
        assert!(e.results()[0].detail.is_some());
        assert!(e.results()[1].detail.is_none());
    }

    #[test]
    fn a_finished_scenario_is_not_disturbed_by_finish_incomplete() {
        let mut e = engine(vec![step(Op::Wait { ms: 0 })]);
        e.poll(&obs(0, b""));
        assert!(e.passed());
        e.finish_incomplete(&obs(10_000_000, b""));
        assert!(e.passed(), "a completed scenario must stay passed");
        assert_eq!(e.results().len(), 1);
    }
}
