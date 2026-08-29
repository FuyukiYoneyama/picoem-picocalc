//! Versioned local IPC framing for the validated realtime preview.
//!
//! This is deliberately independent of the emulator and of the GUI.  The
//! wire contract is frozen by `picocalc_emu/docs/validated-realtime-preview/`
//! (schema 1): a 16-byte little-endian header followed by a bounded payload.
//! A reader validates the complete frame before returning it, so malformed
//! input can never be reinterpreted as another message kind.

use std::io::{self, Read, Write};

use serde_json::Value;

pub const MAGIC: [u8; 4] = *b"PCRP";
pub const VERSION: u16 = 1;
pub const HEADER_SIZE: usize = 16;
pub const MAX_PAYLOAD: usize = 8 * 1024 * 1024;
pub const MAX_AUDIO_FRAMES_PER_BLOCK: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    RunnerToPreview,
    PreviewToRunner,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum Kind {
    Hello = 1,
    Status = 2,
    FrameRgb565 = 3,
    AudioPcmS16 = 4,
    KeyEvent = 5,
    Reset = 6,
    Quit = 7,
    UartTx = 8,
    UartRx = 9,
    Error = 10,
    Goodbye = 11,
}

impl Kind {
    pub fn from_u16(value: u16) -> Result<Self, ProtocolError> {
        match value {
            1 => Ok(Self::Hello),
            2 => Ok(Self::Status),
            3 => Ok(Self::FrameRgb565),
            4 => Ok(Self::AudioPcmS16),
            5 => Ok(Self::KeyEvent),
            6 => Ok(Self::Reset),
            7 => Ok(Self::Quit),
            8 => Ok(Self::UartTx),
            9 => Ok(Self::UartRx),
            10 => Ok(Self::Error),
            11 => Ok(Self::Goodbye),
            other => Err(ProtocolError::new(format!(
                "unknown preview message kind {other}"
            ))),
        }
    }

    fn accepts(self, direction: Direction) -> bool {
        match direction {
            Direction::RunnerToPreview => matches!(
                self,
                Self::Hello
                    | Self::Status
                    | Self::FrameRgb565
                    | Self::AudioPcmS16
                    | Self::UartTx
                    | Self::Error
                    | Self::Goodbye
            ),
            Direction::PreviewToRunner => {
                matches!(
                    self,
                    Self::KeyEvent | Self::Reset | Self::Quit | Self::UartRx
                )
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub kind: Kind,
    pub sequence: u32,
    pub payload: Vec<u8>,
}

impl Frame {
    pub fn new(kind: Kind, sequence: u32, payload: Vec<u8>) -> Result<Self, ProtocolError> {
        let frame = Self {
            kind,
            sequence,
            payload,
        };
        frame.validate_payload()?;
        Ok(frame)
    }

    pub fn json_value(&self) -> Result<Value, ProtocolError> {
        match self.kind {
            Kind::Hello | Kind::Status | Kind::KeyEvent | Kind::Error | Kind::Goodbye => {
                parse_canonical_json(&self.payload)
            }
            _ => Err(ProtocolError::new(format!(
                "message kind {:?} does not carry JSON",
                self.kind
            ))),
        }
    }

    pub(crate) fn validate_for_direction(&self, direction: Direction) -> Result<(), ProtocolError> {
        if !self.kind.accepts(direction) {
            return Err(ProtocolError::new(format!(
                "message kind {:?} is invalid for {:?}",
                self.kind, direction
            )));
        }
        self.validate_payload()
    }

    fn validate_payload(&self) -> Result<(), ProtocolError> {
        if self.payload.len() > MAX_PAYLOAD {
            return Err(ProtocolError::new(format!(
                "payload is {} bytes, limit is {MAX_PAYLOAD}",
                self.payload.len()
            )));
        }
        match self.kind {
            Kind::Hello => {
                let value = parse_canonical_json(&self.payload)?;
                let object = value
                    .as_object()
                    .ok_or_else(|| ProtocolError::new("hello payload must be an object"))?;
                if object.get("protocol").and_then(Value::as_str) != Some("preview-ipc")
                    || object.get("role").and_then(Value::as_str) != Some("runner")
                    || object.get("schema").and_then(Value::as_u64) != Some(1)
                {
                    return Err(ProtocolError::new(
                        "hello must declare protocol=preview-ipc, role=runner, schema=1",
                    ));
                }
            }
            Kind::Status | Kind::Error | Kind::Goodbye => {
                let _ = parse_canonical_json(&self.payload)?;
            }
            Kind::KeyEvent => {
                let value = parse_canonical_json(&self.payload)?;
                let object = value
                    .as_object()
                    .ok_or_else(|| ProtocolError::new("key_event payload must be an object"))?;
                if object.get("key").and_then(Value::as_str).is_none() {
                    return Err(ProtocolError::new("key_event requires string field 'key'"));
                }
                match object.get("state").and_then(Value::as_str) {
                    Some("down" | "held" | "up") => {}
                    _ => {
                        return Err(ProtocolError::new(
                            "key_event state must be down, held, or up",
                        ));
                    }
                }
            }
            Kind::FrameRgb565 => {
                if self.payload.len() < 12 {
                    return Err(ProtocolError::new(
                        "frame_rgb565 payload is shorter than its prefix",
                    ));
                }
                let width = u16::from_le_bytes([self.payload[8], self.payload[9]]) as usize;
                let height = u16::from_le_bytes([self.payload[10], self.payload[11]]) as usize;
                let pixels = width
                    .checked_mul(height)
                    .and_then(|n| n.checked_mul(2))
                    .ok_or_else(|| ProtocolError::new("frame_rgb565 dimensions overflow"))?;
                let expected = 12usize
                    .checked_add(pixels)
                    .ok_or_else(|| ProtocolError::new("frame_rgb565 payload length overflow"))?;
                if self.payload.len() != expected {
                    return Err(ProtocolError::new(format!(
                        "frame_rgb565 payload length {} does not match {expected}",
                        self.payload.len()
                    )));
                }
            }
            Kind::AudioPcmS16 => {
                if self.payload.len() < 16 {
                    return Err(ProtocolError::new(
                        "audio_pcm_s16 payload is shorter than its prefix",
                    ));
                }
                let channels = u16::from_le_bytes([self.payload[12], self.payload[13]]) as usize;
                let frames = u16::from_le_bytes([self.payload[14], self.payload[15]]) as usize;
                if channels == 0 {
                    return Err(ProtocolError::new(
                        "audio_pcm_s16 channels must be non-zero",
                    ));
                }
                if frames > MAX_AUDIO_FRAMES_PER_BLOCK {
                    return Err(ProtocolError::new(format!(
                        "audio_pcm_s16 frames exceed per-block limit {MAX_AUDIO_FRAMES_PER_BLOCK}"
                    )));
                }
                let samples = frames
                    .checked_mul(channels)
                    .and_then(|n| n.checked_mul(2))
                    .ok_or_else(|| ProtocolError::new("audio_pcm_s16 dimensions overflow"))?;
                let expected = 16usize
                    .checked_add(samples)
                    .ok_or_else(|| ProtocolError::new("audio_pcm_s16 payload length overflow"))?;
                if self.payload.len() != expected {
                    return Err(ProtocolError::new(format!(
                        "audio_pcm_s16 payload length {} does not match {expected}",
                        self.payload.len()
                    )));
                }
            }
            Kind::UartTx => {
                if self.payload.len() != 9 {
                    return Err(ProtocolError::new("uart_tx payload must be 9 bytes"));
                }
            }
            Kind::UartRx => {
                if self.payload.len() != 1 {
                    return Err(ProtocolError::new("uart_rx payload must be 1 byte"));
                }
            }
            Kind::Reset | Kind::Quit => {
                if !self.payload.is_empty() {
                    return Err(ProtocolError::new(format!(
                        "message kind {:?} must have an empty payload",
                        self.kind
                    )));
                }
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProtocolError {
    message: String,
}

impl ProtocolError {
    pub(crate) fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl std::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ProtocolError {}

pub struct FrameReader<R> {
    reader: R,
    direction: Direction,
    next_sequence: u32,
}

impl<R: Read> FrameReader<R> {
    pub fn new(reader: R, direction: Direction) -> Self {
        Self {
            reader,
            direction,
            next_sequence: 0,
        }
    }

    pub fn read_frame(&mut self) -> Result<Option<Frame>, ProtocolError> {
        let mut header = [0u8; HEADER_SIZE];
        let mut first = [0u8; 1];
        match self.reader.read(&mut first) {
            Ok(0) => return Ok(None),
            Ok(1) => header[0] = first[0],
            Ok(_) => unreachable!(),
            Err(error) => {
                return Err(ProtocolError::new(format!(
                    "reading preview header: {error}"
                )));
            }
        }
        self.reader
            .read_exact(&mut header[1..])
            .map_err(|error| ProtocolError::new(format!("truncated preview header: {error}")))?;
        if header[0..4] != MAGIC {
            return Err(ProtocolError::new("bad preview IPC magic"));
        }
        let version = u16::from_le_bytes([header[4], header[5]]);
        if version != VERSION {
            return Err(ProtocolError::new(format!(
                "unsupported preview IPC version {version}"
            )));
        }
        let kind = Kind::from_u16(u16::from_le_bytes([header[6], header[7]]))?;
        let length = u32::from_le_bytes([header[8], header[9], header[10], header[11]]) as usize;
        if length > MAX_PAYLOAD {
            return Err(ProtocolError::new(format!(
                "preview payload is {length} bytes, limit is {MAX_PAYLOAD}"
            )));
        }
        let sequence = u32::from_le_bytes([header[12], header[13], header[14], header[15]]);
        if sequence != self.next_sequence {
            return Err(ProtocolError::new(format!(
                "preview sequence discontinuity: expected {}, got {sequence}",
                self.next_sequence
            )));
        }
        let mut payload = vec![0u8; length];
        self.reader
            .read_exact(&mut payload)
            .map_err(|error| ProtocolError::new(format!("truncated preview payload: {error}")))?;
        let frame = Frame {
            kind,
            sequence,
            payload,
        };
        frame.validate_for_direction(self.direction)?;
        self.next_sequence = self
            .next_sequence
            .checked_add(1)
            .ok_or_else(|| ProtocolError::new("preview sequence exhausted"))?;
        Ok(Some(frame))
    }
}

pub struct FrameWriter<W> {
    writer: W,
    direction: Direction,
    next_sequence: u32,
}

impl<W: Write> FrameWriter<W> {
    pub fn new(writer: W, direction: Direction) -> Self {
        Self {
            writer,
            direction,
            next_sequence: 0,
        }
    }

    pub fn write_frame(&mut self, mut frame: Frame) -> Result<u32, ProtocolError> {
        frame.sequence = self.next_sequence;
        frame.validate_for_direction(self.direction)?;
        let length = u32::try_from(frame.payload.len())
            .map_err(|_| ProtocolError::new("preview payload does not fit in u32"))?;
        let mut header = [0u8; HEADER_SIZE];
        header[0..4].copy_from_slice(&MAGIC);
        header[4..6].copy_from_slice(&VERSION.to_le_bytes());
        header[6..8].copy_from_slice(&(frame.kind as u16).to_le_bytes());
        header[8..12].copy_from_slice(&length.to_le_bytes());
        header[12..16].copy_from_slice(&frame.sequence.to_le_bytes());
        self.writer
            .write_all(&header)
            .and_then(|_| self.writer.write_all(&frame.payload))
            .and_then(|_| self.writer.flush())
            .map_err(|error| ProtocolError::new(format!("writing preview frame: {error}")))?;
        let sequence = self.next_sequence;
        self.next_sequence = self
            .next_sequence
            .checked_add(1)
            .ok_or_else(|| ProtocolError::new("preview sequence exhausted"))?;
        Ok(sequence)
    }
}

fn canonical_json(value: &Value) -> Result<Vec<u8>, ProtocolError> {
    serde_json::to_vec(value)
        .map_err(|error| ProtocolError::new(format!("serializing preview JSON: {error}")))
}

fn parse_canonical_json(bytes: &[u8]) -> Result<Value, ProtocolError> {
    let value: Value = serde_json::from_slice(bytes)
        .map_err(|error| ProtocolError::new(format!("malformed preview JSON: {error}")))?;
    let canonical = canonical_json(&value)?;
    if canonical != bytes {
        return Err(ProtocolError::new(
            "preview JSON is not canonical (sorted compact UTF-8 form required)",
        ));
    }
    Ok(value)
}

impl From<ProtocolError> for io::Error {
    fn from(error: ProtocolError) -> Self {
        io::Error::new(io::ErrorKind::InvalidData, error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn frame_hex(hex: &str) -> Vec<u8> {
        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn fixture_hello_round_trips_exactly() {
        let bytes = frame_hex(
            "504352500100010035000000000000007b2270726f746f636f6c223a22707265766965772d697063222c22726f6c65223a2272756e6e6572222c22736368656d61223a317d",
        );
        let mut reader = FrameReader::new(Cursor::new(bytes.clone()), Direction::RunnerToPreview);
        let frame = reader.read_frame().unwrap().unwrap();
        assert_eq!(frame.kind, Kind::Hello);
        assert_eq!(frame.sequence, 0);
        let mut output = Vec::new();
        let mut writer = FrameWriter::new(&mut output, Direction::RunnerToPreview);
        writer.write_frame(frame).unwrap();
        assert_eq!(output, bytes);
    }

    #[test]
    fn fixture_key_event_is_rejected_in_runner_direction() {
        let bytes = frame_hex(
            "50435250010005001e000000000000007b226b6579223a22456e746572222c227374617465223a22646f776e227d",
        );
        let mut reader = FrameReader::new(Cursor::new(bytes), Direction::RunnerToPreview);
        let error = reader.read_frame().unwrap_err();
        assert!(error.to_string().contains("invalid"));
    }

    #[test]
    fn malformed_and_truncated_frames_fail_closed() {
        let unknown = frame_hex("504352500100ffff0000000007000000");
        let mut reader = FrameReader::new(Cursor::new(unknown), Direction::RunnerToPreview);
        assert!(reader.read_frame().is_err());

        let truncated = frame_hex(
            "504352500100020023000000080000007b22636f766572616765223a226f6b222c227669727475616c5f6379636c65223a30",
        );
        let mut reader = FrameReader::new(Cursor::new(truncated), Direction::RunnerToPreview);
        assert!(reader.read_frame().is_err());
    }

    #[test]
    fn json_must_be_canonical() {
        let value = serde_json::json!({"b": 2, "a": 1});
        let canonical = canonical_json(&value).unwrap();
        assert!(parse_canonical_json(&canonical).is_ok());
        assert!(parse_canonical_json(br#"{"b":2,"a":1}"#).is_err());
    }

    #[test]
    fn audio_block_frame_count_is_bounded_by_schema() {
        let mut payload = Vec::with_capacity(16 + 129 * 2);
        payload.extend_from_slice(&0u64.to_le_bytes());
        payload.extend_from_slice(&48_000u32.to_le_bytes());
        payload.extend_from_slice(&1u16.to_le_bytes());
        payload.extend_from_slice(&129u16.to_le_bytes());
        payload.resize(16 + 129 * 2, 0);
        assert!(Frame::new(Kind::AudioPcmS16, 0, payload).is_err());
    }
}
