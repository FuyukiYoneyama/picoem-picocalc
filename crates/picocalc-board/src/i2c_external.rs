//! Optional external I2C device routing for private PicoCalc modules.
//!
//! The RP2040 controller intentionally sees one off-chip device. This board
//! layer supplies the deterministic address mux used by an explicitly selected
//! profile, while the default keyboard-only path continues to use the legacy
//! single-device attachment. New hardware models should be independent child
//! types and register their address here; they must not add runner-specific
//! address conditionals.

use std::fmt;

use rp2040_emu::peripherals::i2c::{I2cExternalDevice, I2cVirtualTimeDelta};

use crate::sha256::StreamingSha256;

/// Configuration errors caught before a profile is attached to the emulator.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum I2cBusMuxError {
    AddressOutOfRange(u16),
    DuplicateAddress(u16),
}

impl fmt::Display for I2cBusMuxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AddressOutOfRange(addr) => {
                write!(f, "I2C address 0x{addr:02x} is outside the 7-bit range")
            }
            Self::DuplicateAddress(addr) => {
                write!(f, "duplicate I2C profile address 0x{addr:02x}")
            }
        }
    }
}

impl std::error::Error for I2cBusMuxError {}

struct Child {
    address: u16,
    device: Box<dyn I2cExternalDevice>,
    observation: ChildObservation,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct ChildObservation {
    address_phases: u64,
    address_acks: u64,
    address_nacks: u64,
    write_bytes: u64,
    read_bytes: u64,
    stop_count: u64,
    data_nacks: u64,
}

/// Per-device wire counters and final model state for an optional I2C
/// profile. This is emitted only in the profile sidecar; it is not part of
/// the primary firmware report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct I2cChildObservation {
    pub address: u16,
    pub model: String,
    pub address_phases: u64,
    pub address_acks: u64,
    pub address_nacks: u64,
    pub write_bytes: u64,
    pub read_bytes: u64,
    pub stop_count: u64,
    pub data_nacks: u64,
    pub protocol_errors: u64,
    pub state_summary: String,
}

/// Deterministic wire observation for one explicitly selected I2C profile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct I2cBusObservation {
    pub address_phases: u64,
    pub address_acks: u64,
    pub address_nacks: u64,
    pub unknown_addresses: u64,
    pub write_bytes: u64,
    pub read_bytes: u64,
    pub stop_count: u64,
    pub data_nacks: u64,
    pub protocol_errors: u64,
    pub transaction_digest_sha256: String,
    pub children: Vec<I2cChildObservation>,
}

/// Deterministic address router for an explicitly selected I2C profile.
pub struct I2cBusMux {
    children: Vec<Child>,
    active: Option<usize>,
    address_phases: u64,
    address_acks: u64,
    address_nacks: u64,
    unknown_addresses: u64,
    write_bytes: u64,
    read_bytes: u64,
    stop_count: u64,
    data_nacks: u64,
    transaction_digest: StreamingSha256,
}

impl I2cBusMux {
    pub fn new() -> Self {
        Self {
            children: Vec::new(),
            active: None,
            address_phases: 0,
            address_acks: 0,
            address_nacks: 0,
            unknown_addresses: 0,
            write_bytes: 0,
            read_bytes: 0,
            stop_count: 0,
            data_nacks: 0,
            transaction_digest: StreamingSha256::new(),
        }
    }

    fn digest_event(&mut self, kind: u8, address: u16, value: u8, ack: bool) {
        self.transaction_digest.update(&[
            kind,
            (address >> 8) as u8,
            address as u8,
            value,
            u8::from(ack),
        ]);
    }

    /// Add one child at its declared 7-bit address.
    pub fn add_device(
        &mut self,
        address: u16,
        device: Box<dyn I2cExternalDevice>,
    ) -> Result<(), I2cBusMuxError> {
        if address >= 0x80 {
            return Err(I2cBusMuxError::AddressOutOfRange(address));
        }
        if self.children.iter().any(|child| child.address == address) {
            return Err(I2cBusMuxError::DuplicateAddress(address));
        }
        self.children.push(Child {
            address,
            device,
            observation: ChildObservation::default(),
        });
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.children.len()
    }

    pub fn is_empty(&self) -> bool {
        self.children.is_empty()
    }

    pub fn addresses(&self) -> impl Iterator<Item = u16> + '_ {
        self.children.iter().map(|child| child.address)
    }

    /// Return deterministic transaction counters, a streaming wire digest,
    /// and final state summaries for all attached children.
    pub fn observation(&self) -> I2cBusObservation {
        let mut protocol_errors = 0;
        let children = self
            .children
            .iter()
            .map(|child| {
                let child_errors = child.device.protocol_error_count();
                protocol_errors += child_errors;
                I2cChildObservation {
                    address: child.address,
                    model: child.device.model_name().to_string(),
                    address_phases: child.observation.address_phases,
                    address_acks: child.observation.address_acks,
                    address_nacks: child.observation.address_nacks,
                    write_bytes: child.observation.write_bytes,
                    read_bytes: child.observation.read_bytes,
                    stop_count: child.observation.stop_count,
                    data_nacks: child.observation.data_nacks,
                    protocol_errors: child_errors,
                    state_summary: child.device.state_summary(),
                }
            })
            .collect();
        I2cBusObservation {
            address_phases: self.address_phases,
            address_acks: self.address_acks,
            address_nacks: self.address_nacks,
            unknown_addresses: self.unknown_addresses,
            write_bytes: self.write_bytes,
            read_bytes: self.read_bytes,
            stop_count: self.stop_count,
            data_nacks: self.data_nacks,
            protocol_errors,
            transaction_digest_sha256: self.transaction_digest.finalize_hex(),
            children,
        }
    }
}

impl Default for I2cBusMux {
    fn default() -> Self {
        Self::new()
    }
}

impl I2cExternalDevice for I2cBusMux {
    fn responds_to(&self, addr: u16) -> bool {
        self.children
            .iter()
            .any(|child| child.address == addr && child.device.responds_to(addr))
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        self.address_phases = self.address_phases.wrapping_add(1);
        let Some(index) = self.children.iter().position(|child| child.address == addr) else {
            self.address_nacks = self.address_nacks.wrapping_add(1);
            self.unknown_addresses = self.unknown_addresses.wrapping_add(1);
            self.digest_event(b'A', addr, 0, false);
            return false;
        };

        // The bounded E1 controller contract permits a repeated START only
        // for the same target. A different target is selected by STOP,
        // disable, IC_TAR, enable, then a new address phase.
        if self.active.is_some_and(|active| active != index) {
            self.address_nacks = self.address_nacks.wrapping_add(1);
            self.children[index].observation.address_nacks = self.children[index]
                .observation
                .address_nacks
                .wrapping_add(1);
            self.digest_event(b'A', addr, 0, false);
            return false;
        }
        let ack = self.children[index].device.address_phase(addr);
        self.digest_event(b'A', addr, 0, ack);
        if ack {
            self.address_acks = self.address_acks.wrapping_add(1);
            self.children[index].observation.address_phases = self.children[index]
                .observation
                .address_phases
                .wrapping_add(1);
            self.children[index].observation.address_acks = self.children[index]
                .observation
                .address_acks
                .wrapping_add(1);
            self.active = Some(index);
            true
        } else {
            self.address_nacks = self.address_nacks.wrapping_add(1);
            self.children[index].observation.address_phases = self.children[index]
                .observation
                .address_phases
                .wrapping_add(1);
            self.children[index].observation.address_nacks = self.children[index]
                .observation
                .address_nacks
                .wrapping_add(1);
            false
        }
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        let Some(index) = self.active else {
            self.data_nacks = self.data_nacks.wrapping_add(1);
            return false;
        };
        let address = self.children[index].address;
        let ack = self.children[index].device.write_byte(byte);
        self.write_bytes = self.write_bytes.wrapping_add(1);
        self.children[index].observation.write_bytes =
            self.children[index].observation.write_bytes.wrapping_add(1);
        if !ack {
            self.data_nacks = self.data_nacks.wrapping_add(1);
            self.children[index].observation.data_nacks =
                self.children[index].observation.data_nacks.wrapping_add(1);
        }
        self.digest_event(b'W', address, byte, ack);
        ack
    }

    fn read_byte(&mut self) -> u8 {
        let Some(index) = self.active else {
            self.read_bytes = self.read_bytes.wrapping_add(1);
            self.digest_event(b'R', 0, 0xff, false);
            return 0xFF;
        };
        let address = self.children[index].address;
        let byte = self.children[index].device.read_byte();
        self.read_bytes = self.read_bytes.wrapping_add(1);
        self.children[index].observation.read_bytes =
            self.children[index].observation.read_bytes.wrapping_add(1);
        self.digest_event(b'R', address, byte, true);
        byte
    }

    fn transaction_end(&mut self) {
        if let Some(index) = self.active.take() {
            let address = self.children[index].address;
            self.children[index].device.transaction_end();
            self.children[index].observation.stop_count =
                self.children[index].observation.stop_count.wrapping_add(1);
            self.stop_count = self.stop_count.wrapping_add(1);
            self.digest_event(b'S', address, 0, true);
        }
    }

    fn advance_virtual_time(&mut self, delta: I2cVirtualTimeDelta) {
        for child in &mut self.children {
            child.device.advance_virtual_time(delta);
        }
    }
}

/// Small public connection-point example for downstream module authors.
///
/// It intentionally NACKs data bytes and returns a fixed read byte. It is not
/// registered in any profile and never attaches to the default board. Its
/// purpose is to make the child lifecycle and fail-closed behavior explicit in
/// unit tests and in forks adding a private module.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct I2cExternalDeviceStub {
    address: u16,
    read_value: u8,
}

impl I2cExternalDeviceStub {
    pub fn new(address: u16, read_value: u8) -> Result<Self, I2cBusMuxError> {
        if address >= 0x80 {
            return Err(I2cBusMuxError::AddressOutOfRange(address));
        }
        Ok(Self {
            address,
            read_value,
        })
    }
}

impl I2cExternalDevice for I2cExternalDeviceStub {
    fn responds_to(&self, addr: u16) -> bool {
        addr == self.address
    }

    fn address_phase(&mut self, addr: u16) -> bool {
        self.responds_to(addr)
    }

    fn write_byte(&mut self, _byte: u8) -> bool {
        false
    }

    fn read_byte(&mut self) -> u8 {
        self.read_value
    }

    fn transaction_end(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct Probe {
        address: u16,
        bytes: Vec<u8>,
        reads: u8,
        stops: u32,
        advances: Vec<u64>,
    }

    #[derive(Clone)]
    struct ProbeWire(Arc<Mutex<Probe>>);

    impl I2cExternalDevice for ProbeWire {
        fn responds_to(&self, addr: u16) -> bool {
            self.0.lock().unwrap().address == addr
        }

        fn address_phase(&mut self, addr: u16) -> bool {
            self.responds_to(addr)
        }

        fn write_byte(&mut self, byte: u8) -> bool {
            self.0.lock().unwrap().bytes.push(byte);
            true
        }

        fn read_byte(&mut self) -> u8 {
            let mut p = self.0.lock().unwrap();
            let value = p.reads;
            p.reads = p.reads.wrapping_add(1);
            value
        }

        fn transaction_end(&mut self) {
            self.0.lock().unwrap().stops += 1;
        }

        fn advance_virtual_time(&mut self, delta: I2cVirtualTimeDelta) {
            self.0.lock().unwrap().advances.push(delta.nanoseconds);
        }
    }

    #[test]
    fn duplicate_and_out_of_range_addresses_are_rejected() {
        let mut mux = I2cBusMux::new();
        mux.add_device(0x38, Box::new(I2cExternalDeviceStub::new(0x38, 0).unwrap()))
            .unwrap();
        assert_eq!(
            mux.add_device(0x38, Box::new(I2cExternalDeviceStub::new(0x38, 0).unwrap())),
            Err(I2cBusMuxError::DuplicateAddress(0x38))
        );
        assert_eq!(
            I2cExternalDeviceStub::new(0x80, 0),
            Err(I2cBusMuxError::AddressOutOfRange(0x80))
        );
    }

    #[test]
    fn only_active_child_receives_wire_and_stop() {
        let first = Arc::new(Mutex::new(Probe {
            address: 0x38,
            ..Default::default()
        }));
        let second = Arc::new(Mutex::new(Probe {
            address: 0x68,
            ..Default::default()
        }));
        let mut mux = I2cBusMux::new();
        mux.add_device(0x38, Box::new(ProbeWire(first.clone())))
            .unwrap();
        mux.add_device(0x68, Box::new(ProbeWire(second.clone())))
            .unwrap();

        assert!(mux.address_phase(0x38));
        assert!(mux.write_byte(0xAA));
        assert_eq!(mux.read_byte(), 0);
        mux.transaction_end();
        assert_eq!(first.lock().unwrap().bytes, vec![0xAA]);
        assert_eq!(first.lock().unwrap().stops, 1);
        assert!(second.lock().unwrap().bytes.is_empty());

        assert!(mux.address_phase(0x68));
        assert!(mux.write_byte(0xBB));
        mux.transaction_end();
        assert_eq!(second.lock().unwrap().bytes, vec![0xBB]);
        assert_eq!(second.lock().unwrap().stops, 1);
    }

    #[test]
    fn repeated_start_to_other_child_is_rejected_without_stop() {
        let first = Arc::new(Mutex::new(Probe {
            address: 0x38,
            ..Default::default()
        }));
        let second = Arc::new(Mutex::new(Probe {
            address: 0x68,
            ..Default::default()
        }));
        let mut mux = I2cBusMux::new();
        mux.add_device(0x38, Box::new(ProbeWire(first.clone())))
            .unwrap();
        mux.add_device(0x68, Box::new(ProbeWire(second.clone())))
            .unwrap();
        assert!(mux.address_phase(0x38));
        assert!(!mux.address_phase(0x68));
        assert_eq!(first.lock().unwrap().stops, 0);
        assert_eq!(second.lock().unwrap().stops, 0);
    }

    #[test]
    fn virtual_time_is_forwarded_to_all_children() {
        let first = Arc::new(Mutex::new(Probe {
            address: 0x38,
            ..Default::default()
        }));
        let second = Arc::new(Mutex::new(Probe {
            address: 0x68,
            ..Default::default()
        }));
        let mut mux = I2cBusMux::new();
        mux.add_device(0x38, Box::new(ProbeWire(first.clone())))
            .unwrap();
        mux.add_device(0x68, Box::new(ProbeWire(second.clone())))
            .unwrap();
        mux.advance_virtual_time(I2cVirtualTimeDelta { nanoseconds: 123 });
        assert_eq!(first.lock().unwrap().advances, vec![123]);
        assert_eq!(second.lock().unwrap().advances, vec![123]);
    }

    #[test]
    fn observation_is_deterministic_and_counts_wire_events() {
        let first = Arc::new(Mutex::new(Probe {
            address: 0x38,
            ..Default::default()
        }));
        let mut mux = I2cBusMux::new();
        mux.add_device(0x38, Box::new(ProbeWire(first.clone())))
            .unwrap();

        assert!(mux.address_phase(0x38));
        assert!(mux.write_byte(0xaa));
        assert_eq!(mux.read_byte(), 0);
        mux.transaction_end();
        assert!(!mux.address_phase(0x39));

        let observation = mux.observation();
        assert_eq!(observation.address_phases, 2);
        assert_eq!(observation.address_acks, 1);
        assert_eq!(observation.address_nacks, 1);
        assert_eq!(observation.unknown_addresses, 1);
        assert_eq!(observation.write_bytes, 1);
        assert_eq!(observation.read_bytes, 1);
        assert_eq!(observation.stop_count, 1);
        assert_eq!(observation.data_nacks, 0);
        assert_eq!(observation.children[0].address, 0x38);
        assert_eq!(observation.children[0].write_bytes, 1);
        assert_eq!(observation.children[0].read_bytes, 1);
        assert_eq!(observation.children[0].stop_count, 1);
        assert_eq!(observation.transaction_digest_sha256.len(), 64);

        let again = mux.observation();
        assert_eq!(observation, again);
    }
}
