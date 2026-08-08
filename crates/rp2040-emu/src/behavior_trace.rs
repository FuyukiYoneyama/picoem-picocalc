//! OPT0-B deterministic streaming event contract.
//!
//! Events use one canonical binary framing and are folded into SHA-256 as
//! they occur. Only the hash state and counters are retained; memory use is
//! constant even for billion-cycle firmware runs.

use sha2::{Digest, Sha256};

pub const BEHAVIOR_TRACE_SCHEMA_VERSION: u32 = 1;
const EVENT_MAGIC: &[u8] = b"PICOEM-EVENT\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BehaviorEventDomain {
    Clock = 0,
    IrqException = 1,
    PioGpio = 2,
    Psram = 3,
    Lcd = 4,
    DmaDreq = 5,
    TimerPwm = 6,
    SerialBus = 7,
    ScenarioInput = 8,
}

impl BehaviorEventDomain {
    pub const ALL: [Self; 9] = [
        Self::Clock,
        Self::IrqException,
        Self::PioGpio,
        Self::Psram,
        Self::Lcd,
        Self::DmaDreq,
        Self::TimerPwm,
        Self::SerialBus,
        Self::ScenarioInput,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Clock => "clock",
            Self::IrqException => "irq_exception",
            Self::PioGpio => "pio_gpio",
            Self::Psram => "psram",
            Self::Lcd => "lcd",
            Self::DmaDreq => "dma_dreq",
            Self::TimerPwm => "timer_pwm",
            Self::SerialBus => "serial_bus",
            Self::ScenarioInput => "scenario_input",
        }
    }

    const fn index(self) -> usize {
        self as usize
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct PwmObservation {
    pub enabled: bool,
    pub ctr: u16,
    pub top: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PsramObservation {
    pub cs_falling_count: u64,
    pub bytes_written: u64,
    pub bytes_read: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct BehaviorObservation {
    pub cycle: u64,
    pub clock_hz: [u32; 3],
    pub irq: [u32; 5],
    pub gpio_in: u32,
    pub pio_state: [u32; 4],
    pub dma_transfers: [u64; crate::dma::NUM_CHANNELS],
    pub timer: [u64; 6],
    pub pwm: [PwmObservation; 8],
    pub psram: Option<PsramObservation>,
    pub serial: [u64; 38],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BehaviorTraceDomainSnapshot {
    pub domain: BehaviorEventDomain,
    pub events: u64,
    pub sha256: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BehaviorTraceSnapshot {
    pub schema_version: u32,
    pub total_events: u64,
    pub sha256: String,
    pub domains: Vec<BehaviorTraceDomainSnapshot>,
}

struct DomainAccumulator {
    hasher: Sha256,
    events: u64,
}

pub(crate) struct BehaviorTracer {
    all: Sha256,
    total_events: u64,
    domains: [DomainAccumulator; 9],
    previous: BehaviorObservation,
    pio_domains: [Option<BehaviorEventDomain>; 2],
    gpio_input_domain: Option<BehaviorEventDomain>,
}

impl BehaviorTracer {
    pub(crate) fn new(initial: BehaviorObservation) -> Self {
        let mut value = Self {
            all: Sha256::new(),
            total_events: 0,
            domains: std::array::from_fn(|_| DomainAccumulator {
                hasher: Sha256::new(),
                events: 0,
            }),
            previous: initial.clone(),
            pio_domains: [None; 2],
            gpio_input_domain: None,
        };
        value.record_initial(&initial);
        value
    }

    pub(crate) fn record(
        &mut self,
        domain: BehaviorEventDomain,
        source: u16,
        cycle: u64,
        payload: &[u8],
    ) {
        let mut framed = Vec::with_capacity(EVENT_MAGIC.len() + 19 + payload.len());
        framed.extend_from_slice(EVENT_MAGIC);
        framed.extend_from_slice(&BEHAVIOR_TRACE_SCHEMA_VERSION.to_be_bytes());
        framed.push(domain as u8);
        framed.extend_from_slice(&source.to_be_bytes());
        framed.extend_from_slice(&cycle.to_be_bytes());
        framed.extend_from_slice(&(payload.len() as u32).to_be_bytes());
        framed.extend_from_slice(payload);
        self.all.update(&framed);
        self.total_events = self.total_events.wrapping_add(1);
        let accumulator = &mut self.domains[domain.index()];
        accumulator.hasher.update(&framed);
        accumulator.events = accumulator.events.wrapping_add(1);
    }

    pub(crate) fn observe(&mut self, current: BehaviorObservation) {
        let previous = self.previous.clone();
        if current.clock_hz != previous.clock_hz {
            self.record_words(
                BehaviorEventDomain::Clock,
                1,
                current.cycle,
                &current.clock_hz,
            );
        }
        if current.irq != previous.irq {
            self.record_words(
                BehaviorEventDomain::IrqException,
                1,
                current.cycle,
                &current.irq,
            );
        }
        if current.gpio_in != previous.gpio_in {
            let mut payload = Vec::with_capacity(4);
            push_u32(&mut payload, current.gpio_in);
            self.record(
                self.gpio_input_domain
                    .unwrap_or(BehaviorEventDomain::PioGpio),
                3,
                current.cycle,
                &payload,
            );
        }
        for block in 0..2 {
            let base = block * 2;
            if current.pio_state[base..base + 2] != previous.pio_state[base..base + 2] {
                let mut payload = Vec::with_capacity(8);
                push_u32(&mut payload, current.pio_state[base]);
                push_u32(&mut payload, current.pio_state[base + 1]);
                self.record(
                    self.pio_domains[block].unwrap_or(BehaviorEventDomain::PioGpio),
                    block as u16 + 1,
                    current.cycle,
                    &payload,
                );
            }
        }
        if current.psram != previous.psram {
            let mut payload = Vec::new();
            if let Some(value) = current.psram {
                for item in [
                    value.cs_falling_count,
                    value.bytes_written,
                    value.bytes_read,
                ] {
                    push_u64(&mut payload, item);
                }
            }
            self.record(BehaviorEventDomain::Psram, 1, current.cycle, &payload);
        }
        if current.dma_transfers != previous.dma_transfers {
            let mut payload = Vec::with_capacity(current.dma_transfers.len() * 8);
            for value in current.dma_transfers {
                push_u64(&mut payload, value);
            }
            self.record(BehaviorEventDomain::DmaDreq, 1, current.cycle, &payload);
        }
        if current.timer != previous.timer || pwm_boundary(&previous.pwm, &current.pwm) {
            let mut payload = Vec::with_capacity(6 * 8 + 8 * 5);
            for value in current.timer {
                push_u64(&mut payload, value);
            }
            for value in current.pwm {
                payload.push(u8::from(value.enabled));
                payload.extend_from_slice(&value.ctr.to_be_bytes());
                payload.extend_from_slice(&value.top.to_be_bytes());
            }
            self.record(BehaviorEventDomain::TimerPwm, 1, current.cycle, &payload);
        }
        if current.serial != previous.serial {
            let mut payload = Vec::with_capacity(current.serial.len() * 8);
            for value in current.serial {
                push_u64(&mut payload, value);
            }
            self.record(BehaviorEventDomain::SerialBus, 2, current.cycle, &payload);
        }
        self.previous = current;
    }

    pub(crate) fn snapshot(&self) -> BehaviorTraceSnapshot {
        BehaviorTraceSnapshot {
            schema_version: BEHAVIOR_TRACE_SCHEMA_VERSION,
            total_events: self.total_events,
            sha256: hex(self.all.clone().finalize().as_slice()),
            domains: BehaviorEventDomain::ALL
                .into_iter()
                .map(|domain| {
                    let value = &self.domains[domain.index()];
                    BehaviorTraceDomainSnapshot {
                        domain,
                        events: value.events,
                        sha256: hex(value.hasher.clone().finalize().as_slice()),
                    }
                })
                .collect(),
        }
    }

    pub(crate) fn map_pio_domain(
        &mut self,
        block: usize,
        domain: BehaviorEventDomain,
        cycle: u64,
        state: [u32; 2],
    ) {
        if block >= self.pio_domains.len() {
            return;
        }
        self.pio_domains[block] = Some(domain);
        let mut payload = Vec::with_capacity(8);
        push_u32(&mut payload, state[0]);
        push_u32(&mut payload, state[1]);
        self.record(domain, 0x100 + block as u16, cycle, &payload);
    }

    pub(crate) fn map_gpio_input_domain(
        &mut self,
        domain: BehaviorEventDomain,
        cycle: u64,
        gpio_in: u32,
    ) {
        self.gpio_input_domain = Some(domain);
        self.record(domain, 0x103, cycle, &gpio_in.to_be_bytes());
    }

    fn record_initial(&mut self, value: &BehaviorObservation) {
        self.record_words(BehaviorEventDomain::Clock, 0, value.cycle, &value.clock_hz);
        self.record_words(
            BehaviorEventDomain::IrqException,
            0,
            value.cycle,
            &value.irq,
        );
        let mut gpio = Vec::new();
        push_u32(&mut gpio, value.gpio_in);
        for item in value.pio_state {
            push_u32(&mut gpio, item);
        }
        self.record(BehaviorEventDomain::PioGpio, 0, value.cycle, &gpio);
        self.record(BehaviorEventDomain::Psram, 0, value.cycle, &[]);
        self.record(BehaviorEventDomain::Lcd, 0, value.cycle, &[]);
        self.record(BehaviorEventDomain::DmaDreq, 0, value.cycle, &[]);
        self.record(BehaviorEventDomain::TimerPwm, 0, value.cycle, &[]);
        let mut serial = Vec::with_capacity(value.serial.len() * 8);
        for item in value.serial {
            push_u64(&mut serial, item);
        }
        self.record(BehaviorEventDomain::SerialBus, 0, value.cycle, &serial);
        self.record(BehaviorEventDomain::ScenarioInput, 0, value.cycle, &[]);
    }

    fn record_words<const N: usize>(
        &mut self,
        domain: BehaviorEventDomain,
        source: u16,
        cycle: u64,
        values: &[u32; N],
    ) {
        let mut payload = Vec::with_capacity(N * 4);
        for value in values {
            push_u32(&mut payload, *value);
        }
        self.record(domain, source, cycle, &payload);
    }
}

fn pwm_boundary(previous: &[PwmObservation; 8], current: &[PwmObservation; 8]) -> bool {
    previous.iter().zip(current).any(|(before, after)| {
        before.enabled != after.enabled
            || before.top != after.top
            || (before.enabled && after.enabled && after.ctr < before.ctr)
    })
}

fn push_u32(dst: &mut Vec<u8>, value: u32) {
    dst.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(dst: &mut Vec<u8>, value: u64) {
    dst.extend_from_slice(&value.to_be_bytes());
}

fn hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write;
        let _ = write!(out, "{byte:02x}");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation() -> BehaviorObservation {
        BehaviorObservation {
            cycle: 0,
            clock_hz: [125_000_000; 3],
            irq: [0; 5],
            gpio_in: 0,
            pio_state: [0; 4],
            dma_transfers: [0; crate::dma::NUM_CHANNELS],
            timer: [0; 6],
            pwm: [PwmObservation::default(); 8],
            psram: None,
            serial: [0; 38],
        }
    }

    #[test]
    fn same_events_produce_same_streaming_digest() {
        let mut a = BehaviorTracer::new(observation());
        let mut b = BehaviorTracer::new(observation());
        a.record(BehaviorEventDomain::ScenarioInput, 7, 11, b"key:a");
        b.record(BehaviorEventDomain::ScenarioInput, 7, 11, b"key:a");
        assert_eq!(a.snapshot(), b.snapshot());
    }

    #[test]
    fn cycle_domain_source_and_payload_all_affect_digest() {
        let baseline = BehaviorTracer::new(observation()).snapshot().sha256;
        for (domain, source, cycle, payload) in [
            (BehaviorEventDomain::Clock, 1, 0, b"".as_slice()),
            (BehaviorEventDomain::ScenarioInput, 2, 0, b"".as_slice()),
            (BehaviorEventDomain::ScenarioInput, 1, 1, b"".as_slice()),
            (BehaviorEventDomain::ScenarioInput, 1, 0, b"x".as_slice()),
        ] {
            let mut value = BehaviorTracer::new(observation());
            value.record(domain, source, cycle, payload);
            assert_ne!(value.snapshot().sha256, baseline);
        }
    }

    #[test]
    fn unchanged_observation_emits_no_event() {
        let initial = observation();
        let mut value = BehaviorTracer::new(initial.clone());
        let before = value.snapshot();
        value.observe(initial);
        assert_eq!(value.snapshot(), before);
    }

    #[test]
    fn mapped_pio_edges_are_partitioned_into_the_device_domain() {
        let initial = observation();
        let mut value = BehaviorTracer::new(initial.clone());
        value.map_pio_domain(0, BehaviorEventDomain::Lcd, 0, [0, 0]);
        let before = value.snapshot();
        let mut changed = initial;
        changed.cycle = 1;
        changed.pio_state[0] = 1;
        value.observe(changed);
        let after = value.snapshot();
        let lcd_before = before
            .domains
            .iter()
            .find(|item| item.domain == BehaviorEventDomain::Lcd)
            .unwrap();
        let lcd_after = after
            .domains
            .iter()
            .find(|item| item.domain == BehaviorEventDomain::Lcd)
            .unwrap();
        assert_eq!(lcd_after.events, lcd_before.events + 1);
    }
}
