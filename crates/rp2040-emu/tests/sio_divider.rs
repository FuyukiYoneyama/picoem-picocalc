//! Specification-based RP2040 divider regression checks, independent of apps.

use rp2040_emu::bus::sio::Sio;
use rp2040_emu::core::bus_trait::CoreBus;
use rp2040_emu::{Bus, Config, CortexM0Plus, Emulator};

const SIO: u32 = 0xd000_0000;

trait Registers {
    fn read(&mut self, core: usize, offset: u32) -> u32;
    fn write(&mut self, core: usize, offset: u32, value: u32);
    fn advance(&mut self, core: usize, cycles: u32);
}

impl Registers for Sio {
    fn read(&mut self, core: usize, offset: u32) -> u32 {
        self.read32(offset, core)
    }
    fn write(&mut self, core: usize, offset: u32, value: u32) {
        self.write32(offset, value, core);
    }
    fn advance(&mut self, core: usize, cycles: u32) {
        self.advance_divider_cycles(core, cycles as u64);
    }
}

impl Registers for Bus {
    fn read(&mut self, core: usize, offset: u32) -> u32 {
        self.set_active_core(core);
        self.read32(SIO + offset)
    }
    fn write(&mut self, core: usize, offset: u32, value: u32) {
        self.set_active_core(core);
        self.write32(SIO + offset, value);
    }
    fn advance(&mut self, core: usize, cycles: u32) {
        self.set_active_core(core);
        self.advance_core_cycles(cycles);
    }
}

fn result(r: &mut impl Registers, core: usize) -> (u32, u32) {
    let remainder = r.read(core, 0x074);
    let quotient = r.read(core, 0x070);
    (quotient, remainder)
}

fn arithmetic(r: &mut impl Registers) {
    // Independent host reference uses wider integers, including i32 MIN/-1.
    let mut random = 0x4178_9523u32;
    for core in 0..2 {
        for signed in [false, true] {
            let (dividend, divisor) = if signed {
                (0x068, 0x06c)
            } else {
                (0x060, 0x064)
            };
            for divisor_first in [false, true] {
                for i in 0..256 {
                    random = random.wrapping_mul(1664525).wrapping_add(1013904223);
                    let a = if i == 0 { 0x8000_0000 } else { random };
                    random = random.wrapping_mul(1664525).wrapping_add(1013904223);
                    let b = if i == 0 { u32::MAX } else { random | 1 };
                    if divisor_first {
                        r.write(core, divisor, b);
                        r.write(core, dividend, a);
                    } else {
                        r.write(core, dividend, a);
                        r.write(core, divisor, b);
                    }
                    assert_eq!(r.read(core, 0x078), 2);
                    r.advance(core, 7);
                    assert_eq!(r.read(core, 0x078), 2);
                    r.advance(core, 1);
                    assert_eq!(r.read(core, 0x078), 3);
                    let expected = if signed {
                        let (a, b) = (a as i32 as i64, b as i32 as i64);
                        ((a / b) as u32, (a % b) as u32)
                    } else {
                        let (a, b) = (a as u64, b as u64);
                        ((a / b) as u32, (a % b) as u32)
                    };
                    assert_eq!(
                        result(r, core),
                        expected,
                        "core={core} signed={signed} divisor_first={divisor_first}"
                    );
                    assert_eq!(r.read(core, 0x078), 1);
                }
            }
        }
    }
}

fn status_and_restart(r: &mut impl Registers) {
    for core in 0..2 {
        r.write(core, 0x064, 3);
        r.write(core, 0x060, 10);
        r.advance(core, 7);
        r.write(core, 0x060, 20); // Restarts, rather than finishing the old divide.
        r.advance(core, 1);
        assert_eq!(r.read(core, 0x078), 2);
        r.advance(core, 7);
        assert_eq!(r.read(core, 0x074), 2);
        assert_eq!(r.read(core, 0x074), 2);
        assert_eq!(r.read(core, 0x078), 3);
        assert_eq!(r.read(core, 0x070), 6);
        assert_eq!(r.read(core, 0x078), 1);
        // Writing the same divisor still restarts the full calculation.
        r.write(core, 0x060, 20);
        r.advance(core, 7);
        r.write(core, 0x064, 3);
        r.advance(core, 1);
        assert_eq!(r.read(core, 0x078), 2);
        r.advance(core, 7);
        assert_eq!(result(r, core), (6, 2));
        r.write(core, 0x068, (-10i32) as u32); // Last alias selects signedness.
        r.advance(core, 8);
        assert_eq!(result(r, core), ((-3i32) as u32, u32::MAX));
        r.write(core, 0x060, u32::MAX); // Switch back to unsigned via dividend.
        r.write(core, 0x064, 65537); // SDK reciprocal divisor.
        r.write(core, 0x060, u32::MAX); // SDK's divisor-first sequence.
        r.advance(core, 8);
        assert_eq!(result(r, core), (65535, 0));

        for offset in [0x070, 0x074] {
            r.write(core, 0x060, 100);
            r.write(core, 0x064, 7);
            r.advance(core, 4);
            r.write(core, offset, 0xdead_beef); // Context restore cancels calculation.
            assert_eq!(r.read(core, 0x078), 3);
            r.advance(core, 32);
            assert_eq!(r.read(core, offset), 0xdead_beef);
        }
        // Completion does not set DIRTY again if QUOTIENT was read while busy.
        r.write(core, 0x060, 10);
        r.write(core, 0x064, 3);
        let _ = r.read(core, 0x070);
        assert_eq!(r.read(core, 0x078), 0);
        r.advance(core, 8);
        assert_eq!(r.read(core, 0x078), 1);
        assert_eq!(result(r, core), (3, 1));

        for (dividend, value, quotient) in [
            (0x060, 42, u32::MAX),
            (0x068, 42, u32::MAX),
            (0x068, (-42i32) as u32, 1),
            (0x068, 0, u32::MAX),
        ] {
            r.write(core, 0x064, 0);
            r.write(core, dividend, value);
            r.advance(core, 8);
            assert_eq!(result(r, core), (quotient, value));
        }
    }
    r.write(0, 0x060, 100);
    r.write(0, 0x064, 7);
    r.write(1, 0x060, 20);
    r.write(1, 0x064, 3);
    r.advance(0, 8);
    assert_eq!(result(r, 0), (14, 2));
    assert_eq!(
        r.read(1, 0x078),
        2,
        "core 0's local cycles do not advance core 1"
    );
    r.advance(1, 8);
    assert_eq!(result(r, 1), (6, 2));

    // Operand/result context save and restore, as required by SDK IRQ use.
    r.write(0, 0x060, 100);
    r.write(0, 0x064, 7);
    r.advance(0, 8);
    let saved = [
        r.read(0, 0x060),
        r.read(0, 0x064),
        r.read(0, 0x074),
        r.read(0, 0x070),
    ];
    r.write(0, 0x060, 999);
    r.write(0, 0x064, 11);
    r.advance(0, 8);
    for (offset, value) in [0x060, 0x064, 0x074, 0x070].into_iter().zip(saved) {
        r.write(0, offset, value);
    }
    assert_eq!(r.read(0, 0x078), 3);
    r.advance(0, 8);
    assert_eq!(result(r, 0), (14, 2));
}

#[test]
fn direct_sio_operand_orders_status_and_context() {
    let mut sio = Sio::new();
    arithmetic(&mut sio);
    status_and_restart(&mut sio);
    sio.reset();
    assert_eq!(sio.read32(0x078, 0), 1);
    assert_eq!(sio.read32(0x078, 1), 1);
}

#[test]
fn serial_bus_operand_orders_status_and_context() {
    let mut bus = Bus::new();
    arithmetic(&mut bus);
    status_and_restart(&mut bus);
}

#[test]
fn serial_cpu_cycles_advance_divider() {
    let mut bus = Bus::new();
    let mut core = CortexM0Plus::new();
    core.regs.set_pc(0x2000_0000);
    for offset in (0..32).step_by(2) {
        bus.write16(0x2000_0000 + offset, 0xbf00); // NOP
    }
    bus.write32(SIO + 0x064, 3);
    bus.write32(SIO + 0x060, 10);
    for _ in 0..7 {
        assert_eq!(core.step(&mut bus), 1);
        assert_eq!(bus.read32(SIO + 0x078), 2);
    }
    assert_eq!(core.step(&mut bus), 1);
    assert_eq!(bus.read32(SIO + 0x078), 3);
    assert_eq!(bus.read32(SIO + 0x070), 3);
}

fn exception_cycles_advance_divider(bus: &mut impl CoreBus) {
    let mut core = CortexM0Plus::new();
    core.regs.set_pc(0x2000_0000);
    core.regs.msp = 0x2000_1000;
    core.regs.r[13] = 0x2000_1000;
    bus.ppb_mut(0).vtor = 0x2000_0100;
    bus.write32(0x2000_0100 + 14 * 4, 0x2000_0201); // PendSV vector
    bus.write32(SIO + 0x064, 3);
    bus.write32(SIO + 0x060, 10);
    bus.ppb_mut(0).icsr |= 1 << 28;
    assert!(core.step(bus) >= 8, "exception entry must consume cycles");
    assert_eq!(bus.read32(SIO + 0x078), 3);
    assert_eq!(bus.read32(SIO + 0x070), 3);
}

#[test]
fn serial_exception_cycles_advance_divider() {
    exception_cycles_advance_divider(&mut Bus::new());
}

#[test]
fn parked_serial_core_finishes_on_system_clock() {
    let mut emu = Emulator::new(Config::default());
    emu.core_mut(0).halt();
    emu.core_mut(1).halt();
    emu.mmio_write32(SIO + 0x064, 3);
    emu.mmio_write32(SIO + 0x060, 10);
    emu.step_until(8).unwrap();
    assert_eq!(emu.mmio_read32(SIO + 0x078), 3);
    assert_eq!(emu.mmio_read32(SIO + 0x070), 3);
}

#[cfg(all(
    feature = "threading",
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
mod threaded {
    use super::*;
    use rp2040_emu::threaded::{SharedState, WorkerBus};
    use std::sync::Arc;
    use std::sync::atomic::Ordering;

    struct Workers([WorkerBus; 2]);
    impl Registers for Workers {
        fn read(&mut self, core: usize, offset: u32) -> u32 {
            self.0[core].read32(SIO + offset)
        }
        fn write(&mut self, core: usize, offset: u32, value: u32) {
            self.0[core].write32(SIO + offset, value);
        }
        fn advance(&mut self, core: usize, cycles: u32) {
            self.0[core].advance_core_cycles(cycles);
        }
    }

    #[test]
    fn worker_bus_operand_orders_status_and_context() {
        let shared = Arc::new(SharedState::new_default());
        let mut workers = Workers([WorkerBus::new(shared.clone(), 0), WorkerBus::new(shared, 1)]);
        arithmetic(&mut workers);
        status_and_restart(&mut workers);
    }

    #[test]
    fn worker_cpu_clock_and_parked_clock_do_not_double_count() {
        let shared = Arc::new(SharedState::new_default());
        let mut bus = WorkerBus::new(shared.clone(), 0);
        let mut core = CortexM0Plus::new();
        core.regs.set_pc(0x2000_0000);
        for offset in (0..32).step_by(2) {
            bus.write16(0x2000_0000 + offset, 0xbf00);
        }
        bus.write32(SIO + 0x064, 3);
        bus.write32(SIO + 0x060, 10);
        for _ in 0..7 {
            assert_eq!(core.step(&mut bus), 1);
        }
        shared.master_cycle.store(7, Ordering::Release);
        assert_eq!(
            bus.read32(SIO + 0x078),
            2,
            "scheduler cycles must not be counted twice"
        );
        assert_eq!(core.step(&mut bus), 1);
        assert_eq!(bus.read32(SIO + 0x078), 3);
        assert_eq!(bus.read32(SIO + 0x070), 3);
        shared.master_cycle.store(100, Ordering::Release);
        bus.write32(SIO + 0x060, 20);
        assert_eq!(bus.read32(SIO + 0x078), 2);
        shared.master_cycle.store(107, Ordering::Release);
        assert_eq!(bus.read32(SIO + 0x078), 2);
        shared.master_cycle.store(108, Ordering::Release);
        assert_eq!(bus.read32(SIO + 0x078), 3);
        assert_eq!(bus.read32(SIO + 0x070), 6);
    }

    #[test]
    fn worker_exception_cycles_advance_divider() {
        let shared = Arc::new(SharedState::new_default());
        exception_cycles_advance_divider(&mut WorkerBus::new(shared, 0));
    }
}
