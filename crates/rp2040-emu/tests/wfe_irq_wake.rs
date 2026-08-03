//! WFE + interrupt wake regression tests (ARMv6-M ARM §B1.5.18).
//!
//! ARMv6-M lists, among the WFE wake-up events, "an asynchronous
//! exception at a priority that preempts any currently active
//! exceptions" — i.e. an enabled+pending IRQ must un-park a core that
//! is waiting on WFE, in addition to the event-register / SEV path.
//!
//! Before the fix, `Emulator::wake_checks` only un-parked on
//! `event_flag`, which dead-locked the Pico SDK `sleep_until` idiom:
//!
//! ```text
//!   timer_hw->alarm[n] = target;  // arm alarm, INTE[n] set
//!   while (!done) __wfe();        // ISR sets `done`
//! ```
//!
//! Once the alarm matched, the IRQ latched in the NVIC but (a) never
//! woke the WFE-parked core and (b) disqualified the tech_debt §1649
//! both-cores-blocked clock-advance branch in `step_serial` (which
//! requires no pending IRQ), so the master clock froze permanently.
//! Observed on `picocalc_helloworld` at cycles=1529101, pc=0x1000198e.

use rp2040_emu::{Config, EmulatorBuilder};

const TIMER_BASE: u32 = 0x4005_4000;
const TIMER_ALARM0: u32 = TIMER_BASE + 0x10;
const TIMER_INTE: u32 = TIMER_BASE + 0x38;
const NVIC_ISER: u32 = 0xE000_E100;
const RESETS_RESET: u32 = 0x4000_C000;

const VTOR: u32 = 0x2000_0000;
/// Vector slot for external IRQ 0 (TIMER_IRQ_0) = exception #16.
const IRQ0_VECTOR: u32 = VTOR + 16 * 4;
const CODE: u32 = 0x2000_1000;
const HANDLER: u32 = 0x2000_2000;
const STACK_TOP: u32 = 0x2003_0000;

/// Build an emulator with core 0 parked on a WFE that follows a
/// `sleep_until`-shaped setup: TIMER alarm 0 armed for a future
/// deadline, TIMER INTE[0] set, NVIC IRQ 0 enabled, core 1 halted.
///
/// Layout: `CODE` holds `WFE; B .`, `HANDLER` holds `B .`.
fn parked_on_wfe_with_pending_alarm(alarm_us: u32) -> rp2040_emu::Emulator {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(64)
        .build()
        .expect("Serial build is infallible");

    // Vector table + code + handler.
    emu.bus.ppb[0].vtor = VTOR;
    emu.bus.write32(IRQ0_VECTOR, HANDLER | 1);
    emu.bus.write16(CODE, 0xBF20); // WFE
    emu.bus.write16(CODE + 2, 0xE7FE); // B .   (instruction after WFE)
    emu.bus.write16(HANDLER, 0xE7FE); // B .   (handler body)

    emu.cores[0].regs.set_pc(CODE);
    emu.cores[0].regs.msp = STACK_TOP;
    emu.cores[0].regs.r[13] = STACK_TOP;
    emu.cores[0].regs.xpsr = 1 << 24; // T bit
    assert!(emu.cores[1].is_halted(), "core 1 halted at boot");

    // Release every peripheral from reset — TIMER is reset-gated at
    // boot and the Bus drops writes to held-in-reset blocks.
    emu.bus.set_active_core(0);
    emu.bus.write32(RESETS_RESET, 0);

    // Arm the alarm and enable delivery, as the SDK's
    // `hardware_alarm_set_target` + `irq_set_enabled` pair does.
    emu.bus.write32(TIMER_INTE, 1);
    emu.bus.write32(TIMER_ALARM0, alarm_us);
    emu.bus.write32(NVIC_ISER, 1);

    emu
}

/// The alarm must actually be scheduled in the future — otherwise the
/// test would be vacuous (it would pass for the wrong reason).
#[test]
fn alarm_is_scheduled_in_the_future() {
    let emu = parked_on_wfe_with_pending_alarm(1000);
    let deadline = emu
        .bus
        .next_scheduled_lazy_deadline()
        .expect("alarm 0 armed with INTE set must produce a lazy deadline");
    assert!(
        deadline > emu.clock.cycles,
        "deadline {deadline} must be in the future (now {})",
        emu.clock.cycles
    );
}

/// Core executes WFE with no event latched → parks.
#[test]
fn wfe_parks_the_core() {
    let mut emu = parked_on_wfe_with_pending_alarm(1000);
    emu.step().expect("serial step");
    assert!(
        emu.bus.wfe_waiting[0],
        "core 0 must be parked after executing WFE"
    );
    assert!(
        !emu.bus.event_flag[0],
        "no event was signalled, so the event register must be clear"
    );
}

/// The regression: run() must keep advancing the master clock past the
/// WFE park, let the TIMER alarm fire, wake the core on the pending
/// IRQ, and land in the IRQ 0 handler.
///
/// Pre-fix this hangs at a frozen master clock: every `step()` returns
/// 0 with core 0 `wfe_waiting` and core 1 halted.
#[test]
fn timer_irq_wakes_wfe_parked_core_and_reaches_handler() {
    let mut emu = parked_on_wfe_with_pending_alarm(1000);
    let deadline = emu.bus.next_scheduled_lazy_deadline().unwrap();

    // Park first so the stall shape matches the firmware repro.
    emu.step().expect("serial step");
    assert!(emu.bus.wfe_waiting[0], "precondition: core 0 parked on WFE");

    // Budget generously past the deadline; the point is that the clock
    // must not freeze, not how fast it gets there.
    let budget = deadline * 2 + 10_000;
    let mut guard = 0u32;
    while emu.clock.cycles < budget {
        let consumed = emu.step().expect("serial step");
        if consumed == 0 {
            panic!(
                "master clock frozen at cycles={} pc={:#010x} \
                 (wfe_waiting={:?}, core1 halted={}) — WFE never woke on the \
                 pending TIMER IRQ",
                emu.clock.cycles,
                emu.cores[0].regs.pc(),
                emu.bus.wfe_waiting,
                emu.cores[1].is_halted()
            );
        }
        if !emu.bus.wfe_waiting[0] && emu.cores[0].regs.pc() == HANDLER {
            break;
        }
        guard += 1;
        assert!(guard < 1_000_000, "runaway loop without reaching handler");
    }

    assert!(
        !emu.bus.wfe_waiting[0],
        "core 0 must have un-parked once the TIMER IRQ went pending+enabled"
    );
    assert!(
        emu.clock.cycles > deadline,
        "master clock must have advanced past the alarm deadline {deadline}, \
         got {}",
        emu.clock.cycles
    );
    assert_eq!(
        emu.cores[0].regs.pc(),
        HANDLER,
        "core 0 must have taken TIMER_IRQ_0 and be executing its handler"
    );
    assert_eq!(
        emu.cores[0].regs.xpsr & 0x1FF,
        16,
        "IPSR must report exception #16 (external IRQ 0)"
    );
}

/// An interrupt-driven WFE wake must NOT consume the event register —
/// only a WFE that finds the latch set clears it (ARMv6-M ARM
/// §B1.5.18). A latched event has to survive to the next WFE.
#[test]
fn irq_wake_does_not_consume_the_event_register() {
    let mut emu = parked_on_wfe_with_pending_alarm(1000);
    emu.step().expect("serial step");
    assert!(emu.bus.wfe_waiting[0], "precondition: core 0 parked on WFE");

    // Latch the pending IRQ directly (NVIC pend), leaving event_flag clear,
    // then let the quantum-end wake check run.
    emu.bus.nvics[0].set_pending(0);
    emu.step().expect("serial step");

    assert!(!emu.bus.wfe_waiting[0], "pending+enabled IRQ must un-park");
    assert!(
        !emu.bus.event_flag[0],
        "the IRQ wake must not have set the event register"
    );
}

/// A pending-but-DISABLED IRQ is not a WFE wake-up event: it cannot
/// preempt, so the core stays parked. Guards against the naive
/// "any pending bit wakes" over-relaxation.
#[test]
fn disabled_irq_does_not_wake_wfe_parked_core() {
    let mut emu = parked_on_wfe_with_pending_alarm(1000);
    // Disable IRQ 0 again (NVIC_ICER at +0x80).
    emu.bus.set_active_core(0);
    emu.bus.write32(0xE000_E180, 1);
    emu.bus.write32(TIMER_INTE, 0);

    emu.step().expect("serial step");
    assert!(emu.bus.wfe_waiting[0], "precondition: core 0 parked on WFE");

    emu.bus.nvics[0].set_pending(0);
    assert_eq!(
        emu.bus.nvics[0].pending_and_enabled(),
        0,
        "IRQ 0 is pending but disabled"
    );
    emu.step().expect("serial step");
    assert!(
        emu.bus.wfe_waiting[0],
        "a disabled pending IRQ must not wake a WFE-parked core"
    );
}
