//! DMA quantum-invariance workloads.
//!
//! `quantum=1` is the reference execution. Every workload below is run to the
//! same actual master-cycle boundary at quantum 1, 16, and 64, and the
//! externally observable DMA state is compared. This file is deliberately
//! an integration test: the test must use the public emulator surface and must
//! not make the product model or its existing tests more permissive.
//!
//! The RP2040 model now uses per-system-clock arbitration for the tested
//! workloads, while retaining an event-driven path for eligible timer-only
//! windows. A failure identifies the workload and the first state that
//! diverges from the quantum-1 reference.

use rp2040_emu::{Config, DmaSchedulerSnapshot, Emulator, EmulatorBuilder};

const DMA_BASE: u32 = 0x5000_0000;
const RESETS_BASE: u32 = 0x4000_c000;
const RESET_DMA: u32 = 2;
const RESET_TIMER: u32 = 23;

const REG_INTR: u32 = 0x400;
const REG_INTE0: u32 = 0x404;
const REG_INTS0: u32 = 0x40c;
const REG_INTE1: u32 = 0x414;
const REG_INTS1: u32 = 0x41c;
const REG_TIMER0: u32 = 0x420;

const CH_READ_ADDR: u32 = 0x00;
const CH_WRITE_ADDR: u32 = 0x04;
const CH_TRANS_COUNT: u32 = 0x08;
const CH_CTRL_TRIG: u32 = 0x0c;
const CH_AL1_CTRL: u32 = 0x10;

const CTRL_EN: u32 = 1 << 0;
const CTRL_HIGH_PRIORITY: u32 = 1 << 1;
const CTRL_DATA_SIZE_32: u32 = 2 << 2;
const CTRL_INCR_READ: u32 = 1 << 4;
const CTRL_INCR_WRITE: u32 = 1 << 5;
const CTRL_RING_SIZE_SHIFT: u32 = 6;
const CTRL_RING_SEL: u32 = 1 << 10;
const CTRL_CHAIN_TO_SHIFT: u32 = 11;
const CTRL_TREQ_SEL_SHIFT: u32 = 15;
const DREQ_FORCE: u8 = 0x3f;
const DREQ_TIMER0: u8 = 59;
const AUDIO_PWM_CC: u32 = 0x4005_0070;

const SRAM_BASE: u32 = 0x2000_0000;
// The public `Emulator::run` contract is "at least N cycles" and may
// overshoot at an instruction boundary. The b . loop below consumes two
// cycles per instruction, so the three tested quanta (1, 16, 64) consume
// 2, 18, and 66 cycles per serial step respectively. 198 is their least
// common multiple; every run therefore stops at the same actual master-cycle
// boundary instead of comparing scheduler state at different times.
const RUN_CYCLES: u64 = 198;

#[derive(Clone, Debug, PartialEq, Eq)]
struct ChannelState {
    read_addr: u32,
    write_addr: u32,
    trans_count: u32,
    trans_count_reload: u32,
    ctrl: u32,
    busy: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct DmaState {
    actual_cycles: u64,
    destination_words: Vec<u32>,
    channels: Vec<ChannelState>,
    intr: u32,
    inte0: u32,
    ints0: u32,
    inte1: u32,
    ints1: u32,
    nvic_dma_irq0_pending: u32,
    nvic_dma_irq1_pending: u32,
    scheduler: DmaSchedulerSnapshot,
}

fn emulator(quantum: u32) -> Emulator {
    let mut emu = EmulatorBuilder::new(Config {
        sys_clk_hz: 1_000_000,
    })
    .step_quantum(quantum)
    .build()
    .expect("serial RP2040 emulator must build");

    // A two-byte Thumb `b .` loop keeps core 0 alive while the public
    // Emulator::run() advances the peripheral and DMA clocks.  The vector
    // table is intentionally tiny and is loaded through the public test API.
    let mut rom = vec![0u8; 0x102];
    rom[0..4].copy_from_slice(&0x2000_4000u32.to_le_bytes());
    rom[4..8].copy_from_slice(&0x0000_0101u32.to_le_bytes());
    rom[0x100..0x102].copy_from_slice(&[0xfe, 0xe7]);
    emu.load_image(0, &rom);
    emu.reset();
    emu
}

fn release_dma(emu: &mut Emulator) {
    emu.bus.write32(
        RESETS_BASE + 0x3000,
        (1u32 << RESET_DMA) | (1u32 << RESET_TIMER),
    );
}

fn ctrl(treq: u8, chain_to: u8, ring_size: u8, ring_on_write: bool) -> u32 {
    let mut value = CTRL_EN | CTRL_DATA_SIZE_32 | CTRL_INCR_READ | CTRL_INCR_WRITE;
    value |= (treq as u32) << CTRL_TREQ_SEL_SHIFT;
    value |= (chain_to as u32) << CTRL_CHAIN_TO_SHIFT;
    value |= (ring_size as u32) << CTRL_RING_SIZE_SHIFT;
    if ring_on_write {
        value |= CTRL_RING_SEL;
    }
    value
}

fn high_priority(control: u32) -> u32 {
    control | CTRL_HIGH_PRIORITY
}

fn program_channel(
    emu: &mut Emulator,
    channel: u32,
    source: u32,
    destination: u32,
    count: u32,
    control: u32,
    trigger: bool,
) {
    let base = DMA_BASE + channel * 0x40;
    emu.bus.write32(base + CH_READ_ADDR, source);
    emu.bus.write32(base + CH_WRITE_ADDR, destination);
    emu.bus.write32(base + CH_TRANS_COUNT, count);
    emu.bus.write32(base + CH_AL1_CTRL, control);
    if trigger {
        emu.bus.write32(base + CH_CTRL_TRIG, control);
    }
}

fn snapshot(
    emu: &mut Emulator,
    actual_cycles: u64,
    destination: u32,
    word_count: u32,
    channels: &[u32],
) -> DmaState {
    let destination_words = (0..word_count)
        .map(|i| emu.bus.read32(destination + i * 4))
        .collect();
    let channels = channels
        .iter()
        .map(|&index| {
            let channel = emu.bus.dma_channel(index as usize);
            ChannelState {
                read_addr: channel.read_addr,
                write_addr: channel.write_addr,
                trans_count: channel.trans_count,
                trans_count_reload: channel.trans_count_reload,
                ctrl: channel.ctrl,
                busy: channel.busy,
            }
        })
        .collect();
    DmaState {
        actual_cycles,
        destination_words,
        channels,
        intr: emu.bus.read32(DMA_BASE + REG_INTR),
        inte0: emu.bus.read32(DMA_BASE + REG_INTE0),
        ints0: emu.bus.read32(DMA_BASE + REG_INTS0),
        inte1: emu.bus.read32(DMA_BASE + REG_INTE1),
        ints1: emu.bus.read32(DMA_BASE + REG_INTS1),
        // DMA IRQ 0/1 are NVIC lines 11/12. Reading ISPR is non-destructive.
        nvic_dma_irq0_pending: emu.bus.read32(0xe000_e200) & (1 << 11),
        nvic_dma_irq1_pending: emu.bus.read32(0xe000_e200) & (1 << 12),
        scheduler: emu.bus.dma_scheduler_snapshot(),
    }
}

fn run_and_snapshot<F>(
    quantum: u32,
    setup: F,
    destination: u32,
    words: u32,
    channels: &[u32],
) -> DmaState
where
    F: FnOnce(&mut Emulator),
{
    let mut emu = emulator(quantum);
    release_dma(&mut emu);
    setup(&mut emu);
    let executed = emu
        .run(RUN_CYCLES)
        .unwrap_or_else(|error| panic!("DMA workload failed at quantum {quantum}: {error:?}"));
    snapshot(&mut emu, executed, destination, words, channels)
}

fn assert_invariant(name: &str, states: &[(u32, DmaState)]) {
    let reference = &states[0].1;
    for (quantum, actual) in &states[1..] {
        assert_eq!(
            reference.actual_cycles, actual.actual_cycles,
            "{name}: quantum=1 and quantum={quantum} stopped at different actual cycles"
        );
        assert_eq!(
            reference.destination_words, actual.destination_words,
            "{name}: destination differs at quantum={quantum}"
        );
        assert_eq!(
            reference.channels, actual.channels,
            "{name}: channel state differs at quantum={quantum}"
        );
        assert_eq!(
            (
                reference.intr,
                reference.inte0,
                reference.ints0,
                reference.inte1,
                reference.ints1,
                reference.nvic_dma_irq0_pending,
                reference.nvic_dma_irq1_pending,
            ),
            (
                actual.intr,
                actual.inte0,
                actual.ints0,
                actual.inte1,
                actual.ints1,
                actual.nvic_dma_irq0_pending,
                actual.nvic_dma_irq1_pending,
            ),
            "{name}: DMA IRQ state differs at quantum={quantum}"
        );
        assert_scheduler_invariant(
            name,
            &reference.scheduler,
            &actual.scheduler,
            *quantum,
            actual.actual_cycles,
        );
    }
}

fn assert_scheduler_invariant(
    name: &str,
    reference: &DmaSchedulerSnapshot,
    actual: &DmaSchedulerSnapshot,
    quantum: u32,
    actual_cycles: u64,
) {
    // These are cumulative or final hardware-visible observations and must
    // be identical when the actual master-cycle boundary is identical.
    assert_eq!(
        reference.timer, actual.timer,
        "{name}: timer registers at quantum={quantum}"
    );
    assert_eq!(
        reference.timer_accum, actual.timer_accum,
        "{name}: timer accumulators at quantum={quantum}"
    );
    assert_eq!(
        reference.timer_event_count, actual.timer_event_count,
        "{name}: timer event counts at quantum={quantum}"
    );
    assert_eq!(
        reference.timer_miss_count, actual.timer_miss_count,
        "{name}: timer miss counts at quantum={quantum}"
    );
    assert_eq!(
        reference.timer_miss_audio_not_busy, actual.timer_miss_audio_not_busy,
        "{name}: audio-not-busy classifications at quantum={quantum}"
    );
    assert_eq!(
        reference.timer_miss_other_dma_selected, actual.timer_miss_other_dma_selected,
        "{name}: arbitration-loss classifications at quantum={quantum}"
    );
    assert_eq!(
        reference.timer_miss_no_dma_selected, actual.timer_miss_no_dma_selected,
        "{name}: no-selection classifications at quantum={quantum}"
    );
    assert_eq!(
        reference.timer_miss_multiple_due_in_window, actual.timer_miss_multiple_due_in_window,
        "{name}: coalesced-event classifications at quantum={quantum}"
    );
    assert_eq!(
        reference.audio_sink, actual.audio_sink,
        "{name}: audio sink observation at quantum={quantum}"
    );

    // `timer_due_cycle`, `last_selected_timer_due_cycle`, and the window
    // counters describe the *last tick window*, whose partition necessarily
    // changes with step quantum. They are still checked for internal
    // consistency rather than incorrectly compared as if they were cumulative
    // state. Audio-selected due cycles are compared above by the sink's digest.
    for (index, snapshot) in [reference, actual].into_iter().enumerate() {
        for timer in 0..4 {
            assert!(
                snapshot.timer_window_events[timer] <= snapshot.timer_event_count[timer],
                "{name}: invalid timer window event count for {timer} in snapshot {index}"
            );
            assert!(
                snapshot.timer_window_misses[timer] <= snapshot.timer_miss_count[timer],
                "{name}: invalid timer window miss count for {timer} in snapshot {index}"
            );
            if snapshot.timer_window_events[timer] == 0 {
                assert_eq!(
                    snapshot.timer_due_cycle[timer], 0,
                    "{name}: due cycle without a window event for timer {timer} in snapshot {index}"
                );
            } else {
                assert!(snapshot.timer_due_cycle[timer] > 0);
            }
        }
        if let Some(cycle) = snapshot.last_selected_timer_due_cycle {
            assert!(cycle <= actual_cycles);
        }
    }
}

#[test]
fn dma_force_transfer_is_quantum_invariant() {
    let setup = |emu: &mut Emulator| {
        for i in 0..2u32 {
            emu.bus.write32(SRAM_BASE + 0x100 + i * 4, 0xa000_0000 | i);
        }
        program_channel(
            emu,
            0,
            SRAM_BASE + 0x100,
            SRAM_BASE + 0x200,
            2,
            ctrl(DREQ_FORCE, 0, 0, false),
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0x200, 2, &[0]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("FORCE transfer", &states);
}

#[test]
fn dma_timer_paced_transfer_is_quantum_invariant() {
    let setup = |emu: &mut Emulator| {
        for i in 0..16u32 {
            emu.bus.write32(SRAM_BASE + 0x300 + i * 4, 0xb000_0000 | i);
        }
        // One timer event every eight master cycles. The timer workload is
        // deliberately long enough for q16/q64 bulk arbitration to diverge.
        emu.bus.write32(DMA_BASE + REG_TIMER0, (1 << 16) | 8);
        program_channel(
            emu,
            0,
            SRAM_BASE + 0x300,
            SRAM_BASE + 0x400,
            16,
            ctrl(DREQ_TIMER0, 0, 0, false),
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0x400, 16, &[0]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("timer-paced transfer", &states);
}

#[test]
fn dma_two_channel_force_competition_is_quantum_invariant() {
    let setup = |emu: &mut Emulator| {
        for i in 0..4u32 {
            emu.bus.write32(SRAM_BASE + 0x500 + i * 4, 0xc000_0000 | i);
            emu.bus.write32(SRAM_BASE + 0x600 + i * 4, 0xc100_0000 | i);
        }
        program_channel(
            emu,
            0,
            SRAM_BASE + 0x500,
            SRAM_BASE + 0x700,
            4,
            ctrl(DREQ_FORCE, 0, 0, false),
            true,
        );
        program_channel(
            emu,
            1,
            SRAM_BASE + 0x600,
            SRAM_BASE + 0x800,
            4,
            ctrl(DREQ_FORCE, 1, 0, false),
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0x700, 8, &[0, 1]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("two-channel FORCE competition", &states);
}

#[test]
fn dma_chain_read_ring_is_quantum_invariant() {
    let setup = |emu: &mut Emulator| {
        for i in 0..4u32 {
            emu.bus.write32(SRAM_BASE + 0x900 + i * 4, 0xd000_0000 | i);
        }
        let chain_source = SRAM_BASE + 0x900;
        let chain_destination = SRAM_BASE + 0xa00;
        let ring_read_destination = chain_destination + 4;
        let chain_ctrl = ctrl(DREQ_FORCE, 1, 0, false);
        let ring_read_ctrl = ctrl(DREQ_FORCE, 1, 4, false);

        // Channel 1 is pre-programmed but not armed. Completing channel 0
        // must arm it; its read address is a 16-byte ring.  The destination
        // is adjacent to channel 0's destination so one snapshot covers both
        // the chain completion and the read-ring data.
        program_channel(
            emu,
            1,
            chain_source,
            ring_read_destination,
            4,
            ring_read_ctrl,
            false,
        );
        program_channel(emu, 0, chain_source, chain_destination, 1, chain_ctrl, true);
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0xa00, 5, &[0, 1]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("chain/read-ring", &states);
}

#[test]
fn dma_audio_timer_paced_observation_is_quantum_invariant() {
    let setup = |emu: &mut Emulator| {
        // Valid stereo PWM8 duty words.  The destination is the PicoCalc
        // PWM5 CC register so the public audio sink observes every DMA write.
        for (i, value) in [0x0080_0080, 0x00c0_0040, 0x0040_00c0, 0x0088_0078]
            .into_iter()
            .enumerate()
        {
            emu.bus.write32(SRAM_BASE + 0xb00 + i as u32 * 4, value);
        }
        // One timer event every eight master cycles; four writes form one
        // complete observed block and exercise due-cycle/PCM/block digests.
        emu.bus.write32(DMA_BASE + REG_TIMER0, (1 << 16) | 8);
        program_channel(
            emu,
            0,
            SRAM_BASE + 0xb00,
            AUDIO_PWM_CC,
            4,
            // Audio samples are written to one fixed PWM CC register. The
            // normal SRAM-to-SRAM helper increments the destination, but a
            // timer-paced PWM sink must keep the write address fixed.
            ctrl(DREQ_TIMER0, 0, 0, false) & !CTRL_INCR_WRITE,
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, AUDIO_PWM_CC, 1, &[0]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("audio timer-paced observation", &states);
    for (quantum, state) in &states {
        assert_eq!(
            state.scheduler.audio_sink.dma_write_count, 4,
            "quantum={quantum}"
        );
        assert_eq!(
            state.scheduler.audio_sink.pcm_sha256.len(),
            64,
            "quantum={quantum}"
        );
        assert_eq!(
            state.scheduler.audio_sink.block_start_count, 1,
            "quantum={quantum}"
        );
    }
}

#[test]
fn dma_high_priority_force_beats_normal_force() {
    let setup = |emu: &mut Emulator| {
        emu.bus.write32(SRAM_BASE + 0xc00, 0xe000_0000);
        emu.bus.write32(SRAM_BASE + 0xc04, 0xe100_0000);
        let destination = SRAM_BASE + 0xd00;
        // Both channels write the same fixed destination. The high-priority
        // channel must issue first; the normal channel then overwrites it.
        program_channel(
            emu,
            0,
            SRAM_BASE + 0xc00,
            destination,
            1,
            high_priority(ctrl(DREQ_FORCE, 0, 0, false) & !CTRL_INCR_WRITE),
            true,
        );
        program_channel(
            emu,
            1,
            SRAM_BASE + 0xc04,
            destination,
            1,
            ctrl(DREQ_FORCE, 1, 0, false) & !CTRL_INCR_WRITE,
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0xd00, 1, &[0, 1]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("HIGH_PRIORITY versus FORCE", &states);
    for (quantum, state) in &states {
        assert_eq!(
            state.destination_words,
            vec![0xe100_0000],
            "quantum={quantum}"
        );
        assert_eq!(state.channels[0].trans_count, 0, "quantum={quantum}");
        assert_eq!(state.channels[1].trans_count, 0, "quantum={quantum}");
    }
}

#[test]
fn dma_high_priority_timer_beats_normal_force_at_due_event() {
    let setup = |emu: &mut Emulator| {
        emu.bus.write32(SRAM_BASE + 0xc20, 0xe200_0000);
        emu.bus.write32(SRAM_BASE + 0xc24, 0xe300_0000);
        emu.bus.write32(DMA_BASE + REG_TIMER0, (1 << 16) | 8);
        let destination = SRAM_BASE + 0xd20;
        program_channel(
            emu,
            0,
            SRAM_BASE + 0xc20,
            destination,
            1,
            high_priority(ctrl(DREQ_TIMER0, 0, 0, false) & !CTRL_INCR_WRITE),
            true,
        );
        // Keep a normal FORCE channel ready beyond the timer event. At the
        // first timer pulse the high-priority timer must win arbitration.
        program_channel(
            emu,
            1,
            SRAM_BASE + 0xc24,
            destination,
            198,
            ctrl(DREQ_FORCE, 1, 0, false) & !CTRL_INCR_WRITE,
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0xd20, 1, &[0, 1]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("HIGH_PRIORITY timer versus FORCE", &states);
    for (quantum, state) in &states {
        assert_eq!(state.channels[0].trans_count, 0, "quantum={quantum}");
        assert_eq!(state.channels[1].trans_count, 1, "quantum={quantum}");
        assert_eq!(
            state.scheduler.timer_event_count[0], 24,
            "quantum={quantum}"
        );
        assert_eq!(state.scheduler.timer_miss_count[0], 23, "quantum={quantum}");
    }
}

#[test]
fn dma_same_cycle_timer_tie_uses_lowest_channel() {
    let setup = |emu: &mut Emulator| {
        emu.bus.write32(SRAM_BASE + 0xc40, 0xe400_0000);
        emu.bus.write32(SRAM_BASE + 0xc44, 0xe500_0000);
        emu.bus.write32(DMA_BASE + REG_TIMER0, (1 << 16) | 8);
        emu.bus.write32(DMA_BASE + (REG_TIMER0 + 4), (1 << 16) | 8);
        let destination = SRAM_BASE + 0xd40;
        program_channel(
            emu,
            0,
            SRAM_BASE + 0xc40,
            destination,
            4,
            ctrl(DREQ_TIMER0, 0, 0, false) & !CTRL_INCR_READ,
            true,
        );
        program_channel(
            emu,
            1,
            SRAM_BASE + 0xc44,
            destination,
            4,
            ctrl(DREQ_TIMER0 + 1, 1, 0, false) & !CTRL_INCR_READ,
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0xd40, 4, &[0, 1]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("same-cycle timer tie", &states);
    for (quantum, state) in &states {
        assert_eq!(
            state.destination_words,
            vec![0xe500_0000; 4],
            "quantum={quantum}"
        );
        assert_eq!(state.channels[0].trans_count, 0, "quantum={quantum}");
        assert_eq!(state.channels[1].trans_count, 0, "quantum={quantum}");
        assert_eq!(
            state.scheduler.timer_event_count[0], 24,
            "quantum={quantum}"
        );
        assert_eq!(
            state.scheduler.timer_event_count[1], 24,
            "quantum={quantum}"
        );
        assert_eq!(state.scheduler.timer_miss_count[0], 20, "quantum={quantum}");
        assert_eq!(state.scheduler.timer_miss_count[1], 20, "quantum={quantum}");
    }
}

#[test]
fn dma_audio_timer_competes_with_normal_force() {
    let setup = |emu: &mut Emulator| {
        for (i, value) in [0x0080_0080, 0x00c0_0040, 0x0040_00c0, 0x0088_0078]
            .into_iter()
            .enumerate()
        {
            emu.bus.write32(SRAM_BASE + 0xc60 + i as u32 * 4, value);
        }
        emu.bus.write32(SRAM_BASE + 0xca0, 0xe600_0000);
        emu.bus.write32(DMA_BASE + REG_TIMER0, (1 << 16) | 8);
        program_channel(
            emu,
            0,
            SRAM_BASE + 0xc60,
            AUDIO_PWM_CC,
            4,
            ctrl(DREQ_TIMER0, 0, 0, false) & !CTRL_INCR_WRITE,
            true,
        );
        // The lower-numbered audio channel wins on timer cycles; this FORCE
        // channel remains ready on all other cycles and records the
        // competition without touching the PWM sink.
        program_channel(
            emu,
            1,
            SRAM_BASE + 0xca0,
            SRAM_BASE + 0xcb0,
            198,
            ctrl(DREQ_FORCE, 1, 0, false),
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, AUDIO_PWM_CC, 1, &[0, 1]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("audio timer versus FORCE", &states);
    for (quantum, state) in &states {
        let audio = &state.scheduler.audio_sink;
        assert_eq!(audio.dma_write_count, 4, "quantum={quantum}");
        assert_eq!(audio.timer_event_count, 24, "quantum={quantum}");
        assert_eq!(audio.timer_miss_count, 20, "quantum={quantum}");
        assert_eq!(audio.timer_miss_audio_not_busy, 20, "quantum={quantum}");
        assert_eq!(audio.timer_miss_other_dma_selected, 0, "quantum={quantum}");
        assert_eq!(state.channels[1].trans_count, 4, "quantum={quantum}");
    }
}

#[test]
fn dma_chain_changes_ready_priority_tier() {
    let setup = |emu: &mut Emulator| {
        emu.bus.write32(SRAM_BASE + 0xcc0, 0xe700_0000);
        emu.bus.write32(SRAM_BASE + 0xcc4, 0xe800_0000);
        emu.bus.write32(SRAM_BASE + 0xcc8, 0xe900_0000);
        let destination = SRAM_BASE + 0xdc0;
        let fixed = |control| control & !CTRL_INCR_WRITE;

        // Channel 2 is armed by channel 0's completion. It is high priority,
        // so it must displace the long-running normal FORCE channel 1 as soon
        // as the chain makes it ready.
        program_channel(
            emu,
            2,
            SRAM_BASE + 0xcc8,
            destination,
            1,
            fixed(high_priority(ctrl(DREQ_FORCE, 2, 0, false))),
            false,
        );
        program_channel(
            emu,
            1,
            SRAM_BASE + 0xcc4,
            destination,
            198,
            fixed(ctrl(DREQ_FORCE, 1, 0, false)),
            true,
        );
        program_channel(
            emu,
            0,
            SRAM_BASE + 0xcc0,
            destination,
            1,
            fixed(high_priority(ctrl(DREQ_FORCE, 2, 0, false))),
            true,
        );
    };
    let states = [1, 16, 64]
        .into_iter()
        .map(|quantum| {
            (
                quantum,
                run_and_snapshot(quantum, setup, SRAM_BASE + 0xdc0, 1, &[0, 1, 2]),
            )
        })
        .collect::<Vec<_>>();
    assert_invariant("chain priority tier", &states);
    for (quantum, state) in &states {
        assert_eq!(state.channels[0].trans_count, 0, "quantum={quantum}");
        assert_eq!(state.channels[2].trans_count, 0, "quantum={quantum}");
        assert_eq!(state.channels[1].trans_count, 2, "quantum={quantum}");
    }
}
