//! DMA quantum-invariance workloads.
//!
//! `quantum=1` is the reference execution.  Every workload below is run for
//! the same requested number of master cycles at quantum 1, 16, and 64, and
//! the externally observable DMA state is compared.  This file is deliberately
//! an integration test: the test must use the public emulator surface and must
//! not make the product model or its existing tests more permissive.
//!
//! The current RP2040 model arbitrates DMA once at the end of a bulk quantum.
//! Therefore these tests are expected to expose a failure until a generic
//! per-sysclk DMA model is implemented.  A failure is useful evidence: it
//! identifies the workload and the first state that diverges from quantum 1.

use rp2040_emu::{Config, Emulator, EmulatorBuilder};

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
const CTRL_DATA_SIZE_32: u32 = 2 << 2;
const CTRL_INCR_READ: u32 = 1 << 4;
const CTRL_INCR_WRITE: u32 = 1 << 5;
const CTRL_RING_SIZE_SHIFT: u32 = 6;
const CTRL_RING_SEL: u32 = 1 << 10;
const CTRL_CHAIN_TO_SHIFT: u32 = 11;
const CTRL_TREQ_SEL_SHIFT: u32 = 15;
const DREQ_FORCE: u8 = 0x3f;
const DREQ_TIMER0: u8 = 59;

const SRAM_BASE: u32 = 0x2000_0000;
const RUN_CYCLES: u64 = 256;

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
    destination_words: Vec<u32>,
    channels: Vec<ChannelState>,
    intr: u32,
    inte0: u32,
    ints0: u32,
    inte1: u32,
    ints1: u32,
    nvic_dma_irq0_pending: u32,
    nvic_dma_irq1_pending: u32,
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

fn snapshot(emu: &mut Emulator, destination: u32, word_count: u32, channels: &[u32]) -> DmaState {
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
    emu.run(RUN_CYCLES)
        .unwrap_or_else(|error| panic!("DMA workload failed at quantum {quantum}: {error:?}"));
    snapshot(&mut emu, destination, words, channels)
}

fn assert_invariant(name: &str, states: &[(u32, DmaState)]) {
    let reference = &states[0].1;
    for (quantum, actual) in &states[1..] {
        assert_eq!(
            reference, actual,
            "{name}: quantum=1 reference differs from quantum={quantum}"
        );
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
