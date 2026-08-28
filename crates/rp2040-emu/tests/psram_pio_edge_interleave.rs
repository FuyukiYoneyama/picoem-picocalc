//! Regression test for Gate 3's PSRAM PIO-integration bug (Sol's
//! diagnosis, 2026-08-04): `Emulator::step_serial`'s slow-path branch
//! called `tick_pio_and_route_irqs(consumed)` + one `update_gpio()` in
//! bulk, taking a single `bus.gpio_in` snapshot for the whole quantum.
//! With `step_quantum > 1`, every SCK/CS edge inside a quantum after
//! the first was invisible to `Psram::tick` until the quantum boundary
//! — a PIO-driven SPI write frame would open/close CS at a plausible
//! cadence (`cs_falling_count` looked healthy) while the reconstructed
//! command byte was essentially noise (99.85% `unknown` in the real
//! `picocalc_helloworld` run), because most bit-level SCK transitions
//! never reached the model.
//!
//! This test hand-assembles the *real* `rp2040-psram` `spi_psram_fudge`
//! PIO program (the variant `picocalc_helloworld`'s `main.c` actually
//! selects — see `crates/picocalc-board/src/pins.rs` doc comment) into
//! PIO0 SM0, drives a single 8-bit PSRAM `WRITE` frame (cmd `0x02` +
//! 3-byte address + 1 data byte, exactly `psram_write8`'s wire format)
//! through it with `step_quantum` deliberately > 1 (so the slow path's
//! bulk-vs-per-cycle distinction actually matters), and asserts the
//! byte lands in the attached `Psram` model's buffer at the right
//! address.
//!
//! Confirmed manually against the pre-fix `step_serial` (bulk
//! `tick_pio_and_route_irqs(consumed)` + one `update_gpio()` on every
//! slow-path call, unconditionally): this test FAILED —
//! `psram.bytes_written` stayed 0 and `buffer[0x10]` stayed 0x00,
//! matching the real-firmware symptom. Restoring the per-cycle
//! interleave (gated on `Bus::has_pin_watching_device`) makes it PASS.
//!
//! `PIO_BASE + 0x000`-style literals mirror `src/pio_tests.rs`'s
//! register-offset idiom (see that file's "Clippy: identity_op" note).

#![allow(clippy::identity_op)]

use picoem_devices::Psram;
use rp2040_emu::bus::PIO0_BASE;
use rp2040_emu::{Config, Emulator, EmulatorBuilder};

// ---------------------------------------------------------------------------
// Hand-assembled `spi_psram_fudge` (rp2040-psram's `psram_spi.pio`) — the
// program `psram_spi_init_clkdiv(pio1, -1, 1.0f, /*fudge=*/true)` selects.
// `.side_set 2` (not `opt`): bit0 of the side value is CS, bit1 is SCK.
// ---------------------------------------------------------------------------

/// Encode one 16-bit PIO instruction. Fixed to this test's program
/// configuration: `SIDESET_COUNT=2`, `SIDE_EN=0` (no "opt"), so the
/// 5-bit delay/side-set field splits as `[side(2) | delay(3)]` — see
/// `picoem_common::pio::decode::decode`.
fn insn(opcode: u16, operand: u8, side: u8, delay: u8) -> u16 {
    let delay_sideset = (((side & 0x3) << 3) | (delay & 0x7)) as u16;
    (opcode << 13) | (delay_sideset << 8) | (operand as u16)
}

const OP_JMP: u16 = 0b000;
const OP_IN: u16 = 0b010;
const OP_OUT: u16 = 0b011;
const OP_MOV: u16 = 0b101;

fn jmp_operand(condition: u8, address: u8) -> u8 {
    (condition << 5) | (address & 0x1F)
}
fn out_operand(destination: u8, bit_count: u8) -> u8 {
    (destination << 5) | (bit_count & 0x1F)
}
fn in_operand(source: u8, bit_count: u8) -> u8 {
    (source << 5) | (bit_count & 0x1F)
}
fn mov_operand(destination: u8, op: u8, source: u8) -> u8 {
    (destination << 5) | ((op & 0x3) << 3) | source
}

// JMP conditions.
const COND_ALWAYS: u8 = 0;
const COND_NOT_Y: u8 = 3;
const COND_X_DEC: u8 = 2;
const COND_Y_DEC: u8 = 4;

// OUT/MOV destinations, MOV source.
const DEST_PINS: u8 = 0;
const DEST_X: u8 = 1;
const DEST_Y: u8 = 2;

/// Assemble `spi_psram_fudge` (10 instructions, addresses 0..9):
///
/// ```text
/// begin:                                          ; addr
///     out x, 8            side 0b01               ; 0
///     out y, 8            side 0b01               ; 1
///     jmp x--, writeloop  side 0b01                ; 2
/// writeloop:
///     out pins, 1         side 0b00               ; 3
///     jmp x--, writeloop  side 0b10                ; 4
///     jmp !y, begin       side 0b00               ; 5
///     nop                 side 0b10               ; 6  (mov y, y)
///     jmp readloop_mid    side 0b00               ; 7
/// readloop:
///     in pins, 1          side 0b00               ; 8
/// readloop_mid:
///     jmp y--, readloop   side 0b10                ; 9
/// ```
fn assemble_spi_psram_fudge() -> [u16; 10] {
    [
        insn(OP_OUT, out_operand(DEST_X, 8), 0b01, 0), // 0: out x, 8
        insn(OP_OUT, out_operand(DEST_Y, 8), 0b01, 0), // 1: out y, 8
        insn(OP_JMP, jmp_operand(COND_X_DEC, 3), 0b01, 0), // 2: jmp x--, 3
        insn(OP_OUT, out_operand(DEST_PINS, 1), 0b00, 0), // 3: out pins, 1
        insn(OP_JMP, jmp_operand(COND_X_DEC, 3), 0b10, 0), // 4: jmp x--, 3
        insn(OP_JMP, jmp_operand(COND_NOT_Y, 0), 0b00, 0), // 5: jmp !y, 0
        insn(OP_MOV, mov_operand(DEST_Y, 0, DEST_Y), 0b10, 0), // 6: nop (mov y, y)
        insn(OP_JMP, jmp_operand(COND_ALWAYS, 9), 0b00, 0), // 7: jmp 9
        insn(OP_IN, in_operand(DEST_PINS, 1), 0b00, 0), // 8: in pins, 1
        insn(OP_JMP, jmp_operand(COND_Y_DEC, 8), 0b10, 0), // 9: jmp y--, 8
    ]
}

// ---------------------------------------------------------------------------
// Pin assignment for this test — matches `Psram::picogus()`
// (MISO=0, CS=1, SCK=2, MOSI=3). Nothing PicoCalc-specific about the
// actual pin numbers here; only the PIO program matters for this
// regression (the real PicoCalc pins are covered by the harness-level
// Gate 3 checks in `crates/picocalc-harness`).
// ---------------------------------------------------------------------------
const PIN_MISO: u8 = 0;
const PIN_CS: u8 = 1;
const PIN_SCK: u8 = 2;
const PIN_MOSI: u8 = 3;

/// Force-execute a `SET PINDIRS` on SM0, temporarily repointing
/// `PINCTRL`'s `SET_BASE`/`SET_COUNT` to the given pin range. Mirrors
/// what `pico_sdk`'s `pio_sm_set_consecutive_pindirs` does on real
/// hardware (and `bus_pio_instr_mem_write_is_observable_via_force_exec`
/// / `blinky_emulator` in `src/pio_tests.rs`).
fn force_set_pindirs(
    emu: &mut Emulator,
    base: u32,
    pio_base: u32,
    pin_base: u8,
    count: u8,
    value: u8,
) {
    let pinctrl = ((count as u32) << 26) | ((pin_base as u32) << 5);
    emu.bus.write32(pio_base + 0x0DC, pinctrl);
    const SET_PINDIRS_DEST: u8 = 4;
    let set_pindirs = insn(0b111, (SET_PINDIRS_DEST << 5) | (value & 0x1F), 0, 0);
    emu.bus.write32(pio_base + 0x0D8, set_pindirs as u32);
    let _ = base; // silence unused in case this fn is reused elsewhere
}

/// Park core 0 on a long run of 1-cycle NOPs so `emu.step()` advances
/// the master clock by exactly `step_quantum` cycles per call,
/// deterministically — matching `park_core0_on_nops` in
/// `src/pio_tests.rs` (kept separate here since integration tests
/// can't reach that private helper).
fn park_core0_on_nops(emu: &mut Emulator) {
    let prog = 0x2000_0000u32;
    for i in 0..4096u32 {
        emu.bus.write16(prog + i * 2, 0xBF00); // NOP
    }
    emu.cores[0].regs.set_pc(prog);
    emu.cores[0].regs.msp = 0x2003_0000;
    emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    emu.cores[0].regs.xpsr = 1 << 24; // Thumb bit
}

/// Build the emulator, load `spi_psram_fudge` into PIO0 SM0 wired to
/// [`PIN_CS`]/[`PIN_SCK`]/[`PIN_MOSI`]/[`PIN_MISO`], attach a
/// [`Psram::picogus`]-pinned PSRAM (no Fast Read output delay), and
/// enable the SM. `step_quantum` is caller-chosen so the same setup
/// can probe both a bulk-unsafe quantum (>1) and the quantum=1
/// always-safe baseline.
fn build_psram_pio_emulator(step_quantum: u32) -> Emulator {
    build_psram_pio_emulator_with(
        Psram::new(PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI),
        step_quantum,
    )
}

/// Same as [`build_psram_pio_emulator`] but takes a caller-configured
/// [`Psram`] instance — used by the Fast Read round-trip regression
/// below, which needs `with_read_output_delay(1)` (Sol's approved fix
/// for the `spi_psram_fudge` read-path bug).
fn build_psram_pio_emulator_with(psram: Psram, step_quantum: u32) -> Emulator {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(step_quantum)
        .psram(psram)
        .build()
        .expect("Serial build is infallible");

    park_core0_on_nops(&mut emu);

    // Load the 10-instruction program into PIO0 SM0's instruction memory.
    for (i, word) in assemble_spi_psram_fudge().iter().enumerate() {
        emu.bus
            .write32(PIO0_BASE + 0x048 + (i as u32) * 4, *word as u32);
    }

    // Pindirs: CS+SCK (base=PIN_CS, count=2) and MOSI (base=PIN_MOSI,
    // count=1) as outputs; MISO (base=PIN_MISO, count=1) as input.
    // Mirrors `pio_spi_psram_cs_init`'s three
    // `pio_sm_set_consecutive_pindirs` calls.
    force_set_pindirs(&mut emu, PIO0_BASE, PIO0_BASE, PIN_CS, 2, 0b11);
    force_set_pindirs(&mut emu, PIO0_BASE, PIO0_BASE, PIN_MOSI, 1, 1);
    force_set_pindirs(&mut emu, PIO0_BASE, PIO0_BASE, PIN_MISO, 1, 0);

    // Final PINCTRL: OUT_BASE=MOSI, OUT_COUNT=1, SIDESET_BASE=CS,
    // SIDESET_COUNT=2, IN_BASE=MISO. Matches
    // `sm_config_set_out_pins(&c, pin_mosi, 1)`,
    // `sm_config_set_in_pins(&c, pin_miso)`,
    // `sm_config_set_sideset_pins(&c, pin_cs)` (2 consecutive pins).
    let pinctrl = (PIN_MOSI as u32) // OUT_BASE [4:0]
        | ((PIN_CS as u32) << 10) // SIDESET_BASE [14:10]
        | ((PIN_MISO as u32) << 15) // IN_BASE [19:15]
        | (1u32 << 20) // OUT_COUNT [25:20] = 1
        | (2u32 << 29); // SIDESET_COUNT [31:29] = 2
    emu.bus.write32(PIO0_BASE + 0x0DC, pinctrl);

    // EXECCTRL: wrap_bottom=0 ("begin"), wrap_top=9 ("readloop_mid").
    // SIDE_EN=0, SIDE_PINDIR=0 (value-drive, not optional) — defaults.
    let execctrl = (9u32 << 12) | (0u32 << 7);
    emu.bus.write32(PIO0_BASE + 0x0CC, execctrl);

    // SHIFTCTRL: autopull+autopush at an 8-bit threshold, MSB-first
    // (shift left) both ways. Matches
    // `sm_config_set_out_shift(&c, false, true, 8)` and
    // `sm_config_set_in_shift(&c, false, true, 8)`.
    let shiftctrl = (8u32 << 25) // PULL_THRESH
        | (8u32 << 20) // PUSH_THRESH
        | (1u32 << 17) // AUTOPULL
        | (1u32 << 16); // AUTOPUSH
    emu.bus.write32(PIO0_BASE + 0x0D0, shiftctrl);

    // Enable SM0.
    emu.bus.write32(PIO0_BASE + 0x000, 0x1);

    emu
}

/// Push one byte into PIO0 SM0's TX FIFO the way DMA does it — an
/// 8-bit-wide write to TXF0, replicated across all four lanes by the
/// PIO TXF narrow-write fix (see `bus/mod.rs` "PIO TXF narrow-write
/// dispatch" tests).
fn push_tx_byte(emu: &mut Emulator, byte: u8) {
    emu.bus.write8(PIO0_BASE + 0x010, byte);
}

/// Drive `emu.step()` `n` times (each advances the master clock by
/// exactly `step_quantum` cycles, per [`park_core0_on_nops`]).
fn run_steps(emu: &mut Emulator, n: u32) {
    for _ in 0..n {
        emu.step().expect("Serial step is infallible");
    }
}

/// Drive one `psram_write8(psram_spi, 0x10, 0xAB)`-equivalent frame:
/// 40 bits to write (cmd `0x02` + 3-byte BE address + 1 data byte), 0
/// bits to read — `rp2040-psram`'s `write8_command` wire format.
/// Bytes are pushed with generous stepping in between so the 4-deep TX
/// FIFO never overflows (mirrors a paced DMA feed without needing
/// cycle-exact interleaving).
fn drive_one_write8_frame(emu: &mut Emulator, addr: u32, val: u8, steps_per_chunk: u32) {
    push_tx_byte(emu, 40); // x: 40 bits to write
    push_tx_byte(emu, 0); // y: 0 bits to read
    push_tx_byte(emu, 0x02); // WRITE command
    push_tx_byte(emu, (addr >> 16) as u8);
    run_steps(emu, steps_per_chunk);
    push_tx_byte(emu, (addr >> 8) as u8);
    run_steps(emu, steps_per_chunk);
    push_tx_byte(emu, addr as u8);
    run_steps(emu, steps_per_chunk);
    push_tx_byte(emu, val);
    // Generous tail: the frame needs ~90 PIO cycles total; give it a
    // wide margin so no ambiguity about "did it finish" clouds the
    // pass/fail signal this test exists to give.
    run_steps(emu, steps_per_chunk * 4);
}

/// Pop one 32-bit word from PIO0 SM0's RX FIFO (autopushed by `IN
/// PINS` once the ISR reaches its 8-bit threshold — see the
/// `SHIFTCTRL` config in [`build_psram_pio_emulator_with`]). Returns
/// the low byte, which is where an 8-bit autopush lands (MSB-first
/// shift-left convention, matching `psram_read8`'s wire format).
fn pop_rx_byte(emu: &mut Emulator) -> u8 {
    emu.bus.read32(PIO0_BASE + 0x020) as u8
}

/// Drive one `psram_read8(psram_spi, addr)`-equivalent Fast Read
/// frame: 40 bits to write (cmd `0x0B` + 3-byte BE address + 1 dummy
/// byte), 8 bits to read — `rp2040-psram`'s `read8_command` wire
/// format. Returns the byte autopushed into the RX FIFO.
fn drive_one_fast_read8_frame(emu: &mut Emulator, addr: u32, steps_per_chunk: u32) -> u8 {
    push_tx_byte(emu, 40); // x: 40 bits to write
    push_tx_byte(emu, 8); // y: 8 bits to read
    push_tx_byte(emu, 0x0B); // Fast Read command
    push_tx_byte(emu, (addr >> 16) as u8);
    run_steps(emu, steps_per_chunk);
    push_tx_byte(emu, (addr >> 8) as u8);
    run_steps(emu, steps_per_chunk);
    push_tx_byte(emu, addr as u8);
    run_steps(emu, steps_per_chunk);
    push_tx_byte(emu, 0x00); // dummy byte
    // Generous tail: cmd+addr+dummy (40 bits, ~80 cycles) + the fudge
    // program's extra settling pulse + 8 read bits (~16 cycles) — give
    // it a wide margin, same rationale as `drive_one_write8_frame`.
    run_steps(emu, steps_per_chunk * 6);
    pop_rx_byte(emu)
}

/// **The regression.** With `step_quantum=8` (> 1, so the slow path's
/// bulk-vs-per-cycle branch matters) and a PSRAM attached
/// (`Bus::has_pin_watching_device() == true`), a single PIO-driven
/// `WRITE` frame must land its data byte in the PSRAM buffer.
///
/// Verified manually against the pre-fix `step_serial` (which always
/// called `tick_pio_and_route_irqs(consumed)` + one `update_gpio()` in
/// bulk on the slow path, regardless of any attached device): this
/// assertion FAILED — `bytes_written == 0`, `buffer[0x10] == 0x00` —
/// reproducing the real-firmware symptom (CS frames open/close at a
/// plausible cadence, but the reconstructed command byte is garbage
/// because most SCK edges inside each `step()`'s 8-cycle quantum never
/// reached `Psram::tick`). The fix (per-cycle interleave gated on
/// `Bus::has_pin_watching_device`) makes it pass.
#[test]
fn bulk_quantum_write_frame_lands_correct_byte_with_psram_attached() {
    let mut emu = build_psram_pio_emulator(8);

    drive_one_write8_frame(&mut emu, 0x10, 0xAB, 8);

    let psram = emu.bus.psram.as_ref().expect("psram attached");
    assert_eq!(
        psram.cmd_write_count, 1,
        "the command byte must decode as WRITE (0x02), not fall into `unknown` \
         — a value other than 1 here means edges were lost inside the quantum, \
         exactly the Gate 3 bug"
    );
    assert_eq!(
        psram.bytes_written, 1,
        "exactly one data byte must be written"
    );
    assert_eq!(
        psram.buffer[0x10], 0xAB,
        "the byte must land at the address the frame specified"
    );
}

/// Baseline: the same frame, same PSRAM, but `step_quantum=1`. This
/// path was never broken (every quantum is trivially "one cycle"), so
/// it must pass both before and after the fix — included so a future
/// regression in the *fast path* or the *quantum=1 case specifically*
/// would be caught by a different assertion than the bulk-quantum one
/// above.
#[test]
fn quantum_one_write_frame_lands_correct_byte_with_psram_attached() {
    let mut emu = build_psram_pio_emulator(1);

    drive_one_write8_frame(&mut emu, 0x20, 0xCD, 40);

    let psram = emu.bus.psram.as_ref().expect("psram attached");
    assert_eq!(psram.cmd_write_count, 1);
    assert_eq!(psram.bytes_written, 1);
    assert_eq!(psram.buffer[0x20], 0xCD);
}

/// **Sol's approved read-path fix, driven through the real
/// `spi_psram_fudge` program.** Write one byte with `0x02`, then read
/// it back with `0x0B` through the *same* PIO frame shape
/// `psram_read8` uses — including the fudge program's extra settling
/// pulse before its read loop, unlike
/// `bulk_quantum_write_frame_lands_correct_byte_with_psram_attached`
/// (which never exercises that code path: a pure write frame has
/// `y=0`, so `jmp !y, begin` skips straight past it).
///
/// Verified manually against the pre-fix `Psram` (no
/// `read_output_delay_sck`, i.e. `Psram::picogus()` without
/// `.with_read_output_delay(1)`): this assertion FAILED —
/// `read_back != val` — reproducing the real-firmware symptom (`main.c`
/// UART: `PSRAM failure at address 1 (1 != 2)`, a clean 1-bit left
/// shift of the expected byte). Attaching the PSRAM with
/// `with_read_output_delay(1)` (what `picocalc_board::pins::
/// psram_picocalc()` now does) makes it pass.
#[test]
fn fast_read_round_trip_through_real_fudge_program_with_output_delay() {
    let psram = Psram::new(PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI).with_read_output_delay(1);
    let mut emu = build_psram_pio_emulator_with(psram, 1);

    let addr = 0x40u32;
    let val = 0x5Au8;
    drive_one_write8_frame(&mut emu, addr, val, 40);

    // Sanity: the write itself must have landed (isolates a read-path
    // failure from a write-path one, matching the "confirm write path
    // is solid before touching read" discipline this Gate 3 pass used
    // throughout).
    {
        let psram = emu.bus.psram.as_ref().expect("psram attached");
        assert_eq!(psram.cmd_write_count, 1);
        assert_eq!(psram.buffer[addr as usize], val);
    }

    let read_back = drive_one_fast_read8_frame(&mut emu, addr, 40);

    let psram = emu.bus.psram.as_ref().expect("psram attached");
    assert_eq!(
        psram.cmd_fast_read_count, 1,
        "the command byte must decode as Fast Read (0x0B)"
    );
    assert_eq!(
        read_back, val,
        "Fast Read through the real fudge program must return the byte \
         just written — a mismatch here is the Gate 3 read-path bug \
         (1-bit stream shift) this test exists to catch"
    );
}

/// Sanity check on the gate itself: with no PSRAM attached, a PIO
/// program driving arbitrary pins at `step_quantum=8` must still work
/// via the (unconditionally bulk) `else` branch — this is the "existing
/// (PSRAM-less) behaviour is unchanged" guarantee, exercised through
/// the same program/harness shape as the regression above so it is
/// directly comparable.
#[test]
fn bulk_quantum_without_psram_still_runs_the_program_to_completion() {
    // No `.psram(..)` — `has_pin_watching_device()` is false, so the
    // slow path must take the single bulk `tick_pio_and_route_irqs`
    // call, same as pre-fix. This only checks the PIO program itself
    // still runs (SM0 must have advanced its PC off address 0) — there
    // is no PSRAM model here to check byte-level correctness against.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(8)
        .build()
        .expect("Serial build is infallible");
    park_core0_on_nops(&mut emu);
    for (i, word) in assemble_spi_psram_fudge().iter().enumerate() {
        emu.bus
            .write32(PIO0_BASE + 0x048 + (i as u32) * 4, *word as u32);
    }
    force_set_pindirs(&mut emu, PIO0_BASE, PIO0_BASE, PIN_CS, 2, 0b11);
    force_set_pindirs(&mut emu, PIO0_BASE, PIO0_BASE, PIN_MOSI, 1, 1);
    let pinctrl = (PIN_MOSI as u32)
        | ((PIN_CS as u32) << 10)
        | ((PIN_MISO as u32) << 15)
        | (1u32 << 20)
        | (2u32 << 29);
    emu.bus.write32(PIO0_BASE + 0x0DC, pinctrl);
    emu.bus
        .write32(PIO0_BASE + 0x0CC, (9u32 << 12) | (0u32 << 7));
    emu.bus.write32(
        PIO0_BASE + 0x0D0,
        (8u32 << 25) | (8u32 << 20) | (1u32 << 17) | (1u32 << 16),
    );
    emu.bus.write32(PIO0_BASE + 0x000, 0x1);

    for b in [40u8, 0, 0x02, 0x00, 0x00, 0x20, 0xEF] {
        emu.bus.write8(PIO0_BASE + 0x010, b);
    }
    run_steps(&mut emu, 40);

    assert_ne!(
        emu.bus.pio[0].sm[0].pc(),
        0,
        "PIO program must have advanced past its reset PC"
    );
}
