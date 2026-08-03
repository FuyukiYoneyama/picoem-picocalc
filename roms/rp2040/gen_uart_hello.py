#!/usr/bin/env python3
"""
Generate a minimal RP2040 Cortex-M0+ UART0 "hello" flash image for the
`picocalc-run` headless runner (picocalc_emu Gate 1).

Purpose: exercise the runner's UART capture path end-to-end with a known
byte string. `blinky.bin` proves determinism but writes nothing to
UART0; `picocalc_helloworld` initialises UART0 but prints to the LCD,
so neither fixture puts real bytes through `drain_uart0_tx_log`.

Pairs with the synthetic `bootrom.bin` produced by `gen_blinky.py`
(word 0 = initial SP, word 1 = 0x10000001), exactly like `blinky.bin`:
there is no pico-sdk vector table at flash+0x100, so `picocalc-run`
reports `boot.mode = "bootrom_reset_vector"`.

Program:
  1. RESETS_CLR (0x4000F000) <- 1 << 22   release UART0 from reset.
     Without this the bus dispatch swallows every UART write.
  2. UARTCR (0x40034030) <- UARTEN | TXE  (0x0101).
     Required: `UartRegs::push_tx` returns early unless both bits are
     set, so a DR write before this is dropped on the floor.
  3. UARTDR (0x40034000) <- each byte of MESSAGE, back to back.
     No UARTFR.TXFF poll: the emulator's `push_tx` taps the byte into
     the diagnostic TX wire log *before* the FIFO-capacity check, and
     the FIFO is 16 deep against a 6-byte message anyway.
  4. B . — park forever so the run ends on the cycle budget.

Layout:
  bootrom.bin (ROM @ 0x00000000): reuse the one gen_blinky.py writes.
  uart_hello.bin (flash @ 0x10000000):
    0x000..0x0xx: reset handler code
    0x0xx..:      literal pool (word-aligned)
"""

import struct
import sys
from pathlib import Path

# =============================================================================
# Constants
# =============================================================================

FLASH_BASE = 0x10000000

# RESETS: SET/CLR/XOR aliases live at base + 0x1000/0x2000/0x3000.
RESETS_BASE = 0x4000C000
RESETS_CLR = RESETS_BASE + 0x3000
RESET_UART0_BIT = 22

# PL011 UART0.
UART0_BASE = 0x40034000
UARTDR = 0x000
UARTCR = 0x030
UARTCR_UARTEN = 1 << 0
UARTCR_TXE = 1 << 8

MESSAGE = b'HELLO\n'

# =============================================================================
# Thumb-16 instruction encoding helpers (Cortex-M0+ subset)
#
# Same encodings as gen_blinky.py — kept local so each generator reads
# standalone.
# =============================================================================

def thumb_movs_imm8(rd, imm8):
    """MOVS Rd, #imm8 — T1 encoding. Rd in r0..r7."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x2000 | (rd << 8) | imm8)

def thumb_ldr_pc(rt, imm8):
    """LDR Rt, [PC, #imm8] — T1 (word load from literal pool).

    imm8 is word-scaled. Effective address = Align(PC, 4) + imm8 * 4,
    where PC = current instruction address + 4.
    """
    assert 0 <= rt <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x4800 | (rt << 8) | imm8)

def thumb_str_imm5(rt, rn, imm5_words):
    """STR Rt, [Rn, #imm5*4] — T1 encoding. imm5 is word-scaled (0..31)."""
    assert 0 <= rt <= 7 and 0 <= rn <= 7 and 0 <= imm5_words <= 31
    return struct.pack('<H', 0x6000 | (imm5_words << 6) | (rn << 3) | rt)

def thumb_b(offset):
    """B label — T2 encoding. offset from (PC+4), range -2048..+2046, even."""
    assert -2048 <= offset <= 2046 and offset % 2 == 0
    imm11 = (offset >> 1) & 0x7FF
    return struct.pack('<H', 0xE000 | imm11)

# =============================================================================
# Code generation
# =============================================================================

def literal_imm8(instr_off, literal_off):
    """Word-scaled imm8 for `LDR Rt, [PC, #imm8]` at `instr_off` reading
    the literal at `literal_off` (both flash-relative byte offsets)."""
    pc = instr_off + 4
    base = pc & ~3
    delta = literal_off - base
    assert delta >= 0 and delta % 4 == 0, (instr_off, literal_off, delta)
    imm8 = delta // 4
    assert imm8 <= 255
    return imm8

def build_uart_hello():
    """
    Build the reset handler + literal pool for uart_hello.bin.

    Register usage:
      r0 = RESETS_CLR, later UART0_BASE
      r1 = scratch value to store

    The code length is fixed and known ahead of time, so the literal
    pool offsets are computed in a first pass and the instructions are
    emitted in a second.
    """
    # Instruction sequence as (kind, args); sizes are all 2 bytes.
    #   ldr r0, =RESETS_CLR
    #   movs r1, #1 ; lsls r1, r1, #22  -> built as a literal instead,
    #                                      keeps the pass structure flat
    #   str r1, [r0, #0]
    #   ldr r0, =UART0_BASE
    #   ldr r1, =UARTCR value
    #   str r1, [r0, #UARTCR/4]
    #   (per byte) movs r1, #c ; str r1, [r0, #0]
    #   b .
    n_instrs = 6 + 2 * len(MESSAGE) + 1
    code_len = n_instrs * 2
    pool_off = (code_len + 3) & ~3   # word-align the literal pool

    lit_resets_clr = pool_off + 0
    lit_uart0_mask = pool_off + 4
    lit_uart0_base = pool_off + 8
    lit_uart0_cr = pool_off + 12

    code = b''
    off = 0

    def emit(chunk):
        nonlocal code, off
        code += chunk
        off += len(chunk)

    # --- 1. Release UART0 from reset -------------------------------------
    emit(thumb_ldr_pc(0, literal_imm8(off, lit_resets_clr)))   # r0 = RESETS_CLR
    emit(thumb_ldr_pc(1, literal_imm8(off, lit_uart0_mask)))   # r1 = 1 << 22
    emit(thumb_str_imm5(1, 0, 0))                              # [r0] = r1

    # --- 2. Enable UART0 TX ----------------------------------------------
    emit(thumb_ldr_pc(0, literal_imm8(off, lit_uart0_base)))   # r0 = UART0_BASE
    emit(thumb_ldr_pc(1, literal_imm8(off, lit_uart0_cr)))     # r1 = UARTEN|TXE
    emit(thumb_str_imm5(1, 0, UARTCR // 4))                    # [r0+0x30] = r1

    # --- 3. Push the message through UARTDR ------------------------------
    for byte in MESSAGE:
        emit(thumb_movs_imm8(1, byte))                         # r1 = byte
        emit(thumb_str_imm5(1, 0, UARTDR // 4))                # [r0+0x00] = r1

    # --- 4. Park ----------------------------------------------------------
    emit(thumb_b(-4))                                          # B . (self)

    assert off == code_len, (off, code_len)

    # --- Literal pool ------------------------------------------------------
    code += b'\x00' * (pool_off - off)
    code += struct.pack('<I', RESETS_CLR)
    code += struct.pack('<I', 1 << RESET_UART0_BIT)
    code += struct.pack('<I', UART0_BASE)
    code += struct.pack('<I', UARTCR_UARTEN | UARTCR_TXE)
    return code

def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
    image = build_uart_hello()
    path = out_dir / 'uart_hello.bin'
    path.write_bytes(image)
    print(f"Wrote {path} ({len(image)} bytes)")
    print(f"  flash base:    {FLASH_BASE:#010x}")
    print(f"  reset handler: {FLASH_BASE:#010x} (Thumb entry {FLASH_BASE | 1:#010x})")
    print(f"  message:       {MESSAGE!r} ({len(MESSAGE)} bytes)")
    print("  pair with:     bootrom.bin (from gen_blinky.py)")

if __name__ == '__main__':
    main()
