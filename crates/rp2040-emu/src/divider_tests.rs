//! Execute pinned Pico SDK integer division assembly against the actual CPUs.

use crate::{Config, Emulator};

pub(crate) const MAILBOX: u32 = 0x2003_0000;
pub(crate) const OUTPUT: u32 = MAILBOX + 0x20;
pub(crate) const FIXTURE: &[u8] = include_bytes!("../tests/fixtures/sio_divider_sdk/fixture.bin");
pub(crate) const CASES: &[(u64, u64, bool)] = &[
    (u64::MAX, 3, false),
    (u64::MAX, 0x1234_5678, false), // y64_x32 reciprocal path
    (u64::MAX, u32::MAX as u64, false),
    (u64::MAX, 0x0000_1234_5678_9abc, false), // y64_x48
    (u64::MAX, 0x1234_5678_9abc_def0, false), // y64_x64
    (0x1234, 0x1234_5678_9abc_def0, false),
    (u64::MAX, u64::MAX, false),
    (i64::MIN as u64, u64::MAX, true), // signed overflow wraps
    ((-1_234_567_890_123_456i64) as u64, 0x1234_5678, true),
    (1_234_567_890_123_456, (-0x1234_5678i64) as u64, true),
    (
        (-1_234_567_890_123_456i64) as u64,
        (-0x1234_5678i64) as u64,
        true,
    ),
    (i64::MAX as u64, 0x1234_5678_9abc, true),
    (i64::MIN as u64, 0x1234_5678_9abc_def0, true),
];

pub(crate) fn seed(core: usize, case: (u64, u64, bool), quantum: u32) -> Emulator {
    let mut emu = Emulator::new(Config::default());
    emu.step_quantum = quantum;
    emu.load_image(0x2000_0000, FIXTURE);
    emu.core_mut(1 - core).halt();
    let cpu = emu.core_mut(core);
    cpu.wake();
    cpu.regs.set_pc(0x2000_0000);
    cpu.regs.msp = 0x2004_0000;
    cpu.regs.r[13] = 0x2004_0000;
    let (a, b, signed) = case;
    for (i, word) in [
        a as u32,
        (a >> 32) as u32,
        b as u32,
        (b >> 32) as u32,
        u32::from(signed),
    ]
    .into_iter()
    .enumerate()
    {
        emu.poke(MAILBOX + i as u32 * 4, word);
    }
    // A dirty, still-running divider must survive promotion and the SDK's
    // save/restore path. The saved 77/11 result is ready by the function call.
    emu.bus.sio.write32(0x060, 77, core);
    emu.bus.sio.write32(0x064, 11, core);
    emu.bus.sio.advance_divider_cycles(core, 2);
    emu
}

pub(crate) fn check(mut read: impl FnMut(u32) -> u32, case: (u64, u64, bool)) {
    assert_eq!(
        read(OUTPUT + 16),
        1,
        "SDK completion marker missing for {case:?}"
    );
    let quotient = u64::from(read(OUTPUT)) | (u64::from(read(OUTPUT + 4)) << 32);
    let remainder = u64::from(read(OUTPUT + 8)) | (u64::from(read(OUTPUT + 12)) << 32);
    let (a, b, signed) = case;
    let expected = if signed {
        let (a, b) = (a as i64 as i128, b as i64 as i128);
        ((a / b) as u64, (a % b) as u64)
    } else {
        let (a, b) = (a as u128, b as u128);
        ((a / b) as u64, (a % b) as u64)
    };
    assert_eq!((quotient, remainder), expected, "SDK case {case:?}");
}

#[test]
fn sdk_64_bit_division_both_cores_and_quantum_sizes() {
    for quantum in [1, 128] {
        for core in 0..2 {
            for &case in CASES {
                let mut emu = seed(core, case, quantum);
                // Budget fails promptly if a stale reciprocal traps correction.
                for _ in 0..512 {
                    emu.run(128).unwrap();
                    if emu.peek(OUTPUT + 16) == 1 {
                        break;
                    }
                }
                check(|addr| emu.peek(addr), case);
                assert_eq!(emu.bus.sio.read32(0x060, core), 77);
                assert_eq!(emu.bus.sio.read32(0x064, core), 11);
                assert_eq!(emu.bus.sio.read32(0x074, core), 0);
                assert_eq!(emu.bus.sio.read32(0x070, core), 7);
            }
        }
    }
}
