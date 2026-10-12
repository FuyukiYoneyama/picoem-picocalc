use std::sync::Arc;
use rp2040_emu::bus::sio::Sio;
use rp2040_emu::core::bus_trait::CoreBus;
use rp2040_emu::threaded::{SharedState, WorkerBus};

trait Registers {
    fn write(&mut self, core: usize, offset: u32, value: u32);
    fn read(&mut self, core: usize, offset: u32) -> u32;
}
impl Registers for Sio {
    fn write(&mut self, c: usize, o: u32, v: u32) { self.write32(o, v, c); }
    fn read(&mut self, c: usize, o: u32) -> u32 { self.read32(o, c) }
}
struct Workers([WorkerBus; 2]);
impl Registers for Workers {
    fn write(&mut self, c: usize, o: u32, v: u32) { self.0[c].write32(0xd0000000 + o, v); }
    fn read(&mut self, c: usize, o: u32) -> u32 { self.0[c].read32(0xd0000000 + o) }
}
fn workers() -> Workers {
    let shared = Arc::new(SharedState::new_default());
    Workers([WorkerBus::new(shared.clone(), 0), WorkerBus::new(shared, 1)])
}
fn observe(r: &mut impl Registers, core: usize) -> (u32, u32) {
    let remainder = r.read(core, 0x74);
    let quotient = r.read(core, 0x70);
    (quotient, remainder)
}
fn report(model: &str, name: &str, actual: (u32,u32), expected: (u32,u32)) {
    println!("{{\"model\":\"{model}\",\"case\":\"{name}\",\"actual\":[{},{}],\"expected\":[{},{}],\"matches_spec\":{}}}",
        actual.0, actual.1, expected.0, expected.1, actual == expected);
}
fn suite<R: Registers>(model: &str, fresh: impl Fn() -> R) {
    // Each arithmetic case is seeded with 0/3 and is independent of reset state.
    for (name, writes, expected) in [
        ("unsigned_dividend_first", vec![(0x60,10),(0x64,3)], (3,1)),
        ("unsigned_divisor_first", vec![(0x64,3),(0x60,10)], (3,1)),
        ("signed_dividend_first", vec![(0x68,(-10i32) as u32),(0x6c,3)], ((-3i32) as u32,u32::MAX)),
        ("signed_divisor_first", vec![(0x6c,3),(0x68,(-10i32) as u32)], ((-3i32) as u32,u32::MAX)),
        ("repeated_dividend", vec![(0x60,10),(0x64,3),(0x60,20)], (6,2)),
        ("repeated_divisor", vec![(0x60,20),(0x64,3),(0x64,7)], (2,6)),
        ("last_signed_dividend_selects_mode", vec![(0x64,3),(0x68,(-10i32) as u32)], ((-3i32) as u32,u32::MAX)),
        ("last_unsigned_dividend_selects_mode", vec![(0x68,(-10i32) as u32),(0x6c,3),(0x60,10)], (3,1)),
        ("sdk_reciprocal_operand_sequence", vec![(0x64,65537),(0x60,u32::MAX)], (65535,0)),
        ("unsigned_zero_divisor_first", vec![(0x64,0),(0x60,42)], (u32::MAX,42)),
        ("signed_zero_divisor_first", vec![(0x6c,0),(0x68,(-42i32) as u32)], (1,(-42i32) as u32)),
        ("signed_overflow_dividend_first", vec![(0x68,0x80000000),(0x6c,u32::MAX)], (0x80000000,0)),
    ] {
        let mut r = fresh();
        r.write(0,0x60,0); r.write(0,0x64,3);
        for (offset,value) in writes { r.write(0,offset,value); }
        report(model,name,observe(&mut r,0),expected);
    }
    let mut r = fresh();
    r.write(0,0x60,100); r.write(0,0x64,7);
    r.write(1,0x60,20); r.write(1,0x64,3);
    report(model,"core0_isolation",observe(&mut r,0),(14,2));
    report(model,"core1_isolation",observe(&mut r,1),(6,2));
    let mut r = fresh();
    r.write(0,0x60,100); r.write(0,0x64,7);
    let _ = r.read(0,0x70);
    report(model,"quotient_read_clears_dirty",(r.read(0,0x78)&2,0),(0,0));
    let mut r = fresh();
    r.write(0,0x60,100); r.write(0,0x64,7);
    let _ = r.read(0,0x74); let _ = r.read(0,0x74);
    report(model,"repeated_remainder_keeps_dirty",(r.read(0,0x78)&2,0),(2,0));
    let mut r = fresh();
    r.write(0,0x60,0); r.write(0,0x64,3);
    let _ = observe(&mut r,0);
    let before = r.read(0,0x78)&2;
    r.write(0,0x60,10);
    report(model,"dividend_write_after_result_consumption",(before,r.read(0,0x78)&2),(0,2));
}
fn main() {
    suite("Serial Sio", Sio::new);
    suite("Threaded WorkerBus (no worker threads started)", workers);
}
