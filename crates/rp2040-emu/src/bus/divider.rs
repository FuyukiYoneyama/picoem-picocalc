//! RP2040 per-core SIO divider, used by both execution models.
//!
//! Every operand write restarts an eight-sysclk calculation. Result writes
//! cancel it; only QUOTIENT reads clear DIRTY. Results while READY=0 are
//! architecturally undefined; this model retains the preceding result.

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct Divider {
    dividend: u32,
    divisor: u32,
    quotient: u32,
    remainder: u32,
    dirty: bool,
    cycle: u64,
    remaining: u8,
    pending: (u32, u32),
}

impl Divider {
    pub(crate) fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            0x060 | 0x068 => self.dividend,
            0x064 | 0x06c => self.divisor,
            0x070 => {
                self.dirty = false;
                self.quotient
            }
            0x074 => self.remainder,
            0x078 => u32::from(self.remaining == 0) | (u32::from(self.dirty) << 1),
            _ => 0,
        }
    }

    pub(crate) fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            0x060 | 0x064 | 0x068 | 0x06c => {
                if offset == 0x060 || offset == 0x068 {
                    self.dividend = value;
                } else {
                    self.divisor = value;
                }
                let signed = offset == 0x068 || offset == 0x06c;
                self.pending = Self::calculate(self.dividend, self.divisor, signed);
                self.remaining = 8;
                self.dirty = true;
            }
            0x070 | 0x074 => {
                if offset == 0x070 {
                    self.quotient = value;
                } else {
                    self.remainder = value;
                }
                self.remaining = 0;
                self.dirty = true;
            }
            _ => {}
        }
    }

    /// Advance this core's clock, including instruction/exception cycles.
    pub(crate) fn advance_cycles(&mut self, cycles: u64) {
        self.cycle = self.cycle.wrapping_add(cycles);
        if self.remaining == 0 {
            return;
        }
        if cycles >= u64::from(self.remaining) {
            (self.quotient, self.remainder) = self.pending;
            self.remaining = 0;
        } else {
            self.remaining -= cycles as u8;
        }
    }

    /// Catch up to elapsed system time after a parked core or quantum.
    /// The local clock may already lead the scheduler by an instruction;
    /// never count those cycles a second time or move the clock backwards.
    pub(crate) fn sync_cycle(&mut self, cycle: u64) {
        let elapsed = cycle.wrapping_sub(self.cycle);
        if elapsed < (1u64 << 63) {
            self.advance_cycles(elapsed);
        }
    }

    fn calculate(dividend: u32, divisor: u32, signed: bool) -> (u32, u32) {
        if divisor == 0 {
            let quotient = if signed && (dividend as i32) < 0 {
                1
            } else {
                u32::MAX
            };
            (quotient, dividend)
        } else if signed {
            let a = dividend as i32;
            let b = divisor as i32;
            (a.wrapping_div(b) as u32, a.wrapping_rem(b) as u32)
        } else {
            (dividend / divisor, dividend % divisor)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Divider;

    #[test]
    fn clock_wrap_and_scheduler_catchup_preserve_eight_cycles() {
        let mut divider = Divider {
            cycle: u64::MAX - 3,
            ..Divider::default()
        };
        divider.write32(0x064, 3);
        divider.write32(0x060, 10);
        divider.advance_cycles(7);
        assert_eq!(divider.read32(0x078), 2);
        divider.sync_cycle(2); // Scheduler lags the local clock (now 3).
        assert_eq!(divider.read32(0x078), 2);
        divider.sync_cycle(4);
        assert_eq!(divider.read32(0x078), 3);
        assert_eq!(divider.read32(0x074), 1);
        assert_eq!(divider.read32(0x070), 3);
    }
}
