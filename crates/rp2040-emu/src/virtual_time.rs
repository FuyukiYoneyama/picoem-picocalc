//! Deterministic conversion between emulator cycles and virtual nanoseconds.
//!
//! The clock is rebased whenever the firmware changes `clk_sys`.  Elapsed
//! time before the change keeps the old rate; only cycles after the rebase use
//! the new rate.  All arithmetic is integer-only so reports and external
//! devices cannot depend on host floating-point behaviour.

#[derive(Clone, Debug)]
pub struct VirtualClock {
    epoch_cycles: u64,
    epoch_ns: u64,
    hz: u64,
    snapshot_ns: u64,
}

impl VirtualClock {
    /// Create a clock whose epoch is cycle zero and whose initial rate is
    /// `hz`. A zero rate is clamped to one to keep all conversions defined.
    pub fn new(hz: u32) -> Self {
        Self {
            epoch_cycles: 0,
            epoch_ns: 0,
            hz: u64::from(hz).max(1),
            snapshot_ns: 0,
        }
    }

    /// Virtual nanoseconds at an absolute master-cycle count.
    #[inline]
    pub fn ns_at(&self, cycles: u64) -> u64 {
        let elapsed = u128::from(cycles.saturating_sub(self.epoch_cycles));
        self.epoch_ns
            .saturating_add((elapsed * 1_000_000_000 / u128::from(self.hz)) as u64)
    }

    /// The cycle count at which the clock reaches `ns`.
    #[inline]
    pub fn cycles_at(&self, ns: u64) -> u64 {
        let ahead = u128::from(ns.saturating_sub(self.epoch_ns));
        let cycles = ahead * u128::from(self.hz) / 1_000_000_000;
        self.epoch_cycles
            .saturating_add(u64::try_from(cycles).unwrap_or(u64::MAX))
    }

    /// Adopt a new clock rate from `cycles` onwards. Returns true when the
    /// rate changed and the epoch was rebased.
    #[inline]
    pub fn rebase(&mut self, cycles: u64, hz: u32) -> bool {
        let hz = u64::from(hz).max(1);
        if hz == self.hz {
            return false;
        }
        self.epoch_ns = self.ns_at(cycles);
        self.epoch_cycles = cycles;
        self.hz = hz;
        self.snapshot_ns = self.epoch_ns;
        true
    }

    /// Advance the snapshot to `cycles` and return the elapsed virtual time
    /// since the previous snapshot. A clock-rate change establishes a new
    /// epoch and deliberately returns zero for that boundary, matching the
    /// harness's historical rebasing behaviour.
    #[inline]
    pub fn advance_to(&mut self, cycles: u64, hz: u32) -> u64 {
        if self.rebase(cycles, hz) {
            return 0;
        }
        let now = self.ns_at(cycles);
        let delta = now.saturating_sub(self.snapshot_ns);
        self.snapshot_ns = now;
        delta
    }

    /// Reset the epoch without carrying time from an earlier machine run.
    #[inline]
    pub(crate) fn reset(&mut self, hz: u32) {
        self.epoch_cycles = 0;
        self.epoch_ns = 0;
        self.hz = u64::from(hz).max(1);
        self.snapshot_ns = 0;
    }

    /// Restore a snapshot after an MCU warm reset. The wall of virtual time
    /// is retained, but subsequent cycles use the reset clock rate.
    #[inline]
    pub(crate) fn restore_after_reset(&mut self, cycles: u64, ns: u64, hz: u32) {
        self.epoch_cycles = cycles;
        self.epoch_ns = ns;
        self.hz = u64::from(hz).max(1);
        self.snapshot_ns = ns;
    }
}

#[cfg(test)]
mod tests {
    use super::VirtualClock;

    #[test]
    fn integer_conversion_and_rebase_are_deterministic() {
        let mut clock = VirtualClock::new(100_000_000);
        assert_eq!(clock.ns_at(100), 1_000);
        assert_eq!(clock.advance_to(100, 100_000_000), 1_000);
        assert_eq!(clock.advance_to(200, 50_000_000), 0);
        assert_eq!(clock.ns_at(250), 3_000);
    }

    #[test]
    fn cycles_at_saturates_far_future() {
        let clock = VirtualClock::new(u32::MAX);
        assert_eq!(clock.cycles_at(u64::MAX), u64::MAX);
    }

    #[test]
    fn incremental_deltas_sum_to_the_global_integer_conversion() {
        let mut clock = VirtualClock::new(3);
        let first = clock.advance_to(1, 3);
        let second = clock.advance_to(2, 3);
        assert_eq!(first + second, clock.ns_at(2));
    }
}
