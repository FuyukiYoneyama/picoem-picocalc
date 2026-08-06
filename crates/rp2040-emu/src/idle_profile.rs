//! OPT0-A Serial idle-path profiler.
//!
//! This module is compiled only with the `idle-profiler` feature. The
//! normal correctness/performance build therefore pays no field, branch,
//! counter, or histogram cost.

/// Machine-readable profile schema emitted by diagnostic harnesses.
pub const IDLE_PROFILE_SCHEMA_VERSION: u32 = 1;

/// Number of power-of-two thresholds: 1 through 2^63 cycles.
pub const IDLE_HISTOGRAM_BUCKETS: usize = 64;

/// Cumulative episode-length distribution.
///
/// At index `i`, the threshold is `2^i`. `episodes_ge[i]` is the number
/// of episodes at least that long and `cycle_mass_ge[i]` is the sum of
/// the full lengths of those episodes. The latter is the plan's `S(K)`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CumulativeHistogramSnapshot {
    /// Number of episodes whose length is at least the bucket threshold.
    pub episodes_ge: [u64; IDLE_HISTOGRAM_BUCKETS],
    /// Total cycles belonging to episodes at least the bucket threshold.
    pub cycle_mass_ge: [u64; IDLE_HISTOGRAM_BUCKETS],
}

impl Default for CumulativeHistogramSnapshot {
    fn default() -> Self {
        Self {
            episodes_ge: [0; IDLE_HISTOGRAM_BUCKETS],
            cycle_mass_ge: [0; IDLE_HISTOGRAM_BUCKETS],
        }
    }
}

impl CumulativeHistogramSnapshot {
    fn record(&mut self, length: u64) {
        if length == 0 {
            return;
        }
        for i in 0..IDLE_HISTOGRAM_BUCKETS {
            let threshold = 1u64 << i;
            if length < threshold {
                break;
            }
            self.episodes_ge[i] = self.episodes_ge[i].saturating_add(1);
            self.cycle_mass_ge[i] = self.cycle_mass_ge[i].saturating_add(length);
        }
    }
}

/// Overlapping cycle totals for autonomous sources that prevented a
/// both-blocked interval from being conservatively classified as safe.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IdleBlockerCycles {
    /// At least one PIO state machine was enabled or a PIO IRQ was pending.
    pub pio: u64,
    /// At least one DMA channel was busy or a DMA interrupt was latched.
    pub dma: u64,
    /// At least one PWM slice was enabled or a PWM interrupt was latched.
    pub pwm: u64,
    /// At least one core-local SysTick was enabled.
    pub systick: u64,
    /// UART0 or UART1 had FIFO/shift/interrupt work outstanding.
    pub uart: u64,
    /// SPI0 or SPI1 had FIFO/shift/interrupt work outstanding.
    pub spi: u64,
    /// I2C0 or I2C1 had FIFO/bus/interrupt work outstanding.
    pub i2c: u64,
    /// ADC conversion/FIFO/interrupt work was outstanding.
    pub adc: u64,
    /// TIMER had a latched or forced interrupt condition.
    pub timer: u64,
    /// A bus or enabled NVIC interrupt was already pending.
    pub pending_irq: u64,
}

/// Number of both-blocked episodes in which each overlapping blocker
/// was observed at least once.
///
/// A single episode may contribute to several fields. These values are
/// therefore diagnostic attribution counts, not a partition of the
/// total episode count.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IdleBlockerEpisodes {
    pub pio: u64,
    pub dma: u64,
    pub pwm: u64,
    pub systick: u64,
    pub uart: u64,
    pub spi: u64,
    pub i2c: u64,
    pub adc: u64,
    pub timer: u64,
    pub pending_irq: u64,
}

/// One observation of the conservative idle gate used by OPT0-A.
///
/// This is deliberately named a "current probe", not a complete event
/// horizon. At schema 1 the only scheduled lazy deadline is TIMER; active
/// PIO/DMA/PWM/serial sources are represented as blocker bits and are not
/// yet converted to exact next-event cycles.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IdleCurrentProbe {
    /// Current emulated master cycle.
    pub master_cycle: u64,
    /// Soonest deadline among lazy scheduled sources modelled today.
    pub next_lazy_deadline: Option<u64>,
    /// Number of overlapping conservative blockers observed.
    pub blocker_count: u32,
    /// True only when the current source checks find no blocker.
    pub proven_quiescent: bool,
}

/// Stable snapshot of the opt-in Serial idle profiler.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IdleProfileSnapshot {
    /// Calls made to the Serial step function while profiling.
    pub step_calls: u64,
    /// Master-clock cycles advanced by all observed steps.
    pub total_master_cycles: u64,
    /// Core-0 cycles actually returned by instruction execution.
    pub core0_executed_cycles: u64,
    /// Core-1 cycles actually returned by instruction execution.
    pub core1_executed_cycles: u64,
    /// Master cycles advanced while both cores were halted or WFE-waiting.
    pub both_blocked_cycles: u64,
    /// Subset of `both_blocked_cycles` with no conservative blocker.
    pub proven_safe_cycles: u64,
    /// Both-blocked calls that could not advance because no wake deadline existed.
    pub zero_progress_blocked_steps: u64,
    /// Blocked cycles during which core 0 was halted.
    pub core0_halted_blocked_cycles: u64,
    /// Blocked cycles during which core 0 was WFE-waiting.
    pub core0_wfe_blocked_cycles: u64,
    /// Blocked cycles during which core 1 was halted.
    pub core1_halted_blocked_cycles: u64,
    /// Blocked cycles during which core 1 was WFE-waiting.
    pub core1_wfe_blocked_cycles: u64,
    /// Length distribution for all both-blocked episodes (upper bound).
    pub blocked_lengths: CumulativeHistogramSnapshot,
    /// Length distribution for conservatively proven-safe sub-episodes.
    pub proven_safe_lengths: CumulativeHistogramSnapshot,
    /// Initial timer-horizon distance distribution for blocked episodes.
    pub initial_horizon_distances: CumulativeHistogramSnapshot,
    /// Overlapping per-source reasons why blocked cycles were not proven safe.
    pub blockers: IdleBlockerCycles,
    /// Overlapping episode counts for the same blocker sources.
    pub blocker_episodes: IdleBlockerEpisodes,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct IdleBlockerMask(u16);

impl IdleBlockerMask {
    pub(crate) const PIO: u16 = 1 << 0;
    pub(crate) const DMA: u16 = 1 << 1;
    pub(crate) const PWM: u16 = 1 << 2;
    pub(crate) const SYSTICK: u16 = 1 << 3;
    pub(crate) const UART: u16 = 1 << 4;
    pub(crate) const SPI: u16 = 1 << 5;
    pub(crate) const I2C: u16 = 1 << 6;
    pub(crate) const ADC: u16 = 1 << 7;
    pub(crate) const TIMER: u16 = 1 << 8;
    pub(crate) const PENDING_IRQ: u16 = 1 << 9;

    pub(crate) fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    fn contains(self, bit: u16) -> bool {
        self.0 & bit != 0
    }

    pub(crate) fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub(crate) fn count(self) -> u32 {
        self.0.count_ones()
    }
}

/// Mutable profiler state owned by an `Emulator` in diagnostic builds.
#[derive(Clone, Debug, Default)]
pub(crate) struct IdleProfiler {
    snapshot: IdleProfileSnapshot,
    open_blocked_length: u64,
    open_safe_length: u64,
    open_blockers: IdleBlockerMask,
}

impl IdleProfiler {
    pub(crate) fn record_running(&mut self, master: u64, core0: u64, core1: u64) {
        self.close_blocked_episode();
        self.snapshot.step_calls = self.snapshot.step_calls.saturating_add(1);
        self.snapshot.total_master_cycles =
            self.snapshot.total_master_cycles.saturating_add(master);
        self.snapshot.core0_executed_cycles =
            self.snapshot.core0_executed_cycles.saturating_add(core0);
        self.snapshot.core1_executed_cycles =
            self.snapshot.core1_executed_cycles.saturating_add(core1);
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn record_blocked(
        &mut self,
        cycles: u64,
        horizon_distance: u64,
        blockers: IdleBlockerMask,
        core0_halted: bool,
        core0_wfe: bool,
        core1_halted: bool,
        core1_wfe: bool,
    ) {
        self.snapshot.step_calls = self.snapshot.step_calls.saturating_add(1);
        self.snapshot.total_master_cycles =
            self.snapshot.total_master_cycles.saturating_add(cycles);
        self.snapshot.both_blocked_cycles =
            self.snapshot.both_blocked_cycles.saturating_add(cycles);
        if self.open_blocked_length == 0 {
            self.snapshot
                .initial_horizon_distances
                .record(horizon_distance);
        }
        self.open_blocked_length = self.open_blocked_length.saturating_add(cycles);
        self.open_blockers = self.open_blockers.union(blockers);

        if core0_halted {
            self.snapshot.core0_halted_blocked_cycles = self
                .snapshot
                .core0_halted_blocked_cycles
                .saturating_add(cycles);
        }
        if core0_wfe {
            self.snapshot.core0_wfe_blocked_cycles = self
                .snapshot
                .core0_wfe_blocked_cycles
                .saturating_add(cycles);
        }
        if core1_halted {
            self.snapshot.core1_halted_blocked_cycles = self
                .snapshot
                .core1_halted_blocked_cycles
                .saturating_add(cycles);
        }
        if core1_wfe {
            self.snapshot.core1_wfe_blocked_cycles = self
                .snapshot
                .core1_wfe_blocked_cycles
                .saturating_add(cycles);
        }

        if blockers.is_empty() {
            self.snapshot.proven_safe_cycles =
                self.snapshot.proven_safe_cycles.saturating_add(cycles);
            self.open_safe_length = self.open_safe_length.saturating_add(cycles);
        } else {
            self.close_safe_episode();
            Self::add_blocker_cycles(&mut self.snapshot.blockers, blockers, cycles);
        }
    }

    pub(crate) fn record_zero_progress_blocked(&mut self) {
        self.close_blocked_episode();
        self.snapshot.step_calls = self.snapshot.step_calls.saturating_add(1);
        self.snapshot.zero_progress_blocked_steps =
            self.snapshot.zero_progress_blocked_steps.saturating_add(1);
    }

    pub(crate) fn snapshot(&self) -> IdleProfileSnapshot {
        let mut copy = self.clone();
        copy.close_blocked_episode();
        copy.snapshot
    }

    fn close_safe_episode(&mut self) {
        self.snapshot
            .proven_safe_lengths
            .record(self.open_safe_length);
        self.open_safe_length = 0;
    }

    fn close_blocked_episode(&mut self) {
        self.close_safe_episode();
        self.snapshot
            .blocked_lengths
            .record(self.open_blocked_length);
        Self::add_blocker_episodes(&mut self.snapshot.blocker_episodes, self.open_blockers);
        self.open_blocked_length = 0;
        self.open_blockers = IdleBlockerMask::default();
    }

    fn add_blocker_cycles(dst: &mut IdleBlockerCycles, mask: IdleBlockerMask, cycles: u64) {
        let add = |bit, value: &mut u64| {
            if mask.contains(bit) {
                *value = value.saturating_add(cycles);
            }
        };
        add(IdleBlockerMask::PIO, &mut dst.pio);
        add(IdleBlockerMask::DMA, &mut dst.dma);
        add(IdleBlockerMask::PWM, &mut dst.pwm);
        add(IdleBlockerMask::SYSTICK, &mut dst.systick);
        add(IdleBlockerMask::UART, &mut dst.uart);
        add(IdleBlockerMask::SPI, &mut dst.spi);
        add(IdleBlockerMask::I2C, &mut dst.i2c);
        add(IdleBlockerMask::ADC, &mut dst.adc);
        add(IdleBlockerMask::TIMER, &mut dst.timer);
        add(IdleBlockerMask::PENDING_IRQ, &mut dst.pending_irq);
    }

    fn add_blocker_episodes(dst: &mut IdleBlockerEpisodes, mask: IdleBlockerMask) {
        let add = |bit, value: &mut u64| {
            if mask.contains(bit) {
                *value = value.saturating_add(1);
            }
        };
        add(IdleBlockerMask::PIO, &mut dst.pio);
        add(IdleBlockerMask::DMA, &mut dst.dma);
        add(IdleBlockerMask::PWM, &mut dst.pwm);
        add(IdleBlockerMask::SYSTICK, &mut dst.systick);
        add(IdleBlockerMask::UART, &mut dst.uart);
        add(IdleBlockerMask::SPI, &mut dst.spi);
        add(IdleBlockerMask::I2C, &mut dst.i2c);
        add(IdleBlockerMask::ADC, &mut dst.adc);
        add(IdleBlockerMask::TIMER, &mut dst.timer);
        add(IdleBlockerMask::PENDING_IRQ, &mut dst.pending_irq);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cumulative_histogram_records_episode_count_and_cycle_mass() {
        let mut h = CumulativeHistogramSnapshot::default();
        h.record(10);
        h.record(4);
        assert_eq!(h.episodes_ge[0], 2);
        assert_eq!(h.cycle_mass_ge[0], 14);
        assert_eq!(h.episodes_ge[2], 2);
        assert_eq!(h.cycle_mass_ge[2], 14);
        assert_eq!(h.episodes_ge[3], 1);
        assert_eq!(h.cycle_mass_ge[3], 10);
        assert_eq!(h.episodes_ge[4], 0);
    }

    #[test]
    fn unsafe_cycles_close_only_the_safe_sub_episode() {
        let mut p = IdleProfiler::default();
        p.record_blocked(4, 20, IdleBlockerMask::default(), false, true, true, false);
        p.record_blocked(
            2,
            16,
            IdleBlockerMask::from_bits(IdleBlockerMask::PWM),
            false,
            true,
            true,
            false,
        );
        p.record_blocked(3, 14, IdleBlockerMask::default(), false, true, true, false);
        let s = p.snapshot();
        assert_eq!(s.both_blocked_cycles, 9);
        assert_eq!(s.proven_safe_cycles, 7);
        assert_eq!(s.blocked_lengths.episodes_ge[0], 1);
        assert_eq!(s.blocked_lengths.cycle_mass_ge[0], 9);
        assert_eq!(s.proven_safe_lengths.episodes_ge[0], 2);
        assert_eq!(s.proven_safe_lengths.cycle_mass_ge[0], 7);
        assert_eq!(s.blockers.pwm, 2);
        assert_eq!(s.initial_horizon_distances.episodes_ge[4], 1);
    }
}
