//! OPT0-A Serial idle-path profiler.
//!
//! This module is compiled only with the `idle-profiler` feature. The
//! normal correctness/performance build therefore pays no field, branch,
//! counter, or histogram cost.

/// Machine-readable profile schema emitted by diagnostic harnesses.
pub const IDLE_PROFILE_SCHEMA_VERSION: u32 = 3;

/// Schema for the complete, conservative all-source event-horizon probe.
pub const IDLE_HORIZON_SCHEMA_VERSION: u32 = 1;

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

/// Overlapping cycle totals for peripheral-source classifications.
///
/// The containing [`IdleProfileSnapshot`] field determines whether the
/// source is a blocker, stationary state, or exact-bulk work.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IdleBlockerCycles {
    /// PIO source classification.
    pub pio: u64,
    /// DMA source classification.
    pub dma: u64,
    /// PWM source classification.
    pub pwm: u64,
    /// Core-local SysTick source classification.
    pub systick: u64,
    /// UART source classification.
    pub uart: u64,
    /// SPI source classification.
    pub spi: u64,
    /// I2C source classification.
    pub i2c: u64,
    /// ADC source classification.
    pub adc: u64,
    /// TIMER source classification.
    pub timer: u64,
    /// Bus or enabled-NVIC pending source classification.
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

/// Shared profiler-only view returned by stateful peripheral models.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct IdlePeripheralState {
    /// Internal state changes as emulated time advances.
    pub(crate) temporal_work: bool,
    /// A currently asserted and enabled source must be routed before a jump.
    pub(crate) routable_irq: bool,
    /// Observable state exists but does not change while CPUs are stopped.
    pub(crate) static_state: bool,
}

/// PWM adds one category: work already supported by its exact O(1) bulk tick.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct IdlePwmState {
    pub(crate) exact_bulk_work: bool,
    pub(crate) temporal_boundary: bool,
    pub(crate) routable_irq: bool,
    pub(crate) static_state: bool,
}

/// One observation of the semantic idle classification used by OPT0-A.
///
/// This is deliberately named a "current probe", not a complete event
/// horizon. At schema 2 TIMER remains the only scheduled lazy deadline;
/// active sources that need a horizon are blockers, while static latches and
/// already-exact bulk work are reported separately.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IdleCurrentProbe {
    /// Current emulated master cycle.
    pub master_cycle: u64,
    /// Soonest deadline among lazy scheduled sources modelled today.
    pub next_lazy_deadline: Option<u64>,
    /// Number of overlapping conservative blockers observed.
    pub blocker_count: u32,
    /// Number of sources with observable but time-invariant state.
    pub stationary_source_count: u32,
    /// Number of active sources whose existing tick is already exact in bulk.
    pub exact_bulk_source_count: u32,
    /// True when no source requires per-cycle work or an unresolved horizon.
    pub proven_jump_safe: bool,
}

/// Source bits used by [`IdleEventHorizonProbe`].
///
/// A source may conservatively limit the horizon to one cycle even when a
/// more distant exact deadline could be derived later.  This keeps the probe
/// complete without claiming unsafe skip distance.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IdleEventSourceMask(u16);

impl IdleEventSourceMask {
    pub const PIO: u16 = 1 << 0;
    pub const DMA: u16 = 1 << 1;
    pub const PWM: u16 = 1 << 2;
    pub const SYSTICK: u16 = 1 << 3;
    pub const UART: u16 = 1 << 4;
    pub const SPI: u16 = 1 << 5;
    pub const I2C: u16 = 1 << 6;
    pub const ADC: u16 = 1 << 7;
    pub const TIMER: u16 = 1 << 8;
    pub const PENDING_IRQ: u16 = 1 << 9;
    pub const EXTERNAL: u16 = 1 << 10;

    pub(crate) const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, bit: u16) -> bool {
        self.0 & bit != 0
    }
}

/// Read-only all-source event horizon for a both-cores-blocked interval.
///
/// `distance_cycles == 0` denotes work already pending at the current
/// boundary. `None` means no autonomous or caller-supplied event is known.
/// Every temporal source is represented: sources without a promoted exact
/// deadline conservatively contribute a one-cycle horizon.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IdleEventHorizonProbe {
    pub master_cycle: u64,
    pub next_event_cycle: Option<u64>,
    pub distance_cycles: Option<u64>,
    pub limiting_sources: IdleEventSourceMask,
    pub one_cycle_fallback_sources: IdleEventSourceMask,
    pub complete_for_current_model: bool,
}

/// Event-boundary counts attributed to the source(s) that limited a safe
/// interval. Simultaneous sources intentionally overlap.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IdleHorizonEvents {
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
    pub external: u64,
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
    /// Safe lengths split at every complete all-source event horizon.
    /// This is the actionable `S(K)` for an exact fast-forward candidate.
    pub event_bounded_safe_lengths: CumulativeHistogramSnapshot,
    /// Number of closed safe boundaries attributed to each limiting source.
    pub horizon_boundary_events: IdleHorizonEvents,
    /// Initial timer-horizon distance distribution for blocked episodes.
    pub initial_horizon_distances: CumulativeHistogramSnapshot,
    /// Overlapping per-source reasons why blocked cycles were not proven safe.
    pub blockers: IdleBlockerCycles,
    /// Overlapping episode counts for the same blocker sources.
    pub blocker_episodes: IdleBlockerEpisodes,
    /// Overlapping source state that remains unchanged throughout the interval.
    pub stationary_sources: IdleBlockerCycles,
    /// Episode counts for stationary source state.
    pub stationary_source_episodes: IdleBlockerEpisodes,
    /// Active work handled exactly by an existing O(1) bulk tick.
    pub exact_bulk_sources: IdleBlockerCycles,
    /// Episode counts for exact-bulk source work.
    pub exact_bulk_source_episodes: IdleBlockerEpisodes,
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

    pub(crate) fn contains(self, bit: u16) -> bool {
        self.0 & bit != 0
    }

    pub(crate) fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub(crate) fn count(self) -> u32 {
        self.0.count_ones()
    }
}

/// Mutually meaningful (but source-overlapping) classification at one cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct IdleSourceObservation {
    pub(crate) blockers: IdleBlockerMask,
    pub(crate) stationary: IdleBlockerMask,
    pub(crate) exact_bulk: IdleBlockerMask,
}

impl IdleSourceObservation {
    pub(crate) fn proven_safe(self) -> bool {
        self.blockers.is_empty()
    }
}

/// Mutable profiler state owned by an `Emulator` in diagnostic builds.
#[derive(Clone, Debug, Default)]
pub(crate) struct IdleProfiler {
    snapshot: IdleProfileSnapshot,
    open_blocked_length: u64,
    open_safe_length: u64,
    open_event_bounded_safe_length: u64,
    open_blockers: IdleBlockerMask,
    open_stationary: IdleBlockerMask,
    open_exact_bulk: IdleBlockerMask,
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
        observation: IdleSourceObservation,
        event_horizon: IdleEventHorizonProbe,
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
        self.open_blockers = self.open_blockers.union(observation.blockers);
        self.open_stationary = self.open_stationary.union(observation.stationary);
        self.open_exact_bulk = self.open_exact_bulk.union(observation.exact_bulk);

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

        if observation.proven_safe() {
            self.snapshot.proven_safe_cycles =
                self.snapshot.proven_safe_cycles.saturating_add(cycles);
            self.open_safe_length = self.open_safe_length.saturating_add(cycles);
            self.open_event_bounded_safe_length =
                self.open_event_bounded_safe_length.saturating_add(cycles);
            if event_horizon
                .distance_cycles
                .is_some_and(|distance| distance <= cycles)
            {
                self.close_event_bounded_safe_episode();
                Self::add_horizon_events(
                    &mut self.snapshot.horizon_boundary_events,
                    event_horizon.limiting_sources,
                );
            }
        } else {
            self.close_safe_episode();
            self.close_event_bounded_safe_episode();
            Self::add_source_cycles(&mut self.snapshot.blockers, observation.blockers, cycles);
        }
        Self::add_source_cycles(
            &mut self.snapshot.stationary_sources,
            observation.stationary,
            cycles,
        );
        Self::add_source_cycles(
            &mut self.snapshot.exact_bulk_sources,
            observation.exact_bulk,
            cycles,
        );
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

    fn close_event_bounded_safe_episode(&mut self) {
        self.snapshot
            .event_bounded_safe_lengths
            .record(self.open_event_bounded_safe_length);
        self.open_event_bounded_safe_length = 0;
    }

    fn close_blocked_episode(&mut self) {
        self.close_safe_episode();
        self.close_event_bounded_safe_episode();
        self.snapshot
            .blocked_lengths
            .record(self.open_blocked_length);
        Self::add_source_episodes(&mut self.snapshot.blocker_episodes, self.open_blockers);
        Self::add_source_episodes(
            &mut self.snapshot.stationary_source_episodes,
            self.open_stationary,
        );
        Self::add_source_episodes(
            &mut self.snapshot.exact_bulk_source_episodes,
            self.open_exact_bulk,
        );
        self.open_blocked_length = 0;
        self.open_blockers = IdleBlockerMask::default();
        self.open_stationary = IdleBlockerMask::default();
        self.open_exact_bulk = IdleBlockerMask::default();
    }

    fn add_source_cycles(dst: &mut IdleBlockerCycles, mask: IdleBlockerMask, cycles: u64) {
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

    fn add_source_episodes(dst: &mut IdleBlockerEpisodes, mask: IdleBlockerMask) {
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

    fn add_horizon_events(dst: &mut IdleHorizonEvents, mask: IdleEventSourceMask) {
        let add = |bit, value: &mut u64| {
            if mask.contains(bit) {
                *value = value.saturating_add(1);
            }
        };
        add(IdleEventSourceMask::PIO, &mut dst.pio);
        add(IdleEventSourceMask::DMA, &mut dst.dma);
        add(IdleEventSourceMask::PWM, &mut dst.pwm);
        add(IdleEventSourceMask::SYSTICK, &mut dst.systick);
        add(IdleEventSourceMask::UART, &mut dst.uart);
        add(IdleEventSourceMask::SPI, &mut dst.spi);
        add(IdleEventSourceMask::I2C, &mut dst.i2c);
        add(IdleEventSourceMask::ADC, &mut dst.adc);
        add(IdleEventSourceMask::TIMER, &mut dst.timer);
        add(IdleEventSourceMask::PENDING_IRQ, &mut dst.pending_irq);
        add(IdleEventSourceMask::EXTERNAL, &mut dst.external);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn horizon(distance_cycles: u64, source: u16) -> IdleEventHorizonProbe {
        IdleEventHorizonProbe {
            next_event_cycle: Some(distance_cycles),
            distance_cycles: Some(distance_cycles),
            limiting_sources: IdleEventSourceMask::from_bits(source),
            complete_for_current_model: true,
            ..IdleEventHorizonProbe::default()
        }
    }

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
        p.record_blocked(
            4,
            20,
            IdleSourceObservation::default(),
            horizon(20, IdleEventSourceMask::TIMER),
            false,
            true,
            true,
            false,
        );
        p.record_blocked(
            2,
            16,
            IdleSourceObservation {
                blockers: IdleBlockerMask::from_bits(IdleBlockerMask::PWM),
                ..IdleSourceObservation::default()
            },
            horizon(1, IdleEventSourceMask::PWM),
            false,
            true,
            true,
            false,
        );
        p.record_blocked(
            3,
            14,
            IdleSourceObservation::default(),
            horizon(14, IdleEventSourceMask::TIMER),
            false,
            true,
            true,
            false,
        );
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

    #[test]
    fn complete_horizon_splits_safe_mass_at_exact_boundaries() {
        let mut p = IdleProfiler::default();
        for distance in [3, 2, 1, 4, 3, 2, 1] {
            p.record_blocked(
                1,
                100,
                IdleSourceObservation::default(),
                horizon(distance, IdleEventSourceMask::PWM),
                false,
                true,
                true,
                false,
            );
        }
        let s = p.snapshot();
        assert_eq!(s.proven_safe_cycles, 7);
        assert_eq!(s.event_bounded_safe_lengths.episodes_ge[0], 2);
        assert_eq!(s.event_bounded_safe_lengths.cycle_mass_ge[0], 7);
        assert_eq!(s.event_bounded_safe_lengths.episodes_ge[2], 1);
        assert_eq!(s.event_bounded_safe_lengths.cycle_mass_ge[2], 4);
        assert_eq!(s.horizon_boundary_events.pwm, 2);
    }
}
