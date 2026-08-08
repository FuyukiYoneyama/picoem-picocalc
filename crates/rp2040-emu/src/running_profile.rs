//! OPT2-B running-path profiler profile data layer.
//!
//! Snapshot fields are aggregated from observed *running* intervals and
//! recorded boundaries in serial execution. These are observed boundary gaps,
//! not guaranteed safe windows.

use crate::idle_profile::{
    CumulativeHistogramSnapshot, IdleEventHorizonProbe, IdleEventSourceMask, IdleHorizonEvents,
};

pub const RUNNING_EVENT_PROFILE_SCHEMA_VERSION: u32 = 1;

/// Bitset describing what terminated the current observed running interval.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RunningBoundaryMask(u16);

impl RunningBoundaryMask {
    pub const CPU_MMIO: u16 = 1 << 0;
    pub const GPIO_IN: u16 = 1 << 1;
    pub const FIFO_DREQ: u16 = 1 << 2;
    pub const IRQ_EXCEPTION: u16 = 1 << 3;
    pub const PIO_DEVICE: u16 = 1 << 4;
    pub const DMA_DREQ: u16 = 1 << 5;
    pub const TIMER_SYSTICK_PWM: u16 = 1 << 6;
    pub const SERIAL: u16 = 1 << 7;
    pub const CLOCK: u16 = 1 << 8;
    pub const EXTERNAL: u16 = 1 << 9;

    pub const fn contains(self, bit: u16) -> bool {
        self.0 & bit != 0
    }

    pub const fn bits(self) -> u16 {
        self.0
    }

    #[cfg(test)]
    pub(crate) const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    pub(crate) fn insert(&mut self, bit: u16) {
        self.0 |= bit;
    }
}

/// Publicly exported running-profile snapshot.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RunningBoundarySnapshot {
    /// Number of `record_running` calls while profiling.
    pub running_steps: u64,
    /// Total running cycles accumulated while profiling.
    pub total_running_cycles: u64,
    /// Number of observed interval boundaries that were closed.
    pub boundary_steps: u64,
    /// Running records for which the current model supplied no autonomous
    /// or external horizon at all.
    pub no_known_horizon_steps: u64,
    pub no_known_horizon_cycles: u64,
    /// Dispatches and cycles in post-hoc candidate intervals. These totals
    /// make the batching arithmetic explicit; the histograms below describe
    /// how the same mass is distributed by interval length.
    pub candidate_dispatches: u64,
    pub candidate_cycles: u64,
    /// Number of dispatcher calls between observed boundaries.
    pub observed_inter_boundary_dispatches: CumulativeHistogramSnapshot,
    /// Cycles between observed boundaries.
    ///
    /// These values describe measured running intervals seen during profiling,
    /// not guaranteed-safe windows.
    pub observed_inter_boundary_cycles: CumulativeHistogramSnapshot,
    /// Post-hoc intervals which had no observed boundary and whose
    /// conservative pre-dispatch horizon lay beyond the completed dispatch.
    /// These remain opportunity estimates, not predictive safe windows.
    pub observed_candidate_dispatches: CumulativeHistogramSnapshot,
    pub observed_candidate_cycles: CumulativeHistogramSnapshot,
    /// Conservative horizon distances observed for each running record.
    pub conservative_horizon_distances: CumulativeHistogramSnapshot,
    /// Counts of observed boundary bits.
    pub boundary_events: RunningBoundaryEvents,
    /// Cycles charged to each source when `horizon.distance_cycles == Some(1)`.
    pub one_cycle_fallback_cycles: IdleHorizonEvents,
}

/// Overlapping boundary counts by observed cause.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RunningBoundaryEvents {
    pub cpu_mmio: u64,
    pub gpio_in: u64,
    pub fifo_dreq: u64,
    pub irq_exception: u64,
    pub pio_device: u64,
    pub dma_dreq: u64,
    pub timer_systick_pwm: u64,
    pub serial: u64,
    pub clock: u64,
    pub external: u64,
}

/// Feature-gated profiler that records observed boundaries during running time.
#[derive(Clone, Debug, Default)]
pub struct RunningProfile {
    state: RunningProfileState,
}

#[derive(Clone, Debug, Default)]
struct RunningProfileState {
    running_steps: u64,
    total_running_cycles: u64,
    boundary_steps: u64,
    no_known_horizon_steps: u64,
    no_known_horizon_cycles: u64,
    candidate_dispatches: u64,
    candidate_cycles: u64,
    open_inter_boundary_dispatches: u64,
    open_inter_boundary_cycles: u64,
    open_candidate_dispatches: u64,
    open_candidate_cycles: u64,
    observed_inter_boundary_dispatches: CumulativeHistogramSnapshot,
    observed_inter_boundary_cycles: CumulativeHistogramSnapshot,
    observed_candidate_dispatches: CumulativeHistogramSnapshot,
    observed_candidate_cycles: CumulativeHistogramSnapshot,
    conservative_horizon_distances: CumulativeHistogramSnapshot,
    boundary_events: RunningBoundaryEvents,
    one_cycle_fallback_cycles: IdleHorizonEvents,
}

impl RunningProfile {
    /// Record an observed run of `cycles` and optionally close at an observed
    /// boundary. This records boundaries as observed inter-interval measurements,
    /// not as guaranteed safe windows.
    pub fn record_running(
        &mut self,
        cycles: u64,
        boundary_mask: RunningBoundaryMask,
        horizon_probe: IdleEventHorizonProbe,
    ) {
        if cycles == 0 {
            return;
        }

        self.state.running_steps = self.state.running_steps.saturating_add(1);
        self.state.total_running_cycles = self.state.total_running_cycles.saturating_add(cycles);
        self.state.open_inter_boundary_dispatches =
            self.state.open_inter_boundary_dispatches.saturating_add(1);
        self.state.open_inter_boundary_cycles =
            self.state.open_inter_boundary_cycles.saturating_add(cycles);

        if let Some(distance) = horizon_probe.distance_cycles {
            self.state
                .conservative_horizon_distances
                .record_weighted(distance, cycles);
            if distance == 1 {
                self.accumulate_one_cycle_fallback(
                    cycles,
                    horizon_probe.one_cycle_fallback_sources,
                );
            }
        } else {
            self.state.no_known_horizon_steps = self.state.no_known_horizon_steps.saturating_add(1);
            self.state.no_known_horizon_cycles =
                self.state.no_known_horizon_cycles.saturating_add(cycles);
        }

        let boundary = boundary_mask.bits() != 0;
        let candidate = !boundary
            && horizon_probe
                .distance_cycles
                .is_some_and(|distance| distance > cycles);
        if candidate {
            self.state.candidate_dispatches = self.state.candidate_dispatches.saturating_add(1);
            self.state.candidate_cycles = self.state.candidate_cycles.saturating_add(cycles);
            self.state.open_candidate_dispatches =
                self.state.open_candidate_dispatches.saturating_add(1);
            self.state.open_candidate_cycles =
                self.state.open_candidate_cycles.saturating_add(cycles);
        } else {
            self.close_candidate();
        }

        if boundary {
            self.close_inter_boundary();
            self.state.boundary_steps = self.state.boundary_steps.saturating_add(1);
            self.accumulate_boundary_events(boundary_mask);
        }
    }

    /// Record a non-running region and close any currently-open interval.
    pub fn record_non_running(&mut self) {
        self.close_inter_boundary();
        self.close_candidate();
    }

    /// Flush the profiler state into a snapshot.
    pub fn snapshot(&self) -> RunningBoundarySnapshot {
        let mut copy = self.clone();
        copy.close_inter_boundary();
        copy.close_candidate();

        RunningBoundarySnapshot {
            running_steps: copy.state.running_steps,
            total_running_cycles: copy.state.total_running_cycles,
            boundary_steps: copy.state.boundary_steps,
            no_known_horizon_steps: copy.state.no_known_horizon_steps,
            no_known_horizon_cycles: copy.state.no_known_horizon_cycles,
            candidate_dispatches: copy.state.candidate_dispatches,
            candidate_cycles: copy.state.candidate_cycles,
            observed_inter_boundary_dispatches: copy.state.observed_inter_boundary_dispatches,
            observed_inter_boundary_cycles: copy.state.observed_inter_boundary_cycles,
            observed_candidate_dispatches: copy.state.observed_candidate_dispatches,
            observed_candidate_cycles: copy.state.observed_candidate_cycles,
            conservative_horizon_distances: copy.state.conservative_horizon_distances,
            boundary_events: copy.state.boundary_events,
            one_cycle_fallback_cycles: copy.state.one_cycle_fallback_cycles,
        }
    }

    fn close_inter_boundary(&mut self) {
        if self.state.open_inter_boundary_dispatches == 0 {
            return;
        }
        self.state
            .observed_inter_boundary_dispatches
            .record_weighted(
                self.state.open_inter_boundary_dispatches,
                self.state.open_inter_boundary_cycles,
            );
        self.state
            .observed_inter_boundary_cycles
            .record(self.state.open_inter_boundary_cycles);
        self.state.open_inter_boundary_dispatches = 0;
        self.state.open_inter_boundary_cycles = 0;
    }

    fn close_candidate(&mut self) {
        if self.state.open_candidate_dispatches == 0 {
            return;
        }
        self.state.observed_candidate_dispatches.record_weighted(
            self.state.open_candidate_dispatches,
            self.state.open_candidate_cycles,
        );
        self.state
            .observed_candidate_cycles
            .record(self.state.open_candidate_cycles);
        self.state.open_candidate_dispatches = 0;
        self.state.open_candidate_cycles = 0;
    }

    fn accumulate_boundary_events(&mut self, mask: RunningBoundaryMask) {
        let bits = mask.bits();
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::CPU_MMIO) {
            self.state.boundary_events.cpu_mmio =
                self.state.boundary_events.cpu_mmio.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::GPIO_IN) {
            self.state.boundary_events.gpio_in =
                self.state.boundary_events.gpio_in.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::FIFO_DREQ) {
            self.state.boundary_events.fifo_dreq =
                self.state.boundary_events.fifo_dreq.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::IRQ_EXCEPTION) {
            self.state.boundary_events.irq_exception =
                self.state.boundary_events.irq_exception.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::PIO_DEVICE) {
            self.state.boundary_events.pio_device =
                self.state.boundary_events.pio_device.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::DMA_DREQ) {
            self.state.boundary_events.dma_dreq =
                self.state.boundary_events.dma_dreq.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::TIMER_SYSTICK_PWM) {
            self.state.boundary_events.timer_systick_pwm = self
                .state
                .boundary_events
                .timer_systick_pwm
                .saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::SERIAL) {
            self.state.boundary_events.serial = self.state.boundary_events.serial.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::CLOCK) {
            self.state.boundary_events.clock = self.state.boundary_events.clock.saturating_add(1);
        }
        if RunningBoundaryMask(bits).contains(RunningBoundaryMask::EXTERNAL) {
            self.state.boundary_events.external =
                self.state.boundary_events.external.saturating_add(1);
        }
    }

    fn accumulate_one_cycle_fallback(&mut self, cycles: u64, fallback_mask: IdleEventSourceMask) {
        if fallback_mask.contains(IdleEventSourceMask::PIO) {
            self.state.one_cycle_fallback_cycles.pio = self
                .state
                .one_cycle_fallback_cycles
                .pio
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::DMA) {
            self.state.one_cycle_fallback_cycles.dma = self
                .state
                .one_cycle_fallback_cycles
                .dma
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::PWM) {
            self.state.one_cycle_fallback_cycles.pwm = self
                .state
                .one_cycle_fallback_cycles
                .pwm
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::SYSTICK) {
            self.state.one_cycle_fallback_cycles.systick = self
                .state
                .one_cycle_fallback_cycles
                .systick
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::UART) {
            self.state.one_cycle_fallback_cycles.uart = self
                .state
                .one_cycle_fallback_cycles
                .uart
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::SPI) {
            self.state.one_cycle_fallback_cycles.spi = self
                .state
                .one_cycle_fallback_cycles
                .spi
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::I2C) {
            self.state.one_cycle_fallback_cycles.i2c = self
                .state
                .one_cycle_fallback_cycles
                .i2c
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::ADC) {
            self.state.one_cycle_fallback_cycles.adc = self
                .state
                .one_cycle_fallback_cycles
                .adc
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::TIMER) {
            self.state.one_cycle_fallback_cycles.timer = self
                .state
                .one_cycle_fallback_cycles
                .timer
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::PENDING_IRQ) {
            self.state.one_cycle_fallback_cycles.pending_irq = self
                .state
                .one_cycle_fallback_cycles
                .pending_irq
                .saturating_add(cycles);
        }
        if fallback_mask.contains(IdleEventSourceMask::EXTERNAL) {
            self.state.one_cycle_fallback_cycles.external = self
                .state
                .one_cycle_fallback_cycles
                .external
                .saturating_add(cycles);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::idle_profile::IdleEventSourceMask;

    fn horizon(distance_cycles: u64, source: u16) -> IdleEventHorizonProbe {
        IdleEventHorizonProbe {
            next_event_cycle: Some(distance_cycles),
            distance_cycles: Some(distance_cycles),
            limiting_sources: IdleEventSourceMask::from_bits(source),
            one_cycle_fallback_sources: IdleEventSourceMask::from_bits(source),
            complete_for_current_model: true,
            ..IdleEventHorizonProbe::default()
        }
    }

    #[test]
    fn no_boundary_accumulates_running_interval() {
        let mut p = RunningProfile::default();
        p.record_running(
            2,
            RunningBoundaryMask::from_bits(0),
            horizon(4, IdleEventSourceMask::from_bits(0).bits()),
        );
        p.record_running(
            3,
            RunningBoundaryMask::from_bits(0),
            horizon(5, IdleEventSourceMask::from_bits(0).bits()),
        );

        let snap = p.snapshot();
        assert_eq!(snap.running_steps, 2);
        assert_eq!(snap.total_running_cycles, 5);
        assert_eq!(snap.boundary_steps, 0);
        assert_eq!(snap.candidate_dispatches, 2);
        assert_eq!(snap.candidate_cycles, 5);
        assert_eq!(snap.observed_inter_boundary_dispatches.episodes_ge[1], 1);
        assert_eq!(snap.observed_inter_boundary_dispatches.cycle_mass_ge[1], 5);
        assert_eq!(snap.observed_inter_boundary_cycles.episodes_ge[2], 1);
        assert_eq!(snap.observed_inter_boundary_cycles.cycle_mass_ge[2], 5);
        assert_eq!(snap.observed_candidate_dispatches.episodes_ge[1], 1);
        assert_eq!(snap.observed_candidate_dispatches.cycle_mass_ge[1], 5);
        assert_eq!(snap.observed_candidate_cycles.cycle_mass_ge[2], 5);
    }

    #[test]
    fn overlapping_boundary_counts() {
        let mut p = RunningProfile::default();
        let bmask = RunningBoundaryMask::from_bits(
            RunningBoundaryMask::CPU_MMIO
                | RunningBoundaryMask::SERIAL
                | RunningBoundaryMask::EXTERNAL,
        );
        p.record_running(
            1,
            bmask,
            horizon(8, IdleEventSourceMask::from_bits(0).bits()),
        );
        p.record_running(
            1,
            bmask,
            horizon(1, IdleEventSourceMask::from_bits(0).bits()),
        );

        let snap = p.snapshot();
        assert_eq!(snap.boundary_events.cpu_mmio, 2);
        assert_eq!(snap.boundary_events.serial, 2);
        assert_eq!(snap.boundary_events.external, 2);
        assert_eq!(snap.boundary_steps, 2);
    }

    #[test]
    fn horizon_histogram_and_fallback_overlap() {
        let mut p = RunningProfile::default();
        let fallback =
            IdleEventSourceMask::from_bits(IdleEventSourceMask::PIO | IdleEventSourceMask::UART);
        let horizon = horizon(1, fallback.bits());
        p.record_running(4, RunningBoundaryMask::from_bits(0), horizon);

        let snap = p.snapshot();
        assert_eq!(snap.conservative_horizon_distances.episodes_ge[0], 1);
        assert_eq!(snap.conservative_horizon_distances.cycle_mass_ge[0], 4);
        assert_eq!(snap.one_cycle_fallback_cycles.pio, 4);
        assert_eq!(snap.one_cycle_fallback_cycles.uart, 4);

        assert_eq!(snap.observed_inter_boundary_cycles.episodes_ge[0], 1);
        assert_eq!(snap.observed_inter_boundary_cycles.cycle_mass_ge[0], 4);
        assert_eq!(snap.observed_candidate_cycles.episodes_ge[0], 0);
    }

    #[test]
    fn snapshot_closes_without_mutating_state() {
        let mut p = RunningProfile::default();
        p.record_running(
            7,
            RunningBoundaryMask::from_bits(0),
            horizon(2, IdleEventSourceMask::from_bits(0).bits()),
        );

        let snap = p.snapshot();
        assert_eq!(snap.total_running_cycles, 7);
        assert_eq!(snap.observed_inter_boundary_cycles.episodes_ge[0], 1);
        assert_eq!(snap.observed_inter_boundary_cycles.cycle_mass_ge[0], 7);

        let second = p.snapshot();
        assert_eq!(second.total_running_cycles, 7);
        assert_eq!(second.observed_inter_boundary_cycles.episodes_ge[0], 1);
        assert_eq!(second.observed_inter_boundary_cycles.cycle_mass_ge[0], 7);
    }
}
