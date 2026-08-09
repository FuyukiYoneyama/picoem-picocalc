//! Running-path horizon profiler, extended by OPT3-A immutable-XIP decode
//! cursor opportunity metrics.
//!
//! Snapshot fields are aggregated from observed *running* intervals and
//! recorded boundaries in serial execution. These are observed boundary gaps,
//! not guaranteed safe windows.

use crate::idle_profile::{
    CumulativeHistogramSnapshot, IdleEventHorizonProbe, IdleEventSourceMask, IdleHorizonEvents,
};

pub const RUNNING_EVENT_PROFILE_SCHEMA_VERSION: u32 = 3;
pub const ONE_CYCLE_FALLBACK_SIGNATURE_BUCKETS: usize = 16;
const XIP_SRAM_BASE: u32 = 0x1500_0000;
const XIP_SRAM_END: u32 = 0x1500_4000;
const XIP_IMMUTABLE_BASE: u32 = 0x1000_0000;
const XIP_IMMUTABLE_END: u32 = 0x1400_0000;
const INVALIDATION_REGION_ROM: u8 = 1 << 0;
const INVALIDATION_REGION_XIP: u8 = 1 << 1;
const INVALIDATION_REGION_SRAM: u8 = 1 << 2;
const INVALIDATION_REGION_BULK: u8 = 1 << 7;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum DecodeLookupRegion {
    #[default]
    Rom,
    ImmutableXip,
    XipSram,
    Sram,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImmutableXipHitRunTerminationReason {
    PostExecuteNextPcRedirect,
    XipMiss,
    RegionExit,
    PrefetchException,
    Fault,
}

/// Signature buckets for one-cycle fallback-source overlap recording.
///
/// Bit `0` is PIO, `1` is UART, `2` is DMA, and `3` is "any other"
/// source that appears in [`IdleEventSourceMask`].
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct OneCycleFallbackSignatureHistogram {
    /// Number of one-cycle fallback events by source signature.
    pub steps: [u64; ONE_CYCLE_FALLBACK_SIGNATURE_BUCKETS],
    /// Total cycles charged to those one-cycle fallback events by signature.
    pub cycle_mass: [u64; ONE_CYCLE_FALLBACK_SIGNATURE_BUCKETS],
}

/// Counts of decode-cache lookup hits and misses by executable region.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DecodeLookupRegionCounters {
    pub rom: u64,
    pub immutable_xip_flash_aliases: u64,
    pub xip_sram: u64,
    pub sram: u64,
    pub other: u64,
}

/// Counts of how immutable XIP hit-only run tracking terminates by cause.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ImmutableXipHitRunTerminationCounters {
    pub post_execute_next_pc_redirect: u64,
    pub xip_miss: u64,
    pub region_exit: u64,
    pub prefetch_exception: u64,
    pub fault: u64,
}

/// Decode-cache invalidation observations from region or entry tracking.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DecodeCacheInvalidationObservations {
    /// Number of invalidation addresses supplied to the per-entry API. This
    /// is not the number of direct-mapped slots that the API clears.
    pub entry_address_count: u64,
    pub rom: u64,
    pub xip: u64,
    pub sram: u64,
    pub bulk: u64,
    pub all: u64,
}

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
    /// One-cycle fallback overlap signatures keyed by `{PIO, UART, DMA, ANY_OTHER}`
    /// source bits.
    pub one_cycle_fallback_signatures: OneCycleFallbackSignatureHistogram,
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
    one_cycle_fallback_signatures: OneCycleFallbackSignatureHistogram,
}

/// Decode-cache lookup opportunities while stepping.
#[derive(Clone, Debug, Default)]
pub struct DecodeProfile {
    state: DecodeProfileState,
}

#[derive(Clone, Debug, Default)]
struct DecodeProfileState {
    cacheable_hits: u64,
    cacheable_misses: u64,
    noncacheable_fetches: u64,
    cacheable_hits_narrow: u64,
    cacheable_hits_wide: u64,
    lookup_hits_by_region: DecodeLookupRegionCounters,
    lookup_misses_by_region: DecodeLookupRegionCounters,
    sequential_cache_hit_runs: CumulativeHistogramSnapshot,
    immutable_xip_hit_runs: CumulativeHistogramSnapshot,
    open_immutable_xip_hit_run_instructions: u64,
    open_immutable_xip_hit_run_next_pc: Option<u32>,
    immutable_xip_hit_run_termination_counters: ImmutableXipHitRunTerminationCounters,
    open_hit_run_instructions: u64,
    open_hit_run_next_pc: Option<u32>,
    invalidation_observations: DecodeCacheInvalidationObservations,
}

/// Snapshot of decode-cache reuse behavior.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DecodeProfileSnapshot {
    pub cacheable_hits: u64,
    pub cacheable_misses: u64,
    pub noncacheable_fetches: u64,
    pub cacheable_hits_narrow: u64,
    pub cacheable_hits_wide: u64,
    pub lookup_hits_by_region: DecodeLookupRegionCounters,
    pub lookup_misses_by_region: DecodeLookupRegionCounters,
    pub sequential_cache_hit_runs: CumulativeHistogramSnapshot,
    pub immutable_xip_hit_runs: CumulativeHistogramSnapshot,
    pub immutable_xip_hit_run_termination_counters: ImmutableXipHitRunTerminationCounters,
    pub decode_cache_invalidation_observations: DecodeCacheInvalidationObservations,
}

/// Complete running-path and OPT3-A decode opportunity snapshot.
///
/// Peripheral fallback occupancy and CPU decode reuse are deliberately kept
/// side by side: both are diagnostic upper-bound inputs, not proofs that the
/// corresponding work can be skipped safely.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RunningEventProfileSnapshot {
    pub boundary: RunningBoundarySnapshot,
    pub decode_by_core: [DecodeProfileSnapshot; 2],
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
                if horizon_probe.one_cycle_fallback_sources.bits() != 0 {
                    self.accumulate_one_cycle_fallback_signatures(
                        cycles,
                        horizon_probe.one_cycle_fallback_sources,
                    );
                }
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
            one_cycle_fallback_signatures: copy.state.one_cycle_fallback_signatures,
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

    fn accumulate_one_cycle_fallback_signatures(
        &mut self,
        cycles: u64,
        sources: IdleEventSourceMask,
    ) {
        let mut signature = 0u8;
        if sources.contains(IdleEventSourceMask::PIO) {
            signature |= 1;
        }
        if sources.contains(IdleEventSourceMask::UART) {
            signature |= 1 << 1;
        }
        if sources.contains(IdleEventSourceMask::DMA) {
            signature |= 1 << 2;
        }

        let any_other = sources.bits()
            & !(IdleEventSourceMask::PIO | IdleEventSourceMask::UART | IdleEventSourceMask::DMA);
        if any_other != 0 {
            signature |= 1 << 3;
        }

        if signature == 0 {
            return;
        }

        let i = signature as usize;
        self.state.one_cycle_fallback_signatures.steps[i] =
            self.state.one_cycle_fallback_signatures.steps[i].saturating_add(1);
        self.state.one_cycle_fallback_signatures.cycle_mass[i] =
            self.state.one_cycle_fallback_signatures.cycle_mass[i].saturating_add(cycles);
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

impl DecodeProfile {
    /// Record a decode cache lookup.
    ///
    /// `entry_width_bytes` is 2 for narrow and 4 for wide lookups and is
    /// used to determine whether a hit continues a sequential run.
    pub fn record_decode_lookup(
        &mut self,
        pc: u32,
        entry_width_bytes: u32,
        cacheable: bool,
        hit: bool,
    ) {
        let region = Self::decode_lookup_region(pc);
        if cacheable && hit {
            self.state.cacheable_hits = self.state.cacheable_hits.saturating_add(1);
            self.increment_lookup_region_hit(region);
            if entry_width_bytes == 2 {
                self.state.cacheable_hits_narrow =
                    self.state.cacheable_hits_narrow.saturating_add(1);
            } else if entry_width_bytes == 4 {
                self.state.cacheable_hits_wide = self.state.cacheable_hits_wide.saturating_add(1);
            }

            if self.state.open_hit_run_instructions == 0 {
                self.state.open_hit_run_instructions = 1;
            } else if self.state.open_hit_run_next_pc == Some(pc) {
                self.state.open_hit_run_instructions =
                    self.state.open_hit_run_instructions.saturating_add(1);
            } else {
                self.close_hit_run();
                self.state.open_hit_run_instructions = 1;
            }
            self.state.open_hit_run_next_pc = Some(pc.wrapping_add(entry_width_bytes));

            self.track_immutable_xip_hit_run_on_lookup(pc, entry_width_bytes);
            return;
        }

        self.increment_lookup_region_miss(region);
        if cacheable {
            self.state.cacheable_misses = self.state.cacheable_misses.saturating_add(1);
            self.track_immutable_xip_hit_run_on_miss(region);
        } else {
            self.state.noncacheable_fetches = self.state.noncacheable_fetches.saturating_add(1);
            self.track_immutable_xip_hit_run_on_miss(DecodeLookupRegion::Other);
        }

        self.close_hit_run();
        self.state.open_hit_run_next_pc = None;
    }

    /// Record observation of an explicit per-entry decode-cache invalidation.
    ///
    /// The region counters follow the same split used in the lookup accounting:
    /// ROM, immutable XIP flash aliases, XIP SRAM, SRAM, and `other`.
    pub fn record_decode_cache_entry_invalidation(&mut self, addr: u32) {
        self.state.invalidation_observations.entry_address_count = self
            .state
            .invalidation_observations
            .entry_address_count
            .saturating_add(1);

        match Self::decode_lookup_region(addr) {
            DecodeLookupRegion::Rom => {
                self.state.invalidation_observations.rom =
                    self.state.invalidation_observations.rom.saturating_add(1)
            }
            DecodeLookupRegion::ImmutableXip | DecodeLookupRegion::XipSram => {
                self.state.invalidation_observations.xip =
                    self.state.invalidation_observations.xip.saturating_add(1)
            }
            DecodeLookupRegion::Sram => {
                self.state.invalidation_observations.sram =
                    self.state.invalidation_observations.sram.saturating_add(1)
            }
            DecodeLookupRegion::Other => {}
        }
    }

    /// Record a bulk or region-scoped invalidation.
    ///
    /// Region bits mirror the `bus::invalidation_regions` bit layout:
    /// `ROM|XIP|SRAM|BULK`. `BULK` increments only the bulk counter.
    pub fn record_decode_cache_region_invalidation(&mut self, region_bits: u8) {
        if region_bits & INVALIDATION_REGION_ROM != 0 {
            self.state.invalidation_observations.rom =
                self.state.invalidation_observations.rom.saturating_add(1);
        }
        if region_bits & INVALIDATION_REGION_XIP != 0 {
            self.state.invalidation_observations.xip =
                self.state.invalidation_observations.xip.saturating_add(1);
        }
        if region_bits & INVALIDATION_REGION_SRAM != 0 {
            self.state.invalidation_observations.sram =
                self.state.invalidation_observations.sram.saturating_add(1);
        }
        if region_bits & INVALIDATION_REGION_BULK != 0 {
            self.state.invalidation_observations.bulk =
                self.state.invalidation_observations.bulk.saturating_add(1);
        }
    }

    /// Record a full decode-cache invalidation observation.
    pub fn record_decode_cache_all_invalidation(&mut self) {
        self.state.invalidation_observations.all =
            self.state.invalidation_observations.all.saturating_add(1);
    }

    /// Manually close the current immutable-XIP hit run with an explicit cause.
    ///
    /// This is used when a non-sequential event is observed outside the
    /// lookup stream (for example, a prefetch exception or decode fault).
    pub fn record_immutable_xip_hit_run_termination(
        &mut self,
        reason: ImmutableXipHitRunTerminationReason,
    ) {
        self.close_immutable_xip_hit_run(Some(reason));
    }

    /// Convenience wrapper for prefetch-exception-driven termination.
    pub fn record_immutable_xip_hit_run_prefetch_exception(&mut self) {
        self.record_immutable_xip_hit_run_termination(
            ImmutableXipHitRunTerminationReason::PrefetchException,
        );
    }

    /// Convenience wrapper for fault-driven termination.
    pub fn record_immutable_xip_hit_run_fault(&mut self) {
        self.record_immutable_xip_hit_run_termination(ImmutableXipHitRunTerminationReason::Fault);
    }

    /// Flush decode-profile counters into a snapshot.
    ///
    /// This closes any open run in a clone so callers can snapshot at any
    /// time without mutating profiler state.
    pub fn snapshot(&self) -> DecodeProfileSnapshot {
        let mut copy = self.clone();
        copy.close_hit_run();
        copy.close_immutable_xip_hit_run(None);
        DecodeProfileSnapshot {
            cacheable_hits: copy.state.cacheable_hits,
            cacheable_misses: copy.state.cacheable_misses,
            noncacheable_fetches: copy.state.noncacheable_fetches,
            cacheable_hits_narrow: copy.state.cacheable_hits_narrow,
            cacheable_hits_wide: copy.state.cacheable_hits_wide,
            lookup_hits_by_region: copy.state.lookup_hits_by_region,
            lookup_misses_by_region: copy.state.lookup_misses_by_region,
            sequential_cache_hit_runs: copy.state.sequential_cache_hit_runs,
            immutable_xip_hit_runs: copy.state.immutable_xip_hit_runs,
            immutable_xip_hit_run_termination_counters: copy
                .state
                .immutable_xip_hit_run_termination_counters,
            decode_cache_invalidation_observations: copy.state.invalidation_observations,
        }
    }

    fn track_immutable_xip_hit_run_on_lookup(&mut self, pc: u32, entry_width_bytes: u32) {
        if Self::decode_lookup_region(pc) != DecodeLookupRegion::ImmutableXip {
            self.close_immutable_xip_hit_run(Some(ImmutableXipHitRunTerminationReason::RegionExit));
            return;
        }

        if self.state.open_immutable_xip_hit_run_instructions == 0 {
            self.state.open_immutable_xip_hit_run_instructions = 1;
            self.state.open_immutable_xip_hit_run_next_pc =
                Some(pc.wrapping_add(entry_width_bytes));
            return;
        }

        if self.state.open_immutable_xip_hit_run_next_pc == Some(pc) {
            self.state.open_immutable_xip_hit_run_instructions = self
                .state
                .open_immutable_xip_hit_run_instructions
                .saturating_add(1);
            self.state.open_immutable_xip_hit_run_next_pc =
                Some(pc.wrapping_add(entry_width_bytes));
            return;
        }

        self.close_immutable_xip_hit_run(Some(
            ImmutableXipHitRunTerminationReason::PostExecuteNextPcRedirect,
        ));
        self.state.open_immutable_xip_hit_run_instructions = 1;
        self.state.open_immutable_xip_hit_run_next_pc = Some(pc.wrapping_add(entry_width_bytes));
    }

    fn track_immutable_xip_hit_run_on_miss(&mut self, region: DecodeLookupRegion) {
        if self.state.open_immutable_xip_hit_run_instructions == 0 {
            return;
        }
        if region == DecodeLookupRegion::ImmutableXip {
            self.close_immutable_xip_hit_run(Some(ImmutableXipHitRunTerminationReason::XipMiss));
        } else {
            self.close_immutable_xip_hit_run(Some(ImmutableXipHitRunTerminationReason::RegionExit));
        }
    }

    fn increment_lookup_region_hit(&mut self, region: DecodeLookupRegion) {
        match region {
            DecodeLookupRegion::Rom => {
                self.state.lookup_hits_by_region.rom =
                    self.state.lookup_hits_by_region.rom.saturating_add(1)
            }
            DecodeLookupRegion::ImmutableXip => {
                self.state.lookup_hits_by_region.immutable_xip_flash_aliases = self
                    .state
                    .lookup_hits_by_region
                    .immutable_xip_flash_aliases
                    .saturating_add(1)
            }
            DecodeLookupRegion::XipSram => {
                self.state.lookup_hits_by_region.xip_sram =
                    self.state.lookup_hits_by_region.xip_sram.saturating_add(1)
            }
            DecodeLookupRegion::Sram => {
                self.state.lookup_hits_by_region.sram =
                    self.state.lookup_hits_by_region.sram.saturating_add(1)
            }
            DecodeLookupRegion::Other => {
                self.state.lookup_hits_by_region.other =
                    self.state.lookup_hits_by_region.other.saturating_add(1)
            }
        }
    }

    fn increment_lookup_region_miss(&mut self, region: DecodeLookupRegion) {
        match region {
            DecodeLookupRegion::Rom => {
                self.state.lookup_misses_by_region.rom =
                    self.state.lookup_misses_by_region.rom.saturating_add(1)
            }
            DecodeLookupRegion::ImmutableXip => {
                self.state
                    .lookup_misses_by_region
                    .immutable_xip_flash_aliases = self
                    .state
                    .lookup_misses_by_region
                    .immutable_xip_flash_aliases
                    .saturating_add(1)
            }
            DecodeLookupRegion::XipSram => {
                self.state.lookup_misses_by_region.xip_sram = self
                    .state
                    .lookup_misses_by_region
                    .xip_sram
                    .saturating_add(1)
            }
            DecodeLookupRegion::Sram => {
                self.state.lookup_misses_by_region.sram =
                    self.state.lookup_misses_by_region.sram.saturating_add(1)
            }
            DecodeLookupRegion::Other => {
                self.state.lookup_misses_by_region.other =
                    self.state.lookup_misses_by_region.other.saturating_add(1)
            }
        }
    }

    fn close_immutable_xip_hit_run(&mut self, reason: Option<ImmutableXipHitRunTerminationReason>) {
        if self.state.open_immutable_xip_hit_run_instructions == 0 {
            return;
        }
        self.state
            .immutable_xip_hit_runs
            .record(self.state.open_immutable_xip_hit_run_instructions);
        if let Some(reason) = reason {
            match reason {
                ImmutableXipHitRunTerminationReason::PostExecuteNextPcRedirect => {
                    self.state
                        .immutable_xip_hit_run_termination_counters
                        .post_execute_next_pc_redirect = self
                        .state
                        .immutable_xip_hit_run_termination_counters
                        .post_execute_next_pc_redirect
                        .saturating_add(1);
                }
                ImmutableXipHitRunTerminationReason::XipMiss => {
                    self.state
                        .immutable_xip_hit_run_termination_counters
                        .xip_miss = self
                        .state
                        .immutable_xip_hit_run_termination_counters
                        .xip_miss
                        .saturating_add(1);
                }
                ImmutableXipHitRunTerminationReason::RegionExit => {
                    self.state
                        .immutable_xip_hit_run_termination_counters
                        .region_exit = self
                        .state
                        .immutable_xip_hit_run_termination_counters
                        .region_exit
                        .saturating_add(1);
                }
                ImmutableXipHitRunTerminationReason::PrefetchException => {
                    self.state
                        .immutable_xip_hit_run_termination_counters
                        .prefetch_exception = self
                        .state
                        .immutable_xip_hit_run_termination_counters
                        .prefetch_exception
                        .saturating_add(1);
                }
                ImmutableXipHitRunTerminationReason::Fault => {
                    self.state.immutable_xip_hit_run_termination_counters.fault = self
                        .state
                        .immutable_xip_hit_run_termination_counters
                        .fault
                        .saturating_add(1);
                }
            }
        }
        self.state.open_immutable_xip_hit_run_instructions = 0;
        self.state.open_immutable_xip_hit_run_next_pc = None;
    }

    fn decode_lookup_region(pc: u32) -> DecodeLookupRegion {
        if (pc >> 28) == 0 {
            return DecodeLookupRegion::Rom;
        }
        if (XIP_IMMUTABLE_BASE..XIP_IMMUTABLE_END).contains(&pc) {
            return DecodeLookupRegion::ImmutableXip;
        }
        if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&pc) {
            return DecodeLookupRegion::XipSram;
        }
        if (pc >> 28) == 0x2 {
            return DecodeLookupRegion::Sram;
        }
        DecodeLookupRegion::Other
    }

    fn close_hit_run(&mut self) {
        if self.state.open_hit_run_instructions == 0 {
            return;
        }
        self.state
            .sequential_cache_hit_runs
            .record(self.state.open_hit_run_instructions);
        self.state.open_hit_run_instructions = 0;
        self.state.open_hit_run_next_pc = None;
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
    fn one_cycle_fallback_signatures_track_overlap() {
        let mut p = RunningProfile::default();
        let signature = IdleEventSourceMask::from_bits(
            IdleEventSourceMask::PIO
                | IdleEventSourceMask::UART
                | IdleEventSourceMask::DMA
                | IdleEventSourceMask::PWM,
        );
        p.record_running(
            4,
            RunningBoundaryMask::from_bits(0),
            horizon(1, signature.bits()),
        );

        let snap = p.snapshot();
        assert_eq!(snap.one_cycle_fallback_signatures.steps[0b1111], 1);
        assert_eq!(snap.one_cycle_fallback_signatures.cycle_mass[0b1111], 4);
        assert_eq!(snap.one_cycle_fallback_cycles.pio, 4);
        assert_eq!(snap.one_cycle_fallback_cycles.uart, 4);
        assert_eq!(snap.one_cycle_fallback_cycles.dma, 4);
        assert_eq!(snap.one_cycle_fallback_cycles.pwm, 4);
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

    #[test]
    fn decode_lookup_region_counters_and_widths_track_narrow_wide_hits() {
        let mut p = DecodeProfile::default();
        p.record_decode_lookup(0x0000_1000, 2, true, true);
        p.record_decode_lookup(0x1000_0020, 4, true, true);
        p.record_decode_lookup(0x1500_0040, 2, true, true);
        p.record_decode_lookup(0x2000_3000, 4, true, true);
        p.record_decode_lookup(0x4000_0000, 2, true, true);
        p.record_decode_lookup(0x1000_0040, 2, true, false);
        p.record_decode_lookup(0x4000_1000, 4, false, false);

        let snap = p.snapshot();
        assert_eq!(snap.cacheable_hits, 5);
        assert_eq!(snap.cacheable_misses, 1);
        assert_eq!(snap.noncacheable_fetches, 1);
        assert_eq!(snap.cacheable_hits_narrow, 3);
        assert_eq!(snap.cacheable_hits_wide, 2);
        assert_eq!(snap.lookup_hits_by_region.rom, 1);
        assert_eq!(snap.lookup_hits_by_region.immutable_xip_flash_aliases, 1);
        assert_eq!(snap.lookup_hits_by_region.xip_sram, 1);
        assert_eq!(snap.lookup_hits_by_region.sram, 1);
        assert_eq!(snap.lookup_hits_by_region.other, 1);
        assert_eq!(snap.lookup_misses_by_region.immutable_xip_flash_aliases, 1);
        assert_eq!(snap.lookup_misses_by_region.other, 1);
    }

    #[test]
    fn immutable_xip_hit_runs_track_termination_reasons() {
        let mut p = DecodeProfile::default();
        let xip0 = 0x1000_0000;
        let xip1 = 0x1000_0010;
        let xip2 = 0x1000_0034;

        p.record_decode_lookup(xip0, 2, true, true);
        p.record_decode_lookup(xip0 + 4, 2, true, true);
        p.record_decode_lookup(0x2000_0000, 2, true, true);
        p.record_decode_lookup(xip1, 2, true, true);
        p.record_decode_lookup(xip1, 4, true, false);
        p.record_decode_lookup(xip2, 2, true, true);
        p.record_immutable_xip_hit_run_prefetch_exception();
        p.record_decode_lookup(xip2 + 0x40, 2, true, true);
        p.record_immutable_xip_hit_run_fault();

        let snap = p.snapshot();
        assert_eq!(snap.immutable_xip_hit_runs.episodes_ge[0], 5);
        assert_eq!(
            snap.immutable_xip_hit_run_termination_counters
                .post_execute_next_pc_redirect,
            1
        );
        assert_eq!(
            snap.immutable_xip_hit_run_termination_counters.region_exit,
            1
        );
        assert_eq!(snap.immutable_xip_hit_run_termination_counters.xip_miss, 1);
        assert_eq!(
            snap.immutable_xip_hit_run_termination_counters
                .prefetch_exception,
            1
        );
        assert_eq!(snap.immutable_xip_hit_run_termination_counters.fault, 1);
    }

    #[test]
    fn snapshot_flushes_open_immutable_xip_run_without_termination_counts() {
        let mut p = DecodeProfile::default();
        p.record_decode_lookup(0x1000_0000, 2, true, true);

        let first = p.snapshot();
        let second = p.snapshot();

        assert_eq!(first.immutable_xip_hit_runs.episodes_ge[0], 1);
        assert_eq!(
            first
                .immutable_xip_hit_run_termination_counters
                .post_execute_next_pc_redirect,
            0
        );
        assert_eq!(
            first.immutable_xip_hit_run_termination_counters.region_exit,
            0
        );
        assert_eq!(first.immutable_xip_hit_run_termination_counters.xip_miss, 0);
        assert_eq!(
            first
                .immutable_xip_hit_run_termination_counters
                .prefetch_exception,
            0
        );
        assert_eq!(first.immutable_xip_hit_run_termination_counters.fault, 0);

        assert_eq!(second.immutable_xip_hit_runs.episodes_ge[0], 1);
        assert_eq!(
            second
                .immutable_xip_hit_run_termination_counters
                .post_execute_next_pc_redirect,
            0
        );
        assert_eq!(
            second
                .immutable_xip_hit_run_termination_counters
                .region_exit,
            0
        );
        assert_eq!(
            second.immutable_xip_hit_run_termination_counters.xip_miss,
            0
        );
        assert_eq!(
            second
                .immutable_xip_hit_run_termination_counters
                .prefetch_exception,
            0
        );
        assert_eq!(second.immutable_xip_hit_run_termination_counters.fault, 0);
    }

    #[test]
    fn decode_cache_invalidation_observations_are_counted_by_region() {
        let mut p = DecodeProfile::default();
        p.record_decode_cache_entry_invalidation(0x0000_1000);
        p.record_decode_cache_entry_invalidation(0x1000_0200);
        p.record_decode_cache_entry_invalidation(0x1500_0004);
        p.record_decode_cache_entry_invalidation(0x2000_0010);
        p.record_decode_cache_region_invalidation(
            INVALIDATION_REGION_ROM | INVALIDATION_REGION_BULK,
        );
        p.record_decode_cache_region_invalidation(
            INVALIDATION_REGION_XIP | INVALIDATION_REGION_SRAM,
        );
        p.record_decode_cache_all_invalidation();
        p.record_decode_cache_all_invalidation();

        let obs = p.snapshot().decode_cache_invalidation_observations;
        assert_eq!(obs.entry_address_count, 4);
        assert_eq!(obs.rom, 2);
        assert_eq!(obs.xip, 3);
        assert_eq!(obs.sram, 2);
        assert_eq!(obs.bulk, 1);
        assert_eq!(obs.all, 2);
    }

    #[test]
    fn immutable_xip_region_boundaries_are_half_open_and_exclude_xip_sram() {
        let mut p = DecodeProfile::default();
        for pc in [0x1000_0000, 0x13ff_fffe, 0x13ff_ffff] {
            p.record_decode_lookup(pc, 2, true, true);
        }
        for pc in [0x1400_0000, 0x14ff_ffff] {
            p.record_decode_lookup(pc, 2, true, false);
        }
        for pc in [0x1500_0000, 0x1500_3fff] {
            p.record_decode_lookup(pc, 2, true, false);
        }
        p.record_decode_lookup(0x1500_4000, 2, true, false);

        let snap = p.snapshot();
        assert_eq!(snap.lookup_hits_by_region.immutable_xip_flash_aliases, 3);
        assert_eq!(snap.lookup_misses_by_region.other, 3);
        assert_eq!(snap.lookup_misses_by_region.xip_sram, 2);
    }
}
