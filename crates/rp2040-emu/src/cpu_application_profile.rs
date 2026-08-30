//! Feature-gated counters for finding CPU-side application hot paths.
//!
//! This profiler is deliberately an emulated-cycle/instruction observer. It
//! never reads a host clock and it is not an acceptance or wall-time
//! measurement mode. The normal emulator build does not compile any of the
//! hooks that call this module.

pub const CPU_APPLICATION_PROFILE_SCHEMA_VERSION: u32 = 1;

const XIP_IMMUTABLE_BASE: u32 = 0x1000_0000;
const XIP_IMMUTABLE_END: u32 = 0x1400_0000;
const XIP_SRAM_BASE: u32 = 0x1500_0000;
const XIP_SRAM_END: u32 = 0x1500_4000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuPcRegionCounters {
    pub boot_rom: u64,
    pub immutable_xip: u64,
    pub xip_sram: u64,
    pub sram: u64,
    pub other: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuDecodeRegionCounters {
    pub lookups: u64,
    pub hits: u64,
    pub misses: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CpuDecodeCounters {
    pub lookups: u64,
    pub hits: u64,
    pub misses: u64,
    pub noncacheable_fetches: u64,
    pub by_region: CpuDecodeRegionCountersByRegion,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuDecodeRegionCountersByRegion {
    pub boot_rom: CpuDecodeRegionCounters,
    pub immutable_xip: CpuDecodeRegionCounters,
    pub xip_sram: CpuDecodeRegionCounters,
    pub sram: CpuDecodeRegionCounters,
    pub other: CpuDecodeRegionCounters,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuInvalidationCounters {
    pub requests: u64,
    pub examined_slots: u64,
    pub matching_clears: u64,
    pub unrelated_would_clear: u64,
    pub wide_predecessor_clears: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuExceptionCounters {
    pub polls: u64,
    pub reject_primask: u64,
    pub reject_no_candidate: u64,
    pub reject_active_handler: u64,
    pub entries: u64,
    pub source: CpuExceptionSourceCounters,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuExceptionSourceCounters {
    pub pendsv: u64,
    pub systick: u64,
    pub nvic: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuHandlerGroupCounters {
    pub thumb16_shift_add_sub: u64,
    pub data_processing: u64,
    pub load_store: u64,
    pub branch_system: u64,
    pub thumb32: u64,
    pub other: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CpuApplicationProfileSnapshot {
    pub active: bool,
    pub retired_instructions: u64,
    pub emulated_cycles: u64,
    pub pc_region: CpuPcRegionCounters,
    pub decode: CpuDecodeCounters,
    pub invalidation: CpuInvalidationCounters,
    pub exception: CpuExceptionCounters,
    pub handler_group: CpuHandlerGroupCounters,
    pub overflowed: bool,
}

/// Mutable per-core counter bank.
#[derive(Clone, Debug, Default)]
pub struct CpuApplicationProfiler {
    state: CpuApplicationProfileSnapshot,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PcRegion {
    Rom,
    ImmutableXip,
    XipSram,
    Sram,
    Other,
}

impl CpuApplicationProfiler {
    pub fn snapshot(&self) -> CpuApplicationProfileSnapshot {
        self.state.clone()
    }

    pub fn record_cycles(&mut self, cycles: u64) {
        add(
            &mut self.state.emulated_cycles,
            cycles,
            &mut self.state.overflowed,
        );
    }

    pub fn record_retirement(&mut self, pc: u32, hw0: u16, wide: bool) {
        self.state.active = true;
        add(
            &mut self.state.retired_instructions,
            1,
            &mut self.state.overflowed,
        );
        add_region(
            &mut self.state.pc_region,
            classify_pc(pc),
            1,
            &mut self.state.overflowed,
        );
        add_handler(
            &mut self.state.handler_group,
            classify_handler(hw0, wide),
            1,
            &mut self.state.overflowed,
        );
    }

    pub fn record_decode_lookup(&mut self, pc: u32, cacheable: bool, hit: bool) {
        add(
            &mut self.state.decode.lookups,
            1,
            &mut self.state.overflowed,
        );
        let counter = if hit {
            &mut self.state.decode.hits
        } else {
            &mut self.state.decode.misses
        };
        add(counter, 1, &mut self.state.overflowed);
        add_decode_region(
            &mut self.state.decode.by_region,
            classify_pc(pc),
            hit,
            &mut self.state.overflowed,
        );
        if !cacheable {
            add(
                &mut self.state.decode.noncacheable_fetches,
                1,
                &mut self.state.overflowed,
            );
            return;
        }
    }

    pub fn record_invalidation_request(&mut self) {
        add(
            &mut self.state.invalidation.requests,
            1,
            &mut self.state.overflowed,
        );
    }

    /// Record one slot inspected by an invalidation operation. `nonempty`
    /// describes the pre-clear entry, while `matching` says that the entry
    /// was actually the address/region being invalidated. Region sweeps pass
    /// `nonempty=false` for untouched slots so they do not look like alias
    /// collateral.
    pub fn record_invalidation_slot(
        &mut self,
        nonempty: bool,
        matching: bool,
        wide_predecessor: bool,
    ) {
        add(
            &mut self.state.invalidation.examined_slots,
            1,
            &mut self.state.overflowed,
        );
        if matching {
            add(
                &mut self.state.invalidation.matching_clears,
                1,
                &mut self.state.overflowed,
            );
            if wide_predecessor {
                add(
                    &mut self.state.invalidation.wide_predecessor_clears,
                    1,
                    &mut self.state.overflowed,
                );
            }
        } else if nonempty {
            add(
                &mut self.state.invalidation.unrelated_would_clear,
                1,
                &mut self.state.overflowed,
            );
        }
    }

    pub fn record_exception_poll(&mut self) {
        add(
            &mut self.state.exception.polls,
            1,
            &mut self.state.overflowed,
        );
    }

    pub fn record_exception_reject_primask(&mut self) {
        add(
            &mut self.state.exception.reject_primask,
            1,
            &mut self.state.overflowed,
        );
    }

    pub fn record_exception_reject_no_candidate(&mut self) {
        add(
            &mut self.state.exception.reject_no_candidate,
            1,
            &mut self.state.overflowed,
        );
    }

    pub fn record_exception_reject_active_handler(&mut self) {
        add(
            &mut self.state.exception.reject_active_handler,
            1,
            &mut self.state.overflowed,
        );
    }

    pub fn record_exception_entry(&mut self, exception: u16) {
        add(
            &mut self.state.exception.entries,
            1,
            &mut self.state.overflowed,
        );
        let counter = match exception {
            14 => &mut self.state.exception.source.pendsv,
            15 => &mut self.state.exception.source.systick,
            _ => &mut self.state.exception.source.nvic,
        };
        add(counter, 1, &mut self.state.overflowed);
    }
}

impl CpuApplicationProfileSnapshot {
    pub fn aggregate(cores: &[Self; 2]) -> Self {
        let mut aggregate = Self::default();
        for core in cores {
            aggregate.merge(core);
        }
        aggregate
    }

    pub fn invariants_valid(&self) -> bool {
        let decode = &self.decode;
        let exception = &self.exception;
        !self.overflowed
            && self.pc_region_total() == self.retired_instructions
            && decode.lookups == decode.hits.saturating_add(decode.misses)
            && self.decode_region_conservation_valid()
            && exception.polls
                == exception
                    .reject_primask
                    .saturating_add(exception.reject_no_candidate)
                    .saturating_add(exception.reject_active_handler)
                    .saturating_add(exception.entries)
            && exception.entries
                == exception
                    .source
                    .pendsv
                    .saturating_add(exception.source.systick)
                    .saturating_add(exception.source.nvic)
            && self.handler_group_conservation_valid()
            && self
                .invalidation
                .matching_clears
                .saturating_add(self.invalidation.unrelated_would_clear)
                <= self.invalidation.examined_slots
    }

    pub fn handler_group_conservation_valid(&self) -> bool {
        self.handler_group_total() == self.retired_instructions
    }

    pub fn decode_region_conservation_valid(&self) -> bool {
        let regions = &self.decode.by_region;
        let region_valid = |value: &CpuDecodeRegionCounters| {
            value.lookups == value.hits.saturating_add(value.misses)
        };
        region_valid(&regions.boot_rom)
            && region_valid(&regions.immutable_xip)
            && region_valid(&regions.xip_sram)
            && region_valid(&regions.sram)
            && region_valid(&regions.other)
            && regions_total(regions) == self.decode.lookups
            && regions_hits(regions) == self.decode.hits
            && regions_misses(regions) == self.decode.misses
    }

    fn pc_region_total(&self) -> u64 {
        self.pc_region
            .boot_rom
            .saturating_add(self.pc_region.immutable_xip)
            .saturating_add(self.pc_region.xip_sram)
            .saturating_add(self.pc_region.sram)
            .saturating_add(self.pc_region.other)
    }

    fn handler_group_total(&self) -> u64 {
        self.handler_group
            .thumb16_shift_add_sub
            .saturating_add(self.handler_group.data_processing)
            .saturating_add(self.handler_group.load_store)
            .saturating_add(self.handler_group.branch_system)
            .saturating_add(self.handler_group.thumb32)
            .saturating_add(self.handler_group.other)
    }

    fn merge(&mut self, other: &Self) {
        self.active |= other.active;
        self.overflowed |= other.overflowed;
        add(
            &mut self.retired_instructions,
            other.retired_instructions,
            &mut self.overflowed,
        );
        add(
            &mut self.emulated_cycles,
            other.emulated_cycles,
            &mut self.overflowed,
        );
        merge_region(&mut self.pc_region, &other.pc_region, &mut self.overflowed);
        add_decode(&mut self.decode, &other.decode, &mut self.overflowed);
        add_invalidation(
            &mut self.invalidation,
            &other.invalidation,
            &mut self.overflowed,
        );
        add_exception(&mut self.exception, &other.exception, &mut self.overflowed);
        add_handlers(
            &mut self.handler_group,
            &other.handler_group,
            &mut self.overflowed,
        );
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum HandlerGroup {
    Thumb16ShiftAddSub,
    DataProcessing,
    LoadStore,
    BranchSystem,
    Thumb32,
    Other,
}

fn classify_pc(pc: u32) -> PcRegion {
    if (pc >> 28) == 0 {
        PcRegion::Rom
    } else if (XIP_IMMUTABLE_BASE..XIP_IMMUTABLE_END).contains(&pc) {
        PcRegion::ImmutableXip
    } else if (XIP_SRAM_BASE..XIP_SRAM_END).contains(&pc) {
        PcRegion::XipSram
    } else if (pc >> 28) == 0x2 {
        PcRegion::Sram
    } else {
        PcRegion::Other
    }
}

fn classify_handler(hw0: u16, wide: bool) -> HandlerGroup {
    if wide {
        return HandlerGroup::Thumb32;
    }
    let top = hw0 >> 11;
    if top <= 0b00011 {
        HandlerGroup::Thumb16ShiftAddSub
    } else if (hw0 & 0xE000) == 0x4000 {
        HandlerGroup::DataProcessing
    } else if (hw0 & 0xE000) == 0x6000
        || (hw0 & 0xF000) == 0x8000
        || (hw0 & 0xF000) == 0x9000
        || (hw0 & 0xF000) == 0xC000
    {
        HandlerGroup::LoadStore
    } else if (hw0 & 0xF000) == 0xD000
        || (hw0 & 0xF800) == 0xE000
        || (hw0 & 0xF800) == 0xA000
        || (hw0 & 0xF800) == 0xB000
    {
        HandlerGroup::BranchSystem
    } else {
        HandlerGroup::Other
    }
}

fn add(value: &mut u64, amount: u64, overflowed: &mut bool) {
    if let Some(next) = value.checked_add(amount) {
        *value = next;
    } else {
        *value = u64::MAX;
        *overflowed = true;
    }
}

fn add_region(
    counters: &mut CpuPcRegionCounters,
    region: PcRegion,
    amount: u64,
    overflowed: &mut bool,
) {
    let value = match region {
        PcRegion::Rom => &mut counters.boot_rom,
        PcRegion::ImmutableXip => &mut counters.immutable_xip,
        PcRegion::XipSram => &mut counters.xip_sram,
        PcRegion::Sram => &mut counters.sram,
        PcRegion::Other => &mut counters.other,
    };
    add(value, amount, overflowed);
}

fn add_handler(
    counters: &mut CpuHandlerGroupCounters,
    group: HandlerGroup,
    amount: u64,
    overflowed: &mut bool,
) {
    let value = match group {
        HandlerGroup::Thumb16ShiftAddSub => &mut counters.thumb16_shift_add_sub,
        HandlerGroup::DataProcessing => &mut counters.data_processing,
        HandlerGroup::LoadStore => &mut counters.load_store,
        HandlerGroup::BranchSystem => &mut counters.branch_system,
        HandlerGroup::Thumb32 => &mut counters.thumb32,
        HandlerGroup::Other => &mut counters.other,
    };
    add(value, amount, overflowed);
}

fn merge_region(
    left: &mut CpuPcRegionCounters,
    right: &CpuPcRegionCounters,
    overflowed: &mut bool,
) {
    add(&mut left.boot_rom, right.boot_rom, overflowed);
    add(&mut left.immutable_xip, right.immutable_xip, overflowed);
    add(&mut left.xip_sram, right.xip_sram, overflowed);
    add(&mut left.sram, right.sram, overflowed);
    add(&mut left.other, right.other, overflowed);
}

fn add_decode_region(
    counters: &mut CpuDecodeRegionCountersByRegion,
    region: PcRegion,
    hit: bool,
    overflowed: &mut bool,
) {
    let value = match region {
        PcRegion::Rom => &mut counters.boot_rom,
        PcRegion::ImmutableXip => &mut counters.immutable_xip,
        PcRegion::XipSram => &mut counters.xip_sram,
        PcRegion::Sram => &mut counters.sram,
        PcRegion::Other => &mut counters.other,
    };
    add(&mut value.lookups, 1, overflowed);
    if hit {
        add(&mut value.hits, 1, overflowed);
    } else {
        add(&mut value.misses, 1, overflowed);
    }
}

fn merge_decode_region(
    left: &mut CpuDecodeRegionCounters,
    right: &CpuDecodeRegionCounters,
    overflowed: &mut bool,
) {
    add(&mut left.lookups, right.lookups, overflowed);
    add(&mut left.hits, right.hits, overflowed);
    add(&mut left.misses, right.misses, overflowed);
}

fn merge_decode_regions(
    left: &mut CpuDecodeRegionCountersByRegion,
    right: &CpuDecodeRegionCountersByRegion,
    overflowed: &mut bool,
) {
    merge_decode_region(&mut left.boot_rom, &right.boot_rom, overflowed);
    merge_decode_region(&mut left.immutable_xip, &right.immutable_xip, overflowed);
    merge_decode_region(&mut left.xip_sram, &right.xip_sram, overflowed);
    merge_decode_region(&mut left.sram, &right.sram, overflowed);
    merge_decode_region(&mut left.other, &right.other, overflowed);
}

fn regions_total(value: &CpuDecodeRegionCountersByRegion) -> u64 {
    value
        .boot_rom
        .lookups
        .saturating_add(value.immutable_xip.lookups)
        .saturating_add(value.xip_sram.lookups)
        .saturating_add(value.sram.lookups)
        .saturating_add(value.other.lookups)
}

fn regions_hits(value: &CpuDecodeRegionCountersByRegion) -> u64 {
    value
        .boot_rom
        .hits
        .saturating_add(value.immutable_xip.hits)
        .saturating_add(value.xip_sram.hits)
        .saturating_add(value.sram.hits)
        .saturating_add(value.other.hits)
}

fn regions_misses(value: &CpuDecodeRegionCountersByRegion) -> u64 {
    value
        .boot_rom
        .misses
        .saturating_add(value.immutable_xip.misses)
        .saturating_add(value.xip_sram.misses)
        .saturating_add(value.sram.misses)
        .saturating_add(value.other.misses)
}

fn add_decode(left: &mut CpuDecodeCounters, right: &CpuDecodeCounters, overflowed: &mut bool) {
    add(&mut left.lookups, right.lookups, overflowed);
    add(&mut left.hits, right.hits, overflowed);
    add(&mut left.misses, right.misses, overflowed);
    add(
        &mut left.noncacheable_fetches,
        right.noncacheable_fetches,
        overflowed,
    );
    merge_decode_regions(&mut left.by_region, &right.by_region, overflowed);
}

fn add_invalidation(
    left: &mut CpuInvalidationCounters,
    right: &CpuInvalidationCounters,
    overflowed: &mut bool,
) {
    add(&mut left.requests, right.requests, overflowed);
    add(&mut left.examined_slots, right.examined_slots, overflowed);
    add(&mut left.matching_clears, right.matching_clears, overflowed);
    add(
        &mut left.unrelated_would_clear,
        right.unrelated_would_clear,
        overflowed,
    );
    add(
        &mut left.wide_predecessor_clears,
        right.wide_predecessor_clears,
        overflowed,
    );
}

fn add_exception(
    left: &mut CpuExceptionCounters,
    right: &CpuExceptionCounters,
    overflowed: &mut bool,
) {
    add(&mut left.polls, right.polls, overflowed);
    add(&mut left.reject_primask, right.reject_primask, overflowed);
    add(
        &mut left.reject_no_candidate,
        right.reject_no_candidate,
        overflowed,
    );
    add(
        &mut left.reject_active_handler,
        right.reject_active_handler,
        overflowed,
    );
    add(&mut left.entries, right.entries, overflowed);
    add(&mut left.source.pendsv, right.source.pendsv, overflowed);
    add(&mut left.source.systick, right.source.systick, overflowed);
    add(&mut left.source.nvic, right.source.nvic, overflowed);
}

fn add_handlers(
    left: &mut CpuHandlerGroupCounters,
    right: &CpuHandlerGroupCounters,
    overflowed: &mut bool,
) {
    add(
        &mut left.thumb16_shift_add_sub,
        right.thumb16_shift_add_sub,
        overflowed,
    );
    add(&mut left.data_processing, right.data_processing, overflowed);
    add(&mut left.load_store, right.load_store, overflowed);
    add(&mut left.branch_system, right.branch_system, overflowed);
    add(&mut left.thumb32, right.thumb32, overflowed);
    add(&mut left.other, right.other, overflowed);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn regions_and_handler_groups_are_counted() {
        let mut profiler = CpuApplicationProfiler::default();
        profiler.record_decode_lookup(0x1000_0000, true, true);
        profiler.record_decode_lookup(0x2000_0000, true, false);
        profiler.record_decode_lookup(0x4000_0000, false, false);
        profiler.record_retirement(0x1000_0000, 0x0000, false);
        profiler.record_retirement(0x2000_0000, 0xF000, true);
        profiler.record_cycles(2);
        let snapshot = profiler.snapshot();
        assert!(snapshot.active);
        assert_eq!(snapshot.retired_instructions, 2);
        assert_eq!(snapshot.emulated_cycles, 2);
        assert_eq!(snapshot.pc_region.immutable_xip, 1);
        assert_eq!(snapshot.pc_region.sram, 1);
        assert_eq!(snapshot.handler_group.thumb16_shift_add_sub, 1);
        assert_eq!(snapshot.handler_group.thumb32, 1);
        assert!(snapshot.invariants_valid());
    }

    #[test]
    fn overflow_is_sticky_and_saturating() {
        let mut profiler = CpuApplicationProfiler::default();
        profiler.state.retired_instructions = u64::MAX;
        profiler.record_retirement(0, 0, false);
        assert_eq!(profiler.state.retired_instructions, u64::MAX);
        assert!(profiler.state.overflowed);
        assert!(!profiler.snapshot().invariants_valid());
    }

    #[test]
    fn aggregate_preserves_core_zero_and_inactive_core() {
        let mut active = CpuApplicationProfiler::default();
        active.record_retirement(0x2000_0000, 0x0000, false);
        active.record_cycles(1);
        let inactive = CpuApplicationProfileSnapshot::default();
        let aggregate = CpuApplicationProfileSnapshot::aggregate(&[active.snapshot(), inactive]);
        assert!(aggregate.active);
        assert_eq!(aggregate.retired_instructions, 1);
        assert!(aggregate.invariants_valid());
    }
}
