//! OPT0-A microbenchmark for the Serial both-blocked path.
//!
//! This is a diagnostic executable, not a conformance runner. Invoke it
//! under an externally pinned CPU and retain every sample in its JSON.

use std::env;
use std::hint::black_box;
use std::path::PathBuf;
use std::time::Instant;

use rp2040_emu::bus::peripheral_dispatch::RESET_TIMER;
use rp2040_emu::bus::{RESETS_BASE, TIMER_BASE};
use rp2040_emu::peripherals::timer::{ALARM0_OFFSET, INTE_OFFSET};
use rp2040_emu::{Config, Emulator, EmulatorBuilder};
use serde_json::{Value, json};

const BUILT_BACKEND_COMMIT: &str = env!("PICOEM_BUILT_COMMIT");
const ADVANCE_LENGTHS: [u32; 4] = [1, 64, 1024, 1_048_576];

#[derive(Clone, Debug, PartialEq, Eq)]
struct Args {
    iterations: u64,
    samples: usize,
    output: PathBuf,
}

fn parse_positive<T: std::str::FromStr>(name: &str, value: &str) -> Result<T, String> {
    value
        .parse::<T>()
        .map_err(|_| format!("invalid {name} '{value}'"))
}

fn parse_args_from(argv: &[String]) -> Result<Args, String> {
    let mut iterations = 1_000_000u64;
    let mut samples = 10usize;
    let mut output = None;
    let mut index = 0;
    while index < argv.len() {
        let option = &argv[index];
        index += 1;
        let value = |index: &mut usize| -> Result<&str, String> {
            let item = argv
                .get(*index)
                .ok_or_else(|| format!("{option} requires a value"))?;
            *index += 1;
            Ok(item)
        };
        match option.as_str() {
            "--iterations" => {
                iterations = parse_positive("--iterations", value(&mut index)?)?;
            }
            "--samples" => samples = parse_positive("--samples", value(&mut index)?)?,
            "--json" => output = Some(PathBuf::from(value(&mut index)?)),
            "-h" | "--help" => {
                return Err(
                    "usage: opt0-idle-cost [--iterations N] [--samples N] --json PATH".to_string(),
                );
            }
            other => return Err(format!("unknown option '{other}'")),
        }
    }
    if iterations < 1_000 {
        return Err("--iterations must be at least 1000".to_string());
    }
    if !(3..=100).contains(&samples) {
        return Err("--samples must be between 3 and 100".to_string());
    }
    Ok(Args {
        iterations,
        samples,
        output: output.ok_or_else(|| "--json is required".to_string())?,
    })
}

fn built_backend_dirty() -> bool {
    env!("PICOEM_BUILT_DIRTY") == "true"
}

fn blocked_emulator() -> Emulator {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .expect("Serial benchmark emulator");
    emu.core_mut(0).halt();
    emu.halt_core1();
    emu.mmio_write32(RESETS_BASE + 0x3000, 1u32 << RESET_TIMER);
    emu.mmio_write32(TIMER_BASE + INTE_OFFSET, 1);
    emu.mmio_write32(TIMER_BASE + ALARM0_OFFSET, 4_000_000_000);
    emu
}

fn ns_per_iteration(iterations: u64, mut body: impl FnMut(u64)) -> f64 {
    let started = Instant::now();
    for iteration in 0..iterations {
        body(black_box(iteration));
    }
    started.elapsed().as_nanos() as f64 / iterations as f64
}

fn collect_samples(samples: usize, mut body: impl FnMut() -> f64) -> Vec<f64> {
    (0..samples).map(|_| body()).collect()
}

fn median(values: &[f64]) -> f64 {
    let mut ordered = values.to_vec();
    ordered.sort_by(f64::total_cmp);
    let midpoint = ordered.len() / 2;
    if ordered.len().is_multiple_of(2) {
        (ordered[midpoint - 1] + ordered[midpoint]) / 2.0
    } else {
        ordered[midpoint]
    }
}

fn measurement(values: Vec<f64>, loop_overhead_median: f64) -> Value {
    let raw_median = median(&values);
    json!({
        "samples_ns_per_op": values,
        "median_ns_per_op": raw_median,
        "median_net_of_loop_ns_per_op": (raw_median - loop_overhead_median).max(0.0),
    })
}

fn run(args: &Args) -> Result<Value, String> {
    // Untimed warm-up forces code pages and allocator state into the same
    // condition before each family of retained samples.
    let mut warm = blocked_emulator();
    for _ in 0..10_000 {
        black_box(warm.step().map_err(|error| error.to_string())?);
    }
    let warm_probe = blocked_emulator();
    for _ in 0..10_000 {
        black_box(warm_probe.idle_current_probe());
    }

    let loop_samples = collect_samples(args.samples, || {
        ns_per_iteration(args.iterations, |iteration| {
            black_box(iteration);
        })
    });
    let loop_median = median(&loop_samples);

    let probe_emu = blocked_emulator();
    let initial_probe = probe_emu.idle_current_probe();
    let probe_samples = collect_samples(args.samples, || {
        ns_per_iteration(args.iterations, |_| {
            black_box(probe_emu.idle_current_probe());
        })
    });

    let blocked_samples = collect_samples(args.samples, || {
        let mut emu = blocked_emulator();
        ns_per_iteration(args.iterations, |_| {
            black_box(emu.step().expect("blocked step"));
        })
    });

    let mut advance_results = serde_json::Map::new();
    for length in ADVANCE_LENGTHS {
        let samples = collect_samples(args.samples, || {
            let mut emu = blocked_emulator();
            let result = ns_per_iteration(args.iterations, |_| {
                emu.bus.tick_peripherals(black_box(length));
            });
            black_box(&emu);
            result
        });
        advance_results.insert(length.to_string(), measurement(samples, loop_median));
    }

    let probe_measurement = measurement(probe_samples, loop_median);
    let blocked_measurement = measurement(blocked_samples, loop_median);
    let probe_net = probe_measurement["median_net_of_loop_ns_per_op"]
        .as_f64()
        .ok_or("probe median missing")?;
    let blocked_net = blocked_measurement["median_net_of_loop_ns_per_op"]
        .as_f64()
        .ok_or("blocked median missing")?;
    let advance_one_net = advance_results["1"]["median_net_of_loop_ns_per_op"]
        .as_f64()
        .ok_or("advance median missing")?;
    let optimistic_break_even = if blocked_net > 0.0 {
        ((probe_net + advance_one_net) / blocked_net).ceil() as u64
    } else {
        0
    };

    Ok(json!({
        "schema_version": 1,
        "kind": "rp2040_serial_idle_cost_microbenchmark",
        "backend_build": {
            "commit": BUILT_BACKEND_COMMIT,
            "dirty": built_backend_dirty(),
        },
        "execution_model": "Serial",
        "diagnostic": true,
        "valid_for_realtime_baseline": false,
        "iterations_per_sample": args.iterations,
        "retained_samples": args.samples,
        "warmup_iterations_per_family": 10_000,
        "current_probe_scope": {
            "complete_event_horizon": false,
            "lazy_deadline_sources": ["timer"],
            "active_sources_are_blockers_not_deadlines": true,
            "initial_probe": {
                "master_cycle": initial_probe.master_cycle,
                "next_lazy_deadline": initial_probe.next_lazy_deadline,
                "blocker_count": initial_probe.blocker_count,
                "proven_quiescent": initial_probe.proven_quiescent,
            },
        },
        "measurements": {
            "loop_overhead": measurement(loop_samples, 0.0),
            "current_conservative_probe": probe_measurement,
            "blocked_step_quantum_1": blocked_measurement,
            "quiescent_tick_peripherals_by_advance_cycles": advance_results,
        },
        "screening": {
            "optimistic_break_even_cycles_excluding_boundary_and_routing_cost": optimistic_break_even,
            "event_fire_and_route_cost_measured": false,
            "clock_update_and_wake_check_cost_measured": false,
            "full_all_source_horizon_cost_measured": false,
            "eligible_for_optimization_priority_decision": false,
        },
    }))
}

fn main() {
    let argv: Vec<String> = env::args().skip(1).collect();
    let result = parse_args_from(&argv).and_then(|args| {
        let report = run(&args)?;
        let encoded = serde_json::to_string_pretty(&report)
            .map_err(|error| format!("encoding report: {error}"))?;
        std::fs::write(&args.output, format!("{encoded}\n"))
            .map_err(|error| format!("writing {}: {error}", args.output.display()))
    });
    if let Err(error) = result {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_required_output_and_bounds() {
        let args = parse_args_from(&[
            "--iterations".into(),
            "1000".into(),
            "--samples".into(),
            "3".into(),
            "--json".into(),
            "out.json".into(),
        ])
        .unwrap();
        assert_eq!(args.iterations, 1000);
        assert_eq!(args.samples, 3);
        assert_eq!(args.output, PathBuf::from("out.json"));
        assert!(parse_args_from(&["--iterations".into(), "999".into()]).is_err());
    }

    #[test]
    fn median_handles_odd_and_even_samples() {
        assert_eq!(median(&[9.0, 1.0, 3.0]), 3.0);
        assert_eq!(median(&[4.0, 1.0, 2.0, 3.0]), 2.5);
    }
}
