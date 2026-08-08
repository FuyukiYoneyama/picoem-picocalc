//! OPT0-A production-path baseline for the Serial both-blocked branch.
//!
//! Build this binary without `idle-profiler`. It intentionally contains no
//! horizon API calls, allowing `Cblocked` to be measured without diagnostic
//! feature overhead.

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
const BENCH_SYS_HZ: u32 = 125_000_000;
const STEP_LENGTHS: [u32; 4] = [1, 64, 125, 1024];

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
                    "usage: opt0-blocked-baseline [--iterations N] [--samples N] --json PATH"
                        .to_string(),
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

fn blocked_emulator(step_quantum: u32) -> Emulator {
    let mut emu = EmulatorBuilder::new(Config {
        sys_clk_hz: BENCH_SYS_HZ,
    })
    .step_quantum(step_quantum)
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

fn measurement(values: Vec<f64>, loop_overhead: f64) -> Value {
    let raw_median = median(&values);
    json!({
        "samples_ns_per_op": values,
        "median_ns_per_op": raw_median,
        "median_net_of_loop_ns_per_op": (raw_median - loop_overhead).max(0.0),
    })
}

fn run(args: &Args) -> Value {
    let loop_samples: Vec<f64> = (0..args.samples)
        .map(|_| {
            ns_per_iteration(args.iterations, |iteration| {
                black_box(iteration);
            })
        })
        .collect();
    let loop_median = median(&loop_samples);
    let mut steps = serde_json::Map::new();
    for length in STEP_LENGTHS {
        let samples = (0..args.samples)
            .map(|_| {
                let mut emu = blocked_emulator(length);
                let result = ns_per_iteration(args.iterations, |_| {
                    black_box(emu.step().expect("blocked step"));
                });
                black_box(&emu);
                result
            })
            .collect();
        steps.insert(length.to_string(), measurement(samples, loop_median));
    }
    json!({
        "schema_version": 1,
        "kind": "rp2040_serial_blocked_production_baseline",
        "backend_build": {
            "commit": BUILT_BACKEND_COMMIT,
            "dirty": env!("PICOEM_BUILT_DIRTY") == "true",
        },
        "execution_model": "Serial",
        "idle_profiler_compiled": cfg!(feature = "idle-profiler"),
        "configured_sys_clk_hz": BENCH_SYS_HZ,
        "iterations_per_sample": args.iterations,
        "retained_samples": args.samples,
        "measurements": {
            "loop_overhead": measurement(loop_samples, 0.0),
            "blocked_step_by_advance_cycles": steps,
        },
    })
}

fn main() {
    let argv: Vec<String> = env::args().skip(1).collect();
    let result = parse_args_from(&argv).and_then(|args| {
        if cfg!(feature = "idle-profiler") {
            return Err("rebuild opt0-blocked-baseline without idle-profiler".to_string());
        }
        let encoded = serde_json::to_string_pretty(&run(&args))
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
    fn arguments_require_output_and_sensible_bounds() {
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
        assert!(parse_args_from(&["--iterations".into(), "999".into()]).is_err());
    }

    #[test]
    fn median_handles_even_and_odd_samples() {
        assert_eq!(median(&[3.0, 1.0, 2.0]), 2.0);
        assert_eq!(median(&[4.0, 1.0, 3.0, 2.0]), 2.5);
    }
}
