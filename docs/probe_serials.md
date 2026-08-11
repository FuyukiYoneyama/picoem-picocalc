# Probe serial → DUT mapping

This document records how to configure a user's own Pico debug probes.
Probe serials are deliberately not part of the public source tree: they are
per-device identifiers for one development rig, not emulator inputs. The
harness binaries that talk to real silicon take a `--probe <VID:PID:SERIAL>`
argument to disambiguate when more than one probe is attached.

## Mapping

| Probe serial | `--probe` argument | DUT |
|---|---|---|
| `<RP2354 probe serial>` | `2e8a:000c:<RP2354 probe serial>` | RP2354 (Pico 2) |
| `<RP2040 probe serial>` | `2e8a:000c:<RP2040 probe serial>` | RP2040 (Pico V1) |

`2e8a:000c` is the USB VID:PID for the Raspberry Pi debug probe
(`2e8a` = Raspberry Pi Foundation, `000c` = debug probe). The serial
suffix is the per-device unique ID burnt into the probe's RP2040.

`probe-rs list` shows the serials of all probes currently attached. Match the
target type reported by `probe-rs info` and pass the corresponding selector;
never copy a serial from another user's machine.

## Why explicit `--probe` is required on this host

`probe-rs auto_attach` picks the first enumerated probe regardless of
target type. On a host with both an RP2354 probe and an RP2040 probe
attached, that succeeds approximately half the time and fails the
other half — the wrong probe attaches to the wrong target and the
session aborts. Passing the full `VID:PID:SERIAL` triplet makes the
selection deterministic.

## Affected harness binaries

All silicon-touching harness binaries accept `--probe`:

- `probe_diff_rp2350`
- `probe_diff_rp2040`
- `probe_verify_rp2350`
- `bank_conflict_test_rp2350`
- `silicon_cycle_oracle_rp2350`
- `silicon_periph_diff_rp2350`
- `silicon_dualcore_diff_rp2350`
- `silicon_isr_diff_rp2350`
- `silicon_periph_diff_rp2040`
- `silicon_isr_diff_rp2040`
- `test_silicon` (orchestrator)
- The `picogus_probe_pc` live-silicon variant
- The OneROM rig oracles (`onerom_*`)

## If the probes are reassigned

Keep the mapping in your private operator notes. Do not commit per-device
serials to this public repository. The public harness accepts an explicit
selector or `auto` when the host has only one compatible probe.
