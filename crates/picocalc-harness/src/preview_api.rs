//! Realtime preview backend adapter.
//!
//! The adapter owns no emulator state of its own.  It drives the shared
//! `MachineSession`, keeps the authoritative serial execution path intact,
//! and exposes only the frozen local preview IPC wire.  The frontend is
//! expected to be a separate process; stdout is therefore reserved for
//! framed protocol bytes and diagnostics stay on stderr.

use std::io;
use std::sync::mpsc::{self, SyncSender, TryRecvError};
use std::thread;
use std::time::Duration;

use picoem_common::{Pacer, PacerSnapshot};
use serde_json::{Value, json};

use crate::preview_protocol::{Direction, Frame, FrameReader, FrameWriter, Kind, ProtocolError};
use crate::session::PREVIEW_OBSERVATION_SCHEMA_VERSION;
use crate::{MachineSession, ScenarioReplay, ScenarioReplayStep};

const PACER_QUANTUM_CYCLES: u64 = 150;
const FRAME_POLL_QUANTA: u64 = 1_000;
const INPUT_QUEUE_CAPACITY: usize = 256;

type InputMessage = Result<Option<Frame>, String>;

/// Run the preview wire until the frontend sends `quit`, closes stdin, or a
/// malformed frame is received.  This function is intentionally separate
/// from the report-producing batch path: it never writes a schema-8 report.
pub(crate) fn run(
    machine: &mut MachineSession,
    mut replay: Option<ScenarioReplay>,
) -> Result<(), String> {
    let (input_tx, input_rx) = mpsc::sync_channel(INPUT_QUEUE_CAPACITY);
    spawn_input_reader(input_tx);

    let stdout = io::stdout();
    let mut output = FrameWriter::new(stdout.lock(), Direction::RunnerToPreview);
    output
        .write_json(
            Kind::Hello,
            &json!({
                "protocol": "preview-ipc",
                "role": "runner",
                "schema": 1,
            }),
        )
        .map_err(protocol_message)?;

    let initial_frame = machine.preview_framebuffer();
    let mut last_frame_sha = None;
    let mut frame_updates = 0u64;
    if let Some(framebuffer) = initial_frame {
        last_frame_sha = Some(framebuffer.rgb565_sha256());
        write_frame(&mut output, machine.cycles(), &framebuffer)?;
        frame_updates = 1;
    }

    let initial_hz = machine.preview_sys_clk_hz().max(1);
    let mut pacer = Pacer::with_quantum(initial_hz, PACER_QUANTUM_CYCLES);
    let stats = pacer.stats();
    let mut quanta = 0u64;
    let mut uart_tx_bytes = 0u64;
    let mut uart_rx_accepted = 0u64;
    let mut uart_rx_disabled = 0u64;
    let mut uart_rx_overrun = 0u64;
    let mut last_status_cycle = machine.cycles();
    write_status(
        &mut output,
        machine,
        &stats.snapshot(),
        frame_updates,
        uart_tx_bytes,
        uart_rx_accepted,
        uart_rx_disabled,
        uart_rx_overrun,
        replay.as_ref(),
    )?;

    let mut running = true;
    let mut replay_finished = false;
    let mut termination_error = None;
    while running {
        loop {
            match input_rx.try_recv() {
                Ok(message) => match message {
                    Ok(Some(frame)) => {
                        handle_input_frame(
                            frame,
                            machine,
                            &mut output,
                            &mut last_frame_sha,
                            &mut frame_updates,
                            &mut uart_rx_accepted,
                            &mut uart_rx_disabled,
                            &mut uart_rx_overrun,
                            &mut running,
                            replay.is_some(),
                        )?;
                        if !running {
                            break;
                        }
                    }
                    Ok(None) => {
                        termination_error =
                            Some("preview input closed without an explicit quit frame".to_string());
                        running = false;
                        break;
                    }
                    Err(error) => {
                        eprintln!("picocalc-preview: protocol error: {error}");
                        termination_error = Some(error);
                        running = false;
                        break;
                    }
                },
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    running = false;
                    break;
                }
            }
        }
        if !running {
            break;
        }

        if replay_finished {
            thread::sleep(Duration::from_millis(1));
            continue;
        }

        machine.refresh_sticky_stop();
        if machine.stopped().is_some() {
            // A stopped machine cannot advance virtual time.  Keep consuming
            // operator commands (notably reset/quit) without a hot spin.
            if machine.cycles() != last_status_cycle {
                write_status(
                    &mut output,
                    machine,
                    &stats.snapshot(),
                    frame_updates,
                    uart_tx_bytes,
                    uart_rx_accepted,
                    uart_rx_disabled,
                    uart_rx_overrun,
                    replay.as_ref(),
                )?;
                last_status_cycle = machine.cycles();
            }
            thread::sleep(Duration::from_millis(1));
            continue;
        }

        let previous_hz = machine.preview_sys_clk_hz();
        pacer.begin_quantum();
        let result = if let Some(replay_state) = replay.as_mut() {
            replay_state.step(machine)
        } else {
            let target = machine.cycles().saturating_add(pacer.quantum_cycles());
            machine
                .advance_once(target)
                .map(|(consumed, rate_changed)| ScenarioReplayStep::Advanced {
                    consumed,
                    rate_changed,
                })
                .map_err(|error| format!("preview emulation: {error}"))
        };
        let (consumed, rate_changed) = match result? {
            ScenarioReplayStep::Advanced {
                consumed,
                rate_changed,
            } => (consumed, rate_changed),
            ScenarioReplayStep::Complete => {
                replay_finished = true;
                pacer.end_quantum_for_cycles(0);
                if let Some(framebuffer) = machine.preview_framebuffer() {
                    let sha = framebuffer.rgb565_sha256();
                    if last_frame_sha.as_deref() != Some(sha.as_str()) {
                        write_frame(&mut output, machine.cycles(), &framebuffer)?;
                        last_frame_sha = Some(sha);
                        frame_updates = frame_updates.saturating_add(1);
                    }
                }
                write_status(
                    &mut output,
                    machine,
                    &stats.snapshot(),
                    frame_updates,
                    uart_tx_bytes,
                    uart_rx_accepted,
                    uart_rx_disabled,
                    uart_rx_overrun,
                    replay.as_ref(),
                )?;
                last_status_cycle = machine.cycles();
                continue;
            }
            ScenarioReplayStep::Failed(error) => {
                let _ = write_error(&mut output, "replay_failed", &error);
                return Err(error);
            }
        };
        pacer.end_quantum_for_cycles(consumed);
        if consumed == 0 {
            machine.mark_clock_stalled();
            write_error(&mut output, "clock_stalled", "emulator clock stopped")?;
            continue;
        }
        let current_hz = machine.preview_sys_clk_hz();
        if current_hz != previous_hz || rate_changed {
            pacer.update_sys_clk_hz(current_hz);
        }

        let new_uart = machine.drain_uart_new_with_cycles();
        let had_uart = !new_uart.is_empty();
        for (cycle, byte) in new_uart {
            let mut payload = Vec::with_capacity(9);
            payload.extend_from_slice(&cycle.to_le_bytes());
            payload.push(byte);
            output
                .write_bytes(Kind::UartTx, payload)
                .map_err(protocol_message)?;
            uart_tx_bytes = uart_tx_bytes.saturating_add(1);
        }

        quanta = quanta.saturating_add(1);
        if quanta.is_multiple_of(FRAME_POLL_QUANTA)
            && let Some(framebuffer) = machine.preview_framebuffer()
        {
            let sha = framebuffer.rgb565_sha256();
            if last_frame_sha.as_deref() != Some(sha.as_str()) {
                write_frame(&mut output, machine.cycles(), &framebuffer)?;
                last_frame_sha = Some(sha);
                frame_updates = frame_updates.saturating_add(1);
            }
        }
        if had_uart || quanta.is_multiple_of(FRAME_POLL_QUANTA) {
            write_status(
                &mut output,
                machine,
                &stats.snapshot(),
                frame_updates,
                uart_tx_bytes,
                uart_rx_accepted,
                uart_rx_disabled,
                uart_rx_overrun,
                replay.as_ref(),
            )?;
            last_status_cycle = machine.cycles();
        }
    }

    if let Some(error) = termination_error {
        return Err(error);
    }

    output
        .write_json(Kind::Goodbye, &json!({"reason": "quit"}))
        .map_err(protocol_message)?;
    Ok(())
}

fn spawn_input_reader(sender: SyncSender<InputMessage>) {
    thread::spawn(move || {
        let stdin = io::stdin();
        let mut reader = FrameReader::new(stdin.lock(), Direction::PreviewToRunner);
        let result = loop {
            match reader.read_frame() {
                Ok(Some(frame)) => {
                    if sender.send(Ok(Some(frame))).is_err() {
                        return;
                    }
                }
                Ok(None) => break Ok(None),
                Err(error) => break Err(error.to_string()),
            }
        };
        let _ = sender.send(result);
    });
}

#[allow(clippy::too_many_arguments)]
fn handle_input_frame<W: io::Write>(
    frame: Frame,
    machine: &mut MachineSession,
    output: &mut FrameWriter<W>,
    last_frame_sha: &mut Option<String>,
    frame_updates: &mut u64,
    uart_rx_accepted: &mut u64,
    uart_rx_disabled: &mut u64,
    uart_rx_overrun: &mut u64,
    running: &mut bool,
    replay_mode: bool,
) -> Result<(), String> {
    if replay_mode && frame.kind != Kind::Quit {
        write_error(
            output,
            "replay_input_rejected",
            "registered-target replay accepts only quit input",
        )?;
        return Ok(());
    }
    match frame.kind {
        Kind::KeyEvent => {
            let object = frame.json_value().map_err(protocol_message)?;
            let key = object
                .get("key")
                .and_then(Value::as_str)
                .unwrap_or_default();
            let state = object
                .get("state")
                .and_then(Value::as_str)
                .unwrap_or_default();
            match machine.preview_key_event(key, state) {
                Ok((_events, dropped)) => {
                    if dropped != 0 {
                        write_error(output, "keyboard_event_dropped", "keyboard queue is full")?;
                    }
                }
                Err(error) => write_error(output, "key_rejected", &error)?,
            }
        }
        Kind::UartRx => match machine.preview_uart_rx(frame.payload[0]) {
            rp2040_emu::peripherals::uart::UartRxResult::Accepted => {
                *uart_rx_accepted = uart_rx_accepted.saturating_add(1);
            }
            rp2040_emu::peripherals::uart::UartRxResult::Disabled => {
                *uart_rx_disabled = uart_rx_disabled.saturating_add(1);
                write_error(output, "uart_rx_disabled", "UART0 RX is not enabled")?;
            }
            rp2040_emu::peripherals::uart::UartRxResult::Overrun => {
                *uart_rx_overrun = uart_rx_overrun.saturating_add(1);
                write_error(output, "uart_rx_overrun", "UART0 RX FIFO is full")?;
            }
        },
        Kind::Reset => {
            machine.reset_for_preview()?;
            *last_frame_sha = None;
            if let Some(framebuffer) = machine.preview_framebuffer() {
                *last_frame_sha = Some(framebuffer.rgb565_sha256());
                write_frame(output, machine.cycles(), &framebuffer)?;
                *frame_updates = frame_updates.saturating_add(1);
            }
        }
        Kind::Quit => *running = false,
        other => {
            // FrameReader already rejects direction mismatches.  This branch
            // is kept defensive so a future Kind cannot silently mutate the
            // machine when this dispatcher is not updated with it.
            write_error(
                output,
                "unsupported_command",
                &format!("message kind {other:?} is not a preview command"),
            )?;
        }
    }
    Ok(())
}

fn write_frame<W: io::Write>(
    output: &mut FrameWriter<W>,
    cycle: u64,
    framebuffer: &picocalc_board::Framebuffer,
) -> Result<(), String> {
    let width = u16::try_from(framebuffer.width)
        .map_err(|_| "framebuffer width does not fit preview protocol".to_string())?;
    let height = u16::try_from(framebuffer.height)
        .map_err(|_| "framebuffer height does not fit preview protocol".to_string())?;
    let pixels = framebuffer.to_rgb565_le_bytes();
    let mut payload = Vec::with_capacity(12 + pixels.len());
    payload.extend_from_slice(&cycle.to_le_bytes());
    payload.extend_from_slice(&width.to_le_bytes());
    payload.extend_from_slice(&height.to_le_bytes());
    payload.extend_from_slice(&pixels);
    output
        .write_bytes(Kind::FrameRgb565, payload)
        .map_err(protocol_message)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn write_status<W: io::Write>(
    output: &mut FrameWriter<W>,
    machine: &MachineSession,
    snapshot: &PacerSnapshot,
    frame_updates: u64,
    uart_tx_bytes: u64,
    uart_rx_accepted: u64,
    uart_rx_disabled: u64,
    uart_rx_overrun: u64,
    replay: Option<&ScenarioReplay>,
) -> Result<(), String> {
    let virtual_ns = machine.elapsed_ns();
    let wall_ns = snapshot.wall_ns;
    let ratio_ppm = if wall_ns == 0 {
        0
    } else {
        ((virtual_ns as u128 * 1_000_000) / wall_ns as u128) as u64
    };
    let lag_ns =
        (virtual_ns as i128 - wall_ns as i128).clamp(i64::MIN as i128, i64::MAX as i128) as i64;
    let observation_projection = machine.preview_observation_projection();
    let observation_digest = machine.preview_observation_digest();
    let mut status = json!({
        "audio": {"queue_frames": 0, "state": "not_streamed"},
        "coverage": if machine.stopped().is_some() { "stopped" } else { "ok" },
        "framebuffer": {"updates": frame_updates},
        "observation": {
            "digest_sha256": observation_digest,
            "projection": observation_projection,
            "schema_version": PREVIEW_OBSERVATION_SCHEMA_VERSION,
        },
        "pacer": {
            "behind_count": snapshot.behind_count,
            "emulated_cycles": snapshot.emulated_cycles,
            "emulation_ns": snapshot.emulation_ns,
            "lag_ns": lag_ns,
            "ratio_ppm": ratio_ppm,
            "spin_ns": snapshot.spin_ns,
            "wall_ns": wall_ns,
        },
        "uart": {
            "rx_accepted": uart_rx_accepted,
            "rx_disabled": uart_rx_disabled,
            "rx_fifo": machine.preview_uart_rx_fifo_len(),
            "rx_overrun": uart_rx_overrun,
            "rx_raw_interrupt_status": machine.preview_uart_raw_status(),
            "tx_bytes": uart_tx_bytes,
        },
        "virtual_cycle": machine.cycles(),
        "virtual_ns": virtual_ns,
    });
    if let Some(replay) = replay {
        status
            .as_object_mut()
            .expect("preview status is an object")
            .insert("replay".into(), replay.status_json());
    }
    output
        .write_json(Kind::Status, &status)
        .map_err(protocol_message)?;
    Ok(())
}

fn write_error<W: io::Write>(
    output: &mut FrameWriter<W>,
    code: &str,
    message: &str,
) -> Result<(), String> {
    output
        .write_json(Kind::Error, &json!({"code": code, "message": message}))
        .map_err(protocol_message)?;
    Ok(())
}

fn protocol_message(error: ProtocolError) -> String {
    error.to_string()
}
