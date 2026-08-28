//! Shared deterministic machine session used by every harness frontend.
//!
//! Keeping the emulator, board handles, boot handoff, UART accumulation, and
//! scheduler boundary in one module is intentional.  Batch scenarios,
//! machine API clients, and the realtime preview adapter must all observe the
//! same state transitions; a frontend-specific session would make their
//! reports incomparable.

use picocalc_board::sha256::sha256_hex;
use picocalc_board::{Framebuffer, KeyEvent, KeyState, pins};
use rp2040_emu::{Emulator, RP2040_SRAM_TOP, WatchdogResetEvent};
use serde_json::{Value, json};

use super::{
    BoardHandles, BootMode, RunOutcome, SDK_VTOR_FLASH_OFFSET, StopReason, UART_DRAIN_INTERVAL,
    fatal_exception_name, park_state,
};

/// One persistent, deterministic headless machine session.
///
/// This is the shared execution boundary for the batch scenario runner,
/// machine API, and preview API.  Keeping UART accumulation and the virtual
/// clock here prevents clients from inventing subtly different stepping,
/// clock-rebase, or observation semantics.
pub(crate) struct MachineSession {
    pub(crate) emu: Emulator,
    pub(crate) uart_bytes: Vec<u8>,
    pub(crate) dispatches: u64,
    pub(crate) board: BoardHandles,
    pub(crate) boot_mode: Option<BootMode>,
    pub(crate) watchdog_resets: Vec<WatchdogResetEvent>,
    /// When true, the preview loop owns UART draining so the cycle-rich tap
    /// cannot be consumed by the legacy periodic batch drain.
    preview_uart_cycle_tap: bool,
    pub(crate) sticky_stop: Option<SessionStop>,
    #[cfg(feature = "event-horizon-profiler")]
    pub(crate) event_profile_after_uart: Option<String>,
    #[cfg(feature = "event-horizon-profiler")]
    pub(crate) event_profile_start_cycle: Option<u64>,
}

/// Version of the read-only observation projection used by the preview
/// backend and the machine API.  This is deliberately separate from the
/// schema-8 batch report: changing the preview projection must not rewrite
/// historical validation report bytes.
pub(crate) const PREVIEW_OBSERVATION_SCHEMA_VERSION: u32 = 1;

#[derive(Clone)]
pub(crate) enum SessionStop {
    Exception(&'static str),
    Error(String),
}

impl MachineSession {
    #[inline]
    pub(crate) fn new(emu: Emulator, board: BoardHandles) -> Self {
        Self {
            emu,
            uart_bytes: Vec::new(),
            dispatches: 0,
            board,
            boot_mode: None,
            watchdog_resets: Vec::new(),
            preview_uart_cycle_tap: false,
            sticky_stop: None,
            #[cfg(feature = "event-horizon-profiler")]
            event_profile_after_uart: None,
            #[cfg(feature = "event-horizon-profiler")]
            event_profile_start_cycle: None,
        }
    }

    #[inline]
    pub(crate) fn set_boot_mode(&mut self, boot_mode: BootMode) {
        self.boot_mode = Some(boot_mode);
    }

    /// Opt in to the cycle-rich UART tap used by the preview wire. Keeping
    /// this explicit avoids adding a second unbounded event queue to normal
    /// batch/report runs.
    pub(crate) fn enable_preview_uart_cycle_tap(&mut self) {
        self.emu.enable_uart0_tx_cycle_tap();
        self.preview_uart_cycle_tap = true;
    }

    /// Re-enter the selected firmware handoff after an emulated watchdog
    /// bite.  The emulator has already performed the MCU warm reset and
    /// retained flash/SD/scratch; this method only selects the same entry
    /// path that was used for the initial run.
    pub(crate) fn handle_watchdog_reset(&mut self) -> Result<bool, String> {
        let Some(event) = self.emu.take_watchdog_reset_event() else {
            return Ok(false);
        };
        self.watchdog_resets.push(event);
        match self.boot_mode {
            Some(BootMode::Boot2FromFlash) => self
                .emu
                .boot2_from_flash(RP2040_SRAM_TOP, 0)
                .map_err(|error| {
                    format!("re-entering flash boot2 after watchdog reset: {error}")
                })?,
            Some(BootMode::DirectBootFromFlash) => {
                self.emu.direct_boot_from_flash(SDK_VTOR_FLASH_OFFSET);
            }
            Some(BootMode::BootromResetVector) | None => {}
        }
        Ok(true)
    }

    #[inline(always)]
    pub(crate) fn cycles(&self) -> u64 {
        self.emu.clock.cycles
    }

    #[inline(always)]
    pub(crate) fn elapsed_ns(&self) -> u64 {
        self.emu.bus.virtual_time_ns()
    }

    pub(crate) fn fatal_exception(&self) -> Option<&'static str> {
        self.emu
            .cores
            .iter()
            .enumerate()
            .find_map(|(core, state)| fatal_exception_name(core, state.regs.xpsr & 0x1FF))
    }

    pub(crate) fn refresh_sticky_stop(&mut self) {
        if self.sticky_stop.is_none()
            && let Some(exception) = self.fatal_exception()
        {
            self.sticky_stop = Some(SessionStop::Exception(exception));
        }
    }

    pub(crate) fn stopped(&self) -> Option<&SessionStop> {
        self.sticky_stop.as_ref()
    }

    #[inline(always)]
    pub(crate) fn drain_uart(&mut self) {
        let _ = self.drain_uart_new();
    }

    /// Drain the guest UART TX tap and return only bytes not previously
    /// exposed to a consumer. Batch and machine API callers keep using this
    /// byte-only path; the preview wire uses
    /// [`Self::drain_uart_new_with_cycles`] to attach virtual-time metadata.
    pub(crate) fn drain_uart_new(&mut self) -> Vec<u8> {
        // Keep the authoritative batch/machine path on the original byte
        // drain. The cycle-rich accessor intentionally clears the legacy
        // queue as part of its bounded diagnostic contract, so calling it
        // while the preview tap is disabled would silently discard UART
        // output from normal reports.
        let bytes = self.emu.drain_uart0_tx_log();
        self.uart_bytes.extend_from_slice(&bytes);
        #[cfg(feature = "event-horizon-profiler")]
        self.maybe_start_event_profile();
        bytes
    }

    /// Drain the guest UART TX tap with the exact bus cycle at which each
    /// UARTDR write happened. The byte-only report accumulation is updated in
    /// the same operation, so no consumer can observe two different streams.
    pub(crate) fn drain_uart_new_with_cycles(&mut self) -> Vec<(u64, u8)> {
        let events = self.emu.drain_uart0_tx_log_with_cycles();
        for &(_, byte) in &events {
            self.uart_bytes.push(byte);
        }
        #[cfg(feature = "event-horizon-profiler")]
        self.maybe_start_event_profile();
        events
    }

    #[cfg(feature = "event-horizon-profiler")]
    pub(crate) fn arm_event_profile_after_uart(&mut self, marker: String) {
        self.event_profile_after_uart = Some(marker);
    }

    #[cfg(feature = "event-horizon-profiler")]
    fn maybe_start_event_profile(&mut self) {
        let Some(marker) = self.event_profile_after_uart.as_deref() else {
            return;
        };
        if self
            .uart_bytes
            .windows(marker.len())
            .any(|window| window == marker.as_bytes())
        {
            self.emu
                .enable_running_event_profiler()
                .expect("deferred event profiler is enabled on Serial emulator");
            self.event_profile_start_cycle = Some(self.cycles());
            self.event_profile_after_uart = None;
        }
    }

    pub(crate) fn poll_scenario(&mut self, engine: &mut super::scenario::Engine) {
        self.drain_uart();
        engine.poll(&super::scenario::Observation {
            now_ns: self.elapsed_ns(),
            cycles: self.cycles(),
            lcd: self.board.lcd.as_deref(),
            keyboard: self.board.keyboard.as_deref(),
            uart: &self.uart_bytes,
        });
    }

    /// Execute one scheduler dispatch without crossing a proven idle
    /// external boundary. Returns `(cycles_consumed, clock_rate_changed)`.
    #[inline(always)]
    pub(crate) fn advance_once(
        &mut self,
        external_event_cycle: u64,
    ) -> Result<(u64, bool), String> {
        let previous_hz = self.emu.bus.clock_tree.sys_clk_hz;
        let consumed = self.emu.step_until(external_event_cycle).map_err(|error| {
            let message = error.to_string();
            self.sticky_stop = Some(SessionStop::Error(message.clone()));
            message
        })?;
        self.handle_watchdog_reset()?;
        self.dispatches = self.dispatches.saturating_add(1);
        if self.dispatches.is_multiple_of(UART_DRAIN_INTERVAL) && !self.preview_uart_cycle_tap {
            self.drain_uart();
        }
        let rate_changed = self.emu.bus.clock_tree.sys_clk_hz != previous_hz;
        Ok((consumed, rate_changed))
    }

    pub(crate) fn mark_clock_stalled(&mut self) {
        let detail = format!(
            "clock stalled: core0 {}, core1 {} — no wake source can fire while the master clock is frozen",
            park_state(&self.emu, 0),
            park_state(&self.emu, 1)
        );
        self.sticky_stop = Some(SessionStop::Error(detail));
    }

    pub(crate) fn finish(
        &mut self,
        stop_reason: StopReason,
        exception: Option<&'static str>,
        error: Option<String>,
    ) -> RunOutcome {
        self.drain_uart();
        RunOutcome {
            stop_reason,
            cycles: self.cycles(),
            elapsed_ns: self.elapsed_ns(),
            pc: self.emu.cores[0].regs.pc(),
            exception,
            error,
            uart_bytes: std::mem::take(&mut self.uart_bytes),
            watchdog_resets: std::mem::take(&mut self.watchdog_resets),
        }
    }

    /// Reset the serial machine for an interactive preview operator.  Flash
    /// contents and attached media remain in the emulator; CPU/peripheral
    /// state is reset and the originally selected boot handoff is replayed.
    /// This is deliberately separate from a guest watchdog reset, which is
    /// recorded in the authoritative run outcome.
    pub(crate) fn reset_for_preview(&mut self) -> Result<(), String> {
        self.emu.reset();
        self.enable_preview_uart_cycle_tap();
        self.uart_bytes.clear();
        self.dispatches = 0;
        self.watchdog_resets.clear();
        self.sticky_stop = None;
        if self.board.sd.is_some() {
            let detect = 1u32 << pins::SD_PIN_DETECT;
            self.emu.bus.external_gpio_in_mask |= detect;
            self.emu.bus.external_gpio_in_override &= !detect;
            self.emu.bus.gpio_in &= !detect;
        }
        match self.boot_mode {
            Some(BootMode::Boot2FromFlash) => self
                .emu
                .boot2_from_flash(RP2040_SRAM_TOP, 0)
                .map_err(|error| format!("re-entering flash boot2 after preview reset: {error}"))?,
            Some(BootMode::DirectBootFromFlash) => {
                self.emu.direct_boot_from_flash(SDK_VTOR_FLASH_OFFSET);
            }
            Some(BootMode::BootromResetVector) | None => {}
        }
        Ok(())
    }

    pub(crate) fn preview_framebuffer(&self) -> Option<Framebuffer> {
        let lcd = self.board.lcd.as_ref()?;
        Some(lcd.lock().expect("LCD model mutex").framebuffer())
    }

    pub(crate) fn preview_uart_rx(
        &mut self,
        byte: u8,
    ) -> rp2040_emu::peripherals::uart::UartRxResult {
        self.emu.inject_uart0_rx(byte)
    }

    pub(crate) fn preview_uart_rx_fifo_len(&self) -> usize {
        self.emu.uart0_rx_fifo_len()
    }

    pub(crate) fn preview_uart_raw_status(&self) -> u32 {
        self.emu.uart0_raw_interrupt_status()
    }

    pub(crate) fn preview_sys_clk_hz(&self) -> u32 {
        self.emu.bus.clock_tree.sys_clk_hz
    }

    /// Return the deterministic, provenance-free observation surface shared
    /// by preview and machine API consumers.  The projection intentionally
    /// contains only device-visible values; virtual cycle is carried by the
    /// surrounding status/response so callers can compare two projections at
    /// an explicitly chosen boundary.
    pub(crate) fn preview_observation_projection(&self) -> Value {
        let framebuffer = self.preview_framebuffer().map(|framebuffer| {
            json!({
                "height": framebuffer.height,
                "non_black_pixels": framebuffer.non_black_pixels(),
                "rgb565_sha256": framebuffer.rgb565_sha256(),
                "width": framebuffer.width,
            })
        });
        let unsupported = self
            .emu
            .bus
            .unsupported_mmio_log()
            .into_iter()
            .map(|(addr, pc, count)| {
                json!({
                    "addr": addr,
                    "count": count,
                    "pc": pc,
                })
            })
            .collect::<Vec<_>>();
        let audio = audio_observation_json(&self.emu.bus.audio_sink_snapshot());
        json!({
            "audio": audio,
            "framebuffer": framebuffer,
            "schema_version": PREVIEW_OBSERVATION_SCHEMA_VERSION,
            "uart": {
                "bytes": self.uart_bytes.len(),
                "sha256": sha256_hex(&self.uart_bytes),
            },
            "unsupported_mmio": {
                "entries": unsupported,
                "truncated": self.emu.bus.unsupported_mmio_log_truncated(),
            },
        })
    }

    /// Hash the canonical JSON encoding of [`Self::preview_observation_projection`].
    /// `serde_json::Map` uses sorted keys in this workspace, so the bytes and
    /// digest are stable across processes and hosts.
    pub(crate) fn preview_observation_digest(&self) -> String {
        let projection = self.preview_observation_projection();
        let canonical = serde_json::to_vec(&projection)
            .expect("preview observation JSON serialization is infallible");
        sha256_hex(&canonical)
    }

    pub(crate) fn preview_key_event(
        &mut self,
        key: &str,
        state: &str,
    ) -> Result<(usize, u64), String> {
        let code = match key {
            "Enter" => b'\r',
            "Escape" | "Esc" => 0x1b,
            "Space" => b' ',
            "Tab" => b'\t',
            "Backspace" => 0x08,
            value if value.chars().count() == 1 => {
                let character = value.chars().next().expect("one-character key");
                u8::try_from(character as u32)
                    .map_err(|_| format!("preview key {key:?} is not an 8-bit code"))?
            }
            _ => return Err(format!("unsupported preview key {key:?}")),
        };
        let state = match state {
            "down" => KeyState::Pressed,
            "held" => KeyState::Held,
            "up" => KeyState::Released,
            _ => return Err(format!("unsupported preview key state {state:?}")),
        };
        let keyboard = self
            .board
            .keyboard
            .as_ref()
            .ok_or_else(|| "preview key input requires --keyboard".to_string())?;
        let mut keyboard = keyboard
            .lock()
            .map_err(|_| "keyboard mutex poisoned".to_string())?;
        let dropped_before = keyboard.key_events_dropped;
        keyboard.push_event(KeyEvent { state, code });
        Ok((
            1,
            keyboard.key_events_dropped.saturating_sub(dropped_before),
        ))
    }
}

fn audio_observation_json(snapshot: &rp2040_emu::AudioSinkSnapshot) -> Value {
    // Keep every field that contributes to the digital DMA-to-PWM
    // observation.  The vectors are bounded edge samples, not the retained
    // PCM capture, so status frames remain bounded even for long runs.
    let mut value = serde_json::Map::new();
    value.insert(
        "active_abs_threshold".into(),
        json!(snapshot.active_abs_threshold),
    );
    value.insert(
        "active_frame_count".into(),
        json!(snapshot.active_frame_count),
    );
    value.insert(
        "active_frame_ratio_ppm".into(),
        json!(snapshot.active_frame_ratio_ppm),
    );
    value.insert(
        "analysis_frame_count".into(),
        json!(snapshot.analysis_frame_count),
    );
    value.insert(
        "analysis_window_frames".into(),
        json!(snapshot.analysis_window_frames),
    );
    value.insert(
        "block_boundary_gap_count".into(),
        json!(snapshot.block_boundary_gap_count),
    );
    value.insert(
        "block_boundary_gap_max_cycles".into(),
        json!(snapshot.block_boundary_gap_max_cycles),
    );
    value.insert(
        "block_boundary_gap_min_cycles".into(),
        json!(snapshot.block_boundary_gap_min_cycles),
    );
    value.insert(
        "block_boundary_gap_sha256".into(),
        json!(snapshot.block_boundary_gap_sha256),
    );
    value.insert("block_frame_max".into(), json!(snapshot.block_frame_max));
    value.insert("block_frame_min".into(), json!(snapshot.block_frame_min));
    value.insert(
        "block_start_count".into(),
        json!(snapshot.block_start_count),
    );
    value.insert("channel_count".into(), json!(snapshot.channel_count));
    value.insert("dc_offset_left".into(), json!(snapshot.dc_offset_left));
    value.insert("dc_offset_right".into(), json!(snapshot.dc_offset_right));
    value.insert("dma_write_count".into(), json!(snapshot.dma_write_count));
    value.insert("first_words".into(), json!(snapshot.first_words));
    value.insert("gap_5208_count".into(), json!(snapshot.gap_5208_count));
    value.insert("gap_5209_count".into(), json!(snapshot.gap_5209_count));
    value.insert("last_words".into(), json!(snapshot.last_words));
    value.insert(
        "malformed_block_count".into(),
        json!(snapshot.malformed_block_count),
    );
    value.insert(
        "max_consecutive_rail_frames".into(),
        json!(snapshot.max_consecutive_rail_frames),
    );
    value.insert("max_window_rms".into(), json!(snapshot.max_window_rms));
    value.insert(
        "missing_due_cycle_count".into(),
        json!(snapshot.missing_due_cycle_count),
    );
    value.insert(
        "other_pwm_cc_write_count".into(),
        json!(snapshot.other_pwm_cc_write_count),
    );
    value.insert(
        "out_of_range_duty_sample_count".into(),
        json!(snapshot.out_of_range_duty_sample_count),
    );
    value.insert("pcm_sha256".into(), json!(snapshot.pcm_sha256));
    value.insert("peak_abs_left".into(), json!(snapshot.peak_abs_left));
    value.insert("peak_abs_right".into(), json!(snapshot.peak_abs_right));
    value.insert(
        "rail_sample_count".into(),
        json!(snapshot.rail_sample_count),
    );
    value.insert(
        "rail_sample_ratio_ppm".into(),
        json!(snapshot.rail_sample_ratio_ppm),
    );
    value.insert(
        "reconstructed_pcm_format".into(),
        json!(snapshot.reconstructed_pcm_format),
    );
    value.insert("sample_rate_hz".into(), json!(snapshot.sample_rate_hz));
    value.insert(
        "service_latency_max_cycles".into(),
        json!(snapshot.service_latency_max_cycles),
    );
    value.insert(
        "service_latency_min_cycles".into(),
        json!(snapshot.service_latency_min_cycles),
    );
    value.insert(
        "service_latency_sha256".into(),
        json!(snapshot.service_latency_sha256),
    );
    value.insert("status".into(), json!(snapshot.status));
    value.insert("stream_rms".into(), json!(snapshot.stream_rms));
    value.insert(
        "target_write_attempt_count".into(),
        json!(snapshot.target_write_attempt_count),
    );
    value.insert(
        "timer_due_cycle_sha256".into(),
        json!(snapshot.timer_due_cycle_sha256),
    );
    value.insert(
        "timer_event_count".into(),
        json!(snapshot.timer_event_count),
    );
    value.insert("timer_fraction_x".into(), json!(snapshot.timer_fraction_x));
    value.insert("timer_fraction_y".into(), json!(snapshot.timer_fraction_y));
    value.insert("timer_index".into(), json!(snapshot.timer_index));
    value.insert(
        "timer_miss_audio_not_busy".into(),
        json!(snapshot.timer_miss_audio_not_busy),
    );
    value.insert("timer_miss_count".into(), json!(snapshot.timer_miss_count));
    value.insert(
        "timer_miss_multiple_due_in_window".into(),
        json!(snapshot.timer_miss_multiple_due_in_window),
    );
    value.insert(
        "timer_miss_no_dma_selected".into(),
        json!(snapshot.timer_miss_no_dma_selected),
    );
    value.insert(
        "timer_miss_other_dma_selected".into(),
        json!(snapshot.timer_miss_other_dma_selected),
    );
    value.insert("treq".into(), json!(snapshot.treq));
    value.insert(
        "unexpected_gap_count".into(),
        json!(snapshot.unexpected_gap_count),
    );
    value.insert("wrong_treq_count".into(), json!(snapshot.wrong_treq_count));
    value.insert(
        "wrong_width_count".into(),
        json!(snapshot.wrong_width_count),
    );
    Value::Object(value)
}
