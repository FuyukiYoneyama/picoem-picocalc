//! Streaming observation of DMA-origin writes to the PicoCalc audio PWM CC register.
//!
//! This is a digital sample sink, not an analog PWM or speaker model. It observes the
//! value that the DMA engine actually commits to PWM slice 5 CC. CPU setup writes and
//! producer buffers are deliberately outside this boundary.

use sha2::{Digest, Sha256};

use crate::bus::PWM_BASE;

pub(crate) const PICOCALC_AUDIO_PWM_SLICE: usize = 5;
pub(crate) const PICOCALC_AUDIO_PWM_CC: u32 =
    PWM_BASE + PICOCALC_AUDIO_PWM_SLICE as u32 * 0x14 + 0x0c;
pub(crate) const PICOCALC_AUDIO_TIMER_INDEX: usize = 0;
pub(crate) const PICOCALC_AUDIO_TIMER_TREQ: u8 = 59;
pub(crate) const PICOCALC_AUDIO_HALF_FRAMES: u64 = 128;

const EDGE_WORDS: usize = 8;
const PWM_MAX_DUTY: u16 = 255;
const PCM_CHANNELS: u64 = 2;
const ANALYSIS_WINDOW_FRAMES: u64 = 1024;
const ACTIVE_ABS_THRESHOLD: u32 = 512;

/// Stable, clone-free report projection of the streaming audio sink.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AudioSinkSnapshot {
    pub status: &'static str,
    pub dma_write_count: u64,
    pub target_write_attempt_count: u64,
    pub other_pwm_cc_write_count: u64,
    pub wrong_width_count: u64,
    pub wrong_treq_count: u64,
    pub missing_due_cycle_count: u64,
    pub pcm_sha256: String,
    pub first_words: Vec<u32>,
    pub last_words: Vec<u32>,
    pub timer_index: usize,
    pub treq: u8,
    pub timer_fraction_x: u16,
    pub timer_fraction_y: u16,
    pub timer_event_count: u64,
    pub timer_miss_count: u64,
    pub timer_due_cycle_sha256: String,
    pub block_start_count: u64,
    pub malformed_block_count: u64,
    pub block_boundary_gap_count: u64,
    pub block_boundary_gap_min_cycles: Option<u64>,
    pub block_boundary_gap_max_cycles: Option<u64>,
    pub block_boundary_gap_sha256: String,
    pub gap_5208_count: u64,
    pub gap_5209_count: u64,
    pub unexpected_gap_count: u64,
    pub service_latency_min_cycles: Option<u64>,
    pub service_latency_max_cycles: Option<u64>,
    pub service_latency_sha256: String,
    pub analysis_frame_count: u64,
    pub sample_rate_hz: u32,
    pub channel_count: u8,
    pub reconstructed_pcm_format: &'static str,
    pub analysis_window_frames: u64,
    pub active_abs_threshold: u32,
    pub peak_abs_left: u32,
    pub peak_abs_right: u32,
    pub stream_rms: u32,
    pub max_window_rms: u32,
    pub dc_offset_left: i32,
    pub dc_offset_right: i32,
    pub active_frame_count: u64,
    pub active_frame_ratio_ppm: u64,
    pub rail_sample_count: u64,
    pub rail_sample_ratio_ppm: u64,
    pub max_consecutive_rail_frames: u64,
    pub out_of_range_duty_sample_count: u64,
}

pub(crate) struct AudioSink {
    target_write_attempt_count: u64,
    dma_write_count: u64,
    other_pwm_cc_write_count: u64,
    wrong_width_count: u64,
    wrong_treq_count: u64,
    missing_due_cycle_count: u64,
    pcm: Sha256,
    due_cycles: Sha256,
    block_boundary_gaps: Sha256,
    service_latencies: Sha256,
    first_words: Vec<u32>,
    last_words: [u32; EDGE_WORDS],
    last_words_count: usize,
    last_words_cursor: usize,
    last_due_cycle: Option<u64>,
    block_start_count: u64,
    block_word_count: u64,
    malformed_block_count: u64,
    block_boundary_gap_count: u64,
    block_boundary_gap_min_cycles: Option<u64>,
    block_boundary_gap_max_cycles: Option<u64>,
    gap_5208_count: u64,
    gap_5209_count: u64,
    unexpected_gap_count: u64,
    service_latency_min_cycles: Option<u64>,
    service_latency_max_cycles: Option<u64>,
    timer_fraction_x: u16,
    timer_fraction_y: u16,
    analysis_frame_count: u64,
    peak_abs_left: u32,
    peak_abs_right: u32,
    sum_square: u128,
    sum_left: i128,
    sum_right: i128,
    window_frame_count: u64,
    window_sum_square: u128,
    max_window_rms: u32,
    active_frame_count: u64,
    rail_sample_count: u64,
    consecutive_rail_frames: u64,
    max_consecutive_rail_frames: u64,
    out_of_range_duty_sample_count: u64,
    captured_pcm: Option<Vec<i16>>,
}

impl Default for AudioSink {
    fn default() -> Self {
        Self {
            target_write_attempt_count: 0,
            dma_write_count: 0,
            other_pwm_cc_write_count: 0,
            wrong_width_count: 0,
            wrong_treq_count: 0,
            missing_due_cycle_count: 0,
            pcm: Sha256::new(),
            due_cycles: Sha256::new(),
            block_boundary_gaps: Sha256::new(),
            service_latencies: Sha256::new(),
            first_words: Vec::with_capacity(EDGE_WORDS),
            last_words: [0; EDGE_WORDS],
            last_words_count: 0,
            last_words_cursor: 0,
            last_due_cycle: None,
            block_start_count: 0,
            block_word_count: 0,
            malformed_block_count: 0,
            block_boundary_gap_count: 0,
            block_boundary_gap_min_cycles: None,
            block_boundary_gap_max_cycles: None,
            gap_5208_count: 0,
            gap_5209_count: 0,
            unexpected_gap_count: 0,
            service_latency_min_cycles: None,
            service_latency_max_cycles: None,
            timer_fraction_x: 0,
            timer_fraction_y: 0,
            analysis_frame_count: 0,
            peak_abs_left: 0,
            peak_abs_right: 0,
            sum_square: 0,
            sum_left: 0,
            sum_right: 0,
            window_frame_count: 0,
            window_sum_square: 0,
            max_window_rms: 0,
            active_frame_count: 0,
            rail_sample_count: 0,
            consecutive_rail_frames: 0,
            max_consecutive_rail_frames: 0,
            out_of_range_duty_sample_count: 0,
            captured_pcm: None,
        }
    }
}

impl AudioSink {
    pub(crate) fn enable_pcm_capture(&mut self) {
        if self.captured_pcm.is_none() {
            self.captured_pcm = Some(Vec::new());
        }
    }

    pub(crate) fn take_pcm_capture(&mut self) -> Option<Vec<i16>> {
        self.captured_pcm.take()
    }

    #[inline]
    fn is_pwm_cc_address(address: u32) -> bool {
        (0..8).any(|slice| address == PWM_BASE + slice * 0x14 + 0x0c)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn observe_dma_write(
        &mut self,
        address: u32,
        width_bytes: u32,
        value: u32,
        treq: u8,
        timer_fraction: Option<(u16, u16)>,
        timer_due_cycle: Option<u64>,
        service_cycle: u64,
        block_start: bool,
    ) {
        if address != PICOCALC_AUDIO_PWM_CC {
            if Self::is_pwm_cc_address(address) {
                self.other_pwm_cc_write_count = self.other_pwm_cc_write_count.saturating_add(1);
            }
            return;
        }

        self.target_write_attempt_count = self.target_write_attempt_count.saturating_add(1);
        if width_bytes != 4 {
            self.wrong_width_count = self.wrong_width_count.saturating_add(1);
            return;
        }

        self.dma_write_count = self.dma_write_count.saturating_add(1);
        if block_start {
            if self.block_start_count != 0 && self.block_word_count != PICOCALC_AUDIO_HALF_FRAMES {
                self.malformed_block_count = self.malformed_block_count.saturating_add(1);
            }
            self.block_start_count = self.block_start_count.saturating_add(1);
            self.block_word_count = 0;
        } else if self.block_word_count == 0 {
            self.malformed_block_count = self.malformed_block_count.saturating_add(1);
        }
        self.block_word_count = self.block_word_count.saturating_add(1);
        if self.block_word_count == PICOCALC_AUDIO_HALF_FRAMES + 1 {
            self.malformed_block_count = self.malformed_block_count.saturating_add(1);
        }
        self.pcm.update(value.to_le_bytes());
        self.observe_audio_frame(value);
        if self.first_words.len() < EDGE_WORDS {
            self.first_words.push(value);
        }
        self.last_words[self.last_words_cursor] = value;
        self.last_words_cursor = (self.last_words_cursor + 1) % EDGE_WORDS;
        self.last_words_count = (self.last_words_count + 1).min(EDGE_WORDS);

        if treq != PICOCALC_AUDIO_TIMER_TREQ {
            self.wrong_treq_count = self.wrong_treq_count.saturating_add(1);
            return;
        }
        if let Some((x, y)) = timer_fraction {
            self.timer_fraction_x = x;
            self.timer_fraction_y = y;
        }
        let Some(due_cycle) = timer_due_cycle else {
            self.missing_due_cycle_count = self.missing_due_cycle_count.saturating_add(1);
            return;
        };

        self.due_cycles.update(due_cycle.to_le_bytes());
        if let Some(previous) = self.last_due_cycle {
            let gap = due_cycle.saturating_sub(previous);
            if block_start {
                self.block_boundary_gap_count = self.block_boundary_gap_count.saturating_add(1);
                self.block_boundary_gaps.update(gap.to_le_bytes());
                self.block_boundary_gap_min_cycles = Some(
                    self.block_boundary_gap_min_cycles
                        .map_or(gap, |current| current.min(gap)),
                );
                self.block_boundary_gap_max_cycles = Some(
                    self.block_boundary_gap_max_cycles
                        .map_or(gap, |current| current.max(gap)),
                );
            } else {
                match gap {
                    5208 => self.gap_5208_count = self.gap_5208_count.saturating_add(1),
                    5209 => self.gap_5209_count = self.gap_5209_count.saturating_add(1),
                    _ => {
                        self.unexpected_gap_count = self.unexpected_gap_count.saturating_add(1);
                    }
                }
            }
        }
        self.last_due_cycle = Some(due_cycle);

        let latency = service_cycle.saturating_sub(due_cycle);
        self.service_latencies.update(latency.to_le_bytes());
        self.service_latency_min_cycles = Some(
            self.service_latency_min_cycles
                .map_or(latency, |current| current.min(latency)),
        );
        self.service_latency_max_cycles = Some(
            self.service_latency_max_cycles
                .map_or(latency, |current| current.max(latency)),
        );
    }

    pub(crate) fn snapshot(&self) -> AudioSinkSnapshot {
        let structurally_valid = self.dma_write_count > 0
            && self.target_write_attempt_count == self.dma_write_count
            && self.wrong_width_count == 0
            && self.wrong_treq_count == 0
            && self.missing_due_cycle_count == 0
            && self.unexpected_gap_count == 0
            && self.malformed_block_count == 0
            && self.block_word_count == PICOCALC_AUDIO_HALF_FRAMES
            && self
                .block_start_count
                .saturating_mul(PICOCALC_AUDIO_HALF_FRAMES)
                == self.dma_write_count
            && self.block_boundary_gap_count == self.block_start_count.saturating_sub(1)
            && self.timer_fraction_x == 3
            && self.timer_fraction_y == 15625
            && self.out_of_range_duty_sample_count == 0;
        let status = if self.target_write_attempt_count == 0 {
            "inactive"
        } else if structurally_valid {
            "pass"
        } else {
            "fail"
        };

        let mut last_words = Vec::with_capacity(self.last_words_count);
        let start = if self.last_words_count == EDGE_WORDS {
            self.last_words_cursor
        } else {
            0
        };
        for offset in 0..self.last_words_count {
            last_words.push(self.last_words[(start + offset) % EDGE_WORDS]);
        }

        AudioSinkSnapshot {
            status,
            dma_write_count: self.dma_write_count,
            target_write_attempt_count: self.target_write_attempt_count,
            other_pwm_cc_write_count: self.other_pwm_cc_write_count,
            wrong_width_count: self.wrong_width_count,
            wrong_treq_count: self.wrong_treq_count,
            missing_due_cycle_count: self.missing_due_cycle_count,
            pcm_sha256: hex(self.pcm.clone().finalize().as_slice()),
            first_words: self.first_words.clone(),
            last_words,
            timer_index: PICOCALC_AUDIO_TIMER_INDEX,
            treq: PICOCALC_AUDIO_TIMER_TREQ,
            timer_fraction_x: self.timer_fraction_x,
            timer_fraction_y: self.timer_fraction_y,
            timer_event_count: 0,
            timer_miss_count: 0,
            timer_due_cycle_sha256: hex(self.due_cycles.clone().finalize().as_slice()),
            block_start_count: self.block_start_count,
            malformed_block_count: self.malformed_block_count,
            block_boundary_gap_count: self.block_boundary_gap_count,
            block_boundary_gap_min_cycles: self.block_boundary_gap_min_cycles,
            block_boundary_gap_max_cycles: self.block_boundary_gap_max_cycles,
            block_boundary_gap_sha256: hex(self.block_boundary_gaps.clone().finalize().as_slice()),
            gap_5208_count: self.gap_5208_count,
            gap_5209_count: self.gap_5209_count,
            unexpected_gap_count: self.unexpected_gap_count,
            service_latency_min_cycles: self.service_latency_min_cycles,
            service_latency_max_cycles: self.service_latency_max_cycles,
            service_latency_sha256: hex(self.service_latencies.clone().finalize().as_slice()),
            analysis_frame_count: self.analysis_frame_count,
            sample_rate_hz: 48_000,
            channel_count: PCM_CHANNELS as u8,
            reconstructed_pcm_format: "stereo_s16le_from_pwm8_duty",
            analysis_window_frames: ANALYSIS_WINDOW_FRAMES,
            active_abs_threshold: ACTIVE_ABS_THRESHOLD,
            peak_abs_left: self.peak_abs_left,
            peak_abs_right: self.peak_abs_right,
            stream_rms: rms(self.sum_square, self.analysis_frame_count),
            max_window_rms: if self.analysis_frame_count < ANALYSIS_WINDOW_FRAMES {
                rms(self.window_sum_square, self.window_frame_count)
            } else {
                self.max_window_rms
            },
            dc_offset_left: mean(self.sum_left, self.analysis_frame_count),
            dc_offset_right: mean(self.sum_right, self.analysis_frame_count),
            active_frame_count: self.active_frame_count,
            active_frame_ratio_ppm: ratio_ppm(self.active_frame_count, self.analysis_frame_count),
            rail_sample_count: self.rail_sample_count,
            rail_sample_ratio_ppm: ratio_ppm(
                self.rail_sample_count,
                self.analysis_frame_count.saturating_mul(PCM_CHANNELS),
            ),
            max_consecutive_rail_frames: self.max_consecutive_rail_frames,
            out_of_range_duty_sample_count: self.out_of_range_duty_sample_count,
        }
    }

    fn observe_audio_frame(&mut self, value: u32) {
        let left_duty = value as u16;
        let right_duty = (value >> 16) as u16;
        self.out_of_range_duty_sample_count = self
            .out_of_range_duty_sample_count
            .saturating_add(u64::from(left_duty > PWM_MAX_DUTY))
            .saturating_add(u64::from(right_duty > PWM_MAX_DUTY));

        let left = pcm_from_duty(left_duty);
        let right = pcm_from_duty(right_duty);
        let left_abs = left.unsigned_abs();
        let right_abs = right.unsigned_abs();
        self.peak_abs_left = self.peak_abs_left.max(left_abs);
        self.peak_abs_right = self.peak_abs_right.max(right_abs);

        let frame_square =
            (i128::from(left) * i128::from(left) + i128::from(right) * i128::from(right)) as u128;
        self.sum_square = self.sum_square.saturating_add(frame_square);
        self.window_sum_square = self.window_sum_square.saturating_add(frame_square);
        self.sum_left = self.sum_left.saturating_add(i128::from(left));
        self.sum_right = self.sum_right.saturating_add(i128::from(right));
        self.analysis_frame_count = self.analysis_frame_count.saturating_add(1);
        self.window_frame_count = self.window_frame_count.saturating_add(1);

        if left_abs >= ACTIVE_ABS_THRESHOLD || right_abs >= ACTIVE_ABS_THRESHOLD {
            self.active_frame_count = self.active_frame_count.saturating_add(1);
        }

        let left_rail = left_duty == 0 || left_duty == PWM_MAX_DUTY;
        let right_rail = right_duty == 0 || right_duty == PWM_MAX_DUTY;
        self.rail_sample_count = self
            .rail_sample_count
            .saturating_add(u64::from(left_rail))
            .saturating_add(u64::from(right_rail));
        if left_rail || right_rail {
            self.consecutive_rail_frames = self.consecutive_rail_frames.saturating_add(1);
            self.max_consecutive_rail_frames = self
                .max_consecutive_rail_frames
                .max(self.consecutive_rail_frames);
        } else {
            self.consecutive_rail_frames = 0;
        }

        if let Some(samples) = self.captured_pcm.as_mut() {
            samples.push(left as i16);
            samples.push(right as i16);
        }

        if self.window_frame_count == ANALYSIS_WINDOW_FRAMES {
            self.max_window_rms = self
                .max_window_rms
                .max(rms(self.window_sum_square, self.window_frame_count));
            self.window_frame_count = 0;
            self.window_sum_square = 0;
        }
    }
}

fn pcm_from_duty(duty: u16) -> i32 {
    i32::from(duty.min(PWM_MAX_DUTY)) * 257 - 32_768
}

fn rms(sum_square: u128, frames: u64) -> u32 {
    if frames == 0 {
        return 0;
    }
    let mean_square = sum_square / u128::from(frames.saturating_mul(PCM_CHANNELS));
    integer_sqrt(mean_square) as u32
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut current = value;
    let mut next = current.div_ceil(2);
    while next < current {
        current = next;
        next = (current + value / current) / 2;
    }
    current
}

fn mean(sum: i128, count: u64) -> i32 {
    if count == 0 {
        0
    } else {
        (sum / i128::from(count)).clamp(i128::from(i32::MIN), i128::from(i32::MAX)) as i32
    }
}

fn ratio_ppm(numerator: u64, denominator: u64) -> u64 {
    if denominator == 0 {
        0
    } else {
        (u128::from(numerator) * 1_000_000 / u128::from(denominator)) as u64
    }
}

fn hex(bytes: &[u8]) -> String {
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hashes_only_dma_writes_to_the_picocalc_audio_cc_register() {
        let mut sink = AudioSink::default();
        for index in 0..PICOCALC_AUDIO_HALF_FRAMES {
            let value = match index {
                0 => 0x00f8_0003,
                1 => 0x00db_0014,
                _ => index as u32,
            };
            let due = 5209 + index * 5208;
            sink.observe_dma_write(
                PICOCALC_AUDIO_PWM_CC,
                4,
                value,
                59,
                Some((3, 15625)),
                Some(due),
                due + if index == 0 { 2 } else { 1 },
                index == 0,
            );
        }
        sink.observe_dma_write(
            PWM_BASE + 0x0c,
            4,
            0xdead_beef,
            59,
            Some((3, 15625)),
            Some(15625),
            15625,
            true,
        );
        let snapshot = sink.snapshot();
        assert_eq!(snapshot.status, "pass");
        assert_eq!(snapshot.dma_write_count, PICOCALC_AUDIO_HALF_FRAMES);
        assert_eq!(snapshot.other_pwm_cc_write_count, 1);
        assert_eq!(&snapshot.first_words[..2], &[0x00f8_0003, 0x00db_0014]);
        assert_eq!(snapshot.last_words.len(), EDGE_WORDS);
        assert_eq!(snapshot.block_start_count, 1);
        assert_eq!(snapshot.block_boundary_gap_count, 0);
        assert_eq!(snapshot.gap_5208_count, PICOCALC_AUDIO_HALF_FRAMES - 1);
        assert_eq!(snapshot.gap_5209_count, 0);
        assert_eq!(snapshot.service_latency_min_cycles, Some(1));
        assert_eq!(snapshot.service_latency_max_cycles, Some(2));
    }

    #[test]
    fn wrong_width_treq_and_gap_fail_closed() {
        let mut sink = AudioSink::default();
        sink.observe_dma_write(
            PICOCALC_AUDIO_PWM_CC,
            2,
            0x12,
            59,
            Some((3, 15625)),
            Some(1),
            1,
            true,
        );
        sink.observe_dma_write(PICOCALC_AUDIO_PWM_CC, 4, 0x34, 58, None, None, 2, false);
        sink.observe_dma_write(
            PICOCALC_AUDIO_PWM_CC,
            4,
            0x56,
            59,
            Some((3, 15625)),
            Some(10),
            10,
            false,
        );
        sink.observe_dma_write(
            PICOCALC_AUDIO_PWM_CC,
            4,
            0x78,
            59,
            Some((3, 15625)),
            Some(20),
            20,
            false,
        );
        let snapshot = sink.snapshot();
        assert_eq!(snapshot.status, "fail");
        assert_eq!(snapshot.wrong_width_count, 1);
        assert_eq!(snapshot.wrong_treq_count, 1);
        assert_eq!(snapshot.unexpected_gap_count, 1);
    }

    #[test]
    fn measures_level_rail_runs_and_optional_pcm_without_normalising() {
        let mut sink = AudioSink::default();
        sink.enable_pcm_capture();
        for index in 0..PICOCALC_AUDIO_HALF_FRAMES {
            let due = 5208 + index * 5208;
            sink.observe_dma_write(
                PICOCALC_AUDIO_PWM_CC,
                4,
                0x00ff_0000,
                PICOCALC_AUDIO_TIMER_TREQ,
                Some((3, 15625)),
                Some(due),
                due,
                index == 0,
            );
        }

        let snapshot = sink.snapshot();
        assert_eq!(snapshot.status, "pass");
        assert_eq!(snapshot.analysis_frame_count, PICOCALC_AUDIO_HALF_FRAMES);
        assert_eq!(snapshot.peak_abs_left, 32_768);
        assert_eq!(snapshot.peak_abs_right, 32_767);
        assert_eq!(snapshot.stream_rms, 32_767);
        assert_eq!(snapshot.max_window_rms, 32_767);
        assert_eq!(snapshot.active_frame_ratio_ppm, 1_000_000);
        assert_eq!(snapshot.rail_sample_ratio_ppm, 1_000_000);
        assert_eq!(
            snapshot.max_consecutive_rail_frames,
            PICOCALC_AUDIO_HALF_FRAMES
        );

        let pcm = sink.take_pcm_capture().expect("capture was enabled");
        assert_eq!(pcm.len(), (PICOCALC_AUDIO_HALF_FRAMES * 2) as usize);
        assert_eq!(&pcm[..4], &[-32_768, 32_767, -32_768, 32_767]);
        assert!(sink.take_pcm_capture().is_none());
    }

    #[test]
    fn rejects_duty_values_outside_the_picocalc_eight_bit_contract() {
        let mut sink = AudioSink::default();
        for index in 0..PICOCALC_AUDIO_HALF_FRAMES {
            let due = 5208 + index * 5208;
            sink.observe_dma_write(
                PICOCALC_AUDIO_PWM_CC,
                4,
                if index == 0 { 0x0100_0080 } else { 0x0080_0080 },
                PICOCALC_AUDIO_TIMER_TREQ,
                Some((3, 15625)),
                Some(due),
                due,
                index == 0,
            );
        }
        let snapshot = sink.snapshot();
        assert_eq!(snapshot.status, "fail");
        assert_eq!(snapshot.out_of_range_duty_sample_count, 1);
    }

    #[test]
    fn a_trailing_partial_window_cannot_make_a_quiet_stream_look_loud() {
        let mut sink = AudioSink::default();
        for _ in 0..ANALYSIS_WINDOW_FRAMES {
            sink.observe_audio_frame(0x0080_0080);
        }
        sink.observe_audio_frame(0x00ff_0000);

        let snapshot = sink.snapshot();
        assert_eq!(snapshot.analysis_frame_count, ANALYSIS_WINDOW_FRAMES + 1);
        assert_eq!(snapshot.max_window_rms, 128);
        assert_eq!(snapshot.peak_abs_left, 32_768);
        assert_eq!(snapshot.peak_abs_right, 32_767);
    }
}
