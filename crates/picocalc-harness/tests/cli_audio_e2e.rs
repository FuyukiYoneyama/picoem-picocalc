//! End-to-end coverage for the public board-less audio CLI path.
//!
//! The fixture is generated from original Thumb-1 instructions instead of a
//! checked-in binary.  This keeps the test hermetic and makes the provenance
//! of the executable input reviewable.  It is intentionally a real raw flash
//! image: the test exercises argument parsing, direct boot, DMA/PWM capture,
//! schema-8 report generation, and WAV output through `picocalc-run`.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;

const XIP_BASE: u32 = 0x1000_0000;
const VTOR_OFFSET: usize = 0x100;
const CODE_OFFSET: usize = 0x108;
const SRAM_SOURCE: u32 = 0x2000_1000;
const RESETS_CLR: u32 = 0x4000_f000;
const UART0_BASE: u32 = 0x4003_4000;
const PWM_BASE: u32 = 0x4005_0000;
const PWM5_CSR: u32 = PWM_BASE + 5 * 0x14;
const PWM5_CC: u32 = PWM5_CSR + 0x0c;
const PWM5_TOP: u32 = PWM5_CSR + 0x10;
const DMA_BASE: u32 = 0x5000_0000;
const DMA_TIMER0: u32 = DMA_BASE + 0x420;
const DMA_CH0: u32 = DMA_BASE;
const RESET_DMA: u32 = 1 << 2;
const RESET_PWM: u32 = 1 << 14;
const RESET_TIMER: u32 = 1 << 21;
const RESET_UART0: u32 = 1 << 22;
const DMA_CTRL_TIMER_32BIT_INCR_READ: u32 = 1 | (2 << 2) | (1 << 4) | (59 << 15);

struct ThumbProgram {
    words: Vec<u16>,
    literals: Vec<u32>,
    loads: Vec<(usize, u8, usize)>,
}

impl ThumbProgram {
    fn new() -> Self {
        Self {
            words: Vec::new(),
            literals: Vec::new(),
            loads: Vec::new(),
        }
    }

    fn literal(&mut self, value: u32) -> usize {
        let index = self.literals.len();
        self.literals.push(value);
        index
    }

    fn load_literal(&mut self, register: u8, literal: usize) {
        let word_index = self.words.len();
        self.words.push(0);
        self.loads.push((word_index, register, literal));
    }

    fn write32(&mut self, address: u32, value: u32) {
        let address_literal = self.literal(address);
        let value_literal = self.literal(value);
        self.load_literal(0, address_literal);
        self.load_literal(1, value_literal);
        // STR r1, [r0, #0] (Thumb-1 encoding).
        self.words.push(0x6001);
    }

    fn finish(mut self) -> Vec<u8> {
        // Stop execution before the literal pool.  The branch target is the
        // branch itself (PC+2 plus signed zero in this encoding).
        self.words.push(0xe7fe);
        while self.words.len() % 2 != 0 {
            self.words.push(0x46c0); // NOP
        }
        let pool_offset = CODE_OFFSET + self.words.len() * 2;
        let pool_address = XIP_BASE + pool_offset as u32;
        for (word_index, register, literal_index) in self.loads {
            let instruction_address = XIP_BASE + (CODE_OFFSET + word_index * 2) as u32;
            let pc = (instruction_address + 4) & !3;
            let target = pool_address + (literal_index * 4) as u32;
            let distance = target
                .checked_sub(pc)
                .expect("fixture literal pool must follow code");
            assert!(distance.is_multiple_of(4));
            let immediate = distance / 4;
            assert!(immediate <= 0xff, "literal pool outside Thumb-1 LDR range");
            self.words[word_index] = 0x4800 | (u16::from(register) << 8) | immediate as u16;
        }

        let mut image = vec![0u8; pool_offset + self.literals.len() * 4];
        image[VTOR_OFFSET..VTOR_OFFSET + 4].copy_from_slice(&0x2000_4000u32.to_le_bytes());
        image[VTOR_OFFSET + 4..VTOR_OFFSET + 8]
            .copy_from_slice(&(XIP_BASE + CODE_OFFSET as u32 + 1).to_le_bytes());
        for (index, word) in self.words.into_iter().enumerate() {
            let offset = CODE_OFFSET + index * 2;
            image[offset..offset + 2].copy_from_slice(&word.to_le_bytes());
        }
        for (index, literal) in self.literals.into_iter().enumerate() {
            let offset = pool_offset + index * 4;
            image[offset..offset + 4].copy_from_slice(&literal.to_le_bytes());
        }
        image
    }
}

fn audio_fixture_image() -> Vec<u8> {
    let mut program = ThumbProgram::new();
    // Release DMA, TIMER, UART0, and PWM.  The fixture uses the same reset
    // clear alias that firmware uses; the low-level bit assignments are
    // pinned in rp2040-emu's peripheral dispatch table.
    program.write32(
        RESETS_CLR,
        RESET_DMA | RESET_TIMER | RESET_UART0 | RESET_PWM,
    );
    program.write32(UART0_BASE + 0x30, 0x101); // UARTEN | TXE
    for byte in b"AUDIO_FIXTURE\n" {
        program.write32(UART0_BASE, u32::from(*byte));
    }

    // Enable PWM slice 5 and establish a legal 8-bit duty range.  The audio
    // sink observes DMA-origin writes to CC; the CPU setup write is outside
    // the captured stream by design.
    program.write32(PWM5_CSR, 1);
    program.write32(PWM5_TOP, 255);
    program.write32(PWM5_CC, 0x0080_0080);

    // Four stereo duty words in SRAM.  They are deliberately non-rail values
    // so the test exercises PCM reconstruction rather than only silence.
    for (index, value) in [0x0040_00c0, 0x0080_0080, 0x00c0_0040, 0x00a0_0060]
        .into_iter()
        .enumerate()
    {
        program.write32(SRAM_SOURCE + (index as u32 * 4), value);
    }

    // A 1/295 timer fraction is intentionally not one of the frozen 48 kHz
    // values.  The report and WAV must carry the observed rate, not a fixed
    // constant.  The DMA destination is fixed at PWM5 CC and the source
    // increments through the four stereo words.
    program.write32(DMA_TIMER0, (1 << 16) | 295);
    program.write32(DMA_CH0, SRAM_SOURCE);
    program.write32(DMA_CH0 + 4, PWM5_CC);
    program.write32(DMA_CH0 + 8, 4);
    program.write32(DMA_CH0 + 0x0c, DMA_CTRL_TIMER_32BIT_INCR_READ);
    program.finish()
}

fn bootrom_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms/rp2040/bootrom-rp2040-b2.bin")
}

fn temp_dir() -> PathBuf {
    let path = std::env::temp_dir().join(format!(
        "picocalc-harness-cli-audio-{}-{}",
        std::process::id(),
        std::thread::current().name().unwrap_or("test")
    ));
    let _ = fs::remove_dir_all(&path);
    fs::create_dir_all(&path).expect("create isolated CLI fixture directory");
    path
}

fn run_cli(firmware: &Path, output_dir: &Path, extra: &[&str]) -> std::process::Output {
    let report = output_dir.join("report.json");
    let analysis = output_dir.join("audio-analysis.json");
    let wav = output_dir.join("audio.wav");
    let mut command = Command::new(env!("CARGO_BIN_EXE_picocalc-run"));
    command.args([
        "--bin",
        firmware.to_str().unwrap(),
        "--bootrom",
        bootrom_path().to_str().unwrap(),
        "--board",
        "none",
        "--cycles",
        "5000",
        "--expect-stop",
        "cycle_limit",
        "--expect-uart",
        "AUDIO_FIXTURE",
        "--json",
        report.to_str().unwrap(),
        "--audio-analysis",
        analysis.to_str().unwrap(),
        "--audio-wav",
        wav.to_str().unwrap(),
    ]);
    command.args(extra);
    command.output().expect("run picocalc-run CLI")
}

#[test]
fn boardless_audio_cli_produces_observed_rate_report_and_wav() {
    let directory = temp_dir();
    let firmware = directory.join("audio-fixture.bin");
    fs::write(&firmware, audio_fixture_image()).expect("write generated raw firmware");

    let output = run_cli(&firmware, &directory, &[]);
    assert!(
        output.status.success(),
        "CLI failed: status={:?}\nstdout={}\nstderr={}",
        output.status,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );

    let report: Value = serde_json::from_slice(
        &fs::read(directory.join("report.json")).expect("read schema-8 report"),
    )
    .expect("parse schema-8 report");
    let analysis: Value = serde_json::from_slice(
        &fs::read(directory.join("audio-analysis.json")).expect("read audio analysis"),
    )
    .expect("parse audio analysis");
    let wav = fs::read(directory.join("audio.wav")).expect("read generated WAV");

    assert_eq!(report["schema_version"], 8);
    assert_eq!(report["verdict"]["status"], "pass");
    assert_eq!(report["audio_sink"]["status"], "pass");
    assert_eq!(report["audio_sink"]["dma_write_count"], 4);
    assert!(report["audio_sink"]["timer_miss_count"].is_number());
    assert!(report["audio_sink"]["timer_miss_audio_not_busy"].is_number());
    assert!(report["audio_sink"]["timer_miss_no_dma_selected"].is_number());
    let timer_misses = [
        "timer_miss_count",
        "timer_miss_audio_not_busy",
        "timer_miss_other_dma_selected",
        "timer_miss_no_dma_selected",
        "timer_miss_multiple_due_in_window",
    ]
    .into_iter()
    .map(|field| {
        report["audio_sink"][field]
            .as_u64()
            .expect("timer miss field is an integer")
    })
    .sum::<u64>();
    assert!(timer_misses > 0, "fixture must exercise a timer miss");
    assert_eq!(analysis["schema_version"], 2);
    assert_eq!(analysis["observation_status"], "pass");
    assert_eq!(analysis["pcm_sha256"], report["audio_sink"]["pcm_sha256"]);
    assert_eq!(
        analysis["sample_rate_hz"],
        report["audio_sink"]["sample_rate_hz"]
    );
    assert_ne!(analysis["sample_rate_hz"], 48_000);
    assert_eq!(&wav[0..4], b"RIFF");
    assert_eq!(&wav[8..12], b"WAVE");
    assert_eq!(&wav[12..16], b"fmt ");
    assert_eq!(
        u32::from_le_bytes(wav[24..28].try_into().unwrap()),
        analysis["sample_rate_hz"]
    );
    assert_eq!(&wav[36..40], b"data");
    assert_eq!(
        u32::from_le_bytes(wav[40..44].try_into().unwrap()) as usize,
        wav.len() - 44
    );
    assert_eq!(wav.len() - 44, 4 * 2 * 2); // four stereo s16le frames

    let _ = fs::remove_dir_all(directory);
}

#[cfg(feature = "event-horizon-profiler")]
#[test]
fn missing_uart_marker_fails_closed_before_profile_artifact() {
    let directory = temp_dir();
    let firmware = directory.join("audio-fixture.bin");
    fs::write(&firmware, audio_fixture_image()).expect("write generated raw firmware");
    let profile = directory.join("event-horizon.json");
    let output = run_cli(
        &firmware,
        &directory,
        &[
            "--event-horizon-profile",
            profile.to_str().unwrap(),
            "--event-horizon-profile-after-uart",
            "NEVER_PRESENT",
        ],
    );
    assert!(!output.status.success(), "missing marker must not pass");
    assert!(
        String::from_utf8_lossy(&output.stderr).contains("marker was not observed"),
        "unexpected stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        !profile.exists(),
        "failed profile must not leave an artifact"
    );
    let _ = fs::remove_dir_all(directory);
}
