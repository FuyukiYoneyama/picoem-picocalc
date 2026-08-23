//! End-to-end coverage for the promoted SD-GEN-1 runtime path.
//!
//! The fixture is deliberately a tiny, repository-owned Thumb program.  It
//! configures SPI0 and the real PicoCalc SD chip-select pin, performs a
//! CMD18 two-block read followed by CMD12 while CS remains asserted, then
//! performs a CMD23/CMD25 one-block write and CMD17 readback before emitting
//! a UART marker.  The test repeats the complete path three times and checks
//! the stable report projection, trace, and exported image.  It therefore
//! exercises the complete path (CPU ->
//! SIO/SPI0 -> `SdCardWire` -> default `SdCard` feature) rather than calling
//! the card state machine directly.

#![cfg(feature = "sd-gen1-multiblock")]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;

const XIP_BASE: u32 = 0x1000_0000;
const VTOR_OFFSET: usize = 0x100;
const CODE_OFFSET: usize = 0x108;

/// Small Thumb-1 emitter using only instructions implemented by the
/// emulator's raw-flash fixture tests.  Addresses are synthesized from
/// their high byte plus 12-bit page offset, so the long transfer stream does
/// not depend on a literal pool or on an ARM cross compiler.
struct Program {
    words: Vec<u16>,
}

impl Program {
    fn new() -> Self {
        Self { words: Vec::new() }
    }

    fn push(&mut self, word: u16) {
        self.words.push(word);
    }

    fn movs(&mut self, register: u8, value: u8) {
        assert!(register < 8);
        self.push(0x2000 | (u16::from(register) << 8) | u16::from(value));
    }

    fn lsls(&mut self, register: u8, shift: u8) {
        assert!(register < 8 && shift < 32);
        self.push((u16::from(shift) << 6) | (u16::from(register) << 3) | u16::from(register));
    }

    fn adds_imm(&mut self, register: u8, value: u8) {
        assert!(register < 8);
        self.push(0x3000 | (u16::from(register) << 8) | u16::from(value));
    }

    fn adds_reg(&mut self, destination: u8, left: u8, right: u8) {
        assert!(destination < 8 && left < 8 && right < 8);
        self.push(
            0x1800 | (u16::from(right) << 6) | (u16::from(left) << 3) | u16::from(destination),
        );
    }

    fn str_word(&mut self, value: u8, address: u8) {
        assert!(value < 8 && address < 8);
        self.push(0x6000 | (u16::from(address) << 3) | u16::from(value));
    }

    fn ldr_word(&mut self, destination: u8, address: u8) {
        assert!(destination < 8 && address < 8);
        self.push(0x6800 | (u16::from(address) << 3) | u16::from(destination));
    }

    fn nops(&mut self, count: usize) {
        self.words.extend(std::iter::repeat_n(0x46c0, count));
    }

    fn load_page_address(&mut self, destination: u8, high_byte: u8, page: u8) {
        // destination = (high_byte << 24) + (page << 12), using r0 as
        // scratch.  All destination registers used by this fixture are r4-
        // r7, leaving r0/r1 available for the construction and stores.
        assert!(destination >= 4);
        self.movs(destination, high_byte);
        self.lsls(destination, 24);
        self.movs(0, page);
        self.lsls(0, 12);
        self.adds_reg(destination, destination, 0);
    }

    fn write_byte(&mut self, address: u8, value: u8) {
        // SPI0 shifts one 8-bit frame in 16 sysclks at the fixture's
        // prescale.  The delay lets the TX word reach the off-chip model;
        // the load drains the corresponding RX byte and prevents a PL022
        // overrun from hiding the SD transaction.
        self.movs(1, value);
        self.str_word(1, address);
        self.nops(16);
        self.ldr_word(2, address);
    }

    fn finish(mut self) -> Vec<u8> {
        self.push(0xe7fe); // forever loop after the marker
        let mut image = vec![0u8; CODE_OFFSET + self.words.len() * 2];
        image[VTOR_OFFSET..VTOR_OFFSET + 4].copy_from_slice(&0x2000_4000u32.to_le_bytes());
        image[VTOR_OFFSET + 4..VTOR_OFFSET + 8]
            .copy_from_slice(&(XIP_BASE + CODE_OFFSET as u32 + 1).to_le_bytes());
        for (index, word) in self.words.into_iter().enumerate() {
            let offset = CODE_OFFSET + index * 2;
            image[offset..offset + 2].copy_from_slice(&word.to_le_bytes());
        }
        image
    }
}

fn multiblock_fixture_image() -> Vec<u8> {
    let mut program = Program::new();

    // Release SPI0 and UART0 reset gates independently through RESETS_CLR.
    program.load_page_address(5, 0x40, 0x0f);
    program.movs(1, 1);
    program.lsls(1, 16);
    program.str_word(1, 5);
    program.movs(1, 1);
    program.lsls(1, 22);
    program.str_word(1, 5);

    // SPI0: 8-bit frames, SSE, and the minimum legal even prescale.
    program.load_page_address(6, 0x40, 0x3c);
    program.movs(1, 7);
    program.str_word(1, 6); // SSPCR0
    program.adds_imm(6, 4);
    program.movs(1, 2);
    program.str_word(1, 6); // SSPCR1.SSE
    program.adds_imm(6, 12);
    program.movs(1, 2);
    program.str_word(1, 6); // SSPCPSR
    program.load_page_address(6, 0x40, 0x3c);
    program.adds_imm(6, 8); // SSPDR

    // SIO output-enable and idle-high CS.
    program.load_page_address(4, 0xd0, 0x00);
    program.adds_imm(4, 0x24); // GPIO_OE_SET
    program.movs(1, 1);
    program.lsls(1, 17);
    program.str_word(1, 4);
    program.load_page_address(4, 0xd0, 0x00);
    program.adds_imm(4, 0x14); // GPIO_OUT_SET
    program.str_word(1, 4);

    // Select the card (CS low) and issue CMD18, block 3.
    program.load_page_address(4, 0xd0, 0x00);
    program.adds_imm(4, 0x18); // GPIO_OUT_CLR
    program.str_word(1, 4); // r1 still holds the CS mask
    for byte in [0x52, 0x00, 0x00, 0x00, 0x03, 0x01] {
        program.write_byte(6, byte);
    }
    // The first block (block 3) is queued with the CMD18 response.  Eight
    // clocks consume its response prefix; the remaining 509 clocks finish
    // it, one clock requests block 4, and 515 clocks consume block 4.
    for _ in 0..8 {
        program.write_byte(6, 0xff);
    }
    for _ in 0..1025 {
        program.write_byte(6, 0xff);
    }

    // CMD12 is framed while CS remains low, as on the real SPI card.
    for byte in [0x4c, 0x00, 0x00, 0x00, 0x00, 0x01] {
        program.write_byte(6, byte);
    }
    for _ in 0..8 {
        program.write_byte(6, 0xff);
    }

    // Start a fresh CS epoch for the multi-block write.  CMD23 and CMD25
    // have only the two-byte [idle, R1] response in this model; clocking
    // beyond it would correctly be treated as an invalid data token.
    program.load_page_address(4, 0xd0, 0x00);
    program.adds_imm(4, 0x14); // GPIO_OUT_SET
    program.movs(1, 1);
    program.lsls(1, 17);
    program.str_word(1, 4);
    program.load_page_address(4, 0xd0, 0x00);
    program.adds_imm(4, 0x18); // GPIO_OUT_CLR
    program.str_word(1, 4);

    // ACMD23-style pre-erase count followed by CMD25 at block 6.
    for byte in [0x57, 0x00, 0x00, 0x00, 0x01, 0x01] {
        program.write_byte(6, byte);
    }
    for _ in 0..2 {
        program.write_byte(6, 0xff);
    }
    for byte in [0x59, 0x00, 0x00, 0x00, 0x06, 0x01] {
        program.write_byte(6, byte);
    }
    for _ in 0..2 {
        program.write_byte(6, 0xff);
    }
    program.write_byte(6, 0xfc); // multi-block data token
    for _ in 0..512 {
        program.write_byte(6, 0xa5);
    }
    program.write_byte(6, 0xff); // CRC high
    program.write_byte(6, 0xff); // CRC low
    program.write_byte(6, 0xff); // data-accepted token
    program.write_byte(6, 0xff); // busy byte
    program.write_byte(6, 0xfd); // stop transmission token

    // Read the written block through the existing single-block path.  The
    // runner exports the COW-backed RAW image after the run and the test
    // below checks that the readback source is byte-for-byte A5.
    for byte in [0x51, 0x00, 0x00, 0x00, 0x06, 0x01] {
        program.write_byte(6, byte);
    }
    for _ in 0..517 {
        program.write_byte(6, 0xff);
    }

    // Deselect before reporting completion.
    program.load_page_address(4, 0xd0, 0x00);
    program.adds_imm(4, 0x14); // GPIO_OUT_SET
    program.movs(1, 1);
    program.lsls(1, 17);
    program.str_word(1, 4);

    // Emit a marker that the runner can use as the app-level completion
    // condition.
    program.load_page_address(7, 0x40, 0x34);
    program.adds_imm(7, 0x30); // UART0_UARTCR
    program.movs(1, 0x01);
    program.lsls(1, 8);
    program.adds_imm(1, 0x01);
    program.str_word(1, 7); // UARTEN|TXE (the fixture only needs TX)
    program.load_page_address(7, 0x40, 0x34);
    for byte in b"SD_MB_FIXTURE\n" {
        program.movs(1, *byte);
        program.str_word(1, 7);
        program.nops(16);
    }
    program.finish()
}

fn bootrom_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms/rp2040/bootrom-rp2040-b2.bin")
}

fn temp_dir() -> PathBuf {
    let path = std::env::temp_dir().join(format!(
        "picocalc-harness-cli-sd-multiblock-{}-{}",
        std::process::id(),
        std::thread::current().name().unwrap_or("test")
    ));
    let _ = fs::remove_dir_all(&path);
    fs::create_dir_all(&path).expect("create isolated CLI fixture directory");
    path
}

fn run_fixture(directory: &Path) -> (Value, Value, Vec<u8>) {
    fs::create_dir_all(directory).expect("create isolated CLI run directory");
    let firmware = directory.join("sd-multiblock-fixture.bin");
    let input_image = directory.join("input.img");
    let output_image = directory.join("output.img");
    let report_path = directory.join("report.json");
    let trace_path = directory.join("sd-trace.json");
    fs::write(&firmware, multiblock_fixture_image()).expect("write generated raw firmware");
    fs::write(&input_image, vec![0u8; 8 * 512]).expect("write isolated RAW SD image");

    let output = Command::new(env!("CARGO_BIN_EXE_picocalc-run"))
        .args([
            "--bin",
            firmware.to_str().unwrap(),
            "--bootrom",
            bootrom_path().to_str().unwrap(),
            "--board",
            "picocalc",
            "--sd-image",
            input_image.to_str().unwrap(),
            "--sd-image-out",
            output_image.to_str().unwrap(),
            "--sd-trace",
            trace_path.to_str().unwrap(),
            "--cycles",
            "100000",
            "--expect-stop",
            "cycle_limit",
            "--expect-uart",
            "SD_MB_FIXTURE",
            "--json",
            report_path.to_str().unwrap(),
        ])
        .output()
        .expect("run picocalc-run CLI");
    assert!(
        output.status.success(),
        "CLI failed: status={:?}\nstdout={}\nstderr={}",
        output.status,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );

    let report: Value = serde_json::from_slice(&fs::read(&report_path).expect("read report"))
        .expect("parse report");
    assert_eq!(report["schema_version"], 8);
    assert_eq!(report["verdict"]["status"], "pass");
    assert_eq!(report["sd"]["protocol_errors"], serde_json::json!([]));
    assert_eq!(report["sd"]["unknown_commands"], serde_json::json!([]));
    assert_eq!(report["sd"]["commands_seen"], 5);
    assert_eq!(report["sd"]["blocks_read"], 3);
    assert_eq!(report["sd"]["blocks_written"], 1);

    let exported = fs::read(&output_image).expect("read exported RAW SD image");
    assert_eq!(exported.len(), 8 * 512);
    assert!(exported[6 * 512..7 * 512].iter().all(|byte| *byte == 0xa5));

    let trace: Value = serde_json::from_slice(&fs::read(&trace_path).expect("read SD trace"))
        .expect("parse SD trace");
    assert_eq!(trace["schema_version"], 1);
    assert!(trace["event_count"].as_u64().unwrap_or(0) >= 4);
    assert!(!trace["digest_sha256"].as_str().unwrap_or("").is_empty());

    (report, trace, exported)
}

fn stable_report_projection(report: &Value) -> Value {
    serde_json::json!({
        "backend_build": report["backend_build"],
        "firmware": report["firmware"],
        "cycles": report["cycles"],
        "stop_reason": report["stop_reason"],
        "verdict": report["verdict"],
        "sd": report["sd"],
        "uart": report["uart"],
    })
}

#[test]
fn default_runtime_executes_multiblock_read_write_and_readback_without_protocol_errors() {
    let directory = temp_dir();
    let mut baseline_projection = None;
    let mut baseline_trace = None;
    let mut baseline_export = None;

    for iteration in 0..3 {
        let run_directory = directory.join(format!("run-{iteration}"));
        let (report, trace, exported) = run_fixture(&run_directory);
        let projection = stable_report_projection(&report);

        if let Some(expected) = baseline_projection.as_ref() {
            assert_eq!(expected, &projection, "stable report projection changed");
        } else {
            baseline_projection = Some(projection);
        }
        if let Some(expected) = baseline_trace.as_ref() {
            assert_eq!(expected, &trace, "structured SD trace changed");
        } else {
            baseline_trace = Some(trace);
        }
        if let Some(expected) = baseline_export.as_ref() {
            assert_eq!(expected, &exported, "exported SD image changed");
        } else {
            baseline_export = Some(exported);
        }
    }

    if std::env::var_os("PICOCALC_P4_KEEP").is_none() {
        let _ = fs::remove_dir_all(directory);
    }
}
