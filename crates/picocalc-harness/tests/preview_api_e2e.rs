//! End-to-end coverage for the VRP-2 framed preview backend.
//!
//! The test speaks the frozen local IPC wire directly.  It intentionally uses
//! the repository-owned UART fixture instead of a GUI so that the runner's
//! process boundary, direction/sequence checks, UART input result, and clean
//! quit behavior are exercised without a WSLg dependency.

use std::fs;
use std::io::{BufRead, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::time::{SystemTime, UNIX_EPOCH};

use picocalc_board::sha256::sha256_hex;
use serde_json::Value;

const MAGIC: &[u8; 4] = b"PCRP";
const VERSION: u16 = 1;
const HELLO: u16 = 1;
const STATUS: u16 = 2;
const FRAME_RGB565: u16 = 3;
const RESET: u16 = 6;
const UART_TX: u16 = 8;
const UART_RX: u16 = 9;
const ERROR: u16 = 10;
const QUIT: u16 = 7;
const UNKNOWN: u16 = u16::MAX;

fn bootrom_path() -> String {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../roms/rp2040/bootrom.bin")
        .to_string_lossy()
        .into_owned()
}

fn firmware_path() -> String {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../roms/rp2040/uart_hello.bin")
        .to_string_lossy()
        .into_owned()
}

fn runner() -> Child {
    runner_with_board(false)
}

fn runner_with_board(board: bool) -> Child {
    let bootrom = bootrom_path();
    let firmware = firmware_path();
    let mut command = Command::new(env!("CARGO_BIN_EXE_picocalc-run"));
    command.args(["--bin", firmware.as_str(), "--bootrom", bootrom.as_str()]);
    if board {
        command.args(["--board", "picocalc", "--lcd-variant", "pio-rgb565"]);
    }
    command
        .arg("--preview-api")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn picocalc-run preview backend")
}

fn runner_with_firmware(firmware: &Path) -> Child {
    let bootrom = bootrom_path();
    let mut command = Command::new(env!("CARGO_BIN_EXE_picocalc-run"));
    command
        .args([
            "--bin",
            firmware.to_str().expect("firmware path is UTF-8"),
            "--bootrom",
            bootrom.as_str(),
            "--preview-api",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    command.spawn().expect("spawn UART RX preview backend")
}

fn unique_fixture_path(name: &str) -> PathBuf {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock before UNIX epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "picocalc-preview-{name}-{}-{nanos}.bin",
        std::process::id()
    ))
}

/// Build a tiny RP2040 Thumb-1 firmware that enables UART0 RX/TX, emits a
/// ready byte, then echoes every byte received from the external UART wire.
///
/// The fixture is deliberately generated here rather than checked in as an
/// opaque binary. It uses only the instruction encodings already covered by
/// the RP2040 raw-flash tests: literal loads, word MMIO loads/stores, MOVS,
/// ANDS, CMP, and short backward branches.
fn uart_echo_firmware() -> Vec<u8> {
    // `bootrom.bin` is the repository's synthetic handoff and starts the
    // reset handler at flash offset zero.  Keep this fixture in that layout;
    // a B2 bootrom image would instead require a vector table at +0x100.
    const CODE_OFFSET: usize = 0;
    const RESETS_CLR: u32 = 0x4000_f000;
    const RESET_UART0: u32 = 1 << 22;
    const UART0_BASE: u32 = 0x4003_4000;

    let literals = [RESETS_CLR, RESET_UART0, UART0_BASE, 0x70, 0x301];
    let mut words = vec![
        0x4800, // LDR r0, =RESETS_CLR (patched below)
        0x4900, // LDR r1, =RESET_UART0
        0x6001, // STR r1, [r0]
        0x4800, // LDR r0, =UART0_BASE
        0x4900, // LDR r1, =UARTLCR_H
        0x62c1, // STR r1, [r0, #0x2c]
        0x4900, // LDR r1, =UARTCR
        0x6301, // STR r1, [r0, #0x30]
        0x2145, // MOVS r1, #'E' (ready marker)
        0x6001, // STR r1, [r0, #UARTDR]
        0x2210, // MOVS r2, #UARTFR.RXFE
        0x6981, // loop: LDR r1, [r0, #UARTFR]
        0x4011, // ANDS r1, r2
        0x2900, // CMP r1, #0
        0xd1fb, // BNE loop (-10 bytes)
        0x6801, // LDR r1, [r0, #UARTDR]
        0x6001, // STR r1, [r0, #UARTDR]
        0xe7f8, // B loop (-16 bytes)
    ];

    // The loop starts at word 11. The literal pool follows the code and is
    // reachable by the first LDR with a word-scaled PC-relative immediate.
    let pool_offset = (CODE_OFFSET + words.len() * 2 + 3) & !3;
    for (literal_index, word_index) in [0usize, 1, 3, 4, 6].into_iter().enumerate() {
        let instruction_offset = CODE_OFFSET + word_index * 2;
        let pc_aligned = (instruction_offset + 4) & !3;
        let target_offset = pool_offset + literal_index * 4;
        let distance = target_offset
            .checked_sub(pc_aligned)
            .expect("literal pool follows echo program");
        assert!(distance.is_multiple_of(4) && distance / 4 <= 0xff);
        words[word_index] = 0x4800
            | ((if word_index == 0 || word_index == 3 {
                0
            } else {
                1
            }) << 8)
            | (distance / 4) as u16;
    }

    let mut image = vec![0u8; pool_offset + literals.len() * 4];
    for (index, word) in words.into_iter().enumerate() {
        let offset = CODE_OFFSET + index * 2;
        image[offset..offset + 2].copy_from_slice(&word.to_le_bytes());
    }
    for (index, literal) in literals.into_iter().enumerate() {
        let offset = pool_offset + index * 4;
        image[offset..offset + 4].copy_from_slice(&literal.to_le_bytes());
    }
    image
}

fn machine_api_runner_with_board(board: bool) -> Child {
    let bootrom = bootrom_path();
    let firmware = firmware_path();
    let mut command = Command::new(env!("CARGO_BIN_EXE_picocalc-run"));
    command.args(["--bin", firmware.as_str(), "--bootrom", bootrom.as_str()]);
    if board {
        command.args(["--board", "picocalc", "--lcd-variant", "pio-rgb565"]);
    }
    command
        .arg("--machine-api")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn picocalc-run machine API")
}

fn replay_scenario_runner(scenario: &Path, machine_api: bool) -> Child {
    let bootrom = bootrom_path();
    let firmware = firmware_path();
    let mut command = Command::new(env!("CARGO_BIN_EXE_picocalc-run"));
    command.args([
        "--bin",
        firmware.as_str(),
        "--bootrom",
        bootrom.as_str(),
        "--cycles",
        "1000000",
        "--replay-scenario",
        scenario.to_str().expect("scenario path is UTF-8"),
    ]);
    command.arg(if machine_api {
        "--machine-api"
    } else {
        "--preview-api"
    });
    command
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn replay runner")
}

fn replay_scenario_fixture() -> PathBuf {
    let path = unique_fixture_path("scenario");
    fs::write(
        &path,
        r#"{
  "schema": 1,
  "name": "replay-smoke",
  "poll_ms": 5,
  "steps": [{"op": "wait", "ms": 1}]
}"#,
    )
    .expect("write replay scenario fixture");
    path
}

fn canonical_observation_digest(projection: &Value) -> String {
    sha256_hex(&serde_json::to_vec(projection).expect("serialize observation projection"))
}

const REPORT_AUDIO_FIELDS: &[&str] = &[
    "status",
    "dma_write_count",
    "target_write_attempt_count",
    "other_pwm_cc_write_count",
    "wrong_width_count",
    "wrong_treq_count",
    "missing_due_cycle_count",
    "pcm_sha256",
    "first_words",
    "last_words",
    "timer_index",
    "treq",
    "sample_rate_hz",
    "timer_event_count",
    "timer_miss_count",
    "timer_miss_audio_not_busy",
    "timer_miss_other_dma_selected",
    "timer_miss_no_dma_selected",
    "timer_miss_multiple_due_in_window",
    "timer_due_cycle_sha256",
    "block_start_count",
    "block_frame_min",
    "block_frame_max",
    "malformed_block_count",
    "block_boundary_gap_count",
    "block_boundary_gap_min_cycles",
    "block_boundary_gap_max_cycles",
    "block_boundary_gap_sha256",
    "gap_5208_count",
    "gap_5209_count",
    "unexpected_gap_count",
    "service_latency_min_cycles",
    "service_latency_max_cycles",
    "service_latency_sha256",
];

fn normalize_audio_words(value: &Value) -> Value {
    Value::Array(
        value
            .as_array()
            .expect("audio edge words are arrays")
            .iter()
            .map(|word| match word {
                Value::String(text) => {
                    let number = text
                        .strip_prefix("0x")
                        .or_else(|| text.strip_prefix("0X"))
                        .expect("report audio word is hexadecimal");
                    Value::from(u64::from_str_radix(number, 16).expect("parse audio word"))
                }
                Value::Number(_) => word.clone(),
                other => panic!("unexpected audio edge word {other:?}"),
            })
            .collect(),
    )
}

fn common_audio_projection(source: &Value, report: bool) -> Value {
    let object = source.as_object().expect("audio projection is an object");
    let mut output = serde_json::Map::new();
    for field in REPORT_AUDIO_FIELDS {
        let value = object
            .get(*field)
            .unwrap_or_else(|| panic!("audio projection misses {field}"));
        output.insert(
            (*field).to_string(),
            if matches!(*field, "first_words" | "last_words") {
                normalize_audio_words(value)
            } else {
                value.clone()
            },
        );
    }
    let (x, y) = if report {
        let fraction = object
            .get("timer_fraction")
            .and_then(Value::as_str)
            .expect("report timer fraction");
        let (x, y) = fraction
            .split_once('/')
            .expect("report timer fraction slash");
        (
            x.parse::<u64>().expect("report timer fraction numerator"),
            y.parse::<u64>().expect("report timer fraction denominator"),
        )
    } else {
        (
            object["timer_fraction_x"]
                .as_u64()
                .expect("preview timer fraction numerator"),
            object["timer_fraction_y"]
                .as_u64()
                .expect("preview timer fraction denominator"),
        )
    };
    output.insert("timer_fraction_x".into(), Value::from(x));
    output.insert("timer_fraction_y".into(), Value::from(y));
    Value::Object(output)
}

fn common_projection_from_preview(projection: &Value) -> Value {
    let mut output = serde_json::Map::new();
    output.insert("schema_version".into(), Value::from(1));
    output.insert(
        "audio".into(),
        common_audio_projection(&projection["audio"], false),
    );
    output.insert("framebuffer".into(), projection["framebuffer"].clone());
    output.insert("uart".into(), projection["uart"].clone());
    output.insert(
        "unsupported_mmio".into(),
        projection["unsupported_mmio"].clone(),
    );
    Value::Object(output)
}

fn common_projection_from_report(report: &Value) -> Value {
    let mut output = serde_json::Map::new();
    output.insert("schema_version".into(), Value::from(1));
    output.insert(
        "audio".into(),
        common_audio_projection(&report["audio_sink"], true),
    );
    output.insert(
        "framebuffer".into(),
        serde_json::json!({
            "height": report["framebuffer"]["height"],
            "non_black_pixels": report["framebuffer"]["non_black_pixels"],
            "rgb565_sha256": report["framebuffer"]["rgb565_sha256"],
            "width": report["framebuffer"]["width"],
        }),
    );
    output.insert("uart".into(), report["uart"].clone());
    let entries = report["unsupported_mmio"]
        .as_array()
        .expect("report unsupported-MMIO entries")
        .iter()
        .map(|entry| {
            serde_json::json!({
                "addr": u64::from_str_radix(
                    entry["addr"].as_str().expect("report MMIO address").trim_start_matches("0x"),
                    16,
                ).expect("parse report MMIO address"),
                "count": entry["count"],
                "pc": u64::from_str_radix(
                    entry["pc"].as_str().expect("report MMIO PC").trim_start_matches("0x"),
                    16,
                ).expect("parse report MMIO PC"),
            })
        })
        .collect::<Vec<_>>();
    output.insert(
        "unsupported_mmio".into(),
        serde_json::json!({
            "entries": entries,
            "truncated": report["unsupported_mmio_truncated"],
        }),
    );
    Value::Object(output)
}

fn read_frame(stdout: &mut ChildStdout) -> (u16, u32, Vec<u8>) {
    let mut header = [0u8; 16];
    stdout.read_exact(&mut header).expect("read preview header");
    assert_eq!(&header[0..4], MAGIC);
    assert_eq!(u16::from_le_bytes([header[4], header[5]]), VERSION);
    let kind = u16::from_le_bytes([header[6], header[7]]);
    let length = u32::from_le_bytes([header[8], header[9], header[10], header[11]]) as usize;
    let sequence = u32::from_le_bytes([header[12], header[13], header[14], header[15]]);
    let mut payload = vec![0u8; length];
    stdout
        .read_exact(&mut payload)
        .expect("read preview payload");
    (kind, sequence, payload)
}

fn send_frame(stdin: &mut ChildStdin, kind: u16, sequence: u32, payload: &[u8]) {
    let mut header = [0u8; 16];
    header[0..4].copy_from_slice(MAGIC);
    header[4..6].copy_from_slice(&VERSION.to_le_bytes());
    header[6..8].copy_from_slice(&kind.to_le_bytes());
    header[8..12].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    header[12..16].copy_from_slice(&sequence.to_le_bytes());
    stdin.write_all(&header).expect("write preview header");
    stdin.write_all(payload).expect("write preview payload");
    stdin.flush().expect("flush preview command");
}

#[test]
fn preview_uart_direction_and_quit_are_process_safe() {
    let mut child = runner();
    let mut stdin = child.stdin.take().expect("runner stdin");
    let mut stdout = child.stdout.take().expect("runner stdout");

    let (kind, sequence, payload) = read_frame(&mut stdout);
    assert_eq!((kind, sequence), (HELLO, 0));
    let hello: Value = serde_json::from_slice(&payload).expect("hello JSON");
    assert_eq!(hello["protocol"], "preview-ipc");
    assert_eq!(hello["schema"], 1);

    let (kind, sequence, payload) = read_frame(&mut stdout);
    assert_eq!((kind, sequence), (STATUS, 1));
    let mut last_output_sequence = sequence;
    let status: Value = serde_json::from_slice(&payload).expect("status JSON");
    assert_eq!(status["audio"]["state"], "not_streamed");
    assert_eq!(status["virtual_cycle"], 0);
    assert_eq!(status["observation"]["schema_version"], 1);
    let digest = status["observation"]["digest_sha256"]
        .as_str()
        .expect("observation digest");
    assert_eq!(digest.len(), 64);
    assert!(digest.bytes().all(|byte| byte.is_ascii_hexdigit()));
    assert_eq!(status["observation"]["projection"]["schema_version"], 1);
    assert_eq!(status["observation"]["projection"]["uart"]["bytes"], 0);
    assert_eq!(
        status["observation"]["projection"]["unsupported_mmio"]["truncated"],
        false
    );

    // uart_hello enables TX but not RX.  The preview command must report a
    // disabled RX wire rather than silently pretending that the byte entered
    // the guest FIFO.
    send_frame(&mut stdin, UART_RX, 0, b"Z");
    let mut response_kinds = Vec::new();
    loop {
        let (kind, sequence, _) = read_frame(&mut stdout);
        assert_eq!(sequence, last_output_sequence + 1);
        last_output_sequence = sequence;
        response_kinds.push(kind);
        if kind == ERROR || kind == UART_TX {
            break;
        }
    }
    assert!(
        response_kinds.contains(&ERROR) || response_kinds.contains(&UART_TX),
        "preview did not produce a response to UART RX"
    );

    send_frame(&mut stdin, QUIT, 1, &[]);
    let mut expected_sequence = last_output_sequence + 1;
    let (kind, sequence, payload) = loop {
        let frame = read_frame(&mut stdout);
        assert_eq!(frame.1, expected_sequence);
        expected_sequence += 1;
        if frame.0 == 11 {
            break frame;
        }
    };
    assert_eq!(sequence, expected_sequence - 1);
    let goodbye: Value = serde_json::from_slice(&payload).expect("goodbye JSON");
    assert_eq!(kind, 11);
    assert_eq!(goodbye["reason"], "quit");

    let output = child.wait_with_output().expect("wait for preview runner");
    assert!(
        output.status.success(),
        "preview runner failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn preview_uart_rx_echo_and_overrun_are_directional_and_bounded() {
    let firmware = unique_fixture_path("uart-echo");
    fs::write(&firmware, uart_echo_firmware()).expect("write UART echo fixture");
    let mut child = runner_with_firmware(&firmware);
    let mut stdin = child.stdin.take().expect("runner stdin");
    let mut stdout = child.stdout.take().expect("runner stdout");

    let (kind, sequence, _) = read_frame(&mut stdout);
    assert_eq!((kind, sequence), (HELLO, 0));
    let (kind, sequence, _) = read_frame(&mut stdout);
    assert_eq!((kind, sequence), (STATUS, 1));
    let mut last_output_sequence = sequence;

    // The fixture emits E only after UART0 has been released and RXE/TXE are
    // enabled. Waiting for that byte makes the positive RX assertion
    // independent of host scheduling between startup and the first command.
    loop {
        let (kind, sequence, payload) = read_frame(&mut stdout);
        assert_eq!(sequence, last_output_sequence + 1);
        last_output_sequence = sequence;
        if kind == UART_TX {
            assert_eq!(payload.len(), 9);
            assert_eq!(payload[8], b'E');
            break;
        }
    }

    // No stepping occurs between these queued commands: the preview input
    // drain handles all of them in one turn. UART0's 16-byte RX FIFO therefore
    // accepts the first sixteen bytes and reports the seventeenth as an
    // explicit overrun rather than silently dropping it.
    let input = b"abcdefghijklmnopq";
    for (sequence, byte) in input.iter().copied().enumerate() {
        send_frame(&mut stdin, UART_RX, sequence as u32, &[byte]);
    }

    let mut echoes = Vec::new();
    let mut overrun = 0u64;
    let mut disabled = 0u64;
    let mut accepted = None;
    let mut reported_overrun = None;
    while echoes.len() < 16 || overrun == 0 || accepted != Some(16) {
        let (kind, sequence, payload) = read_frame(&mut stdout);
        assert_eq!(sequence, last_output_sequence + 1);
        last_output_sequence = sequence;
        match kind {
            UART_TX => {
                assert_eq!(payload.len(), 9);
                echoes.push(payload[8]);
            }
            ERROR => {
                let error: Value = serde_json::from_slice(&payload).expect("UART error JSON");
                match error["code"].as_str() {
                    Some("uart_rx_overrun") => overrun += 1,
                    Some("uart_rx_disabled") => disabled += 1,
                    other => panic!("unexpected preview UART error {other:?}: {error}"),
                }
            }
            STATUS => {
                let status: Value = serde_json::from_slice(&payload).expect("UART status JSON");
                accepted = status["uart"]["rx_accepted"].as_u64();
                reported_overrun = status["uart"]["rx_overrun"].as_u64();
            }
            _ => {}
        }
    }
    assert_eq!(disabled, 0, "RX was unexpectedly reported disabled");
    assert_eq!(overrun, 1, "exactly one byte must overrun the RX FIFO");
    assert_eq!(reported_overrun, Some(1));
    assert_eq!(accepted, Some(16));
    assert_eq!(echoes, input[..16]);

    send_frame(&mut stdin, QUIT, input.len() as u32, &[]);
    loop {
        let (kind, sequence, payload) = read_frame(&mut stdout);
        assert_eq!(sequence, last_output_sequence + 1);
        last_output_sequence = sequence;
        if kind == 11 {
            let goodbye: Value = serde_json::from_slice(&payload).expect("goodbye JSON");
            assert_eq!(goodbye["reason"], "quit");
            break;
        }
    }
    let output = child.wait_with_output().expect("wait for UART RX runner");
    assert!(
        output.status.success(),
        "UART RX preview runner failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    fs::remove_file(&firmware).expect("remove UART echo fixture");
}

#[test]
fn preview_protocol_error_is_fail_closed() {
    let mut child = runner();
    let mut stdin = child.stdin.take().expect("runner stdin");
    let mut stdout = child.stdout.take().expect("runner stdout");

    // Consume the deterministic startup frames before injecting malformed
    // input.  The runner must not reinterpret the unknown kind or turn the
    // protocol error into a successful preview session.
    assert_eq!(read_frame(&mut stdout).0, HELLO);
    assert_eq!(read_frame(&mut stdout).0, STATUS);
    send_frame(&mut stdin, UNKNOWN, 0, &[]);
    drop(stdin);

    let output = child
        .wait_with_output()
        .expect("wait for malformed preview runner");
    assert_eq!(output.status.code(), Some(2));
    assert!(
        String::from_utf8_lossy(&output.stderr).contains("protocol error"),
        "stderr did not identify the protocol failure: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn registered_replay_finishes_with_a_pass_status() {
    let scenario = replay_scenario_fixture();
    let mut child = replay_scenario_runner(&scenario, false);
    let mut stdin = child.stdin.take().expect("replay stdin");
    let mut stdout = child.stdout.take().expect("replay stdout");

    assert_eq!(read_frame(&mut stdout).0, HELLO);
    let mut last_sequence = read_frame(&mut stdout).1;
    let mut saw_pass = false;
    let mut replay_cycle = 0;
    while !saw_pass {
        let (kind, sequence, payload) = read_frame(&mut stdout);
        assert_eq!(sequence, last_sequence + 1);
        last_sequence = sequence;
        if kind != STATUS {
            continue;
        }
        let status: Value = serde_json::from_slice(&payload).expect("replay status JSON");
        if status["replay"]["status"] == "pass" {
            saw_pass = true;
            replay_cycle = status["virtual_cycle"]
                .as_u64()
                .expect("replay status cycle");
            assert!(replay_cycle > 0);
            assert_eq!(status["replay"]["steps_completed"], 1);
            assert_eq!(status["replay"]["steps_total"], 1);
        }
    }

    send_frame(&mut stdin, QUIT, 0, &[]);
    loop {
        let (kind, sequence, _) = read_frame(&mut stdout);
        assert_eq!(sequence, last_sequence + 1);
        last_sequence = sequence;
        if kind == 11 {
            break;
        }
    }
    let output = child.wait_with_output().expect("wait for replay runner");
    assert!(
        output.status.success(),
        "replay runner failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(replay_cycle > 0);
    fs::remove_file(&scenario).expect("remove replay scenario fixture");
}

#[test]
fn machine_api_registered_replay_is_observable_at_final_cycle() {
    let scenario = replay_scenario_fixture();
    let mut child = replay_scenario_runner(&scenario, true);
    let mut stdin = child.stdin.take().expect("machine replay stdin");
    let stdout = child.stdout.take().expect("machine replay stdout");
    let mut stdout = BufReader::new(stdout);
    writeln!(
        stdin,
        r#"{{"schema":1,"id":"observe","op":"observe","domains":["preview"]}}"#
    )
    .expect("write machine replay observe");
    stdin.flush().expect("flush machine replay observe");
    let mut line = String::new();
    stdout
        .read_line(&mut line)
        .expect("read machine replay response");
    let response: Value = serde_json::from_str(&line).expect("machine replay JSON");
    assert_eq!(response["ok"], true);
    assert!(response["cycle"].as_u64().expect("machine replay cycle") > 0);
    assert_eq!(response["result"]["preview"]["schema_version"], 1);
    drop(stdin);
    let output = child
        .wait_with_output()
        .expect("wait for machine replay runner");
    assert!(
        output.status.success(),
        "machine replay runner failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    fs::remove_file(&scenario).expect("remove replay scenario fixture");
}

#[test]
fn preview_reset_and_key_rejection_are_explicit() {
    let mut child = runner_with_board(true);
    let mut stdin = child.stdin.take().expect("runner stdin");
    let mut stdout = child.stdout.take().expect("runner stdout");

    assert_eq!(read_frame(&mut stdout).0, HELLO);
    assert_eq!(read_frame(&mut stdout).0, FRAME_RGB565);
    let (kind, sequence, _) = read_frame(&mut stdout);
    assert_eq!(kind, STATUS);
    let mut last_output_sequence = sequence;

    // No keyboard model is attached in this process. The command is accepted
    // by the wire but rejected at the device boundary; it must not be
    // silently converted into process stdin or a different input device.
    let key_payload = serde_json::to_vec(&serde_json::json!({
        "key": "X",
        "state": "down",
    }))
    .expect("serialize key event");
    send_frame(&mut stdin, 5, 0, &key_payload);
    loop {
        let (kind, sequence, payload) = read_frame(&mut stdout);
        assert_eq!(sequence, last_output_sequence + 1);
        last_output_sequence = sequence;
        if kind == ERROR {
            let error: Value = serde_json::from_slice(&payload).expect("key error JSON");
            assert_eq!(error["code"], "key_rejected");
            break;
        }
    }

    // Reset is an explicit preview operation. It must emit a new frame with
    // virtual cycle zero and then continue normally; it is not a guest
    // watchdog event and does not terminate the process.
    send_frame(&mut stdin, RESET, 1, &[]);
    loop {
        let (kind, sequence, payload) = read_frame(&mut stdout);
        assert_eq!(sequence, last_output_sequence + 1);
        last_output_sequence = sequence;
        if kind == FRAME_RGB565 {
            assert!(payload.len() >= 12);
            let cycle = u64::from_le_bytes(payload[0..8].try_into().unwrap());
            assert_eq!(cycle, 0, "preview reset frame must start at cycle zero");
            break;
        }
    }

    send_frame(&mut stdin, QUIT, 2, &[]);
    let mut expected_sequence = last_output_sequence + 1;
    loop {
        let (kind, sequence, _) = read_frame(&mut stdout);
        assert_eq!(sequence, expected_sequence);
        expected_sequence += 1;
        if kind == 11 {
            break;
        }
    }
    let output = child.wait_with_output().expect("wait for reset runner");
    assert!(
        output.status.success(),
        "preview reset runner failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn preview_observation_matches_machine_api_at_a_shared_cycle() {
    // Use the board-backed path here so the gate covers the actual LCD model,
    // not only the board-less UART fixture. The firmware intentionally does
    // not write pixels; the initial 320x320 RGB565 frame is still part of the
    // shared observation boundary and must be identical in all three paths.
    let mut preview = runner_with_board(true);
    let mut preview_stdin = preview.stdin.take().expect("preview stdin");
    let mut preview_stdout = preview.stdout.take().expect("preview stdout");

    assert_eq!(read_frame(&mut preview_stdout).0, HELLO);
    let (kind, _, payload) = read_frame(&mut preview_stdout);
    assert_eq!(kind, 3, "board-backed preview must emit an initial frame");
    assert!(payload.len() >= 12, "frame payload must contain its prefix");
    assert_eq!(u16::from_le_bytes([payload[8], payload[9]]), 320);
    assert_eq!(u16::from_le_bytes([payload[10], payload[11]]), 320);
    assert_eq!(payload.len(), 12 + 320 * 320 * 2);
    assert_eq!(read_frame(&mut preview_stdout).0, STATUS);

    // Pick a cycle boundary emitted by the preview itself after the fixture
    // has produced UART output. The batch runner and machine API must be able
    // to run to that exact boundary and expose the same projection, without
    // comparing host wall-clock timestamps.
    let (target_cycle, preview_digest, preview_projection) = loop {
        let (kind, _, payload) = read_frame(&mut preview_stdout);
        if kind != STATUS {
            continue;
        }
        let status: Value = serde_json::from_slice(&payload).expect("preview status JSON");
        let cycle = status["virtual_cycle"]
            .as_u64()
            .expect("preview status cycle");
        if cycle == 0 || status["observation"]["projection"]["uart"]["bytes"] == 0 {
            continue;
        }
        break (
            cycle,
            status["observation"]["digest_sha256"]
                .as_str()
                .expect("preview observation digest")
                .to_string(),
            status["observation"]["projection"].clone(),
        );
    };

    send_frame(&mut preview_stdin, QUIT, 0, &[]);
    loop {
        if read_frame(&mut preview_stdout).0 == 11 {
            break;
        }
    }
    let preview_output = preview.wait_with_output().expect("wait for preview runner");
    assert!(
        preview_output.status.success(),
        "preview runner failed: {}",
        String::from_utf8_lossy(&preview_output.stderr)
    );

    let mut machine = machine_api_runner_with_board(true);
    let mut machine_stdin = machine.stdin.take().expect("machine API stdin");
    let machine_stdout = machine.stdout.take().expect("machine API stdout");
    let mut machine_stdout = BufReader::new(machine_stdout);
    writeln!(
        machine_stdin,
        "{{\"schema\":1,\"id\":\"run\",\"op\":\"run\",\"max_cycles\":{target_cycle}}}"
    )
    .expect("write machine run request");
    machine_stdin.flush().expect("flush machine run request");
    let mut line = String::new();
    machine_stdout
        .read_line(&mut line)
        .expect("read machine run response");
    let run_response: Value = serde_json::from_str(&line).expect("machine run JSON");
    assert_eq!(run_response["ok"], true);
    assert_eq!(run_response["cycle"], target_cycle);

    writeln!(
        machine_stdin,
        "{{\"schema\":1,\"id\":\"observe\",\"op\":\"observe\",\"domains\":[\"preview\"]}}"
    )
    .expect("write machine observe request");
    machine_stdin
        .flush()
        .expect("flush machine observe request");
    line.clear();
    machine_stdout
        .read_line(&mut line)
        .expect("read machine observe response");
    let observe_response: Value = serde_json::from_str(&line).expect("machine observe JSON");
    let machine_preview = &observe_response["result"]["preview"];
    assert_eq!(observe_response["ok"], true);
    assert_eq!(machine_preview["virtual_cycle"], target_cycle);
    assert_eq!(
        machine_preview["digest_sha256"], preview_digest,
        "preview and machine API observation digests diverged"
    );
    assert_eq!(
        machine_preview["projection"], preview_projection,
        "preview and machine API observation projections diverged"
    );

    drop(machine_stdin);
    let machine_output = machine
        .wait_with_output()
        .expect("wait for machine API runner");
    assert!(
        machine_output.status.success(),
        "machine API runner failed: {}",
        String::from_utf8_lossy(&machine_output.stderr)
    );

    // The authoritative batch path still writes the historical schema-8
    // report rather than a preview projection. Compare the fields that form
    // the shared observation boundary, including UART and the audio digest,
    // and require that the report stopped at the exact selected cycle. This
    // is the three-way VRP-2 smoke gate: batch == machine API == preview.
    let report_path = std::env::temp_dir().join(format!(
        "picocalc-vrp2-batch-report-{}-{}.json",
        std::process::id(),
        target_cycle
    ));
    let analysis_path = report_path.with_extension("audio.json");
    let target_cycles = target_cycle.to_string();
    let firmware = firmware_path();
    let bootrom = bootrom_path();
    let batch_output = Command::new(env!("CARGO_BIN_EXE_picocalc-run"))
        .args([
            "--bin",
            firmware.as_str(),
            "--bootrom",
            bootrom.as_str(),
            "--board",
            "picocalc",
            "--lcd-variant",
            "pio-rgb565",
            "--quantum",
            "1",
            "--cycles",
            target_cycles.as_str(),
            "--expect-stop",
            "cycle_limit",
            "--audio-analysis",
            analysis_path
                .to_str()
                .expect("batch analysis path is UTF-8"),
            "--json",
            report_path.to_str().expect("batch report path is UTF-8"),
        ])
        .output()
        .expect("run batch runner for shared-cycle gate");
    assert!(
        batch_output.status.success(),
        "batch runner failed: status={:?}\nstdout={}\nstderr={}",
        batch_output.status,
        String::from_utf8_lossy(&batch_output.stdout),
        String::from_utf8_lossy(&batch_output.stderr)
    );
    let report: Value =
        serde_json::from_slice(&std::fs::read(&report_path).expect("read batch schema-8 report"))
            .expect("parse batch schema-8 report");
    assert_eq!(report["cycles"].as_u64(), Some(target_cycle));
    assert_eq!(report["stop_reason"], "cycle_limit");
    assert_eq!(report["verdict"]["status"], "pass");
    // Normalize only the report-compatible, device-visible fields. Report
    // provenance (backend, BIN, stop reason, PNG path, audio oracle) is not
    // part of the preview observation schema. The resulting canonical
    // projection is hashed, so adding a field or changing its representation
    // cannot silently leave the three paths partially compared.
    let preview_common = common_projection_from_preview(&preview_projection);
    let report_common = common_projection_from_report(&report);
    assert_eq!(
        preview_common, report_common,
        "batch/report observation diverged"
    );
    assert_eq!(
        canonical_observation_digest(&preview_common),
        canonical_observation_digest(&report_common),
        "batch and preview observation boundary digests diverged"
    );
    assert_eq!(
        preview_digest,
        canonical_observation_digest(&preview_common),
        "preview's declared digest does not cover the report-compatible projection"
    );
    let _ = std::fs::remove_file(&analysis_path);
    let _ = std::fs::remove_file(&report_path);
}
