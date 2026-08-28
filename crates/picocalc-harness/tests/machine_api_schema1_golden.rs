//! VRP-2-c compatibility guard for the established NEXT-4 machine API.
//!
//! The JSONL fixture is deliberately an exchange transcript rather than a
//! snapshot of internal structs.  It fixes the schema-1 request/response
//! contract for all existing operations that a headless client uses.  The
//! preview-only `observe` domain is tested separately by `preview_api_e2e`;
//! adding it here would turn an additive domain into an accidental change to
//! the established transcript.

use std::fs;
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::Value;

const GOLDEN: &str = include_str!("fixtures/machine-api-schema1-golden.jsonl");

fn fixture_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../roms/rp2040")
        .join(name)
}

fn unique_snapshot_dir() -> PathBuf {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock before UNIX epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "picocalc-machine-api-golden-{}-{nanos}",
        std::process::id()
    ))
}

fn spawn_machine_api(snapshot_dir: &Path) -> Child {
    let mut command = Command::new(env!("CARGO_BIN_EXE_picocalc-run"));
    command.args([
        "--bin",
        fixture_path("uart_hello.bin")
            .to_str()
            .expect("firmware path UTF-8"),
        "--bootrom",
        fixture_path("bootrom.bin")
            .to_str()
            .expect("bootrom path UTF-8"),
        "--board",
        "picocalc",
        "--lcd-variant",
        "pio-rgb565",
        "--keyboard",
        "--machine-api",
        "--snapshot-dir",
        snapshot_dir.to_str().expect("snapshot path UTF-8"),
    ]);
    command
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn machine API runner")
}

#[test]
fn machine_api_schema1_replays_the_golden_jsonl_transcript() {
    let snapshot_dir = unique_snapshot_dir();
    fs::create_dir_all(&snapshot_dir).expect("create snapshot directory");

    let mut child = spawn_machine_api(&snapshot_dir);
    let mut input = child.stdin.take().expect("machine API stdin");
    let stdout = child.stdout.take().expect("machine API stdout");
    let mut output = BufReader::new(stdout);

    for (line_number, line) in GOLDEN.lines().enumerate() {
        let exchange: Value = serde_json::from_str(line).unwrap_or_else(|error| {
            panic!("invalid golden JSONL line {}: {error}", line_number + 1)
        });
        let request = exchange
            .get("request")
            .unwrap_or_else(|| panic!("golden line {} misses request", line_number + 1));
        let expected = exchange
            .get("response")
            .unwrap_or_else(|| panic!("golden line {} misses response", line_number + 1));
        serde_json::to_writer(&mut input, request).expect("write machine request");
        input.write_all(b"\n").expect("terminate machine request");
        input.flush().expect("flush machine request");

        let mut response_line = String::new();
        let read = output
            .read_line(&mut response_line)
            .expect("read machine response");
        assert!(
            read > 0,
            "machine API ended before golden line {}",
            line_number + 1
        );
        let actual: Value =
            serde_json::from_str(response_line.trim_end()).unwrap_or_else(|error| {
                panic!("invalid response at line {}: {error}", line_number + 1)
            });
        assert_eq!(
            actual,
            *expected,
            "machine API changed at golden line {}",
            line_number + 1
        );
    }

    drop(input);
    let result = child
        .wait_with_output()
        .expect("wait for machine API runner");
    assert!(
        result.status.success(),
        "machine API runner failed: {}",
        String::from_utf8_lossy(&result.stderr)
    );
    let snapshot = snapshot_dir.join("golden.png");
    assert!(
        snapshot.is_file(),
        "snapshot operation did not create golden.png"
    );
    fs::remove_dir_all(&snapshot_dir).expect("remove golden snapshot directory");
}
