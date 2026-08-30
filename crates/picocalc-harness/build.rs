use std::path::PathBuf;
use std::process::Command;

fn git(repo: &PathBuf, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(repo)
        .args(args)
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn main() {
    let manifest = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let repo = manifest.join("../..");
    // The compile-time provenance also includes whether tracked files were
    // modified. Watch the tracked paths so Cargo reruns this build script when
    // a source or documentation edit changes that dirty-state value; watching
    // only .git metadata leaves a stale `backend_build.dirty=false` in the next
    // binary. The index watcher below also refreshes this list when files are
    // added or removed from the checkout.
    if let Some(files) = git(&repo, &["ls-files"]) {
        for file in files.lines() {
            println!("cargo:rerun-if-changed={}", repo.join(file).display());
        }
    }
    let commit = git(&repo, &["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".to_string());
    let dirty = git(&repo, &["status", "--porcelain", "--untracked-files=no"])
        .is_none_or(|status| !status.is_empty());

    println!("cargo:rustc-env=PICOEM_BUILT_COMMIT={commit}");
    println!("cargo:rustc-env=PICOEM_BUILT_DIRTY={dirty}");

    // Keep the diagnostic profile's feature_set tied to Cargo's actual
    // feature environment rather than a self-attested CLI label. The
    // benchmark runner compares this list with the build-provenance sidecar.
    let feature_env = [
        ("CARGO_FEATURE_BEHAVIOR_TRACE", "behavior-trace"),
        (
            "CARGO_FEATURE_COMPACT_DISPATCH_KEY_PROTOTYPE",
            "compact-dispatch-key-prototype",
        ),
        (
            "CARGO_FEATURE_CPU_APPLICATION_PROFILER",
            "cpu-application-profiler",
        ),
        (
            "CARGO_FEATURE_DECODED_OP_8BYTE_PROTOTYPE",
            "decoded-op-8byte-prototype",
        ),
        (
            "CARGO_FEATURE_DIAGNOSTIC_PC_COMPILE_OUT_PROTOTYPE",
            "diagnostic-pc-compile-out-prototype",
        ),
        (
            "CARGO_FEATURE_EVENT_HORIZON_PROFILER",
            "event-horizon-profiler",
        ),
        ("CARGO_FEATURE_IDLE_PROFILER", "idle-profiler"),
        (
            "CARGO_FEATURE_NVIC_BITMAP_SCAN_PROTOTYPE",
            "nvic-bitmap-scan-prototype",
        ),
        ("CARGO_FEATURE_SD_GEN1_MULTIBLOCK", "sd-gen1-multiblock"),
        ("CARGO_FEATURE_THREADING", "threading"),
        ("CARGO_FEATURE_TESTING", "testing"),
        ("CARGO_FEATURE_TEST_HOOKS", "test-hooks"),
        (
            "CARGO_FEATURE_UNCONDITIONAL_CACHE_LOOKUP_PROTOTYPE",
            "unconditional-cache-lookup-prototype",
        ),
    ];
    let mut enabled_features: Vec<&str> = feature_env
        .iter()
        .filter_map(|(env_name, feature)| {
            println!("cargo:rerun-if-env-changed={env_name}");
            std::env::var_os(env_name).map(|_| *feature)
        })
        .collect();
    enabled_features.sort_unstable();
    println!(
        "cargo:rustc-env=PICOEM_FEATURE_SET={}",
        enabled_features.join(",")
    );

    if let Some(git_dir) = git(&repo, &["rev-parse", "--git-dir"]) {
        let git_dir = PathBuf::from(git_dir);
        let git_dir = if git_dir.is_absolute() {
            git_dir
        } else {
            repo.join(git_dir)
        };
        println!("cargo:rerun-if-changed={}", git_dir.join("HEAD").display());
        println!("cargo:rerun-if-changed={}", git_dir.join("index").display());
        println!(
            "cargo:rerun-if-changed={}",
            git_dir.join("packed-refs").display()
        );
        if let Some(head) = git(&repo, &["symbolic-ref", "-q", "HEAD"]) {
            println!("cargo:rerun-if-changed={}", git_dir.join(head).display());
        }
    }
}
