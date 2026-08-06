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
    let commit = git(&repo, &["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".to_string());
    let dirty = git(&repo, &["status", "--porcelain", "--untracked-files=no"])
        .is_none_or(|status| !status.is_empty());

    println!("cargo:rustc-env=PICOEM_BUILT_COMMIT={commit}");
    println!("cargo:rustc-env=PICOEM_BUILT_DIRTY={dirty}");

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
