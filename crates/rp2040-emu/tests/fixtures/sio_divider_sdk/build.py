#!/usr/bin/env python3
"""Rebuild the RP2040 SDK divider test fixture using the pinned SDK checkout."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

SDK_COMMIT = "a1438dff1d38bd9c65dbd693f0e5db4b9ae91779"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", required=True, type=Path)
    args = parser.parse_args()
    sdk = args.sdk.resolve()
    here = Path(__file__).resolve().parent
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=sdk, text=True).strip()
    if commit != SDK_COMMIT:
        raise SystemExit(f"Expected SDK {SDK_COMMIT}, got {commit}")
    inputs = [
        "src/rp2_common/pico_divider/divider_hardware.S",
        "src/rp2_common/hardware_divider/include/hardware/divider_helper.S",
        "src/rp2040/pico_platform/include/pico/asm_helper.S",
        "src/rp2040/hardware_regs/include/hardware/platform_defs.h",
        "src/rp2040/hardware_regs/include/hardware/regs/addressmap.h",
        "src/rp2040/hardware_regs/include/hardware/regs/sio.h",
    ]
    hashes = {}
    for name in inputs:
        data = (sdk / name).read_bytes()
        pinned = subprocess.check_output(["git", "show", f"{SDK_COMMIT}:{name}"], cwd=sdk)
        if data != pinned:
            raise SystemExit(f"SDK input differs from pinned revision: {name}")
        hashes[name] = hashlib.sha256(data).hexdigest()
    with tempfile.TemporaryDirectory(prefix="sio-divider-sdk-") as temporary:
        work = Path(temporary)
        common = ["arm-none-eabi-gcc", "-mcpu=cortex-m0plus", "-mthumb"]
        includes = [
            here,
            sdk / "src/rp2040/pico_platform/include",
            sdk / "src/rp2040/hardware_regs/include",
            sdk / "src/rp2_common/hardware_divider/include",
        ]
        commands = [
            common + ["-c", "-x", "assembler-with-cpp"]
            + [flag for path in includes for flag in ["-I", str(path)]]
            + [str(sdk / inputs[0]), "-o", str(work / "divider.o")],
            common + ["-c", str(here / "harness.S"), "-o", str(work / "harness.o")],
            common + ["-nostdlib", "-Wl,--build-id=none", "-T", str(here / "link.ld"),
                      str(work / "harness.o"), str(work / "divider.o"), "-o", str(work / "fixture.elf")],
            ["arm-none-eabi-objcopy", "-O", "binary", str(work / "fixture.elf"), str(here / "fixture.bin")],
        ]
        for command in commands:
            subprocess.run(command, check=True)
        (here / "fixture.bin").chmod(0o644)
        symbols = subprocess.check_output(["arm-none-eabi-nm", "-n", str(work / "fixture.elf")], text=True)
        (here / "symbols.txt").write_text(symbols)
        metadata = {
            "sdk_commit": SDK_COMMIT,
            "sdk_input_sha256": hashes,
            "local_input_sha256": {name: hashlib.sha256((here / name).read_bytes()).hexdigest()
                                   for name in ["harness.S", "pico.h", "link.ld", "build.py"]},
            "compiler": subprocess.check_output(["arm-none-eabi-gcc", "--version"], text=True).splitlines()[0],
            "load_address": "0x20000000",
            "fixture_sha256": hashlib.sha256((here / "fixture.bin").read_bytes()).hexdigest(),
            "elf_sha256": hashlib.sha256((work / "fixture.elf").read_bytes()).hexdigest(),
            "commands": [[item.replace(str(work), "<temporary>").replace(str(sdk), "<sdk>")
                          .replace(str(here), "<fixture>") for item in command] for command in commands],
        }
        (here / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
