# SeaBIOS license texts

The PicoEM harness includes a 256 KiB SeaBIOS payload as a test fixture:

- `crates/picoem-harness/fixtures/sources/seabios-256k.bin`
- `crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-seabios-cpu.bin`
- `crates/picoem-harness/fixtures/onerom-fire-32-a-rp2350-seabios.bin`

The payload was obtained from the SeaBIOS image used by the corresponding
OneROM fixture build. Its SHA-256 is recorded in
`crates/picoem-harness/fixtures/README.md`. The two OneROM files are generated
carrier images that contain the SeaBIOS bytes; the carrier code and the
SeaBIOS payload retain their respective license boundaries.

SeaBIOS is maintained by the coreboot project:

<https://github.com/coreboot/seabios>

The payload is distributed under the GNU Lesser General Public License,
version 3 (LGPL-3.0). The complete upstream license texts are kept here:

- [`COPYING.LESSER`](COPYING.LESSER) — LGPL-3.0
- [`COPYING`](COPYING) — GPL-3.0 terms incorporated by LGPL-3.0

These files are faithful copies of the corresponding files from the upstream
SeaBIOS repository with trailing whitespace normalized for repository hygiene.
This directory contains license assets and provenance only; it does not vendor
the SeaBIOS source tree.
