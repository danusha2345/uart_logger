## Project Map

- Purpose: passive RP2350 UART logger that records two UART directions to SD card.
- Entry point: `src/main.rs`.
- Key modules: `src/config.rs`, `src/sd_writer.rs`, `src/led.rs`.
- Build helpers/artifacts: `build.rs`, `build_uf2.sh`, `uart_logger*.uf2`.
- Hardware and file-format docs: `README.md`.

## Build And Test

- Check/build: `cargo build --release`.
- Flash via runner: `cargo run --release`.
- UF2 helper: `./build_uf2.sh`.

## Local Pitfalls

- Logger is RX-only and must be wired in parallel; do not add TX behavior casually.
- Firmware supports FAT32/FAT16 SD cards, not exFAT.
- Do not commit generated `target/` output or fresh UF2 artifacts unless explicitly requested.
