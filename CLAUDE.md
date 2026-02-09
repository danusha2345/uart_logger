# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Passive bidirectional UART traffic logger on **Spotpear RP2350-Core-A** (ARM Cortex-M85). Captures two UART lines at 921600 baud and writes to SD card in binary format. The device is listen-only (RX), never transmits. Written in embedded Rust (no_std) with Embassy async runtime.

## Build & Flash Commands

```bash
cargo build --release              # Build firmware
cargo run --release                # Build + flash via probe-rs (SWD)
cargo rb                           # Alias: run --bin uart_logger --release
./build_uf2.sh                     # Generate UF2 for USB boot mode flashing
cargo run --bin blink_test --release  # LED test binary
```

**Toolchain:** Rust nightly, target `thumbv8m.main-none-eabihf`
**Flashing:** `probe-rs` (SWD) or UF2 copy to RP2350 USB drive
**Logging:** defmt via RTT, level controlled by `DEFMT_LOG` env var (default: `debug` in `.cargo/config.toml`)

## Architecture

Async multi-task design on Embassy runtime, no OS. All inter-task communication via channels and atomics.

**Task flow:**
```
UART0 RX (GPIO1) ──┐
                    ├──> Channel(32) ──> sd_writer_task ──> LOG_NNNN.BIN (FAT32)
UART1 RX (GPIO5) ──┘
                         led_task reads AtomicU8 state for status LED
                         Watchdog (5s) reboots on hang
```

**Tasks (src/main.rs):**
- `uart_rx_task` (pool_size=2) — blocks on UART RX, non-blocking send to channel. Drops packets on overflow.
- `sd_writer_task` — receives from channel, batch-writes to SD via SPI1. Handles file rotation (100 MB), heartbeat flush (2s), sync (5s), error recovery.
- `led_task` — reads `SYSTEM_STATE` atomic, drives WS2812B via PIO.
- Main — init peripherals, spawn tasks, idle loop with watchdog feed.

**Key modules:**
- `src/config.rs` — all tuneable constants (baud rate, buffer sizes, timeouts, state codes)
- `src/led.rs` — WS2812B PIO driver with color definitions
- `src/sd_writer.rs` — binary record encoding, write buffer, time source, filename generation

**Buffering chain:** UART RX buf (16 KB per channel) → Channel (32 packets) → SD write buf (8 KB) → SD card

## Hardware Pinout

| Function | GPIO | Notes |
|----------|------|-------|
| UART0 RX (Line A) | GPIO1 | From device A |
| UART1 RX (Line B) | GPIO5 | From device B |
| SPI1 SCK/MOSI/MISO/CS | GPIO10-13 | SD card |
| WS2812B LED | GPIO25 | On-board RGB |

## Binary Log Format

Files: `LOG_0001.BIN` .. `LOG_9999.BIN` (auto-rotated at 100 MB)

```
Offset  Size  Field
0       1     Direction (0x00=Line A, 0x01=Line B)
1       4     Timestamp ms from boot (u32 LE)
5       2     Data length (u16 LE, max 256)
7       N     Payload bytes
```

## Conventions

- Documentation and comments are in Russian
- defmt is used for all logging (not println)
- Shared state between tasks uses `portable-atomic` AtomicU8/AtomicU32
- `build.rs` generates the memory layout (`memory.x`) — do not create `memory.x` manually
- SD card must be FAT32 (exFAT not supported); cards ≥64 GB need reformatting
