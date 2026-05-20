# CLAUDE.md

Этот файл — инструкции для Claude Code при работе с репозиторием.

## Обзор проекта

Пассивный двунаправленный логгер UART-трафика на **Spotpear RP2350-Core-A** (ARM Cortex-M85). Захватывает две UART-линии на 921600 baud и пишет на SD-карту в бинарном формате. Устройство только слушает (RX), никогда не передаёт. Написано на embedded Rust (no_std) с асинхронным рантаймом Embassy.

## Команды сборки и прошивки

```bash
cargo build --release              # Build firmware
cargo run --release                # Build + flash via probe-rs (SWD)
cargo rb                           # Alias: run --bin uart_logger --release
./build_uf2.sh                     # Generate UF2 for USB boot mode flashing
cargo run --bin blink_test --release  # LED test binary
```

**Тулчейн:** Rust nightly, target `thumbv8m.main-none-eabihf`
**Прошивка:** `probe-rs` (SWD) или копирование UF2 на USB-диск RP2350
**Логирование:** defmt через RTT, уровень управляется переменной окружения `DEFMT_LOG` (по умолчанию `debug` в `.cargo/config.toml`)

## Архитектура

Асинхронный многозадачный дизайн на рантайме Embassy, без ОС. Все межзадачные взаимодействия — через каналы и атомики.

**Поток задач:**
```
UART0 RX (GPIO1) ──┐
                    ├──> Channel(32) ──> sd_writer_task ──> LOG_NNNN.BIN (FAT32)
UART1 RX (GPIO5) ──┘
                         led_task reads AtomicU8 state for status LED
                         Watchdog (5s) reboots on hang
```

**Задачи (src/main.rs):**
- `uart_rx_task` (pool_size=2) — блокируется на UART RX, неблокирующая отправка в канал. Дропает пакеты при переполнении.
- `sd_writer_task` — принимает из канала, пишет на SD через SPI1 пачками. Делает ротацию файлов (100 МБ), heartbeat flush (2 с), sync (5 с), error recovery.
- `led_task` — читает атомик `SYSTEM_STATE`, управляет WS2812B через PIO.
- Main — инит периферии, спавн задач, idle-loop с feed watchdog.

**Ключевые модули:**
- `src/config.rs` — все тюнинг-константы (baud rate, размеры буферов, таймауты, коды состояний)
- `src/led.rs` — драйвер WS2812B на PIO с определениями цветов
- `src/sd_writer.rs` — кодирование бинарных записей, write-буфер, источник времени, генерация имён файлов

**Цепочка буферизации:** UART RX buf (16 KB на канал) → Channel (32 packets) → SD write buf (8 KB) → SD card

## Распиновка железа

| Function | GPIO | Notes |
|----------|------|-------|
| UART0 RX (Line A) | GPIO1 | From device A |
| UART1 RX (Line B) | GPIO5 | From device B |
| SPI1 SCK/MOSI/MISO/CS | GPIO10-13 | SD card |
| WS2812B LED | GPIO25 | On-board RGB |

## Формат бинарного лога

Файлы: `LOG_0001.BIN` .. `LOG_9999.BIN` (автоматическая ротация на 100 МБ)

```
Offset  Size  Field
0       1     Direction (0x00=Line A, 0x01=Line B)
1       4     Timestamp ms from boot (u32 LE)
5       2     Data length (u16 LE, max 256)
7       N     Payload bytes
```

## Соглашения

- Документация и комментарии — на русском
- Для логирования используется defmt (не println)
- Разделяемое состояние между задачами — через `portable-atomic` AtomicU8/AtomicU32
- `build.rs` генерирует memory layout (`memory.x`) — `memory.x` руками не создавать
- SD-карта должна быть FAT32 (exFAT не поддерживается); карты ≥64 ГБ нужно переформатировать

<!-- serena:start -->
# Serena — Symbolic Code Tools

This project is configured for Serena (`.serena/`). Serena's MCP tools give
language-server-backed symbolic access to the code. Prefer them over reading
whole files or grepping.

## Use Serena for
- **Surveying a file** — `get_symbols_overview` before reading a file in full.
- **Finding a symbol** — `find_symbol` by name path instead of grep.
- **Tracing usage** — `find_referencing_symbols` to see callers/dependents.
- **Editing precisely** — `replace_symbol_body`, `insert_after_symbol`,
  `insert_before_symbol` instead of manual line edits.

Read full file bodies only when symbolic tools don't cover the need.
<!-- serena:end -->
