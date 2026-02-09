//! Passive UART Logger for RP2350A
//!
//! Captures bidirectional UART traffic (921600 baud) and writes to SD card
//! in binary format: [dir:1][timestamp_ms:4 LE][len:2 LE][data...]

#![no_std]
#![no_main]

mod config;
mod led;
mod sd_writer;

use config::*;
use led::{Color, Ws2812};
use sd_writer::{
    find_next_log_number, format_log_filename, CortexMDelay, DummyTimeSource, SdWriteBuffer,
};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, UART0, UART1};
use embassy_rp::pio::Pio;
use embassy_rp::spi;
use embassy_rp::uart::{BufferedUart, BufferedUartRx, Config as UartConfig};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};

use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io_async::Read;
use embedded_sdmmc::{Mode, SdCard, VolumeIdx, VolumeManager};
use portable_atomic::{AtomicU32, AtomicU8, Ordering};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

// ============================================================================
// Interrupt bindings
// ============================================================================

bind_interrupts!(struct Irqs {
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => embassy_rp::uart::BufferedInterruptHandler<UART1>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

// ============================================================================
// Shared state
// ============================================================================

/// UART packet for inter-task communication
struct UartPacket {
    direction: u8,
    timestamp_ms: u32,
    len: u16,
    data: [u8; MAX_PACKET_DATA],
}

/// Channel: uart_rx_task(s) → sd_writer_task
static UART_LOG_CHANNEL: Channel<CriticalSectionRawMutex, UartPacket, 32> = Channel::new();

/// System state for LED indication
static SYSTEM_STATE: AtomicU8 = AtomicU8::new(STATE_INIT);

/// Dropped packets counter
static PACKETS_DROPPED: AtomicU32 = AtomicU32::new(0);

// ============================================================================
// Entry point
// ============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("UART Logger starting...");

    // ── LED init (blue = STATE_INIT) ─────────────────────────────────────
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_25);
    spawner.must_spawn(led_task(ws2812));
    Timer::after(Duration::from_millis(500)).await;

    // ── Watchdog ──────────────────────────────────────────────────────
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(5));

    // ── SPI ────────────────────────────────────────────────────────────
    let mut spi_config = spi::Config::default();
    spi_config.frequency = SD_SPI_INIT_FREQ;
    let spi1 = spi::Spi::new_blocking(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, spi_config);
    let cs = Output::new(p.PIN_13, Level::High);

    // ── UART0 ───────────────────────────────────────────────────────────
    static TX_BUF0: StaticCell<[u8; 32]> = StaticCell::new();
    static RX_BUF0: StaticCell<[u8; UART_RX_BUF_SIZE]> = StaticCell::new();
    let tx_buf0 = &mut TX_BUF0.init([0; 32])[..];
    let rx_buf0 = &mut RX_BUF0.init([0; UART_RX_BUF_SIZE])[..];
    let mut uart0_config = UartConfig::default();
    uart0_config.baudrate = UART_BAUDRATE;
    let uart0 = BufferedUart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        tx_buf0,
        rx_buf0,
        uart0_config,
    );
    let (_uart0_tx, uart0_rx) = uart0.split();

    // ── UART1 ──────────────────────────────────────────────────────────
    static TX_BUF1: StaticCell<[u8; 32]> = StaticCell::new();
    static RX_BUF1: StaticCell<[u8; UART_RX_BUF_SIZE]> = StaticCell::new();
    let tx_buf1 = &mut TX_BUF1.init([0; 32])[..];
    let rx_buf1 = &mut RX_BUF1.init([0; UART_RX_BUF_SIZE])[..];
    let mut uart1_config = UartConfig::default();
    uart1_config.baudrate = UART_BAUDRATE;
    let uart1 = BufferedUart::new(
        p.UART1,
        p.PIN_4,
        p.PIN_5,
        Irqs,
        tx_buf1,
        rx_buf1,
        uart1_config,
    );
    let (_uart1_tx, uart1_rx) = uart1.split();

    // ── Spawn tasks ─────────────────────────────────────────────────────
    spawner.must_spawn(uart_rx_task(uart0_rx, DIR_LINE_A, "A"));
    spawner.must_spawn(uart_rx_task(uart1_rx, DIR_LINE_B, "B"));
    spawner.must_spawn(sd_writer_task(spi1, cs, watchdog));

    info!("All tasks spawned");
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

// ============================================================================
// UART RX task — reads from one UART, sends packets to channel
// ============================================================================

#[embassy_executor::task(pool_size = 2)]
async fn uart_rx_task(mut rx: BufferedUartRx, direction: u8, name: &'static str) {
    info!("uart_rx_task({}) started", name);

    let mut buf = [0u8; MAX_PACKET_DATA];

    loop {
        // Read available data (blocks until at least 1 byte)
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                let ts = Instant::now().as_millis() as u32;

                let mut packet = UartPacket {
                    direction,
                    timestamp_ms: ts,
                    len: n as u16,
                    data: [0u8; MAX_PACKET_DATA],
                };
                packet.data[..n].copy_from_slice(&buf[..n]);

                if UART_LOG_CHANNEL.try_send(packet).is_err() {
                    let dropped = PACKETS_DROPPED.fetch_add(1, Ordering::Relaxed) + 1;
                    if dropped % 100 == 1 {
                        warn!("Channel full, dropped {} packets (line {})", dropped, name);
                    }
                    // Ставим OVERFLOW только если сейчас RECORDING — не затираем SD_ERROR и т.д.
                    let _ = SYSTEM_STATE.compare_exchange(
                        STATE_RECORDING,
                        STATE_OVERFLOW,
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    );
                }
            }
            Ok(_) => {} // 0 bytes, retry
            Err(e) => {
                warn!("UART {} read error: {:?}", name, e);
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}

// ============================================================================
// SD writer task — receives packets, encodes and writes to SD card
// ============================================================================

type SdSpi = spi::Spi<'static, embassy_rp::peripherals::SPI1, spi::Blocking>;
type SdCs = Output<'static>;
type SdSpiDev = ExclusiveDevice<SdSpi, SdCs, CortexMDelay>;
type SdVolumeManager = VolumeManager<SdCard<SdSpiDev, CortexMDelay>, DummyTimeSource>;

/// Flush write buffer to SD card. Returns Ok(bytes_written) or Err on failure.
fn flush_buf_to_sd(
    volume_mgr: &SdVolumeManager,
    file: embedded_sdmmc::RawFile,
    write_buf: &mut SdWriteBuffer,
) -> Result<usize, ()> {
    if !write_buf.has_data() {
        return Ok(0);
    }
    let n = write_buf.as_slice().len();
    if let Err(e) = volume_mgr.write(file, write_buf.as_slice()) {
        error!("SD write error: {:?}", defmt::Debug2Format(&e));
        return Err(());
    }
    write_buf.clear();
    Ok(n)
}

/// Encode a packet into write_buf, flushing to SD if needed.
/// Returns false on SD write error.
fn encode_and_maybe_flush(
    volume_mgr: &SdVolumeManager,
    file: embedded_sdmmc::RawFile,
    write_buf: &mut SdWriteBuffer,
    bytes_written: &mut u32,
    direction: u8,
    timestamp_ms: u32,
    data: &[u8],
) -> bool {
    let n = write_buf.encode_into(direction, timestamp_ms, data);
    if n == 0 {
        // Buffer full — flush first, then encode
        match flush_buf_to_sd(volume_mgr, file, write_buf) {
            Ok(written) => *bytes_written = bytes_written.saturating_add(written as u32),
            Err(()) => return false,
        }
        if write_buf.encode_into(direction, timestamp_ms, data) == 0 {
            warn!(
                "Packet too large for write buffer ({}+7 > {})",
                data.len(),
                SD_WRITE_BUF_SIZE
            );
        }
    }
    true
}

#[embassy_executor::task]
async fn sd_writer_task(spi: SdSpi, cs: SdCs, mut watchdog: Watchdog) {
    info!("sd_writer_task started");

    // Create SPI device wrapper for embedded-sdmmc
    let spi_device = match ExclusiveDevice::new(spi, cs, CortexMDelay) {
        Ok(dev) => dev,
        Err(_) => {
            error!("Failed to create SPI device");
            SYSTEM_STATE.store(STATE_SD_ERROR, Ordering::Relaxed);
            loop {
                watchdog.feed();
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let sd_card = SdCard::new(spi_device, CortexMDelay);

    // SD init with retries
    let mut init_ok = false;
    for attempt in 1..=SD_INIT_RETRIES {
        watchdog.feed();
        info!("SD init attempt {}/{}...", attempt, SD_INIT_RETRIES);
        match sd_card.num_bytes() {
            Ok(bytes) => {
                info!("SD card: {} MB", bytes / (1024 * 1024));
                init_ok = true;
                break;
            }
            Err(e) => {
                warn!("SD init failed: {:?}", defmt::Debug2Format(&e));
                Timer::after(Duration::from_millis(500)).await;
                sd_card.mark_card_uninit();
            }
        }
    }

    if !init_ok {
        error!("SD card init failed after {} attempts", SD_INIT_RETRIES);
        SYSTEM_STATE.store(STATE_SD_ERROR, Ordering::Relaxed);
        // Watchdog will reboot us
        loop {
            watchdog.feed();
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    // P0: Switch SPI to working frequency (16 MHz) after SD init
    sd_card.spi(|dev| {
        dev.bus_mut().set_frequency(SD_SPI_WORK_FREQ);
    });
    info!("SPI switched to {} Hz", SD_SPI_WORK_FREQ);

    // Open volume manager (owns sd_card for the rest of the program lifetime)
    let volume_mgr = VolumeManager::new(sd_card, DummyTimeSource);

    let raw_volume = match volume_mgr.open_raw_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            error!("Failed to open volume: {:?}", defmt::Debug2Format(&e));
            SYSTEM_STATE.store(STATE_SD_ERROR, Ordering::Relaxed);
            loop {
                watchdog.feed();
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let root_dir = match volume_mgr.open_root_dir(raw_volume) {
        Ok(d) => d,
        Err(e) => {
            error!("Failed to open root dir: {:?}", defmt::Debug2Format(&e));
            SYSTEM_STATE.store(STATE_SD_ERROR, Ordering::Relaxed);
            loop {
                watchdog.feed();
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    // ── SD card write/read/verify test ──────────────────────────────────
    SYSTEM_STATE.store(STATE_SD_TEST, Ordering::Relaxed);
    info!("SD card test: write/read/verify...");
    {
        let test_ok = (|| -> Result<(), &'static str> {
            // Write test pattern (512 bytes: 0x00..0xFF, 0x00..0xFF)
            let test_file = volume_mgr
                .open_file_in_dir(root_dir, "TEST.BIN", Mode::ReadWriteCreateOrTruncate)
                .map_err(|_| "create test file")?;
            let mut pattern = [0u8; 512];
            for (i, b) in pattern.iter_mut().enumerate() {
                *b = i as u8;
            }
            volume_mgr
                .write(test_file, &pattern)
                .map_err(|_| "write test data")?;
            volume_mgr
                .close_file(test_file)
                .map_err(|_| "close after write")?;

            // Read back and verify
            let test_file = volume_mgr
                .open_file_in_dir(root_dir, "TEST.BIN", Mode::ReadOnly)
                .map_err(|_| "open test file for read")?;
            let mut readback = [0u8; 512];
            let bytes_read = volume_mgr
                .read(test_file, &mut readback)
                .map_err(|_| "read test data")?;
            volume_mgr
                .close_file(test_file)
                .map_err(|_| "close after read")?;

            if bytes_read != 512 {
                return Err("wrong read length");
            }
            if readback != pattern {
                return Err("data mismatch");
            }

            // Cleanup: delete test file
            let _ = volume_mgr.delete_file_in_dir(root_dir, "TEST.BIN");

            Ok(())
        })();

        match test_ok {
            Ok(()) => info!("SD card test PASSED"),
            Err(step) => {
                error!("SD card test FAILED at: {}", step);
                SYSTEM_STATE.store(STATE_SD_ERROR, Ordering::Relaxed);
                loop {
                    watchdog.feed();
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        }
    }
    info!("SD test passed, starting recording");

    let mut log_num = find_next_log_number(&volume_mgr, root_dir);

    // ── Main loop: open file → write → rotate/recover ───────────────────
    loop {
        watchdog.feed();

        let filename_bytes = format_log_filename(log_num);
        let filename = core::str::from_utf8(&filename_bytes).unwrap_or("LOG_0001.BIN");
        info!("Opening log file: {}", filename);

        let file = match volume_mgr.open_file_in_dir(
            root_dir,
            filename,
            Mode::ReadWriteCreateOrTruncate,
        ) {
            Ok(f) => f,
            Err(e) => {
                error!("Failed to create log file: {:?} — watchdog reboot in 5s...", defmt::Debug2Format(&e));
                SYSTEM_STATE.store(STATE_SD_ERROR, Ordering::Relaxed);
                while UART_LOG_CHANNEL.try_receive().is_ok() {}
                loop {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        };

        info!("Recording to {}", filename);
        SYSTEM_STATE.store(STATE_RECORDING, Ordering::Relaxed);

        // ── Write loop for current file ─────────────────────────────────
        let mut write_buf = SdWriteBuffer::new();
        let mut last_sync = Instant::now();
        let mut bytes_written: u32 = 0;
        let mut sd_error = false;

        'write_loop: loop {
            watchdog.feed();

            // Wait for packet or timeout
            let packet = match embassy_futures::select::select(
                UART_LOG_CHANNEL.receive(),
                Timer::after(Duration::from_millis(SD_FLUSH_TIMEOUT_MS)),
            )
            .await
            {
                embassy_futures::select::Either::First(pkt) => Some(pkt),
                embassy_futures::select::Either::Second(_timeout) => None,
            };

            if let Some(pkt) = packet {
                // Encode directly into write buffer
                let len = (pkt.len as usize).min(MAX_PACKET_DATA);
                if !encode_and_maybe_flush(
                    &volume_mgr,
                    file,
                    &mut write_buf,
                    &mut bytes_written,
                    pkt.direction,
                    pkt.timestamp_ms,
                    &pkt.data[..len],
                ) {
                    sd_error = true;
                    break 'write_loop;
                }

                // Batch receive: drain all pending packets
                while let Ok(batch_pkt) = UART_LOG_CHANNEL.try_receive() {
                    let batch_len = (batch_pkt.len as usize).min(MAX_PACKET_DATA);
                    if !encode_and_maybe_flush(
                        &volume_mgr,
                        file,
                        &mut write_buf,
                        &mut bytes_written,
                        batch_pkt.direction,
                        batch_pkt.timestamp_ms,
                        &batch_pkt.data[..batch_len],
                    ) {
                        sd_error = true;
                        break 'write_loop;
                    }
                }

                // Channel drained — clear overflow indication if it was set
                if SYSTEM_STATE.load(Ordering::Relaxed) == STATE_OVERFLOW {
                    SYSTEM_STATE.store(STATE_RECORDING, Ordering::Relaxed);
                }

                // Flush if buffer is >75% full
                if write_buf.remaining() < SD_WRITE_BUF_SIZE / 4 {
                    match flush_buf_to_sd(&volume_mgr, file, &mut write_buf) {
                        Ok(written) => bytes_written = bytes_written.saturating_add(written as u32),
                        Err(()) => {
                            sd_error = true;
                            break 'write_loop;
                        }
                    }
                }
            } else {
                // Timeout — flush if there's data
                if write_buf.has_data() {
                    match flush_buf_to_sd(&volume_mgr, file, &mut write_buf) {
                        Ok(written) => bytes_written = bytes_written.saturating_add(written as u32),
                        Err(()) => {
                            sd_error = true;
                            break 'write_loop;
                        }
                    }
                    last_sync = Instant::now();
                }

                // SD heartbeat: periodic flush to detect card removal during idle
                if last_sync.elapsed() > Duration::from_millis(SD_HEARTBEAT_INTERVAL_MS) {
                    if volume_mgr.flush_file(file).is_err() {
                        warn!("SD heartbeat flush failed, retry in 100ms...");
                        Timer::after(Duration::from_millis(100)).await;
                        if let Err(e) = volume_mgr.flush_file(file) {
                            error!("SD heartbeat flush retry failed: {:?}", defmt::Debug2Format(&e));
                            sd_error = true;
                            break 'write_loop;
                        }
                    }
                    last_sync = Instant::now();
                }
            }

            // Periodic sync during active writing (every SD_SYNC_INTERVAL_MS)
            if last_sync.elapsed() > Duration::from_millis(SD_SYNC_INTERVAL_MS) {
                if write_buf.has_data() {
                    match flush_buf_to_sd(&volume_mgr, file, &mut write_buf) {
                        Ok(written) => bytes_written = bytes_written.saturating_add(written as u32),
                        Err(()) => {
                            sd_error = true;
                            break 'write_loop;
                        }
                    }
                }
                if volume_mgr.flush_file(file).is_err() {
                    warn!("SD sync flush failed, retry in 100ms...");
                    Timer::after(Duration::from_millis(100)).await;
                    if let Err(e) = volume_mgr.flush_file(file) {
                        error!("SD sync retry failed: {:?}", defmt::Debug2Format(&e));
                        sd_error = true;
                        break 'write_loop;
                    }
                }
                last_sync = Instant::now();
            }

            // File rotation by size
            if bytes_written >= MAX_LOG_FILE_SIZE {
                info!("Log file reached {} bytes, rotating...", bytes_written);
                break 'write_loop;
            }
        }

        // ── Cleanup current file ────────────────────────────────────────
        let _ = flush_buf_to_sd(&volume_mgr, file, &mut write_buf);
        let _ = volume_mgr.flush_file(file);
        let _ = volume_mgr.close_file(file);

        if sd_error {
            error!("SD fatal error — watchdog reboot in 5s...");
            SYSTEM_STATE.store(STATE_SD_ERROR, Ordering::Relaxed);
            // Drain channel to prevent uart_rx_task from blocking
            while UART_LOG_CHANNEL.try_receive().is_ok() {}
            // Не кормим watchdog → перезагрузка через ~5 секунд
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }

        log_num = log_num.wrapping_add(1);
        // Continue outer loop: open next file (rotation)
    }
}

// ============================================================================
// LED task — displays system state via WS2812B
// ============================================================================

#[embassy_executor::task]
async fn led_task(mut ws2812: Ws2812<'static, PIO0, 0>) {
    info!("led_task started");

    let mut tick = false;

    loop {
        let state = SYSTEM_STATE.load(Ordering::Relaxed);

        match state {
            STATE_INIT => {
                // Blue fast blink
                let color = if tick { Color::blue() } else { Color::off() };
                ws2812.write_color(color).await;
                Timer::after(Duration::from_millis(150)).await;
            }
            STATE_SD_ERROR => {
                // Red slow blink
                let color = if tick { Color::red() } else { Color::off() };
                ws2812.write_color(color).await;
                Timer::after(Duration::from_millis(500)).await;
            }
            STATE_RECORDING => {
                // Solid green
                ws2812.write_color(Color::green()).await;
                Timer::after(Duration::from_millis(500)).await;
            }
            STATE_OVERFLOW => {
                // Yellow blink — stays until cleared by sd_writer_task
                let color = if tick { Color::yellow() } else { Color::off() };
                ws2812.write_color(color).await;
                Timer::after(Duration::from_millis(250)).await;
            }
            STATE_SD_TEST => {
                // Magenta pulse — SD test in progress
                let color = if tick { Color::magenta() } else { Color::off() };
                ws2812.write_color(color).await;
                Timer::after(Duration::from_millis(200)).await;
            }
            _ => {
                ws2812.write_color(Color::off()).await;
                Timer::after(Duration::from_millis(500)).await;
            }
        }

        tick = !tick;
    }
}
