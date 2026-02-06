//! Configuration constants for UART logger

/// UART baudrate for both channels
pub const UART_BAUDRATE: u32 = 921_600;

/// Direction markers in binary log format
pub const DIR_LINE_A: u8 = 0x00; // UART0 RX (GPIO1)
pub const DIR_LINE_B: u8 = 0x01; // UART1 RX (GPIO5)

/// Embassy BufferedUart RX buffer size per channel (~44 ms at 921600 baud)
pub const UART_RX_BUF_SIZE: usize = 4096;

/// Max data payload per log record
pub const MAX_PACKET_DATA: usize = 256;

/// SD write buffer size — 16 sectors (flushed when full or on timeout)
pub const SD_WRITE_BUF_SIZE: usize = 8192;

/// Flush SD buffer if no new data for this many ms
pub const SD_FLUSH_TIMEOUT_MS: u64 = 100;

/// SD SPI initialization frequency (slow for card init)
pub const SD_SPI_INIT_FREQ: u32 = 400_000;

/// SD SPI working frequency after init
pub const SD_SPI_WORK_FREQ: u32 = 16_000_000;

/// Max retries for SD card initialization
pub const SD_INIT_RETRIES: u8 = 3;

/// Max log file size before rotation (100 MB)
pub const MAX_LOG_FILE_SIZE: u32 = 100 * 1024 * 1024;

/// Flush file metadata to SD every N seconds during active writing
pub const SD_SYNC_INTERVAL_MS: u64 = 5_000;

/// System state values for LED indication (AtomicU8)
pub const STATE_INIT: u8 = 0;
pub const STATE_SD_ERROR: u8 = 1;
pub const STATE_RECORDING: u8 = 2;
pub const STATE_OVERFLOW: u8 = 3;
