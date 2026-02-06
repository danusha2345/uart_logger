//! SD card writer — init, binary record encoding, buffered writes

use embedded_hal::delay::DelayNs;
use embedded_sdmmc::{BlockDevice, RawDirectory, TimeSource, Timestamp, VolumeManager};

use crate::config::*;

/// Dummy time source — returns fixed timestamp (no RTC on board)
pub struct DummyTimeSource;

impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        // 2025-01-01 00:00:00
        Timestamp::from_calendar(2025, 1, 1, 0, 0, 0).unwrap()
    }
}

/// Encode a binary log record: [dir:1][timestamp_ms:4 LE][length:2 LE][data...]
/// Returns number of bytes written to `out`
pub fn encode_record(out: &mut [u8], direction: u8, timestamp_ms: u32, data: &[u8]) -> usize {
    let len = data.len().min(MAX_PACKET_DATA);
    let total = 1 + 4 + 2 + len; // header + data
    if out.len() < total {
        return 0;
    }

    out[0] = direction;
    out[1..5].copy_from_slice(&timestamp_ms.to_le_bytes());
    out[5..7].copy_from_slice(&(len as u16).to_le_bytes());
    out[7..7 + len].copy_from_slice(&data[..len]);

    total
}

/// 4KB write buffer with flush-on-full semantics
pub struct SdWriteBuffer {
    buf: [u8; SD_WRITE_BUF_SIZE],
    pos: usize,
}

impl SdWriteBuffer {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; SD_WRITE_BUF_SIZE],
            pos: 0,
        }
    }

    /// Append data to the buffer. Returns true if buffer became full and needs flush.
    pub fn append(&mut self, data: &[u8]) -> bool {
        let remaining = SD_WRITE_BUF_SIZE - self.pos;
        let to_copy = data.len().min(remaining);
        self.buf[self.pos..self.pos + to_copy].copy_from_slice(&data[..to_copy]);
        self.pos += to_copy;
        self.pos >= SD_WRITE_BUF_SIZE
    }

    /// Get buffered data as slice
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    /// Reset buffer position after flush
    pub fn clear(&mut self) {
        self.pos = 0;
    }

    /// Check if buffer has data
    pub fn has_data(&self) -> bool {
        self.pos > 0
    }
}

/// Find the next available log file number by scanning root directory.
/// Uses Raw API to avoid complex type inference with wrapper types.
pub fn find_next_log_number<D, T, const DIRS: usize, const FILES: usize, const VOLS: usize>(
    volume_mgr: &VolumeManager<D, T, DIRS, FILES, VOLS>,
    root_dir: RawDirectory,
) -> u16
where
    D: BlockDevice,
    T: TimeSource,
    <D as BlockDevice>::Error: core::fmt::Debug,
{
    let mut max_num: u16 = 0;

    let _ = volume_mgr.iterate_dir(root_dir, |entry| {
        let name = entry.name.base_name();
        let ext = entry.name.extension();

        // Check for "LOG_" prefix and "BIN" extension
        if name.len() >= 8 && ext == b"BIN" && name[0..4] == *b"LOG_" {
            if let Some(num) = parse_4digit_ascii(&name[4..8]) {
                if num > max_num {
                    max_num = num;
                }
            }
        }
    });

    max_num + 1
}

/// Parse 4 ASCII digits into u16
fn parse_4digit_ascii(digits: &[u8]) -> Option<u16> {
    if digits.len() < 4 {
        return None;
    }
    let mut result: u16 = 0;
    for &d in &digits[0..4] {
        if !d.is_ascii_digit() {
            return None;
        }
        result = result * 10 + (d - b'0') as u16;
    }
    Some(result)
}

/// Format log filename: "LOG_0001.BIN" etc.
pub fn format_log_filename(num: u16) -> [u8; 12] {
    let mut name = *b"LOG_0000.BIN";
    let d0 = (num / 1000) % 10;
    let d1 = (num / 100) % 10;
    let d2 = (num / 10) % 10;
    let d3 = num % 10;
    name[4] = b'0' + d0 as u8;
    name[5] = b'0' + d1 as u8;
    name[6] = b'0' + d2 as u8;
    name[7] = b'0' + d3 as u8;
    name
}

/// Minimal delay implementation using busy-wait (cortex-m)
pub struct CortexMDelay;

impl DelayNs for CortexMDelay {
    fn delay_ns(&mut self, ns: u32) {
        // At 150 MHz (RP2350), ~6.67 ns per cycle
        let cycles = ns / 7 + 1;
        cortex_m::asm::delay(cycles);
    }
}
