//! ESP-IDF shell shared by all SDR Lab boards: UART links, trigger I/O,
//! board identity. Board logic lives in `lab-core` (host-tested); this crate
//! only performs the I/O that logic asks for.

pub mod board;
pub mod led;
pub mod link;
pub mod peer;
pub mod trigger;

use esp_idf_svc::hal::uart::config::Config as UartConfig;
use esp_idf_svc::hal::units::Hertz;

/// All inter-board and host links run at this rate (8N1, no flow control).
pub const LINK_BAUD: u32 = 921_600;

pub fn link_uart_config() -> UartConfig {
    UartConfig::default()
        .baudrate(Hertz(LINK_BAUD))
        // CSI bursts (M3) need headroom; 4 KiB ≈ 44 ms at 921 600 baud.
        .rx_fifo_size(4096)
        .tx_fifo_size(4096)
}

/// Milliseconds since boot (monotonic, from `esp_timer`).
pub fn now_ms() -> u64 {
    now_us() / 1000
}

/// Microseconds since boot (monotonic, from `esp_timer`).
pub fn now_us() -> u64 {
    // SAFETY: esp_timer_get_time has no preconditions and is ISR-safe.
    unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 }
}

/// Stack for link reader / tick threads: decoder (~1.6 KiB) + action buffer
/// (~2.5 KiB) + encode buffers (~1.6 KiB) with margin.
pub const THREAD_STACK: usize = 12 * 1024;
