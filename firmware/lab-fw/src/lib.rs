//! ESP-IDF shell shared by all SDR Lab boards: UART links, trigger I/O,
//! board identity. Board logic lives in `lab-core` (host-tested); this crate
//! only performs the I/O that logic asks for.

pub mod board;
pub mod led;
pub mod link;
pub mod peer;
#[cfg(feature = "radio")]
pub mod radio;
pub mod trigger;

use esp_idf_svc::hal::uart::config::Config as UartConfig;
use esp_idf_svc::hal::units::Hertz;

/// Board ↔ board UART links (short direct wires): 8N1, no flow control.
pub const LINK_BAUD: u32 = 921_600;

/// Links to the Mac through a devkit's USB-UART bridge (UART0 / COM port).
/// macOS's built-in CH34x driver garbles 921 600 baud; 115 200 is reliable
/// and matches the boot ROM/console rate, so boot logs stay readable too.
/// High-rate streams (CSI, M3) should use the native USB port instead.
pub const HOST_UART_BAUD: u32 = 115_200;

pub fn link_uart_config() -> UartConfig {
    uart_config(LINK_BAUD)
}

pub fn host_uart_config() -> UartConfig {
    uart_config(HOST_UART_BAUD)
}

fn uart_config(baud: u32) -> UartConfig {
    UartConfig::default()
        .baudrate(Hertz(baud))
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
