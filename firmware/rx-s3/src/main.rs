//! ESP32-S3 "RX" board. M1: link to CTRL, heartbeat, trigger timestamps.
//!
//! Wiring (docs/wiring.md): UART1 TX=GPIO17 → CTRL, RX=GPIO18 ← CTRL,
//! trigger in = GPIO4. Status LED (WS2812, GPIO48) blinks green while running.

use esp_idf_svc::hal::peripherals::Peripherals;
use lab_fw::board::BoardInfo;
use lab_fw::led::{spawn_blink, Rgb};
use lab_proto::Board;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    // DevKitC-1 v1.1 boards use GPIO38 for the LED instead.
    spawn_blink(p.pins.gpio48, Rgb::GREEN)?;
    let info = BoardInfo {
        board: Board::Rx,
        chip: "esp32s3",
    };
    lab_fw::peer::run(info, p.uart1, p.pins.gpio17, p.pins.gpio18, p.pins.gpio4)
}
