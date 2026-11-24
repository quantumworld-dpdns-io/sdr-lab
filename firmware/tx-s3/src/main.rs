//! ESP32-S3 "TX" board. Status LED (WS2812, GPIO48) blinks blue while running.
//!
//! Default build — link to the controller board (docs/wiring.md):
//!   UART1 TX=GPIO17 → CTRL, RX=GPIO18 ← CTRL; trigger in = GPIO4.
//! `--features host-link` — link to the Mac over native USB, and act as the
//!   trigger master: GPIO5 → own GPIO4 and RX GPIO4 (jumpers), shared GND.

use esp_idf_svc::hal::peripherals::Peripherals;
use lab_fw::board::BoardInfo;
use lab_fw::led::{spawn_blink, Rgb};
use lab_proto::Board;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    // DevKitC-1 v1.1 boards use GPIO38 for the LED instead.
    spawn_blink(p.pins.gpio48, Rgb::BLUE)?;
    let info = BoardInfo {
        board: Board::Tx,
        chip: "esp32s3",
    };

    #[cfg(feature = "host-link")]
    let (link, rx, trigger_out) = {
        let (link, rx) = lab_fw::link::usb_jtag::link(
            "usb", p.usb_serial, p.pins.gpio19, p.pins.gpio20, info.board,
        )?;
        let out = lab_fw::trigger::TriggerOut::new(p.pins.gpio5)?;
        (link, rx, Some(out))
    };
    #[cfg(not(feature = "host-link"))]
    let (link, rx, trigger_out) = {
        let (link, rx) = lab_fw::peer::uart_link(&info, p.uart1, p.pins.gpio17, p.pins.gpio18)?;
        (link, rx, None)
    };

    lab_fw::peer::run(info, link, rx, p.pins.gpio4, trigger_out)
}
