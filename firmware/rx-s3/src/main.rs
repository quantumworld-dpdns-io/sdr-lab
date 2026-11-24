//! ESP32-S3 "RX" board. Sync methods: wired trigger, Wi-Fi TSF, ESP-NOW
//! frames, BLE (lab_fw::radio). Status LED (WS2812, GPIO48) blinks green while running.
//!
//! Default build — link to the controller board (docs/wiring.md):
//!   UART1 TX=GPIO17 → CTRL, RX=GPIO18 ← CTRL; trigger in = GPIO4.
//! `--features host-link` — link to the Mac over the UART/COM USB connector
//!   (`host-link-usb`: native USB connector); trigger in = GPIO4, driven by
//!   TX GPIO5. Shared GND with TX.

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use lab_fw::board::BoardInfo;
use lab_fw::led::{spawn_blink, Rgb};
use lab_fw::radio::{Radio, Role};
use lab_proto::Board;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    // DevKitC-1 v1.1 boards use GPIO38 for the LED instead.
    spawn_blink(p.pins.gpio48, Rgb::GREEN)?;
    let info = BoardInfo {
        board: Board::Rx,
        chip: "esp32s3",
    };

    #[cfg(feature = "host-link")]
    let (link, rx) = lab_fw::peer::host_uart0_link(&info, p.uart0, p.pins.gpio43, p.pins.gpio44)?;
    #[cfg(feature = "host-link-usb")]
    let (link, rx) = lab_fw::link::usb_jtag::link(
        "usb", p.usb_serial, p.pins.gpio19, p.pins.gpio20, info.board,
    )?;
    #[cfg(not(any(feature = "host-link", feature = "host-link-usb")))]
    let (link, rx) = lab_fw::peer::uart_link(&info, p.uart1, p.pins.gpio17, p.pins.gpio18)?;

    let radio = Radio::new(Role::Rx, p.modem, sysloop, nvs)?;
    lab_fw::peer::run(info, link, rx, p.pins.gpio4, None, Some(Box::new(radio)))
}
