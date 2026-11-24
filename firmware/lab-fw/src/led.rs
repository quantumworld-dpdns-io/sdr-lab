//! On-board WS2812 RGB status LED (ESP32-S3-DevKitC-1), driven by RMT.
//!
//! The LED is on GPIO48 on DevKitC-1 v1.0 and most clones, GPIO38 on v1.1.
//!
//! Board colours: TX = blue, RX = green, blinking 500 ms on / 500 ms off
//! while the firmware is running.

use std::thread;
use std::time::Duration;

use anyhow::Result;
use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::rmt::config::TransmitConfig;
use esp_idf_svc::hal::rmt::config::TxChannelConfig;
use esp_idf_svc::hal::rmt::encoder::{BytesEncoder, BytesEncoderConfig};
use esp_idf_svc::hal::rmt::{PinState, Symbol, TxChannelDriver};
use esp_idf_svc::hal::units::Hertz;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rgb(pub u8, pub u8, pub u8);

impl Rgb {
    pub const OFF: Rgb = Rgb(0, 0, 0);
    /// Kept dim: full brightness is blinding at desk distance.
    pub const BLUE: Rgb = Rgb(0, 0, 40);
    pub const GREEN: Rgb = Rgb(0, 40, 0);
}

/// 10 MHz RMT clock → 100 ns resolution, enough for WS2812 timing.
const RESOLUTION: Hertz = Hertz(10_000_000);
pub const BLINK_HALF_PERIOD: Duration = Duration::from_millis(500);

pub struct StatusLed {
    ch: TxChannelDriver<'static>,
    enc: BytesEncoder,
}

impl StatusLed {
    pub fn new(pin: impl OutputPin + 'static) -> Result<Self> {
        let ch = TxChannelDriver::new(
            pin,
            &TxChannelConfig {
                resolution: RESOLUTION,
                ..Default::default()
            },
        )?;
        let ns = Duration::from_nanos;
        // WS2812: 0 = 0.4 µs high / 0.85 µs low, 1 = 0.8 µs high / 0.45 µs low.
        let enc = BytesEncoder::with_config(&BytesEncoderConfig {
            bit0: Symbol::new_with(RESOLUTION, PinState::High, ns(400), PinState::Low, ns(850))?,
            bit1: Symbol::new_with(RESOLUTION, PinState::High, ns(800), PinState::Low, ns(450))?,
            msb_first: true,
            ..Default::default()
        })?;
        Ok(Self { ch, enc })
    }

    pub fn set(&mut self, Rgb(r, g, b): Rgb) -> Result<()> {
        // WS2812 wire order is GRB. The line idles low (eot_level = false),
        // which provides the >50 µs reset/latch between updates.
        self.ch
            .send_and_wait(&mut self.enc, &[g, r, b], &TransmitConfig::default())?;
        Ok(())
    }
}

/// Blink `color` forever on a background thread.
pub fn spawn_blink(pin: impl OutputPin + Send + 'static, color: Rgb) -> Result<()> {
    // The RMT encoder is !Send, so the driver is created on the LED thread;
    // setup errors are handed back so they still fail boot loudly.
    let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel::<Result<()>>(1);
    thread::Builder::new()
        .name("status-led".into())
        .stack_size(4096)
        .spawn(move || {
            let mut led = match StatusLed::new(pin).and_then(|mut l| l.set(Rgb::OFF).map(|_| l)) {
                Ok(l) => {
                    let _ = ready_tx.send(Ok(()));
                    l
                }
                Err(e) => {
                    let _ = ready_tx.send(Err(e));
                    return;
                }
            };
            let mut on = false;
            loop {
                on = !on;
                if let Err(e) = led.set(if on { color } else { Rgb::OFF }) {
                    log::warn!("status LED: {e}");
                }
                thread::sleep(BLINK_HALF_PERIOD);
            }
        })?;
    ready_rx.recv()?
}
