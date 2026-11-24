//! ESP32-WROOM-32 controller. M1: host bridge, peer liveness, trigger sync.
//!
//! Wiring (docs/wiring.md):
//!   UART0 (USB bridge, GPIO1/3)  ↔ host
//!   UART1 TX=GPIO25 RX=GPIO26    ↔ TX-S3
//!   UART2 TX=GPIO17 RX=GPIO16    ↔ RX-S3
//!   GPIO27                       → trigger line

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use anyhow::Result;
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use lab_core::ctrl::Controller;
use lab_core::{Action, Actions, Port};
use lab_fw::board::BoardInfo;
use lab_fw::link::Link;
use lab_fw::trigger::TriggerOut;
use lab_proto::{Board, Msg, RunState, TriggerEvt};

const INFO: BoardInfo = BoardInfo {
    board: Board::Ctrl,
    chip: "esp32",
};

struct Links {
    host: Arc<Link>,
    tx: Arc<Link>,
    rx: Arc<Link>,
    /// Peer frames forwarded to the host (shown as Status.records).
    forwarded: AtomicU32,
    pulse: mpsc::SyncSender<(u16, u16)>,
}

impl Links {
    fn port(&self, p: Port) -> &Link {
        match p {
            Port::Host => &self.host,
            Port::Tx => &self.tx,
            Port::Rx => &self.rx,
        }
    }

    fn errors(&self) -> u32 {
        [&self.host, &self.tx, &self.rx]
            .iter()
            .map(|l| l.rx_errors.load(Ordering::Relaxed) + l.tx_errors.load(Ordering::Relaxed))
            .sum()
    }

    fn dispatch(&self, actions: Actions) {
        for a in actions {
            match a {
                Action::Send(p, m) => self.port(p).send(&m),
                Action::SendHello(p) => self.port(p).send(&Msg::Hello(INFO.hello())),
                Action::SendStatus(p) => {
                    let s = INFO.status(
                        RunState::Idle,
                        self.forwarded.load(Ordering::Relaxed),
                        self.errors(),
                    );
                    self.port(p).send(&Msg::Status(s));
                }
                Action::Pulse { pulses, period_ms } => {
                    if self.pulse.try_send((pulses, period_ms)).is_err() {
                        log::warn!("sync test already running; request dropped");
                    }
                }
            }
        }
    }
}

fn uart(
    uart: impl Uart + 'static,
    tx: impl OutputPin + 'static,
    rx: impl InputPin + 'static,
) -> Result<UartDriver<'static>> {
    Ok(UartDriver::new(
        uart,
        tx,
        rx,
        None::<AnyIOPin>,
        None::<AnyIOPin>,
        &lab_fw::link_uart_config(),
    )?)
}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    // Logs share UART0 with the host link. Keep them sparse; the link's
    // leading delimiter makes any interleaved text harmless.
    log::set_max_level(log::LevelFilter::Warn);

    let p = Peripherals::take()?;
    let host_uart = uart(p.uart0, p.pins.gpio1, p.pins.gpio3)?;
    // Route console writes through the driver too, so each write is atomic
    // with respect to our frame writes (no mid-frame interleaving).
    // SAFETY: the UART0 driver was installed just above.
    unsafe { esp_idf_svc::sys::uart_vfs_dev_use_driver(0) };
    let (host, host_rx) = Link::new("host", host_uart, Board::Ctrl);
    let (tx, tx_rx) = Link::new("tx", uart(p.uart1, p.pins.gpio25, p.pins.gpio26)?, Board::Ctrl);
    let (rx, rx_rx) = Link::new("rx", uart(p.uart2, p.pins.gpio17, p.pins.gpio16)?, Board::Ctrl);
    let mut trigger = TriggerOut::new(p.pins.gpio27)?;

    let (pulse_tx, pulse_rx) = mpsc::sync_channel::<(u16, u16)>(1);
    let links = Arc::new(Links {
        host,
        tx,
        rx,
        forwarded: AtomicU32::new(0),
        pulse: pulse_tx,
    });
    let ctrl = Arc::new(Mutex::new(Controller::new()));

    // Host → controller commands.
    {
        let (l, c) = (Arc::clone(&links), Arc::clone(&ctrl));
        links.host.spawn_reader(host_rx, move |_h, msg, _| match msg {
            Ok(m) => {
                let a = c.lock().unwrap().on_host(&m);
                l.dispatch(a);
            }
            Err(e) => log::warn!("host: undecodable message: {e}"),
        })?;
    }
    // Peer → controller: track liveness, forward everything to the host
    // verbatim (original src/seq preserved, unknown types included).
    for (port, link, rxd) in [(Port::Tx, &links.tx, tx_rx), (Port::Rx, &links.rx, rx_rx)] {
        let (l, c) = (Arc::clone(&links), Arc::clone(&ctrl));
        link.spawn_reader(rxd, move |h, msg, body| {
            if let Ok(m) = &msg {
                c.lock().unwrap().on_peer(port, h.src, m, lab_fw::now_ms());
            }
            l.host.forward(body);
            l.forwarded.fetch_add(1, Ordering::Relaxed);
        })?;
    }
    // Sync-test pulses: own thread, so command handling stays responsive.
    {
        let l = Arc::clone(&links);
        thread::Builder::new()
            .name("pulse".into())
            .stack_size(lab_fw::THREAD_STACK)
            .spawn(move || {
                for (pulses, period_ms) in pulse_rx {
                    for _ in 0..pulses {
                        match trigger.pulse() {
                            Ok((edge, t)) => l.host.send(&Msg::TriggerEvt(TriggerEvt {
                                exp_id: 0,
                                t0_local_us: t,
                                edge,
                            })),
                            Err(e) => log::error!("trigger pulse failed: {e}"),
                        }
                        thread::sleep(Duration::from_millis(period_ms as u64));
                    }
                }
            })?;
    }

    let boot = ctrl.lock().unwrap().on_boot();
    links.dispatch(boot);
    loop {
        let a = ctrl.lock().unwrap().tick(lab_fw::now_ms());
        links.dispatch(a);
        thread::sleep(Duration::from_millis(50));
    }
}
