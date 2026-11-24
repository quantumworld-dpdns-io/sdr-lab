//! Main loop shared by TX and RX: link up (to CTRL or the Mac), heartbeat,
//! trigger timestamps, and — when this board is the trigger master — pulses.
//! Role-specific tasks (Wi-Fi TX, sniffer, CSI) attach here from M2 on.

use std::sync::atomic::Ordering;
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::Duration;

use anyhow::Result;
use esp_idf_svc::hal::gpio::InputPin;
use lab_core::{peer::Peer, Action, Actions};
use lab_proto::{Msg, RunState, TriggerEvt};

use crate::board::BoardInfo;
use crate::link::{Link, LinkRead};
use crate::trigger::TriggerOut;

struct Ctx {
    info: BoardInfo,
    link: Arc<Link>,
    pulse: Option<mpsc::SyncSender<(u16, u16)>>,
}

impl Ctx {
    fn dispatch(&self, actions: Actions) {
        for a in actions {
            match a {
                Action::Send(_, m) => self.link.send(&m),
                Action::SendHello(_) => self.link.send(&Msg::Hello(self.info.hello())),
                Action::SendStatus(_) => {
                    let dropped = self.link.rx_errors.load(Ordering::Relaxed)
                        + self.link.tx_errors.load(Ordering::Relaxed);
                    self.link
                        .send(&Msg::Status(self.info.status(RunState::Idle, 0, dropped)));
                }
                Action::Pulse { pulses, period_ms } => match &self.pulse {
                    Some(tx) => {
                        if tx.try_send((pulses, period_ms)).is_err() {
                            log::warn!("sync test already running; request dropped");
                        }
                    }
                    None => log::warn!("not the trigger master; pulse ignored"),
                },
            }
        }
    }
}

/// Run the peer firmware forever.
///
/// * `link`/`rx` — the upstream link (UART to CTRL, or USB to the Mac).
/// * `trigger_in` — GPIO4, timestamped in an ISR.
/// * `trigger_out` — `Some` on the trigger master (host-link TX): its pulses
///   go out on GPIO5, which is jumpered back to its own GPIO4 and to RX GPIO4,
///   so both boards timestamp the same edge through the same ISR path.
pub fn run(
    info: BoardInfo,
    link: Arc<Link>,
    rx: Box<dyn LinkRead>,
    trigger_in: impl InputPin + 'static,
    trigger_out: Option<TriggerOut>,
) -> Result<()> {
    let master = trigger_out.is_some();
    let pulse = match trigger_out {
        Some(mut out) => {
            let (tx, prx) = mpsc::sync_channel::<(u16, u16)>(1);
            thread::Builder::new()
                .name("pulse".into())
                .stack_size(crate::THREAD_STACK)
                .spawn(move || {
                    for (pulses, period_ms) in prx {
                        for _ in 0..pulses {
                            if let Err(e) = out.pulse() {
                                log::error!("trigger pulse failed: {e}");
                            }
                            thread::sleep(Duration::from_millis(period_ms as u64));
                        }
                    }
                })?;
            Some(tx)
        }
        None => None,
    };

    let ctx = Arc::new(Ctx { info, link, pulse });
    let core = Arc::new(Mutex::new(if master {
        Peer::trigger_master()
    } else {
        Peer::new()
    }));

    {
        let (ctx, core) = (Arc::clone(&ctx), Arc::clone(&core));
        ctx.link.clone().spawn_reader(rx, move |_h, msg, _body| match msg {
            Ok(m) => {
                let actions = core.lock().unwrap().on_msg(&m);
                ctx.dispatch(actions);
            }
            Err(e) => log::warn!("upstream: undecodable message: {e}"),
        })?;
    }
    {
        let link = Arc::clone(&ctx.link);
        crate::trigger::spawn_trigger_in(trigger_in, move |edge, t| {
            link.send(&Msg::TriggerEvt(TriggerEvt {
                exp_id: 0,
                t0_local_us: t,
                edge,
            }));
        })?;
    }

    let boot = core.lock().unwrap().on_boot();
    ctx.dispatch(boot);
    log::info!(
        "{:?} ready (link: {}, trigger master: {master})",
        info.board,
        ctx.link.name
    );
    loop {
        let actions = core.lock().unwrap().tick(crate::now_ms());
        ctx.dispatch(actions);
        thread::sleep(Duration::from_millis(50));
    }
}

/// UART link to the controller board (default build).
pub fn uart_link(
    info: &BoardInfo,
    uart: impl esp_idf_svc::hal::uart::Uart + 'static,
    tx_pin: impl esp_idf_svc::hal::gpio::OutputPin + 'static,
    rx_pin: impl InputPin + 'static,
) -> Result<(Arc<Link>, Box<dyn LinkRead>)> {
    use esp_idf_svc::hal::gpio::AnyIOPin;
    use esp_idf_svc::hal::uart::UartDriver;
    let drv = UartDriver::new(
        uart,
        tx_pin,
        rx_pin,
        None::<AnyIOPin>,
        None::<AnyIOPin>,
        &crate::link_uart_config(),
    )?;
    Ok(Link::uart("ctrl", drv, info.board))
}
