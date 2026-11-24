//! Main loop shared by TX and RX in M1: link to CTRL, heartbeat, trigger.
//! Role-specific tasks (Wi-Fi TX, sniffer, CSI) attach here from M2 on.

use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use anyhow::Result;
use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use lab_core::{peer::Peer, Action, Actions};
use lab_proto::{Msg, RunState, TriggerEvt};

use crate::board::BoardInfo;
use crate::link::Link;

pub fn dispatch(info: &BoardInfo, link: &Link, actions: Actions) {
    for a in actions {
        match a {
            Action::Send(_, m) => link.send(&m),
            Action::SendHello(_) => link.send(&Msg::Hello(info.hello())),
            Action::SendStatus(_) => {
                let dropped = link.rx_errors.load(Ordering::Relaxed)
                    + link.tx_errors.load(Ordering::Relaxed);
                link.send(&Msg::Status(info.status(RunState::Idle, 0, dropped)));
            }
            Action::Pulse { .. } => log::warn!("peer asked to pulse; ignored"),
        }
    }
}

pub fn run(
    info: BoardInfo,
    uart: impl Uart + 'static,
    tx_pin: impl OutputPin + 'static,
    rx_pin: impl InputPin + 'static,
    trigger_pin: impl InputPin + 'static,
) -> Result<()> {
    let drv = UartDriver::new(
        uart,
        tx_pin,
        rx_pin,
        None::<AnyIOPin>,
        None::<AnyIOPin>,
        &crate::link_uart_config(),
    )?;
    let (link, rx) = Link::new("ctrl", drv, info.board);
    let core = Arc::new(Mutex::new(Peer::new()));

    {
        let (link2, core) = (Arc::clone(&link), Arc::clone(&core));
        link.spawn_reader(rx, move |_h, msg, _body| match msg {
            Ok(m) => {
                let actions = core.lock().unwrap().on_msg(&m);
                dispatch(&info, &link2, actions);
            }
            Err(e) => log::warn!("ctrl link: undecodable message: {e}"),
        })?;
    }
    {
        let link = Arc::clone(&link);
        crate::trigger::spawn_trigger_in(trigger_pin, move |edge, t| {
            link.send(&Msg::TriggerEvt(TriggerEvt {
                exp_id: 0,
                t0_local_us: t,
                edge,
            }));
        })?;
    }

    let boot = core.lock().unwrap().on_boot();
    dispatch(&info, &link, boot);
    log::info!("{:?} ready", info.board);
    loop {
        let actions = core.lock().unwrap().tick(crate::now_ms());
        dispatch(&info, &link, actions);
        thread::sleep(Duration::from_millis(50));
    }
}
