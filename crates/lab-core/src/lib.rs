//! Board logic for the SDR Lab, kept free of ESP-IDF so it runs in host tests.
//!
//! Each state machine consumes decoded messages and clock ticks and returns
//! [`Action`]s; the firmware shell (`firmware/lab-fw`) performs the I/O.
#![no_std]

pub mod ctrl;
pub mod peer;

use heapless::{String, Vec};
use lab_proto::{Ack, Msg, SyncMethod};

/// A UART link as seen from the controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    Host,
    Tx,
    Rx,
}

// Msg is held inline (no allocator). Actions carry only small control
// messages; bulk data (CSI, packets) bypasses this path.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    Send(Port, Msg),
    /// Send this board's own `Hello` (the shell knows MAC, versions, chip).
    SendHello(Port),
    /// Send this board's own `Status` (the shell knows heap, uptime).
    SendStatus(Port),
    /// Fire `pulses` trigger edges, `period_ms` apart (trigger master only).
    Pulse {
        pulses: u16,
        period_ms: u16,
    },
    /// Bring the radio into the mode `method` needs, then Ack `cmd_id`
    /// (the shell sends the Ack once the radio is actually ready).
    RadioPrepare {
        cmd_id: u32,
        method: SyncMethod,
    },
    /// Run a wireless sync measurement; the shell emits `SyncEvt`s.
    RadioRun {
        method: SyncMethod,
        events: u16,
        period_ms: u16,
    },
}

/// Worst case today: controller `StatusReq` reply = 4. Each slot is ~416 B
/// of stack, so keep this tight; firmware threads size their stacks for it.
pub const MAX_ACTIONS: usize = 6;
pub type Actions = Vec<Action, MAX_ACTIONS>;

/// How often boards emit an unsolicited `Status` heartbeat.
pub const HEARTBEAT_MS: u64 = 1000;

pub fn ack(cmd_id: u32, result: Result<(), &str>) -> Msg {
    let mut error = String::new();
    if let Err(e) = result {
        // Truncate to capacity rather than fail: an ack must always go out.
        for c in e.chars() {
            if error.push(c).is_err() {
                break;
            }
        }
    }
    Msg::Ack(Ack {
        cmd_id,
        ok: result.is_ok(),
        error,
    })
}

pub(crate) fn push(actions: &mut Actions, a: Action) {
    // Capacity is sized for the worst case of every handler; a failure here
    // is a programming error caught by tests.
    debug_assert!(actions.len() < MAX_ACTIONS, "action buffer full");
    let _ = actions.push(a);
}
