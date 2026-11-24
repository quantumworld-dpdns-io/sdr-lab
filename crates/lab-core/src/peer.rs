//! Logic shared by the TX and RX boards (M1: liveness + commands).

use lab_proto::{Cmd, CmdBody, Msg};

use crate::{ack, push, Action, Actions, Port, HEARTBEAT_MS};

#[derive(Debug, Default)]
pub struct Peer {
    last_heartbeat_ms: Option<u64>,
}

impl Peer {
    pub const fn new() -> Self {
        Self {
            last_heartbeat_ms: None,
        }
    }

    /// Call once at boot.
    pub fn on_boot(&mut self) -> Actions {
        let mut a = Actions::new();
        push(&mut a, Action::SendHello(Port::Host));
        a
    }

    /// Call periodically (every ~10–100 ms).
    pub fn tick(&mut self, now_ms: u64) -> Actions {
        let mut a = Actions::new();
        let due = self
            .last_heartbeat_ms
            .is_none_or(|t| now_ms.saturating_sub(t) >= HEARTBEAT_MS);
        if due {
            self.last_heartbeat_ms = Some(now_ms);
            push(&mut a, Action::SendStatus(Port::Host));
        }
        a
    }

    /// Handle a message from the controller. Replies go "up" the single link,
    /// which a peer addresses as [`Port::Host`].
    pub fn on_msg(&mut self, msg: &Msg) -> Actions {
        let mut a = Actions::new();
        let Msg::Cmd(Cmd { cmd_id, body }) = msg else {
            return a;
        };
        match body {
            CmdBody::Ping => push(&mut a, Action::Send(Port::Host, ack(*cmd_id, Ok(())))),
            CmdBody::StatusReq => {
                push(&mut a, Action::SendHello(Port::Host));
                push(&mut a, Action::SendStatus(Port::Host));
            }
            CmdBody::Configure(_) | CmdBody::Arm | CmdBody::Stop | CmdBody::SyncTest { .. } => {
                push(
                    &mut a,
                    Action::Send(Port::Host, ack(*cmd_id, Err("not supported by peer in M1"))),
                )
            }
        }
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_once_per_second() {
        let mut p = Peer::new();
        let beats: usize = (0..=3000u64).step_by(10).map(|t| p.tick(t).len()).sum();
        assert_eq!(beats, 4); // t = 0, 1000, 2000, 3000
    }

    #[test]
    fn answers_ping_and_status() {
        let mut p = Peer::new();
        let ping = Msg::Cmd(Cmd {
            cmd_id: 9,
            body: CmdBody::Ping,
        });
        assert!(matches!(
            p.on_msg(&ping).as_slice(),
            [Action::Send(Port::Host, Msg::Ack(a))] if a.cmd_id == 9 && a.ok
        ));
        let sr = Msg::Cmd(Cmd {
            cmd_id: 1,
            body: CmdBody::StatusReq,
        });
        assert_eq!(
            p.on_msg(&sr).as_slice(),
            [
                Action::SendHello(Port::Host),
                Action::SendStatus(Port::Host)
            ]
        );
    }
}
