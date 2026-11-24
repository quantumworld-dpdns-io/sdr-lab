//! Logic shared by the TX and RX boards (M1: liveness + commands).

use lab_proto::{Cmd, CmdBody, Msg, SyncMethod};

use crate::{ack, push, Action, Actions, Port, HEARTBEAT_MS};

#[derive(Debug, Default)]
pub struct Peer {
    last_heartbeat_ms: Option<u64>,
    /// Drives the trigger line itself (host-link mode, where no controller
    /// board exists). Then `SyncTest` is accepted and turned into pulses.
    trigger_master: bool,
}

impl Peer {
    pub const fn new() -> Self {
        Self {
            last_heartbeat_ms: None,
            trigger_master: false,
        }
    }

    pub const fn trigger_master() -> Self {
        Self {
            last_heartbeat_ms: None,
            trigger_master: true,
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
            CmdBody::SyncTest { pulses, period_ms } if self.trigger_master => {
                match crate::ctrl::validate_sync(*pulses, *period_ms) {
                    Ok(()) => {
                        push(&mut a, Action::Send(Port::Host, ack(*cmd_id, Ok(()))));
                        push(
                            &mut a,
                            Action::Pulse {
                                pulses: *pulses,
                                period_ms: *period_ms,
                            },
                        );
                    }
                    Err(e) => push(&mut a, Action::Send(Port::Host, ack(*cmd_id, Err(e)))),
                }
            }
            CmdBody::SyncTest { .. } => push(
                &mut a,
                Action::Send(
                    Port::Host,
                    ack(*cmd_id, Err("this board is not the trigger master")),
                ),
            ),
            CmdBody::SyncPrepare { method } => push(
                &mut a,
                Action::RadioPrepare {
                    cmd_id: *cmd_id,
                    method: *method,
                },
            ),
            // The wired method is the existing trigger test.
            CmdBody::SyncRun {
                method: SyncMethod::Wired,
                events,
                period_ms,
            } => {
                return self.on_msg(&Msg::Cmd(Cmd {
                    cmd_id: *cmd_id,
                    body: CmdBody::SyncTest {
                        pulses: *events,
                        period_ms: *period_ms,
                    },
                }))
            }
            CmdBody::SyncRun {
                method,
                events,
                period_ms,
            } => match crate::ctrl::validate_sync(*events, *period_ms) {
                Ok(()) => {
                    push(&mut a, Action::Send(Port::Host, ack(*cmd_id, Ok(()))));
                    push(
                        &mut a,
                        Action::RadioRun {
                            method: *method,
                            events: *events,
                            period_ms: *period_ms,
                        },
                    );
                }
                Err(e) => push(&mut a, Action::Send(Port::Host, ack(*cmd_id, Err(e)))),
            },
            CmdBody::Configure(_) | CmdBody::Arm | CmdBody::Stop => push(
                &mut a,
                Action::Send(Port::Host, ack(*cmd_id, Err("not supported by peer in M1"))),
            ),
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

    #[test]
    fn only_trigger_master_pulses() {
        let st = Msg::Cmd(Cmd {
            cmd_id: 3,
            body: CmdBody::SyncTest {
                pulses: 10,
                period_ms: 100,
            },
        });
        let a = Peer::new().on_msg(&st);
        assert!(matches!(&a[..], [Action::Send(_, Msg::Ack(x))] if !x.ok));
        let a = Peer::trigger_master().on_msg(&st);
        assert!(matches!(&a[0], Action::Send(_, Msg::Ack(x)) if x.ok));
        assert_eq!(
            a[1],
            Action::Pulse {
                pulses: 10,
                period_ms: 100
            }
        );
        let bad = Msg::Cmd(Cmd {
            cmd_id: 4,
            body: CmdBody::SyncTest {
                pulses: 0,
                period_ms: 100,
            },
        });
        assert!(
            matches!(&Peer::trigger_master().on_msg(&bad)[..], [Action::Send(_, Msg::Ack(x))] if !x.ok)
        );
    }

    #[test]
    fn radio_commands_map_to_actions() {
        let mut p = Peer::new();
        let prep = Msg::Cmd(Cmd {
            cmd_id: 5,
            body: CmdBody::SyncPrepare {
                method: SyncMethod::Ble,
            },
        });
        assert_eq!(
            p.on_msg(&prep).as_slice(),
            [Action::RadioPrepare {
                cmd_id: 5,
                method: SyncMethod::Ble
            }],
            "prepare is acked by the shell once the radio is ready"
        );
        let run = Msg::Cmd(Cmd {
            cmd_id: 6,
            body: CmdBody::SyncRun {
                method: SyncMethod::WifiTsf,
                events: 20,
                period_ms: 100,
            },
        });
        let a = p.on_msg(&run);
        assert!(matches!(&a[0], Action::Send(_, Msg::Ack(x)) if x.ok && x.cmd_id == 6));
        assert_eq!(
            a[1],
            Action::RadioRun {
                method: SyncMethod::WifiTsf,
                events: 20,
                period_ms: 100
            }
        );
        // Wired via SyncRun == SyncTest: only the trigger master accepts it.
        let wired = Msg::Cmd(Cmd {
            cmd_id: 7,
            body: CmdBody::SyncRun {
                method: SyncMethod::Wired,
                events: 10,
                period_ms: 100,
            },
        });
        assert!(matches!(&p.on_msg(&wired)[..], [Action::Send(_, Msg::Ack(x))] if !x.ok));
        assert!(matches!(
            &Peer::trigger_master().on_msg(&wired)[1],
            Action::Pulse { pulses: 10, .. }
        ));
    }
}
