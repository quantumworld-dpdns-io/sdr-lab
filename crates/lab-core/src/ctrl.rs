//! Controller (ESP32-WROOM-32) logic. M1: peer liveness, host commands,
//! trigger sync tests. Frames from TX/RX are also forwarded verbatim to the
//! host by the shell; this module only tracks what it needs.

use lab_proto::{limits, Board, Cmd, CmdBody, ExperimentConfig, Hello, Msg, Status};

use crate::{ack, push, Action, Actions, Port, HEARTBEAT_MS};

/// A peer is "ready" if it said Hello and was heard from within this window.
pub const PEER_TIMEOUT_MS: u64 = 3 * HEARTBEAT_MS;
/// While a peer is not ready, re-request its Hello this often.
pub const DISCOVER_MS: u64 = 2000;
pub const SYNC_MAX_PULSES: u16 = 1000;
pub const SYNC_MIN_PERIOD_MS: u16 = 10;

#[derive(Debug, Clone, Default)]
pub struct PeerState {
    pub hello: Option<Hello>,
    pub status: Option<Status>,
    pub last_seen_ms: Option<u64>,
    last_discover_ms: Option<u64>,
}

impl PeerState {
    pub fn ready(&self, now_ms: u64) -> bool {
        self.hello.is_some()
            && self
                .last_seen_ms
                .is_some_and(|t| now_ms.saturating_sub(t) <= PEER_TIMEOUT_MS)
    }
}

#[derive(Debug, Default)]
pub struct Controller {
    pub tx: PeerState,
    pub rx: PeerState,
    /// Last validated experiment (run in M2).
    pub pending: Option<ExperimentConfig>,
    last_heartbeat_ms: Option<u64>,
    next_cmd_id: u32,
}

impl Controller {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn peer(&self, port: Port) -> Option<&PeerState> {
        match port {
            Port::Tx => Some(&self.tx),
            Port::Rx => Some(&self.rx),
            Port::Host => None,
        }
    }

    fn peer_mut(&mut self, port: Port) -> Option<&mut PeerState> {
        match port {
            Port::Tx => Some(&mut self.tx),
            Port::Rx => Some(&mut self.rx),
            Port::Host => None,
        }
    }

    pub fn on_boot(&mut self) -> Actions {
        let mut a = Actions::new();
        push(&mut a, Action::SendHello(Port::Host));
        a
    }

    pub fn tick(&mut self, now_ms: u64) -> Actions {
        let mut a = Actions::new();
        if self
            .last_heartbeat_ms
            .is_none_or(|t| now_ms.saturating_sub(t) >= HEARTBEAT_MS)
        {
            self.last_heartbeat_ms = Some(now_ms);
            push(&mut a, Action::SendStatus(Port::Host));
        }
        for port in [Port::Tx, Port::Rx] {
            let cmd_id = self.next_cmd_id;
            let p = self.peer_mut(port).expect("peer port");
            let due = p
                .last_discover_ms
                .is_none_or(|t| now_ms.saturating_sub(t) >= DISCOVER_MS);
            if !p.ready(now_ms) && due {
                p.last_discover_ms = Some(now_ms);
                self.next_cmd_id = self.next_cmd_id.wrapping_add(1);
                push(
                    &mut a,
                    Action::Send(
                        port,
                        Msg::Cmd(Cmd {
                            cmd_id,
                            body: CmdBody::StatusReq,
                        }),
                    ),
                );
            }
        }
        a
    }

    /// A decoded frame arrived from TX or RX.
    pub fn on_peer(&mut self, port: Port, src: u8, msg: &Msg, now_ms: u64) {
        let expected = match port {
            Port::Tx => Board::Tx,
            Port::Rx => Board::Rx,
            Port::Host => return,
        };
        if src != expected as u8 {
            // Miswired (TX/RX swapped) — ignore rather than mix up state.
            return;
        }
        let Some(p) = self.peer_mut(port) else { return };
        p.last_seen_ms = Some(now_ms);
        match msg {
            Msg::Hello(h) => p.hello = Some(h.clone()),
            Msg::Status(s) => p.status = Some(*s),
            _ => {}
        }
    }

    /// A decoded frame arrived from the host.
    pub fn on_host(&mut self, msg: &Msg) -> Actions {
        let mut a = Actions::new();
        let Msg::Cmd(Cmd { cmd_id, body }) = msg else {
            return a;
        };
        let id = *cmd_id;
        let reply = |a: &mut Actions, r: Result<(), &str>| {
            push(a, Action::Send(Port::Host, ack(id, r)));
        };
        match *body {
            CmdBody::Ping => reply(&mut a, Ok(())),
            CmdBody::StatusReq => {
                push(&mut a, Action::SendHello(Port::Host));
                push(&mut a, Action::SendStatus(Port::Host));
                // Replay cached peer identity so the host sees all boards
                // even if it connected after they booted. Hello names its
                // board in-band, so it is safe to send with src = CTRL.
                // Peer Status is NOT replayed: the host attributes Status by
                // header src, and peers' own heartbeats are forwarded
                // verbatim every second anyway.
                for p in [&self.tx, &self.rx] {
                    if let Some(h) = &p.hello {
                        push(&mut a, Action::Send(Port::Host, Msg::Hello(h.clone())));
                    }
                }
            }
            CmdBody::Configure(cfg) => match limits::validate(&cfg) {
                Ok(()) => {
                    self.pending = Some(cfg);
                    reply(&mut a, Ok(()));
                }
                Err(_) => reply(&mut a, Err("config rejected by safety limits")),
            },
            CmdBody::SyncTest { pulses, period_ms } => {
                if pulses == 0 || pulses > SYNC_MAX_PULSES {
                    reply(&mut a, Err("pulses must be 1..=1000"));
                } else if period_ms < SYNC_MIN_PERIOD_MS {
                    reply(&mut a, Err("period_ms must be >= 10"));
                } else {
                    reply(&mut a, Ok(()));
                    push(&mut a, Action::Pulse { pulses, period_ms });
                }
            }
            CmdBody::Arm | CmdBody::Stop => reply(&mut a, Err("experiment runs land in M2")),
        }
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lab_proto::{Mode, RunState};

    fn hello(board: Board) -> Msg {
        Msg::Hello(Hello {
            board,
            mac: [board as u8; 6],
            fw_version: "0.1.0".try_into().unwrap(),
            git_hash: "dev".try_into().unwrap(),
            chip: "esp32s3".try_into().unwrap(),
        })
    }

    fn status() -> Msg {
        Msg::Status(Status {
            state: RunState::Idle,
            uptime_ms: 1,
            free_heap: 2,
            dropped: 0,
            records: 0,
        })
    }

    fn cmd(cmd_id: u32, body: CmdBody) -> Msg {
        Msg::Cmd(Cmd { cmd_id, body })
    }

    fn discovers(a: &Actions, port: Port) -> bool {
        a.iter().any(|x| {
            matches!(x, Action::Send(p, Msg::Cmd(Cmd { body: CmdBody::StatusReq, .. })) if *p == port)
        })
    }

    #[test]
    fn discovery_until_ready_then_timeout() {
        let mut c = Controller::new();
        let a = c.tick(0);
        assert!(discovers(&a, Port::Tx) && discovers(&a, Port::Rx));
        assert!(!discovers(&c.tick(500), Port::Tx), "rate-limited");

        c.on_peer(Port::Tx, Board::Tx as u8, &hello(Board::Tx), 600);
        assert!(c.tx.ready(600) && !c.rx.ready(600));
        let a = c.tick(2100);
        assert!(!discovers(&a, Port::Tx) && discovers(&a, Port::Rx));

        // Heartbeats keep it alive; silence times it out.
        c.on_peer(Port::Tx, Board::Tx as u8, &status(), 3000);
        assert!(c.tx.ready(6000));
        assert!(!c.tx.ready(6001));
        assert!(discovers(&c.tick(6100), Port::Tx));
    }

    #[test]
    fn miswired_source_ignored() {
        let mut c = Controller::new();
        c.on_peer(Port::Tx, Board::Rx as u8, &hello(Board::Rx), 0);
        assert!(c.tx.hello.is_none());
    }

    #[test]
    fn status_req_replays_peers() {
        let mut c = Controller::new();
        c.on_peer(Port::Tx, Board::Tx as u8, &hello(Board::Tx), 0);
        c.on_peer(Port::Rx, Board::Rx as u8, &hello(Board::Rx), 0);
        c.on_peer(Port::Rx, Board::Rx as u8, &status(), 0);
        let a = c.on_host(&cmd(1, CmdBody::StatusReq));
        assert_eq!(a[0], Action::SendHello(Port::Host));
        assert_eq!(a[1], Action::SendStatus(Port::Host));
        assert_eq!(a[2], Action::Send(Port::Host, hello(Board::Tx)));
        assert_eq!(a[3], Action::Send(Port::Host, hello(Board::Rx)));
        assert_eq!(
            a.len(),
            4,
            "peer Status must not be replayed under src=CTRL"
        );
    }

    #[test]
    fn sync_test_validated() {
        let mut c = Controller::new();
        let a = c.on_host(&cmd(
            2,
            CmdBody::SyncTest {
                pulses: 20,
                period_ms: 100,
            },
        ));
        assert!(matches!(&a[0], Action::Send(Port::Host, Msg::Ack(x)) if x.ok && x.cmd_id == 2));
        assert_eq!(
            a[1],
            Action::Pulse {
                pulses: 20,
                period_ms: 100
            }
        );
        let a = c.on_host(&cmd(
            3,
            CmdBody::SyncTest {
                pulses: 5,
                period_ms: 1,
            },
        ));
        assert!(matches!(&a[..], [Action::Send(Port::Host, Msg::Ack(x))] if !x.ok));
    }

    #[test]
    fn configure_goes_through_safety_limits() {
        let mut c = Controller::new();
        let mut cfg = ExperimentConfig {
            exp_id: 1,
            mode: Mode::Rssi,
            channel: 6,
            interval_ms: 100,
            duration_s: 60,
            tx_power_qdbm: 34,
            payload_len: 64,
            replay_protect: false,
        };
        let a = c.on_host(&cmd(4, CmdBody::Configure(cfg)));
        assert!(matches!(&a[0], Action::Send(_, Msg::Ack(x)) if x.ok));
        assert_eq!(c.pending, Some(cfg));
        cfg.channel = 14;
        let a = c.on_host(&cmd(5, CmdBody::Configure(cfg)));
        assert!(matches!(&a[0], Action::Send(_, Msg::Ack(x)) if !x.ok));
    }
}
