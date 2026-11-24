//! `labctl hub`: the Mac stands in for the controller board (host-link mode).
//!
//! The TX and RX ESP32-S3 boards connect over their native USB ports. This
//! module runs the same `lab_core::ctrl::Controller` state machine the
//! WROOM runs, routes its actions to the right serial port, and serves a
//! dashboard + JSON API on localhost.
//!
//! Threads: one reader per serial port → one hub thread (owns all state) →
//! snapshots/events → the HTTP server (tokio, axum).

use std::collections::BTreeMap;
use std::convert::Infallible;
use std::net::SocketAddr;
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use axum::extract::State;
use axum::http::StatusCode;
use axum::response::sse::{Event as SseEvent, KeepAlive, Sse};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use lab_core::ctrl::Controller;
use lab_core::{Action, Port};
use lab_proto::{Board, Cmd, CmdBody, Hello, Msg, Status};
use serde::{Deserialize, Serialize};
use tokio::sync::broadcast;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::StreamExt;

use crate::serial::{self, Conn, ConnWriter, Event};
use crate::sync::{self, PairReport};

const DASHBOARD: &str = include_str!("dashboard.html");
const TICK: Duration = Duration::from_millis(50);
/// Pass threshold for TX↔RX residual jitter (M1 acceptance).
pub const MAX_JITTER_US: f64 = 50.0;

// ---------- state shared with the HTTP side ----------

#[derive(Debug, Clone, Serialize)]
pub struct BoardView {
    pub board: Board,
    pub port: Option<String>,
    pub ready: bool,
    pub hello: Option<Hello>,
    pub status: Option<Status>,
    pub last_seen_ms_ago: Option<u64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SyncResult {
    pub ok: bool,
    pub pass: bool,
    pub message: String,
    pub pulses: u16,
    pub period_ms: u16,
    pub edges: BTreeMap<Board, usize>,
    pub pairs: Vec<PairReport>,
    pub max_jitter_us: f64,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Snapshot {
    pub boards: Vec<BoardView>,
    /// Ports that have not identified themselves yet.
    pub unknown_ports: Vec<String>,
    pub sync_running: bool,
    pub last_sync: Option<SyncResult>,
    pub warnings: Vec<String>,
}

// Same reasoning as serial::Event: host-side, size is irrelevant.
#[allow(clippy::large_enum_variant)]
enum In {
    Ev(usize, Event),
    Closed(usize),
    Sync {
        pulses: u16,
        period_ms: u16,
        reply: mpsc::Sender<SyncResult>,
    },
}

// ---------- hub thread ----------

struct ConnState {
    path: String,
    writer: ConnWriter,
    port: Option<Port>,
    open: bool,
}

struct SyncRun {
    pulses: u16,
    period_ms: u16,
    cmd_id: u32,
    deadline: Instant,
    edges: BTreeMap<Board, Vec<u64>>,
    reply: mpsc::Sender<SyncResult>,
}

struct Hub {
    ctrl: Controller,
    conns: Vec<ConnState>,
    sync: Option<SyncRun>,
    next_cmd_id: u32,
    t0: Instant,
    snap: Arc<RwLock<Snapshot>>,
    events: broadcast::Sender<String>,
}

fn port_of(board: Board) -> Option<Port> {
    match board {
        Board::Tx => Some(Port::Tx),
        Board::Rx => Some(Port::Rx),
        _ => None,
    }
}

impl Hub {
    fn now_ms(&self) -> u64 {
        self.t0.elapsed().as_millis() as u64
    }

    fn emit(&self, v: serde_json::Value) {
        let _ = self.events.send(v.to_string());
    }

    fn cmd_id(&mut self) -> u32 {
        self.next_cmd_id = self.next_cmd_id.wrapping_add(1);
        // High bit marks hub-originated ids, apart from the controller's own.
        self.next_cmd_id | 0x8000_0000
    }

    /// Send to the conn assigned to `port`; while none is, try every
    /// unidentified conn (that is how discovery reaches new boards).
    fn send_to(&mut self, port: Port, msg: &Msg) {
        let assigned = self.conns.iter().any(|c| c.open && c.port == Some(port));
        for c in self.conns.iter_mut().filter(|c| c.open) {
            if c.port == Some(port) || (!assigned && c.port.is_none()) {
                if let Err(e) = c.writer.send(msg) {
                    eprintln!("hub: write to {} failed: {e:#}", c.path);
                }
            }
        }
    }

    fn run_actions(&mut self, actions: lab_core::Actions) {
        for a in actions {
            match a {
                Action::Send(Port::Host, _) | Action::SendHello(_) | Action::SendStatus(_) => {
                    // Addressed to the host — that is us. Nothing to forward.
                }
                Action::Send(p, m) => self.send_to(p, &m),
                Action::Pulse { pulses, period_ms } => {
                    // No controller board: the TX peer is the trigger master.
                    let cmd_id = self.cmd_id();
                    if let Some(s) = &mut self.sync {
                        s.cmd_id = cmd_id;
                    }
                    self.send_to(
                        Port::Tx,
                        &Msg::Cmd(Cmd {
                            cmd_id,
                            body: CmdBody::SyncTest { pulses, period_ms },
                        }),
                    );
                }
            }
        }
    }

    fn on_event(&mut self, i: usize, ev: Event) {
        let now = self.now_ms();
        let path = self.conns[i].path.clone();
        match ev {
            Event::Text(t) => {
                if !t.is_empty() {
                    self.emit(serde_json::json!({"t": now, "port": path, "text": t}));
                }
            }
            Event::Frame { hdr, msg } => {
                let src = Board::from_src(hdr.src).unwrap_or(Board::Unknown);
                let m = match msg {
                    Ok(m) => m,
                    Err(e) => {
                        self.emit(serde_json::json!({"t": now, "port": path, "src": src, "seq": hdr.seq, "error": e.to_string()}));
                        return;
                    }
                };
                if let Msg::Hello(h) = &m {
                    if let Some(p) = port_of(h.board) {
                        if self.conns[i].port != Some(p) {
                            self.conns[i].port = Some(p);
                            eprintln!("hub: {} is {}", path, h.board.name());
                        }
                    }
                }
                if let Some(p) = self.conns[i].port {
                    self.ctrl.on_peer(p, hdr.src, &m, now);
                }
                self.on_sync_msg(src, &m);
                self.emit(serde_json::json!({"t": now, "port": path, "src": src, "seq": hdr.seq, "msg": m}));
            }
        }
    }

    fn on_sync_msg(&mut self, src: Board, m: &Msg) {
        let Some(run) = &mut self.sync else { return };
        match m {
            Msg::TriggerEvt(t) if matches!(src, Board::Tx | Board::Rx) => {
                run.edges.entry(src).or_default().push(t.t0_local_us);
            }
            Msg::Ack(a) if src == Board::Tx && a.cmd_id == run.cmd_id && !a.ok => {
                let msg = format!("TX rejected the sync test: {}", a.error);
                self.finish_sync(false, msg);
            }
            _ => {}
        }
    }

    fn start_sync(&mut self, pulses: u16, period_ms: u16, reply: mpsc::Sender<SyncResult>) {
        let fail = |msg: &str| SyncResult {
            ok: false,
            pass: false,
            message: msg.to_string(),
            pulses,
            period_ms,
            edges: BTreeMap::new(),
            pairs: vec![],
            max_jitter_us: MAX_JITTER_US,
        };
        if self.sync.is_some() {
            let _ = reply.send(fail("a sync test is already running"));
            return;
        }
        let now = self.now_ms();
        if !self.ctrl.tx.ready(now) {
            let _ = reply.send(fail("TX is not connected/ready"));
            return;
        }
        let actions = self.ctrl.on_host(&Msg::Cmd(Cmd {
            cmd_id: 0,
            body: CmdBody::SyncTest { pulses, period_ms },
        }));
        if let Some(Action::Send(_, Msg::Ack(a))) = actions.first() {
            if !a.ok {
                let _ = reply.send(fail(&a.error));
                return;
            }
        }
        let run_ms = pulses as u64 * period_ms as u64 + 1500;
        self.sync = Some(SyncRun {
            pulses,
            period_ms,
            cmd_id: 0,
            deadline: Instant::now() + Duration::from_millis(run_ms),
            edges: BTreeMap::new(),
            reply,
        });
        self.run_actions(actions);
        self.emit(serde_json::json!({"t": now, "sync_started": {"pulses": pulses, "period_ms": period_ms}}));
    }

    fn poll_sync(&mut self) {
        let Some(run) = &self.sync else { return };
        let want = run.pulses as usize;
        let complete = [Board::Tx, Board::Rx]
            .iter()
            .all(|b| run.edges.get(b).is_some_and(|v| v.len() >= want));
        if complete || Instant::now() >= run.deadline {
            self.finish_sync(true, String::new());
        }
    }

    fn finish_sync(&mut self, ok: bool, message: String) {
        let Some(run) = self.sync.take() else { return };
        let want = run.pulses as usize;
        let mut edges = run.edges;
        for v in edges.values_mut() {
            v.truncate(want);
        }
        let report = sync::analyze(&edges, want);
        let txrx = report
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == (Board::Tx, Board::Rx));
        let (pass, message) = match (ok, txrx) {
            (false, _) => (false, message),
            (true, Some(p)) if p.jitter_us <= MAX_JITTER_US => (
                true,
                format!(
                    "PASS: TX↔RX residual jitter {:.1} µs ≤ {MAX_JITTER_US} µs",
                    p.jitter_us
                ),
            ),
            (true, Some(p)) => (
                false,
                format!(
                    "FAIL: TX↔RX residual jitter {:.1} µs > {MAX_JITTER_US} µs",
                    p.jitter_us
                ),
            ),
            (true, None) => {
                let counts: Vec<String> = [Board::Tx, Board::Rx]
                    .iter()
                    .map(|b| {
                        format!(
                            "{} {}/{want}",
                            b.name(),
                            edges.get(b).map_or(0, |v| v.len())
                        )
                    })
                    .collect();
                (
                    false,
                    format!(
                        "FAIL: missing edges ({}). Check jumpers TX GPIO5 → TX GPIO4 and RX GPIO4, and shared GND.",
                        counts.join(", ")
                    ),
                )
            }
        };
        let result = SyncResult {
            ok,
            pass,
            message,
            pulses: run.pulses,
            period_ms: run.period_ms,
            edges: edges.iter().map(|(b, v)| (*b, v.len())).collect(),
            pairs: report.pairs,
            max_jitter_us: MAX_JITTER_US,
        };
        eprintln!("hub: sync {}", result.message);
        self.snap.write().unwrap().last_sync = Some(result.clone());
        self.emit(serde_json::json!({"t": self.now_ms(), "sync": result}));
        let _ = run.reply.send(result);
    }

    fn publish(&self) {
        let now = self.now_ms();
        let mut boards = Vec::new();
        for b in [Board::Tx, Board::Rx] {
            let port = port_of(b).expect("peer board");
            let st = self.ctrl.peer(port).expect("peer port");
            boards.push(BoardView {
                board: b,
                port: self
                    .conns
                    .iter()
                    .find(|c| c.open && c.port == Some(port))
                    .map(|c| c.path.clone()),
                ready: st.ready(now),
                hello: st.hello.clone(),
                status: st.status,
                last_seen_ms_ago: st.last_seen_ms.map(|t| now.saturating_sub(t)),
            });
        }
        let mut warnings = Vec::new();
        for p in [Port::Tx, Port::Rx] {
            let n = self
                .conns
                .iter()
                .filter(|c| c.open && c.port == Some(p))
                .count();
            if n > 1 {
                warnings.push(format!(
                    "{n} ports claim to be {p:?} — flash one board as the other role"
                ));
            }
        }
        for c in self.conns.iter().filter(|c| !c.open) {
            warnings.push(format!(
                "{} disconnected (restart the hub after replugging)",
                c.path
            ));
        }
        let mut s = self.snap.write().unwrap();
        s.boards = boards;
        s.unknown_ports = self
            .conns
            .iter()
            .filter(|c| c.open && c.port.is_none())
            .map(|c| c.path.clone())
            .collect();
        s.sync_running = self.sync.is_some();
        s.warnings = warnings;
    }

    fn run(mut self, rx: mpsc::Receiver<In>) {
        let mut last_tick = Instant::now() - TICK;
        loop {
            match rx.recv_timeout(TICK) {
                Ok(In::Ev(i, ev)) => self.on_event(i, ev),
                Ok(In::Closed(i)) => {
                    self.conns[i].open = false;
                    eprintln!("hub: {} closed", self.conns[i].path);
                }
                Ok(In::Sync {
                    pulses,
                    period_ms,
                    reply,
                }) => self.start_sync(pulses, period_ms, reply),
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => return,
            }
            if last_tick.elapsed() >= TICK {
                last_tick = Instant::now();
                let a = self.ctrl.tick(self.now_ms());
                self.run_actions(a);
                self.poll_sync();
                self.publish();
            }
        }
    }
}

// ---------- HTTP ----------

#[derive(Clone)]
struct AppState {
    snap: Arc<RwLock<Snapshot>>,
    events: broadcast::Sender<String>,
    hub: mpsc::Sender<In>,
}

#[derive(Deserialize)]
struct SyncReq {
    #[serde(default = "default_pulses")]
    pulses: u16,
    #[serde(default = "default_period")]
    period_ms: u16,
}
fn default_pulses() -> u16 {
    20
}
fn default_period() -> u16 {
    100
}

async fn api_status(State(st): State<AppState>) -> Json<Snapshot> {
    Json(st.snap.read().unwrap().clone())
}

async fn api_events(
    State(st): State<AppState>,
) -> Sse<impl tokio_stream::Stream<Item = Result<SseEvent, Infallible>>> {
    let stream = BroadcastStream::new(st.events.subscribe())
        .filter_map(|r| r.ok())
        .map(|s| Ok(SseEvent::default().data(s)));
    Sse::new(stream).keep_alive(KeepAlive::default())
}

async fn api_sync(State(st): State<AppState>, Json(req): Json<SyncReq>) -> Response {
    let (tx, rx) = mpsc::channel();
    let wait = Duration::from_millis(req.pulses as u64 * req.period_ms as u64 + 5000);
    if st
        .hub
        .send(In::Sync {
            pulses: req.pulses,
            period_ms: req.period_ms,
            reply: tx,
        })
        .is_err()
    {
        return (StatusCode::SERVICE_UNAVAILABLE, "hub stopped").into_response();
    }
    match tokio::task::spawn_blocking(move || rx.recv_timeout(wait)).await {
        Ok(Ok(r)) => {
            let code = if r.ok {
                StatusCode::OK
            } else {
                StatusCode::CONFLICT
            };
            (code, Json(r)).into_response()
        }
        _ => (StatusCode::GATEWAY_TIMEOUT, "sync test did not finish").into_response(),
    }
}

pub fn run(ports: Vec<String>, listen: SocketAddr) -> Result<()> {
    let ports = if ports.is_empty() {
        serial::lab_ports()?
    } else {
        ports
    };
    if ports.is_empty() {
        bail!("no ESP32 USB ports found (Espressif native USB or CH34x/CP210x/FTDI bridge). Plug in TX/RX or pass --port.");
    }

    let (hub_tx, hub_rx) = mpsc::channel::<In>();
    let mut conns = Vec::new();
    for (i, path) in ports.into_iter().enumerate() {
        let conn = Conn::open_path(path.clone()).with_context(|| format!("opening {path}"))?;
        let (path, writer, events) = conn.split();
        let tx = hub_tx.clone();
        thread::spawn(move || {
            for ev in events {
                if tx.send(In::Ev(i, ev)).is_err() {
                    return;
                }
            }
            let _ = tx.send(In::Closed(i));
        });
        eprintln!("hub: opened {path}");
        conns.push(ConnState {
            path,
            writer,
            port: None,
            open: true,
        });
    }

    let snap = Arc::new(RwLock::new(Snapshot::default()));
    let (events, _) = broadcast::channel::<String>(1024);
    let hub = Hub {
        ctrl: Controller::new(),
        conns,
        sync: None,
        next_cmd_id: 0,
        t0: Instant::now(),
        snap: Arc::clone(&snap),
        events: events.clone(),
    };
    thread::Builder::new()
        .name("hub".into())
        .spawn(move || hub.run(hub_rx))?;

    let state = AppState {
        snap,
        events,
        hub: hub_tx,
    };
    let app = Router::new()
        .route("/", get(|| async { Html(DASHBOARD) }))
        .route("/api/status", get(api_status))
        .route("/api/events", get(api_events))
        .route("/api/sync", post(api_sync))
        .with_state(state);

    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    rt.block_on(async move {
        let listener = tokio::net::TcpListener::bind(listen)
            .await
            .with_context(|| format!("binding {listen}"))?;
        eprintln!("hub: dashboard at http://{listen}/");
        axum::serve(listener, app).await?;
        Ok(())
    })
}
