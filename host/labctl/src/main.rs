//! `labctl`: SDR Lab host CLI.
//!
//! Offline: `validate`, `encode`. Over the controller's USB serial link:
//! `ports`, `status`, `monitor`, `sync`.

mod experiment;
mod hub;
mod serial;
mod sync;

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use clap::{Parser, Subcommand};
use lab_proto::{frame::MAX_WIRE, limits, Board, Cmd, CmdBody, Hello, Msg, Status};

use serial::{Conn, Event};

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    cmd: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Check experiment files against the lab's safety limits.
    Validate { files: Vec<PathBuf> },
    /// Print the Configure command frame for an experiment as hex.
    Encode { file: PathBuf },
    /// List serial ports and which one would be used as the controller.
    Ports,
    /// Show TX / RX / CTRL liveness. Exit code 1 unless all three are ready.
    Status {
        #[arg(long)]
        port: Option<String>,
        /// Seconds to listen for heartbeats.
        #[arg(long, default_value_t = 2.5)]
        wait: f64,
    },
    /// Print every decoded frame (and stray console text) until Ctrl-C.
    Monitor {
        #[arg(long)]
        port: Option<String>,
    },
    /// Mac-as-controller mode: talk to TX/RX S3 boards over their native USB
    /// (firmware built with `--features host-link`) and serve a dashboard.
    Hub {
        /// Serial ports to use (default: every Espressif native-USB port).
        #[arg(long = "port")]
        ports: Vec<String>,
        /// Dashboard address. Loopback by default; the lab is not a server.
        #[arg(long, default_value = "127.0.0.1:8080")]
        listen: std::net::SocketAddr,
    },
    /// Fire trigger pulses and report cross-board timing agreement.
    Sync {
        #[arg(long)]
        port: Option<String>,
        #[arg(long, default_value_t = 20)]
        pulses: u16,
        #[arg(long, default_value_t = 100)]
        period_ms: u16,
        /// Pass threshold for TX↔RX residual jitter.
        #[arg(long, default_value_t = 50.0)]
        max_jitter_us: f64,
    },
}

fn board_name(src: u8) -> &'static str {
    Board::from_src(src).map_or("?", Board::name)
}

/// Send a command to the controller and wait for its Ack.
fn command(conn: &mut Conn, cmd_id: u32, body: CmdBody) -> Result<()> {
    conn.send(&Msg::Cmd(Cmd { cmd_id, body }))?;
    let deadline = Instant::now() + Duration::from_secs(2);
    while let Some(ev) = conn.recv_until(deadline) {
        if let Event::Frame {
            hdr,
            msg: Ok(Msg::Ack(a)),
        } = ev
        {
            if hdr.src == Board::Ctrl as u8 && a.cmd_id == cmd_id {
                if a.ok {
                    return Ok(());
                }
                bail!("controller rejected command: {}", a.error);
            }
        }
    }
    bail!(
        "no Ack from controller on {} within 2 s (is it flashed and at {} baud?)",
        conn.path,
        serial::BAUD
    )
}

fn cmd_status(port: Option<&str>, wait: f64) -> Result<bool> {
    let mut conn = Conn::open(port)?;
    command(&mut conn, 1, CmdBody::StatusReq)?;
    let mut hellos: BTreeMap<u8, Hello> = BTreeMap::new();
    let mut status: BTreeMap<u8, (Status, Instant)> = BTreeMap::new();
    conn.collect_for(Duration::from_secs_f64(wait), |ev| {
        if let Event::Frame { hdr, msg: Ok(m) } = ev {
            match m {
                // Hello names its board in-band (CTRL replays peer Hellos).
                Msg::Hello(h) => {
                    hellos.insert(h.board as u8, h);
                }
                // Status is attributed by header src (peers' own heartbeats).
                Msg::Status(s) => {
                    status.insert(hdr.src, (s, Instant::now()));
                }
                _ => {}
            }
        }
        true
    });

    println!("port {}\n", conn.path);
    println!(
        "{:<5} {:<8} {:<8} {:<17} {:<7} {:>8} {:>7} {:>5}",
        "BOARD", "STATE", "CHIP", "MAC", "FW", "UPTIME", "HEAP", "ERRS"
    );
    let mut all_ready = true;
    for b in [Board::Tx, Board::Rx, Board::Ctrl] {
        let id = b as u8;
        let h = hellos.get(&id);
        let s = status.get(&id).map(|(s, _)| s);
        let ready = h.is_some() && s.is_some();
        all_ready &= ready;
        let state = match (h, s) {
            (Some(_), Some(_)) => "ready",
            (Some(_), None) => "stale",
            (None, Some(_)) => "no-hello",
            (None, None) => "missing",
        };
        let mac = h
            .map(|h| {
                h.mac
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<Vec<_>>()
                    .join(":")
            })
            .unwrap_or_else(|| "-".into());
        println!(
            "{:<5} {:<8} {:<8} {:<17} {:<7} {:>8} {:>7} {:>5}",
            board_name(id),
            state,
            h.map(|h| h.chip.as_str()).unwrap_or("-"),
            mac,
            h.map(|h| h.fw_version.as_str()).unwrap_or("-"),
            s.map(|s| format!("{:.1}s", s.uptime_ms as f64 / 1000.0))
                .unwrap_or_else(|| "-".into()),
            s.map(|s| format!("{}k", s.free_heap / 1024))
                .unwrap_or_else(|| "-".into()),
            s.map(|s| s.dropped.to_string())
                .unwrap_or_else(|| "-".into()),
        );
    }
    Ok(all_ready)
}

fn cmd_monitor(port: Option<&str>) -> Result<()> {
    let mut conn = Conn::open(port)?;
    eprintln!("monitoring {} (Ctrl-C to stop)", conn.path);
    let _ = command(&mut conn, 1, CmdBody::StatusReq);
    let t0 = Instant::now();
    loop {
        conn.collect_for(Duration::from_secs(3600), |ev| {
            let t = t0.elapsed().as_secs_f64();
            match ev {
                Event::Frame { hdr, msg: Ok(m) } => {
                    println!("{t:9.3} {:<4} #{:<5} {m:?}", board_name(hdr.src), hdr.seq)
                }
                Event::Frame { hdr, msg: Err(e) } => println!(
                    "{t:9.3} {:<4} #{:<5} <undecodable type {}: {e}>",
                    board_name(hdr.src),
                    hdr.seq,
                    hdr.msg_type
                ),
                Event::Text(s) if !s.is_empty() => println!("{t:9.3} >    {s}"),
                Event::Text(_) => {}
            }
            true
        });
    }
}

fn cmd_sync(port: Option<&str>, pulses: u16, period_ms: u16, max_jitter_us: f64) -> Result<bool> {
    let mut conn = Conn::open(port)?;
    // Drain heartbeats etc. queued before the test starts.
    conn.collect_for(Duration::from_millis(200), |_| true);
    command(&mut conn, 2, CmdBody::SyncTest { pulses, period_ms })?;
    let mut edges: BTreeMap<Board, Vec<u64>> = BTreeMap::new();
    let total =
        Duration::from_millis(pulses as u64 * period_ms as u64) + Duration::from_millis(1500);
    let want = pulses as usize;
    conn.collect_for(total, |ev| {
        if let Event::Frame {
            hdr,
            msg: Ok(Msg::TriggerEvt(t)),
        } = ev
        {
            if let Some(b) = Board::from_src(hdr.src) {
                edges.entry(b).or_default().push(t.t0_local_us);
            }
        }
        // Stop early once every board has reported every edge.
        !(edges.len() == 3 && edges.values().all(|v| v.len() >= want))
    });

    let report = sync::analyze(&edges, want);
    println!(
        "sync test: {pulses} pulses × {period_ms} ms on {}\n",
        conn.path
    );
    for b in [Board::Tx, Board::Rx, Board::Ctrl] {
        println!(
            "  {:<4} edges: {}",
            board_name(b as u8),
            edges.get(&b).map_or(0, |v| v.len())
        );
    }
    println!(
        "\n  {:<10} {:>10} {:>12} {:>12}",
        "PAIR", "DRIFT ppm", "JITTER µs", "RAW MAX µs"
    );
    for p in &report.pairs {
        println!(
            "  {:<10} {:>10.2} {:>12.1} {:>12.1}",
            format!("{}↔{}", board_name(p.a as u8), board_name(p.b as u8)),
            p.drift_ppm,
            p.jitter_us,
            p.raw_max_us
        );
    }
    let txrx = report
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (Board::Tx, Board::Rx));
    match txrx {
        Some(p) if p.jitter_us <= max_jitter_us => {
            println!(
                "\nPASS: TX↔RX residual jitter {:.1} µs ≤ {max_jitter_us} µs",
                p.jitter_us
            );
            Ok(true)
        }
        Some(p) => {
            println!(
                "\nFAIL: TX↔RX residual jitter {:.1} µs > {max_jitter_us} µs",
                p.jitter_us
            );
            Ok(false)
        }
        None => {
            for (b, n) in &report.missing {
                println!(
                    "\nFAIL: {} reported {n}/{want} edges (check trigger wire to GPIO4)",
                    board_name(*b as u8)
                );
            }
            if report.missing.is_empty() {
                println!("\nFAIL: no TX↔RX data");
            }
            Ok(false)
        }
    }
}

fn run() -> Result<bool> {
    match Cli::parse().cmd {
        Command::Validate { files } => {
            if files.is_empty() {
                bail!("no experiment files given");
            }
            let mut failed = 0;
            for f in &files {
                match experiment::load(f)
                    .and_then(|e| limits::validate(&e.config()).map(|_| e).map_err(Into::into))
                {
                    Ok(e) => println!(
                        "ok    {} ({}, {:?}, ch {})",
                        f.display(),
                        e.name,
                        e.mode,
                        e.channel
                    ),
                    Err(err) => {
                        failed += 1;
                        println!("FAIL  {}: {err:#}", f.display());
                    }
                }
            }
            Ok(failed == 0)
        }
        Command::Encode { file } => {
            let cfg = experiment::load(&file)?.config();
            limits::validate(&cfg)?;
            let msg = Msg::Cmd(Cmd {
                cmd_id: 1,
                body: CmdBody::Configure(cfg),
            });
            let mut out = [0u8; MAX_WIRE];
            let n = msg.to_wire(Board::Host, 0, &mut out)?;
            println!(
                "{}",
                out[..n]
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<String>()
            );
            Ok(true)
        }
        Command::Ports => {
            for p in serialport::available_ports()? {
                println!("{:<40} {:?}", p.port_name, p.port_type);
            }
            match serial::find_port(None) {
                Ok(p) => println!("\ncontroller: {p}"),
                Err(e) => println!("\n{e:#}"),
            }
            Ok(true)
        }
        Command::Status { port, wait } => cmd_status(port.as_deref(), wait),
        Command::Monitor { port } => cmd_monitor(port.as_deref()).map(|_| true),
        Command::Hub { ports, listen } => hub::run(ports, listen).map(|_| true),
        Command::Sync {
            port,
            pulses,
            period_ms,
            max_jitter_us,
        } => cmd_sync(port.as_deref(), pulses, period_ms, max_jitter_us),
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("error: {e:#}");
            ExitCode::from(2)
        }
    }
}
