//! Serial connection to the controller (or a single board for debugging).

use std::io::{ErrorKind, Read, Write};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use lab_proto::link::MAX_LINK_WIRE;
use lab_proto::{Board, Error, Header, LinkTx, Msg, RxDecoder};
use serialport::{SerialPort, SerialPortType};

pub const BAUD: u32 = 921_600;

/// USB-UART bridges used on ESP32 devkits (CTRL board).
const BRIDGE_VIDS: [(u16, &str); 3] = [(0x10C4, "CP210x"), (0x1A86, "CH34x"), (0x0403, "FTDI")];
/// Espressif native USB (an S3's USB-Serial-JTAG, not the controller).
const ESPRESSIF_VID: u16 = 0x303A;

// Size is irrelevant on the host; keeping Msg inline keeps matching simple.
#[allow(clippy::large_enum_variant)]
#[derive(Debug)]
pub enum Event {
    Frame {
        hdr: Header,
        msg: Result<Msg, Error>,
    },
    /// Non-frame bytes on the line (boot ROM output, console logs).
    Text(String),
}

pub struct Conn {
    pub path: String,
    port: Box<dyn SerialPort>,
    tx: LinkTx,
    rx: mpsc::Receiver<Event>,
}

/// Pick the controller port: an explicit path, else the only USB-UART bridge.
pub fn find_port(explicit: Option<&str>) -> Result<String> {
    if let Some(p) = explicit {
        return Ok(p.to_string());
    }
    let ports = serialport::available_ports().context("listing serial ports")?;
    let mut bridges = Vec::new();
    let mut listing = Vec::new();
    for p in &ports {
        // macOS lists both tty.* and cu.*; cu.* doesn't wait for carrier.
        if p.port_name.starts_with("/dev/tty.") {
            continue;
        }
        if let SerialPortType::UsbPort(u) = &p.port_type {
            let kind = BRIDGE_VIDS
                .iter()
                .find(|(v, _)| *v == u.vid)
                .map(|(_, n)| *n)
                .or((u.vid == ESPRESSIF_VID).then_some("Espressif USB (S3 native, not CTRL)"))
                .unwrap_or("other USB");
            listing.push(format!(
                "  {}  {:04x}:{:04x}  {kind}",
                p.port_name, u.vid, u.pid
            ));
            if BRIDGE_VIDS.iter().any(|(v, _)| *v == u.vid) {
                bridges.push(p.port_name.clone());
            }
        }
    }
    match bridges.as_slice() {
        [one] => Ok(one.clone()),
        [] => bail!(
            "no USB-UART bridge found for the controller; pass --port. USB ports:\n{}",
            if listing.is_empty() {
                "  (none)".into()
            } else {
                listing.join("\n")
            }
        ),
        _ => bail!(
            "several USB-UART bridges found; pass --port:\n{}",
            listing.join("\n")
        ),
    }
}

impl Conn {
    pub fn open(explicit: Option<&str>) -> Result<Self> {
        let path = find_port(explicit)?;
        let mut port = serialport::new(&path, BAUD)
            .timeout(Duration::from_millis(50))
            .open()
            .with_context(|| format!("opening {path}"))?;
        // Devkit auto-reset wiring: DTR/RTS both deasserted = run normally
        // (never leave IO0 low while EN rises, which enters the bootloader).
        let _ = port.write_data_terminal_ready(false);
        let _ = port.write_request_to_send(false);

        let mut reader = port.try_clone().context("cloning serial port")?;
        let (etx, erx) = mpsc::channel();
        thread::spawn(move || {
            let mut dec = RxDecoder::new();
            let mut buf = [0u8; 1024];
            loop {
                let n = match reader.read(&mut buf) {
                    Ok(n) => n,
                    Err(e) if e.kind() == ErrorKind::TimedOut => continue,
                    Err(_) => return,
                };
                for &b in &buf[..n] {
                    let ev = match dec.feed(b) {
                        None => continue,
                        Some(Ok((hdr, p))) => Event::Frame {
                            hdr,
                            msg: Msg::from_frame(&hdr, p),
                        },
                        Some(Err(_)) => {
                            let s = String::from_utf8_lossy(dec.last_body());
                            Event::Text(s.trim_end().to_string())
                        }
                    };
                    if etx.send(ev).is_err() {
                        return;
                    }
                }
            }
        });
        Ok(Self {
            path,
            port,
            tx: LinkTx::new(Board::Host),
            rx: erx,
        })
    }

    pub fn send(&mut self, msg: &Msg) -> Result<()> {
        let mut buf = [0u8; MAX_LINK_WIRE];
        let n = self.tx.encode(msg, &mut buf)?;
        self.port.write_all(&buf[..n])?;
        self.port.flush()?;
        Ok(())
    }

    pub fn recv_until(&self, deadline: Instant) -> Option<Event> {
        let left = deadline.checked_duration_since(Instant::now())?;
        self.rx.recv_timeout(left).ok()
    }

    /// Feed every event to `f` until `dur` elapses or `f` returns false.
    pub fn collect_for(&self, dur: Duration, mut f: impl FnMut(Event) -> bool) {
        let deadline = Instant::now() + dur;
        while let Some(ev) = self.recv_until(deadline) {
            if !f(ev) {
                break;
            }
        }
    }
}
