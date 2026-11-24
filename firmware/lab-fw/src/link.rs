//! Framed UART link: a thread-safe sender plus a reader thread.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::Result;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::uart::{UartDriver, UartRxDriver, UartTxDriver};
use lab_proto::link::{reframe, MAX_LINK_WIRE};
use lab_proto::{Board, Error, Header, LinkTx, Msg, RxDecoder};

pub struct Link {
    pub name: &'static str,
    tx: Mutex<(UartTxDriver<'static>, LinkTx)>,
    /// Frames that failed to decode on the RX side of this link.
    pub rx_errors: AtomicU32,
    /// Frames that could not be written (encode failure / UART error).
    pub tx_errors: AtomicU32,
}

impl Link {
    /// Take ownership of a configured UART; returns the shared sender and the
    /// receive half for [`Link::spawn_reader`].
    pub fn new(
        name: &'static str,
        uart: UartDriver<'static>,
        src: Board,
    ) -> (Arc<Self>, UartRxDriver<'static>) {
        let (tx, rx) = uart.into_split();
        let link = Arc::new(Self {
            name,
            tx: Mutex::new((tx, LinkTx::new(src))),
            rx_errors: AtomicU32::new(0),
            tx_errors: AtomicU32::new(0),
        });
        (link, rx)
    }

    /// Encode and send one message. Errors are counted, logged, and swallowed:
    /// a dead link must never take the board down.
    pub fn send(&self, msg: &Msg) {
        let mut buf = [0u8; MAX_LINK_WIRE];
        let mut guard = self.tx.lock().unwrap();
        let (uart, ltx) = &mut *guard;
        match ltx.encode(msg, &mut buf) {
            Ok(n) => self.write_all(uart, &buf[..n]),
            Err(e) => {
                self.tx_errors.fetch_add(1, Ordering::Relaxed);
                log::warn!("{}: encode {:?} failed: {e}", self.name, msg.msg_type());
            }
        }
    }

    /// Forward a frame received elsewhere, unchanged (keeps its src/seq).
    pub fn forward(&self, body: &[u8]) {
        let mut buf = [0u8; MAX_LINK_WIRE + 1];
        let Some(n) = reframe(body, &mut buf) else {
            self.tx_errors.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let mut guard = self.tx.lock().unwrap();
        self.write_all(&mut guard.0, &buf[..n]);
    }

    fn write_all(&self, uart: &mut UartTxDriver<'static>, mut bytes: &[u8]) {
        // One frame per lock hold, so concurrent senders never interleave.
        while !bytes.is_empty() {
            match uart.write(bytes) {
                Ok(0) => thread::yield_now(),
                Ok(n) => bytes = &bytes[n..],
                Err(e) => {
                    self.tx_errors.fetch_add(1, Ordering::Relaxed);
                    log::warn!("{}: uart write failed: {e}", self.name);
                    return;
                }
            }
        }
    }

    /// Spawn the reader thread. `on_frame` receives every valid frame, its
    /// decoded message (or a decode error for unknown/newer types) and the
    /// raw body for verbatim forwarding.
    pub fn spawn_reader<F>(self: &Arc<Self>, rx: UartRxDriver<'static>, mut on_frame: F) -> Result<()>
    where
        F: FnMut(&Header, Result<Msg, Error>, &[u8]) + Send + 'static,
    {
        let link = Arc::clone(self);
        thread::Builder::new()
            .name(format!("rx-{}", self.name))
            .stack_size(crate::THREAD_STACK)
            .spawn(move || {
                let mut dec = RxDecoder::new();
                let mut buf = [0u8; 256];
                let timeout = TickType::new_millis(20).ticks();
                loop {
                    let n = match rx.read(&mut buf, timeout) {
                        Ok(n) => n,
                        Err(e) => {
                            log::warn!("{}: uart read failed: {e}", link.name);
                            thread::sleep(std::time::Duration::from_millis(100));
                            continue;
                        }
                    };
                    for &b in &buf[..n] {
                        let res = match dec.feed(b) {
                            None => continue,
                            Some(Ok((h, p))) => Ok((h, Msg::from_frame(&h, p))),
                            Some(Err(e)) => Err(e),
                        };
                        match res {
                            Ok((h, msg)) => on_frame(&h, msg, dec.last_body()),
                            Err(e) => {
                                link.rx_errors.fetch_add(1, Ordering::Relaxed);
                                let body = dec.last_body();
                                // Stray console text is expected on UART0; keep it quiet.
                                if body.iter().all(|c| c.is_ascii_graphic() || c.is_ascii_whitespace()) {
                                    log::debug!("{}: text: {}", link.name, String::from_utf8_lossy(body).trim_end());
                                } else {
                                    log::debug!("{}: bad frame ({e}), {} bytes", link.name, body.len());
                                }
                            }
                        }
                    }
                }
            })?;
        Ok(())
    }
}
