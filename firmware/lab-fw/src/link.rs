//! Framed link over any byte transport: a thread-safe sender plus a reader
//! thread. Transports: UART (board ↔ board, CTRL ↔ host) and the ESP32-S3's
//! native USB-Serial-JTAG (board ↔ Mac in host-link mode).

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::Result;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::uart::{UartDriver, UartRxDriver, UartTxDriver};
use esp_idf_svc::sys::EspError;
use lab_proto::link::{reframe, MAX_LINK_WIRE};
use lab_proto::{Board, Error, Header, LinkTx, Msg, RxDecoder};

/// Write half of a transport. Returns bytes accepted (may be partial).
pub trait LinkWrite: Send + 'static {
    fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError>;
}

/// Read half of a transport. Returns 0 on timeout.
pub trait LinkRead: Send + 'static {
    fn read(&mut self, buf: &mut [u8], timeout_ms: u64) -> Result<usize, EspError>;
}

impl LinkWrite for UartTxDriver<'static> {
    fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError> {
        UartTxDriver::write(self, bytes)
    }
}

impl LinkRead for UartRxDriver<'static> {
    fn read(&mut self, buf: &mut [u8], timeout_ms: u64) -> Result<usize, EspError> {
        UartRxDriver::read(self, buf, TickType::new_millis(timeout_ms).ticks())
    }
}

pub struct Link {
    pub name: &'static str,
    tx: Mutex<(Box<dyn LinkWrite>, LinkTx)>,
    /// Frames that failed to decode on the RX side of this link.
    pub rx_errors: AtomicU32,
    /// Frames that could not be written (encode failure / transport error).
    pub tx_errors: AtomicU32,
}

impl Link {
    pub fn new(
        name: &'static str,
        writer: Box<dyn LinkWrite>,
        src: Board,
    ) -> Arc<Self> {
        Arc::new(Self {
            name,
            tx: Mutex::new((writer, LinkTx::new(src))),
            rx_errors: AtomicU32::new(0),
            tx_errors: AtomicU32::new(0),
        })
    }

    /// Link over a configured UART; returns the sender and the read half.
    pub fn uart(
        name: &'static str,
        uart: UartDriver<'static>,
        src: Board,
    ) -> (Arc<Self>, Box<dyn LinkRead>) {
        let (tx, rx) = uart.into_split();
        (Self::new(name, Box::new(tx), src), Box::new(rx))
    }

    /// Encode and send one message. Errors are counted, logged, and swallowed:
    /// a dead link must never take the board down.
    pub fn send(&self, msg: &Msg) {
        let mut buf = [0u8; MAX_LINK_WIRE];
        let mut guard = self.tx.lock().unwrap();
        let (w, ltx) = &mut *guard;
        match ltx.encode(msg, &mut buf) {
            Ok(n) => self.write_all(w.as_mut(), &buf[..n]),
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
        self.write_all(guard.0.as_mut(), &buf[..n]);
    }

    fn write_all(&self, w: &mut dyn LinkWrite, mut bytes: &[u8]) {
        // One frame per lock hold, so concurrent senders never interleave.
        // A transport that stops accepting bytes (USB host not reading) gets a
        // bounded number of retries, then the frame is dropped and counted.
        let mut stalls = 0;
        while !bytes.is_empty() {
            match w.write(bytes) {
                Ok(0) => {
                    stalls += 1;
                    if stalls > 5 {
                        self.tx_errors.fetch_add(1, Ordering::Relaxed);
                        return;
                    }
                    thread::yield_now();
                }
                Ok(n) => bytes = &bytes[n..],
                Err(e) => {
                    self.tx_errors.fetch_add(1, Ordering::Relaxed);
                    log::warn!("{}: write failed: {e}", self.name);
                    return;
                }
            }
        }
    }

    /// Spawn the reader thread. `on_frame` receives every valid frame, its
    /// decoded message (or a decode error for unknown/newer types) and the
    /// raw body for verbatim forwarding.
    pub fn spawn_reader<F>(self: &Arc<Self>, mut rx: Box<dyn LinkRead>, mut on_frame: F) -> Result<()>
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
                loop {
                    let n = match rx.read(&mut buf, 20) {
                        Ok(n) => n,
                        Err(e) => {
                            log::warn!("{}: read failed: {e}", link.name);
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
                                log::debug!("{}: bad frame ({e}), {} bytes", link.name, dec.last_body().len());
                            }
                        }
                    }
                }
            })?;
        Ok(())
    }
}

/// ESP32-S3 native USB-Serial-JTAG transport (host-link mode).
#[cfg(feature = "usb-jtag")]
pub mod usb_jtag {
    use super::*;
    use esp_idf_svc::hal::usb_serial::{config::Config, UsbDMinGpio, UsbDPlusGpio, UsbSerialDriver, USB_SERIAL};
    use esp_idf_svc::sys;

    struct Tx;
    struct Rx;

    impl LinkWrite for Tx {
        fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError> {
            // SAFETY: driver installed in `link()` and never uninstalled; the
            // IDF driver is internally synchronised (separate TX/RX rings).
            let n = unsafe {
                sys::usb_serial_jtag_write_bytes(bytes.as_ptr().cast(), bytes.len(), TickType::new_millis(20).ticks())
            };
            Ok(n.max(0) as usize)
        }
    }

    impl LinkRead for Rx {
        fn read(&mut self, buf: &mut [u8], timeout_ms: u64) -> Result<usize, EspError> {
            // SAFETY: as above.
            let n = unsafe {
                sys::usb_serial_jtag_read_bytes(buf.as_mut_ptr().cast(), buf.len() as u32, TickType::new_millis(timeout_ms).ticks())
            };
            Ok(n.max(0) as usize)
        }
    }

    /// Install the USB-Serial-JTAG driver and build a link over it. The ESP-IDF
    /// console shares this port; it is switched to driver mode so every log
    /// write is atomic with respect to frame writes, and the link's leading
    /// 0x00 delimiter keeps any log text from corrupting a frame.
    pub fn link(
        name: &'static str,
        usb: USB_SERIAL<'static>,
        dm: UsbDMinGpio<'static>,
        dp: UsbDPlusGpio<'static>,
        src: Board,
    ) -> Result<(Arc<Link>, Box<dyn LinkRead>)> {
        let drv = UsbSerialDriver::new(usb, dm, dp, &Config::new().tx_buffer_size(4096).rx_buffer_size(1024))?;
        // Keep the driver installed for the life of the firmware; the two
        // halves call the thread-safe C API directly.
        std::mem::forget(drv);
        // SAFETY: the driver was installed just above.
        unsafe { sys::esp_vfs_usb_serial_jtag_use_driver() };
        Ok((Link::new(name, Box::new(Tx), src), Box::new(Rx)))
    }
}
