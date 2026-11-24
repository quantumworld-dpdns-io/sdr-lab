//! Sender side of a link: per-link sequence numbers and wire encoding.

use crate::frame::{Error, MAX_WIRE};
use crate::msg::{Board, Msg};

/// Encoded size budget for [`LinkTx::encode`] (leading delimiter + frame).
pub const MAX_LINK_WIRE: usize = MAX_WIRE + 1;

/// Stamps outgoing frames with `src` and a wrapping per-link `seq`.
///
/// Every frame is written as `0x00 | frame | 0x00`. The extra leading
/// delimiter terminates any stray bytes already on the line (e.g. ESP-IDF
/// console text sharing UART0 with the host link), so they are rejected as
/// one garbage frame instead of corrupting the real one.
#[derive(Debug, Clone)]
pub struct LinkTx {
    pub src: Board,
    seq: u16,
}

impl LinkTx {
    pub const fn new(src: Board) -> Self {
        Self { src, seq: 0 }
    }

    pub fn next_seq(&self) -> u16 {
        self.seq
    }

    /// Encode `msg` into `out` (needs [`MAX_LINK_WIRE`] bytes); returns length.
    pub fn encode(&mut self, msg: &Msg, out: &mut [u8]) -> Result<usize, Error> {
        let (first, rest) = out.split_first_mut().ok_or(Error::BufferTooSmall)?;
        *first = 0;
        let n = msg.to_wire(self.src, self.seq, rest)?;
        self.seq = self.seq.wrapping_add(1);
        Ok(n + 1)
    }
}

/// Re-frame a raw COBS body (from [`crate::RxDecoder::last_body`]) for
/// verbatim forwarding, preserving the original `src` and `seq`.
pub fn reframe(body: &[u8], out: &mut [u8]) -> Option<usize> {
    let n = body.len() + 2;
    let dst = out.get_mut(..n)?;
    dst[0] = 0;
    dst[1..n - 1].copy_from_slice(body);
    dst[n - 1] = 0;
    Some(n)
}
