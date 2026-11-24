//! Framing: header + payload + CRC16, COBS-stuffed, 0x00-delimited.

use crate::{cobs, crc};

pub const PROTO_VER: u8 = 1;
pub const HDR_LEN: usize = 8;
pub const CRC_LEN: usize = 2;
pub const MAX_PAYLOAD: usize = 512;
pub const MAX_RAW: usize = HDR_LEN + MAX_PAYLOAD + CRC_LEN;
/// Encoded frame without the trailing delimiter.
pub const MAX_WIRE_BODY: usize = cobs::max_encoded_len(MAX_RAW);
/// Encoded frame including the trailing 0x00 delimiter.
pub const MAX_WIRE: usize = MAX_WIRE_BODY + 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub ver: u8,
    pub msg_type: u8,
    pub src: u8,
    pub flags: u8,
    pub seq: u16,
    pub len: u16,
}

impl Header {
    pub fn new(msg_type: u8, src: u8, seq: u16) -> Self {
        Self {
            ver: PROTO_VER,
            msg_type,
            src,
            flags: 0,
            seq,
            len: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    PayloadTooLong,
    BufferTooSmall,
    Cobs,
    Short,
    Length,
    Crc,
    Version(u8),
    Overflow,
    UnknownType(u8),
    Postcard,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

/// Encode a complete wire frame (including trailing 0x00) into `out`.
/// `hdr.len` is ignored and taken from `payload`. Returns bytes written.
pub fn encode_frame(hdr: &Header, payload: &[u8], out: &mut [u8]) -> Result<usize, Error> {
    if payload.len() > MAX_PAYLOAD {
        return Err(Error::PayloadTooLong);
    }
    let mut raw = [0u8; MAX_RAW];
    raw[0] = hdr.ver;
    raw[1] = hdr.msg_type;
    raw[2] = hdr.src;
    raw[3] = hdr.flags;
    raw[4..6].copy_from_slice(&hdr.seq.to_le_bytes());
    raw[6..8].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    let body = HDR_LEN + payload.len();
    raw[HDR_LEN..body].copy_from_slice(payload);
    let c = crc::crc16(&raw[..body], crc::INIT);
    raw[body..body + CRC_LEN].copy_from_slice(&c.to_le_bytes());
    let n = body + CRC_LEN;

    if out.len() < cobs::max_encoded_len(n) + 1 {
        return Err(Error::BufferTooSmall);
    }
    let w = cobs::encode(&raw[..n], out).ok_or(Error::BufferTooSmall)?;
    out[w] = 0;
    Ok(w + 1)
}

/// Decode one frame body (delimiter stripped). The payload borrows `scratch`.
pub fn decode_frame<'a>(
    wire: &[u8],
    scratch: &'a mut [u8; MAX_RAW],
) -> Result<(Header, &'a [u8]), Error> {
    let n = cobs::decode(wire, scratch).ok_or(Error::Cobs)?;
    if n < HDR_LEN + CRC_LEN {
        return Err(Error::Short);
    }
    let raw = &scratch[..n];
    let len = u16::from_le_bytes([raw[6], raw[7]]) as usize;
    if len > MAX_PAYLOAD || n != HDR_LEN + len + CRC_LEN {
        return Err(Error::Length);
    }
    let body = HDR_LEN + len;
    let want = u16::from_le_bytes([raw[body], raw[body + 1]]);
    if crc::crc16(&raw[..body], crc::INIT) != want {
        return Err(Error::Crc);
    }
    if raw[0] != PROTO_VER {
        return Err(Error::Version(raw[0]));
    }
    let hdr = Header {
        ver: raw[0],
        msg_type: raw[1],
        src: raw[2],
        flags: raw[3],
        seq: u16::from_le_bytes([raw[4], raw[5]]),
        len: len as u16,
    };
    Ok((hdr, &scratch[HDR_LEN..body]))
}

/// Byte-stream reassembler for UART/USB. Statically sized; no allocation.
pub struct RxDecoder {
    buf: [u8; MAX_WIRE_BODY],
    n: usize,
    overflow: bool,
    /// Length of the most recently completed body in `buf`.
    last: usize,
    scratch: [u8; MAX_RAW],
    pub n_ok: u32,
    pub n_err: u32,
}

impl Default for RxDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl RxDecoder {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_WIRE_BODY],
            n: 0,
            overflow: false,
            last: 0,
            scratch: [0; MAX_RAW],
            n_ok: 0,
            n_err: 0,
        }
    }

    /// Feed one byte. Returns `Some` when a delimiter completes a frame
    /// (valid or not); the decoder has already resynchronised either way.
    pub fn feed(&mut self, byte: u8) -> Option<Result<(Header, &[u8]), Error>> {
        if byte != 0 {
            if self.n < self.buf.len() {
                self.buf[self.n] = byte;
                self.n += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }
        let (n, ovf) = (self.n, self.overflow);
        self.last = n;
        self.n = 0;
        self.overflow = false;
        if n == 0 {
            return None;
        }
        if ovf {
            self.n_err += 1;
            return Some(Err(Error::Overflow));
        }
        let r = decode_frame(&self.buf[..n], &mut self.scratch);
        match r {
            Ok(_) => self.n_ok += 1,
            Err(_) => self.n_err += 1,
        }
        Some(r)
    }

    /// Raw COBS body (delimiter stripped) of the frame or garbage run that the
    /// last `Some(..)` from [`feed`](Self::feed) reported. Valid until the next
    /// non-zero byte is fed. Used to forward frames verbatim and to surface
    /// console text that arrived between frames.
    pub fn last_body(&self) -> &[u8] {
        &self.buf[..self.last.min(self.buf.len())]
    }
}
