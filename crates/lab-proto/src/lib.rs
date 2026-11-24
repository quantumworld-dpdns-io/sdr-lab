//! SDR Lab link protocol, shared by the TX/RX/CTRL firmware and the host tools.
//!
//! Wire format (see `docs/protocol.md`):
//!
//! ```text
//! COBS( header[8] | payload[len] | crc16_le[2] ) 0x00
//! header  = ver u8 | msg_type u8 | src u8 | flags u8 | seq u16le | len u16le
//! crc16   = CRC-16/CCITT-FALSE over header + payload
//! payload = postcard encoding of the message selected by msg_type
//! ```
//!
//! Schema stability: postcard encodes enum variants and struct fields by
//! position. **Only append** variants/fields; never reorder or remove.
#![cfg_attr(not(feature = "std"), no_std)]

pub mod cobs;
pub mod crc;
pub mod frame;
pub mod limits;
pub mod msg;
pub mod testframe;

pub use frame::{decode_frame, encode_frame, Error, Header, RxDecoder};
pub use msg::*;
