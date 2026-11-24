//! Ground-truth test payload placed by TX inside its 802.11 / BLE frames and
//! parsed by RX to match captures against [`crate::TxRecord`]s.
//!
//! ```text
//! magic "SDRL" | ver u8 | exp_id u32le | seq u32le | tx_time_us u32le | tx_power_qdbm i8 | channel u8 | pad…
//! ```

pub const MAGIC: [u8; 4] = *b"SDRL";
pub const VER: u8 = 1;
pub const LEN: usize = 4 + 1 + 4 + 4 + 4 + 1 + 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TestPayload {
    pub exp_id: u32,
    pub seq: u32,
    pub tx_time_us: u32,
    pub tx_power_qdbm: i8,
    pub channel: u8,
}

impl TestPayload {
    /// Write into `out` and pad the rest with a seq-derived pattern (never
    /// all-zero, so CSI/compression artefacts don't depend on padding).
    pub fn write(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < LEN {
            return None;
        }
        out[0..4].copy_from_slice(&MAGIC);
        out[4] = VER;
        out[5..9].copy_from_slice(&self.exp_id.to_le_bytes());
        out[9..13].copy_from_slice(&self.seq.to_le_bytes());
        out[13..17].copy_from_slice(&self.tx_time_us.to_le_bytes());
        out[17] = self.tx_power_qdbm as u8;
        out[18] = self.channel;
        for (i, b) in out[LEN..].iter_mut().enumerate() {
            *b = (self.seq as u8).wrapping_add(i as u8) | 0x01;
        }
        Some(out.len())
    }

    /// Find and parse a test payload anywhere in `frame` (e.g. after an
    /// 802.11 MAC header). Returns `None` for foreign frames.
    pub fn find(frame: &[u8]) -> Option<Self> {
        let pos = frame.windows(4).position(|w| w == MAGIC)?;
        let p = frame.get(pos..pos + LEN)?;
        if p[4] != VER {
            return None;
        }
        let u32_at = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        Some(Self {
            exp_id: u32_at(5),
            seq: u32_at(9),
            tx_time_us: u32_at(13),
            tx_power_qdbm: p[17] as i8,
            channel: p[18],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_inside_mac_frame() {
        let tp = TestPayload {
            exp_id: 7,
            seq: 123_456,
            tx_time_us: 999,
            tx_power_qdbm: 34,
            channel: 6,
        };
        let mut frame = [0xAAu8; 24 + 64]; // fake 802.11 header + body
        tp.write(&mut frame[24..]).unwrap();
        assert_eq!(TestPayload::find(&frame), Some(tp));
        assert_eq!(TestPayload::find(&[0u8; 40]), None);
    }
}
