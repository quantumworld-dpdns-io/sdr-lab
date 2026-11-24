//! Safety limits for active transmission. Enforced by TX firmware, the
//! controller and the MCP server — all call [`validate`]. See docs/safety.md.

use crate::msg::{ExperimentConfig, Mode};

/// 2.4 GHz channels the lab may transmit on (1–11: legal in all regions).
pub const CHANNEL_ALLOWLIST: core::ops::RangeInclusive<u8> = 1..=11;
/// 15 dBm, in quarter-dBm. Keep low: TX and RX sit close together.
pub const MAX_TX_POWER_QDBM: i8 = 60;
pub const MIN_TX_POWER_QDBM: i8 = 8;
pub const MIN_INTERVAL_MS: u32 = 10;
pub const MAX_DURATION_S: u32 = 3600;
pub const MAX_TEST_PAYLOAD: u16 = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitError {
    Channel(u8),
    TxPower(i8),
    Interval(u32),
    Duration(u32),
    PayloadLen(u16),
    IdleMode,
}

impl core::fmt::Display for LimitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Channel(c) => write!(f, "channel {c} outside allowlist 1..=11"),
            Self::TxPower(p) => write!(
                f,
                "tx_power_qdbm {p} outside {MIN_TX_POWER_QDBM}..={MAX_TX_POWER_QDBM}"
            ),
            Self::Interval(i) => write!(f, "interval_ms {i} < {MIN_INTERVAL_MS}"),
            Self::Duration(d) => write!(f, "duration_s {d} not in 1..={MAX_DURATION_S}"),
            Self::PayloadLen(l) => write!(f, "payload_len {l} > {MAX_TEST_PAYLOAD}"),
            Self::IdleMode => write!(f, "mode IDLE cannot be run"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for LimitError {}

pub fn validate(c: &ExperimentConfig) -> Result<(), LimitError> {
    if c.mode == Mode::Idle {
        return Err(LimitError::IdleMode);
    }
    if !CHANNEL_ALLOWLIST.contains(&c.channel) {
        return Err(LimitError::Channel(c.channel));
    }
    if !(MIN_TX_POWER_QDBM..=MAX_TX_POWER_QDBM).contains(&c.tx_power_qdbm) {
        return Err(LimitError::TxPower(c.tx_power_qdbm));
    }
    if c.interval_ms < MIN_INTERVAL_MS {
        return Err(LimitError::Interval(c.interval_ms));
    }
    if c.duration_s == 0 || c.duration_s > MAX_DURATION_S {
        return Err(LimitError::Duration(c.duration_s));
    }
    if c.payload_len > MAX_TEST_PAYLOAD {
        return Err(LimitError::PayloadLen(c.payload_len));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ok() -> ExperimentConfig {
        ExperimentConfig {
            exp_id: 1,
            mode: Mode::Rssi,
            channel: 6,
            interval_ms: 100,
            duration_s: 60,
            tx_power_qdbm: 34,
            payload_len: 64,
            replay_protect: false,
        }
    }

    #[test]
    fn accepts_baseline() {
        assert_eq!(validate(&ok()), Ok(()));
    }

    #[test]
    fn rejects_out_of_bounds() {
        assert_eq!(
            validate(&ExperimentConfig {
                channel: 13,
                ..ok()
            }),
            Err(LimitError::Channel(13))
        );
        assert_eq!(
            validate(&ExperimentConfig {
                tx_power_qdbm: 80,
                ..ok()
            }),
            Err(LimitError::TxPower(80))
        );
        assert_eq!(
            validate(&ExperimentConfig {
                interval_ms: 1,
                ..ok()
            }),
            Err(LimitError::Interval(1))
        );
        assert_eq!(
            validate(&ExperimentConfig {
                duration_s: 0,
                ..ok()
            }),
            Err(LimitError::Duration(0))
        );
        assert_eq!(
            validate(&ExperimentConfig {
                mode: Mode::Idle,
                ..ok()
            }),
            Err(LimitError::IdleMode)
        );
    }
}
