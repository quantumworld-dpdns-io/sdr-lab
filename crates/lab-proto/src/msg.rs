//! Typed messages carried in frame payloads (postcard-encoded).
//!
//! Append-only: new variants/fields go at the end (see crate docs).

use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

use crate::frame::{self, Error, Header, MAX_PAYLOAD};

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MsgType {
    Hello = 1,
    Status = 2,
    Cmd = 3,
    Ack = 4,
    TriggerEvt = 5,
    TxRecord = 6,
    PktMeta = 7,
    CsiFrame = 8,
    RssiSample = 9,
    BleAdv = 10,
    Log = 11,
}

impl TryFrom<u8> for MsgType {
    type Error = Error;
    fn try_from(v: u8) -> Result<Self, Error> {
        use MsgType::*;
        Ok(match v {
            1 => Hello,
            2 => Status,
            3 => Cmd,
            4 => Ack,
            5 => TriggerEvt,
            6 => TxRecord,
            7 => PktMeta,
            8 => CsiFrame,
            9 => RssiSample,
            10 => BleAdv,
            11 => Log,
            _ => return Err(Error::UnknownType(v)),
        })
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum Board {
    Unknown = 0,
    Tx = 1,
    Rx = 2,
    Ctrl = 3,
    Host = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum Mode {
    Idle,
    Rssi,
    /// Sniffer + packet-loss benchmark.
    Wifi,
    Csi,
    Ble,
    Replay,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RunState {
    Idle,
    Configure,
    Armed,
    Running,
    Draining,
    Done,
    Error,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Hello {
    pub board: Board,
    pub mac: [u8; 6],
    pub fw_version: String<24>,
    pub git_hash: String<16>,
    pub chip: String<12>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Status {
    pub state: RunState,
    pub uptime_ms: u32,
    pub free_heap: u32,
    /// Records dropped by local queues.
    pub dropped: u32,
    /// Records produced in the current run.
    pub records: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExperimentConfig {
    pub exp_id: u32,
    pub mode: Mode,
    pub channel: u8,
    pub interval_ms: u32,
    pub duration_s: u32,
    /// Quarter-dBm, as `esp_wifi_set_max_tx_power`.
    pub tx_power_qdbm: i8,
    pub payload_len: u16,
    pub replay_protect: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CmdBody {
    Configure(ExperimentConfig),
    Arm,
    Stop,
    Ping,
    StatusReq,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Cmd {
    /// Echoed in [`Ack`].
    pub cmd_id: u32,
    pub body: CmdBody,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Ack {
    pub cmd_id: u32,
    pub ok: bool,
    pub error: String<48>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TriggerEvt {
    pub exp_id: u32,
    /// `esp_timer_get_time()` latched at the trigger edge.
    pub t0_local_us: u64,
}

/// Ground truth: one per transmitted test frame / advertisement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TxRecord {
    pub exp_id: u32,
    pub seq: u32,
    /// µs since t0.
    pub t_us: u32,
    pub channel: u8,
    pub tx_power_qdbm: i8,
    pub is_replay: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PktMeta {
    pub exp_id: u32,
    /// Parsed from the SDRL test payload; `None` for foreign frames.
    pub seq: Option<u32>,
    pub t_us: u32,
    pub rssi: i8,
    pub noise_floor: i8,
    pub channel: u8,
    pub rate: u8,
    pub sig_len: u16,
    pub src_mac: [u8; 6],
    pub replay_accepted: bool,
}

pub const CSI_MAX_IQ: usize = 384;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CsiFrame {
    pub exp_id: u32,
    pub seq: u32,
    pub t_us: u32,
    pub rssi: i8,
    pub channel: u8,
    pub first_word_invalid: bool,
    /// int8 interleaved (imag, real) as delivered by ESP-IDF.
    pub iq: Vec<i8, CSI_MAX_IQ>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RssiSample {
    pub t_us: u32,
    pub rssi: i8,
    pub channel: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BleAdv {
    pub exp_id: u32,
    pub seq: u32,
    pub t_us: u32,
    pub rssi: i8,
    pub addr: [u8; 6],
    /// 37/38/39 when known, else 0.
    pub adv_channel: u8,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Log {
    pub level: u8,
    pub tag: String<16>,
    pub msg: String<96>,
}

/// Any message, tagged by the frame header's `msg_type`.
// CsiFrame is held inline on purpose: firmware has no allocator on this path.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum Msg {
    Hello(Hello),
    Status(Status),
    Cmd(Cmd),
    Ack(Ack),
    TriggerEvt(TriggerEvt),
    TxRecord(TxRecord),
    PktMeta(PktMeta),
    CsiFrame(CsiFrame),
    RssiSample(RssiSample),
    BleAdv(BleAdv),
    Log(Log),
}

impl Msg {
    pub fn msg_type(&self) -> MsgType {
        match self {
            Msg::Hello(_) => MsgType::Hello,
            Msg::Status(_) => MsgType::Status,
            Msg::Cmd(_) => MsgType::Cmd,
            Msg::Ack(_) => MsgType::Ack,
            Msg::TriggerEvt(_) => MsgType::TriggerEvt,
            Msg::TxRecord(_) => MsgType::TxRecord,
            Msg::PktMeta(_) => MsgType::PktMeta,
            Msg::CsiFrame(_) => MsgType::CsiFrame,
            Msg::RssiSample(_) => MsgType::RssiSample,
            Msg::BleAdv(_) => MsgType::BleAdv,
            Msg::Log(_) => MsgType::Log,
        }
    }

    /// Serialize the payload (postcard) into `out`; returns the used slice.
    pub fn encode_payload<'a>(&self, out: &'a mut [u8]) -> Result<&'a mut [u8], Error> {
        let r = match self {
            Msg::Hello(m) => postcard::to_slice(m, out),
            Msg::Status(m) => postcard::to_slice(m, out),
            Msg::Cmd(m) => postcard::to_slice(m, out),
            Msg::Ack(m) => postcard::to_slice(m, out),
            Msg::TriggerEvt(m) => postcard::to_slice(m, out),
            Msg::TxRecord(m) => postcard::to_slice(m, out),
            Msg::PktMeta(m) => postcard::to_slice(m, out),
            Msg::CsiFrame(m) => postcard::to_slice(m, out),
            Msg::RssiSample(m) => postcard::to_slice(m, out),
            Msg::BleAdv(m) => postcard::to_slice(m, out),
            Msg::Log(m) => postcard::to_slice(m, out),
        };
        r.map_err(|_| Error::Postcard)
    }

    pub fn decode_payload(ty: MsgType, payload: &[u8]) -> Result<Self, Error> {
        fn p<'de, T: Deserialize<'de>>(b: &'de [u8]) -> Result<T, Error> {
            postcard::from_bytes(b).map_err(|_| Error::Postcard)
        }
        Ok(match ty {
            MsgType::Hello => Msg::Hello(p(payload)?),
            MsgType::Status => Msg::Status(p(payload)?),
            MsgType::Cmd => Msg::Cmd(p(payload)?),
            MsgType::Ack => Msg::Ack(p(payload)?),
            MsgType::TriggerEvt => Msg::TriggerEvt(p(payload)?),
            MsgType::TxRecord => Msg::TxRecord(p(payload)?),
            MsgType::PktMeta => Msg::PktMeta(p(payload)?),
            MsgType::CsiFrame => Msg::CsiFrame(p(payload)?),
            MsgType::RssiSample => Msg::RssiSample(p(payload)?),
            MsgType::BleAdv => Msg::BleAdv(p(payload)?),
            MsgType::Log => Msg::Log(p(payload)?),
        })
    }

    /// Encode as a complete wire frame (with trailing 0x00) into `out`.
    pub fn to_wire(&self, src: Board, seq: u16, out: &mut [u8]) -> Result<usize, Error> {
        let mut buf = [0u8; MAX_PAYLOAD];
        let payload = self.encode_payload(&mut buf)?;
        let hdr = Header::new(self.msg_type() as u8, src as u8, seq);
        frame::encode_frame(&hdr, payload, out)
    }

    /// Decode a typed message from an already-validated frame.
    pub fn from_frame(hdr: &Header, payload: &[u8]) -> Result<Self, Error> {
        Self::decode_payload(MsgType::try_from(hdr.msg_type)?, payload)
    }
}
