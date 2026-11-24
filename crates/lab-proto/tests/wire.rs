//! Wire-format conformance: frozen golden vectors (tests/golden/frames.json)
//! are the protocol spec. Any change that breaks them is a protocol break.

use lab_proto::frame::{Header, MAX_PAYLOAD, MAX_WIRE, PROTO_VER};
use lab_proto::*;

#[derive(serde::Deserialize)]
struct Golden {
    name: String,
    msg_type: u8,
    src: u8,
    seq: u16,
    flags: u8,
    payload: String,
    wire: String,
}

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn golden() -> Vec<Golden> {
    serde_json::from_str(include_str!("golden/frames.json")).unwrap()
}

#[test]
fn golden_vectors_encode_and_decode() {
    for g in golden() {
        let payload = hex(&g.payload);
        let wire = hex(&g.wire);
        let hdr = Header {
            ver: PROTO_VER,
            msg_type: g.msg_type,
            src: g.src,
            flags: g.flags,
            seq: g.seq,
            len: 0,
        };
        let mut out = [0u8; MAX_WIRE];
        let n = encode_frame(&hdr, &payload, &mut out).unwrap();
        assert_eq!(&out[..n], &wire[..], "encode {}", g.name);

        let mut scratch = [0u8; frame::MAX_RAW];
        let (h, p) = decode_frame(&wire[..wire.len() - 1], &mut scratch).unwrap();
        assert_eq!(
            h,
            Header {
                len: payload.len() as u16,
                ..hdr
            },
            "hdr {}",
            g.name
        );
        assert_eq!(p, &payload[..], "payload {}", g.name);
    }
}

#[test]
fn stream_resyncs_after_garbage_and_chunks() {
    let mut stream = b"\x13\x37garbage\x00".to_vec();
    for g in golden() {
        stream.extend(hex(&g.wire));
    }
    let mut rx = RxDecoder::new();
    let mut seqs = Vec::new();
    let mut errs = 0;
    for b in stream {
        match rx.feed(b) {
            Some(Ok((h, _))) => seqs.push(h.seq),
            Some(Err(_)) => errs += 1,
            None => {}
        }
    }
    assert_eq!(seqs, golden().iter().map(|g| g.seq).collect::<Vec<_>>());
    assert_eq!(errs, 1);
}

#[test]
fn corruption_and_overflow_detected() {
    let g = &golden()[3];
    let mut wire = hex(&g.wire);
    wire[4] ^= 0x01;
    let mut scratch = [0u8; frame::MAX_RAW];
    assert!(decode_frame(&wire[..wire.len() - 1], &mut scratch).is_err());

    let mut out = [0u8; MAX_WIRE + 8];
    let hdr = Header::new(1, 1, 0);
    assert_eq!(
        encode_frame(&hdr, &[0u8; MAX_PAYLOAD + 1], &mut out),
        Err(Error::PayloadTooLong)
    );

    let mut rx = RxDecoder::new();
    for _ in 0..MAX_WIRE + 10 {
        assert!(rx.feed(0x01).is_none());
    }
    assert!(matches!(rx.feed(0), Some(Err(Error::Overflow))));
}

fn s<const N: usize>(v: &str) -> heapless::String<N> {
    heapless::String::try_from(v).unwrap()
}

fn sample_messages() -> Vec<Msg> {
    let cfg = ExperimentConfig {
        exp_id: 1,
        mode: Mode::Csi,
        channel: 6,
        interval_ms: 10,
        duration_s: 60,
        tx_power_qdbm: 34,
        payload_len: 64,
        replay_protect: true,
    };
    let mut iq = heapless::Vec::new();
    for i in 0..CSI_MAX_IQ {
        iq.push((i as i32 * 7 - 128) as i8).unwrap();
    }
    vec![
        Msg::Hello(Hello {
            board: Board::Rx,
            mac: [1, 2, 3, 4, 5, 6],
            fw_version: s("0.1.0"),
            git_hash: s("deadbeef"),
            chip: s("esp32s3"),
        }),
        Msg::Status(Status {
            state: RunState::Running,
            uptime_ms: u32::MAX,
            free_heap: 123_456,
            dropped: 0,
            records: 9,
        }),
        Msg::Cmd(Cmd {
            cmd_id: 5,
            body: CmdBody::Configure(cfg),
        }),
        Msg::Cmd(Cmd {
            cmd_id: 6,
            body: CmdBody::Arm,
        }),
        Msg::Ack(Ack {
            cmd_id: 5,
            ok: false,
            error: s("channel 13 outside allowlist 1..=11"),
        }),
        Msg::TriggerEvt(TriggerEvt {
            exp_id: 1,
            t0_local_us: u64::MAX,
            edge: 7,
        }),
        Msg::Cmd(Cmd {
            cmd_id: 7,
            body: CmdBody::SyncTest {
                pulses: 20,
                period_ms: 100,
            },
        }),
        Msg::TxRecord(TxRecord {
            exp_id: 1,
            seq: 42,
            t_us: 100_000,
            channel: 6,
            tx_power_qdbm: 34,
            is_replay: false,
        }),
        Msg::PktMeta(PktMeta {
            exp_id: 1,
            seq: None,
            t_us: 1,
            rssi: -90,
            noise_floor: -96,
            channel: 6,
            rate: 11,
            sig_len: 120,
            src_mac: [0xAA; 6],
            replay_accepted: false,
        }),
        // Worst case: must fit MAX_PAYLOAD.
        Msg::CsiFrame(CsiFrame {
            exp_id: u32::MAX,
            seq: u32::MAX,
            t_us: u32::MAX,
            rssi: -128,
            channel: 11,
            first_word_invalid: true,
            iq,
        }),
        Msg::RssiSample(RssiSample {
            t_us: 3,
            rssi: -42,
            channel: 1,
        }),
        Msg::BleAdv(BleAdv {
            exp_id: 1,
            seq: 2,
            t_us: 3,
            rssi: -60,
            addr: [9; 6],
            adv_channel: 38,
        }),
        Msg::Log(Log {
            level: 3,
            tag: s("wifi_csi"),
            msg: s(&"x".repeat(96)),
        }),
    ]
}

#[test]
fn every_message_roundtrips_through_the_wire() {
    let mut rx = RxDecoder::new();
    for (i, m) in sample_messages().into_iter().enumerate() {
        let mut out = [0u8; MAX_WIRE];
        let n = m
            .to_wire(Board::Ctrl, i as u16, &mut out)
            .unwrap_or_else(|e| panic!("{m:?}: {e}"));
        let mut got = None;
        for &b in &out[..n] {
            if let Some(r) = rx.feed(b) {
                let (h, p) = r.unwrap();
                assert_eq!((h.src, h.seq), (Board::Ctrl as u8, i as u16));
                got = Some(Msg::from_frame(&h, p).unwrap());
            }
        }
        assert_eq!(got.as_ref(), Some(&m));
    }
}

#[test]
fn unknown_msg_type_rejected() {
    let hdr = Header::new(200, 1, 0);
    assert_eq!(Msg::from_frame(&hdr, &[]), Err(Error::UnknownType(200)));
}

#[test]
fn link_tx_prefix_recovers_from_console_text_and_forwarding_is_verbatim() {
    use lab_proto::link::{reframe, MAX_LINK_WIRE};
    let mut tx = LinkTx::new(Board::Tx);
    let ping = Msg::Cmd(Cmd {
        cmd_id: 1,
        body: CmdBody::Ping,
    });
    // Console text with no trailing delimiter, then two frames.
    let mut line = b"I (123) boot: hello\r\n".to_vec();
    for _ in 0..2 {
        let mut buf = [0u8; MAX_LINK_WIRE];
        let n = tx.encode(&ping, &mut buf).unwrap();
        line.extend_from_slice(&buf[..n]);
    }
    let mut rx = RxDecoder::new();
    let (mut ok, mut garbage, mut forwarded) = (Vec::new(), Vec::new(), Vec::new());
    for b in line {
        let Some(r) = rx.feed(b) else { continue };
        match r {
            Ok((h, _)) => {
                ok.push(h.seq);
                let mut out = [0u8; MAX_LINK_WIRE];
                let n = reframe(rx.last_body(), &mut out).unwrap();
                forwarded.extend_from_slice(&out[..n]);
            }
            Err(_) => garbage.push(String::from_utf8_lossy(rx.last_body()).into_owned()),
        }
    }
    assert_eq!(ok, vec![0, 1]);
    assert_eq!(garbage, vec!["I (123) boot: hello\r\n".to_string()]);

    // A downstream decoder sees the forwarded frames with src/seq intact.
    let mut rx2 = RxDecoder::new();
    let got: Vec<(u8, u16)> = forwarded
        .into_iter()
        .filter_map(|b| rx2.feed(b).map(|r| r.map(|(h, _)| (h.src, h.seq)).unwrap()))
        .collect();
    assert_eq!(got, vec![(Board::Tx as u8, 0), (Board::Tx as u8, 1)]);
}
