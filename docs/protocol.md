# Link protocol

The protocol is implemented in `crates/lab-proto`. It runs the same way over every UART link and over USB to the host.

## Frame
```
wire    = COBS( raw ) 0x00
raw     = header[8] | payload[len] | crc16_le[2]
header  = ver u8 | msg_type u8 | src u8 | flags u8 | seq u16le | len u16le
crc16   = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over header+payload
payload = postcard(message)        len ≤ 512
```
- `ver` is currently `1`.
- `src` is a `Board` value: 1 = TX, 2 = RX, 3 = CTRL, 4 = HOST.
- `seq` is a per-sender u16 counter that wraps. Receivers use it to count link drops.
- `msg_type` selects the payload struct:

| id | type | direction |
|---|---|---|
| 1 | Hello | board → CTRL/host on boot |
| 2 | Status | periodic heartbeat |
| 3 | Cmd (`Configure`/`Arm`/`Stop`/`Ping`/`StatusReq`) | CTRL/host → board |
| 4 | Ack | reply to Cmd (`cmd_id` echoed) |
| 5 | TriggerEvt | TX/RX → CTRL: local t0 |
| 6 | TxRecord | TX: ground truth per frame |
| 7 | PktMeta | RX: per captured frame |
| 8 | CsiFrame | RX |
| 9 | RssiSample | RX |
| 10 | BleAdv | RX |
| 11 | Log | any |

## Compatibility rules
- Postcard encodes by position, so **only append** variants and fields.
- A breaking change bumps `ver`.
- `tests/golden/frames.json` holds frozen vectors that pin the framing layer byte for byte. Don't regenerate them. If they must change, that's a protocol version bump.

## Stream handling
`RxDecoder` reassembles frames from arbitrary chunks and resyncs on the next `0x00` after any corrupt or oversize frame. It keeps counts in `n_ok` and `n_err`.
