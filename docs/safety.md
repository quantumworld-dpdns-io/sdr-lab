# Safety & scope

This lab transmits only **its own test signals**, on its own devices, in licence-free 2.4 GHz channels.

## Enforced in code: `lab_proto::limits::validate`
TX firmware, the controller and the MCP server all call `validate`. A config that fails it is rejected with an `Ack { ok: false }`.

| Limit | Value |
|---|---|
| Channels | 1–11 |
| TX power | 2–15 dBm (`tx_power_qdbm` 8–60) |
| Minimum interval | 10 ms |
| Run duration | 1–3600 s |
| Test payload | ≤ 1024 B |

## Not implemented, by design
- Deauthentication or disassociation frames, and any frame aimed at a MAC that isn't part of the lab.
- Jamming, continuous carrier, or flooding modes.
- Replaying other people's traffic. Replay tests use only the lab's own protocol.

## Passive capture
- The sniffer keeps full records only for the lab MACs.
- Foreign frames are reduced to metadata: RSSI, channel, length, and a hashed source MAC.
- `scan_wifi` and `inspect_ble` return aggregates only.

## Hardware
- Never connect a transmitter straight to an SDR input with coax unless an attenuator is fitted. Check the SDR's maximum input power first; the HackRF limit is about −5 dBm.
- Use an RF shield box when you need reproducible or isolated measurements.
