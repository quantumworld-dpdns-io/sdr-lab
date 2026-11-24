# SDR Lab — Architecture & Implementation Plan

**Language: Rust everywhere.** The firmware, the shared protocol crate and all host tooling are Rust. Python appears only as optional notebooks that read the exported Parquet files.

## Context
This is a 2.4 GHz RF security testbed built from two ESP32-S3 boards (TX and RX) and an ESP32-WROOM-32 controller. A HackRF may be added later.
- The ESP32s are **signal sources and limited observers, not SDRs**. They can do Wi-Fi/BLE TX, promiscuous sniffing, RSSI and CSI. They cannot do raw I/Q capture, arbitrary waveforms or spectrum analysis.
- The lab **runs on its own once flashed**. The three boards talk over UART, so control traffic never touches the RF being measured. Data goes to microSD, and a phone web UI controls the lab.
- Every capture has **ground truth**: TX sends sequenced test frames, which gives reproducible datasets that a future SDR can reuse.
- The lab grows into an **RF Security MCP framework** with tools such as `scan_wifi`, `capture_csi` and `capture_iq`.

The topology is **hybrid**: a standalone 3-board core plus an optional host (Mac M1 or Raspberry Pi) for analysis, MCP and the future SDR.

---

## 1. System architecture

```
                 ┌──────────────── Embedded core (runs standalone) ────────────────┐
  iPhone ──Wi-Fi AP (off during RUN)──►  ESP32-WROOM-32  "CTRL"                    │
                 │   experiment state machine · web UI · microSD · OLED (opt)      │
                 │   UART1 ◄──► TX-S3        UART2 ◄──► RX-S3                      │
                 │   GPIO TRIG ──────────────►(both)  shared GND                   │
                 │   ESP32-S3 "TX"  ── 2.4 GHz test frames / BLE adv ──►  ESP32-S3 "RX"
                 └──────────┬───────────────────────────────────────┬──────────────┘
                     USB (CTRL UART0)                   USB-native (RX, high-rate CSI)
                            ▼                                       ▼
                 ┌──────────── Host workspace (Rust, optional) ────────────────────┐
                 │ labctl (CLI) · lab-analysis · lab-mcp (rmcp) · lab-sdr (HackRF) │
                 └──────────────────────────────────────────────────────────────────┘
```

### Firmware stack
- **Framework:** `esp-idf-svc` (std Rust on ESP-IDF and FreeRTOS), on the `esp` toolchain installed by `espup`. Flash with `espflash`.
  - We chose it over bare-metal `esp-hal` because it gives direct access to the mature ESP-IDF APIs through `esp-idf-sys`: `esp_wifi_set_promiscuous*`, `esp_wifi_set_csi*`, `esp_wifi_80211_tx`, FATFS-on-SD, `esp_http_server` with WebSockets, and NimBLE.
- **Wrapping:** the unsafe FFI callbacks for CSI, promiscuous RX and TX are wrapped once in small safe modules. Each callback pushes into a bounded channel that a normal thread drains.
- **BLE:** the `esp32-nimble` crate.
- **Shared code:** `lab-proto` is a `no_std` crate, used unchanged by every board and by the host.

| Board | Crate (`firmware/…`) | Target | Responsibilities |
|---|---|---|---|
| ESP32-S3 A (TX) | `tx-s3` | `xtensa-esp32s3-espidf` | Sends sequenced test frames (own MAC, fixed channel and power) and BLE adverts with seq. Runs the replay-test sender. Emits a `TxRecord` per frame (ground truth). |
| ESP32-S3 B (RX) | `rx-s3` | `xtensa-esp32s3-espidf` | Sniffer filtered to the lab MACs, CSI from TX frames, RSSI and the BLE scanner. Validates the replay protection. |
| ESP32-WROOM-32 (CTRL) | `controller-esp32` | `xtensa-esp32-espidf` | Runs the experiment state machine and aggregates both UART links. Writes the SD dataset and serves the web UI over SoftAP, with the AP off during RUN. Bridges to the host over USB. |

Each firmware crate is its own cargo project, with its own `rust-toolchain.toml` (`channel = "esp"`), `.cargo/config.toml` and `sdkconfig.defaults`. The ESP-IDF version is pinned in `.idf-version` (passed as `ESP_IDF_VERSION`). The projects share code through path dependencies on `crates/`.

### Wiring
See [`wiring.md`](wiring.md). In short: CTRL UART1 goes to TX, UART2 goes to RX, GPIO27 is the trigger, the microSD sits on VSPI and all boards share ground.

### Link protocol
The protocol is implemented in `crates/lab-proto` and specified in [`protocol.md`](protocol.md).
- **Framing:** `COBS(header[8] | postcard payload | CRC16) 0x00`.
- **Messages:** typed serde structs (`Hello`, `Status`, `Cmd`/`Ack`, `TriggerEvt`, `TxRecord`, `PktMeta`, `CsiFrame`, `RssiSample`, `BleAdv`, `Log`). They use `heapless` types, so nothing allocates.
- **Schema rule:** append-only, because postcard encodes by position. The frozen golden vectors in `crates/lab-proto/tests/golden/` are the wire spec.
- **Link speed:** UART at 921 600 baud. CSI at about 400 B/frame × 100 Hz fits. Higher rates stream from the RX-S3 over native USB.
- **Time base:** the CTRL trigger edge defines t0. Each board latches `esp_timer_get_time()` at the edge, and every record carries µs since t0.

### Ground-truth test frame
The format lives in `lab_proto::testframe`:
```
"SDRL" | ver | exp_id | seq | tx_time_us | tx_power_qdbm | channel | pad
```
TX embeds it in vendor-specific action or data frames, and in the BLE manufacturer data. RX finds it with `TestPayload::find` and matches captures on seq. That gives loss, duplicates, latency and per-frame RSSI/CSI.

### Experiment lifecycle and datasets
- `experiments/*.json` hold the on-wire `ExperimentConfig` plus host metadata. `labctl validate` checks them.
- CTRL runs `IDLE → CONFIGURE → ARMED → RUNNING (AP off) → DRAINING → DONE`.
- Run directory `/sd/runs/<exp_id>/`:
  - `manifest.json`
  - `tx.bin`, `rx_pkt.bin`, `csi.bin`, `ble.bin`: raw lab-proto frames, appended
  - `capture.pcap`: radiotap, so it opens in Wireshark
- `labctl export` writes Parquet to `datasets/` (using the `arrow` and `parquet` crates).

### Safety
Safety rules are in [`safety.md`](safety.md). `lab_proto::limits::validate` is the single gate, and TX firmware, the controller and the MCP server all call it.

---

## 2. Repository layout
```
sdr-lab/
├── Cargo.toml                 # host workspace: crates/*, host/*  (firmware excluded)
├── rust-toolchain.toml        # stable (host); firmware crates pin "esp"
├── .idf-version
├── crates/
│   ├── lab-proto/             # no_std: framing, messages, limits, testframe   ✅ M0
│   ├── lab-fw-common/         # std+esp-idf: uart_link, exp_sync, sd_store, wifi/csi/ble wrappers
│   └── replay-guard/          # no_std: HMAC-SHA256 + monotonic counter window (hmac/sha2)
├── firmware/
│   ├── tx-s3/   rx-s3/   controller-esp32/     # esp-idf-svc apps
│   └── controller-esp32/web/                   # tiny static SPA, gzipped, embedded via include_bytes!
├── host/
│   ├── labctl/                # CLI: validate ✅, encode ✅, status, run, export, monitor
│   ├── lab-analysis/          # ndarray + rustfft: rssi path-loss, csi amp/phase, motion, anomaly
│   ├── lab-mcp/               # MCP server (rmcp, stdio), tools from ref.txt
│   └── lab-sdr/               # IqSource trait; NullIq now, HackRF via soapysdr crate later
├── experiments/               # *.json
├── notebooks/                 # optional Python, read-only over datasets/*.parquet
├── datasets/                  # gitignored
└── docs/
```

## 3. Host workspace
- **labctl** (`clap`, `tokio-serial`): `validate`, `encode`, `status`, `run <exp.json>`, `monitor` (live decoded stream), `export <run_dir>` (to Parquet and JSONL).
- **lab-analysis:**
  - path-loss fit
  - CSI amplitude, plus phase sanitization (unwrap, then linear detrend across subcarriers)
  - subcarrier heatmaps (`plotters`)
  - motion detection from CSI variance
  - anomaly baseline (isolation-forest / z-score, using `linfa`)
  - fingerprint features
- **lab-mcp** (the `rmcp` crate, the official Rust MCP SDK): `scan_wifi`, `inspect_ble`, `capture_csi`, `measure_rssi`, `detect_rf_anomaly`, `generate_test_signal` (gated by `limits::validate`), `validate_replay_protection`, `capture_iq` (returns "no SDR attached" until M9).
- **lab-sdr:** the `IqSource` trait (`tune`, `set_rate`, `read_iq`). It has `NullIq` now and `HackRfIq` (soapysdr) later.

---

## 4. Milestones

| # | Milestone | Deliverables | Done when |
|---|---|---|---|
| **M0 ✅** | Scaffold and protocol | Workspace, `lab-proto` (framing, messages, limits, testframe), golden vectors, `labctl validate/encode`, CI | `cargo test` passes. `lab-proto` builds `no_std` for `xtensa-esp32s3-none-elf` and `xtensa-esp32-none-elf`. Clippy runs with `-D warnings`. |
| M1 | UART link and sync | `lab-fw-common::{uart_link, exp_sync}`, the three firmware apps with Hello/Status heartbeat, `labctl status` | All three boards show Ready. TX and RX t0 agree within ~50 µs. |
| M2 | P0: Wi-Fi TX, sniffer, RSSI, loss | `wifi_tx` (`esp_wifi_80211_tx`), sniffer and MAC filter, `sd_store`, pcap writer, `labctl run` | A 60 s run gives matched seqs, a loss % and an RSSI series. The pcap opens in Wireshark. |
| M3 | P0: CSI | CSI wrapper and `CsiFrame` streaming (UART/USB), `lab-analysis csi` | Amplitude/phase plots. Static vs. motion runs are clearly different. |
| M4 | P0: BLE | `esp32-nimble` adv (TX) and scan (RX) | BLE loss and RSSI per run. |
| M5 | Standalone UX | Web UI + WebSocket telemetry, AP-off-during-RUN, OLED (opt) | A full experiment runs from an iPhone with no computer. |
| M6 | MCP | `lab-mcp` | Claude Code calls `measure_rssi` and `capture_csi` against the live lab. |
| M7 | P1 | Motion detection, `replay-guard` + replay attack mode | 0 accepted replays with protection on, >0 with it off. Motion ROC from labelled runs. |
| M8 | P2 | Anomaly model and fingerprinting study | A baseline metric on held-out data. |
| M9 | SDR | `HackRfIq`, live `capture_iq`, FFT/waterfall | HackRF captures are aligned to TX ground truth. |

## 5. Risks
- **CSI throughput:** compact int8 I/Q, a rate cap and the USB fallback.
- **CTRL AP interfering with runs:** the AP is off during RUN, and all data runs over wires.
- **`esp_wifi_80211_tx` frame limits:** use vendor action or data frames. Validate early in M2.
- **WROOM RAM (~320 KB):** static `heapless` buffers, gzipped web assets, and no CSI analysis on device.
- **esp-rs and ESP-IDF version drift:** pin `.idf-version`, the `esp-idf-svc` version and the `esp` toolchain version.

## 6. Verification
- Host checks: `cargo test --workspace`, `cargo clippy -- -D warnings`, `labctl validate experiments/*.json`.
- `lab-proto` `no_std` builds for both Xtensa targets (in CI).
- From M1, firmware builds: `cargo build --release` in each `firmware/*` crate.
- Hardware in the loop for each milestone: `labctl status`, then `labctl run`, then Wireshark or the plots, the phone UI, and the MCP calls.
