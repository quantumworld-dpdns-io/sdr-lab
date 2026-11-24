# sdr-lab

A standalone embedded 2.4 GHz RF security testbed, written in Rust. Two ESP32-S3 boards act as TX and RX, and an ESP32-WROOM-32 runs the experiments. An optional Rust host layer adds analysis, an MCP server and (later) a HackRF.

It is **not** an SDR. It produces and captures Wi-Fi/BLE test signals with known ground truth (RSSI, CSI, packets), which a real SDR can reuse as test material later.

- Architecture and plan: [`docs/architecture.md`](docs/architecture.md)
- Wire protocol: [`docs/protocol.md`](docs/protocol.md)
- Wiring: [`docs/wiring.md`](docs/wiring.md)
- Safety: [`docs/safety.md`](docs/safety.md)

## Quick start (host)
```sh
cargo test --workspace
cargo run -p labctl -- validate experiments/*.json
cargo run -p labctl -- encode experiments/exp_rssi_ch6.json
# lab-proto on the real targets (no_std):
cargo +esp build -p lab-proto --release --target xtensa-esp32s3-none-elf -Zbuild-std=core
```

Firmware (from M1): run `cargo build --release` and then `espflash flash --monitor` inside `firmware/<board>/`. This uses the `esp` toolchain from `espup`.
