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

## Host-link mode: no controller board needed
The Mac runs the controller logic. Wiring is in [`docs/wiring.md`](docs/wiring.md#host-link-mode-mac-as-controller-no-wroom).
```sh
cd firmware/tx-s3 && cargo build --release --features host-link && espflash flash --port <TX port> ../target/xtensa-esp32s3-espidf/release/tx-s3
cd firmware/rx-s3 && cargo build --release --features host-link && espflash flash --port <RX port> ../target/xtensa-esp32s3-espidf/release/rx-s3
cargo run --release -p labctl -- hub      # dashboard: http://127.0.0.1:8080/
```

## Firmware
This uses the `esp` toolchain from `espup`. The first build downloads ESP-IDF v5.5.1 into `~/.espressif`.
```sh
cd firmware/tx-s3            && cargo run --release   # build + flash + monitor (espflash)
cd firmware/rx-s3            && cargo run --release
cd firmware/controller-esp32 && cargo run --release
```
Wire the boards as in [`docs/wiring.md`](docs/wiring.md), connect the controller's USB, then:
```sh
cargo run -p labctl -- status         # TX / RX / CTRL ready?
cargo run -p labctl -- sync           # trigger alignment (pass: TX↔RX jitter ≤ 50 µs)
cargo run -p labctl -- monitor        # live decoded frames
```
