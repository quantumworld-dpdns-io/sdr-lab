# Wiring

All boards share **GND**, and all signals are 3.3 V logic. The controller's pins avoid GPIO 6–11 (flash) and the strapping pins.

| Function | CTRL (ESP32-WROOM-32) | Peer |
|---|---|---|
| UART1 ↔ TX-S3 | TX GPIO25 → | TX-S3 GPIO18 (UART1 RX) |
| | RX GPIO26 ← | TX-S3 GPIO17 (UART1 TX) |
| UART2 ↔ RX-S3 | TX GPIO17 → | RX-S3 GPIO18 (UART1 RX) |
| | RX GPIO16 ← | RX-S3 GPIO17 (UART1 TX) |
| Trigger (rising edge = t0) | GPIO27 (out) → | GPIO4 (in, ISR) on **both** S3 boards |
| microSD (VSPI) | SCK 18 · MISO 19 · MOSI 23 · CS 5 | SD module (3.3 V) |
| OLED SSD1306 (optional) | SDA 21 · SCL 22 | OLED |
| USB | UART0: console and host bridge | RX-S3 native USB: optional high-rate CSI to the host |

Link settings: 921 600 baud, 8N1, no flow control.

Power: use a stable USB supply. A shared hub works if the boards stay powered while the host is unplugged.
