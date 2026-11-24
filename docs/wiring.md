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

Link settings: 8N1, no flow control. Board ↔ board UARTs run at 921 600 baud. Any link to the Mac through a USB-UART bridge (the CTRL UART0, an S3's COM port) runs at 115 200, because macOS's built-in CH34x driver garbles 921 600.

Power: use a stable USB supply. A shared hub works if the boards stay powered while the host is unplugged.

## Host-link mode (Mac as controller, no WROOM)

Use this when the controller board isn't available. The S3 firmware is built with `--features host-link`, and `labctl hub` runs the controller logic on the Mac.

| Function | Connection |
|---|---|
| TX ↔ Mac | TX devkit **UART/COM** USB connector (CH340/CP210x, 115 200 baud) |
| RX ↔ Mac | RX devkit **UART/COM** USB connector |
| Trigger | Jumper **TX GPIO5 → TX GPIO4** and **TX GPIO5 → RX GPIO4** |
| Ground | **TX GND ↔ RX GND** |

TX drives the trigger and timestamps its own edge through the same GPIO4 ISR path as RX, so ISR latency cancels out. With `--features host-link-usb`, the boards use the native **USB** connector instead. That mode is needed for high-rate streams such as CSI (M3).

Opening a USB-UART port resets the board (devkit auto-reset wiring). That's expected: the hub always starts with freshly booted boards.
