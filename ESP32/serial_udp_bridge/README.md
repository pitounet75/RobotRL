# ESP32 serial ↔ UDP bridge

Transparent bridge between an **STM32 UART** and **Wi‑Fi UDP**. Bytes on the wire are forwarded unchanged in both directions — suitable for telemetry, debug text, or a custom binary protocol.

## Wiring (STM32 ↔ ESP32)

| STM32 | ESP32 (UART2, default pins) |
|-------|-------------------------------|
| TX    | GPIO **16** (RX)              |
| RX    | GPIO **17** (TX)              |
| GND   | GND                           |

Use **3.3 V** logic levels on both sides. Do not connect STM32 5 V UART to the ESP32.

Default baud: **115200** (`SERIAL_BAUD` in `include/config.h`). Match this on the STM32.

## Buffers

For bursty or high baud traffic, data is not forwarded synchronously in one step:

| Path | Buffering |
|------|-----------|
| STM32 → WiFi | ESP32 UART hardware RX FIFO → **ring** (`RING_BUF_SIZE`) → UDP |
| WiFi → STM32 | UDP → **ring** → UART TX |

UART RX is filled from an `onReceive` callback so bytes are not lost while `endPacket()` or WiFi runs. Increase `RING_BUF_SIZE` and `UART_HW_RX_BUF_SIZE` in `config.h` if you see overflow messages on USB debug (or raise `DEBUG_USB` and watch the serial monitor).

Both ring sizes must stay a **power of two** (default 4096 bytes each).

## Network behaviour

1. ESP32 joins your Wi‑Fi (station mode).
2. It **binds** UDP port `LOCAL_UDP_PORT` (default **5000**).
3. **STM32 → PC**: UART bytes are sent with `sendto` to the remote host/port.
4. **PC → STM32**: send UDP to `ESP32_IP:LOCAL_UDP_PORT`; payload is written to UART.

With `LEARN_REMOTE_ENABLED` (default **1**), the first UDP packet from your PC sets the return address for serial data. Until then, traffic uses `REMOTE_IP` / `REMOTE_PORT` from `config.h`.

## Configuration

Edit `include/config.h`:

- `WIFI_SSID`, `WIFI_PASSWORD`
- `LOCAL_UDP_PORT`, `REMOTE_IP`, `REMOTE_PORT`
- `SERIAL_BAUD`, `UART_RX_PIN`, `UART_TX_PIN`

## Build and flash

Requires [PlatformIO](https://platformio.org/).

```bash
cd ESP32/serial_udp_bridge
pio run -t upload
pio device monitor
```

USB serial (115200) prints Wi‑Fi status when `DEBUG_USB` is 1. Telemetry uses UART2, not USB.

## PC quick test

Replace `192.168.x.x` with the ESP32 IP from the serial monitor.

**Listen for STM32 telemetry** (learn mode: also send one byte so the ESP knows your return address):

```bash
# Linux / WSL
nc -u -l 5000

# In another terminal, poke the ESP once so it learns your IP:
echo -n ping | nc -u 192.168.x.x 5000
```

**Send to STM32:**

```bash
echo hello | nc -u 192.168.x.x 5000
```

On Windows, use `ncat` from Nmap or a small Python script with `socket.socket(socket.AF_INET, socket.SOCK_DGRAM)`.

## Board variants

In `platformio.ini`, change `board` for ESP32-C3, S3, etc. Update `UART_RX_PIN` / `UART_TX_PIN` in `config.h` per your module’s pinout.
