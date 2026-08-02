# ESP32Telemetry

Transparent UART ↔ UDP bridge between **STM32 UART4** and the PC **TelemetryServer**.

## Wiring (STM32 ↔ ESP32 classic)

| STM32 UART4 | ESP32 Serial2 |
|-------------|---------------|
| PD1 TX      | GPIO16 RX     |
| PD0 RX      | GPIO17 TX     |
| GND         | GND           |

Baud: **115200** (match `APP_TELEMETRY_BAUD` in `app_config.h`).

## Latency

Design choices for low ESP32-side delay:

| Mechanism | Role |
|-----------|------|
| **`Serial.onReceive` callback** | UART bytes go to ring buffer immediately (not waiting for `loop()`) |
| **Ring buffers (4 KB)** | Absorb WiFi/UDP jitter without dropping bytes |
| **`WiFi.setSleep(false)`** | Avoid 802.11 power-save adding **10–80 ms** bursts |
| **Tight `loop()`** | No `delay()`; drain UART→UDP every iteration |
| **~59 B/frame @ 100 Hz** | One small UDP datagram per frame (no wait for 512 B fill) |

Typical extra delay ESP32+WiFi: **~2–15 ms** on a good LAN (vs **50–100 ms** if WiFi sleep were on).

Watch USB debug: `overflow uart->udp` means the ring filled — increase `RING_BUF_SIZE` or raise baud later.

## Configure

Edit `include/config.h`: WiFi SSID/password, `REMOTE_IP` (optional if `LEARN_REMOTE_ENABLED=1`).

## Build / flash

```powershell
cd H:\Projects\RobotRL\ESP32Telemetry
pio run -t upload
pio device monitor
```

Note the ESP32 IP on USB serial.

## Subscribe from PC

Send any UDP packet to `ESP32_IP:5000` so the bridge learns your PC address (see `TelemetryServer/README.md`).

## Note on `ESP32/serial_udp_bridge/`

Same idea; **ESP32Telemetry** is the supported path for balance bring-up. The old folder can stay for reference.
