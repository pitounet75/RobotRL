# ESP32Telemetry

Transparent UART ↔ UDP bridge between **STM32 UART4** and the PC **TelemetryServer**.

## Wiring (STM32 ↔ ESP32)

| STM32 UART4 | ESP32 Serial2 |
|-------------|---------------|
| PD1 TX      | GPIO16 RX     |
| PD0 RX      | GPIO17 TX     |
| GND         | GND           |

Baud: **921600** 8N1 — must match `SERIAL_BAUD` in `include/config.h` and `APP_TELEMETRY_BAUD` in `STM32/RobotSTM32FirmWare/Core/Inc/app_config.h`.

USB debug (optional): **115200** on the ESP32 USB serial when `DEBUG_USB=1` in `config.h`. This is separate from the STM32 link.

## Boot sequence (order-independent)

Order on ESP32 (`setup()`):

1. Open UART to STM32, enable ring capture, and immediately send `READY\n`.
2. Start the bridge task; it keeps servicing UART while WiFi connects and UDP binds.
3. Enable UDP forwarding once the socket and remote endpoint are ready.

On STM32 (`task_telemetry`):

1. `app_telemetry_init()` — UART4 HAL DMA TX/RX (`ReceiveToIdle_DMA` + `Transmit_DMA`).
2. Wait for ESP32 **`READY\n`** (`APP_TELEMETRY_WAIT_BRIDGE_READY`, 5 s safety timeout).
3. Stream **BalanceFrame @ 500 Hz** (`APP_TELEMETRY_PERIOD_MS=2`).

### Handshake (no fixed sleep required)

| Step | Direction | Message / check |
|------|-----------|-----------------|
| 1 | ESP32 | Enables UART ring capture (bytes kept, not discarded) |
| 2 | ESP32 → STM32 | ASCII `READY\n` every `TM_READY_INTERVAL_MS` until streaming starts |
| 3 | STM32 | Starts BalanceFrame stream after `READY` received |
| 4 | ESP32 | Stops sending `READY` after receiving a complete TM frame with valid length, version and CRC |
| 5 | ESP32 | After `TM_RESTART_SILENCE_MS` without a valid frame, clears any partial UART frame and restarts the handshake |
| 6 | ESP32 | The continuous TM parser resynchronizes on malformed input; UDP forwarding remains independent of handshake state |

WiFi reconnect pauses forwarding but not UART capture or handshake maintenance. The
bridge continues draining UART into a bounded complete-frame queue using an
explicit `drop_oldest` policy.

Config knobs: `TM_READY_INTERVAL_MS`, `TM_RESTART_SILENCE_MS` (ESP32),
`APP_TELEMETRY_BRIDGE_READY_TIMEOUT_MS` / `APP_TELEMETRY_WAIT_BRIDGE_READY`
(STM32). The timeout is only a fallback: normal startup is driven by `READY`.

### Validation

Validate all three cases: ESP32 first, STM32 first, and simultaneous power-up.
Then restart each MCU independently. Streaming should resume within about 2 s
(`READY` every 500 ms, restart detection after 1.5 s). USB debug counters
`ready` and `restart` expose handshake activity.

## Latency

Design choices for low ESP32-side delay:

| Mechanism | Role |
|-----------|------|
| **`Serial.onReceive` callback** | Wakes the bridge task; only that task reads `Serial2` |
| **Continuous TM parser** | Validates length/version/CRC and never exposes partial frames to UDP |
| **Bounded frame queue** | Absorbs WiFi jitter; overflow drops the oldest complete frame and increments a counter |
| **`WiFi.setSleep(false)`** | Avoid 802.11 power-save adding **10–80 ms** bursts |
| **`bridgeTask` on core 1** | Sole owner of UART reads, socket lifecycle, pending datagram and reconnect FSM |
| **RT envelope** | Sequence, frame count, payload length and CRC16 around complete TM frames |

Typical extra delay ESP32+WiFi: **~2–15 ms** on a good LAN (vs **50–100 ms** if WiFi sleep were on).

Every 5 s USB debug reports UART errors, raw-ring high-water, valid/invalid TM frames,
resync bytes, sequence gaps, handshake signals/restarts, frame-queue drops, and
UDP attempts/retries/successes.

## UDP envelope v1 (ESP32 → PC)

Each datagram is self-contained and never splits a TM frame:

```text
offset  size  field
0       2     magic "RT" (0x52 0x54)
2       1     version (1)
3       1     flags
4       4     datagram_sequence, little-endian
8       1     frame_count
9       1     reserved
10      2     payload_length, little-endian
12      N     complete TM frames
12+N    2     CRC16-CCITT over header + payload, little-endian
```

PC → ESP32 RPC commands remain raw TM frames for compatibility.

## Configure

Edit `include/config.h`: WiFi SSID/password, `REMOTE_IP` (optional if `LEARN_REMOTE_ENABLED=1`).

The build is pinned to PlatformIO Espressif32 7.0.1 (Arduino-ESP32 2.0.17 /
ESP-IDF 4.4.7). Arduino’s precompiled SDK does not honor arbitrary project
`sdkconfig` changes, so AMPDU remains at the framework default. A native
ESP-IDF 5.5.x migration should only follow an A/B hardware benchmark of this
corrected baseline; it improves configuration and diagnostics but cannot make
WiFi deterministic.

## Build / flash

```powershell
cd H:\Projects\RobotRL\ESP32Telemetry
pio run -t upload
pio device monitor
```

Note the ESP32 IP on USB serial.

## Subscribe from PC

Send any UDP packet to `ESP32_IP:5000` so the bridge learns your PC address (see `TelemetryServer/README.md`).

## Related docs

- STM32 protocol: [`STM32/Telemetry/README.md`](../STM32/Telemetry/README.md)
- PC client: [`TelemetryServer/README.md`](../TelemetryServer/README.md)
- Legacy reference (not maintained): `ESP32/serial_udp_bridge/`
