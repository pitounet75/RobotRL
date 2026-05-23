# STM32 telemetry (UART binary) — protocol version 1

Framed binary protocol over UART. Works with the [ESP32 serial UDP bridge](../../ESP32/serial_udp_bridge/README.md).

- Concepts: [MessageModel.md](MessageModel.md)
- Sample: `telemetry_example.c` (define `TELEMETRY_EXAMPLE_ENABLE`)

**Python client:** planned after firmware is stable on hardware.

## Frame layout (little-endian, protocol v1)

| Offset | Size | Field |
|--------|------|--------|
| 0 | 2 | Magic (`0x544D`, `TM`) |
| 2 | 2 | **Length** — bytes from version through CRC (inclusive) |
| 4 | 1 | **Protocol version** (`1`) |
| 5 | 2 | Sequence ID |
| 7 | 2 | Message type |
| 9 | 1 | **Error code** (STM32 → host on responses; host sends `NONE` on requests) |
| 10 | N | Payload |
| 10+N | 1 | **CRC-8** (poly 0x07) |

**Length** = `7 + N` · Min frame **11 bytes** (empty payload).

### Error codes (`error_code` byte)

| Code | Name | When |
|------|------|------|
| 0 | `INVALID_MESSAGE` | CRC mismatch |
| 1 | `NONE` | Success |
| 2 | `PROTOCOL_VERSION` | Unsupported `version` byte |
| 3 | `INVALID_PAYLOAD` | Wrong payload size/shape |
| 4 | `UNKNOWN_MESSAGE_TYPE` | Unhandled type id |
| 5 | `UNKNOWN_MESSAGE_KEY` | MessageDescription key not found |
| 6 | `TX_FAILED` | Reserved (UART/DMA write) |

On error, STM32 sets an internal fault string (also readable via **GlobalError** type 0). The response frame still carries `error_code != NONE`.

**Requests:** host sets `error_code = 1`; STM32 **ignores** it on RX.

### CRC-8 vs CRC-16

| | CRC-8 | CRC-16 |
|---|-------|--------|
| ROM | ~40 B | ~80 B |
| Time/frame | ~200–400 µs @ 40 B payload | ~2× |
| Strength | Good for UART links | Stronger |

CRC-8 is used in v1.

---

## Implemented messages

| Id | Key | Request | Response |
|----|-----|---------|----------|
| 0 | `GlobalError` | empty | fault string + `NUL` |
| 1 | `Dictionary` | empty | catalog string + `NUL` |
| 2 | `MessageDescription` | `key\0` | `request\0` + req dict + `response\0` + resp dict |
| 3 | `TelemetryFrame` | `streaming_interval_ms` **UInt32** | 40 B binary (`telemetry_frame_t`) |
| 4 | `GetConfig` | empty | `telemetry_config_t` (skeleton) |
| 5 | `SetEncoderSpeeds` | 2 × **float** LE | empty (ack) |
| ≥0x0100 | user key | per registration | per registration |

Catalog string (`GetDictionary`):

```text
GlobalError : 0, Dictionary : 1, MessageDescription : 2, TelemetryFrame : 3, GetConfig : 4, SetEncoderSpeeds : 5
```

### Application vs protocol messages (Dictionary example)

**GetDictionary** lists **protocol** ids `0–5` only.

If you register an app message:

```c
telemetry_register(&tel, &(telemetry_message_def_t){ .key = "imu", .message_type = 0x0100, ... });
```

- **`imu` does not appear** in the GetDictionary string.
- Host must know the name `imu` (from your docs/firmware).
- Host calls **GetMessageDescription** with payload `imu\0` to get request/response field dictionaries.

### MessageDescription response format

```text
request\0<field : type, ...>\0response\0<field : type, ...>\0
```

Example `TelemetryFrame\0`:

```text
request\0streaming_interval_ms : UInt32\0response\0frame_number : UInt32, encoder_speed_l : float, ...
```

### TelemetryFrame (`telemetry_frame_t`)

| Field | Unit |
|-------|------|
| `frame_number` | counter (stale if unchanged between samples) |
| `encoder_speed_l/r` | rad/s |
| `omega` | rad/s |
| `angle`, `angle_setpoint` | rad |
| `distance_x/y` | m |
| `setpoint_l/r` | ODrive-dependent (TBD) |

**Streaming:** `streaming_interval_ms = 0` → one shot; `N > 0` → immediate frame + every **N ms** via `telemetry_tick_1ms()`.

### SetEncoderSpeeds (testing)

Host sends two floats; firmware stores them — read in your loop:

```c
float l, r;
telemetry_get_command_encoder_speeds(&tel, &l, &r);
```

### GetConfig (skeleton)

Response is `telemetry_config_t` (`config_version` only for now). Extend `telemetry_config.h` when ODrive fields are defined.

---

## Integration

| Call | When |
|------|------|
| `telemetry_init()` | Startup — pass **DMA** UART write fn |
| `telemetry_rx_feed()` | UART RX |
| `telemetry_set_frame()` | Control loop — updates snapshot + `frame_number` |
| `telemetry_tick_1ms()` | 1 ms timer ISR — streaming |
| `telemetry_set_error()` | Application faults |
| `telemetry_set_odrive_config()` | Before GetConfig replies |

```c
static int uart_write_dma(void *ctx, const uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit_DMA(&huart4, (uint8_t *)data, len) == HAL_OK ? 0 : -1;
}
```

See `telemetry_example.c` for full sketch.

---

## API

```c
int telemetry_send_frame(telemetry_t *tel, uint16_t type, uint16_t seq,
                         telemetry_error_code_t err, const uint8_t *payload, uint16_t len);
void telemetry_tick_1ms(telemetry_t *tel);
void telemetry_get_command_encoder_speeds(const telemetry_t *tel, float *l, float *r);
```
