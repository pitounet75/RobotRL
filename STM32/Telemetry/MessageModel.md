# Telemetry message model

This document describes **how messages are modeled conceptually** in the RobotRL STM32 telemetry system. For byte offsets, hex examples, and integration steps, see [README.md](README.md).

## Layers

| Layer | What it is | Example |
|-------|------------|---------|
| **Frame** | One UART packet on the wire | Magic + length + header + payload + checksum |
| **Message** | One logical unit inside a frame | GetDictionary *request*, `imu` *response* |
| **Message type** | A named contract + numeric id | Type `0x0100`, key `"imu"` |

Every message is carried in exactly one frame. A frame always carries exactly one message.

```
┌─────────────────────────────────────────────────────────────┐
│                        FRAME (wire)                          │
├──────────┬────────┬──────────┬──────────────┬───────────────┤
│  Magic   │ Length │  Header  │   Payload    │   Checksum    │
│  (fixed) │        │ seq+type │ (per role)   │               │
└──────────┴────────┴──────────┴──────────────┴───────────────┘
                              └── MESSAGE ──┘
```

The **wire envelope is identical** for every message. What changes is the **role** (request vs response) and therefore often the **payload layout**.

## Request and response

Each **message type** defines up to two roles:

| Role | Direction | Who sends | Purpose |
|------|-----------|-----------|---------|
| **Request** | Descending (host → STM32) | Host | Ask, command, subscribe |
| **Response** | Ascending (STM32 → host) | STM32 | Answer, data, ack |

- Both use the **same message type id** in the frame header.
- **Sequence ID** on the response must match the request.
- **Request and response payloads may differ** — field lists are defined separately.

```
Host                                    STM32
  │  REQUEST  type=T, seq=N, payload=A     │
  │──────────────────────────────────────>│
  │  RESPONSE type=T, seq=N, payload=B    │
  │<──────────────────────────────────────│
```

Not every type requires both roles:

| Pattern | Request | Response | Example |
|---------|---------|----------|---------|
| RPC | yes | yes | GetDictionary |
| Query | yes | yes | GetMessageDescription |
| Publish | no | yes | Periodic `imu` telemetry |
| Command-only | yes | optional | Future config writes |

**Unsolicited telemetry** is a **response with no request**: the STM32 sends an ascending frame using the response schema and allocates its own sequence id.

## Envelope vs payload

Shared **envelope** (all frames):

- **Magic** — `0x544D` (`TM`)
- **Length** — bytes from sequence through checksum (inclusive)
- **Sequence ID** — correlates request/response
- **Message type** — selects contract
- **Error code** — `uint8` (`TELEMETRY_ERR_*`); `INVALID_MESSAGE` (0) on checksum failure
- **Payload** — request or response body
- **Checksum** — over magic, length, sequence, type, error, payload

Only the **payload** (and usually its length) differs between request and response for the same type.

## Direction (reference)

| Direction | Flow | Typical role |
|-----------|------|----------------|
| **Descending** | Host → STM32 | Request |
| **Ascending** | STM32 → Host | Response |

## Message type identifiers

| Id | Name | Purpose |
|----|------|---------|
| `0` | GlobalError | Last error string |
| `1` | Dictionary | Protocol message catalog |
| `2` | MessageDescription | Response-field dictionary per message |
| `3` | TelemetryFrame | Balance telemetry snapshot |
| `0x0100` … | Application | User-registered types |

Full request/response tables and integration: **[README.md](README.md#implemented-messages-reference)**.

Application types need a **key** (string) and separate **request** / **response** field schemas in the registry.

## Built-in message types (protocol)

### GlobalError (type `0`)

| Role | Payload |
|------|---------|
| **Request** | *(empty)* |
| **Response** | Error message ASCII + `NUL` |

Set from firmware with `telemetry_set_error()`.

### Dictionary (type `1`)

| Role | Payload |
|------|---------|
| **Request** | *(empty)* |
| **Response** | `GlobalError : 0, Dictionary : 1, MessageDescription : 2, TelemetryFrame : 3` + `NUL` |

Answers: *which protocol messages exist and their numeric ids?*  
Does not list application types (`≥ 0x0100`); those use keys from firmware registration.

### MessageDescription (type `2`)

| Role | Payload |
|------|---------|
| **Request** | Target message name + `NUL` (e.g. `TelemetryFrame\0`) |
| **Response** | **Response-field dictionary** — one ASCII string + `NUL` |

Answers: *for this message, what do the **response** bytes mean?*

**Response format** (same style as the protocol catalog):

```
field_a : float, field_b : float, field_c : Int32\0
```

Only **response** fields are listed (not request). Example for `TelemetryFrame`:

```
encoder_speed_l : float, encoder_speed_r : float, omega : float, angle : float, distance_x : float, distance_y : float, setpoint_l : float, setpoint_r : float, angle_setpoint : float
```

### TelemetryFrame (type `3`, key `TelemetryFrame`)

| Role | Payload |
|------|---------|
| **Request** | `streaming_interval_ms` — **UInt32** LE: `0` = one-shot, `N` = stream every N ms |
| **Response** | `telemetry_frame_t` — 9 × `float` little-endian (36 bytes) |

| Field | Description |
|-------|-------------|
| `encoder_speed_l` | Left encoder speed |
| `encoder_speed_r` | Right encoder speed |
| `omega` | Yaw rate |
| `angle` | Measured tilt angle |
| `distance_x` | X position / estimate |
| `distance_y` | Y position / estimate |
| `setpoint_l` | Left wheel command (setpoint) |
| `setpoint_r` | Right wheel command (setpoint) |
| `angle_setpoint` | Target angle (setpoint) |

Firmware updates the snapshot with `telemetry_set_frame()`. The host sets streaming in the request; firmware calls `telemetry_tick_1ms()` from a 1 ms timer to emit periodic frames.

## Registration model (application types)

`telemetry_message_def_t` on the STM32:

| Member | Role |
|--------|------|
| `key` | Dictionary name |
| `message_type` | Wire id (`≥ 0x0100`) |
| `request_fields` / `request_field_count` | Request payload schema |
| `response_fields` / `response_field_count` | Response payload schema |
| `encode` | Builds **response** payload bytes (ascending) |
| `on_descend` | Handles **request** payload (descending) |

At least one of request or response field lists must be non-empty.

## Field types

Used in schemas (GetMessageDescription and registration):

| Name | Binary size / rule |
|------|---------------------|
| `String` | NUL-terminated |
| `Int8` | 1 byte signed |
| `Int16` | 2 bytes LE |
| `Int32` | 4 bytes LE |
| `float` | 4 bytes IEEE LE |
| `double` | 8 bytes IEEE LE |

### Encoding rules

For a given role (request or response), payload = fields concatenated in order:

1. Fixed types use their natural width; multi-byte values are **little-endian**.
2. `String` is NUL-terminated only (no length prefix).
3. Frame `length` = `6 + payload_len` (sequence + type + error + payload + checksum).

The host derives expected size from the schema; the length field confirms what arrived on the wire.

## Sequence IDs

- Set by the **sender** of that frame.
- **Responses** reuse the request’s sequence id.
- **Unsolicited responses** use ids allocated by the firmware.

## Checksum

8-bit sum (mod 256) over magic, length, sequence, type, and payload (excluding the checksum byte).

## Typical host workflow

```
1. Connect (UART or UDP bridge)
2. GetDictionary (1)              → message ids 0–3
3. GetMessageDescription (2)      → field dict per name (e.g. TelemetryFrame)
4. GetTelemetryFrame (3)          → binary snapshot
5. GetError (0)                   → fault string
6. Optional: app types >= 0x0100  → register keys + on_descend / telemetry_send
```

## Firmware integration (summary)

| Call | Role |
|------|------|
| `telemetry_init()` | Wire UART TX |
| `telemetry_rx_feed()` | Parse RX, dispatch all built-ins |
| `telemetry_set_frame()` | Data for type 3 |
| `telemetry_tick_1ms()` | Streaming scheduler (1 ms tick) |
| `telemetry_set_error()` | Text for type 0 |

Full sketch: **[README.md — Integration](README.md#integration-three-responsibilities)**.

## Design principles

1. **One frame, one message** — one role per frame.
2. **Same type id for request/response pair** — sequence id links them.
3. **Separate payload schemas** — request and response may differ.
4. **Length-prefixed frame** — read 4 bytes, then `length` bytes.
5. **Self-describing** — dictionary + description for tools.
6. **Firmware is authoritative** — registry on the STM32 defines contracts.

## Related files

| File | Content |
|------|---------|
| [README.md](README.md) | All implemented messages + integration sketch |
| `telemetry_protocol.h` | Message ids, catalog string |
| `telemetry_frame.h` | TelemetryFrame binary layout |
| `telemetry.h` / `telemetry.c` | Core library |
| `telemetry_example.c` | Optional HAL sample |
