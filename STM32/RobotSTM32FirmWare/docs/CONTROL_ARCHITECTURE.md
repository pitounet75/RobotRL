# Control & RTOS architecture

Architecture and tuning notes for the **self-balancing robot** firmware on STM32H743. **Implemented** in `Core/Src/tasks/`, `app_samples`, `control_strategy_*`, and `app_config.h` (2026-05 bench bring-up).

See also: [README.md](../README.md) (hardware, build, debug), [sensor drivers](../../sensor_drivers/README.md), [Telemetry protocol](../../Telemetry/README.md), [KiCad board](../../../Board/README.md).

---

## Goals

- **500 Hz** balance / drive control loop on the MCU (Jetson supervises, does not own CAN).
- **Low-latency** pitch-rate feedback (fresh gyro, minimal software filtering on the control path).
- **Fast wheel speed** from a **21-bit ABZ** encoder on **one** wheel; **other** wheel speed from **ODrive CAN** at a lower rate.
- **Common-mode bias** between wheels, valid in straight line and in turn, applied at control rate via a **held, low-passed** value.
- Timestamped snapshots for control, telemetry, and Jetson (double-buffer or seqlock publish pattern).

---

## System data flow

```
┌─────────────┐   1 kHz SPI    ┌──────────────┐
│ ICM-45686   │ ─────────────► │ IMU task     │──► ImuSample {t_us, pitch, pitch_rate, ...}
└─────────────┘                └──────────────┘
┌─────────────┐   TIM encoder  ┌──────────────┐
│ ABZ (1 whl) │ ─────────────► │ Encoder task │──► WheelEncoderSample {t_us, v_fast, count}
└─────────────┘                └──────────────┘
┌─────────────┐   CAN RX       ┌──────────────┐
│ ODrive ×2   │ ─────────────► │ ODrive task  │──► OdriveSample[2] {t_us, vel, pos, state}
└─────────────┘                └──────────────┘
                                        │
        ┌───────────────────────────────┼───────────────────────────────┐
        ▼                               ▼                               ▼
 ┌─────────────┐                 ┌─────────────┐                 ┌─────────────┐
 │ Bias update │ 10–100 Hz       │ Control     │ 500 Hz          │ Telemetry   │ 25–50 Hz
 │ (slow)      │                 │ task        │                 │ task        │
 └──────┬──────┘                 └──────┬──────┘                 └─────────────┘
        │                               │
        └──────── bias_hold ───────────►│──► MotorCommand ──► Motor TX ──► CAN
                                        │
 Jetson (USART2) ── commands ──────────►│
 ESP32 (USART1) ◄── telemetry ─────────┴── (reads snapshots only)
```

---

## FreeRTOS tasks (implemented)

Configured in `app_config.h`, created in `app_tasks.c`. CMSIS-RTOS v2 / FreeRTOS priorities in `app_tasks.c`.

| Task | Rate (`*_PERIOD_MS`) | Priority | Responsibility | Status |
|------|----------------------|----------|----------------|--------|
| **control** | **500 Hz** (2 ms) | Realtime | Snapshots → `control_strategy_*` → `app_motor_command` | **Done** |
| **motor_tx** | **500 Hz** (2 ms) | High | `SET_INPUT_VEL` ×2 → **sole** `odrive_can_dma_process_tx()` | **Done** |
| **imu** | **1 kHz** (1 ms) | High | SPI ICM-45686 → complementary fusion → `app_samples` | **Done** |
| **encoder** | **500 Hz** (2 ms) | High | TIM2 ABZ → wheel velocity snapshot | **Done** |
| **odrive** | **100 Hz** (10 ms) | Above normal | RTR encoder estimates; RX ISR updates snapshots | **Done** |
| **bias** | 100 Hz | Above normal | Placeholder / future wheel bias | Stub |
| **watchdog** | 50 Hz | Above normal | Placeholder | Stub |
| **jetson** | 50 Hz | Normal | Placeholder | Stub |
| **telemetry** | 25 Hz | Low | Placeholder | Stub |

**CAN TX rule:** only `task_motor_tx` drains the TX queue (mutex-protected). `task_odrive` enqueues RTRs only.

**Boot (before `osKernelStart()`):**

1. `odrive_can_fdcan_start()` — fixes CubeMX zero TX/RX FIFO depth, starts FDCAN2.
2. `odrive_velocity_mode_startup()` — heartbeats, closed loop, velocity mode (nodes 0 & 1).

**Fallback:** `app_drivers_rtos_init()` calls `odrive_can_fdcan_ensure_started()` + startup if `g_odrive_startup_last_error == IN_PROGRESS` (old flash or desk path).

**Desk build without CAN:** define `DEBUG_DESK_NO_ODRIVE` to skip steps 1–2 in `main`.

---

## Sample rates

| Signal | Acquire / update | Into 500 Hz control |
|--------|------------------|---------------------|
| **Gyro / accel** | **1 kHz** SPI read; IMU ODR ≥ read rate (e.g. **1600 Hz**) | Latest `ImuSample`: **pitch**, **pitch_rate** (gyro); not raw 3-axis accel in PID |
| **Wheel (ABZ)** | **500 Hz–1 kHz**; TIM **encoder mode** (hardware counts edges) | **`v_fast`**; minimal filtering (21-bit → low quantisation noise) |
| **Wheel (ODrive, other)** | **10–100 Hz** (encoder estimate / feedback) | **Not** directly in PID; feeds **bias** only |
| **Control loop** | **500 Hz** | — |
| **Motor CAN TX** | **500 Hz** (same as control) | ~1000 frames/s (`SET_INPUT_VEL` ×2) @ 250 kbit/s — OK with margin |
| **Telemetry** | **25–50 Hz** | Filtered copies OK for ESP32 |

### IMU filtering policy

- **Control path:** prefer **fresh gyro** over heavy software EMA; use **ICM-45686 hardware DLPF** if needed.
- **Telemetry path:** EMA / downsample OK for smooth ESP32 plots.
- **Tilt angle:** complementary filter (fast gyro + slow accel correction), not one EMA on all axes.

### SPI IMU throughput (6 axes)

One burst read ≈ **15 bytes** on SPI1. At **2.5 MHz** ≈ **100–200 µs** with blocking HAL; at **10 MHz** ≈ **40–80 µs**. **1 kHz** reads are safe; **2 kHz** is optional. Set **ODR ≥ read rate** to avoid duplicate samples.

---

## Control loop inputs

### Plant state (500 Hz)

| Input | Symbol | Source | Notes |
|-------|--------|--------|--------|
| Pitch angle | `pitch_rad` | IMU fusion | Lean from vertical; **pitch only** for balance |
| Pitch rate | `pitch_rate_rads` | IMU **gyro** | Primary damping; low lag |
| Fast wheel velocity | `v_fast` | **ABZ** on instrumented wheel | turn/s or m/s at wheel |

### References (commands)

| Input | Source | Typical |
|-------|--------|---------|
| `pitch_ref` | Trim / Jetson | **0** (upright) |
| `vel_ref` | Jetson / teleop | Desired travel speed |
| Steer (later) | Jetson | Yaw rate or `v_cmd_left - v_cmd_right` |

### Optional / later

- `yaw_rate` (gyro Z) for steering.
- Wheel position from encoder integration for odometry (not required for basic balance).
- ODrive velocity on ABZ wheel as cross-check only.

### Control output

- **`v_cmd_left`**, **`v_cmd_right`** (turn/s) → CAN `SET_INPUT_VEL`, **torque FF = 0** unless deliberately used.

Conceptual law (tuning TBD):

```text
u = Kp*(pitch_ref - pitch) + Kd*(0 - pitch_rate) + Kv*(vel_ref - v_for_control)
v_cmd_left  = u + steer
v_cmd_right = u - steer
(+ common-mode bias trim on commands if used)
```

---

## Dual-wheel speed: one ABZ + one ODrive

Only **one** wheel has a local **ABZ** encoder (21-bit). The **other** wheel velocity comes from **ODrive CAN** at **10–100 Hz**.

**Do not** average `v_fast` with a stale ODrive sample inside the 500 Hz PID.

### Bias definition

At the slow update rate (10–100 Hz):

```text
bias_raw = v_odrive_other - v_fast - ΔV_ref
```

| Term | Meaning |
|------|---------|
| `v_fast` | ABZ velocity, instrumented wheel |
| `v_odrive_other` | ODrive feedback for the **other** wheel (same units) |
| `ΔV_ref` | **Expected** speed difference from **commands**: `v_cmd_other - v_cmd_fast` |

When both motors track and slip is low:

```text
v_odrive_other - v_fast ≈ ΔV_ref   →   bias_raw ≈ 0
```

During a turn, `ΔV_ref` removes the **intentional** left/right difference so `bias` stays a **residual** (slip, scale mismatch, trim), not steer.

### Bias conditioning

```text
bias_hold += α * (bias_raw - bias_hold)    // low-pass at bias update rate
```

Optional: rate-limit `bias_hold`, clamp magnitude, disable updates if `\|pitch\|` large or wheel lift detected.

### Use at 500 Hz

Use **`bias_hold`** (frozen between slow updates), not raw ODrive in the hot loop.

**Velocity estimates (example):**

```text
v_fast_est  = v_fast
v_other_est = v_fast + ΔV_ref + bias_hold
v_avg       = 0.5 * (v_fast_est + v_other_est)
```

**Common-mode command trim (optional):**

```text
v_cmd_left  += bias_cmd
v_cmd_right += bias_cmd    // same correction on both wheels
```

Start with **bias on estimates OR commands**, not both at high gain.

### Sign convention

Document once on the bench:

- Which wheel has ABZ (left/right).
- Which CAN node is `other` (0 / 1).
- `ΔV_ref = v_cmd_other - v_cmd_fast` (fixed sign).
- Straight-line run: `bias_raw` should hover near zero after trim.

---

## Timestamped snapshots

Each producer publishes a struct with **`t_us`** (monotonic µs from a free-running HW timer), **`seq`**, and **`valid`**.

Suggested types:

```c
typedef struct {
    uint32_t t_us;
    uint32_t seq;
    bool valid;
    float pitch_rad;
    float pitch_rate_rads;
    /* optional: accel[3], gyro[3] for logging */
} ImuSample;

typedef struct {
    uint32_t t_us;
    uint32_t seq;
    bool valid;
    int32_t count;
    float v_fast;       /* m/s or turn/s at wheel */
} WheelEncoderSample;

typedef struct {
    uint32_t t_us;
    uint32_t seq;
    bool valid;
    float vel;
    float pos_turns;
    uint32_t axis_state;
} OdriveAxisSample;

typedef struct {
    uint32_t t_us;
    float bias_hold;
    float v_avg;
    float pitch_ref;
    float vel_ref;
    float v_cmd_left;
    float v_cmd_right;
} ControlState;
```

**Publish pattern:** double buffer or seqlock — control reads **latest complete** sample without blocking producers.

---

## CAN / ODrive notes

- **250 kbit/s**, nodes **0** and **1** (two single-axis XDrive boards).
- Raising `task_odrive` above **100 Hz** is possible for fresher bias; do not scale all tasks ×5 (bus saturation).
- **`SET_INPUT_VEL`:** `(velocity turn/s, torque_ff Nm)` — torque FF **0** in `task_motor_tx`.

### Control strategies (`g_ctrl_strategy` in debugger)

| ID | Name | File |
|----|------|------|
| 0 | dual_pid | `control_strategy_dual_pid.c` (default) |
| 1 | linear | `control_strategy_linear.c` (F407-style) |
| 2 | cascade | `control_strategy_cascade.c` |

Gains in `app_config.h`. Failsafe: IMU invalid or `|pitch| > APP_CTRL_PITCH_FAILSAFE_RAD` → estop, zero velocity.

---

## Debug globals (Live Expressions)

| Variable | Meaning |
|----------|---------|
| `g_odrive_startup_last_error` | **0** = OK; **255** = startup never ran |
| `g_odrive_drive_valid_mask` | Bit0/1 = encoder snapshot valid per node |
| `g_odrive_can_tx_hal_fail` | FDCAN TX enqueue failures (should stay low) |
| `g_motor_tx_vel_left/right` | Last command sent (turn/s) |
| `g_ctrl_pitch_rad`, `g_ctrl_cmd_turns_s` | Control loop state |
| `g_imu_init_err`, `g_imu_read_ok_count` | IMU bring-up |

---

## Implementation checklist

- [x] FreeRTOS + task set (`app_tasks.c`)
- [x] Monotonic `t_us` (`app_time_us`)
- [x] TIM2 ABZ encoder @ 500 Hz
- [x] IMU task @ 1 kHz + fusion
- [x] ODrive ingest @ 100 Hz + encoder RTR
- [x] Control @ 500 Hz + swappable strategies
- [x] Motor TX @ 500 Hz, mutex TX queue
- [x] Estop on IMU fault / pitch limit
- [ ] Bias task wired to control
- [ ] Jetson / telemetry protocols
- [ ] `configENABLE_FPU` = 1 + ARM_CM7 FreeRTOS port (optional hardening)

---

## Revision history

| Date | Notes |
|------|--------|
| 2026-05 | Implemented RTOS pipeline, CAN boot fixes, debug globals |
| 2026-05 | Initial design: rates, bias formula, task map |
