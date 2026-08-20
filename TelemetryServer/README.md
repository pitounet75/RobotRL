# TelemetryServer setup (Windows)

```powershell
cd H:\Projects\RobotRL\TelemetryServer
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

For tests: `pip install -r requirements-dev.txt`.

## Run (live plot + CSV)

```powershell
cd H:\Projects\RobotRL\TelemetryServer
.\venv\Scripts\Activate.ps1
python scripts/run_server.py --esp32-host 192.168.1.7 --plot --record logs\session.csv
```

Plot only (no CSV):

```powershell
python scripts/run_server.py --esp32-host 192.168.1.7 --plot
```

- `--esp32-host`: sends UDP `subscribe` so ESP32 learn-remote forwards STM32 UART data to your PC.
- With `--plot`: Qt window with **Graphs** (live) and **Gains** (edit all control params; needs `--esp32-host`).
- Omit `--plot` for headless record only.
- Omit `--record` for plot only.
- `--receive-buffer` requests the UDP socket receive buffer (default 1 MiB).
- Sanity limits are configurable with `--max-torque-nm`, `--max-pitch-rad`,
  `--max-wheel-turns-s`, and `--max-strategy-id`.
- `--verbose` separates UDP sequence gaps, envelope errors, TM CRC/length/version
  errors, STM32 error responses, decode errors, and physical-limit rejects.

## Control params (GET / SET)

Runtime gains live on the STM32 (`app_ctrl_params`). The PC talks to them over the
ESP32 UDP bridge via `GetControlParams` / `SetControlParam` (see
`telemetry/ctrl_params.py` — IDs must match firmware).

Snapshot **version** is in the GET payload (current: **10**). Older firmware still
GETs (trailing fields padded with 0); unknown SET ids are rejected.

### CLI

```powershell
# Dump all params
python scripts/tune_params.py --esp32-host 192.168.x.x get

# Subset
python scripts/tune_params.py --esp32-host 192.168.x.x get --only cascade_vel_kp cascade_vel_kd heading_kp

# Write one param (name or numeric id)
python scripts/tune_params.py --esp32-host 192.168.x.x set cascade_vel_kd 0.0
python scripts/tune_params.py --esp32-host 192.168.x.x set 18 0.002
```

Uses bind port **5000** and sends `subscribe` (steals the BalanceFrame stream while
running). Close the plotter first, or stop it briefly.

### UI

- **Gains** tab in `run_server.py --plot`: Refresh / Apply / `pos_reset` /
  `heading_reset` buttons.
- Graphs tab: speed + heading sliders → `vel_ref_turns_s` / `heading_ref_rad`.

### Action params (one-shot)

| Name | Effect |
|------|--------|
| `pos_reset` | Zero position odometry / EMA (GET always 0) |
| `heading_reset` | Zero integrated yaw ψ at current heading (GET always 0) |
| `heading_inc` | `heading_ref += \|value\|` (rad, wrap ±π); ACK returns new ref |
| `heading_dec` | `heading_ref -= \|value\|` (rad, wrap ±π); ACK returns new ref |

### Param reference (id → name)

| Id | Name | Notes |
|----|------|-------|
| 0 | `strategy` | 0..3 (`ff_cascade` = 3) |
| 1 | `pitch_ref_rad` | Base pitch setpoint |
| 2 | `vel_ref_turns_s` | Motor-shaft turn/s (robot frame) |
| 3 | `pitch_failsafe_rad` | Cut drive beyond \|pitch\| |
| 4–6 | `pitch_kp/ki/kd` | Legacy pitch PID (often unused in ff_cascade) |
| 7–9 | `vel_kp/ki/kd` | Legacy vel PID |
| 10 | `cmd_max_torque_nm` | Torque clamp |
| 11–15 | `linear_*` | Linear strategy |
| 16 | `cascade_vel_kp` | Outer vel → pitch (P) |
| 17 | `cascade_vel_ki` | Legacy; unused (prefer EMA) |
| 18 | `cascade_vel_kd` | D on filtered wheel `v̇` |
| 19 | `cascade_pitch_ref_max_rad` | Lean limit |
| 20–23 | `ff_grav_k`, `ff_fb_k_*`, `ff_output_alpha` | Balance FF+PD |
| 24 | `wheel_encoder_vel_lpf_alpha` | ABZ vel EMA (0=off) |
| 25–27 | `torque_deadband_*` | Gated Coulomb boost |
| 28–35 | `alpha_*`, `motor_J`, `motor_friction_c` | Motor accel P (often off) |
| 36–42 | `pos_*`, `wheel_radius_m` | Position outer loop |
| 43 | `pos_reset` | Action |
| 44–45 | `pos_err_ema_alpha`, `pos_ema_kp` | Pos EMA |
| 46 | `outer_mode` | 0=vel, 1=pos |
| 47–50 | `heading_kp/kd`, `heading_ref_rad`, `heading_torque_max_nm` | Yaw hold |
| 51 | `heading_reset` | Action |
| 52–53 | `cascade_vel_err_ema_alpha`, `cascade_vel_ema_kp` | Leaky I on vel error |
| 54 | `vel_ref_slew_turns_s2` | \|d v_ref/dt\| limit (0=off) |
| 55 | `cascade_vel_accel_kp` | Lean FF on `v̇_ref` |
| 56–57 | `heading_inc`, `heading_dec` | Action nudges |
| 58–61 | `friction_mode`, `friction_static_nm`, `friction_kinetic_nm`, `friction_vel_eps_turns_s` | Two-level friction (`hypothesis_lab`) |

Boot defaults (velocity cascade checkpoint) live in STM32 `app_config.h`. See `STM32/RobotSTM32FirmWare/docs/HYPOTHESIS_LAB.md` for friction A/B.

## Remote control (mouse)

```powershell
python scripts/remote_control.py --esp32-host 192.168.x.x
```

- Up/down → speed (`vel_ref_turns_s`).
- Left/right → **yaw rate** at 10 Hz (accumulates `heading_ref_rad`; does not set absolute heading).
- Sliders: max speed (m/s), max yaw rate (°/s) at full stick.
- Release / STOP → `v=0` (heading_ref kept). `heading_reset` button zeros ψ + ψ_ref.
- Default `--bind-port 0` (ephemeral) and **no** `subscribe`, so it does not steal the
  plotter stream (ESP32 learns remote only on `subscribe`).
- Can run alongside `--plot`. Avoid `--bind-port 5000` if the plotter is open.

## Offline plot

```powershell
python scripts/plot_file.py logs\session.csv --channels pitch_rad,cmd_torque_nm,pitch_rate_rads
```

## Chain

```
STM32 UART4 (921600, BalanceFrame @ 500 Hz, HAL DMA TX/RX)
  -> ESP32Telemetry (TM parse/CRC + complete-frame queue)
  -> sequenced RT/UDP envelope :5000
  -> TelemetryServer (envelope CRC + TM CRC + plot/CSV)
```

The server still accepts legacy raw-TM UDP datagrams. New RT envelopes carry a
32-bit datagram sequence, complete-frame count, payload length, and CRC16 so
network loss is measured independently from STM32 `frame_number` gaps.

## Tests

```powershell
python -m pytest -q
```

Tests cover arbitrary TM fragmentation, malformed lengths/version/CRC, false
magic, parser recovery, configurable sanity checks, UDP draining, and RT
envelope truncation/corruption.

## STM32 firmware (UART4)

1. UART4 @ **921600** (CubeMX + `APP_TELEMETRY_BAUD`).
2. DMA **TX** (`DMA1_Stream2`) and **RX** (`DMA1_Stream3`) linked in CubeMX.
3. `APP_TELEMETRY_UART4_USE_DMA = 1` in `app_config.h` (default after HAL refactor).
4. `APP_TELEMETRY_WAIT_BRIDGE_READY = 1` — wait for ESP32 `READY\n` before streaming.

See [`ESP32Telemetry/README.md`](../ESP32Telemetry/README.md) for the boot handshake.

## Wheel encoders (MT6835 ABZ → STM32 TIM)

Wheel speed comes from local ABZ into TIM2/TIM4 encoder mode (`wheel_encoder_abz.c`).

| Item | Value |
|------|--------|
| Sensor | MagnTek MT6835 (ABZ + SPI) |
| ABZ PPR (programmed) | **16384** (`ESP32MT6835Setup`, constant `MT6835_ABZ_PPR`) |
| STM32 TIM mode | `TIM_ENCODERMODE_TI1` = **×2** (not ×4) |
| `WHEEL_ENCODER_CPR` | **32768** = `16384 × 2` in `app_config.h` |

`TIM_ENCODERMODE_TI12` would be true ×4 (`PPR × 4`). Do not use ×4 unless CubeMX/HAL is set to `TI12`.

To reprogram ABZ PPR over SPI: see [`ESP32MT6835Setup/README.md`](../ESP32MT6835Setup/README.md).

## Wiring

| STM32 | ESP32 |
|-------|-------|
| PD1 TX | GPIO16 RX |
| PD0 RX | GPIO17 TX |
| GND | GND |
