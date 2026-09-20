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

## Run (web dashboard)

```powershell
cd H:\Projects\RobotRL\TelemetryServer
.\venv\Scripts\Activate.ps1
python scripts/run_server.py --esp32-host 192.168.1.7 --web
```

Open `http://localhost:8765/` in a browser. `--web` and `--plot` are mutually
exclusive per process — run two `run_server.py` processes if you want both.

- `--web-host` / `--web-port`: bind address/port for the web UI (default
  `0.0.0.0:8765` — reachable from other devices on the LAN).
- Without `--esp32-host`, telemetry streaming still works; the Gains panel and
  speed/heading sliders report "RPC unavailable" (same as `--plot` without
  `--esp32-host`).
- No hardware needed to try it: `python scripts/fake_telemetry_feed.py --hz 500`
  sends synthetic `BalanceFrame` UDP traffic to `127.0.0.1:5000`, which
  `run_server.py --web` (run without `--esp32-host`) will pick up and display.

## Control params (GET / SET)

Runtime gains live on the STM32 (`app_ctrl_params`). The PC talks to them over the
ESP32 UDP bridge via `GetControlParams` / `SetControlParam` (see
`telemetry/ctrl_params.py` — IDs must match firmware).

Snapshot **version** is the first `u32` of the GET payload. It must match
`APP_CTRL_PARAMS_SNAPSHOT_VERSION` in firmware and `SNAPSHOT_VERSION` in
`telemetry/ctrl_params.py` (current: **15**). The PC does **not** reject a
mismatch: a shorter (older) payload is padded with zeros so Refresh still
works; a longer payload is truncated to the PC struct. **SET does not send
the version** — only `param_id u16` + `value f32`. An id the flashed firmware
does not know returns `INVALID_PAYLOAD` (the field can still appear in the UI
as `0` from padding). Append new floats at the end of the packed snapshot and
bump both constants together. `TELEMETRY_CTRL_PARAMS_VERSION` in
`telemetry_ctrl_params.h` is unused leftover — ignore it.

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

- **Gains** tab in `run_server.py --plot`: grouped line edits (consignes, cascade,
  équilibre, lacet, …). Refresh / Apply / Enter on a field / `pos_reset` /
  `heading_reset`.
- Graphs tab: speed + heading sliders → `vel_ref_turns_s` / `heading_ref_rad`.

### Action params (one-shot)

| Name | Effect |
|------|--------|
| `pos_reset` | Zero position odometry / EMA (GET always 0) |
| `heading_reset` | Zero integrated yaw ψ at current heading (GET always 0) |
| `heading_inc` | `ψ̇_ref = value` (signed rad/s, clamp ±4); GET/ACK = current ref |
| `heading_dec` | alias: `ψ̇_ref = −value` (old UI) |

### Param reference (id → name)

| Id | Name | Notes |
|----|------|-------|
| 0 | `strategy` | `ff_cascade` only (`0`) |
| 1 | `pitch_ref_rad` | Base pitch setpoint |
| 2 | `vel_ref_turns_s` | Motor-shaft turn/s (robot frame) |
| 3 | `pitch_failsafe_rad` | Cut drive beyond \|pitch\| |
| 4 | `cmd_max_torque_nm` | Torque clamp |
| 5 | `cascade_vel_kp` | Outer vel → pitch (P) |
| 6 | `cascade_vel_kd` | D on filtered wheel `v̇` |
| 7 | `cascade_pitch_ref_max_rad` | Lean limit |
| 8–11 | `meca_k_grav`, `err_k_pitch`, `meca_k_pitch_damp`, `balance_output_alpha` | `u_meca` + `u_err` + LPF |
| 48 | `err_k_vel` | Live `Kv` of `u_v` (snapshot v13) |
| 49 | `heading_ema` | EMA on yaw-rate gyro (0=raw) |
| 50 | `heading_d_ema` | EMA after dψ̇/dt of raw gyro (snapshot v15 ; 0=raw ψ̈) |
| 51–79 | `antipat_*` / `antipat_sync_*` / `antipat_both_*` | Antipatinage live (shared / sync / both) |
| 12 | `wheel_encoder_vel_lpf_alpha` | ABZ vel EMA (0=off) |
| 13–15 | `torque_deadband_*` | Gated Coulomb boost |
| 16–23 | `motor_torque_correction_*`, `motor_J`, `motor_friction_c`, `motor_accel_lpf` | Rustine α roue (off if kp=0) |
| 24–28 | `pos_kp/kd`, `pos_x_ref_m`, `pos_v_max_turns_s`, `wheel_radius_m` | Position outer loop |
| 29 | `pos_reset` | Action |
| 30–31 | `pos_err_ema_alpha`, `pos_ema_kp` | Pos EMA |
| 32 | `outer_mode` | 0=vel, 1=pos |
| 33–36 | `heading_kp/kd`, `heading_ref_rad`, `heading_torque_max_nm` | Yaw hold |
| 37 | `heading_reset` | Action |
| 38–39 | `cascade_vel_err_ema_alpha`, `cascade_vel_ema_kp` | Leaky I on vel error |
| 40 | `vel_ref_slew_turns_s2` | \|d v_ref/dt\| limit (0=off) |
| 41 | `cascade_vel_accel_kp` | Lean FF on `v̇_ref` |
| 42 | `heading_inc` | `ψ̇_ref` (rad/s, same store as `heading_ref_rad`) |
| 43 | `heading_dec` | SET `ψ̇_ref = −value` (old UI) |
| 44–47 | `friction_mode`, `friction_static_nm`, `friction_kinetic_nm`, `friction_vel_eps_turns_s` | Two-level friction (`hypothesis_lab`) |

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
