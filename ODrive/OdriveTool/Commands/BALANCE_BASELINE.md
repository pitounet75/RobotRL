# Balance baseline (solid + low vib)

**Status:** validated live 2026-08-09 — best feeling so far (push-resistant, low vib, early catch).

**Reference log:** `logs/krate013_db0017.csv` (~9 s, strategy 3 `ff_cascade`)

| metric | krate013+db0017 | prior 014/0015 | user 013/002 |
|---|---|---|---|
| \|pitch\| mean / max | 0.59° / 1.88° | 0.66° / 2.09° | 0.41° / 1.49° |
| cmd flips/s | **~10** | ~9 | ~17 |
| soft rate std | ~0.20 | ~0.19 | ~0.18 |
| \|v\| mean | ~4.7 turn/s | ~4.7 | ~4.2 |

Compromise: almost as quiet as 014/0015, better lean than that run, without the chatter of deadband 0.002.

---

## Firmware / telemetry gains

Strategy: **`3`** (`CTRL_STRATEGY_FF_CASCADE`)

| Param | Value | Notes |
|---|---|---|
| `ff_grav_k` | **0.14** | ~80% of full gravity FF (~0.178) |
| `ff_fb_k_pitch` | **0.055** | |
| `ff_fb_k_rate` | **0.013** | |
| `ff_output_alpha` | **0.50** | output LPF |
| `cmd_max_torque_nm` | **0.04** | motor-shaft Nm |
| `cascade_vel_*` | **0** | off |
| `torque_deadband_nm` | **0.0017** | gated Coulomb |
| `torque_deadband_pitch_max_rad` | **0.05** | ~3° |
| `torque_deadband_rate_max_rads` | **0.30** | |
| `Kv` (`APP_CTRL_FF_FB_K_VEL`) | **−0.0005** | compile-time only for now |
| `APP_IMU_COMPLEMENTARY_ALPHA` | **0.999** | compile-time; gyro boot-cal TODO |

Plant (for reference): m=1.335 kg, h=0.145 m, gear 3:16, `K_ff_full≈0.178` Nm/motor.

---

## Restore via telemetry (after flash matching this doc)

```powershell
cd H:\Projects\RobotRL
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set ff_grav_k 0.14
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set ff_fb_k_pitch 0.055
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set ff_fb_k_rate 0.013
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set ff_output_alpha 0.50
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set torque_deadband_nm 0.0017
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set torque_deadband_pitch_max_rad 0.05
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set torque_deadband_rate_max_rads 0.30
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 get
```

Defaults in `STM32/RobotSTM32FirmWare/Core/Inc/app_config.h` should match this table.

---

## What not to regress

- Do **not** use ungated Coulomb (~0.0035) — ~13 Hz limit-cycle (`osc4`).
- Deadband **0.002** with `k_rate=0.013` improved pitch but raised flips/s (~17, `tune_user.csv`).
- Deadband **0.0015** with `k_rate=0.017` was too chatty (`db0015.csv`).
- Change **one** live param at a time; keep a CSV per change.

## Optional next (not baseline)

- Startup gyro bias average
- Expose / tune `Kv` for station-keeping
