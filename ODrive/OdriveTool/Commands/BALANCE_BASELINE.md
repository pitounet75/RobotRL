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
| `outer_mode` | **0** | velocity hold |
| `cascade_vel_kp` | **0.001** | gentle vel → pitch |
| `cascade_vel_ki` | **0.004** | |
| `cascade_vel_kd` | **0.0002** | damp on filtered \(v̇\) |
| `cascade_pitch_ref_max_rad` | **0.262** | ~15° |
| `wheel_encoder_vel_lpf_alpha` | **0.75** | |
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
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set outer_mode 0
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set cascade_vel_kp 0.001
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set cascade_vel_ki 0.004
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set cascade_vel_kd 0.0002
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.5 set cascade_pitch_ref_max_rad 0.262
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
- Tune outer loops: `outer_mode` 0=vel / 1=pos (pos: x→bounded `v_ref` → `cascade_vel_*`; needs `cascade_vel_kp>0`)
- See **Shelved** below for motor-accel P (do not treat as active work)

---

## Shelved (2026-08): motor accel P vs residual cogging

**Status:** implemented in firmware, **left OFF** (`alpha_kp = 0`). Decision after vacation tests: do **not** pursue STM32-side accel asservissement to fight cogging for now. Prefer ODrive anticogging (custom FW: bidirectional map cal is better). Revisit only if maps still leave low-speed ripple.

### Idea

Near upright / slow motion, free-wheel model:

\[
\alpha_{\mathrm{ref}} = \frac{u - c\,\mathrm{sign}(\omega)}{J},\quad
\Delta\tau = K_\alpha\,(\alpha_{\mathrm{ref}} - \alpha_{\mathrm{mes}})
\]

- \(u\) = balance command **before** \(\Delta\tau\) (avoids algebraic loop)
- \(\omega,\alpha_{\mathrm{mes}}\) from **ODrive** `vel_estimate` (not ABZ), per wheel
- Gated on pitch / rate / motor speed; \(\lvert\Delta\tau\rvert \le\) `alpha_max_nm`

Caveat: when the robot pitches, torque also goes into body dynamics — model is only local near \(x\approx0\).

### Identified plant (free wheel, USB, both drives close)

| | Value |
|---|---|
| \(J\) (motor shaft) | **\(1.12\times10^{-5}\,\mathrm{kg\cdot m^2}\)** (avg L/R) |
| \(c\) Coulomb | **0.0052 Nm** |
| \(b\) viscous | ≈ 0 (negligible) |

Script: `ODrive/OdriveTool/Commands/identify_motor_inertia.py` (step torque + coast, LS fit \(J,b,c\)).

**Used by:** [ANTIPATINAGE.md](../../../STM32/RobotSTM32FirmWare/docs/ANTIPATINAGE.md) § *Plant mesuré* — runaway estimate in BOTH_AIR and `η` criterion.

### Code / telemetry (still present)

- Law: `control_strategy_ff_cascade.c`
- Defaults: `app_config.h` (`APP_CTRL_MOTOR_J_*`, `APP_CTRL_ALPHA_*`)
- Live params (snapshot ≥ v3): `alpha_kp`, `alpha_max_nm`, `motor_J`, `motor_friction_c`, gates, `alpha_lpf`
- Keep `alpha_kp = 0` in normal use so baseline is unchanged

### Related (active path for cogging)

- ODrive flash anticogging map (`APP_ODRIVE_ANTICOGGING_ENABLED`)
- User custom ODrive FW: anticogging calibration passes **both directions** (better map than stock one-way)
